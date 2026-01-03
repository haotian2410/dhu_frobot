from time import sleep
import serial
import numpy as np
from enum import IntEnum

# 导入完整的官方电机库（替换之前的导入）
from CAN import (
    Motor, MotorControl, Control_Type, DM_Motor_Type, LIMIT_MIN_MAX,
    DM_variable
    
)

# ===================== 夹爪配置参数（根据实际硬件修改）=====================
# 串口配置（Windows: "COM3" | Linux/Mac: "/dev/ttyUSB0"）
SERIAL_PORT = "COM4"
BAUDRATE = 921600
TIMEOUT = 1  # 延长超时时间，确保串口通信稳定

# 电机配置（DM4310对应SlaveID需和硬件一致，默认1）
GRIPPER_MOTOR_TYPE = DM_Motor_Type.DM4310  # DM4310对应库中索引0
GRIPPER_SLAVE_ID = 1  # 电机硬件Slave ID（关键！必须和实际一致）
GRIPPER_MASTER_ID = 10  # 主机ID（非0即可）

# MIT控制核心参数（适配DM4310的Limit_Param[0]: Q_MAX=3.14, DQ_MAX=20, TAU_MAX=12）
GRIPPER_MAX_POS = -0.4   # 最大张开位置（rad，不超过3.14）
GRIPPER_MIN_POS = -3.5  # 最小闭合位置（rad，不低于-3.14）
KP = 5.0                # 位置增益（0-500，建议5-20）
KD = 0.5                # 速度阻尼（0-5，建议0.5-2）
GRIPPER_TORQUE = 1.0    # 驱动力矩（0-12Nm，DM4310建议1-3）
GRIPPER_ENABLE_DELAY = 1.0  # 使能延迟（确保电机完全响应）


# ===================== 全局控制对象 ======================
motor_control = None
gripper_motor = None


def init_gripper():

    global motor_control, gripper_motor

    # 如果已经初始化并且串口是打开的，直接复用，避免重复打开COM4
    if motor_control is not None and hasattr(motor_control, "serial_"):
        try:
            if motor_control.serial_.is_open:
                print(f" 串口 {SERIAL_PORT} 已经打开，复用现有连接")
                return True
        except Exception:
            # 如果检查串口状态出错，继续走重新初始化逻辑
            pass

    try:
        # 1. 初始化串口（仅创建对象，库会处理打开/关闭）
        serial_device = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=TIMEOUT,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        print(f" 串口 {SERIAL_PORT} 对象创建成功")

        # 2. 创建电机对象（严格匹配库的参数要求）
        gripper_motor = Motor(
            MotorType=GRIPPER_MOTOR_TYPE,
            SlaveID=GRIPPER_SLAVE_ID,
            MasterID=GRIPPER_MASTER_ID
        )

        # 3. 创建电机控制对象（库会自动处理串口打开）
        motor_control = MotorControl(serial_device)

        # 4. 添加电机到控制对象（关键：SlaveID必须正确）
        add_ok = motor_control.addMotor(gripper_motor)
        if not add_ok:
            print(" 电机添加到控制对象失败")
            return False
        print(f" 电机(SlaveID={GRIPPER_SLAVE_ID})添加成功")

        # 5. 切换到MIT力矩控制模式（库通过写RID=10实现）
        print(" 切换到MIT力矩控制模式...")
        mode_switch_ok = motor_control.switchControlMode(
            gripper_motor,
            Control_Type.MIT  # 模式类型 MIT模式对应1
        )
        if not mode_switch_ok:
            print(" 切换到MIT力矩控制模式失败")
            return False

        # 6. 使能电机（库的enable会发送0xFC指令）
        print(" 使能电机...")
        motor_control.enable(gripper_motor)
        sleep(GRIPPER_ENABLE_DELAY)

        return True

    except serial.SerialException as e:
        print(f" 串口初始化失败: {e}（检查端口是否被占用/接线）")
        return False
    except Exception as e:
        print(f" 夹爪初始化失败: {e}")
        return False


def control_gripper(target_pos):
    """
    通用夹爪控制函数（发送MIT指令并读取实际状态）
    :param target_pos: 目标位置（rad）
    """
    if not motor_control or not gripper_motor:
        print(" 请先调用init_gripper()初始化夹爪")
        return

    try:
        # 1. 安全限位（适配DM4310的±3.14rad范围）
        safe_pos = LIMIT_MIN_MAX(target_pos, -3.14, 3.14)
        print(f"\n 发送MIT指令 | 目标位置: {safe_pos:.2f} rad")

        # 2. 发送MIT控制指令（库内部已调用recv解析反馈）
        motor_control.controlMIT(
            DM_Motor=gripper_motor,
            kp=KP,
            kd=KD,
            q=safe_pos,
            dq=0.0,           # 期望速度0
            tau=GRIPPER_TORQUE  # 驱动力矩
        )

        # 3. 等待电机运动（给足够响应时间）
        sleep(1.5)

        # 4. 主动刷新状态（确保读取最新反馈）
        motor_control.refresh_motor_status(gripper_motor)

    except Exception as e:
        print(f" 夹爪控制失败: {e}")


def open_gripper_max():
    init_gripper()
    """控制夹爪开到最大位置"""
    control_gripper(GRIPPER_MAX_POS)


def close_gripper_min():
    init_gripper()
    """控制夹爪闭合到最小位置"""
    control_gripper(GRIPPER_MIN_POS)


def stop_gripper():
    """
    失能夹爪（程序结束时调用，确保硬件安全）
    """
    if motor_control and gripper_motor:
        try:
            # 1. 发送0力矩指令停止运动
            motor_control.controlMIT(
                DM_Motor=gripper_motor,
                kp=0,
                kd=0,
                q=gripper_motor.getPosition(),
                dq=0.0,
                tau=0.0
            )
            sleep(0.5)

            # 2. 失能电机
            motor_control.disable(gripper_motor)
            sleep(0.1)

            # 3. 关闭串口
            if hasattr(motor_control, "serial_") and motor_control.serial_.is_open:
                motor_control.serial_.close()
            print("\n 夹爪已失能，串口已关闭")
        except Exception as e:
            print(f" 失能夹爪失败: {e}")


# ===================== 测试示例 =====================
if __name__ == "__main__":
    # 1. 初始化夹爪（只初始化一次）
    if init_gripper():

        # 2. 张开夹爪到最大
        print("\n===== 张开夹爪 =====")
        open_gripper_max()
        sleep(2)  # 停留2秒

        # 3. 闭合夹爪到最小
        print("\n===== 闭合夹爪 =====")
        close_gripper_min()
        sleep(2)  # 停留2秒

        # 4. 再次张开夹爪
        print("\n===== 再次张开夹爪 =====")
        open_gripper_max()
        sleep(2)  # 停留2秒

        # 5. 程序结束前统一停机、关串口
        stop_gripper()
