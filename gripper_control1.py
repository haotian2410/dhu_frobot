import serial
from elegripper_modbus import Gripper
import time

def wait_until_stop():
    """等待夹爪停止运动（状态0=运动中）"""
    while True:
        #获取夹爪的当前状态，0:正在运动 1:停止，未检测到物体 2:停止，夹到物体 3:夹到物体以后，物体掉落
        status = g.get_gripper_status()
        if status != 0:
            return status
        time.sleep(0.05)
# ===============================
# 主程序
# ===============================

g = Gripper("COM3", baudrate=115200, id=14)


def gripper_close():
    g.get_gripper_Id()
    g.set_gripper_value(0, 100)  # 闭合
    wait_until_stop()  # 等待
    print("夹爪已完全闭合")

def gripper_open():
    g.get_gripper_Id()
    g.set_gripper_value(100, 100) # 张开
    wait_until_stop()  # 等待
    print("夹爪已完全闭合")



# 测试用（直接运行Python脚本时生效）
if __name__ == "__main__":
    gripper_close()
    gripper_open()