//#include <iostream>
//#include <string>
//
//#include "DobotClient.h"
//#include "Dashboard.h"
//#include "DobotMove.h"
//#include "DescartesPoint.h"
//
//int main()
//{
//    using namespace Dobot;
//
//    // 1. ��ʼ�����磨Windows ���룬Linux ����Ҳû���⣩
//    if (!CDobotClient::InitNet()) {  // �� DobotClient �ж���ľ�̬���� :contentReference[oaicite:2]{index=2}
//        std::cerr << "InitNet failed!" << std::endl;
//        return 1;
//    }
//
//    // TODO: �����������ʵ�ʵ� IP ��ַ
//    const std::string ROBOT_IP = "192.168.5.1";
//
//    // 2. �������ƺ��˶�����
//    CDashboard dashboard;  // ���ƣ�������ϵ硢ʹ�ܡ��ٶȵ� :contentReference[oaicite:3]{index=3}
//    CDobotMove move;       // �˶���MovJ��MovL �ȹؽ�/�ѿ����˶� :contentReference[oaicite:4]{index=4}
//
//    // 3. ���ӿ��ƶ˿� 29999
//    if (!dashboard.Connect(ROBOT_IP, 29999)) {
//        std::cerr << "Connect dashboard (29999) failed!" << std::endl;
//        CDobotClient::UinitNet();
//        return 1;
//    }
//
//    // 4. �����˶��˿� 30003
//    if (!move.Connect(ROBOT_IP, 30003)) {
//        std::cerr << "Connect move (30003) failed!" << std::endl;
//        dashboard.Disconnect();
//        CDobotClient::UinitNet();
//        return 1;

//    }
//
//    // 5. �ϵ硢�����ʹ�ܣ������ٶȱ���
//    std::cout << "ClearError:  " << dashboard.ClearError() << std::endl;
//    //std::cout << "PowerOn:     " << dashboard.PowerOn() << std::endl;
//    //std::cout << "EnableRobot: " << dashboard.EnableRobot() << std::endl;
//    //std::cout << "SpeedFactor: " << dashboard.SpeedFactor(50) << std::endl; // ȫ���ٶ� 50% :contentReference[oaicite:5]{index=5}
//    //double pointa[] = { 450, 200, 1000, 180, 0, -135 };
//    // 6. ����һ���ѿ���Ŀ��㣨��λ��mm �� deg�� :contentReference[oaicite:6]{index=6}
//    CDescartesPoint target{};
//    target.x = 450;  // X ��λ�� (mm)
//    target.y = 200;  // Y ��λ�� (mm)
//    target.z = 1100;  // Z ��λ�� (mm)
//    target.rx = 180.0;  // Rx (deg)
//    target.ry = 0.0;  // Ry (deg)
//    target.rz = -135;  // Rz (deg)
//
//    std::cout << "Send MovJ..." << std::endl;
//    std::cout << "MovJ: " << move.MovL(target) << std::endl;   // �ؽڲ岹��Ŀ��ѿ���λ�� :contentReference[oaicite:7]{index=7}
//
//    // 7. �ȴ��˶���ɣ�����ͬ����
//    std::cout << "Sync: " << move.Sync() << std::endl;          // �ȴ������˶���� :contentReference[oaicite:8]{index=8}
//
//    // 8. �Ͽ����Ӳ��ͷ�������Դ
//    move.Disconnect();
//    dashboard.Disconnect();
//    CDobotClient::UinitNet();
//
//    return 0;
//}
