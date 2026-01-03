#include <vector>
#include <string>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "function.h"
#include <pybind11/embed.h>
#include "jktypes.h"
#include "JAKAZuRobot.h"
#ifdef _WIN32
#include <WinSock2.h> //1
#include <WS2tcpip.h> //1
#else
#include <unistd.h>
#include <thread>
#include <chrono>
inline void Sleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#endif
#include <iomanip>
#include <limits>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <regex>
#include <mutex>


#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif


#define PI 3.14159265358979323846
#define DEG2RAD(d) ((d) * PI / 180.0)
#define RAD2DEG(r) ((r) * 180.0 / PI)

using namespace std;
using namespace cv;
using namespace rs2;
using json = nlohmann::json;
namespace py = pybind11;
namespace fs = filesystem;

//=========================================================================/=================================================/
//export ROBOT_HOME=/media/ubuntu/3063-3833/Program/Project/lht_fr/dhu_frobot-main
//文件读取、存储路径
 std::string BASE =
    (std::getenv("ROBOT_HOME") ? std::getenv("ROBOT_HOME") : std::string("."));
 const char* ip = "192.168.1.100";										// 机械臂控制IP
 std::string dir     = BASE + "/Result/";								// 机器人自身信息存储路径
 std::string camera  = dir + "Reprojection/CameraCali/param.txt";		// 相机标定重投影结果文件
 std::string handeye = dir + "Reprojection/HandeyeCali/param.txt";	// 手眼标定重投影结果文件
 std::string bottle  = dir + "VLM/bottle.txt";						// 存储vlm输出信息（用于矫正瓶子位姿）
 std::string flower  = dir + "VLM/flower.txt";						// 存储vlm输出信息（用于存储花束位姿）
 std::string judge   = dir + "VLM/judge.txt";							// 存储vlm输出信息（用于矫正花束位姿）

 std::string picture = BASE + "/Data/Capture/";						// 所捕获图像的存储路径
 std::string quest   = BASE + "/Data/Prompt/quest.txt";
 std::string order   = BASE + "/Data/Prompt/order.txt";

 std::string scan_json = dir + "VLM/scan.json";

//=========================================================================/=================================================/
// 机械臂运动变换链
 Mat dist_coeffs = Mat::zeros(5, 1, CV_64F);							// 畸变系数
 Mat camera_matrix = Mat::zeros(3, 3, CV_64F);						// 相机矩阵
 Mat e2b_matrix = Mat::zeros(4, 4, CV_64F);							// 机械臂位姿信息
 Mat c2e_matrix = Mat::zeros(4, 4, CV_64F);							// 手眼变换矩阵
 Mat o2c_matrix = Mat::zeros(4, 4, CV_64F);							// 图像到相机的变换矩阵
 double camera_rate[4][2] = { {400, 0.836}, {400, 0.836}, { 150, 0.45 } ,{0, 0.223} };// 相机在不同绝对距离下对应图像的像素长度比（mm, mm/px)
//=========================================================================/=================================================/
 Size photo = Size(1280.0, 800.0);									// 相机拍摄图像规格(px)
 double handsize = 114;												// 夹爪可夹取的范围(mm)
double horizon = 426;												// 从环境感知出的桌面位置(mm)
double bottle_height;												// vlm估计的瓶子高度(mm)
vector<double> xy;													// 存储用于估算瓶子的数组
vector<double> pose_bottle;											// 当前对瓶子位置的认知
vector<double> pose_flower;											// 当前对花束位置的认知
Point2f fix_l, fix_r;												// 对花束位置的偏移修正
//=========================================================================/=================================================/
vector<double> pose_init = { 700, -250, camera_rate[0][0] + horizon, 180, 0, -130 };					// 相机初始拍摄位置
vector<double> pose_temp;
json target;

// RealSense相机类
class RealSenseCamera {
private:
	pipeline pipe;
	config cfg;
	frameset frames;

public:
	// 初始化相机
	bool initialize() {
		try {
			// 配置流：RGB图像，1280*800的帧率必须为8，软件上有固定搭配
			cfg.enable_stream(RS2_STREAM_COLOR, 1280, 800, RS2_FORMAT_BGR8, 8);
			// 启动管道
			pipeline_profile profile = pipe.start(cfg);
			std::cout << "RealSense D455相机初始化成功!" << endl;
			std::cout << "RGB相机信息:" << endl;
			// 获取设备信息
			auto device = profile.get_device();
			std::cout << "设备名称: " << device.get_info(RS2_CAMERA_INFO_NAME) << endl;
			std::cout << "序列号: " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << endl;

			// 等待几帧让相机稳定
			for (int i = 0; i < 5; i++) {
				pipe.wait_for_frames();
			}
			std::cout << "相机预热完成" << endl;
			return true;
		}
		catch (const rs2::error& e) {//获取相机错误
			std::cout << "RealSense错误: " << e.what() << endl;
			return false;
		}
		catch (const exception& e) {//获取c++库错误
			std::cout << "错误: " << e.what() << endl;
			return false;
		}
	}

	// 获取一帧RGB图像
	Mat captureFrame(int counts = 3) {
		for (int i = 0; i < counts; i++) {
			try {
				// 设置较短的超时时间（1秒）
				frames = pipe.wait_for_frames(1000);

				// 获取颜色帧
				frame color_frame = frames.get_color_frame();

				if (!color_frame) {
					std::cout << "第 " << (i + 1) << " 次尝试: 未获取到颜色帧" << endl;
					continue;
				}

				// 将RealSense帧转换为OpenCV Mat
				Mat image(Size(1280, 800), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

				if (image.empty()) {
					cerr << "第 " << (i + 1) << " 次尝试: 图像为空" << endl;
					continue;
				}

				std::cout << "成功捕获图像，尺寸: " << image.cols << "*" << image.rows << endl;
				return image.clone(); // 返回拷贝以避免内存问题
			}
			catch (const rs2::error& e) {
				cerr << "第 " << (i + 1) << " 次尝试失败: " << e.what() << endl;
				if (i == counts - 1) {
					return Mat();
				}
			}
		}
		return Mat();
	}

	// 拍摄单张照片并保存到指定路径
	bool captureAndSave(const string filename) {
		std::cout << "开始捕获图像..." << endl;
		Mat frame = captureFrame();

		if (!frame.empty()) {
			bool tp = imwrite(filename, frame);
			if (tp) {
				std::cout << "图像已成功保存为: " << filename << endl;
				return true;
			}
			else {
				std::cout << "保存图像文件失败: " << filename << endl;
				return false;
			}
		}
		else {
			cerr << "捕获图像失败!" << endl;
			return false;
		}
	}

	// 释放资源
	void release() {
		pipe.stop();
		std::cout << "相机已释放" << endl;
	}
};
JAKAZuRobot robot;
RealSenseCamera D455;
// 初始化机械臂类


// ------------------ Step 结构体 ------------------
struct Step {
	int id;
	std::string skill;
	std::vector<std::string> target;  // 统一成数组，方便处理
	std::string description;
};

// 加载标定好的参数
void Loadparam() {
	ifstream fin;
	fin.open(camera);
	for (int i = 0; i < 4; i++)
	{
		if (i < 3)
			fin >> camera_matrix.at<double>(i, 0) >> camera_matrix.at<double>(i, 1) >> camera_matrix.at<double>(i, 2);
		else
			fin >> dist_coeffs.at<double>(0, 0) >> dist_coeffs.at<double>(1, 0) >> dist_coeffs.at<double>(2, 0) >> dist_coeffs.at<double>(3, 0) >> dist_coeffs.at<double>(4, 0);
	}

	fin.close();
	fin.open(handeye);
	for (int i = 0; i < 4; i++)
	{
		fin >> c2e_matrix.at<double>(i, 0) >> c2e_matrix.at<double>(i, 1) >> c2e_matrix.at<double>(i, 2) >> c2e_matrix.at<double>(i, 3);
	}
	fin.close();

	Mat R = eulerToRodrigues(pose_init[3] * PI / 180, pose_init[4] * PI / 180, pose_init[5] * PI / 180);
	// 获取平移向量
	Mat tvec = (Mat_<double>(3, 1) << pose_init[0], pose_init[1], pose_init[2]);
	compose(e2b_matrix, R, tvec);

	std::cout << "DistCoeffs: \n" << dist_coeffs << endl;
	std::cout << "CameraMatrix: \n" << camera_matrix << endl;
	std::cout << "RobotPose: \n" << e2b_matrix << endl;
	std::cout << "HandEyeMatrix: \n" << c2e_matrix << endl;
}

// 更新机械臂位姿
void LoadPose(vector<double> pose) {
	Mat R = eulerToRodrigues(pose[3] * PI / 180, pose[4] * PI / 180, pose[5] * PI / 180);
	// 获取平移向量
	Mat tvec = (Mat_<double>(3, 1) << pose[0], pose[1], pose[2]);
	compose(e2b_matrix, R, tvec);
	cout << "Load RobotPose: \n" << e2b_matrix << endl;
}

int Move_jaka(vector<double> pose, double speed) {
	CartesianPose cp{};
	cp.tran.x = pose[0];
	cp.tran.y = pose[1];
	cp.tran.z = pose[2];
	cp.rpy.rx = DEG2RAD(pose[3]);
	cp.rpy.ry = DEG2RAD(pose[4]);
	cp.rpy.rz = DEG2RAD(pose[5]);
	double spd = speed;
	int ret = robot.linear_move(&cp, ABS, TRUE, spd);
	std::cout << "linear_move ret = " << ret << "\n";
	return ret;
}

// 输入位姿进行末端直线运动
int Move(vector<double> pose, double speed) {
// 	double* point = pose.data();
// 	demo->moveRobot(point);
 	return Move_jaka(pose, speed);
	//return 0;
}

// 捕获相机当前图像
int Capture(string jpg) {

	if (!fs::exists(picture)) {
		try {
			fs::create_directories(picture);
			std::cout << "已创建保存目录: " << picture << endl;
		}
		catch (const fs::filesystem_error& e) {
			std::cout << "创建目录失败: " << e.what() << endl;
			return -1;
		}
	}

	// 等待机械臂稳定
	std::cout << "等待机械臂稳定..." << endl;
	Sleep(300);

	std::cout << "正在拍照..." << endl;
	if (!D455.captureAndSave(picture + jpg)) {
		cerr << "拍照失败!" << endl;
	}

	return 0;
}

// 控制夹爪张闭合
void Gripper(bool state) {
	try {
		// 导入Python模块
		//py::module_ gripper_mod = py::module_::import("gripper_control");
		py::module gripper_mod = py::module::import("gripper_control");

		// 根据state调用对应的Python函数
		if (state) {
			// 调用gripper_close()：闭合夹爪
			gripper_mod.attr("close_gripper_min")().cast<bool>();
		}
		else {
			// 调用gripper_open()：张开夹爪
			gripper_mod.attr("open_gripper_max")().cast<bool>();
			gripper_mod.attr("stop_gripper")().cast<bool>();
		}
	}
	catch (const py::error_already_set& e) {
		cout << "调用Python函数失败：" << e.what() << endl;
	}
}

// 改变夹爪抓取姿态
void Transform() {
// 	pose_temp = pose_init;
// 	pose_temp[3] = -90;
// 	pose_temp[4] = 40;
// 	pose_temp[5] = -90;
// 	Move(pose_temp, 20);
// 	pose_temp = pose_bottle;
// 	pose_temp[2] += 300;
// 	pose_temp[3] = -90;
// 	pose_temp[4] = 40;
// 	pose_temp[5] = -90;
// 	Move(pose_temp, 100);
// 	pose_temp[2] -= 270;
// 	// pose_temp[2] -= 320;
// 	pose_temp[1] += c2e_matrix.at<double>(2, 3) + 10;
// 	pose_temp[0] -= 10;
}

// 定位目标点在基底坐标系下的坐标
void Locate(double& dx, double& dy, double dz) {
	Mat R = Mat::eye(3, 3, CV_64F), t = Mat::zeros(3, 1, CV_64F);
	t.at<double>(0, 0) = dx;
	t.at<double>(1, 0) = dy;
	t.at<double>(2, 0) = dz + 130;
	compose(o2c_matrix, R, t);
	Mat o12b = e2b_matrix * c2e_matrix * o2c_matrix;
	decompose(o12b, R, t);

	Vec3f rxyz;
	rxyz = rodriguesToEuler(R);
	dx = t.at<double>(0, 0);
	dy = t.at<double>(1, 0);
}

// 扫描图像，感知花瓶信息
void Findbottle() {
	try {
		// 使用 eval_file 执行Python文件
		py::eval_file("vlm_findBottle.py");
	}
	catch (const py::error_already_set& e) {
		cout << "调用python文件失败:" << e.what() << endl;
	}
	// 读取Python代码生成的结果
	ifstream inputtext(bottle);
	double val[2];
	int count = 0;
	char c;
	while (inputtext.get(c) && count < 2) {
		// 将 char 转换为 unsigned char 避免断言错误
		unsigned char uc = c;

		// 构建数字字符串
		string numStr = "";
		if (c == '-' || c == '.' || isdigit(uc)) {
			numStr += c;
			// 继续读取数字部分
			while (inputtext.get(c) && (isdigit(static_cast<unsigned char>(c)) || c == '.')) {
				numStr += c;
			}
			// 转换为数字并存储
			val[count++] = stod(numStr);
		}
	}
	inputtext.close();
	Point2f P1;
	double dx, dy;

	// 比例换算成坐标
	P1.x = photo.width * val[0];
	P1.y = photo.height * val[1];
	cout << P1 << endl;

	// 粗定位花瓶
	dx = (P1.x - photo.width / 2) * camera_rate[0][1];
	dy = (P1.y - photo.height / 2) * camera_rate[0][1];
	Locate(dx, dy, camera_rate[0][0]);
	xy.push_back(dx - pose_init[0] - c2e_matrix.at<double>(0, 3));
	xy.push_back(dy - pose_init[1] + c2e_matrix.at<double>(2, 3)); // 第一次定位

	double tx, ty;
	tx = dx + c2e_matrix.at<double>(0, 3);
	ty = dy - c2e_matrix.at<double>(2, 3);

	pose_bottle.clear();
	pose_bottle.push_back(tx);
	pose_bottle.push_back(ty);
	pose_bottle.push_back(horizon + camera_rate[1][0]);
	pose_bottle.push_back(pose_init[3]);
	pose_bottle.push_back(pose_init[4]);
	pose_bottle.push_back(pose_init[5]);

	cout << "bottle pose：" << endl;
	for (auto i : pose_bottle)
		cout << i << endl;
}

// 计算并优化花瓶位置
void Judgebottle(double& r1, double& r2) {
	try {
		// 使用 eval_file 执行Python文件
		py::eval_file("vlm_findBottle.py");

	}
	catch (const py::error_already_set& e) {
		cout << "调用python文件失败:" << e.what() << endl;
	}
	// 读取Python代码生成的结果
	ifstream inputtext(bottle);
	double val[2];
	int count = 0;
	char c;

	while (inputtext.get(c) && count < 2) {
		// 将 char 转换为 unsigned char 避免断言错误
		unsigned char uc = c;

		// 构建数字字符串
		string numStr = "";
		if (c == '-' || c == '.' || isdigit(uc)) {
			numStr += c;
			// 继续读取数字部分
			while (inputtext.get(c) && (isdigit(static_cast<unsigned char>(c)) || c == '.')) {
				numStr += c;
			}
			// 转换为数字并存储
			val[count++] = stod(numStr);
		}
	}
	inputtext.close();
	cout << val[0] << endl << val[1] << endl;
	r1 = val[0];
	r2 = val[1];
}

void Caculate_bottle(double r1, double r2) {

	Point2f P;
	double dx, dy;
	P.x = photo.width * r1;
	P.y = photo.height * r2;
	cout << P << endl << endl;

	dx = (P.x - photo.width / 2) * camera_rate[1][1];
	dy = (P.y - photo.height / 2) * camera_rate[1][1];
	Locate(dx, dy, camera_rate[1][0]);
	xy.push_back(dx - pose_init[0] - c2e_matrix.at<double>(0, 3));
	xy.push_back(dy - pose_init[1] + c2e_matrix.at<double>(2, 3)); // 第二次定位

	//利用两次定位，获取瓶身高度，完成点位矫正
	double l1 = sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
	double l2 = sqrt(xy[2] * xy[2] + xy[3] * xy[3]);
	bottle_height = (camera_rate[0][0] + camera_rate[1][0]) / 2 * (l1 - l2) / (2 * l1 - l2); //两条射影线与桌面形成的三角形通过相似性求解
	cout << bottle_height << endl;
	dx = xy[0] * (1 - bottle_height / (camera_rate[0][0] + camera_rate[1][0]) / 2) + pose_init[0] + c2e_matrix.at<double>(0, 3);
	dy = xy[1] * (1 - bottle_height / (camera_rate[0][0] + camera_rate[1][0]) / 2) + pose_init[1] - c2e_matrix.at<double>(2, 3);

	pose_bottle.clear();
	pose_bottle.push_back(dx + c2e_matrix.at<double>(0, 3));
	pose_bottle.push_back(dy - c2e_matrix.at<double>(2, 3));
	pose_bottle.push_back(horizon + camera_rate[2][0]);
	pose_bottle.push_back(pose_init[3]);
	pose_bottle.push_back(pose_init[4]);
	pose_bottle.push_back(pose_init[5]);

	cout << "bottle better pose：" << endl;
	for (auto i : pose_bottle)
		cout << i << endl;
	LoadPose(pose_bottle);
	Move(pose_bottle, 100);
	Capture("bottle.jpg");

	Judgebottle(r1, r2);
	P.x = photo.width * r1;
	P.y = photo.height * r2;
	cout << P << endl << endl;

	if (P.x != photo.width / 2 || P.y != photo.height / 2) {
		dx = (P.x - photo.width / 2) * camera_rate[3][1];
		dy = (P.y - photo.height / 2) * camera_rate[3][1];

		double x, y;
		x = pose_bottle[0];
		y = pose_bottle[1];
		pose_bottle.clear();
		pose_bottle.push_back(dy + x);
		pose_bottle.push_back(dx + y);
		pose_bottle.push_back(horizon + camera_rate[2][0]);
		pose_bottle.push_back(pose_init[3]);
		pose_bottle.push_back(pose_init[4]);
		pose_bottle.push_back(pose_init[5]);
	}

	cout << "bottle optimized pose：" << endl;
	for (auto i : pose_bottle)
		cout << i << endl;
	LoadPose(pose_bottle);
	Move(pose_bottle,100);
	Capture("bottle.jpg");
}

// 获取当前决策的抓取对象
void GetTarget() {
	vector<string> lines;
	string line;
	ifstream infile(quest);
	while (getline(infile, line)) {
		lines.push_back(line);
	}
	infile.close();
	lines[1] = target;

	ofstream outputtext(quest);
	for (size_t i = 0; i < lines.size(); ++i) {
		outputtext << lines[i] << endl;
	}
	outputtext.close();
}

// 扫描图像，感知花束信息
void Findflower() {
	GetTarget();
	try {
		// 使用 eval_file 执行Python文件
		py::eval_file("vlm_findFlower.py");
	}
	catch (const py::error_already_set& e) {
		cout << "调用python文件失败:" << e.what() << endl;
	}
	// 读取Python代码生成的结果
	ifstream inputtext(flower);
	double val[4];
	int count = 0;
	char c;
	while (inputtext.get(c) && count < 4) {
		// 将 char 转换为 unsigned char 避免断言错误
		unsigned char uc = c;

		// 构建数字字符串
		string numStr = "";
		if (c == '-' || c == '.' || isdigit(uc)) {
			numStr += c;
			// 继续读取数字部分
			while (inputtext.get(c) && (isdigit(static_cast<unsigned char>(c)) || c == '.')) {
				numStr += c;
			}
			// 转换为数字并存储
			val[count++] = stod(numStr);
		}
	}
	inputtext.close();
	LoadPose(pose_init);
	Point2f P1, P2, P;
	double dx, dy;

	// 比例换算成坐标

	P1.x = photo.width * val[0];
	P1.y = photo.height * val[1];

	P2.x = photo.width * val[2];
	P2.y = photo.height * val[3];

	P.x = P2.x + (P1.x - P2.x) * 0.3;
	P.y = P2.y + (P1.y - P2.y) * 0.3;


	cout << P << endl << endl;

	// 定位花束
	dx = (P.x - photo.width / 2) * camera_rate[0][1];
	dy = (P.y - photo.height / 2) * camera_rate[0][1];
	Locate(dx, dy, camera_rate[0][0]);

	// 抓取方向与花束朝向平行
	double theta_x;
	theta_x = atan2(P1.y - P2.y, P1.x - P2.x);
	double theta = -PI / 4 - theta_x; //相机是倒着的
	if (theta > PI)
		theta -= 2 * PI;

	pose_flower.clear();
	pose_flower.push_back(dx);
	pose_flower.push_back(dy);
	pose_flower.push_back(horizon);
	pose_flower.push_back(pose_init[3]);
	pose_flower.push_back(pose_init[4]);
	pose_flower.push_back(theta / PI * 180);

	cout << "flower pose:" << endl;
	for (auto i : pose_flower)
		cout << i << endl;
	
	// 可能的修正偏移量
	double omega = PI / 2 - theta_x;
	if (omega > PI)
		omega -= 2 * PI;

	// 由于相机观测角度是颠倒的（安装问题）， 所以左偏则向右修正， 右偏则向左修正
	fix_l.x = dx - (handsize / 5) * sin(omega);	// 向左修正
	fix_l.y = dy + (handsize / 5) * cos(omega);
	fix_r.x = dx + (handsize / 5) * sin(omega); // 向右修正
	fix_r.y = dy - (handsize / 5) * cos(omega);
}

// 判断花束是否符合抓取条件
void Judgeflower() {
	try {
		// 使用 eval_file 执行Python文件
		py::eval_file("vlm_judgeFlower.py");

	}
	catch (const py::error_already_set& e) {
		cout << "调用python文件失败:" << e.what() << endl;
	}
	// 读取Python代码生成的结果
	ifstream inputtext(judge);
	double val;
	inputtext >> val;
	inputtext.close();
	if (val != 0) {
		if (val == 1) {
			pose_flower[0] = fix_r.x;
			pose_flower[1] = fix_r.y;
		}
		else if (val == 2) {
			pose_flower[0] = fix_l.x;
			pose_flower[1] = fix_l.y;
		}
		for (auto i : pose_flower)
			cout << i << endl;
		Move(pose_flower, 100);
	}
}

void Scanbottle() {	// 花瓶定位
	Findbottle();
	LoadPose(pose_bottle);
	Move(pose_bottle, 100);
	Capture("bottle.jpg");
	double r1, r2;
	Judgebottle(r1, r2);
	Caculate_bottle(r1, r2);
}

void Scanflower(){	// 花束定位
	Move(pose_init, 100);
	Sleep(300);
	Capture("flower.jpg");
	Findflower();
	LoadPose(pose_flower);
	Move(pose_flower, 100);
	Gripper(1);
	// Judgeflower();
}

void Catch()
{	// 抓取并移动到自然位置
	Move(pose_init, 100);
	Transform();
}

void Place() 
{	// 移动到花瓶上方，进行插花操作。
	Move(pose_temp, 100);
	Gripper(0);
	pose_temp[0] -= 100;
	pose_temp[2] += 50;
	pose_temp[3] = -155;
	pose_temp[4] = 20;
	pose_temp[5] = -125;
	Move(pose_temp, 40);
	Sleep(300);
	Move(pose_init, 40);
}

void Check() {
	//第三方相机；
	//TODO；
}

void Decide(string skill) {
	if (skill == "Fw");
	if (skill == "Scanbottle") Scanbottle();
	if (skill == "Scanflower") Scanflower();
	if (skill == "Catch") Catch();
	if (skill == "Place") Place();
	if (skill == "Check") Check();
}

void init_mechanism() {
	// 初始化机械臂
	int ret = robot.login_in(ip);
	std::cout << "login_in return: " << ret << endl;
	if (ret != 0) {
		cerr << "机械臂登录失败，请检查 IP/网络。\n";
		return;
	}

	std::cout << "机械臂上电: " << robot.power_on() << endl;
	std::cout << "机械臂上使能: " << robot.enable_robot() << endl;

	// 初始化相机
	if (!D455.initialize()) {
		cerr << "相机初始化失败，程序将继续运行但无法拍照。\n";
	}
	try {
		// 导入Python模块
		//py::module_ gripper_mod = py::module_::import("gripper_control");
		py::module gripper_mod = py::module::import("gripper_control");

		gripper_mod.attr("init_gripper")().cast<bool>();
	}
	catch (const py::error_already_set& e) {
		cout << "调用Python函数失败：" << e.what() << endl;
	}
}

void init_system() {
	Loadparam();
	Sleep(300);
	Capture("bottle.jpg");
	Capture("flower.jpg");
}


// int main() {
// 	// 抑制 OpenCV 的信息消息
// 	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
// 	// 初始化Python解释器
// 	py::scoped_interpreter guard{};
// 	// 添加openai包所在路径
// 	py::exec(R"(
// 		import sys, os
// 		sys.path.append(os.getcwd())
// 		)");

// 	init_mechanism();
// 	Move(pose_init, 100);
// 	init_system();
// 	//// 给予指令
// 	//cout << "请给出您的命令：" << endl;
// 	//string s;
// 	//getline(cin, s);
// 	//ofstream outputtext(order);
// 	//outputtext << s;  // 将完整命令写入文件
// 	//outputtext.close();

// 	//任务分解生成json
// 	try {
// 		// 使用 eval_file 执行Python文件
// 		py::eval_file("vlm_prompt.py");
// 	}
// 	catch (const py::error_already_set& e) {
// 		cout << "调用python文件失败:" << e.what() << endl;
// 	}
// 	ifstream file(scan_json, ios::binary);
// 	json j = json::parse(file);

// 	auto steps = j["steps"];

// 	//循环运行给出skill json
// 	for (auto& step : steps) {
// 		int id = step["id"];
// 		string skill = step["skill"];
// 		target = step["target"];
// 		string description = step["description"];
// 		cout << target << skill << endl;
// 		Decide(skill);
// 	}
// 	try {
// 		// 导入Python模块
// 		//py::module_ gripper_mod = py::module_::import("gripper_control");
// 		py::module gripper_mod = py::module::import("gripper_control");

// 		gripper_mod.attr("stop_gripper")().cast<bool>();
// 	}
// 	catch (const py::error_already_set& e) {
// 		cout << "调用Python函数失败：" << e.what() << endl;
// 	}
// 	return 0;
// }