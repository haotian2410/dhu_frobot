#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

// 统一 json 类型
using json = nlohmann::json;

// ================================
// ✅ 全局变量声明（来自 Robot_Flower.cpp）
// ================================

// 这三个在 Robot_Flower.cpp 里是 static 的话，外部无法访问
// 如果你需要 robot_core.cpp 写 order/读取 scan_json/访问 target
// 那你要把 Robot_Flower.cpp 里对应变量的 static 去掉
extern std::string order;
extern std::string scan_json;
extern json target;

// pose_init 你在 Robot_Flower.cpp 里写的是：static vector<double> pose_init = {...}
// ⚠️ 如果你希望 robot_core.cpp / server 使用它，也必须去掉 static
extern std::vector<double> pose_init;

// ================================
// ✅ 机器人核心接口声明（来自 Robot_Flower.cpp）
// ================================

void init_mechanism();
void init_system();

// 你 Robot_Flower.cpp 里定义的是：int Move(vector<double> pose, double speed)
int Move(std::vector<double> pose, double speed);

// Decide 在 Robot_Flower.cpp 里定义的是：void Decide(string skill)
void Decide(std::string skill);

// 可选：如果你还要在 core / server 里用这些函数，也可以声明
int Capture(std::string jpg);
void Gripper(bool state);
void Scanbottle();
void Scanflower();
void Catch();
void Place();
void Check();
