//#include <iostream>
//#include <fstream>
//#include <string>
//#include <vector>
//#include <unordered_map>
//
//#include "json.hpp"  // nlohmann/json 单头文件
//
//using json = nlohmann::json;
//
//// ------------------ 你的技能函数接口示例 ------------------
//// 你已经有这些函数的话，按你自己的声明改名即可
//
//void skill_coarse_locate(const std::vector<std::string>& targets,
//    const std::string& desc)
//{
//    std::cout << "[粗定位] desc = " << desc << "\n";
//    for (auto& t : targets) {
//        std::cout << "  - target: " << t << "\n";
//    }
//    // TODO: 在这里调用你的机械臂粗定位逻辑  
//}
//
//void skill_fine_locate1(const std::vector<std::string>& targets,
//    const std::string& desc)
//{
//    std::cout << "[精定位花瓶] " << desc << "\n";
//    // TODO: 精定位逻辑
//}
//void skill_fine_locate2(const std::vector<std::string>& targets,
//    const std::string& desc)
//{
//    std::cout << "[精定位花朵] " << desc << "\n";
//    // TODO: 精定位逻辑
//}
//
//void skill_grasp(const std::vector<std::string>& targets,
//    const std::string& desc)
//{
//    std::cout << "[抓取] " << desc << "\n";
//    // TODO: 抓取逻辑
//}
//
//void skill_check_grasp(const std::vector<std::string>& targets,
//    const std::string& desc)
//{
//    std::cout << "[检查抓取结果] " << desc << "\n";
//    // TODO: 检查逻辑
//}
//
//void skill_release(const std::vector<std::string>& targets,
//    const std::string& desc)
//{
//    std::cout << "[释放] " << desc << "\n";
//    // TODO: 释放逻辑
//}
//
//// ------------------ Step 结构体 ------------------
//
//struct Step {
//    int id;
//    std::string skill;
//    std::vector<std::string> target;  // 统一成数组，方便处理
//    std::string description;
//};
//
//// ------------------ 从 json::value 解析单个 Step ------------------
//
//Step parse_step(const json& jstep)
//{
//    Step s;
//    s.id = jstep.at("id").get<int>();
//    s.skill = jstep.at("skill").get<std::string>();
//    s.description = jstep.at("description").get<std::string>();
//
//    // target 可能是字符串，也可能是数组
//    if (jstep.at("target").is_string()) {
//        s.target.push_back(jstep.at("target").get<std::string>());
//    }
//    else if (jstep.at("target").is_array()) {
//        for (auto& t : jstep.at("target")) {
//            s.target.push_back(t.get<std::string>());
//        }
//    }
//    else {
//        // 你也可以在这里抛异常或打印错误
//    }
//
//    return s;
//}
//
//// ------------------ 根据 skill 字段分发调用 ------------------
//
//void execute_step(const Step& step)
//{
//    // 为了不打错字，可以用完整匹配
//    if (step.skill == "粗定位") {
//        skill_coarse_locate(step.target, step.description);
//    }
//    else if (step.skill == "精定位放置目标") {
//        skill_fine_locate1(step.target, step.description);
//    }
//    else if (step.skill == "精定位抓取目标") {
//        skill_fine_locate2(step.target, step.description);
//    }
//    else if (step.skill == "抓取") {
//        skill_grasp(step.target, step.description);
//    }
//    else if (step.skill == "检查抓取结果") {
//        skill_check_grasp(step.target, step.description);
//    }
//    else if (step.skill == "释放") {
//        skill_release(step.target, step.description);
//    }
//    else {
//        std::cerr << "未知 skill: " << step.skill << "\n";
//    }
//}
//
//// ------------------ 主流程：读文件 -> 执行全部 steps ------------------
//
//int h()
//{
//    // 1. 打开 JSON 文件
//    std::ifstream in("F:/dhu/project/Robot/Data/prompt/plan.json");
//    if (!in.is_open()) {
//        std::cerr << "无法打开 plan.json\n";
//        return 1;
//    }
//
//    // 2. 解析 JSON
//    json j;
//    try {
//        in >> j;
//    }
//    catch (const std::exception& e) {
//        std::cerr << "JSON 解析失败: " << e.what() << "\n";
//        return 1;
//    }
//
//    // 3. 读取 steps 数组
//    if (!j.contains("steps") || !j["steps"].is_array()) {
//        std::cerr << "JSON 中缺少 steps 数组\n";
//        return 1;
//    }
//
//    std::vector<Step> steps;
//    for (auto& jstep : j["steps"]) {
//        steps.push_back(parse_step(jstep));
//    }
//
//    // 4. 按顺序执行每一步
//    for (const auto& step : steps) {
//        std::cout << "=== 执行步骤 " << step.id
//            << " (" << step.skill << ") ===\n";
//        execute_step(step);
//    }
//
//    return 0;
//}
