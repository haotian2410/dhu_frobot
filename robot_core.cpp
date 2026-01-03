#include "robot_core.h"

#include <atomic>
#include <mutex>
#include <queue>
#include <fstream>
#include <iostream>

#include <pybind11/embed.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "robot_flower_api.h"

using json = nlohmann::json;
namespace py = pybind11;

// ======== ① 日志队列（步骤1） ========

static std::mutex log_mtx;
static std::queue<std::string> log_q;

static void LOG(const std::string &s)
{
    std::lock_guard<std::mutex> lk(log_mtx);
    log_q.push(s);
    std::cout << s << std::endl;
}

json PopLogs()
{
    std::lock_guard<std::mutex> lk(log_mtx);
    json arr = json::array();
    while (!log_q.empty())
    {
        arr.push_back(log_q.front());
        log_q.pop();
    }
    return arr;
}

// ======== ② stop_flag / busy / inited（步骤2） ========

static std::atomic<bool> stop_flag{false};
static std::atomic<bool> busy{false};
static std::atomic<bool> inited{false};

static inline void CHECK_STOP()
{
    if (stop_flag.load())
    {
        throw std::runtime_error("STOPPED");
    }
}

// Python interpreter 只能 init 一次
static std::unique_ptr<py::scoped_interpreter> py_guard;

// ======== ③ InitAll / RunTask（步骤3） ========

bool InitAll()
{
    if (inited.load())
        return true;

    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    try
    {
        py_guard = std::make_unique<py::scoped_interpreter>();

        py::exec(R"(
            import sys, os
            root = os.getenv("ROBOT_HOME", os.getcwd())
            sys.path.append(root)
            print("python sys.path add:", root)
        )");

        LOG("Python interpreter initialized.");
    }
    catch (const std::exception &e)
    {
        LOG(std::string("Python init failed: ") + e.what());
        return false;
    }

    LOG("Init mechanism...");
    init_mechanism();

    LOG("Move to init pose...");
    Move(pose_init, 100);

    LOG("Init system (camera/arm/gripper)...");
    init_system();

    inited = true;
    LOG("InitAll done.");
    return true;
}

bool RunTask(const std::string &user_text)
{
    if (!inited.load())
    {
        LOG("System not initialized.");
        return false;
    }
    if (busy.load())
    {
        LOG("System is busy.");
        return false;
    }

    busy = true;
    stop_flag = false;

    try
    {
        LOG("Write command to order file...");
        std::ofstream outputtext(order);
        outputtext << user_text;
        outputtext.close();

        LOG("Move to init pose for capture...");
        Move(pose_init, 100);

        CHECK_STOP();

        LOG("Run vlm_prompt.py...");
        py::eval_file("vlm_prompt.py");

        CHECK_STOP();

        LOG("Parse JSON steps...");
        std::ifstream file(scan_json, std::ios::binary);
        json j = json::parse(file);

        auto steps = j["steps"];
        for (auto &step : steps)
        {
            CHECK_STOP();

            int id = step["id"];
            std::string skill = step["skill"];
            target = step["target"];
            std::string description = step["description"];

            LOG("Step " + std::to_string(id) + 
                " skill=" + skill + 
                " target=" + target.dump());

            Decide(skill);
        }

        LOG("Stop gripper...");
        py::module gripper_mod = py::module::import("gripper_control");
        gripper_mod.attr("stop_gripper")().cast<bool>();

        LOG("Task finished.");
        busy = false;
        return true;
    }
    catch (const std::exception &e)
    {
        if (std::string(e.what()) == "STOPPED")
        {
            LOG("Task aborted by emergency stop.");
        }
        else
        {
            LOG(std::string("RunTask error: ") + e.what());
        }
        busy = false;
        return false;
    }
}

void EmergencyStop()
{
    stop_flag = true;
    LOG("Emergency stop triggered.");

    // TODO: 在这里调用 JAKA SDK stop motion
    // TODO: 调用夹爪串口 stop / release

    try
    {
        py::module gripper_mod = py::module::import("gripper_control");
        gripper_mod.attr("stop_gripper")().cast<bool>();
    }
    catch (...)
    {
    }
}

json GetStatus()
{
    return json{
        {"inited", inited.load()},
        {"busy", busy.load()},
        {"stop", stop_flag.load()}};
}
