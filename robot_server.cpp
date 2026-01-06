// robot_server.cpp
#include "httplib.h"
#include "robot_core.h"              // ✅ InitAll/RunTask/EmergencyStop/PopLogs/GetStatus 都在这里
#include <nlohmann/json.hpp>         // ✅ 必须用系统 nlohmann/json，不要 include 你项目里的 json.hpp
#include <iostream>
#include <thread>

using json = nlohmann::json;

int main() {
    httplib::Server svr;

    // ✅ 初始化
    svr.Get("/init", [](const httplib::Request& req, httplib::Response& res) {
        bool ok = InitAll();
        json j;
        j["ok"] = ok;
        res.set_content(j.dump(), "application/json");
    });

    // ✅ 运行任务：/run?text=xxx
    svr.Get("/run", [](const httplib::Request& req, httplib::Response& res) {
        if (!req.has_param("text")) {
            res.status = 400;
            res.set_content(json{{"ok", false}, {"error", "missing param: text"}}.dump(),
                            "application/json");
            return;
        }

        std::string text = req.get_param_value("text");

        // ✅ 异步执行，避免阻塞 http
        std::thread([text]() {
            RunTask(text);
        }).detach();

        res.set_content(json{{"ok", true}}.dump(), "application/json");
    });

    // ✅ 急停
    svr.Get("/stop", [](const httplib::Request& req, httplib::Response& res) {
        EmergencyStop();
        res.set_content(json{{"ok", true}}.dump(), "application/json");
    });

    // ✅ 读取日志
    svr.Get("/logs", [](const httplib::Request& req, httplib::Response& res) {
        json logs = PopLogs();
        res.set_content(json{{"logs", logs}}.dump(), "application/json");
    });

    // ✅ 状态查询
    svr.Get("/status", [](const httplib::Request& req, httplib::Response& res) {
        json st = GetStatus();
        res.set_content(st.dump(), "application/json");
    });

    // ✅ 最新画面
    svr.Get("/frame", [](const httplib::Request& req, httplib::Response& res) {
        std::vector<uint8_t> jpg;
        if (!GetLatestFrame(jpg)) {
            res.status = 404;
            res.set_content("no frame", "text/plain");
            return;
        }

        res.set_content(reinterpret_cast<const char*>(jpg.data()),
                    jpg.size(),
                    "image/jpeg");
    });


    std::cout << "[robot_server] started at 0.0.0.0:8000" << std::endl;
    svr.listen("0.0.0.0", 8000);
    return 0;
}
