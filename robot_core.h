// robot_core.h
#pragma once

#include <string>
#include <nlohmann/json.hpp>

bool InitAll();
bool RunTask(const std::string& user_text);
void EmergencyStop();

nlohmann::json PopLogs();
nlohmann::json GetStatus();
