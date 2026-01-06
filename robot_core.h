// robot_core.h
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <vector>
#include <opencv2/core.hpp>

// 返回一张 JPEG（RGB）
bool GetLatestFrame(std::vector<uint8_t>& jpeg_data);
// 更新最新帧（线程安全）
void UpdateLatestFrame(const cv::Mat& frame);
bool InitAll();
bool RunTask(const std::string& user_text);
void EmergencyStop();

nlohmann::json PopLogs();
nlohmann::json GetStatus();
