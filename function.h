#pragma once
#ifndef FUNCTION_H
#define FUNCTION_H

#include <vector>
#include <fstream>
#include <iostream>
//#include <Eigen/Dense>
//#include <ceres/ceres.h>
//#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utils/logger.hpp>

// 辅助函数声明
/**
 * @brief 优化标定图像质量
 * @param input 输入图像
 * @param output 输出增强后的图像
 */
void enhanceImage(cv::Mat& input, cv::Mat& output);

/**
 * @brief 将欧拉角转换为旋转矩阵（罗德里格斯形式）
 * @param rx X轴旋转角度（弧度）
 * @param ry Y轴旋转角度（弧度）
 * @param rz Z轴旋转角度（弧度）
 * @return 对应的旋转矩阵
 */
cv::Mat eulerToRodrigues(double rx, double ry, double rz);

/**
 * @brief 将旋转矩阵转换为欧拉角（罗德里格斯形式）
 * @param 对应的旋转矩阵
 * @return rx X轴旋转角度（弧度）
 * @return ry Y轴旋转角度（弧度）
 * @return rz Z轴旋转角度（弧度）
 */
cv::Vec3f rodriguesToEuler(const cv::Mat& R);

/**
 * @brief 从旋转矩阵和平移向量构建4x4齐次变换矩阵
 * @param matrix 输出的4x4变换矩阵
 * @param R 输入的3x3旋转矩阵
 * @param t 输入的3x1平移向量
 */
void compose(cv::Mat& matrix, const cv::Mat& R, const cv::Mat& t);

/**
 * @brief 从4x4齐次变换矩阵提取旋转矩阵和平移向量
 * @param matrix 输入的4x4变换矩阵
 * @param R 输出的3x3旋转矩阵
 * @param t 输出的3x1平移向量
 */
void decompose(const cv::Mat& matrix, cv::Mat& R, cv::Mat& t);

/**
 * @brief 计算4x4齐次变换矩阵的逆矩阵
 * @param T 输入的4x4变换矩阵
 * @param method 求逆方法（0-自动选择，1-解析解，2-SVD）
 * @return 逆矩阵
 */
cv::Mat inver(const cv::Mat& T, int method = 0);

/**
 * @brief 使用变换矩阵变换3D点云
 * @param object_points 输入的对象点云
 * @param b2o 从对象坐标系到基坐标系的变换矩阵
 * @return 变换后的点云
 */
std::vector<cv::Point3f> transformPoints(const std::vector<cv::Point3f>& object_points, const cv::Mat& b2o);

/**
 * @brief 将OpenCV的Mat转换为Eigen的Matrix4d
 * @param cv_mat 输入的4x4 OpenCV矩阵
 * @return 对应的Eigen矩阵
 */
//Eigen::Matrix4d Mat2EigenM4d(const cv::Mat& cv_mat);

#endif // FUNCTION_H