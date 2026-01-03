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

using namespace std;
using namespace cv;
//using namespace ceres;
//using namespace Eigen;

// 各种辅助函数
//============================================================================================/
// 辅助函数：优化标定图像
void enhanceImage(Mat& input, Mat& output) {
    cv::Mat enhanced;
    // 高斯滤波去噪
    cv::GaussianBlur(input, enhanced, cv::Size(3, 3), 0.8);

    // 锐化增强
    cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
        -1, -1, -1,
        -1, 9, -1,
        -1, -1, -1);
    cv::filter2D(enhanced, output, -1, kernel);
}

// 辅助函数：欧拉角-弧度(ZYX顺序)转旋转矩阵
Mat eulerToRodrigues(double rx, double ry, double rz) {
    // 计算ZYX欧拉角的旋转矩阵
    double cy = cos(rz); double sy = sin(rz);
    double cp = cos(ry); double sp = sin(ry);
    double cr = cos(rx); double sr = sin(rx);

    Mat R = (Mat_<double>(3, 3) <<
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr
        );
    return R;
}

// 辅助函数：旋转矩阵转欧拉角-弧度(ZYX顺序)
Vec3f rodriguesToEuler(const Mat& R) {
    // 检查矩阵维度
    CV_Assert(R.rows == 3 && R.cols == 3);

    double rx, ry, rz;

    // 处理万向锁情况
    if (abs(R.at<double>(2, 0)) > 0.9999) {
        // 万向锁情况：ry = ±π/2
        ry = -asin(R.at<double>(2, 0));  // 或者 ry = π + asin(R.at<double>(2, 0))
        rz = atan2(-R.at<double>(0, 1), R.at<double>(1, 1));
        rx = 0.0;  // 任意值，通常设为0
    }
    else {
        // 正常情况
        ry = atan2(-R.at<double>(2, 0),
            sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                R.at<double>(1, 0) * R.at<double>(1, 0)));

        rz = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        rx = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    }

    return Vec3f(rx, ry, rz);
}

// 辅助函数：组合变换矩阵（从旋转矩阵和平移向量构建4x4齐次变换矩阵）
void compose(Mat& matrix, const Mat& R, const Mat& t) {
    // 输入验证
    if (R.rows != 3 || R.cols != 3 || t.rows != 3 || t.cols != 1) {
        cerr << "Error: Invalid input dimensions" << endl;
        return;
    }
    // 确保matrix是4x4的浮点矩阵
    matrix = Mat::zeros(4, 4, CV_64F);

    // 复制旋转部分
    R.copyTo(matrix(Rect(0, 0, 3, 3)));

    // 复制平移部分
    t.copyTo(matrix(Rect(3, 0, 1, 3)));

    // 设置齐次坐标的最后一行
    matrix.at<double>(3, 3) = 1.0;
}

// 辅助函数：分解变换矩阵（从4x4齐次变换矩阵提取旋转矩阵和平移向量）
void decompose(const Mat& matrix, Mat& R, Mat& t) {
    // 检查输入矩阵的维度
    if (matrix.rows != 4 || matrix.cols != 4) {
        cerr << "Error: Input matrix must be 4x4 for decomposition" << endl;
        return;
    }

    // 类型转换
    Mat matrix_double;
    matrix.convertTo(matrix_double, CV_64F);

    // 提取旋转部分 (3x3
    matrix_double(Rect(0, 0, 3, 3)).copyTo(R);
    // 提取平移部分 (3x1)
    matrix_double(Rect(3, 0, 1, 3)).copyTo(t);

    // 确保输出是正确类型
    R.convertTo(R, CV_64F);
    t.convertTo(t, CV_64F);
}

// 辅助函数：4x4的齐次变换矩阵求逆
Mat inver(const Mat& T, int method = 0) {
    // ==================== 1. 输入验证 ====================
    if (T.empty()) {
        cerr << "Error: Input matrix is empty" << endl;
        return Mat();
    }

    if (T.rows != 4 || T.cols != 4) {
        cerr << "Error: Input matrix must be 4x4" << endl;
        return Mat();
    }

    // ==================== 2. 转换为双精度 ====================
    Mat T_double;
    T.convertTo(T_double, CV_64F);

    // ==================== 3. 检查是否是刚体变换 ====================
    bool is_rigid_transform = true;

    // 检查最后一行是否为 [0,0,0,1]
    const double* last_row = T_double.ptr<double>(3);
    if (std::abs(last_row[0]) > 1e-12 ||
        std::abs(last_row[1]) > 1e-12 ||
        std::abs(last_row[2]) > 1e-12 ||
        std::abs(last_row[3] - 1.0) > 1e-12) {
        is_rigid_transform = false;
        cout << "Not a rigid transform (last row invalid)" << endl;
    }

    // 检查旋转矩阵部分的正交性
    Mat R = T_double(Rect(0, 0, 3, 3));
    Mat R_t;
    transpose(R, R_t);
    Mat shouldBeIdentity = R_t * R;
    Mat I_3x3 = Mat::eye(3, 3, CV_64F);
    double ortho_error = norm(I_3x3, shouldBeIdentity);

    if (ortho_error > 1e-8) {
        is_rigid_transform = false;
        cout << "Not a rigid transform (rotation not orthogonal), error: " << ortho_error << endl;
    }

    // ==================== 4. 如果是刚体变换，使用解析解 ====================
    if (is_rigid_transform) {

        Mat inv_T = Mat::eye(4, 4, CV_64F);
        Mat t = T_double(Rect(3, 0, 1, 3));

        // R^{-1} = R^T
        R_t.copyTo(inv_T(Rect(0, 0, 3, 3)));

        // t^{-1} = -R^T * t
        Mat t_inv = -R_t * t;
        t_inv.copyTo(inv_T(Rect(3, 0, 1, 3)));

        return inv_T;
    }

    // ==================== 5. 通用SVD求逆（最稳定） ====================
    Mat U, W, Vt;
    try {
        // 执行SVD分解
        SVD::compute(T_double, W, U, Vt, SVD::FULL_UV);

        // ==================== 6. 诊断矩阵条件 ====================
        double max_sv = W.at<double>(0);
        double min_sv = W.at<double>(W.rows - 1);
        double condition_number = max_sv / min_sv;


        cout << "Singular values: ";
        for (int i = 0; i < W.rows; i++) {
            cout << W.at<double>(i) << " ";
        }
        cout << endl;
        cout << "Condition number: " << condition_number << endl;


        // ==================== 7. 自适应奇异值截断 ====================
        double truncation_threshold;

        if (condition_number > 1e12) {
            // 极端病态情况
            truncation_threshold = max_sv * 1e-12;

        }
        else if (condition_number > 1e8) {
            // 严重病态情况
            truncation_threshold = max_sv * 1e-8;

        }
        else if (condition_number > 1e4) {
            // 中等病态情况
            truncation_threshold = max_sv * 1e-4;

        }
        else {
            // 良好条件
            truncation_threshold = max_sv * 1e-12;
        }

        // ==================== 8. 构建逆奇异值矩阵 ====================
        Mat W_inv = Mat::zeros(W.size(), CV_64F);
        int kept_singular_values = 0;

        for (int i = 0; i < W.rows; i++) {
            double singular_value = W.at<double>(i);

            if (singular_value > truncation_threshold) {
                W_inv.at<double>(i) = 1.0 / singular_value;
                kept_singular_values++;
            }
            else {
                // 设置为0而不是很小的值，避免放大噪声
                W_inv.at<double>(i) = 0.0;
            }
        }

        if (kept_singular_values == 0) {
            return Mat();
        }

        // ==================== 9. 计算逆矩阵 ====================
        // T^{-1} = V * W_inv * U^T
        Mat inv_T = Vt.t() * Mat::diag(W_inv) * U.t();

        // ==================== 10. 验证结果 ====================
        Mat verification = T_double * inv_T;
        Mat identity = Mat::eye(4, 4, CV_64F);
        double inversion_error = norm(identity, verification);
        cout << "Inversion error: " << inversion_error << endl;

        if (inversion_error > 1e-6) {
            cout << "Warning: Inversion error is relatively high" << endl;
        }

        return inv_T;
    }
    catch (const Exception& e) {
        cerr << "SVD computation failed: " << e.what() << endl;
        return Mat();
    }
}

// 辅助函数: 变换3D点
vector<Point3f> transformPoints(const vector<Point3f>& object_points, const Mat& b2o) {
    vector<Point3f> base_points;
    base_points.reserve(object_points.size());

    // 确保变换矩阵是float类型
    Mat b2o_float;
    b2o.convertTo(b2o_float, CV_32F);

    for (const auto& point : object_points) {
        // 将Point3f转换为4x1齐次坐标向量 [x, y, z, 1]^T
        Mat homogeneous_point = (Mat_<float>(4, 1) << point.x, point.y, point.z, 1.0f);

        // 应用变换矩阵
        Mat transformed_point = b2o_float * homogeneous_point;

        // 转换回3D坐标（齐次坐标归一化）
        float w = transformed_point.at<float>(3, 0);
        Point3f base_point(
            transformed_point.at<float>(0, 0) / w,
            transformed_point.at<float>(1, 0) / w,
            transformed_point.at<float>(2, 0) / w
        );

        base_points.push_back(base_point);
    }

    return base_points;
}

//// 辅助函数：将Mat转换为Eigen::Matrix4d变换矩阵
//Eigen::Matrix4d Mat2EigenM4d(const cv::Mat& cv_mat) {
//    Eigen::Matrix4d eigen_mat = Eigen::Matrix4d::Identity();
//
//    // 检查输入矩阵的尺寸和类型
//    if (cv_mat.rows != 4 || cv_mat.cols != 4) {
//        std::cerr << "Error: Input cv::Mat must be 4x4" << std::endl;
//        return eigen_mat;
//    }
//
//    // 处理不同的数据类型
//    if (cv_mat.type() == CV_64F) {
//        // double 类型
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                eigen_mat(i, j) = cv_mat.at<double>(i, j);
//            }
//        }
//    }
//    else if (cv_mat.type() == CV_32F) {
//        // float 类型
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                eigen_mat(i, j) = static_cast<double>(cv_mat.at<float>(i, j));
//            }
//        }
//    }
//    else {
//        std::cerr << "Error: Unsupported cv::Mat type. Use CV_32F or CV_64F" << std::endl;
//    }
//
//    return eigen_mat;
//}
////============================================================================================/