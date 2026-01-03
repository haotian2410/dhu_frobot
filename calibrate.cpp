#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "function.h"
#include <cstdlib>  

#define PI 3.14159265358979323846

using namespace std;
using namespace cv;
using namespace ceres;
using namespace Eigen;

//============================================================================================/
//文件读取、存储路径
static std::string BASE =
    (std::getenv("ROBOT_HOME") ? std::getenv("ROBOT_HOME") : std::string("."));
static std::string dir = BASE + "/";   // 项目根目录
static string img_dir = dir + "Data/Calibration/Image/";                // 参与标定的图像的路径源
static string image = dir + "Data//Calibration/image_list.txt";         // 参与标定的图像名称文件
static string robot = dir + "Data//Calibration/robot_pose.txt";         // 机械臂末端位姿信息文件
static string chessboard_dir = dir + "Result/Match/ChessBoard/";        // 棋盘格角点检测结果文件
static string camera = dir + "Result/Reprojection/CameraCali/";         // 相机标定重投影结果文件
static string handeye = dir + "Result/Reprojection/HandeyeCali/";       // 相机标定重投影结果文件
//============================================================================================/
// 读取基本信息
static Size image_size;                            // 待标定的图像尺寸
static int cali_imgs = 24;                         // 待标定的图像数量
static vector<Mat> e2b_matrices;                   // 末端到基底的变换矩阵
static vector<string> image_list;                   // 待标定图像的路径列表
//==============================================================/
// 角点检测
static vector<Mat> images;                         // 保存所有标定图像
static vector<vector<Point3f>> object_points_all;  // 物体坐标系中三维点
static vector<vector<Point2f>> image_points_all;   // 图像坐标系中二维点
static vector<vector<Point3f>> base_points_all;    // 基底坐标系中三维点
static Size board_size = Size(8, 6);               // 棋盘内角点数量
static double square_size = 25;                     // 每格对应的距离(mm）
//==============================================================/
// 相机标定部分
static Mat camera_matrix;                          // 相机矩阵
static Mat dist_coeffs;                            // 畸变系数
static vector<Mat> o2c_matrices;                   // 物体到相机的变换矩阵
//==============================================================/
// 手眼标定部分
static Mat c2e_matrix = Mat::eye(4, 4, CV_64F);    // 相机到末端的(手眼变换)矩阵
static Mat o2b_matrix = Mat::eye(4, 4, CV_64F);    // 物体到基底变换矩阵
//============================================================================================/


// 1.读取各个图像
//============================================================================================/
static void ReadImage()
{
    ifstream inputtext(image);
    image_list.resize(0); // 初始化传入的list向量
    string img;
    for (int i = 0; i < cali_imgs; i++) {
        inputtext >> img;
        image_list.push_back(img_dir + img); // 将每个图像名称存入list
    }
    inputtext.close();
}

// 2.读入机器人末端执行器的位姿。
//============================================================================================/
static void ReadRobot(){
    ifstream inputtext(robot);
    double val[6];// x, y, z ,rx, ry, rz
    for (int i = 0; i < cali_imgs; i++) {

        // 按行读取
        inputtext >> val[0] >> val[1] >> val[2] >> val[3] >> val[4] >> val[5];
        val[3] = val[3] * PI / 180;
        val[4] = val[4] * PI / 180;
        val[5] = val[5] * PI / 180;

        // 获取旋转矩阵
        Mat R = eulerToRodrigues(val[3], val[4], val[5]);
        // 获取平移向量
        Mat tvec = (Mat_<double>(3, 1) << val[0], val[1], val[2]); 

        // 获取末端到基底的变换矩阵
        Mat e2b = Mat::zeros(4, 4, CV_64F);
        compose(e2b, R, tvec);
        e2b_matrices.push_back(e2b);
    }
    inputtext.close();
}

// 3.使用棋盘格检测角点
//============================================================================================/
static void ChessBoard() {

    vector<Point3f> objp;
    // 生成世界坐标系中的3D点
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            objp.push_back(Point3f(j * square_size, i * square_size, 0));
        }
    }
    // 处理每张图像，检测棋盘格角点
    for (int i = 0; i < cali_imgs; i++) {
        Mat gray;
        Mat img = imread(image_list[i], IMREAD_COLOR);
        if (i == 0) {
            image_size.width = img.cols;
            image_size.height = img.rows;
            cout << "image width : " << image_size.width << "px" << endl;
            cout << "image height : " << image_size.height << "px" << endl;
        }
        images.push_back(img.clone());
        enhanceImage(img, img);
        cvtColor(img, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, board_size, corners);
        if (found) {

            // 寻找角点
            cv::find4QuadCornerSubpix(gray, corners, cv::Size(30, 30));
            // 如果内角点合理，则进行记录
            object_points_all.push_back(objp);
            image_points_all.push_back(corners);

            // 绘制内角点
            Mat imageCopy = img.clone();
            drawChessboardCorners(imageCopy, board_size, corners, found);
            string match_filename = chessboard_dir + to_string(i + 1) + ".jpg";
            cout << "Corners Finding...(" << i + 1 << "/" << cali_imgs << ")\r";
            imwrite(match_filename, imageCopy);
        }
    }
    cout << endl << "All Corners Have Found!" << endl << endl;
}

// 4.使用calibrateCamera函数优化相机内参和每张图像的外参。
//============================================================================================/
static void Calibration() {
    cout << "Camera Calibrating...\r";
    // 使用calibrateCamera函数进行优化
    vector<Mat> rvecs, tvecs; // 每张图像的位姿信息
    double reproj_error = calibrateCamera(object_points_all, image_points_all, image_size,
        camera_matrix, dist_coeffs, rvecs, tvecs, 0);

    cout << "\nCalibration results:" << endl;
    cout << "Reprojection error: " << reproj_error << " pixels" << endl; // RMS均方根误差
    cout << "Camera matrix:\n" << camera_matrix << endl;
    cout << "Distortion coefficients: " << endl << dist_coeffs << endl << endl;

    // 计算每幅图像的位姿信息
    for (int i = 0; i < object_points_all.size(); i++) {
        Mat R;
        Rodrigues(rvecs[i], R);//罗德里格斯公式，旋转向量转旋转矩阵

        Mat o2c = Mat::zeros(4, 4, CV_64F);
        compose(o2c, R, tvecs[i]);
        o2c_matrices.push_back(o2c);  //存入每张图像到相机的变换矩阵

        // 可视化重投影误差
        vector<Point2f> projected_points;
        projectPoints(object_points_all[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);

        Mat img_reprojection = images[i].clone();
        for (size_t j = 0; j < image_points_all[i].size(); j++) {
            // 绿色为原始点
            circle(img_reprojection, image_points_all[i][j], 1, Scalar(0, 255, 0), -1);
            // 红色为重投影点
            circle(img_reprojection, projected_points[j], 1, Scalar(0, 0, 255), -1);
            //line(img_reprojection, image_points_all[i][j], projected_points[j], Scalar(255, 255, 0), 1);
        }
        string reprojection_filename = camera + to_string(i + 1) + ".jpg";
        cv::imwrite(reprojection_filename, img_reprojection);
    }
}

// 5.使用calibrateHanaeye()函数计算机械臂末端到相机坐标系的变换
//============================================================================================/
static void HandEye() {

    vector<Mat> e2b_R, e2b_t, o2c_R, o2c_t;
    Mat c2e_R, c2e_t;

    for (int i = 0; i < cali_imgs; i++) {
        Mat R1, t1, R2, t2;

        // 分解e2b
        decompose(e2b_matrices[i], R1, t1);
        e2b_R.push_back(R1);
        e2b_t.push_back(t1);

        // 分解e2c
        decompose(o2c_matrices[i], R2, t2);
        o2c_R.push_back(R2);
        o2c_t.push_back(t2);
    }

    calibrateHandEye(e2b_R, e2b_t, o2c_R, o2c_t, c2e_R, c2e_t, CALIB_HAND_EYE_TSAI);

    compose(c2e_matrix, c2e_R, c2e_t);
    cout << endl << "HandeyeCalibration results:" << endl;
    cout << "c2e matrix:" << endl << c2e_matrix << endl;
}

// 6.计算物体到基底的变换矩阵
//============================================================================================/
static void O2bTransform() {

    // 直接平均所有变换向量
    cv::Mat avgTranslation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat avgRotation = cv::Mat::zeros(3, 1, CV_64F);

    vector<Mat> o2b_matrices;
    // 计算每个视图的物体到基底变换
    for (int i = 0; i < cali_imgs; i++) {
        // 计算变换
        Mat o2b = e2b_matrices[i] * c2e_matrix * o2c_matrices[i];
        o2b_matrices.push_back(o2b);
        
        Mat R, rvec, tvec;
        decompose(o2b, R, tvec);
        Rodrigues(R, rvec);

        avgTranslation += tvec;
        avgRotation += rvec;

    }

    Mat avgR = cv::Mat::zeros(3, 3, CV_64F);
    Mat avgT = cv::Mat::zeros(3, 1, CV_64F);
    Rodrigues(avgRotation, avgR);
    avgT = avgTranslation / o2b_matrices.size();

    // 对旋转矩阵进行正交化，确保它是有效的旋转矩阵
    Mat U, W, Vt;
    cv::SVD::compute(avgR, W, U, Vt);
    avgR = U * Vt;
    // 确保行列式为+1
    if (cv::determinant(avgR) < 0) {
        cv::Mat temp = Vt.clone();
        temp.row(2) = -temp.row(2);
        avgR = U * temp;
    }
    compose(o2b_matrix, avgR, avgT);
    cout << endl << "o2b matrix:" << endl << o2b_matrix << endl;
}

// 7.验证手眼变换矩阵c2e_matrix
//============================================================================================/
void validateHandeye(bool option) {
    double err = 0.0;
    for (int i = 0; i < cali_imgs; i++) {

        vector<Point2f> reprojected;
        Mat R, rvec, tvec;

        // 根据变换链计算变换矩阵: o2c = e2c * b2e * o2b
        Mat o2c = inver(c2e_matrix) * inver(e2b_matrices[i]) * o2b_matrix;
        decompose(o2c, R, tvec);
        Rodrigues(R, rvec);

        // 进行重投影误差
        projectPoints(object_points_all[i], rvec, tvec, camera_matrix, dist_coeffs, reprojected);
        double error = norm(image_points_all[i], reprojected, NORM_L2);
        err += error / sqrt(reprojected.size());

        // 选择是否可视化手眼矩阵重投影
        if (option) {
            Mat img_reprojection = images[i].clone();
            for (int j = 0; j < image_points_all[i].size(); j++) {
                // 绿色为原始点
                circle(img_reprojection, image_points_all[i][j], 1, Scalar(0, 255, 0), -1);

                // 红色为重投影点
                circle(img_reprojection, reprojected[j], 1, Scalar(0, 0, 255), -1);
            }
            string reprojection_filename = handeye + to_string(i + 1) + ".jpg";
            cv::imwrite(reprojection_filename, img_reprojection);
        }
    }
    cout << "Reprojection error :" << err / cali_imgs << endl;
}

// 8.通过ceres联合优化c2e_matrix, o2b_matrix以及相机内参
//============================================================================================/
// 手眼标定残差块
struct HandEyeObjectBaseCostFunction {
    HandEyeObjectBaseCostFunction(const Eigen::Matrix4d& e2b,
        const Eigen::Matrix4d& o2c)
        : e2b_(e2b), o2c_(o2c) {
    }

    template<typename T>
    bool operator()(const T* const c2e_params, const T* const o2b_params, T* residual) const {
        // c2e_params: [rx, ry, rz, tx, ty, tz] 相机到末端的变换
        // o2b_params: [rx, ry, rz, tx, ty, tz] 物体到基底的变换

        // 1. 构建c2e变换矩阵
        T c2e_rotation_vector[3] = { c2e_params[0], c2e_params[1], c2e_params[2] };
        T c2e_rotation_matrix[9];
        ceres::AngleAxisToRotationMatrix(c2e_rotation_vector, c2e_rotation_matrix);

        Eigen::Matrix<T, 4, 4> c2e = Eigen::Matrix<T, 4, 4>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                c2e(i, j) = c2e_rotation_matrix[3 * i + j];
            }
        }
        c2e(0, 3) = c2e_params[3];
        c2e(1, 3) = c2e_params[4];
        c2e(2, 3) = c2e_params[5];

        // 2. 构建o2b变换矩阵
        T o2b_rotation_vector[3] = { o2b_params[0], o2b_params[1], o2b_params[2] };
        T o2b_rotation_matrix[9];
        ceres::AngleAxisToRotationMatrix(o2b_rotation_vector, o2b_rotation_matrix);

        Eigen::Matrix<T, 4, 4> o2b = Eigen::Matrix<T, 4, 4>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                o2b(i, j) = o2b_rotation_matrix[3 * i + j];
            }
        }
        o2b(0, 3) = o2b_params[3];
        o2b(1, 3) = o2b_params[4];
        o2b(2, 3) = o2b_params[5];

        // 3. 将Eigen::Matrix转换为Eigen::Matrix<T, 4, 4>
        Eigen::Matrix<T, 4, 4> e2b_T = e2b_.cast<T>();
        Eigen::Matrix<T, 4, 4> o2c_T = o2c_.cast<T>();

        // 4. 计算残差：e2b * c2e * o2c - o2b
        Eigen::Matrix<T, 4, 4> transform_chain = e2b_T * c2e * o2c_T;
        Eigen::Matrix<T, 4, 4> error_matrix = transform_chain - o2b;

        // 5. 将4x4误差矩阵展平为16维残差向量
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                residual[4 * i + j] = error_matrix(i, j);
            }
        }

        return true;
    }

private:
    const Eigen::Matrix4d e2b_;
    const Eigen::Matrix4d o2c_;
};

// 重投影误差残差块
struct ReprojectionCostFunctionWithCameraParams {
    ReprojectionCostFunctionWithCameraParams(const cv::Point3f& object_point,
        const cv::Point2f& image_point,
        const Eigen::Matrix4d& e2b)
        : object_point_(object_point), image_point_(image_point), e2b_(e2b) {
    }

    template<typename T>
    bool operator()(const T* const c2e_params,
        const T* const o2b_params,
        const T* const camera_params,  // [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        const T* const dist_params,    // 畸变系数 [k1, k2, p1, p2, k3]
        T* residual) const {

        // 1. 提取相机参数
        T fx = camera_params[0];
        T fy = camera_params[1];
        T cx = camera_params[2];
        T cy = camera_params[3];

        // 2. 提取畸变系数
        T k1 = dist_params[0];
        T k2 = dist_params[1];
        T p1 = dist_params[2];
        T p2 = dist_params[3];
        T k3 = dist_params[4];

        // 3. 构建c2e变换矩阵
        T c2e_rotation_vector[3] = { c2e_params[0], c2e_params[1], c2e_params[2] };
        T c2e_rotation_matrix[9];
        ceres::AngleAxisToRotationMatrix(c2e_rotation_vector, c2e_rotation_matrix);

        Eigen::Matrix<T, 4, 4> c2e = Eigen::Matrix<T, 4, 4>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                c2e(i, j) = c2e_rotation_matrix[3 * i + j];
            }
        }
        c2e(0, 3) = c2e_params[3];
        c2e(1, 3) = c2e_params[4];
        c2e(2, 3) = c2e_params[5];

        // 4. 构建o2b变换矩阵
        T o2b_rotation_vector[3] = { o2b_params[0], o2b_params[1], o2b_params[2] };
        T o2b_rotation_matrix[9];
        ceres::AngleAxisToRotationMatrix(o2b_rotation_vector, o2b_rotation_matrix);

        Eigen::Matrix<T, 4, 4> o2b = Eigen::Matrix<T, 4, 4>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                o2b(i, j) = o2b_rotation_matrix[3 * i + j];
            }
        }
        o2b(0, 3) = o2b_params[3];
        o2b(1, 3) = o2b_params[4];
        o2b(2, 3) = o2b_params[5];

        // 5. 计算物体到相机的变换：o2c = c2e^{-1} * e2b^{-1} * o2b
        Eigen::Matrix<T, 4, 4> e2b_T = e2b_.cast<T>();

        Eigen::Matrix<T, 4, 4> c2e_inv = c2e.inverse();
        Eigen::Matrix<T, 4, 4> e2b_inv = e2b_T.inverse();
        Eigen::Matrix<T, 4, 4> o2c = c2e_inv * e2b_inv * o2b;

        // 6. 提取旋转和平移
        Eigen::Matrix<T, 3, 3> R = o2c.block(0, 0, 3, 3);
        Eigen::Matrix<T, 3, 1> t = o2c.block(0, 3, 3, 1);

        // 7. 将3D点转换到相机坐标系
        Eigen::Matrix<T, 3, 1> object_pt;
        object_pt << T(object_point_.x), T(object_point_.y), T(object_point_.z);

        Eigen::Matrix<T, 3, 1> camera_pt = R * object_pt + t;

        // 8. 投影到图像平面，即归一化
        T x = camera_pt(0) / camera_pt(2);
        T y = camera_pt(1) / camera_pt(2);

        // 9. 应用畸变
        T r2 = x * x + y * y;
        T r4 = r2 * r2;
        T r6 = r4 * r2;

        T radial_distortion = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
        T x_distorted = x * radial_distortion + T(2.0) * p1 * x * y + p2 * (r2 + T(2.0) * x * x);
        T y_distorted = y * radial_distortion + p1 * (r2 + T(2.0) * y * y) + T(2.0) * p2 * x * y;

        // 10. 应用相机内参
        T u = fx * x_distorted + cx;
        T v = fy * y_distorted + cy;

        // 11. 计算重投影误差
        residual[0] = u - T(image_point_.x);
        residual[1] = v - T(image_point_.y);

        return true;
    }

private:
    const cv::Point3f object_point_;
    const cv::Point2f image_point_;
    const Eigen::Matrix4d e2b_;
};

// 主优化函数 
static void Optimization() {
    cout << endl << "Starting Full Ceres optimization (c2e, o2b, camera, distortion)..." << endl;

    // 初始化c2e优化参数
    Mat c2e_R, c2e_t;
    decompose(c2e_matrix, c2e_R, c2e_t);
    c2e_R.convertTo(c2e_R, CV_64F);
    c2e_t.convertTo(c2e_t, CV_64F);

    Mat c2e_rvec;
    Rodrigues(c2e_R, c2e_rvec);
    c2e_rvec.convertTo(c2e_rvec, CV_64F);

    double c2e_params[6] = {
        c2e_rvec.at<double>(0), c2e_rvec.at<double>(1), c2e_rvec.at<double>(2),
        c2e_t.at<double>(0), c2e_t.at<double>(1), c2e_t.at<double>(2)
    };

    // 初始化o2b优化参数
    Mat o2b_R, o2b_t;
    decompose(o2b_matrix, o2b_R, o2b_t);
    o2b_R.convertTo(o2b_R, CV_64F);
    o2b_t.convertTo(o2b_t, CV_64F);

    Mat o2b_rvec;
    Rodrigues(o2b_R, o2b_rvec);
    o2b_rvec.convertTo(o2b_rvec, CV_64F);

    double o2b_params[6] = {
        o2b_rvec.at<double>(0), o2b_rvec.at<double>(1), o2b_rvec.at<double>(2),
        o2b_t.at<double>(0), o2b_t.at<double>(1), o2b_t.at<double>(2)
    };

    // 初始化相机参数 [fx, fy, cx, cy]
    double camera_params[4] = {
        camera_matrix.at<double>(0, 0),  // fx
        camera_matrix.at<double>(1, 1),  // fy
        camera_matrix.at<double>(0, 2),  // cx
        camera_matrix.at<double>(1, 2)   // cy
    };

    // 初始化畸变系数 [k1, k2, p1, p2, k3]
    double dist_params[5] = {
        dist_coeffs.at<double>(0),  // k1
        dist_coeffs.at<double>(1),  // k2
        dist_coeffs.at<double>(2),  // p1
        dist_coeffs.at<double>(3),  // p2
        dist_coeffs.at<double>(4)   // k3
    };

    cout << "Initial camera parameters: " << endl;
    cout << "fx: " << camera_params[0] << ", fy: " << camera_params[1]
        << ", cx: " << camera_params[2] << ", cy: " << camera_params[3] << endl;
    cout << "Distortion: " << dist_params[0] << ", " << dist_params[1] << ", "
        << dist_params[2] << ", " << dist_params[3] << ", " << dist_params[4] << endl << endl;

    // 创建Ceres问题
    ceres::Problem problem;

    // 添加手眼标定残差块（不使用相机参数）
    for (int i = 0; i < cali_imgs; ++i) {
        Eigen::Matrix4d e2b_eigen = Mat2EigenM4d(e2b_matrices[i]);
        Eigen::Matrix4d o2c_eigen = Mat2EigenM4d(o2c_matrices[i]);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<HandEyeObjectBaseCostFunction, 16, 6, 6>(
                new HandEyeObjectBaseCostFunction(e2b_eigen, o2c_eigen));

        problem.AddResidualBlock(cost_function, nullptr, c2e_params, o2b_params);
    }

    // 添加重投影误差残差块（使用相机参数）
    for (int img_idx = 0; img_idx < cali_imgs; ++img_idx) {
        if (img_idx >= object_points_all.size()) continue;

        // 使用部分点以减少计算量
        for (int pt_idx = 0; pt_idx < object_points_all[img_idx].size(); pt_idx += 2) {
            Eigen::Matrix4d e2b_eigen = Mat2EigenM4d(e2b_matrices[img_idx]);

            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<ReprojectionCostFunctionWithCameraParams, 2, 6, 6, 4, 5>(
                    new ReprojectionCostFunctionWithCameraParams(
                        object_points_all[img_idx][pt_idx],
                        image_points_all[img_idx][pt_idx],
                        e2b_eigen));

            problem.AddResidualBlock(cost_function, nullptr,
                c2e_params, o2b_params,
                camera_params, dist_params);
        }
    }

    // 添加约束
    // 焦距应为正
    problem.SetParameterLowerBound(camera_params, 0, 0.1);  // fx > 0.1
    problem.SetParameterLowerBound(camera_params, 1, 0.1);  // fy > 0.1

    // 主点应在图像范围内
    problem.SetParameterLowerBound(camera_params, 2, 0);    // cx > 0
    problem.SetParameterLowerBound(camera_params, 3, 0);    // cy > 0
    problem.SetParameterUpperBound(camera_params, 2, image_size.width);   // cx < image_width
    problem.SetParameterUpperBound(camera_params, 3, image_size.height);  // cy < image_height

    // 配置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 200;  // 更多迭代因为参数更多
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-10;
    options.gradient_tolerance = 1e-10;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;

    // 更新手眼
    double c2e_rotation_vector[3] = { c2e_params[0], c2e_params[1], c2e_params[2] };
    double c2e_rotation_matrix[9];
    ceres::AngleAxisToRotationMatrix(c2e_rotation_vector, c2e_rotation_matrix);

    Mat optimized_c2e_R = (Mat_<double>(3, 3) <<
        c2e_rotation_matrix[0], c2e_rotation_matrix[1], c2e_rotation_matrix[2],
        c2e_rotation_matrix[3], c2e_rotation_matrix[4], c2e_rotation_matrix[5],
        c2e_rotation_matrix[6], c2e_rotation_matrix[7], c2e_rotation_matrix[8]);

    Mat optimized_c2e_t = (Mat_<double>(3, 1) <<
        c2e_params[3], c2e_params[4], c2e_params[5]);

    compose(c2e_matrix, optimized_c2e_R, optimized_c2e_t);

    // 更新o2b_matrix
    double o2b_rotation_vector[3] = { o2b_params[0], o2b_params[1], o2b_params[2] };
    double o2b_rotation_matrix[9];
    ceres::AngleAxisToRotationMatrix(o2b_rotation_vector, o2b_rotation_matrix);

    Mat optimized_o2b_R = (Mat_<double>(3, 3) <<
        o2b_rotation_matrix[0], o2b_rotation_matrix[1], o2b_rotation_matrix[2],
        o2b_rotation_matrix[3], o2b_rotation_matrix[4], o2b_rotation_matrix[5],
        o2b_rotation_matrix[6], o2b_rotation_matrix[7], o2b_rotation_matrix[8]);

    Mat optimized_o2b_t = (Mat_<double>(3, 1) <<
        o2b_params[3], o2b_params[4], o2b_params[5]);

    compose(o2b_matrix, optimized_o2b_R, optimized_o2b_t);

    // 更新相机矩阵和畸变系数
    camera_matrix.at<double>(0, 0) = camera_params[0];  // fx
    camera_matrix.at<double>(1, 1) = camera_params[1];  // fy
    camera_matrix.at<double>(0, 2) = camera_params[2];  // cx
    camera_matrix.at<double>(1, 2) = camera_params[3];  // cy

    dist_coeffs.at<double>(0) = dist_params[0];  // k1
    dist_coeffs.at<double>(1) = dist_params[1];  // k2
    dist_coeffs.at<double>(2) = dist_params[2];  // p1
    dist_coeffs.at<double>(3) = dist_params[3];  // p2
    dist_coeffs.at<double>(4) = dist_params[4];  // k3

    cout << endl << "Optimization results:" << endl;
    cout << "c2e_matrix:" << endl << c2e_matrix << endl;
    cout << "o2b_matrix:" << endl << o2b_matrix << endl;
    cout << "camera_matrix:" << endl << camera_matrix << endl;
    cout << "dist_coeffs: "  << endl << dist_coeffs << endl;
}

// 9.存储手眼标定的信息
void Record() {
    ofstream f_camera;
    f_camera.open(camera + "param.txt");
    for (int i = 0; i < 3; i++)
        f_camera << camera_matrix.at<double>(i, 0) << " " << camera_matrix.at<double>(i, 1) << " " << camera_matrix.at<double>(i, 2) << endl;
    for (int i = 0; i < 5; i++)
        f_camera << dist_coeffs.at<double>(i) << " ";
    f_camera.close();

    ofstream f_handeye;
    f_handeye.open(handeye + "param.txt");
    for (int i = 0; i < 4; i++)
        f_handeye << c2e_matrix.at<double>(i, 0) << " " << c2e_matrix.at<double>(i, 1) << " " << c2e_matrix.at<double>(i, 2) << " " << c2e_matrix.at<double>(i, 3) << endl;
    for (int i = 0; i < 4; i++)
        f_handeye << o2b_matrix.at<double>(i, 0) << " " << o2b_matrix.at<double>(i, 1) << " " << o2b_matrix.at<double>(i, 2) << " " << o2b_matrix.at<double>(i, 3) << endl;
    f_handeye.close();

}

//============================================================================================/
int mai1n() {

    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);// 抑制 OpenCV 的信息消息

    // 1.读取各个图像信息。
    ReadImage();

    // 2.读入机械臂末端信息。
    ReadRobot();

    // 3.获取各图像角点信息。
    ChessBoard();

    // 4.使用calibrateCamera()函数计算相机内参和每张图像的外参。
    Calibration();

    // 5.使用calibrateHandeye()函数计算机械臂末端到相机坐标系的变换
    HandEye();

    // 6.优化基底到物体的变换矩阵o2b_matrix
    O2bTransform();

    // 7.验证手眼变换矩阵c2e_matrix
    validateHandeye(false);

    // 8.通过ceres联合优化 & 验证优化后的结果
    Optimization();
    validateHandeye(true);

    // 9.存储手眼标定的信息
    Record();

    return 0;
}
//============================================================================================/


