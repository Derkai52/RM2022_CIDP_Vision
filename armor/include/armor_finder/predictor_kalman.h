#ifndef CIDP_RM_CV_PREDICTOR_KALMAN_H
#define CIDP_RM_CV_PREDICTOR_KALMAN_H


#include "kalman.h"
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <serial.h>

//#include "../autoaim.hpp"
//#include <robot.hpp>
//#include <umt/umt.hpp>
#include <vector>

constexpr int S = 2;

class PredictorKalman {
private:
    struct ROI {
        bool ROI_selected = false;
        cv::Rect2f ROI_bbox;
        int last_class = -1;

        ROI()=default;
        ROI(cv::Rect2f &bbox, int &last): ROI_selected(true), ROI_bbox(bbox), last_class(last) {}
        inline void clear() {
            ROI_selected = false;
            last_class = -1;
        }
        ~ROI()=default;
    };

private:
    Eigen::Matrix3d R_CI;           // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
    Eigen::Matrix3d F;              // 相机内参矩阵EIGEN-Matrix
    Eigen::Matrix<double, 1, 5> C;  // 相机畸变矩阵EIGEN-Matrix
    cv::Mat R_CI_MAT;               // 陀螺仪坐标系到相机坐标系旋转矩阵CV-Mat
    cv::Mat F_MAT;                  // 相机内参矩阵CV-Mat
    cv::Mat C_MAT;                  // 相机畸变矩阵CV-Mat
    PredictorKalman::ROI roi;       // 划定之前击打装甲板的图像位置
    Serial serial;                 // 串口对象，引用外部变量，用于和能量机关共享同一个变量

    using _Kalman = Kalman<1, S>;
    _Kalman kalman;

    // 相机坐标系内坐标--->世界坐标系内坐标
    inline Eigen::Vector3d pc_to_pw(const Eigen::Vector3d &pc, const Eigen::Matrix3d &R_IW) {
        auto R_WC = (R_CI * R_IW).transpose();
        return R_WC * pc;
    }

    // 世界坐标系内坐标--->相机坐标系内坐标
    inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_IW) {
        auto R_CW = R_CI * R_IW;
        return R_CW * pw;
    }

    // 相机坐标系内坐标--->图像坐标系内像素坐标
    inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc) {
        return F * pc / pc(2, 0);
    }

    // 将世界坐标系内一点，投影到图像中，并绘制该点
    inline void re_project_point(cv::Mat &image, const Eigen::Vector3d &pw,
                                 const Eigen::Matrix3d &R_IW, const cv::Scalar &color) {
        Eigen::Vector3d pc = pw_to_pc(pw, R_IW);
        Eigen::Vector3d pu = pc_to_pu(pc);
        cv::circle(image, {int(pu(0, 0)), int(pu(1, 0))}, 3, color, 2);
    }

    // pnp解算:获取相机坐标系内装甲板坐标
    Eigen::Vector3d pnp_get_pc(const cv::Point2f p[4], int);
    bool sendTarget(Serial &serial, double x, double y, double z);

public:
    PredictorKalman();
    bool predict(const cv::Point2f armor_box_points[4], int &, cv::Mat &);


};

#endif //CIDP_RM_CV_PREDICTOR_KALMAN_H
