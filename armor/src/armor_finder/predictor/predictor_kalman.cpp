#include "armor_finder/predictor_kalman.h"
#include "armor_finder/armor_finder.h"
#include <opencv2/core/eigen.hpp>
#include <fmt/format.h>
#include <cmath>
#include <log.h>


#define PROJECT_SOURCE_DIR

constexpr double shoot_delay = 0.09;   // 射击延迟: 90ms

PredictorKalman::PredictorKalman() {
    // 1、初始化相机内参、坐标系转换关系
    cv::FileStorage fin(PROJECT_SOURCE_DIR"../camera-param.yml", cv::FileStorage::READ);
    if(!fin.isOpened()){
        LOGE("camera-param.yml don`t find or open fail!");
        throw 1;
    }

    fin["Tcb"] >> R_CI_MAT; // 陀螺仪坐标系到相机坐标系旋转矩阵
    fin["K"] >> F_MAT;      // 相机内参矩阵
    fin["D"] >> C_MAT;      // 相机畸变矩阵

    cv::cv2eigen(R_CI_MAT, R_CI);
    cv::cv2eigen(F_MAT, F);
    cv::cv2eigen(C_MAT, C);

    // 2、初始化卡尔曼
    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity();  // 初始化转移矩阵（单位矩阵）
    _Kalman::Matrix_zxd H;  // 初始化观测矩阵
    H(0, 0) = 1;
    _Kalman::Matrix_xxd R;  // 初始化预测过程噪声偏差的方差
    R(0, 0) = 0.01;
    for (int i = 1; i < S; i++) {
        R(i, i) = 100;
    }
    _Kalman::Matrix_zzd Q{4}; // 初始化测量噪声偏差
    _Kalman::Matrix_x1d init{0, 0}; //初始化 t-1 时刻的值
    kalman = _Kalman(A, H, R, Q, init, 0);
}

// 计算任意四边形的中心
cv::Point2f points_center(cv::Point2f pts[4]){
    for (int i = 0; i < 4; ++i) {
        for (int j = i+1; j < 4; ++j) {
            if (pts[i] == pts[j]) {
                std::cout << "[Error] Unable to calculate center point." << std::endl;
                return cv::Point2f{0, 0};
            }
        }
    }
    cv::Point2f center(0, 0);
    if (pts[0].x == pts[2].x && pts[1].x == pts[3].x) {
        std::cout << "[Error] Unable to calculate center point." << std::endl;
    }
    else if (pts[0].x == pts[2].x && pts[1].x != pts[3].x) {
        center.x = pts[0].x;
        center.y = (pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*(pts[0].x-pts[3].x)+pts[3].y;
    }
    else if (pts[1].x == pts[3].x && pts[0].x != pts[2].x) {
        center.x = pts[1].x;
        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(pts[1].x-pts[0].x)+pts[0].y;
    }
    else {
        center.x = (((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*pts[3].x - pts[3].y + \
                    pts[0].y - (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*pts[0].x)) / \
                    ((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)-(pts[2].y-pts[0].y)/(pts[2].x-pts[0].x));
        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(center.x-pts[0].x)+pts[0].y;
    }

    return center;
}

bool PredictorKalman::predict(const cv::Point2f armor_box_points[4], int &t, cv::Mat &im2show) {
    std::array<double, 4> q_;  // 初始化云台当前姿态角
    double robot_speed_mps = 18.0; // TODO: 通过下位机知晓当前发射初速度（m/s）
    q_[0] = -0.707;
    q_[1] = 0.707;
    q_[2] = 0.0;
    q_[3] = 0.0;

    Eigen::Quaternionf q_raw(q_[0], q_[1], q_[2], q_[3]);
    Eigen::Quaternionf q(q_raw.matrix().transpose());
//	std::cout<<qt[0]<<qt[1]<<qt[2]<<qt[3]<<std::endl;
    Eigen::Matrix3d R_IW = q.matrix().cast<double>();
    Eigen::Vector3d m_pc = pnp_get_pc(armor_box_points, 3);             // point camera: 目标在相机坐标系下的坐标  // TODO: 装甲板类型需要从识别类别中获取。
    Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);          // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）

    std::cout << "目标在世界坐标系下的坐标: " << m_pw << std::endl;

    static double last_yaw = 0, last_speed = 0;
    double mc_yaw = std::atan2(m_pc(1,0), m_pc(0,0));
    double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));   // yaw的测量值，单位弧度
//    std::cout << "m_yaw=" << m_yaw * 180. / M_PI << std::endl;
    if(std::fabs(last_yaw - m_yaw) > 5. / 180. * M_PI){
        kalman.reset(m_yaw, t);
        last_yaw = m_yaw;
        std::cout << "reset" << std::endl;
        return false;
    }

    last_yaw = m_yaw;
    Eigen::Matrix<double, 1, 1> z_k{m_yaw};
    _Kalman::Matrix_x1d state = kalman.update(z_k, t);                        // 更新卡尔曼滤波
    last_speed = state(1, 0);
    double c_yaw = state(0, 0);                                      // current yaw: yaw的滤波值，单位弧度
    double c_speed = state(1, 0) * m_pw.norm();                      // current speed: 角速度转线速度，单位m/s

//    std::cout << "[m_yaw测量值=" << m_yaw * 180. / M_PI<< "] [c_yaw滤波值: " << c_yaw * 180. / M_PI << "] [c_speed线速度:" << c_speed << std::endl;
//    std::cout << "t: " << t << " state(1, 0): " << state(1, 0) << std::endl;


//    std::cout << "角速度: " << c_speed << std::endl;
    double compensate_speed_signal = c_speed > 0 ? 1 : -1;
    double compensate_speed = compensate_speed_signal * c_speed * 1.0;
    c_speed += compensate_speed;                                   // speed compensate

//    std::cout << "state: " << state << std::endl;
    double predict_time = m_pw.norm() / robot_speed_mps + shoot_delay;           // 预测时间=发射延迟+飞行时间（单位:s）
    double p_yaw = m_yaw + atan2(predict_time * c_speed, m_pw.norm());     // predict yaw: yaw的预测值，直线位移转为角度，单位弧度
//    cout << "滤波值："<<c_yaw <<" | 预测值："<< p_yaw <<" | 预测时间："<<predict_time<<" | 目标距离："<< m_pw.norm()<<endl;

    double length = sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0));
    Eigen::Vector3d c_pw{length * cos(m_yaw), length * sin(m_yaw), m_pw(2, 0)};//反解位置(世界坐标系)
    Eigen::Vector3d p_pw{length * cos(p_yaw), length * sin(p_yaw), m_pw(2, 0)};

    double distance = p_pw.norm();                          // 目标距离（单位:m）
    double distance_xy = p_pw.topRows<2>().norm();
    double p_pitch = std::atan2(p_pw(2, 0), distance_xy);
    //std::cout << fmt::format("distance: {}", distance) << std::endl;
//    std::cout << state << std::endl;

    // 计算抛物线
    // 先解二次方程
//    std::cout << "p_pitch: " << p_pitch <<std::endl;
    double a = 9.8 * 9.8 * 0.25;
    double b = -robot_speed_mps * robot_speed_mps - distance * 9.8 * cos(M_PI_2 + p_pitch);
    double c = distance * distance;
    // 带入求根公式，解出t^2
    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
//    std::cout << fmt::format("a:{}, b:{}, c:{}", a, b, c) << std::endl;
//    std::cout << "t2:" << t_2 << std::endl;
    double fly_time = sqrt(t_2);                                       // 子弹飞行时间（单位:s）
    // 解出抬枪高度，即子弹下坠高度
    double height = 0.5 * 9.8 * t_2;

    //std::cout << m_pw << std::endl;
    float bs = robot_speed_mps;
    //std::cout << fmt::format("bullet_speed:{}, distance: {}, height: {}, p_pitch:{}",
    //                         bs, distance, height, p_pitch / M_PI * 180) << std::endl;
    Eigen::Vector3d s_pw{p_pw(0, 0), p_pw(1, 0), p_pw(2, 0) + height}; // 抬枪后预测点


    /// re-project test
    /// 把世界坐标系中的点，重投影到图像中
//    im2show = img.clone();
    re_project_point(im2show, c_pw, R_IW, {0, 255, 0}); // 绿色点（当前目标点）
    re_project_point(im2show, p_pw, R_IW, {255, 0, 0}); // 蓝色点（目标预测点）
    // here is special change for no.4
    re_project_point(im2show, s_pw, R_IW, {0, 0, 255}); // 红色点（枪口补偿点）
    for (int i = 0; i < 4; ++i)
        cv::circle(im2show, armor_box_points[i], 3, {0, 255, 0}); // 绿色点（灯条四顶点）
    cv::circle(im2show, {im2show.cols / 2, im2show.rows / 2}, 3, {0, 255 ,0});

    Eigen::Vector3d s_pc = pw_to_pc(s_pw, R_IW);
    double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
    double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;
    // 绘制角度波形图

    double yaw_angle = s_yaw;
    double pitch_angle = s_pitch;
    double yaw_speed = c_speed;
//    std::cout << "s_pitch: " << s_pitch << std::endl;
//
//    if (std::abs(yaw_angle) < 90. && std::abs(pitch_angle) < 90.)
//        shoot_mode = static_cast<uint8_t>(ShootMode::COMMON);
//    else
//        shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
//	std::cout << "yaw角度: " << yaw_angle << " pitch角度: " << pitch_angle << " yaw速度: " << yaw_speed << "距离" << distance << std::endl;
    sendTarget(serial, yaw_angle, pitch_angle, distance);
    return true;

}

Eigen::Vector3d PredictorKalman::pnp_get_pc(const cv::Point2f p[4], int armor_number) {
    static const std::vector<cv::Point3d> pw_small = {  // 单位：m
            {-0.066, 0.027,  0.},
            {-0.066, -0.027, 0.},
            {0.066,  -0.027, 0.},
            {0.066,  0.027,  0.}
    };
    static const std::vector<cv::Point3d> pw_big = {    // 单位：m
            {-0.115, 0.029,  0.},
            {-0.115, -0.029, 0.},
            {0.115,  -0.029, 0.},
            {0.115,  0.029,  0.}
    };
    std::vector<cv::Point2d> pu(p, p + 4);
    cv::Mat rvec, tvec;

    if (armor_number == 0 || armor_number == 1)
        cv::solvePnP(pw_big, pu, F_MAT, C_MAT, rvec, tvec);
    else
        cv::solvePnP(pw_small, pu, F_MAT, C_MAT, rvec, tvec);

    Eigen::Vector3d pc;
    cv::cv2eigen(tvec, pc);
    return pc;
}

// 卡尔曼预测模式的数据发送
bool PredictorKalman::sendTarget(Serial &serial, double x, double y, double z) {
    static short x_tmp, y_tmp, z_tmp;
    uint16_t shoot_delay = 0;
    uint8_t buff[10];

    x_tmp = static_cast<short>(x * (32768 - 1) / 100);
    y_tmp = static_cast<short>(y * (32768 - 1) / 100);
    z_tmp = static_cast<short>(z * (32768 - 1) / 1000);

    buff[0] = 's';
    buff[1] = static_cast<char>((x_tmp >> 8) & 0xFF);
    buff[2] = static_cast<char>((x_tmp >> 0) & 0xFF);
    buff[3] = static_cast<char>((y_tmp >> 8) & 0xFF);
    buff[4] = static_cast<char>((y_tmp >> 0) & 0xFF);
    buff[5] = static_cast<char>((z_tmp >> 8) & 0xFF);
    buff[6] = static_cast<char>((z_tmp >> 0) & 0xFF);
    buff[7] = static_cast<char>((shoot_delay >> 8) & 0xFF);
    buff[8] = static_cast<char>((shoot_delay >> 0) & 0xFF);
    buff[9] = 'e';

//    if(buff[7]<<8 | buff[8])
//        cout << (buff[7]<<8 | buff[8]) << endl;
    return serial.WriteData(buff, sizeof(buff));
}

//std::unique_ptr<PredictorBase> make_predictor_kalman() {
//    return std::make_unique<PredictorKalman>();
//}