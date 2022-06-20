#include "armor_finder/predictor_kalman.h"
#include "armor_finder/armor_finder.h"
#include <opencv2/core/eigen.hpp>
#include <fmt/format.h>
#include <cmath>
#include <log.h>


extern McuData mcu_data;     // 下位机数据
extern double bind_yaw;
constexpr double shoot_delay = 0.09;   // 射击延迟: 90ms
constexpr double gimbal_delay = 0.3;   // 云台响应延迟: 1ms

PredictorKalman::PredictorKalman() {
    // 1、初始化相机内参、坐标系转换关系
    cv::FileStorage fin(PROJECT_DIR"/camera-param-6mm.yml", cv::FileStorage::READ); // 根据实际使用相机使用的镜头型号
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

// 使用预测模式(自动瞄准)
bool PredictorKalman::predict(const cv::Point2f armor_box_points[4], int id, long long int t, cv::Mat &im2show) {
    std::array<double, 4> q_;  // 初始化云台当前姿态角
    double robot_speed_mps = 15.0; // TODO: 应当通过下位机知晓当前发射初速度（m/s）
    DebugT(1, 1,"目标颜色:" << (int)mcu_data.enemy_color << " 当前模式: 自瞄" )
    DebugT(1, 2,"云台姿态(Yaw):" <<std::left<<setw(10)<<bind_yaw*180.0/M_PI)
    DebugT(30, 2,"目标:" <<id2name[id])
    DebugT(40, 2," 射速:" << robot_speed_mps << " m/s")
    DebugT(1, 7,"预测状态:")


    // TODO: 若能通过陀螺仪知晓当前云台姿态角度，则注释这段
    //////////////////////////////////////////////////////////////////////////
    // 在线可视化四元数转欧拉角 https://quaternions.online/
    q_[0] =  0.500;
    q_[1] =  -0.500; // 示例欧拉角XYZ（-90，90，0）
    q_[2] =  0.500;
    q_[3] =  -0.500;

    Eigen::Quaternionf q_raw(q_[0], q_[1], q_[2], q_[3]);  // 显示陀螺仪姿态数据
    Eigen::Quaternionf q(q_raw.matrix().transpose());
    Eigen::Matrix3d R_IW = q.matrix().cast<double>();
/////////////////////////////////////////////////////////////////////////////////
    Eigen::Vector3d m_pc = pnp_get_pc(armor_box_points, id);  // point camera: 目标在相机坐标系下的坐标
    Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);          // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）

    m_pw(0, 0) /=2;
    DebugT(1, 4,"相机坐标系(m):"<< " X:"<<m_pw(1,0)<<" Y:"<<m_pw(2,0)<<" Z:"<<std::left<<setw(10)<<m_pw(0,0));
    // 相机坐标系转换为云台坐标系
    double ptz_camera_x = 0.;      //    PTZ_CAMERA_X - 相机与云台的 X 轴偏移(左负右正) 【云台与相机 相机作为参考点 单位m】
    double ptz_camera_y = 0.;      //    PTZ_CAMERA_Y - 相机与云台的 Y 轴偏移(上负下正)
    double ptz_camera_z = -0.10;   //    PTZ_CAMERA_Z - 相机与云台的 Z 轴偏移(前正后负) //理论应为实际值5cm，这里取4cm是因为后续弹道计算没有考虑空气阻力，最终误差"相互抵消"取得了平衡

    static double theta = 0; // 定义相机旋转角度
    // 定义旋转矩阵
    static double r_data[] = {1, 0,           0,
                              0, cos(theta),  sin(theta),
                              0, -sin(theta), cos(theta)};
    // 定义平移矩阵
    static double t_data[] = {static_cast<double>(ptz_camera_z),
                              static_cast<double>(ptz_camera_y),
                              static_cast<double>(ptz_camera_x)};

    Eigen::Matrix<double, 3, 3> r_camera_ptz = Eigen::Matrix<double, 3, 3>(r_data);
    Eigen::Vector3d t_camera_ptz = Eigen::Vector3d(t_data);
    m_pw = r_camera_ptz * m_pw - t_camera_ptz; // 相机坐标系 转换为 云台坐标系
    DebugT(1, 5,"云台坐标系(m):"<< " X:"<<m_pw(1,0)<<" Y:"<<m_pw(2,0)<<" Z:"<<std::left<<setw(10)<<m_pw(0,0))

    static double last_yaw = 0, last_speed = 0;
    double mc_yaw = std::atan2(m_pc(1,0), m_pc(0,0));
    double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));   // yaw的测量值，单位弧度

//    // 数据拟合
//    double a_fitting = -0.00804152124557575;
//    double b_fitting = -1.11954055687107;
//    double c_fitting = 0.180746072162041;
//    double temp = m_yaw;
//
//    m_yaw = -(a_fitting + b_fitting*temp + c_fitting*temp*temp);

    DebugT(1, 8,"观测值(Yaw):"<<std::left<<setw(10)<<m_yaw * 180. / M_PI)
    m_yaw = (m_yaw+bind_yaw); // Yaw角度融合，获得目标绝对Yaw角度
    DebugT(1, 9,"融合值(Yaw):"<<std::left<<setw(10)<<m_yaw * 180. / M_PI)

    // TODO: 需要增加对（陀螺方向与移动方向相同）移动陀螺的鉴别，否则会超调
    if(std::fabs(last_yaw - m_yaw) > 3. / 180. * M_PI){  // 两帧解算的Yaw超过5度则认为出现新目标，重置卡尔曼的状态值
        kalman.reset(m_yaw, t);
        last_yaw = m_yaw;
        DebugT(16, 7,"目标切换,卡尔曼重置")
        return false;
    }
    else{
        DebugT(16, 7,"正在预测          ")
    }

    last_yaw = m_yaw;
    Eigen::Matrix<double, 1, 1> z_k{m_yaw};
    _Kalman::Matrix_x1d state = kalman.update(z_k, t);                        // 更新卡尔曼滤波
    last_speed = state(1, 0);
    double c_yaw = state(0, 0);                                      // current yaw: yaw的滤波值，单位弧度
    double c_speed = state(1, 0) * m_pw.norm();                      // current speed: 角速度转线速度，单位m/s

    DebugT(22, 8,"滤波值(Yaw): " <<std::left<<setw(12)<< c_yaw * 180. / M_PI )

    // 限定目标速度范围
    if (abs(c_speed) < 0.005){
        c_speed = 0;
    }
    if (abs(c_speed) > 2.000){
        c_speed = c_speed/c_speed * 2; // 为了保证符号的一致性
    }

    // 线速度比例补偿
    DebugT(1, 10,"线速度: " <<std::left<<setw(12)<< c_speed << "m/s")
    double compensate_speed = c_speed * 1.1;
    c_speed = compensate_speed;
    DebugT(24, 10," 补偿结果: "<<std::left<<setw(12)<< compensate_speed << "m/s")

    double predict_time = m_pw.norm() / robot_speed_mps + shoot_delay + gimbal_delay;        // 预测时间=飞行时间+发射延迟（单位:s）
    DebugT(1, 11,"预测总时间(s):" << predict_time <<" = 飞行时间("<< m_pw.norm() / robot_speed_mps << ") + 发射延迟("<< shoot_delay<<") + 云台响应延迟("<<gimbal_delay<<")")
    double p_yaw = c_yaw + atan2(predict_time * c_speed, m_pw.norm());     // yaw的预测值，直线位移转为角度，单位弧度
    DebugT(22, 9,"预测值(Yaw): "<<std::left<<setw(12) << p_yaw*180.0/M_PI)

    double length = sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0));
    Eigen::Vector3d c_pw{length * cos(c_yaw), length * sin(c_yaw), m_pw(2, 0)};//反解位置(世界坐标系)
    Eigen::Vector3d p_pw{length * cos(p_yaw), length * sin(p_yaw), m_pw(2, 0)};

    double distance = p_pw.norm();                          // 目标距离（单位:m）
    double distance_xy = p_pw.topRows<2>().norm();
    double p_pitch = std::atan2(p_pw(2, 0), distance_xy);

    //  计算抛物线，先解二次方程
    double a = 9.8 * 9.8 * 0.25;
    double b = -robot_speed_mps * robot_speed_mps - distance * 9.8 * cos(M_PI_2 + p_pitch);
    double c = distance * distance;
    // 带入求根公式，解出t^2
    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
    double fly_time = sqrt(t_2);                                       // 子弹飞行时间（单位:s）
    // 解出抬枪高度，即子弹下坠高度(m)
    double height = 0.5 * 9.8 * t_2;
    float bs = robot_speed_mps;

    Eigen::Vector3d s_pw{p_pw(0, 0), p_pw(1, 0), p_pw(2, 0) + height}; // 抬枪后预测点

    /// 把世界坐标系中的点，重投影到图像中
    re_project_point(im2show, c_pw, R_IW, {0, 255, 0}); // 绿色点（当前目标点）
    re_project_point(im2show, p_pw, R_IW, {255, 0, 0}); // 蓝色点（目标预测点）
    re_project_point(im2show, s_pw, R_IW, {0, 0, 255}); // 红色点（枪口补偿点）
    for (int i = 0; i < 4; ++i){
        cv::circle(im2show, armor_box_points[i], 3, {0, 255, 0});} // 绿色点（灯条四顶点）
    cv::circle(im2show, {im2show.cols / 2, im2show.rows / 2}, 3, {0, 255 ,0}); // 图像中心点（绿色）

    Eigen::Vector3d s_pc = pw_to_pc(s_pw, R_IW);
    double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
    double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;
    // 绘制角度波形图
    double yaw_angle = s_yaw;
    double pitch_angle = s_pitch;
    double yaw_speed = c_speed;

////// TODO: 数据文件写入用于分析
//    ofstream outFile;
//    outFile.open(PROJECT_DIR"/distance_data.txt", ios::app);//保存的文件名
////    outFile<<to_string(m_yaw / M_PI * 180) + " " +to_string(c_yaw / M_PI * 180)+ " " + to_string(p_yaw / M_PI * 180) ;
//    outFile<<to_string(distance);
//    outFile<<"\n";
//    outFile.close();//关闭文件写入流

//	std::cout << " 程序耗时： "<<deltaTime_ms<< " m_yaw: " << yaw_angle<<"yaw角度: " << yaw_angle+mcu_data.curr_yaw/ M_PI * 180 << " pitch角度: " << -pitch_angle <<" "<< -mcu_data.curr_pitch/ M_PI * 180<< " yaw速度: " << yaw_speed << " 距离 " << distance << std::endl;
    DebugT(1, 14,"最终解算结果: Yaw:" <<yaw_angle+mcu_data.curr_yaw/ M_PI * 180<< " Pitch:"<<-pitch_angle<< " 目标距离(m):"<<std::left<<setw(10)<<distance);
    sendTarget(serial, yaw_angle+mcu_data.curr_yaw/ M_PI * 180, -pitch_angle, distance);
    return true;
}


// 不使用预测模式，仅发送相机坐标系下目标坐标【相对坐标】
bool PredictorKalman::none_predict(const cv::Point2f armor_box_points[4], int id, long long int t, cv::Mat &im2show) {
    double robot_speed_mps = 17.0; // TODO: 应当通过下位机知晓当前发射初速度（m/s）
    Eigen::Vector3d m_pc = pnp_get_pc(armor_box_points, id);  // point camera: 目标在相机坐标系下的坐标

    // 相机坐标系转换为云台坐标系
    double ptz_camera_x = 0.0;     //    PTZ_CAMERA_X - 相机与云台的 X 轴偏移(左负右正) 【云台与相机 相机作为参考点 单位m】
    double ptz_camera_y = 0.0;     //    PTZ_CAMERA_Y - 相机与云台的 Y 轴偏移(上负下正)
    double ptz_camera_z = -0.10;   //    PTZ_CAMERA_Z - 相机与云台的 Z 轴偏移(前正后负)
    static double theta = 0; // 定义相机旋转角度
    // 定义旋转矩阵
    static double r_data[] = {1, 0,           0,
                              0, cos(theta),  sin(theta),
                              0, -sin(theta), cos(theta)};
    // 定义平移矩阵
    static double t_data[] = {static_cast<double>(ptz_camera_x),
                              static_cast<double>(ptz_camera_y),
                              static_cast<double>(ptz_camera_z)};

    Eigen::Matrix<double, 3, 3> r_camera_ptz = Eigen::Matrix<double, 3, 3>(r_data);
    Eigen::Vector3d t_camera_ptz = Eigen::Vector3d(t_data);

    m_pc = r_camera_ptz * m_pc - t_camera_ptz; // 相机坐标系 转换为 云台坐标系

    double mc_yaw = std::atan2(m_pc(1,0), m_pc(0,0));    // yaw的测量值，单位弧度
    double length = sqrt(m_pc(0, 0) * m_pc(0, 0) + m_pc(1, 0) * m_pc(1, 0));
    Eigen::Vector3d c_pw{length * cos(mc_yaw), length * sin(mc_yaw), m_pc(2, 0)}; //反解位置(世界坐标系)
    double distance = c_pw.norm()/2;                          // 目标距离（单位:m）


    double distance_xy = c_pw.topRows<2>().norm();
    double p_pitch = std::atan2(c_pw(2, 0), distance_xy);

    // 弹道解算，计算抛物线，先解二次方程
    double a = 9.8 * 9.8 * 0.25;
    double b = -robot_speed_mps * robot_speed_mps - distance * 9.8 * cos(M_PI_2 + p_pitch);
    double c = distance * distance;
    // 带入求根公式，解出t^2
    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
    double fly_time = sqrt(t_2);                              // 子弹飞行时间（单位:s）
    // 解出抬枪高度，即子弹下坠高度(m)
    double height = 0.5 * 9.8 * t_2;

    Eigen::Vector3d s_pw{c_pw(0, 0), c_pw(1, 0), c_pw(2, 0) + height}; // 抬枪后解算点
    Eigen::Vector3d h_pw{c_pw(0, 0), c_pw(1, 0) - height, c_pw(2, 0)}; // 用于显示弹道解算点

    /// 把世界坐标系中的点，重投影到图像中
    Eigen::Vector3d pu = pc_to_pu(c_pw);
    cv::circle(im2show, {int(pu(0, 0)), int(pu(1, 0))}, 4, {0, 255 ,0}, 3);  // 绿色点（当前目标点）

    Eigen::Vector3d ps = pc_to_pu(h_pw);
    cv::circle(im2show, {int(ps(0, 0)), int(ps(1, 0))}, 4, {0, 0 ,255}, 3);  // 红色点（弹道解算点）

    for (int i = 0; i < 4; ++i){
        cv::circle(im2show, armor_box_points[i], 3, {0, 255, 0});} // 绿色点（灯条四顶点）
    cv::circle(im2show, {im2show.cols / 2, im2show.rows / 2}, 3, {0, 255 ,0}); // 图像中心点（绿色）

    double s_yaw = atan(s_pw(0, 0) / s_pw(2, 0)) / M_PI * 180.;
    double s_pitch = atan(h_pw(1, 0) / h_pw(2, 0)) / M_PI * 180.;
    // 绘制角度波形图
    double yaw_angle = s_yaw;
    double pitch_angle = s_pitch;

//// TODO: 数据文件写入用于分析
//    ofstream outFile;
//    outFile.open(PROJECT_DIR"/distance_filter_1_data.txt", ios::app);//保存的文件名
//    outFile<<to_string(cur_distance);
//    outFile<<"\n";
//    outFile.close();//关闭文件写入流

//	std::cout << "yaw角度: " << yaw_angle << " pitch角度: " << -pitch_angle  << " 距离 " << cur_distance << " cur_mcu_pitch: "<<mcu_data.curr_pitch*180/PI<<std::endl;
    sendTarget(serial, yaw_angle, -pitch_angle, distance);
    return true;

}


Eigen::Vector3d PredictorKalman::pnp_get_pc(const cv::Point2f p[4], int armor_number) {
    static const std::vector<cv::Point3d> pw_small = {  // 单位：m
            {0.066,  0.027,  0.},
            {0.066,  -0.027, 0.},
            {-0.066, -0.027, 0.},
            {-0.066, 0.027,  0.}

    };
    static const std::vector<cv::Point3d> pw_big = {    // 单位：m
            {0.1,  0.027,  0.},
            {0.1,  -0.027, 0.},
            {-0.1, -0.027, 0.},
            {-0.1, 0.027,  0.}

    };

    std::vector<cv::Point2d> pu(p, p + 4);
//    std::cout << "4 Points:"<<pu<< std::endl;

    cv::Mat rvec, tvec;
    // 当编号为【1、英雄  7、哨兵 8、基地】时，判定目标为大装甲板，其余情况均为小装甲板
    if (armor_number >= 8){ // 由于分类序号原因，这里仅获得实际的数字编号 （详细分类参阅 armor_finder.cpp）
        armor_number -= 7;}
    if (armor_number == 1 || armor_number == 7 || armor_number == 8) {
        cv::solvePnP(pw_big, pu, F_MAT, C_MAT, rvec, tvec);
    }
    else{
        cv::solvePnP(pw_small, pu, F_MAT, C_MAT, rvec, tvec);
    }
    Eigen::Vector3d pc;
    cv::cv2eigen(tvec, pc);
    return pc;
}

// 我服了这个。。。仅当预测模式使用
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
