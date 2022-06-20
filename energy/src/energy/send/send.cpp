#include "energy/energy.h"
#include <iostream>
#include "log.h"
#include "config/setconfig.h"
#include "fstream"
#include "string"

using namespace std;

#define MINMAX(value, min, max) value = ((value) < (min)) ? (min) : ((value) > (max) ? (max) : (value))

//----------------------------------------------------------------------------------------------------------------------
// 此函数用于发送小能量机关数据
// ---------------------------------------------------------------------------------------------------------------------
void Energy::sendSmallEnergy() {
    sum_yaw += yaw_rotation;
    sum_pitch += pitch_rotation;
    float yaw_I_component = YAW_AIM_KI * sum_yaw;
    float pitch_I_component = PITCH_AIM_KI * sum_pitch;
    MINMAX(yaw_I_component, -2, 2);
    MINMAX(pitch_I_component, -2, 2);

    double tmp_yaw = yaw_rotation;
    double tmp_pitch = pitch_rotation;
    yaw_rotation = YAW_AIM_KP * yaw_rotation + YAW_AIM_KI * sum_yaw +
                   YAW_AIM_KD * (yaw_rotation - last_yaw);
    pitch_rotation = PITCH_AIM_KP * pitch_rotation + PITCH_AIM_KI * sum_pitch +
                     PITCH_AIM_KD * (pitch_rotation - last_pitch);

    last_yaw = tmp_yaw;
    last_pitch = tmp_pitch;

    if (change_target) {
        sendTarget(serial, -yaw_rotation, pitch_rotation, 3, 0);//表示目标切换
    } else if (is_guessing) {
        sendTarget(serial, -yaw_rotation, pitch_rotation, 4, 0);//表示猜测模式
    } else {
        sendTarget(serial, -yaw_rotation, pitch_rotation, shoot, 0);//跟随或发弹
    }

}


//----------------------------------------------------------------------------------------------------------------------
// 此函数用于发送大能量机关数据
// ---------------------------------------------------------------------------------------------------------------------
void Energy::sendBigEnergy() {
    double robot_speed_mps = 28; // TODO: 应当通过下位机知晓当前发射初速度（m/s）
    double p_pitch = std::atan2(predict_worldPTZ.y, target_distance);

    // 弹道解算，计算抛物线，先解二次方程
    double a = 9.8 * 9.8 * 0.25;
    double b = -robot_speed_mps * robot_speed_mps - target_distance * 9.8 * cos(M_PI_2 + p_pitch);
    double c = target_distance * target_distance;
    // 带入求根公式，解出t^2
    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
    double fly_time = sqrt(t_2);                              // 子弹飞行时间（单位:s）
    // 解出抬枪高度，即子弹下坠高度(m)
    double height = 0.5 * 9.8 * t_2;

    p_pitch = std::atan2(predict_worldPTZ.y-height, target_distance);
    double m_yaw = std::atan2(predict_worldPTZ.x, predict_worldPTZ.z);   // yaw的测量值，单位弧度
    DebugT(1, 14,"最终解算结果:"<<" Yaw:"<<(m_yaw*180.0/M_PI) <<" pitch: "<< -(p_pitch*180.0/M_PI));
    DebugT(40, 2,"射速: " << robot_speed_mps << " m/s")

//    //    ////     TODO: 数据文件写入用于分析
//    ofstream outFile;
//    outFile.open(PROJECT_DIR"/bind_result_yaw.txt", ios::app);//保存的文件名
//    outFile<<to_string(m_yaw*180.0/M_PI)<<" "<<to_string(mcu_data.curr_yaw*180.0/M_PI) ;
////    outFile<<to_string(distance);
//    outFile<<"\n";
//    outFile.close();//关闭文件写入流
//
//    cout <<""<<predict_worldPTZ << endl;

    sendTarget(serial, (m_yaw*180.0/M_PI), -(p_pitch*180.0/M_PI), 3, 0);//表示目标切换
}



//----------------------------------------------------------------------------------------------------------------------
// 此函数用于发送数据给主控板
// ---------------------------------------------------------------------------------------------------------------------
void Energy::sendTarget(Serial &serial, float x, float y, float z, uint16_t u) {
    short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

#ifdef WITH_COUNT_FPS
    static auto last_time = time(nullptr);
    static int fps = 0;
    time_t t = time(nullptr);
    if (last_time != t) {
        last_time = t;
//        cout << "Energy: fps:" << fps << ", (" << x << "," << y << "," << z << "," << u << ")" << endl;
        curr_fps = fps;
        fps = 0;
    }
    fps += 1;
#endif

    x_tmp = static_cast<short>(x * (32768 - 1) / 100);
    y_tmp = static_cast<short>(y * (32768 - 1) / 100);
    z_tmp = static_cast<short>(z * (32768 - 1) / 100);
    buff[0] = 's';
    buff[1] = static_cast<char>((x_tmp >> 8) & 0xFF);
    buff[2] = static_cast<char>((x_tmp >> 0) & 0xFF);
    buff[3] = static_cast<char>((y_tmp >> 8) & 0xFF);
    buff[4] = static_cast<char>((y_tmp >> 0) & 0xFF);
    buff[5] = static_cast<char>((z_tmp >> 8) & 0xFF);
    buff[6] = static_cast<char>((z_tmp >> 0) & 0xFF);
    buff[7] = static_cast<char>((u >> 8) & 0xFF);
    buff[8] = static_cast<char>((u >> 0) & 0xFF);
    buff[9] = 'e';
    serial.WriteData(buff, sizeof(buff));
    send_cnt += 1;
//    LOGM(STR_CTR(WORD_LIGHT_PURPLE, "send"));
}
