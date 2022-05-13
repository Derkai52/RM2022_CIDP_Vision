#include <armor_finder/armor_finder.h>
#include <config/setconfig.h>
#include <options.h>
#include <log.h>
#include <math.h>
#include "armor_finder/predictor_kalman.h"
#include "armor_finder/kalman.h"
#include <chrono>

extern PredictorKalman predictor;
extern cv::Mat ori_src;

//static bool sendTarget(Serial &serial, double x, double y, double z, uint16_t shoot_delay) {
//    static short x_tmp, y_tmp, z_tmp;
//    uint8_t buff[10];
//    x_tmp = static_cast<short>(x * (32768 - 1) / 100);
//    y_tmp = static_cast<short>(y * (32768 - 1) / 100);
//    z_tmp = static_cast<short>(z * (32768 - 1) / 1000);
//
//    buff[0] = 's';
//    buff[1] = static_cast<char>((x_tmp >> 8) & 0xFF);
//    buff[2] = static_cast<char>((x_tmp >> 0) & 0xFF);
//    buff[3] = static_cast<char>((y_tmp >> 8) & 0xFF);
//    buff[4] = static_cast<char>((y_tmp >> 0) & 0xFF);
//    buff[5] = static_cast<char>((z_tmp >> 8) & 0xFF);
//    buff[6] = static_cast<char>((z_tmp >> 0) & 0xFF);
//    buff[7] = static_cast<char>((shoot_delay >> 8) & 0xFF);
//    buff[8] = static_cast<char>((shoot_delay >> 0) & 0xFF);
//    buff[9] = 'e';
////    if(buff[7]<<8 | buff[8])
////        cout << (buff[7]<<8 | buff[8]) << endl;
//    return serial.WriteData(buff, sizeof(buff));
//}

//bool ArmorFinder::old_run(){ // 三角测距法【已弃用】
//    auto rect = target_box.rect;
//    double dx = rect.x + rect.width / 2 - IMAGE_CENTER_X;
//    double dy = rect.y + rect.height / 2 - IMAGE_CENTER_Y;
//    double yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
//    double pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
//    double dist = DISTANCE_HEIGHT / rect.height;
//
//    return sendTarget(yaw, -pitch, dist, 0);
//}

bool ArmorFinder::target_solving() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    long long int sys_time_stamp = tv.tv_sec * 1000 + tv.tv_usec / 1000; // 获取当前系统时间戳

    target_box.getFourPoint(target_box); // 获取目标装甲板灯条四点
    cv::Point lt = cv::Point(target_box.four_point[0]); // 左上
    cv::Point lb = cv::Point(target_box.four_point[1]); // 左下
    cv::Point rb = cv::Point(target_box.four_point[2]); // 右下
    cv::Point rt = cv::Point(target_box.four_point[3]); // 右上

    // 装甲板的四点排序： 左上 -> 左下 -> 右下 -> 右上
    const cv::Point2f armor_box_points[4]{lt, lb, rb, rt};

    cv::Mat im2show = ori_src;
    if(armor_predictor){ // 使用预测器
        bool ok = predictor.predict(armor_box_points, target_box.id, sys_time_stamp, im2show);
        if(!ok) {
            cout << "预测失败,按未预测输出" << endl;
            return false;
        }
    }else{ // 不使用预测器
        bool ok = predictor.none_predict(armor_box_points, target_box.id, sys_time_stamp, im2show);
    }
    return true;
}


bool ArmorFinder::sendBoxPosition(uint16_t shoot_delay) {
    if (target_box.rect == cv::Rect2d()) return false;
    if (shoot_delay) {
        LOGM(STR_CTR(WORD_BLUE, "next box %dms"), shoot_delay);
    }

    #ifdef WITH_COUNT_FPS // 【注】FPS表示的是当前秒给下位机发送的频率
        static time_t last_time = time(nullptr);
        static int fps;
        time_t t = time(nullptr);

    if (last_time != t) {
        last_time = t;
        cout << "识别到装甲板: 发送帧率:" << fps << endl;
        fps = 0;
    }
    fps += 1;
    #endif

    ArmorFinder::target_solving(); // 目标解算 & 预测

//    auto rect = target_box.rect;
//    double yaw;   // 发送给电控yaw角度
//    double pitch; // 发送给电控pitch角度
//    double dist;  // 发送给电控dist目标距离
//
//    if(!armor_predictor){ // 不使用预测模式（可用于测试电控）// TODO：旧解算器（三角测距法）不再使用
//        auto result = ArmorFinder::none_predict_run();
//        yaw = result[0];
//        pitch = result[1];
//        dist = result[2];
//        cout << "识别到装甲板: 帧率:" << 1000/ave_cal_time << ", ( Yaw轴偏移量：" << yaw << ", Pitch轴偏移量：" << -pitch << ", 距离：" << dist << " )" << endl;
//        return sendTarget(serial, yaw, -pitch, dist, shoot_delay);
//    }
//    else{
//    }
}
