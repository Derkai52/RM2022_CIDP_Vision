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

static bool sendTarget(Serial &serial, double x, double y, double z, uint16_t shoot_delay) {
    static short x_tmp, y_tmp, z_tmp;
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

vector<double> ArmorFinder::none_predict_run(){
    auto rect = target_box.rect;
    double dx = rect.x + rect.width / 2 - IMAGE_CENTER_X;
    double dy = rect.y + rect.height / 2 - IMAGE_CENTER_Y;
    double yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
    double pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
    double dist = DISTANCE_HEIGHT / rect.height;

    return vector<double>{yaw, pitch, dist};
}

bool ArmorFinder::predict_run() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    long long int sys_time_stamp = tv.tv_sec * 1000 + tv.tv_usec / 1000; // 获取当前系统时间戳

    target_box.getFourPoint(target_box); // 获取目标装甲板灯条四点
    cv::Point lt = cv::Point(target_box.four_point[0]);
    cv::Point lb = cv::Point(target_box.four_point[1]);
    cv::Point rb = cv::Point(target_box.four_point[2]);
    cv::Point rt = cv::Point(target_box.four_point[3]);

    // 装甲板的四点排序： 左上 -> 左下 -> 右下 -> 右上
    const cv::Point2f armor_box_points[4]{lt, lb, rb, rt};

    cv::Mat im2show = ori_src;
    bool ok = predictor.predict(armor_box_points, target_box.id, sys_time_stamp, im2show);
    if(!ok) {
        cout << "预测失败,按未预测输出" << endl;
        return false;
    }
    return true;
}


int ave_cal_time = 10; // 打印发送帧率 （采样间隔1000ms）
bool ArmorFinder::sendBoxPosition(uint16_t shoot_delay) {
    bool ok = false;
    if (target_box.rect == cv::Rect2d()) return false;
    if (shoot_delay) {
        LOGM(STR_CTR(WORD_BLUE, "next box %dms"), shoot_delay);
    }

    #ifdef WITH_COUNT_FPS // FPS表示的是每秒给下位机发送的帧率
        static time_t last_time = time(nullptr);
        static int fps;
        time_t t = time(nullptr);

    if (last_time != t) {
            last_time = t;
            cout << "识别到装甲板: 发送帧率:" << fps << endl;
            ave_cal_time = (int)1000/fps;

        fps = 0;
        }
        fps += 1;
    #endif

    auto rect = target_box.rect;
    double yaw;   // 发送给电控yaw角度
    double pitch; // 发送给电控pitch角度
    double dist;  // 发送给电控dist目标距离
    armor_predictor = true;
    if(!armor_predictor){ // 不使用预测模式（可用于测试电控）
        auto result = ArmorFinder::none_predict_run();
        yaw = result[0];
        pitch = result[1];
        dist = result[2];
        cout << "识别到装甲板: 帧率:" << 1000/ave_cal_time << ", ( Yaw轴偏移量：" << yaw << ", Pitch轴偏移量：" << -pitch << ", 距离：" << dist << " )" << endl;
        return sendTarget(serial, yaw, -pitch, dist, shoot_delay);
    }
    else{
        ArmorFinder::predict_run();
    }
}
