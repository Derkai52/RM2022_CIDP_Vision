#include <armor_finder/armor_finder.h>
#include <config/setconfig.h>
#include <options.h>
#include <log.h>
#include <math.h>
#include "armor_finder/predictor_kalman.h"
#include "armor_finder/kalman.h"

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
//   yaw -= 0.3;  //消除摩擦轮不同步导致的弹道偏移
    double pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
    pitch = pitch - atan(400000/DISTANCE_HEIGHT)-2;  // 消除相机与枪口的误差,相机在枪口线上0.04m,数值对应0.04*10000  == 40000  //1.4
    //cout<<"转换前的pitch:"<<pitch << "转换后的pitch:"<< pitch1 << endl;
    double dist = DISTANCE_HEIGHT / rect.height;

    return vector<double>{yaw, pitch, dist};
}

bool ArmorFinder::predict_run(int ave_cal_time) {
//    if (target_box.light_blobs.size()==2)
    cv::Point lt = cv::Point(target_box.light_blobs[0].rect.center.x, target_box.light_blobs[0].rect.center.y - target_box.light_blobs[0].length*0.5);
    cv::Point lb = cv::Point(target_box.light_blobs[0].rect.center.x, target_box.light_blobs[0].rect.center.y + target_box.light_blobs[0].length*0.5);
    cv::Point rb = cv::Point(target_box.light_blobs[1].rect.center.x, target_box.light_blobs[1].rect.center.y + target_box.light_blobs[1].length*0.5);
    cv::Point rt = cv::Point(target_box.light_blobs[1].rect.center.x, target_box.light_blobs[1].rect.center.y - target_box.light_blobs[1].length*0.5);
//        std::vector<cv::Point2f> strVec{light_lt, light_rt, light_rb, light_lb};

//    auto rect = target_box.rect;                                   // 获取当前目标装甲板
//    cv::Point2f lt(rect.x, rect.y);                                // 左上
//    cv::Point2f lb(rect.x, rect.y+rect.height);                // 左下
//    cv::Point2f rb(rect.x+rect.width, rect.y+rect.height); // 右下
//    cv::Point2f rt(rect.x+rect.width, rect.y);                 // 右上
//    cv::Point2f armor_center(rect.x+rect.width*0.5, rect.y+rect.height*0.5);  // 装甲板中心点

    // 装甲板的四点排序： 左上 -> 左下 -> 右下 -> 右上
    const cv::Point2f armor_box_points[4]{lt, lb, rb, rt};

    cv::Mat im2show = ori_src;
    bool ok = predictor.predict(armor_box_points, ave_cal_time , im2show);
    if(!ok) {
        cout << "预测失败,按未预测输出" << endl;
        return false;
    }
    return true;
}


int ave_cal_time = 10; // 程序耗时（采样间隔1000ms）
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
//    armor_predictor = true;
    if(!armor_predictor){ // 不使用预测模式（可用于测试电控）
        auto result = ArmorFinder::none_predict_run();
        yaw = result[0];
        pitch = result[1];
        dist = result[2];
        cout << "识别到装甲板: 帧率:" << 1000/ave_cal_time << ", ( Yaw轴偏移量：" << yaw << ", Pitch轴偏移量：" << -pitch << ", 距离：" << dist << " )" << endl;
        return sendTarget(serial, yaw, -pitch, dist, shoot_delay);
    }
    else{
        ArmorFinder::predict_run(ave_cal_time);
    }
}
