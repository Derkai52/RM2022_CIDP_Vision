#include <armor_finder/armor_finder.h>
#include <options.h>
#include <log.h>
#include "armor_finder/predictor_kalman.h"
#include "armor_finder/kalman.h"
#include <chrono>

extern PredictorKalman predictor;
extern cv::Mat ori_src;

bool ArmorFinder::target_solving() {
    if (target_box.id == BLUE2 || target_box.id == RED2){    // 识别到工程机器人时不处理，防止骗弹
        return true;
    }
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
//            cout << "预测失败,按未预测输出" << endl;
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

    // 测量程序耗时
    static float nowTime_ms = 0.0f, deltaTime_ms = 0.0f, lastTime_ms = 0.0f;
    nowTime_ms = cv::getTickCount() / cv::getTickFrequency() * 1000;	//ms
    deltaTime_ms = (float)(nowTime_ms - lastTime_ms);				    //ms
    lastTime_ms = nowTime_ms;
//    DebugT(30, 1,"帧率:" << 1000.0/deltaTime_ms); // 右边

    // 目标解算 & 预测
    ArmorFinder::target_solving();
}
