/*******************************************************************************************/
/*     ______    ____    ____     ____          ______                                     */
/*    / ____/   /  _/   / __ \   / __ \        /_  __/ ___    ____ _   ____ ___            */
/*   / /        / /    / / / /  / /_/ / ______  / /   / _ \  / __ `/  / __ `__ \           */
/*  / /___    _/ /    / /_/ /  / ____/ /_____/ / /   /  __/ / /_/ /  / / / / / /           */
/*  \____/   /___/   /_____/  /_/             /_/    \___/  \__,_/  /_/ /_/ /_/            */
/*                                                                                         */
/*          ____            __              __  ___                  __                    */
/*         / __ \  ____    / /_   ____     /  |/  / ____ _   _____  / /_  ___    _____     */
/*        / /_/ / / __ \  / __ \ / __ \   / /|_/ / / __ `/  / ___/ / __/ / _ \  / ___/     */
/*       / _, _/ / /_/ / / /_/ // /_/ /  / /  / / / /_/ /  (__  ) / /_  /  __/ / /         */
/*      /_/ |_|  \____/ /_.___/ \____/  /_/  /_/  \__,_/  /____/  \__/  \___/ /_/          */
/*                                                                                         */
/*********************<< Designed by SX-CV-RobotTeam in 2022 >>*****************************/

// 本程序使用 640 * 480 作为输入图像尺寸

#include <iostream>
#include <thread>
#include <opencv2/core/core.hpp>
#include <serial.h>                    // 串口模块
#include <camera/video_wrapper.h>      // 从文件读取视频的包装器
#include <camera/wrapper_head.h>
#include <energy/energy.h>             // 能量机关部分
#include <armor_finder/armor_finder.h> // 自瞄装甲板部分
#include "armor_finder/kalman.h"                     // 卡尔曼基类
#include "armor_finder/predictor_kalman.h"           // 卡尔曼滤波与预测
#include <options.h>                   // 调试指令交互
#include <additions.h>                 // 拓展模块
#include "hikvision_camera.h"          // 海康相机设备操作

#define DO_NOT_CNT_TIME                // 模块记时器(调试用)
#include <log.h>                       // 日志模块


using namespace cv;
using namespace std;
using namespace camera;
using std::this_thread::sleep_for;

// 从下位机接收的数据
McuData mcu_data = {
        0,              // 当前云台yaw角
        0,              // 当前云台pitch角
        ARMOR_STATE,    // 当前状态，自瞄-大符-小符
        0,              // 云台角度标记位
        0,              // 是否为反陀螺模式
        ENEMY_RED,      // 敌方颜色
        0,              // 能量机关x轴补偿量
        0,              // 能量机关y轴补偿量
};

uint8_t last_state = ARMOR_STATE;// 上次状态，用于初始化
WrapperHead *video = nullptr;    // 云台摄像头视频源
Serial serial(115200);    // 串口对象
HikCamera MVS_cap;
PredictorKalman predictor;       // 初始化卡尔曼
cv::Mat ori_src;

// 自瞄主程序对象
ArmorFinder armor_finder(mcu_data.enemy_color, serial, PROJECT_DIR"/tools/para/", false);
// 能量机关主程序对象
Energy energy(serial, mcu_data.enemy_color);

double bind_yaw = mcu_data.curr_yaw;
double bind_pitch = mcu_data.curr_pitch;
extern double bind_yaw;
extern double bind_pitch;

int main(int argc, char *argv[]) {
    processOptions(argc, argv);             // 处理命令行参数
    thread receive(uartReceive, &serial);   // 开启串口接收线程

    // 如果不能从裁判系统读取颜色则手动设置目标颜色
    if (!recv_close) // 默认为【红色】装甲板为目标，更改目标装甲板颜色指令请查阅 options.cpp 功能列表
        mcu_data.enemy_color = ENEMY_RED;
    else
        mcu_data.enemy_color = ENEMY_BLUE;

    // 根据条件输入选择视频源 (1、海康相机  0、视频文件)
    int from_camera = 0; // 默认视频源
//    if (!run_with_camera) {
//        cout << "输入 1 使用海康相机, 输入 0 运行视频" << endl;
//        cin >> from_camera;
//    }

    // 打开视频源
    if (from_camera) {
        MVS_cap.Init(); // 初始化海康相机
    } else {
        video = new VideoWrapper(PROJECT_DIR"/videoTest/armor_red.mp4"); // 视频文件路径
        if (video->init()) {
            LOGM("video_source initialization successfully.");
        } else {
            LOGW("video_source unavailable!");
        }
    }

    cout << "\x1B[2J\x1B[H"; // 程序开始前清空终端区显示
    while (true) {
        bind_yaw = mcu_data.curr_yaw; // 获取程序开始时下位机云台姿态数据（yaw），用于图像-时间戳 对齐
        bind_pitch = mcu_data.curr_pitch; // 获取程序开始时下位机云台姿态数据（yaw），用于图像-时间戳 对齐

        // 从相机捕获一帧图像
        if (from_camera) {
            MVS_cap.ReadImg(ori_src);
            if (ori_src.empty())          // 海康相机初始化时开启线程需要一定时间,防止空图
                continue;
        } else {
            video->read(ori_src);
        }
//        flip(ori_src, ori_src, -1); // 图像翻转（视实际相机安装情况）# TODO:记得改回来！

        // 测量程序耗时并限制程序最大帧率
        static float nowTime_ms = 0.0f, deltaTime_ms = 0.0f, lastTime_ms = 0.0f, sleep_time = 0.0f, vision_max_hz = 100.0f;
        nowTime_ms = cv::getTickCount() / cv::getTickFrequency() * 1000;	//ms
        deltaTime_ms = (float)(nowTime_ms - lastTime_ms);				    //ms
        lastTime_ms = nowTime_ms;
        DebugT(30, 1,"帧率:" <<std::left<<setw(5)<< 1000.0/deltaTime_ms);
        DebugT(40, 1," 程序耗时:"<<std::left<<setw(11)<<deltaTime_ms<<"ms");
        if (deltaTime_ms < (1000.0/vision_max_hz)) {
            sleep_time = ((1000.0/vision_max_hz)-deltaTime_ms)*1000.0;
//            usleep(sleep_time); // ms
        }

        //  char curr_state = mcu_data.state; // # 下位机控制模式
        char curr_state = ARMOR_STATE; // 手动设定模式
        CNT_TIME("Total", {
            if (curr_state != ARMOR_STATE) {  // 能量机关模式
                DebugT(1, 1,"目标颜色:" << (int)mcu_data.enemy_color << " 图像源:"<< from_camera<< " 当前模式: 能量机关" ); // 右边
                DebugT(1, 2,"云台姿态 Yaw:" << bind_yaw*180.0/M_PI << "  "<<"Pitch:"<<bind_pitch*180.0/M_PI); // 右边

                if (last_state == ARMOR_STATE) {//若上一帧不是大能量机关模式，即刚往完成切换，则需要初始化
                    destroyAllWindows();

                    if (curr_state == BIG_ENERGY_STATE) {            // 大能量机关模式
                        energy.is_small = false;
                        energy.is_big = true;
                        LOGM(STR_CTR(WORD_BLUE, "开始大能量机关模式!"));
                    } else if (curr_state == SMALL_ENERGY_STATE) {   // 小能量机关模式
                        energy.is_small = true;
                        energy.is_big = false;
                        LOGM(STR_CTR(WORD_GREEN, "开始小能量机关模式"));
                    }
                    energy.setEnergyInit();
                }
                extract(ori_src);  // 画幅 resize
                if (save_video) saveVideos(ori_src); // 保存视频
                showOrigin(ori_src);// 显示原始图像
                energy.run(ori_src);
            }

            else {                                         // 自瞄模式
                if (last_state != ARMOR_STATE) {
                    LOGM(STR_CTR(WORD_RED, "开始自瞄模式"));
                    destroyAllWindows();
                };
                CNT_TIME("something whatever", {
                        extract(ori_src);  // 画幅 resize
                        if (save_video) saveVideos(ori_src); // 保存视频
                        if (show_origin) showOrigin(ori_src);// 显示原始图像
                });
                CNT_TIME(STR_CTR(WORD_CYAN, "Armor Time"), {
                        armor_finder.run(ori_src);
                });
            }

            last_state = curr_state; // 更新上一帧状态
            cv::waitKey(1);

        });
    }
return 0;
}