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
#include <energy/energy.h>             // 能量机关部分【TODO：将在未来版本更新】
#include <armor_finder/armor_finder.h> // 自瞄装甲板部分
#include <options.h>                   // 调试指令交互
#include <additions.h>                 // 拓展模块
#include "hikvision_camera.h"          // 海康相机设备操作

#define DO_NOT_CNT_TIME                // 模块记时器(调试用)
#include <log.h>                       // 日志模块


using namespace cv;
using namespace std;
using namespace camera;

McuData mcu_data = {    // 单片机端回传结构体
        0,              // 当前云台yaw角
        0,              // 当前云台pitch角
        ARMOR_STATE,    // 当前状态，自瞄-大符-小符
        0,              // 云台角度标记位
        0,              // 是否为反陀螺模式
        ENEMY_RED,      // 敌方颜色
        0,              // 能量机关x轴补偿量
        0,              // 能量机关y轴补偿量
};

WrapperHead *video = nullptr;    // 云台摄像头视频源
Serial serial(115200);    // 串口对象
HikCamera MVS_cap;

// 自瞄主程序对象
ArmorFinder armor_finder(mcu_data.enemy_color, serial, PROJECT_DIR"/tools/para/", mcu_data.anti_top);
// 能量机关主程序对象
Energy energy(serial, mcu_data.enemy_color);


int main(int argc, char *argv[]) {
    processOptions(argc, argv);             // 处理命令行参数
    thread receive(uartReceive, &serial);   // 开启串口接收线程

    // 如果不能从裁判系统读取颜色则手动设置目标颜色
    if (recv_close) // 默认为【红色】装甲板为目标，更改目标装甲板颜色指令请查阅 options.cpp 功能列表
        mcu_data.enemy_color = ENEMY_RED;
    else
        mcu_data.enemy_color = ENEMY_BLUE;

    // 根据条件输入选择视频源 (1、海康相机  2、视频文件)
    int from_camera = 1; // 默认视频源
    if (!run_with_camera) {
        cout << "Input 1 for camera, 0 for video files" << endl;
        cin >> from_camera;
    }


        // 打开视频源
        if (from_camera) {
            MVS_cap.Init(); // 初始化海康相机
        } else {
            video = new VideoWrapper(PROJECT_DIR"/armor_red.mp4"); // 视频文件路径
            if (video->init()) {
                LOGM("video_source initialization successfully.");
            } else {
                LOGW("video_source unavailable!");
            }
        }

    while (true) {
        // 从相机捕获一帧图像
        Mat ori_src;
        if (from_camera) {
            MVS_cap.ReadImg(ori_src);
            if (ori_src.empty())          // 海康相机初始化时开启线程需要一定时间,防止空图
                continue;}
        else {
            video->read(ori_src);}

        CNT_TIME("Total", {
            extract(ori_src);          //图像预处理，resize至640×480的大小
            if (save_video)            // 视频录制
                saveVideos(ori_src);

            armor_finder.run(ori_src); // 自动瞄准主程序入口
            cv::waitKey(1);
        });
    }
    cout << "AutoAiming is Over!!!" << endl;
    return 0;
}