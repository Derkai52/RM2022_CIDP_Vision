#include "energy/energy.h"
#include "log.h"

using namespace cv;
using std::cout;
using std::endl;
using std::vector;


//----------------------------------------------------------------------------------------------------------------------
// 此函数对能量机关成员变量进行初始化
// ---------------------------------------------------------------------------------------------------------------------
void Energy::initEnergy() {
    is_guessing = false;
    is_predicting = true;
    energy_mode_init = true;
    energy_rotation_init = true;
    start_guess = false;
    change_target = false;

    curr_fps = 0;
    send_cnt = 0;
    fans_cnt = 0;
    last_fans_cnt = 0;
    guess_devide = 0;
    energy_rotation_direction = ANTICLOCKWISE;
    clockwise_rotation_init_cnt = 0;
    anticlockwise_rotation_init_cnt = 0;
    last_mode = -1;//既不是大符也不是小符
    manual_delta_x = 0;
    manual_delta_y = 0;
    extra_delta_y = 0;
    extra_delta_x = 0;

    target_polar_angle = -1000;
    last_target_polar_angle_judge_change = -1000;
    last_target_polar_angle_judge_rotation = -1000;
    guess_polar_angle = -1000;
    last_base_angle = -1000;
    predict_rad = 0;
    predict_rad_norm = 25;
    attack_distance = ATTACK_DISTANCE;
    center_delta_yaw = 1000;
    center_delta_pitch = 1000;
    yaw_rotation = 0;
    pitch_rotation = 0;
    shoot = 0;
    last_yaw = 0;
    last_pitch = 0;
    sum_yaw = 0;
    sum_pitch = 0;

    circle_center_point = Point(0, 0);
    target_point = Point(0, 0);
    guess_point = Point(0, 0);
    predict_point = Point(0, 0);

    fans.clear();
    armors.clear();
    flow_strip_fans.clear();
    target_armors.clear();
    flow_strips.clear();
    all_target_armor_centers.clear();
    while (!recent_target_armor_centers.empty())recent_target_armor_centers.pop();

}


//----------------------------------------------------------------------------------------------------------------------
// 此函数对能量机关参数进行初始化
// ---------------------------------------------------------------------------------------------------------------------
void Energy::initEnergyPartParam() {
    energy_part_param_.RED_GRAY_THRESH = 180;//game 180  //红方二值化阈值
    energy_part_param_.BLUE_GRAY_THRESH = 20;//game 100  //蓝方二值化阈值
    energy_part_param_.SPLIT_GRAY_THRESH = 180; // 180   //通道分离二值化阈值

    energy_part_param_.FAN_CONTOUR_AREA_MAX = 5000; //扇叶面积最大值
    energy_part_param_.FAN_CONTOUR_AREA_MIN = 1500;
    energy_part_param_.FAN_CONTOUR_LENGTH_MIN = 45;  //扇叶长边长度最小值
    energy_part_param_.FAN_CONTOUR_LENGTH_MAX = 100;
    energy_part_param_.FAN_CONTOUR_WIDTH_MIN = 10;  //扇叶宽边长度最小值
    energy_part_param_.FAN_CONTOUR_WIDTH_MAX = 52;
    energy_part_param_.FAN_CONTOUR_HW_RATIO_MAX = 3.5; //扇叶长宽比最大值
    energy_part_param_.FAN_CONTOUR_HW_RATIO_MIN = 1.2;
    energy_part_param_.FAN_CONTOUR_AREA_RATIO_MIN = 0.6; //装甲板轮廓占旋转矩形面积比最小值

    energy_part_param_.ARMOR_CONTOUR_AREA_MAX = 900; //装甲板面积最大值 500
    energy_part_param_.ARMOR_CONTOUR_AREA_MIN = 180; //装甲板面积最小值
    energy_part_param_.ARMOR_CONTOUR_LENGTH_MIN = 10; //装甲板长边长度最小值
    energy_part_param_.ARMOR_CONTOUR_LENGTH_MAX = 50;
    energy_part_param_.ARMOR_CONTOUR_WIDTH_MIN = 0;  //装甲板宽边长度最小值
    energy_part_param_.ARMOR_CONTOUR_WIDTH_MAX = 30;
    energy_part_param_.ARMOR_CONTOUR_HW_RATIO_MAX = 3; //装甲板长宽比最大值
    energy_part_param_.ARMOR_CONTOUR_HW_RATIO_MIN = 1;

    energy_part_param_.CENTER_R_CONTOUR_AREA_MAX = 200; //风车中心R面积最大值
    energy_part_param_.CENTER_R_CONTOUR_AREA_MIN = 40;
    energy_part_param_.CENTER_R_CONTOUR_LENGTH_MIN = 6; //风车中心R长边长度最小值
    energy_part_param_.CENTER_R_CONTOUR_LENGTH_MAX = 20;
    energy_part_param_.CENTER_R_CONTOUR_WIDTH_MIN = 6; //风车中心R长边长度最小值
    energy_part_param_.CENTER_R_CONTOUR_WIDTH_MAX = 20;
    energy_part_param_.CENTER_R_CONTOUR_HW_RATIO_MAX = 2; //风车中心R长宽比最大值
    energy_part_param_.CENTER_R_CONTOUR_HW_RATIO_MIN = 1;
    energy_part_param_.CENTER_R_CONTOUR_AREA_RATIO_MIN = 0.6; //装甲板轮廓占旋转矩形面积比最小值
    energy_part_param_.CENTER_R_CONTOUR_INTERSETION_AREA_MIN = 10; //中心R占ROI区的面积最小值

    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_AREA_MAX = 3000; //流动条扇叶（待击打）面积最大值 2000
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_AREA_MIN = 500; // 500
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN = 100; //流动条扇叶（待击打）长边长度最小值 60
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX = 140; // 100
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN = 20; //流动条扇叶（待击打）宽边长度最小值 //20
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX = 70; // 52
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX = 2.8; //流动条扇叶（待击打）长宽比最大值 //2.8
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN = 1.2;
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX = 0.58; //流动条扇叶轮廓占旋转矩形面积比最小值
    energy_part_param_.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN = 0.30; //0.34

    energy_part_param_.FLOW_STRIP_CONTOUR_AREA_MAX = 5000; //流动条（待击打）面积最大值
    energy_part_param_.FLOW_STRIP_CONTOUR_AREA_MIN = 100;
    energy_part_param_.FLOW_STRIP_CONTOUR_LENGTH_MIN = 32; //流动条（待击打）长边长度最小值
    energy_part_param_.FLOW_STRIP_CONTOUR_LENGTH_MAX = 80; //55
    energy_part_param_.FLOW_STRIP_CONTOUR_WIDTH_MIN = 4; //流动条（待击打）宽边长度最小值
    energy_part_param_.FLOW_STRIP_CONTOUR_WIDTH_MAX = 20;
    energy_part_param_.FLOW_STRIP_CONTOUR_HW_RATIO_MAX = 7; //流动条（待击打）长宽比最大值
    energy_part_param_.FLOW_STRIP_CONTOUR_HW_RATIO_MIN = 3;
    energy_part_param_.FLOW_STRIP_CONTOUR_AREA_RATIO_MIN = 0.6; //流动条占旋转矩形面积比最小值
    energy_part_param_.FLOW_STRIP_CONTOUR_INTERSETION_AREA_MIN = 100;

    energy_part_param_.TWIN_ANGEL_MAX = 10; //两个理论上相等的角度在计算时具有的可能最大差值
    energy_part_param_.TARGET_INTERSETION_CONTOUR_AREA_MIN = 40; //扇叶与装甲板匹配时的最小重合面积

    energy_part_param_.TWIN_POINT_MAX = 20; //两个点相同时距离最大值

    energy_part_param_.STRIP_ARMOR_DISTANCE_MIN = 28; //流动条中心和目标装甲板中心距离最小值
    energy_part_param_.STRIP_ARMOR_DISTANCE_MAX = 52;
}


//----------------------------------------------------------------------------------------------------------------------
// 此函数对能量机关旋转方向进行初始化
// ---------------------------------------------------------------------------------------------------------------------
void Energy::initRotation() {
    if (target_polar_angle >= -180 && last_target_polar_angle_judge_rotation >= -180
        && fabs(target_polar_angle - last_target_polar_angle_judge_rotation) < 30) {
        //target_polar_angle和last_target_polar_angle_judge_rotation的初值均为1000，大于-180表示刚开始几帧不要
        //若两者比较接近，则说明没有切换目标，因此可以用于顺逆时针的判断
        if (target_polar_angle < last_target_polar_angle_judge_rotation) clockwise_rotation_init_cnt++;
        else if (target_polar_angle > last_target_polar_angle_judge_rotation) anticlockwise_rotation_init_cnt++;
    }
    //由于刚开始圆心判断不准，角度变化可能计算有误，因此需要在角度正向或逆向变化足够大时才可确定是否为顺逆时针
    if (clockwise_rotation_init_cnt == 15) {
        energy_rotation_direction = CLOCKWISE;//顺时针变化30次，确定为顺时针
        cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    } else if (anticlockwise_rotation_init_cnt == 15) {
        energy_rotation_direction = ANTICLOCKWISE;//逆时针变化30次，确定为顺时针
        cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    }
    last_target_polar_angle_judge_rotation = target_polar_angle;
}
