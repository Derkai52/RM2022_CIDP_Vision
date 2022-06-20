#include "energy/energy.h"
#include "energy/constant.h"
#include <angleFactory.h>
#include "log.h"


extern double bind_yaw;
extern double bind_pitch;


using namespace cv;
using std::cout;
using std::endl;
using std::vector;


//----------------------------------------------------------------------------------------------------------------------
// 此函数获取预测点坐标
// ---------------------------------------------------------------------------------------------------------------------
const string serialNumber = "calibCameraData"; // 使用的相机参数文件（内参、畸变矩阵）
AngleFactory *angleFactory_ = new AngleFactory(serialNumber); // Warning
TX2CmdStruct _tx2Command;
ArmorCoordinateSolver::CoordinateStruct _coordinateStruct;
void Energy::getPredictPoint(cv::Mat src, cv::Point target_point) {
    if (is_big) {

        angleFactory_->setBuffSolverStruct((bool)!change_target, true, circle_center_point, target_armor, NO_CHANGE, 28.0);
        angleFactory_->calculateFinalResult(target_armor, ARMOR_BIG, TX2_DISTINGUISH_BIG_BUFF, INFANTRY, bind_pitch, bind_yaw, 28.0,
                                            false,0);
        float result_x = angleFactory_->get_result().actualCoordinate.x/1000;
        float result_y = angleFactory_->get_result().actualCoordinate.y/1000;
        float result_z = angleFactory_->get_result().actualCoordinate.z/1000;
        predict_worldPTZ = {result_x, result_y, result_z};
        target_distance = sqrt(pow(predict_worldPTZ.x,2)+pow(predict_worldPTZ.z, 2)); //步兵与待击打装甲板平面距离
        DebugT(1, 11,"预测枪口坐标系:" << " X:"<< predict_worldPTZ.x*1000<< " Y:"<< predict_worldPTZ.y*1000<< " Z:"<< predict_worldPTZ.z*1000 ); // 右边

//        cout << "result: "<<result_x<<" " <<result_y<<" "<<result_z<<" target_distance:"<<target_distance<< endl;


    } else if (is_small){
        if (energy_rotation_direction == 1) predict_rad = predict_rad_norm;
        else if (energy_rotation_direction == -1) predict_rad = -predict_rad_norm;
        rotate(target_point);
    }
//    if (show_energy_predict)showPredict("predict", src);
}

