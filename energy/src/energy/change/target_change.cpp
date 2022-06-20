#include "energy/energy.h"

using namespace std;
using namespace cv;


//----------------------------------------------------------------------------------------------------------------------
// 此函数用于判断目标是否切换
// ---------------------------------------------------------------------------------------------------------------------
void Energy::changeTarget() {

    if (abs(last_target_polar_angle_judge_change - target_polar_angle) < 20 ||
        abs(last_target_polar_angle_judge_change - target_polar_angle) > 340) {
        change_target = false;
    } else {
//        cout <<"changetarget: "<< abs(last_target_polar_angle_judge_change - target_polar_angle) << endl;
        change_target = true;
    }
    last_target_polar_angle_judge_change = target_polar_angle;
}