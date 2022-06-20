#include "energy/energy.h"

using namespace std;
using namespace cv;


//----------------------------------------------------------------------------------------------------------------------
// 此函数用于切换预测模式和猜测模式，但最终未使用
// ---------------------------------------------------------------------------------------------------------------------
void Energy::multipleMode(cv::Mat &src) {
    if (is_predicting) {
        getPredictPoint(src, target_point);
        getAimPoint(predict_point);
        judgeShoot();
        sendSmallEnergy();
    } else if (is_guessing && stayGuessing()) {
        findFans(src);
        if (show_energy)showFans("fans", src);
        if (save_mark)writeDownMark(src);
        guessTarget();
        if (show_energy)showGuessTarget("guess", src);
        getPredictPoint(src, guess_point);
        getAimPoint(predict_point);
        sendSmallEnergy();
    }
}