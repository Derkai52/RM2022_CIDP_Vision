#include "BuffCoordinateSolver.h"

extern cv::Mat ori_src;

BuffCoordinateSolver::BuffCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct &calibrationData) : ArmorCoordinateSolver(armorRealData, calibrationData) {

}

//神符预判流程函数
ArmorCoordinateSolver::CoordinateStruct BuffCoordinateSolver::calculateWorldCoordinate(cv::Point2f * verticesToDraw, float pitchAngle, float yawAngle, CarType carType) {
   _pitchAngle = pitchAngle;
    _yawAngle = yawAngle;

    //预判
    makeFinalVertices(verticesToDraw);

    //位姿解算
    ArmorCoordinateSolver::realCoordinateCalculate(verticesToDraw, carType);

    //真实坐标解算
    ArmorCoordinateSolver::actualCoordinateSolver();
    //滤波以及数据装填
    return ArmorCoordinateSolver::mixCoordinate();
}

void BuffCoordinateSolver::makeFinalVertices(cv::Point2f *vertices) {

    buffCoordinateCalculateInit();
    if (_buffDirOfRota != UNKNOW) {
        buffPredictCoordinate2D(vertices);
        _captureFlag = true;
    } else {
        _captureFlag = false;
    }
}

//预判前的初始化
void BuffCoordinateSolver::buffCoordinateCalculateInit() {
    //cout<<"init"<<endl;
    static float lastCircleAngle = 0.0f;
    static cv::Point2f lastTargetRectCenter = cv::Point2f(0.0f, 0.0f);
    static float circleAngleBias = 0.0f;
    _circleAngle180 = Util::getAngle(_buffSolverStruct.targetRect.center, cv::Point2f(2000.0f, _buffSolverStruct.R_2D_Point.y), _buffSolverStruct.R_2D_Point);
//    cout<<"当前目标位置： "<<_buffSolverStruct.targetRect.center<<endl;

    ////     TODO: 数据文件写入用于分析
//    ofstream outFile;
//    outFile.open(PROJECT_DIR"/energy_pixel_xy_.txt", ios::app);//保存的文件名
//    outFile<<to_string(_buffSolverStruct.targetRect.center.x);
//    outFile<<"\n";
//    outFile.close();//关闭文件写入流


    //旋转角度处理
    if (_buffSolverStruct.R_2D_Point.y < _buffSolverStruct.targetRect.center.y) {
        _circleAngle360 = 360.0f - _circleAngle180;
        _circleAngle180 = -_circleAngle180;
    } else {
        _circleAngle360 = _circleAngle180;
    }
    //神符方向重置
    if (_resetBuff) {
        if (_buffDirOfRota != UNKNOW) {
            _buffDirOfRota = UNKNOW;
            _buffInit = false;
            _buffAngleList.resize(0);
            _resetBuff = false;
        }
    }
    circleAngleBias = _circleAngle360 - lastCircleAngle;


    //判断是否是同一片叶片
    if ((fabsf(circleAngleBias) > 10.0f || (fabsf(lastTargetRectCenter.y - _buffSolverStruct.targetRect.center.y) > 40.0f) || (fabsf(lastTargetRectCenter.x - _buffSolverStruct.targetRect.center.x) > 40.0f))) {
        _sameTargetFlag = false;
    }
    else {
        _sameTargetFlag = true;
    }


    if (!_buffInit) {
        //过去角度处理
        if (!_sameTargetFlag) {
            _buffAngleList.resize(0);//角度变化过大认为叶片跳变
        } else {
            _buffAngleList.push_back(_circleAngle360);
        }
        //数据足够判断方向

        if (_buffAngleList.size() > 5) {//存五个数据用来判断
            float averageAngle = 0.0f;
            float buff = 0.0f;
            uint8_t count = 0;
            //逐差法计算
            for (size_t i = 0; i < _buffAngleList.size() / 2; i++) {
                buff = _buffAngleList[_buffAngleList.size() / 2 + i] - _buffAngleList[i];
                if (!buff) {
                    continue;
                } else {
                    averageAngle += buff;
                    count++;
                }
            }
            if (count) {
                averageAngle = averageAngle / count;
            }
            //判断旋转方向
            if (averageAngle < -1.0f) {
                _buffDirOfRota = CLOCKWISE;
            } else if (averageAngle > 1.0f) {
                _buffDirOfRota = ANTICLOCKWISE;
            } else {
                _buffDirOfRota = STOP;
            }
            _buffInit = true;
        }
    }

//    cout<<"yeah"<<_predictCoordinate<<endl;
    lastCircleAngle = _circleAngle360;
    lastTargetRectCenter = _buffSolverStruct.targetRect.center;
}

//二维预判
void BuffCoordinateSolver::buffPredictCoordinate2D(cv::Point2f *vertices) {
    float r = 0.0f;
    float addAngle = 0.0f;
    float resAngle = 0.0f;
    static cv::Point2f predictBias = cv::Point2f(0.0f, 0.0f);
//    cout << _buffDirOfRota<< endl;
//cout <<"目标中心点： "<< _buffSolverStruct.targetRect.center<< endl;

    switch (_buffDirOfRota) {
        case STOP: {
            addAngle = 0.0f;
        }
            break;
        case CLOCKWISE: {
            addAngle = calculateAddAngle();
        }
            break;
        case ANTICLOCKWISE: {
            addAngle = -calculateAddAngle();
        }
            break;
        case UNKNOW:
            break;
    }
//    cout << addAngle<< endl;
    if (_buffDirOfRota) {
        resAngle = -_circleAngle180 + addAngle;
        if (_circleAngle180 >= 180.0f) {
            resAngle = resAngle - 360.0f;
        } else if (resAngle < -180.0f) {
            resAngle = 360.0f + resAngle;
        }
        cv::Point2f points[4];
        _buffSolverStruct.targetRect.points(points);
        int index = 3;
        if(!_buffSolverStruct.shootSpeedLevel){
            index = 5;
        }

        //操作手外部按键调参接口
        if (_buffBias == UP) {
            _buffArmor.length += index * _armorRatio;
            _buffArmor.width += index * _armorRatio;
//            cout << "_buffArmor.length" << _buffArmor.length << "," << "_buffArmor.width" << _buffArmor.width << endl;
        } else if (_buffBias == DOWN) {
            _buffArmor.length -= index * _armorRatio;
            _buffArmor.width -= index * _armorRatio;
//            cout << "_buffArmor.length" << _buffArmor.length << "," << "_buffArmor.width" << _buffArmor.width << endl;
        }

        //位置预判
//        r = Util::pointDistance(_buffSolverStruct.R_2D_Point, _buffSolverStruct.targetRect.center)*1.08;
        r = Util::pointDistance(_buffSolverStruct.R_2D_Point, _buffSolverStruct.targetRect.center)*1.01;
        _predictCoordinate.x = _buffSolverStruct.R_2D_Point.x + r * cosf(resAngle * Util::PI_F() / 180.0f);
        _predictCoordinate.y = _buffSolverStruct.R_2D_Point.y + r * sinf(resAngle * Util::PI_F() / 180.0f);
    } else {
        _predictCoordinate = _buffSolverStruct.targetRect.center;
    }

    // 显示当前目标点
    static cv::Mat image2show;
    if (ori_src.type() == CV_8UC1) { // 黑白图像
        cvtColor(ori_src, image2show, cv::COLOR_GRAY2RGB);
    }
    cv::Point2f point = _buffSolverStruct.targetRect.center;
    cv::circle(image2show, point, 15, cv::Scalar(0, 255, 0),2);//在图像中画出特征点，2是圆的半径


    int cx = (int)_predictCoordinate.x;
    int cy = (int)_predictCoordinate.y;
//    cout << cx << endl;
    cv::Point2f point_pre = _predictCoordinate;
    cv::circle(image2show,point_pre,15,cv::Scalar(0,0,255),2);
    cv::line(image2show, cv::Point(320,0),cv::Point(320,480),(180,255,100),2);
    cv::line(image2show, cv::Point(0,240),cv::Point(640,240),(180,255,100),2);


    imshow("predict", image2show);

    predictBias= _predictCoordinate -_buffSolverStruct.targetRect.center;

    //为在图像上呈现预判点存储数据
    for (int i = 0; i < 4; i++) {
        vertices[i] = vertices[i] + predictBias;
    }
}

bool off_set = true;
float BuffCoordinateSolver::calculateAddAngle() {
    float time = 0.0f;
    //计算实时角速度
    calculateRotateSpeed();
    // Warning：应当写在下面，因为数据刷新的缘故
    DebugT(1, 4,"当前角速度:"<< _rotateSpeed.realRotateSpeed);
    DebugT(1, 5,"波峰速度:"<< _speedRange.realMaxSpeed<<" 波谷速度:"<< _speedRange.realMinSpeed<<" 振幅A:"<< _sineFunction.amplitude<<" 角频率W:"<< _sineFunction.rotateIndex<<" 偏移量B:"<< _sineFunction.para);

    //计算积分开始时刻（该时刻作为角速度函数的横坐标）
    time = calculateShootTime();
    if (time == 0.0f) {
        return float();
    }
    //小符
    if (!_isBig) {
        if (_buffBias == FRONT) {
            if(_buffSolverStruct.shootSpeedLevel)
            _paraCircle.buffPredictAngle += FRONT_BACK_PARA;
            else
            _paraCircle.buffPredictAngle += (FRONT_BACK_PARA+1.5);
            //cout << " _paraCircle.buffPredictAngle " << _paraCircle.buffPredictAngle << endl;
        } else if (_buffBias == BACK) {
            if(_buffSolverStruct.shootSpeedLevel)
            _paraCircle.buffPredictAngle -= FRONT_BACK_PARA;
            else
            _paraCircle.buffPredictAngle -= (FRONT_BACK_PARA+1.5);
            //cout<<"para"<<(FRONT_BACK_PARA+1)<<endl;
            //cout << " _paraCircle.buffPredictAngle " << _paraCircle.buffPredictAngle << endl;
        }
        return _paraCircle.buffPredictAngle;
    }
    //大符
    else {
        if (off_set){
//                    _sineFunction.para += 4*FRONT_BACK_SIN; // TODO: 修改初相
//                    _sineFunction.amplitude += 4*FRONT_BACK_SIN;
                _delayTime = DELAY_TIME+0.1f;  // TODO: BISAI
                    off_set = false;
            }

        if (_buffBias == FRONT) {
            _sineFunction.para += FRONT_BACK_SIN;
            _sineFunction.amplitude += FRONT_BACK_SIN;
            cout << " _sineFunction.para"  << _sineFunction.para << endl;
            cout << " _sineFunction.amplitude"  << _sineFunction.amplitude << endl;
        } else if (_buffBias == BACK) {
            _sineFunction.para -= FRONT_BACK_SIN;
            _sineFunction.amplitude -= FRONT_BACK_SIN;
            cout << " _sineFunction.para"  << _sineFunction.para << endl;
            cout << " _sineFunction.amplitude"  << _sineFunction.amplitude << endl;
        }
//        cout << " _sineFunction.para"  << _sineFunction.para << endl;
//        cout << " _sineFunction.amplitude"  << _sineFunction.amplitude << endl;

        //cout<<"shoot"<<_buffSolverStruct.shootSpeedLevel<<endl;
        if(!_buffSolverStruct.shootSpeedLevel){
            _delayTime = DELAY_TIME+0.25f;
            //cout<<"delay"<<_delayTime<<endl;
        }

        //积分出改变的角度，对时间进行积分
        _realAddAngle = (_sineFunction.amplitude / _sineFunction.rotateIndex
                         * (cosf(_sineFunction.rotateIndex * time) - cosf(_sineFunction.rotateIndex * (time + _delayTime))) + _sineFunction.para * DELAY_TIME)
                        * 180 / Util::PI_F() + _para;
        //_realAddAngle = 30;
        //_realAddAngle = _realAddAngle*0.8;
//        cout << "predictAddAngle:"<<_realAddAngle <<endl;
        DebugT(30, 4,"预测角度:"<< _realAddAngle<<" 度" );
        DebugT(1, 6,"当前角速度目标函数: Spd: "<< _sineFunction.amplitude<<" * sin("<<_sineFunction.rotateIndex<<" * "<<time<< ") + "<< _sineFunction.para );

//                ////     TODO: 数据文件写入用于分析
//    ofstream outFile;
//    outFile.open(PROJECT_DIR"/energy_addangle.txt", ios::app);//保存的文件名
//    outFile<<to_string(_realAddAngle);
//    outFile<<"\n";
//    outFile.close();//关闭文件写入流


        return _realAddAngle;
    }
}

void BuffCoordinateSolver::calculateRotateSpeed() {
    //定义静态过去和现在角度；
    static double nowAngle = 0.0f;
    static double lastAngle = 0.0f;
    static int count = 0;
    //定义过去和现在时间
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; // ms
    double curTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000;
    //如果叶片没有跳变，则把过去和现在角度以及过去和现在速度置零
//cout << _buffSolverStruct.targetRect.center<< endl;

    if (!_buffSolverStruct.sameTargetFlag) {
        lastAngle = nowAngle = _rotateSpeed.lastRotateSpeed = _rotateSpeed.nowRotateSpeed = 0.0f;
        DebugT(35, 2,"目标切换");
        return;
    }


    //如果过去角度已经被清零，则过去角度进行初始化为现在绝对角度
    if (lastAngle == 0.0f) {
        lastAngle = _circleAngle360;
        return;
    }

    //每0.1s一次数据刷新
    if (curTime - lastTime < 100) {
        return;
    }

    //帧数递增
    count++;
    nowAngle = _circleAngle360;
    //计算实时角速度
    _rotateSpeed.nowRotateSpeed = (float) fabs(Util::angleToRadian((nowAngle - lastAngle)) * (1000.0f / (curTime - lastTime))); //curTime - lastTime

//            ////     TODO: 数据文件写入用于分析
//    ofstream outFile;
//    outFile.open(PROJECT_DIR"/energy_speed_fast.txt", ios::app);//保存的文件名
//    outFile<<to_string(_rotateSpeed.nowRotateSpeed);
//    outFile<<"\n";
//    outFile.close();//关闭文件写入流


    cout << _rotateSpeed.nowRotateSpeed << endl;
//    cout << "FPS: "<<(1000.0f / (curTime - lastTime)) << endl;

    //过去角度和时间更新
    lastAngle = nowAngle;
    lastTime = curTime;
    //如果过去角速度已被清零，则对过去速度进行更新
    if (_rotateSpeed.lastRotateSpeed == 0.0f) {
        _rotateSpeed.lastRotateSpeed = _rotateSpeed.nowRotateSpeed;
        return;
    }
    //防止出现异常数据
    if (_rotateSpeed.nowRotateSpeed > 5 || _rotateSpeed.nowRotateSpeed < -5) {
        return;
    }
    //如果速度没有替换最小速度，则计数加1
    if (_speedRange.nowMinSpeed > _rotateSpeed.nowRotateSpeed) {
        _speedRange.nowMinSpeed = _rotateSpeed.nowRotateSpeed;
    } else {
        _speedRange.minSameNumber++;
    }
    //如果速度没有替换最大速度，则计数加1
    if (_speedRange.nowMaxSpeed < _rotateSpeed.nowRotateSpeed) {
        _speedRange.nowMaxSpeed = _rotateSpeed.nowRotateSpeed;
    } else {
        _speedRange.maxSameNumber++;
    }
    //如果连续20帧没有刷新最小速度，则该速度为波谷速度（该速度一旦更新，便不再更新）
    if (_speedRange.minSameNumber > 20 && !_speedRange.minSpeedFlag) {
        _speedRange.realMinSpeed = _speedRange.nowMinSpeed;
        _speedRange.minSpeedFlag = true;
    }
    //如果连续20帧没有刷新最大速度，则该速度为波峰速度（该速度一旦更新，便不再更新）
    if (_speedRange.maxSameNumber > 20 && !_speedRange.maxSpeedFlag) {
        _speedRange.realMaxSpeed = _speedRange.nowMaxSpeed;
        _speedRange.maxSpeedFlag = true;
    }
//    cout<<"update"<<_speedRange.realMinSpeed <<" "<<_speedRange.realMaxSpeed<<endl;
//    cout<<"update"<<_speedRange.realMinSpeed <<" "<<_speedRange.realMaxSpeed<<endl;
//    _speedRange.realMinSpeed = 0.265;
    _speedRange.realMinSpeed = 0.265;
    _speedRange.realMaxSpeed = 2.09;// TODO: 应当使用拟合，而不是定值


    //此时更新正弦函数的三个参数（振幅、角速度、偏移量）//因为在不同的角度计算出的速度不同，导致函数的基本参数发生了改变
//    if (_speedRange.minSpeedFlag && _speedRange.maxSpeedFlag) {
//        _sineFunction.para = (_speedRange.realMaxSpeed + _speedRange.realMinSpeed) / 2;
//        _sineFunction.amplitude = _speedRange.realMaxSpeed - _sineFunction.para;
//        //cout<<"para"<<_sineFunction.para<<endl;
//        //cout<<"amplitude"<<_sineFunction.amplitude<<endl;
//    }
    //赋值真实速度，方便后面使用
    _rotateSpeed.realRotateSpeed = _rotateSpeed.nowRotateSpeed;

    _rotateSpeed.speedType = (_rotateSpeed.nowRotateSpeed > _rotateSpeed.lastRotateSpeed ? SPEED_UP : SPEED_DOWN);



//    cout << "SpeedType:"<< _rotateSpeed.speedType<<" RealRotateSpeed: "<<_rotateSpeed.realRotateSpeed<< endl;

//// TODO: 数据文件写入用于分析
//    ofstream outFile;
//    outFile.open(PROJECT_DIR"/rotatespeed.txt", ios::app);//保存的文件名
//    outFile<<to_string(_rotateSpeed.realRotateSpeed);
//    outFile<<"\n";
//    outFile.close();//关闭文件写入流

}

float BuffCoordinateSolver::calculateShootTime() {
    if (_rotateSpeed.realRotateSpeed <= 0.0f) {
        return float();
    }
    //spd=0.9125*sin(1.942*t)+1.1775
    float possibleTime[2];
    float realTime;

    //速度超限要改
    if (_rotateSpeed.realRotateSpeed < _sineFunction.para - _sineFunction.amplitude) {
        _rotateSpeed.realRotateSpeed = _sineFunction.para - _sineFunction.amplitude;
    }
    if (_rotateSpeed.realRotateSpeed > _sineFunction.para + _sineFunction.amplitude) {
        _rotateSpeed.realRotateSpeed = _sineFunction.para + _sineFunction.amplitude;
    }

    possibleTime[0] = (asinf((_rotateSpeed.realRotateSpeed - _sineFunction.para) / _sineFunction.amplitude)) / _sineFunction.rotateIndex;
    possibleTime[1] = (possibleTime[0] > 0 ? Util::PI_F() / (_sineFunction.rotateIndex) - possibleTime[0] : Util::PI_F() / (-_sineFunction.rotateIndex) - possibleTime[0]);
   // cout << "    " << _rotateSpeed.speedType << endl;
    realTime = (_rotateSpeed.speedType == SPEED_UP ? possibleTime[0] : possibleTime[1]);
//    cout << "time--" << fabs(possibleTime[0] - possibleTime[1]) << endl;
    return realTime;
}



