//#include <opencv2/core/eigen.hpp>
//
//
//// 计算任意四边形的中心
//cv::Point2f points_center(cv::Point2f pts[4]) {
//    for (int i = 0; i < 4; ++i) {
//        for (int j = i+1; j < 4; ++j) {
//            if (pts[i] == pts[j]) {
//                std::cout << "[Error] Unable to calculate center point." << std::endl;
//                return cv::Point2f{0, 0};
//            }
//        }
//    }
//    cv::Point2f center(0, 0);
//    if (pts[0].x == pts[2].x && pts[1].x == pts[3].x) {
//        std::cout << "[Error] Unable to calculate center point." << std::endl;
//    }
//    else if (pts[0].x == pts[2].x && pts[1].x != pts[3].x) {
//        center.x = pts[0].x;
//        center.y = (pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*(pts[0].x-pts[3].x)+pts[3].y;
//    }
//    else if (pts[1].x == pts[3].x && pts[0].x != pts[2].x) {
//        center.x = pts[1].x;
//        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(pts[1].x-pts[0].x)+pts[0].y;
//    }
//    else {
//        center.x = (((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*pts[3].x - pts[3].y + \
//                    pts[0].y - (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*pts[0].x)) / \
//                    ((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)-(pts[2].y-pts[0].y)/(pts[2].x-pts[0].x));
//        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(center.x-pts[0].x)+pts[0].y;
//    }
//
//    return center;
//}
//
//// 欧拉角转旋转矩阵     代码参考：https://blog.csdn.net/coldplayplay/article/details/79271139
//Eigen::Matrix3d euler2RotationMatrix(double roll, double pitch, double yaw) {
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
//
//    Eigen::Quaterniond q = rollAngle*yawAngle*pitchAngle;
////    Eigen::Quaterniond q = pitchAngle*yawAngle*rollAngle;
//    Eigen::Matrix3d R = q.matrix().cast<double>();
////    cout << "Euler2RotationMatrix result is:" <<endl;
////    cout << "R = " << R <<endl;
//    return R;
//}
//
//// 欧拉角转四元数
//Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw) {
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
//
//    Eigen::Quaterniond q = rollAngle* yawAngle* pitchAngle;
////    cout << "Euler2Quaternion result is:" <<endl;
////    cout << "x = " << q.x() <<endl;
////    cout << "y = " << q.y() <<endl;
////    cout << "z = " << q.z() <<endl;
////    cout << "w = " << q.w() <<endl<<endl;
//    return q;
//}
//
//// 四元数转欧拉角
//Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
//{
//    Eigen::Quaterniond q;
//    q.x() = x;
//    q.y() = y;
//    q.z() = z;
//    q.w() = w;
//
//    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
//    cout << "Quaterniond2Euler result is:" <<endl;
//    cout << "x="  << euler[2]*180/PI << endl ;
//    cout << "y="  << euler[1]*180/PI << endl ;
//    cout << "z="  << euler[0]*180/PI << endl << endl;
//}
//
//// 旋转矩阵转欧拉角
//Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R)
//{
//    Eigen::Matrix3d m;
//    m = R;
//    Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
//    cout << "RotationMatrix2euler result is:" <<endl;
//    cout << "x="  << euler[2] << endl ;
//    cout << "y="  << euler[1] << endl ;
//    cout << "z="  << euler[0] << endl << endl;
//    return euler;
//}