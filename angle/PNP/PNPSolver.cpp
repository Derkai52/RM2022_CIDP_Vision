#include <string>
#include "PNPSolver.h"
#include "additions.h"

// �������ڿ��ٽ��PNP���⣬˳������ռ�������ת�Լ�ͼ��ϵ�����ϵ������ϵ��ϵ����ͶӰ����
// ����˳��
// 1.��ʼ������
// 2.����setCameraMatrix(),setDistortionCoefficients()���ú�����ڲ����뾵ͷ�������
// 3.��Points3D��Points2D�����һһ��Ӧ���������
// 4.����Solve()�������м���
// 5.��RoteM, TransM, W2CTheta��������������
PNPSolver::PNPSolver(CameraCalibrationStruct &calibrationData) {
    std::cout << calibrationData.fx<<std::endl;
    setCameraMatrix(calibrationData.fx, calibrationData.fy, calibrationData.u0, calibrationData.v0);
    setDistortionCoefficients(calibrationData.k_1, calibrationData.k_2, calibrationData.p_1, calibrationData.p_2, calibrationData.k_3);
}

int PNPSolver::Solve(METHOD method) {
    //����У��
    if (camera_matrix.cols == 0 || distortion_coefficients.cols == 0) {
        printf("ErrCode:-1,����ڲ�����������δ���ã�\r\n");
        return -1;
    }

    if (Points3D.size() != Points2D.size()) {
        printf("ErrCode:-2��3D��������2D��������һ�£�\r\n");
        return -2;
    }
    if (method == METHOD::P3P || method == METHOD::ITERATIVE) {
        if (Points3D.size() != 4) {
            printf("ErrCode:-2,ʹ��CV_ITERATIVE��CV_P3P����ʱ���������������ӦΪ4��\r\n");
            return -2;
        }
    } else {
        if (Points3D.size() < 4) {
            printf("ErrCode:-2,���������������Ӧ����4��\r\n");
            return -2;
        }
    }

    ////�����Ƿ��ǹ�����ĵ�
    //if ((method == METHOD::CV_ITERATIVE || method == METHOD::CV_EPNP) && Points2D.size() == 4){
    //	//ͨ������������˻�÷����������������Ƿ�ƽ��
    //}






    /*******************���PNP����*********************/
    //�����ַ������
    cv::solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, method);    //ʵ��������ƺ�ֻ���ù�����������λ��
//    cv::solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec);    //ʵ��������ƺ�ֻ���ù�����������λ��
    //solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);	//ʵ��������ƺ�ֻ���ù�����������λ��
    //solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_P3P);		//Gao�ķ�������ʹ�������ĸ�������
    //solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);

    /*******************��ȡ��ת����*********************/
    /*
        r11 r12 r13
        r21 r22 r23
        r31 r32 r33
    */
    double rm[9];
    RoteM = cv::Mat(3, 3, CV_64FC1, rm);

    Rodrigues(rvec, RoteM);

    double r11 = RoteM.ptr<double>(0)[0];
    double r12 = RoteM.ptr<double>(0)[1];
    double r13 = RoteM.ptr<double>(0)[2];
    double r21 = RoteM.ptr<double>(1)[0];
    double r22 = RoteM.ptr<double>(1)[1];
    double r23 = RoteM.ptr<double>(1)[2];
    double r31 = RoteM.ptr<double>(2)[0];
    double r32 = RoteM.ptr<double>(2)[1];
    double r33 = RoteM.ptr<double>(2)[2];
    TransM = tvec;

    //����ת����zxy˳����ŷ����
    //�����������������pitch������90��(һ�㲻�ᷢ��)
    double thetaz = atan2(-r12, r22) / CV_PI * 180;
    double thetax = atan2(r32, sqrt(r31 * r31 + r33 * r33)) / CV_PI * 180;
    double thetay = atan2(-r31, r33) / CV_PI * 180;

    //˳��Ϊzxy
    Theta_C2W.z = (float) thetaz;
    Theta_C2W.x = (float) thetax;
    Theta_C2W.y = (float) thetay;

    //������ת˳��ӦΪyxz
    Theta_W2C.y = -1 * (float) thetay;
    Theta_W2C.x = -1 * (float) thetax;
    Theta_W2C.z = -1 * (float) thetaz;

    ////������������ϵ��������תŷ���ǣ���ת�����ת����������ϵ��
    ////��ת˳��Ϊz��y��x
    //double thetaz = atan2(r21, r11) / CV_PI * 180;
    //double thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
    //double thetax = atan2(r32, r33) / CV_PI * 180;

    ////���ϵ������ϵ��������תŷ���ǣ��������ϵ�մ���ת���������������ϵ��ȫƽ�С�
    ////��ת˳��Ϊz��y��x
    //Theta_C2W.z = (float) thetaz;
    //Theta_C2W.y = (float) thetay;
    //Theta_C2W.x = (float) thetax;

    ////���������ϵ�����ϵ��������תŷ���ǣ�����ϵ�մ���ת�����ת���������ϵ��
    ////��ת˳��Ϊx��y��z
    //Theta_W2C.x = -1 * (float) thetax;
    //Theta_W2C.y = -1 * (float) thetay;
    //Theta_W2C.z = -1 * (float) thetaz;


    /*************************************�˴�������������ϵԭ��Oc����������ϵ�е�λ��**********************************************/

    /***********************************************************************************/
    /* ��ԭʼ����ϵ������תz��y��x������ת������������ϵƽ�У�����OcOw�������ת */
    /* ��������֪��������������ϵ��ȫƽ��ʱ��OcOw��ֵ */
    /* ��ˣ�ԭʼ����ϵÿ����ת��ɺ󣬶�����OcOw����һ�η�����ת�����տ��Եõ���������ϵ��ȫƽ��ʱ��OcOw */
    /* ����������-1������������ϵ����������� */
    /***********************************************************************************/

    //���ƽ�ƾ��󣬱�ʾ���������ϵԭ�㣬��������(x,y,z)�ߣ��͵�����������ϵԭ��
    double tx = tvec.ptr<double>(0)[0];
    double ty = tvec.ptr<double>(0)[1];
    double tz = tvec.ptr<double>(0)[2];
//    std::cout<<"PNP测出来平移向量trev:"<<tx<<" " <<ty<<" "<<tz<<std::endl<<std::endl;

    // TODO: 这里因为相机标定和PNP解算的原因，暂时用这种方法使之变成正常值
//    tx = tx*4.07; // 这里很奇怪，x和y分量单位应该是mm，但是是cm，所以这里统一单位为mm。而z分量却正确，迷惑。
//    ty = ty*4.27;
//
    tx = tx*2; // 这里很奇怪，x和y分量单位应该是mm，但是是cm，所以这里统一单位为mm。而z分量却正确，迷惑。
    ty = ty*2;
//    tz = tz*1.13;
    tz = 7000;
//    std::cout<<"PNP测出来平移向量trev:"<<tx<<" " <<ty<<" "<<tz<<std::endl<<std::endl;
    DebugT(1, 8,"相机坐标系 X:" << tx << " Y:"<<ty <<" Z: "<< tz ); // 右边



    ////x y z ΪΨһ���������ԭʼ����ϵ�µ�����ֵ
    ////Ҳ��������OcOw���������ϵ�µ�ֵ    
    double x = tx, y = ty, z = tz;
    Position_OwInC.x = (float) x;
    Position_OwInC.y = (float) y;
    Position_OwInC.z = (float) z;

    //������ת˳��ӦΪyxz
    CodeRotateByY(x, z, -1 * thetay, x, z);
    CodeRotateByX(y, z, -1 * thetax, y, z);
    CodeRotateByZ(x, y, -1 * thetaz, x, y);

    //ע�⣬����ı���δ����ʹ�ù�����֪�Դ�
    //����������������ϵ�µ�λ������
    //������OcOw����������ϵ�µ�ֵ
    Position_OcInW.x = (float) x * -1;
    Position_OcInW.y = (float) y * -1;
    Position_OcInW.z = (float) z * -1;

    ////x y z ΪΨһ���������ԭʼ����ϵ�µ�����ֵ
    ////Ҳ��������OcOw���������ϵ�µ�ֵ
    //double x = tx, y = ty, z = tz;
    //Position_OwInC.x = (float) x;
    //Position_OwInC.y = (float) y;
    //Position_OwInC.z = (float) z;
    ////�������η�����ת
    //CodeRotateByZ(x, y, -1 * thetaz, x, y);
    //CodeRotateByY(x, z, -1 * thetay, x, z);
    //CodeRotateByX(y, z, -1 * thetax, y, z);

    ////����������������ϵ�µ�λ������
    ////������OcOw����������ϵ�µ�ֵ
    //Position_OcInW.x = (float) x * -1;
    //Position_OcInW.y = (float) y * -1;
    //Position_OcInW.z = (float) z * -1;

    return 0;
}


//���ݼ�����Ľ��������������ͶӰ��ͼ�񣬷�����������㼯
//����Ϊ��������ϵ�ĵ����꼯��
//���Ϊ��ͶӰ��ͼ���ϵ�ͼ�����꼯��
std::vector<cv::Point2f> PNPSolver::WordFrame2ImageFrame(std::vector<cv::Point3f> WorldPoints) {
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(WorldPoints, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);
    return projectedPoints;
}

//��������Ĳ�����ͼ������ת�������������
//ʹ��ǰ��Ҫ����Solve()������λ��
//����Ϊͼ���ϵĵ�����
//double FΪ��ͷ����
//���Ϊ���ڽ���=Fʱ���������ϵ����
cv::Point3f PNPSolver::ImageFrame2CameraFrame(cv::Point2f p, double F) {
    double fx;
    double fy;
    double u0;
    double v0;

    fx = camera_matrix.ptr<double>(0)[0];
    u0 = camera_matrix.ptr<double>(0)[2];
    fy = camera_matrix.ptr<double>(1)[1];
    v0 = camera_matrix.ptr<double>(1)[2];
    double zc = F;
    double xc = (p.x - u0) * F / fx;
    double yc = (p.y - v0) * F / fy;
    return cv::Point3f((float) xc, (float) yc, (float) zc);
}
