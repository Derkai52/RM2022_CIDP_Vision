#ifndef _CONF_H_
#define _CONF_H_

#include "fileOperation/fileOperation.h"
#include "PointUtil.h"
#include "additions.h"

//??????????????
struct ArmorRealData {
//    float length = 120.0f;
//    float width = 55.0f;
//    float length = 110.0f; // 207
//    float width = 70.0f; // 207
    float length = 130.0f; // SJ
    float width = 90.0f; //SJ
};
//??????????
struct ArmorPara {
    EnemyColor enemyColor;
    int minLightBarArea = 20;
    int minLightBarLength = 5;
    int minLightBarWidth = 2;
    int maxLightBarLength = 200;
    int maxLightBarWidth = 50;
    float maxErrorAngle = 15;
    int grayThreshold_PURPLE = 150;
    int grayThreshold_RED = 50;
    int separationThreshold_RED = 50;
    int grayThreshold_BLUE = 160;
    int separationThreshold_BLUE = 80;
    int separationThreshold_GREEN = 10;
};

//?????????
struct BuffPara {
    int minLightBarGray_RED_R = 220;
    int minLightBarGray_RED_G = 100;
    int minLightBarGray_BLUE_R = 30;
    int minLightBarGray_BLUE_G = 150;
    int minLightBarSize = 15;
    int maxLightBarSize = 200;
    int grayThreshold_RED = 60;                            //??????????-???
    int grayThreshold_BLUE = 100;                            //??????????-???
    int separationThreshold_RED = 40;                    //?????????????-???
    int separationThreshold_BLUE = 100;                    //?????????????-???
};
//????????
struct ParaCircle {
    float buffPredictAngle = 25.0f;
    float buffRadBias = 2.0f;
    float realCantileverLength = 730.0f;
    float armorAngleBias = 5.0f;
    float minToArmorDistance = 220;//80;
    float maxToArmorDistance = 600;
    float minArea = 80.0f;
    float maxArea = 400;
    float minLengthWidthRatio = 0.7f;
    float maxLengthWidthRatio = 1.5f;
};

//?°????????
struct NewBuffPara {
    float grayThresholdValue = 60;
    float channelThresholdValue = 60;
    int singleIterations = 1;
    int binMatIterations = 2;
    float armorSideRatio_MAX = 4.2f;
    float armorSideRatio_MIN = 1.4f;
    float armorTo_R_SizeRatio_MAX = 9;
    float armorTo_R_SizeRatio_MIN = 1.5f;
    float armorTo_R_DistanceRatio_MAX = 11.0f;
    float armorTo_R_DistanceRatio_MIN = 6.0f;
    float longSideRatioForROI = 2.0f;
    float shortSideRatioForROI = 0.8f;
    float targetSizeRadio = 0.8f;
};

//??????
struct R_Para {
    float squreSideRatio = 1.5f;
    float squreSizeLimit_MAX = 160;
    float squreSizeLimit_MIN = 12;
};

struct BuffPredict {

};

//???????
struct CameraConfiguration {
    double gainValue = 3;                       //????
    int exposureValue = 5000;                   //??????
    double gammaValue = 1;                      //gamma
};

//???????
struct InstallData {
    cv::Point3f PTZToCameraBias;                //???????????????????????????
    cv::Point3f PTZToBarrelBias;                //????????????????????????????????
    cv::Point3f BarrelToCameraBias;                //??????????????????????????????
};

class ArmorDataFactory {
private:
    static ArmorDataFactory &instance() {
        static ArmorDataFactory armorDataFactory;
        return armorDataFactory;
    }

    static void initArmorData() {
        //С???
        instance().smallArmor.length = 130.0f;
        instance().smallArmor.width = 55.0f;
        //?????
        instance().bigArmor.length = 225.0f;
        instance().bigArmor.width = 55.0f;
        //???
        instance().buffArmor.length = 245.0f;
        instance().buffArmor.width = 137.0f;
        //???
        instance().r_Armor.length = 60.0f;
        instance().r_Armor.width = 60.0f;
    }
public:
    static ArmorRealData getSmallArmor() {
        return instance().smallArmor;
    }

    static ArmorRealData getBigArmor() {
        return instance().bigArmor;
    }

    static ArmorRealData getBuffArmor() {
        return instance().buffArmor;
    }

    static ArmorRealData get_R_Armor() {
        return instance().r_Armor;
    }


    static void resetAllConfig() {
        initArmorData();
    }

    static void readArmor(const std::string &filename) {
        initArmorData();
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeArmor(filename);
            cout<< "default config not find, write config to " + filename << endl;
        } else {
            cv::FileNode map_node = fs["SmallArmor"];
            map_node["length"] >> instance().smallArmor.length;
            map_node["width"] >> instance().smallArmor.width;
//            LOG::debug("smallArmor.length= " + std::to_string(instance().smallArmor.length));
//            LOG::debug("smallArmor.width= " + std::to_string(instance().smallArmor.width));

            map_node = fs["BigArmor"];
            map_node["length"] >> instance().bigArmor.length;
            map_node["width"] >> instance().bigArmor.width;
//            LOG::debug("bigArmor.length= " + std::to_string(instance().bigArmor.length));
//            LOG::debug("bigArmor.width= " + std::to_string(instance().bigArmor.width));

            map_node = fs["BuffArmor"];
            map_node["length"] >> instance().buffArmor.length;
            map_node["width"] >> instance().buffArmor.width;
//            LOG::debug("buffArmor.length= " + std::to_string(instance().buffArmor.length));
//            LOG::debug("buffArmor.width= " + std::to_string(instance().buffArmor.width));

            map_node = fs["R_Armor"];
            map_node["length"] >> instance().r_Armor.length;
            map_node["width"] >> instance().r_Armor.width;
//            LOG::debug("r_Armor.length= " + std::to_string(instance().r_Armor.length));
//            LOG::debug("r_Armor.width= " + std::to_string(instance().r_Armor.width));

        }
    }

    static void writeArmor(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //д?????
        fs.writeComment("????????С????\n");
        //cv::cvWriteComment(*fs, "????????С????\n", 0);

        //д??ARMOR???
        fs.writeComment("С????????");
        fs << "SmallArmor" << "{"
           << "length" << instance().smallArmor.length
           << "width" << instance().smallArmor.width
           << "}";

        fs.writeComment("??????????");
        fs << "BigArmor" << "{"
           << "length" << instance().bigArmor.length
           << "width" << instance().bigArmor.width
           << "}";

        fs.writeComment("???????????");
        fs << "BuffArmor" << "{"
           << "length" << instance().buffArmor.length
           << "width" << instance().buffArmor.width
           << "}";

        fs.writeComment("???R????");
        fs << "R_Armor" << "{"
            << "length" << instance().r_Armor.length
            << "width" << instance().r_Armor.width
            << "}";

        //????
        fs.release();
    }

public:
    ArmorRealData smallArmor;
    ArmorRealData bigArmor;
    ArmorRealData buffArmor;
    ArmorRealData r_Armor;
};

class ArmorParaFactory {
private:
    static ArmorParaFactory &instance() {
        static ArmorParaFactory armorParaFactory;
        return armorParaFactory;
    }

public:
    static ArmorPara getArmorPara() {
        return instance().armorPara;
    }

    static void resetAllConfig() {
        instance().armorPara = ArmorPara();
    }

    static void readArmorPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeArmorPara(filename);
             cout<<"default config not find, write config to " + filename<<endl;
        } else {
            // ??????????
            //????map???????
            cv::FileNode map_node = fs["armorPara"];
            map_node["minLightBarArea"] >> instance().armorPara.minLightBarArea;
            map_node["minLightBarLength"] >> instance().armorPara.minLightBarLength;
            map_node["minLightBarWidth"] >> instance().armorPara.minLightBarWidth;
            map_node["maxLightBarLength"] >> instance().armorPara.maxLightBarLength;
            map_node["maxLightBarWidth"] >> instance().armorPara.maxLightBarWidth;
            map_node["maxErrorAngle"] >> instance().armorPara.maxErrorAngle;
            map_node["grayThreshold_PURPLE"] >> instance().armorPara.grayThreshold_PURPLE;
            map_node["grayThreshold_RED"] >> instance().armorPara.grayThreshold_RED;
            map_node["separationThreshold_RED"] >> instance().armorPara.separationThreshold_RED;
            map_node["grayThreshold_BLUE"] >> instance().armorPara.grayThreshold_BLUE;
            map_node["separationThreshold_BLUE"] >> instance().armorPara.separationThreshold_BLUE;
            map_node["separationThreshold_GREEN"] >> instance().armorPara.separationThreshold_GREEN;
//            LOG::debug("armorPara.minLightBarArea=" + std::to_string(instance().armorPara.minLightBarArea));
//            LOG::debug("armorPara.minLightBarLength=" + std::to_string(instance().armorPara.minLightBarLength));
//            LOG::debug("armorPara.minLightBarWidth=" + std::to_string(instance().armorPara.minLightBarWidth));
//            LOG::debug("armorPara.maxLightBarLength=" + std::to_string(instance().armorPara.maxLightBarLength));
//            LOG::debug("armorPara.maxLightBarWidth=" + std::to_string(instance().armorPara.maxLightBarWidth));
//            LOG::debug("armorPara.maxErrorAngle=" + std::to_string(instance().armorPara.maxErrorAngle));
//            LOG::debug("armorPara.grayThreshold_PURPLE=" + std::to_string(instance().armorPara.grayThreshold_PURPLE));
//            LOG::debug("armorPara.grayThreshold_RED=" + std::to_string(instance().armorPara.grayThreshold_RED));
//            LOG::debug("armorPara.separationThreshold_RED=" + std::to_string(instance().armorPara.separationThreshold_RED));
//            LOG::debug("armorPara.grayThreshold_BLUE=" + std::to_string(instance().armorPara.grayThreshold_BLUE));
//            LOG::debug("armorPara.separationThreshold_BLUE=" + std::to_string(instance().armorPara.separationThreshold_BLUE));
//            LOG::debug("armorPara.separationThreshold_GREEN=" + std::to_string(instance().armorPara.separationThreshold_GREEN));
        }
    }

    static void writeArmorPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);

        fs.writeComment("???????????");
        fs << "armorPara" << "{";
        fs.writeComment("??С????????");
        fs << "minLightBarArea" << instance().armorPara.minLightBarArea;
        fs.writeComment("??С?????????");
        fs << "minLightBarLength" << instance().armorPara.minLightBarLength;
        fs.writeComment("??С????????");
        fs << "minLightBarWidth" << instance().armorPara.minLightBarWidth;
        fs.writeComment("??????????");
        fs << "maxLightBarLength" << instance().armorPara.maxLightBarLength;
        fs.writeComment("?????????");
        fs << "maxLightBarWidth" << instance().armorPara.maxLightBarWidth;
        fs.writeComment("???????????");
        fs << "maxErrorAngle" << instance().armorPara.maxErrorAngle;
        fs.writeComment("??????????-???");
        fs << "grayThreshold_PURPLE" << instance().armorPara.grayThreshold_PURPLE;
        fs.writeComment("??????????-???");
        fs << "grayThreshold_RED" << instance().armorPara.grayThreshold_RED;
        fs.writeComment("?????????????-???");
        fs << "separationThreshold_RED" << instance().armorPara.separationThreshold_RED;
        fs.writeComment("??????????-???");
        fs << "grayThreshold_BLUE" << instance().armorPara.grayThreshold_BLUE;
        fs.writeComment("?????????????-???");
        fs << "separationThreshold_BLUE" << instance().armorPara.separationThreshold_BLUE;
        fs.writeComment("?????????????-???");
        fs << "separationThreshold_GREEN" << instance().armorPara.separationThreshold_GREEN;
        fs << "}";
        //????
        fs.release();
    }

public:
    ArmorPara armorPara;
};

class BuffParaFactory {
private:
    static BuffParaFactory &instance() {
        static BuffParaFactory buffParaFactory;
        return buffParaFactory;
    }

public:
    static BuffPara getPara() {
        return instance().buffPara;
    }

    static ParaCircle getParaCircle() {
        return instance().paraCircle;
    }

    static void resetAllConfig() {
        instance().buffPara = BuffPara();
        instance().paraCircle = ParaCircle();
    }

    static void readBuffPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeBuffPara(filename);
            cout<<"default config not find, write config to " + filename<<endl;
        } else {
            //????map???????
            cv::FileNode map_node = fs["BuffPara"];
            map_node["minLightBarGray_RED_R"] >> instance().buffPara.minLightBarGray_RED_R;
            map_node["minLightBarGray_RED_G"] >> instance().buffPara.minLightBarGray_RED_G;
            map_node["minLightBarGray_BLUE_R"] >> instance().buffPara.minLightBarGray_BLUE_R;
            map_node["minLightBarGray_BLUE_G"] >> instance().buffPara.minLightBarGray_BLUE_G;
            map_node["minLightBarSize"] >> instance().buffPara.minLightBarSize;
            map_node["maxLightBarSize"] >> instance().buffPara.maxLightBarSize;
            map_node["grayThreshold_RED"] >> instance().buffPara.grayThreshold_RED;
            map_node["grayThreshold_BLUE"] >> instance().buffPara.grayThreshold_BLUE;
            map_node["separationThreshold_RED"] >> instance().buffPara.separationThreshold_RED;
            map_node["separationThreshold_BLUE"] >> instance().buffPara.separationThreshold_BLUE;
//            LOG::debug("BuffPara.minLightBarGray_RED_R=" + std::to_string(instance().buffPara.minLightBarGray_RED_R));
//            LOG::debug("BuffPara.minLightBarGray_RED_G=" + std::to_string(instance().buffPara.minLightBarGray_RED_G));
//            LOG::debug("BuffPara.minLightBarGray_BLUE_R=" + std::to_string(instance().buffPara.minLightBarGray_BLUE_R));
//            LOG::debug("BuffPara.minLightBarGray_BLUE_G=" + std::to_string(instance().buffPara.minLightBarGray_BLUE_G));
//            LOG::debug("BuffPara.grayThreshold_RED=" + std::to_string(instance().buffPara.grayThreshold_RED));
//            LOG::debug("BuffPara.grayThreshold_BLUE=" + std::to_string(instance().buffPara.grayThreshold_BLUE));
//            LOG::debug("BuffPara.separationThreshold_RED=" + std::to_string(instance().buffPara.separationThreshold_RED));
//            LOG::debug("BuffPara.separationThreshold_BLUE=" + std::to_string(instance().buffPara.separationThreshold_BLUE));
//            LOG::debug("BuffPara.minLightBarSize=" + std::to_string(instance().buffPara.minLightBarSize));
//            LOG::debug("BuffPara.maxLightBarSize=" + std::to_string(instance().buffPara.maxLightBarSize));

            map_node = fs["paraCircle"];
            map_node["armorAngleBias"] >> instance().paraCircle.armorAngleBias;
            map_node["minToArmorDistance"] >> instance().paraCircle.minToArmorDistance;
            map_node["maxToArmorDistance"] >> instance().paraCircle.maxToArmorDistance;
            map_node["minArea"] >> instance().paraCircle.minArea;
            map_node["maxArea"] >> instance().paraCircle.maxArea;
            map_node["minLengthWidthRatio"] >> instance().paraCircle.minLengthWidthRatio;
            map_node["maxLengthWidthRatio"] >> instance().paraCircle.maxLengthWidthRatio;
            map_node["buffRadBias"] >> instance().paraCircle.buffRadBias;
            map_node["buffPredictAngle"] >> instance().paraCircle.buffPredictAngle;
            map_node["realCantileverLength"] >> instance().paraCircle.realCantileverLength;
//            LOG::debug("paraCircle.armorAngleBias=" + std::to_string(instance().paraCircle.armorAngleBias));
//            LOG::debug("paraCircle.minToArmorDistance=" + std::to_string(instance().paraCircle.minToArmorDistance));
//            LOG::debug("paraCircle.maxToArmorDistance=" + std::to_string(instance().paraCircle.maxToArmorDistance));
//            LOG::debug("paraCircle.minArea=" + std::to_string(instance().paraCircle.minArea));
//            LOG::debug("paraCircle.maxArea=" + std::to_string(instance().paraCircle.maxArea));
//            LOG::debug("paraCircle.minLengthWidthRatio=" + std::to_string(instance().paraCircle.minLengthWidthRatio));
//            LOG::debug("paraCircle.maxLengthWidthRatio=" + std::to_string(instance().paraCircle.maxLengthWidthRatio));
//            LOG::debug("paraCircle.buffRadBias=" + std::to_string(instance().paraCircle.buffRadBias));
//            LOG::debug("paraCircle.buffPredictAngle=" + std::to_string(instance().paraCircle.buffPredictAngle));
//            LOG::debug("paraCircle.realCantileverLength=" + std::to_string(instance().paraCircle.realCantileverLength));
        }
    }

    static void writeBuffPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //д?????
        fs.writeComment("???????");
        //д??para???
        fs << "BuffPara" << "{";
        fs.writeComment("RED???red  channel??С????");
        fs << "minLightBarGray_RED_R" << instance().buffPara.minLightBarGray_RED_R;
        fs.writeComment("RED???Green channel??С????");
        fs << "minLightBarGray_RED_G" << instance().buffPara.minLightBarGray_RED_G;
        fs.writeComment("BLUE???red channel??С????");
        fs << "minLightBarGray_BLUE_R" << instance().buffPara.minLightBarGray_BLUE_R;
        fs.writeComment("BLUE???Green channel??С????");
        fs << "minLightBarGray_BLUE_G" << instance().buffPara.minLightBarGray_BLUE_G;
        fs.writeComment("??С???????");
        fs << "minLightBarSize" << instance().buffPara.minLightBarSize;
        fs.writeComment("??????????");
        fs << "maxLightBarSize" << instance().buffPara.maxLightBarSize;
        fs.writeComment("?????????????");
        fs << "grayThreshold_RED" << instance().buffPara.grayThreshold_RED;
        fs.writeComment("?????????????");
        fs << "grayThreshold_BLUE" << instance().buffPara.grayThreshold_BLUE;
        fs.writeComment("????????????????");
        fs << "separationThreshold_RED" << instance().buffPara.separationThreshold_RED;
        fs.writeComment("????????????????");
        fs << "separationThreshold_BLUE" << instance().buffPara.separationThreshold_BLUE;
        fs << "}";
        //д??paraCircle???
        fs << "paraCircle" << "{";
        fs.writeComment("?????");
        fs << "armorAngleBias" << instance().paraCircle.armorAngleBias;
        fs.writeComment("??С????");
        fs << "minToArmorDistance" << instance().paraCircle.minToArmorDistance;
        fs.writeComment("??????");
        fs << "maxToArmorDistance" << instance().paraCircle.maxToArmorDistance;
        fs.writeComment("??С???");
        fs << "minArea" << instance().paraCircle.minArea;
        fs.writeComment("??????");
        fs << "maxArea" << instance().paraCircle.maxArea;
        fs.writeComment("??С?????");
        fs << "minLengthWidthRatio" << instance().paraCircle.minLengthWidthRatio;
        fs.writeComment("?????");
        fs << "maxLengthWidthRatio" << instance().paraCircle.maxLengthWidthRatio;
        fs.writeComment("?????н??");
        fs << "buffPredictAngle" << instance().paraCircle.buffPredictAngle;
        fs.writeComment("??????");
        fs << "buffRadBias" << instance().paraCircle.buffRadBias;
        fs.writeComment("???????????");
        fs << "realCantileverLength" << instance().paraCircle.realCantileverLength;
        fs << "}";
        //????
        fs.release();
    }

public:
    BuffPara buffPara;
    ParaCircle paraCircle;
};

class NewBuffParaFactory {
private:
    static NewBuffParaFactory& instance() {
        static NewBuffParaFactory newBuffParaFactory;
        return newBuffParaFactory;
    }
public:
    static NewBuffPara getNewBuff() {
        return instance().newBuffPara;
    }

    static R_Para get_R() {
        return instance().r_Para;
    }

    static void resetAllConfig() {
        instance().newBuffPara = NewBuffPara();
        instance().r_Para = R_Para();
    }

    static void readNewBuffPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeNewBuffPara(filename);
            cout<<"default config not find, write config to " + filename<<endl;
        }
        else {
            //????map??????
            cv::FileNode buffNode = fs["NewBuffPara"];
            buffNode["grayThresholdValue"] >> instance().newBuffPara.grayThresholdValue;
            buffNode["channelThresholdValue"] >> instance().newBuffPara.channelThresholdValue;
            buffNode["singleIterations"] >> instance().newBuffPara.singleIterations;
            buffNode["binMatIterations"] >> instance().newBuffPara.binMatIterations;
            buffNode["armorSideRatio_MAX"] >> instance().newBuffPara.armorSideRatio_MAX;
            buffNode["armorSideRatio_MIN"] >> instance().newBuffPara.armorSideRatio_MIN;
            buffNode["armorTo_R_SizeRatio_MAX"] >> instance().newBuffPara.armorTo_R_SizeRatio_MAX;
            buffNode["armorTo_R_SizeRatio_MIN"] >> instance().newBuffPara.armorTo_R_SizeRatio_MIN;
            buffNode["armorTo_R_DistanceRatio_MAX"] >> instance().newBuffPara.armorTo_R_DistanceRatio_MAX;
            buffNode["armorTo_R_DistanceRatio_MIN"] >> instance().newBuffPara.armorTo_R_DistanceRatio_MIN;
            buffNode["longSideRatioForROI"] >> instance().newBuffPara.longSideRatioForROI;
            buffNode["shortSideRatioForROI"] >> instance().newBuffPara.shortSideRatioForROI;
            buffNode["targetSizeRadio"] >> instance().newBuffPara.targetSizeRadio;
//            LOG::debug("NewBuffPara.grayThresholdValue_R=" + std::to_string(instance().newBuffPara.grayThresholdValue));
//            LOG::debug("NewBuffPara.channelThresholdValue_R=" + std::to_string(instance().newBuffPara.channelThresholdValue));
//            LOG::debug("NewBuffPara.singleIterations=" + std::to_string(instance().newBuffPara.singleIterations));
//            LOG::debug("NewBuffPara.binMatIterations=" + std::to_string(instance().newBuffPara.binMatIterations));
//            LOG::debug("NewBuffPara.armorSideRatio_MAX=" + std::to_string(instance().newBuffPara.armorSideRatio_MAX));
//            LOG::debug("NewBuffPara.armorSideRatio_MIN=" + std::to_string(instance().newBuffPara.armorSideRatio_MIN));
//            LOG::debug("NewBuffPara.armorTo_R_SizeRatio_MAX=" + std::to_string(instance().newBuffPara.armorTo_R_SizeRatio_MAX));
//            LOG::debug("NewBuffPara.armorTo_R_SizeRatio_MIN=" + std::to_string(instance().newBuffPara.armorTo_R_SizeRatio_MIN));
//            LOG::debug("NewBuffPara.armorTo_R_DistanceRatio_MAX=" + std::to_string(instance().newBuffPara.armorTo_R_DistanceRatio_MAX));
//            LOG::debug("NewBuffPara.armorTo_R_DistanceRatio_MIN=" + std::to_string(instance().newBuffPara.armorTo_R_DistanceRatio_MIN));
//            LOG::debug("NewBuffPara.longSideRatioForROI=" + std::to_string(instance().newBuffPara.longSideRatioForROI));
//            LOG::debug("NewBuffPara.shortSideRatioForROI=" + std::to_string(instance().newBuffPara.shortSideRatioForROI));
//            LOG::debug("NewBuffPara.targetSizeRadio=" + std::to_string(instance().newBuffPara.targetSizeRadio));


            cv::FileNode R_Node = fs["R_Para"];
            R_Node["squreSideRatio"] >> instance().r_Para.squreSideRatio;
            R_Node["squreSizeLimit_MAX"] >> instance().r_Para.squreSizeLimit_MAX;
            R_Node["squreSizeLimit_MIN"] >> instance().r_Para.squreSizeLimit_MIN;
//            LOG::debug("R_Para.squreSideRatio=" + std::to_string(instance().r_Para.squreSideRatio));
//            LOG::debug("R_Para.squreSizeLimit_MAX=" + std::to_string(instance().r_Para.squreSizeLimit_MAX));
//            LOG::debug("R_Para.squreSizeLimit_MIN=" + std::to_string(instance().r_Para.squreSizeLimit_MIN));

        }
    }
    static void writeNewBuffPara(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //д?????
        fs.writeComment("?°????????");
        //д????
        fs << "NewBuffPara" << "{";
        fs.writeComment("???????");
        fs << "grayThresholdValue" << instance().newBuffPara.grayThresholdValue;
        fs.writeComment("???????");
        fs << "channelThresholdValue" << instance().newBuffPara.channelThresholdValue;
        fs.writeComment("?????????????????");
        fs << "singleIterations" << instance().newBuffPara.singleIterations;
        fs.writeComment("???????????????");
        fs << "binMatIterations" << instance().newBuffPara.binMatIterations;
        fs.writeComment("???峤???????");
        fs << "armorSideRatio_MAX" << instance().newBuffPara.armorSideRatio_MAX;
        fs.writeComment("???峤???????");
        fs << "armorSideRatio_MIN" << instance().newBuffPara.armorSideRatio_MIN;
        fs.writeComment("??????????");
        fs << "armorTo_R_SizeRatio_MAX" << instance().newBuffPara.armorTo_R_SizeRatio_MAX;
        fs.writeComment("??????????");
        fs << "armorTo_R_SizeRatio_MIN" << instance().newBuffPara.armorTo_R_SizeRatio_MIN;
        fs.writeComment("??????R???????????");
        fs << "armorTo_R_DistanceRatio_MAX" << instance().newBuffPara.armorTo_R_DistanceRatio_MAX;
        fs.writeComment("??????R???????????");
        fs << "armorTo_R_DistanceRatio_MIN" << instance().newBuffPara.armorTo_R_DistanceRatio_MIN;
        fs.writeComment("????ROI????");
        fs << "longSideRatioForROI" << instance().newBuffPara.longSideRatioForROI;
        fs.writeComment("???ROI????");
        fs << "shortSideRatioForROI" << instance().newBuffPara.shortSideRatioForROI;
        fs.writeComment("?????????С????");
        fs << "targetSizeRadio" << instance().newBuffPara.targetSizeRadio;
        fs << "}";

        fs << "R_Para" << "{";
        fs.writeComment("???????γ????????");
        fs << "squreSideRatio" << instance().r_Para.squreSideRatio;
        fs.writeComment("???????????????");
        fs << "squreSizeLimit_MAX" << instance().r_Para.squreSizeLimit_MAX;
        fs.writeComment("???????????????");
        fs << "squreSizeLimit_MIN" << instance().r_Para.squreSizeLimit_MIN;
        fs << "}";
        //????
        fs.release();
    }
public:
    NewBuffPara newBuffPara;
    R_Para r_Para;
};

class CameraConfigurationFactory {
private:
    static CameraConfigurationFactory &instance() {
        static CameraConfigurationFactory cameraConfigurationFactory;
        return cameraConfigurationFactory;
    }

    /**
     * ???????????д??????????????
     * @param cameraConfiguration ??д?????
     * @param name ???????
     * @param comment ??д??????
     * @param fs ??????????
     */
    static void writeCameraConfigurationStruct(CameraConfiguration &cameraConfiguration, const char *name, const char *comment, cv::FileStorage &fs) {
        fs.writeComment("\n");
        fs.writeComment(comment);
        fs << name << "{"
           << "gainValue" << cameraConfiguration.gainValue
           << "exposureValue" << cameraConfiguration.exposureValue
           << "gammaValue" << cameraConfiguration.gammaValue
           << "}";
    }

    /**
     * ??????????ж???????????????
     * @param cameraConfiguration ?????????
     * @param name ???????
     * @param fs ??????????
     */
    static void readCameraConfigurationStruct(CameraConfiguration &cameraConfiguration, const char *name, cv::FileStorage &fs) {
        cv::FileNode node = fs[name];
        node["gammaValue"] >> cameraConfiguration.gammaValue;
        node["exposureValue"] >> cameraConfiguration.exposureValue;
        node["gainValue"] >> cameraConfiguration.gainValue;
//        LOG::debug(LOG::tostring(name) + ".gammaValue=" + std::to_string(cameraConfiguration.gammaValue));
//        LOG::debug(LOG::tostring(name) + ".exposureValue=" + std::to_string(cameraConfiguration.exposureValue));
//        LOG::debug(LOG::tostring(name) + ".gainValue=" + std::to_string(cameraConfiguration.gainValue));
    }

public:
    static CameraConfiguration getArmorCameraConfiguration() {
        return instance().armorCameraConfiguration;
    }

    static CameraConfiguration getBuffCameraConfiguration() {
        return instance().buffCameraConfiguration;
    }

    static void resetAllConfig() {
        instance().armorCameraConfiguration = CameraConfiguration();
        instance().buffCameraConfiguration = CameraConfiguration();
    }

    static void readCameraConfiguration(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeCameraConfiguration(filename);
            cout<<"default config not find, write config to " + filename<<endl;
        } else {
            readCameraConfigurationStruct(instance().armorCameraConfiguration, "ArmorCameraConfiguration", fs);
            readCameraConfigurationStruct(instance().buffCameraConfiguration, "BuffCameraConfiguration", fs);
        }
        fs.release();
    }

    static void writeCameraConfiguration(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //д?????
        fs.writeComment("???????");
        writeCameraConfigurationStruct(instance().armorCameraConfiguration, "ArmorCameraConfiguration", "??????????????", fs);
        writeCameraConfigurationStruct(instance().buffCameraConfiguration, "BuffCameraConfiguration", "?????????????", fs);
        //????
        fs.release();
    }

public:

    CameraConfiguration armorCameraConfiguration;
    CameraConfiguration buffCameraConfiguration;
};

class CodeSet {
private:
    bool useDebugGuiFlag = false;
    bool takePictureFlag = false;
    int grayAverage = 20;
    bool useAutoSaveSampleMode = 0;  //0:?????? 1:???????    ???:0

    static CodeSet &instance() {
        static CodeSet codeSet;
        return codeSet;
    }

public:
    static void resetAllConfig() {
        instance() = CodeSet();
    }

    static void readCodeSet(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeCodeSet(filename);
            cout<<"default config not find, write config to " + filename<<endl;
        } else {
            fs["takePictureFlag"] >> instance().takePictureFlag;
            fs["useDebugGuiFlag"] >> instance().useDebugGuiFlag;
            fs["grayAverage"] >> instance().grayAverage;
            fs["useAutoSaveSampleMode"] >> instance().useAutoSaveSampleMode;

//            LOG::debug("codeSet.takePictureFlag=" + std::to_string(instance().takePictureFlag));
//            LOG::debug("codeSet.useDebugGuiFlag=" + std::to_string(instance().useDebugGuiFlag));
//            LOG::debug("codeSet.grayAverage=" + std::to_string(instance().grayAverage));
//            LOG::debug("codeSet.useAutoSaveSampleMode=" + std::to_string(instance().useAutoSaveSampleMode));
        }
        fs.release();
    }

    static void writeCodeSet(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //д?????
        fs.writeComment("????????\n");
        fs.writeComment("Linux????????");
        fs << "useDebugGuiFlag" << instance().useDebugGuiFlag;
        fs.writeComment("????????????");
        fs << "takePictureFlag" << instance().takePictureFlag;
        fs.writeComment("?????????????");
        fs << "grayAverage" << instance().grayAverage;
        fs.writeComment("0:?????? 1:???????  ???:0");
        fs << "useAutoSaveSampleMode" << instance().useAutoSaveSampleMode;
        //????
        fs.release();
    }

    static bool getUseDebugGuiFlag() {
        return instance().useDebugGuiFlag;
    }

    static bool getTakePictureFlag() {
        return instance().takePictureFlag;
    }

    static int getGrayAverage() {
        return instance().grayAverage;
    }

    static SaveSampleMode getUseAutoSaveSampleMode() {
        return (SaveSampleMode) instance().useAutoSaveSampleMode;
    }
};

class InstallDataFactory {
private:
    static InstallDataFactory &instance() {
        static InstallDataFactory installDataFactory;
        return installDataFactory;
    }

    /**
     * ??????????????
     * @param worldOriginInPZT ?????????????????????λ??(???????????)
     * @param worldOriginInBarrel ?????????????????????λ??(??????????)
     * @param worldOriginInCamera ??????????????????λ??(???????)
     * @param installData ???????
     */
    static void calculateRealCoordinate(cv::Point3f &worldOriginInPZT, cv::Point3f &worldOriginInBarrel, cv::Point3f &worldOriginInCamera, const InstallData &installData) {
        //????????????????λ??(???????????)

//        worldOriginInPZT.z = worldOriginInCamera.z - installData.PTZToCameraBias.z;
//        worldOriginInPZT.x = worldOriginInCamera.x - installData.PTZToCameraBias.x;
//        worldOriginInPZT.y = worldOriginInCamera.y - installData.PTZToCameraBias.y;
        worldOriginInPZT.x = worldOriginInCamera.x - 0.0;
        worldOriginInPZT.y = worldOriginInCamera.y - (5.5);
        worldOriginInPZT.z = worldOriginInCamera.z - (-100.0);

        //?????????????????????λ??(???????????)
//        worldOriginInBarrel.z = worldOriginInPZT.z - installData.BarrelToCameraBias.z;
//        worldOriginInBarrel.x = worldOriginInPZT.x - installData.BarrelToCameraBias.x;
//        worldOriginInBarrel.y = worldOriginInPZT.y - installData.BarrelToCameraBias.y;

        worldOriginInBarrel.x = worldOriginInPZT.x - 0;
        worldOriginInBarrel.y = worldOriginInPZT.y - 0;
        worldOriginInBarrel.z = worldOriginInPZT.z - (40.0);

//        cout << "worldOriginInCamera: "<<worldOriginInCamera.x<<" "<<worldOriginInCamera.y<<" "<<worldOriginInCamera.z<< endl;
//        cout << "worldOriginInPZT: "<<worldOriginInPZT.x<<" "<<worldOriginInPZT.y<<" "<<worldOriginInPZT.z<< endl;
//        cout << "worldOriginInBarrel: "<<worldOriginInBarrel.x<<" "<<worldOriginInBarrel.y<<" "<<worldOriginInBarrel.z<< endl;
        DebugT(1, 8,"相机坐标系 X:" << worldOriginInCamera.x << " Y:"<<worldOriginInCamera.y <<" Z: "<< worldOriginInCamera.z )
        DebugT(1, 9,"云台坐标系 X:" << worldOriginInPZT.x << " Y:"<<worldOriginInPZT.y <<" Z: "<< worldOriginInPZT.z )
        DebugT(1, 10,"枪口坐标系 X:" << worldOriginInBarrel.x << " Y:"<<worldOriginInBarrel.y <<" Z: "<< worldOriginInBarrel.z )

    }
    /**
     * ???????????д???????????????
     * @param installData ??д?????
     * @param name ???????
     * @param comment ??д??????
     * @param fs ??????????
     */
    static void writeInstallDataStruct(InstallData &installData, const char *name, const char *comment, cv::FileStorage &fs) {
        fs.writeComment("\n");
        fs.writeComment(comment);
        fs << name << "{";
        fs.writeComment("???????????????????????????");
        fs << "PTZToCameraBias_x" << installData.PTZToCameraBias.x
           << "PTZToCameraBias_y" << installData.PTZToCameraBias.y
           << "PTZToCameraBias_z" << installData.PTZToCameraBias.z;
        fs.writeComment("????????????????????????????????");
        fs << "PTZToBarrelBias_x" << installData.PTZToBarrelBias.x
           << "PTZToBarrelBias_y" << installData.PTZToBarrelBias.y
           << "PTZToBarrelBias_z" << installData.PTZToBarrelBias.z
           << "}";
    }

    /**
     * ??????????ж????????????????
     * @param installData ?????????
     * @param name ???????
     * @param fs ??????????
     */
    static void readInstallDataStruct(InstallData &installData, const char *name, cv::FileStorage &fs) {
        cv::FileNode node = fs[name];
        node["PTZToCameraBias_x"] >> installData.PTZToCameraBias.x;
        node["PTZToCameraBias_y"] >> installData.PTZToCameraBias.y;
        node["PTZToCameraBias_z"] >> installData.PTZToCameraBias.z;
        node["PTZToBarrelBias_x"] >> installData.PTZToBarrelBias.x;
        node["PTZToBarrelBias_y"] >> installData.PTZToBarrelBias.y;
        node["PTZToBarrelBias_z"] >> installData.PTZToBarrelBias.z;
//        LOG::debug(LOG::tostring(name) + ".PTZToCameraBias.x=" + std::to_string(installData.PTZToCameraBias.x));
//        LOG::debug(LOG::tostring(name) + ".PTZToCameraBias.y=" + std::to_string(installData.PTZToCameraBias.y));
//        LOG::debug(LOG::tostring(name) + ".PTZToCameraBias.z=" + std::to_string(installData.PTZToCameraBias.z));
//        LOG::debug(LOG::tostring(name) + ".PTZToBarrelBias.x=" + std::to_string(installData.PTZToBarrelBias.x));
//        LOG::debug(LOG::tostring(name) + ".PTZToBarrelBias.y=" + std::to_string(installData.PTZToBarrelBias.y));
//        LOG::debug(LOG::tostring(name) + ".PTZToBarrelBias.z=" + std::to_string(installData.PTZToBarrelBias.z));
        //?????????????????????????????????
        installData.BarrelToCameraBias.x = installData.PTZToCameraBias.x - installData.PTZToBarrelBias.x;
        installData.BarrelToCameraBias.y = installData.PTZToCameraBias.y - installData.PTZToBarrelBias.y;
        installData.BarrelToCameraBias.z = installData.PTZToCameraBias.z - installData.PTZToBarrelBias.z;
    }

    //????????????
    static void initInstallData() {
        //PS:??????????????????????????
        //????
        instance().infantry.PTZToCameraBias.x = 0.0f;
        instance().infantry.PTZToCameraBias.y = 6.0f;
        instance().infantry.PTZToCameraBias.z = -10.0f;
        instance().infantry.PTZToBarrelBias.x = 0;
        instance().infantry.PTZToBarrelBias.y = 0;
        instance().infantry.PTZToBarrelBias.z = -66.6f;
        //?????
        instance().oldHero.PTZToCameraBias.x = 0;
        instance().oldHero.PTZToCameraBias.y = 48.5f;
        instance().oldHero.PTZToCameraBias.z = -182.51f;
        instance().oldHero.PTZToBarrelBias.x = 0;
        instance().oldHero.PTZToBarrelBias.y = 0;
        instance().oldHero.PTZToBarrelBias.z = -189.94f;
        //?????
        instance().newHero.PTZToCameraBias.x = -71.88f;
        instance().newHero.PTZToCameraBias.y = 69.5f;
        instance().newHero.PTZToCameraBias.z = -127.41f;
        instance().newHero.PTZToBarrelBias.x = 0;
        instance().newHero.PTZToBarrelBias.y = 0;
        instance().newHero.PTZToBarrelBias.z = -114.76f;
        //??????????
        instance().oldSentinelAbove.PTZToCameraBias.x = 0;
        instance().oldSentinelAbove.PTZToCameraBias.y = 41.4f;
        instance().oldSentinelAbove.PTZToCameraBias.z = 43.26f;
        instance().oldSentinelAbove.PTZToBarrelBias.x = 0;
        instance().oldSentinelAbove.PTZToBarrelBias.y = 0;
        instance().oldSentinelAbove.PTZToBarrelBias.z = -14.04f;
        //??????????
        instance().oldSentinelBelow.PTZToCameraBias.x = 0;
        instance().oldSentinelBelow.PTZToCameraBias.y = 41.4f;
        instance().oldSentinelBelow.PTZToCameraBias.z = 178.0f;
        instance().oldSentinelBelow.PTZToBarrelBias.x = 0;
        instance().oldSentinelBelow.PTZToBarrelBias.y = 0;
        instance().oldSentinelBelow.PTZToBarrelBias.z = 88.0f;
        //??????????
        instance().newSentinelAbove.PTZToCameraBias.x = -56.5f;
        instance().newSentinelAbove.PTZToCameraBias.y = 43.50f;
        instance().newSentinelAbove.PTZToCameraBias.z = -38.46f;
        instance().newSentinelAbove.PTZToBarrelBias.x = 0;
        instance().newSentinelAbove.PTZToBarrelBias.y = 0;
        instance().newSentinelAbove.PTZToBarrelBias.z = -75.07f;
        //??????????
        instance().newSentinelBelow.PTZToCameraBias.x = -56.50f;
        instance().newSentinelBelow.PTZToCameraBias.y = 43.50f;
        instance().newSentinelBelow.PTZToCameraBias.z = -123.60f;
        instance().newSentinelBelow.PTZToBarrelBias.x = 0;
        instance().newSentinelBelow.PTZToBarrelBias.y = 0;
        instance().newSentinelBelow.PTZToBarrelBias.z = -183.41f;
        //???
        instance().plane.PTZToCameraBias.x = 0;
        instance().plane.PTZToCameraBias.y = 65.3f;
        instance().plane.PTZToCameraBias.z = -119.65f;
        instance().plane.PTZToBarrelBias.x = 0;
        instance().plane.PTZToBarrelBias.y = 0.13f;
        instance().plane.PTZToBarrelBias.z = -78.00f;
    }

public:
    static void resetAllConfig() {
        instance().initInstallData();
    }

    //???????????????????
    static void readInstallData(const std::string &filename) {
        //??????????????
        initInstallData();

        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeInstallData(filename);
            cout<<"default config not find, write config to " + filename<<endl;
        } else {
            readInstallDataStruct(instance().infantry, "Infantry", fs);
            readInstallDataStruct(instance().oldHero, "OldHero", fs);
            readInstallDataStruct(instance().newHero, "NewHero", fs);
            readInstallDataStruct(instance().oldSentinelBelow, "OldSentinelBelow", fs);
            readInstallDataStruct(instance().oldSentinelAbove, "OldSentinelAbove", fs);
            readInstallDataStruct(instance().newSentinelBelow, "NewSentinelBelow", fs);
            readInstallDataStruct(instance().newSentinelAbove, "NewSentinelAbove", fs);
            readInstallDataStruct(instance().plane, "Plane", fs);
        }
        fs.release();
    }

    //д????????????????
    static void writeInstallData(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        writeInstallDataStruct(instance().infantry, "Infantry", "???????????", fs);
        writeInstallDataStruct(instance().oldHero, "OldHero", "???????????", fs);
        writeInstallDataStruct(instance().newHero, "NewHero", "???????????", fs);
        writeInstallDataStruct(instance().oldSentinelAbove, "OldSentinelBelow", "?????????????????", fs);
        writeInstallDataStruct(instance().oldSentinelBelow, "OldSentinelAbove", "?????????????????", fs);
        writeInstallDataStruct(instance().newSentinelAbove, "NewSentinelBelow", "?????????????????", fs);
        writeInstallDataStruct(instance().newSentinelBelow, "NewSentinelAbove", "?????????????????", fs);
        writeInstallDataStruct(instance().plane, "Plane", "??????????", fs);
        //????
        fs.release();

    }

    //???????????????????????????????????????
    static void getRealCoordinateDataByCarType(cv::Point3f &worldOriginInPZT, cv::Point3f &worldOriginInBarrel, cv::Point3f &worldOriginInCamera, CarType carType) {
        switch (carType) {
            case OLD_HERO:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().oldHero);
                break;
            case NEW_HERO:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().newHero);
                break;
            case NEW_SENTRY_BELOW:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().newSentinelBelow);
                break;
            case NEW_SENTRY_ABOVE:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().newSentinelAbove);
                break;
            case OLD_SENTRY_BELOW:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().oldSentinelBelow);
                break;
            case OLD_SENTRY_ABOVE:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().oldSentinelAbove);
                break;
            case PLANE:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().plane);
                break;
            default :
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().infantry);
                break;
        }
    }

public:
    InstallData infantry;                //????
    InstallData oldHero;                //?????
    InstallData newHero;                //?????
    InstallData oldSentinelAbove;        //??????????
    InstallData oldSentinelBelow;        //??????????
    InstallData newSentinelAbove;        //??????????
    InstallData newSentinelBelow;        //??????????
    InstallData plane;                    //???
};

#endif // !_CONF_H_
