#include <options.h>
#include <log.h>
#include <cstring>
#include <map>

bool show_armor_box = false;   // 显示装甲板图
bool show_armor_boxes = false; // 显示所有装甲板图
bool show_light_blobs = false; // 显示灯条图
bool show_origin = false;      // 显示源图【暂时不用】
bool run_with_camera = false;  // 图像源仅使用相机
bool save_video = false;       // 录制视频
bool debug_camera = false;     // 调试相机
bool wait_uart = false;        // 串口检测
bool save_labelled_boxes = false;  // 数据集采集模式
bool show_process = false;     // 显示能量机关图
bool show_energy = false;      // 显示能量机关
bool save_mark = false;        // 保存能量机关标志点
bool show_info = false;        // 仅终端显示目标信息
bool run_by_frame = false;     // 图像源仅使用视频流
bool recv_close = true; // 无法获取裁判系统颜色时，手动设置目标装甲板颜色 【true:红色 false：蓝色】

// 使用map保存所有选项及其描述和操作，加快查找速度。
std::map<std::string, std::pair<std::string, void(*)(void)>> options = {
    {"--help",{
        "show the help information.", [](){
            LOG(LOG_MSG, "<HELP>: " STR_CTR(WORD_BLUE, "All options below are for debug use."));
            for(const auto &option : options){
                LOG(LOG_MSG, "<HELP>: " STR_CTR(WORD_GREEN, "%s: %s"), option.first.data(), option.second.first.data());
            }
        }
    }},
    {"--show-armor-box", {
        "show the aim box.", []() {
            show_armor_box = true;
            LOGM("Enable show armor box");
        }
    }},
    {"--show-armor-boxes",{
        "show the candidate aim boxes.", [](){
            show_armor_boxes = true;
            LOGM("Enable show armor boxes");
        }
    }},
    {"--show-light-blobs",{
        "show the candidate light blobs.", [](){
            show_light_blobs = true;
            LOGM("Enable show light blobs");
        }
    }},
    {"--show-origin", {
        "show the origin image.", [](){
            show_origin = true;
            LOGM("Enable show origin");
        }
    }},
    {"--run-with-camera", {
        "start the program with camera directly without asking.", []() {
            run_with_camera = true;
            LOGM("Run with camera!");
        }
    }},
    {"--save-video", {
        "save the video.", [](){
            save_video = true;
            LOGM("Enable save video!");
        }
    }},
    {"--save-labelled-boxes",{
        "save the candidate boxes with their id labels.", [](){
            save_labelled_boxes = true;
            LOGM("labelled armor boxes will be saved!");
        }
    }},
    {"--wait-uart", {
        "wait uart until ready before running.", [](){
            wait_uart = true;
            LOGM("Enable wait uart!");
        }
    }},
    {"--run-by-frame",{
        "run the code frame by frame.(normally used when run video)", [](){
            run_by_frame = true;
            LOGM("Enable run frame by frame");
        }
    }},
    {"--show-process", {
        "", [](){
            show_process = true;
            LOGM("Enable show processed image!");
        }
    }},
    {"--show-energy", {
        "",[](){
            show_energy = true;
            LOGM("Enable show energy part!");
        }
    }},
    {"--save-mark", {
        "", [](){
            save_mark = true;
            LOGM("Write down mark");
        }
    }},
    {"--show-info", {
        "", [](){
            show_info = true;
            LOGM("Show information!");
        }
    }},
    {"--show-all", {
        "show all image windows.", [](){
            show_armor_box = true;
            LOGM("Enable show armor box");
            show_armor_boxes = true;
            LOGM("Enable show armor boxes");
            show_light_blobs = true;
            LOGM("Enable show light blobs");
            show_origin = true;
            LOGM("Enable show origin");
            show_process = true;
            LOGM("Enable show processed image");
            show_energy = true;
            LOGM("Enable show energy part");
        }
    }}
};

void processOptions(int argc, char **argv) {
    if (argc >= 2) {
        for (int i = 1; i < argc; i++) {
            auto key = options.find(std::string(argv[i])); // 寻找对应选项。
            if(key != options.end()){
                key->second.second();
            }else{
                LOGW("Unknown option: %s. Use --help to see options.", argv[i]);
            }
        }
    }
}