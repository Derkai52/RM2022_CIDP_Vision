#include <armor_finder/armor_finder.h>
#include <show_images/show_images.h>
#include <options.h>
#include <opencv2/highgui.hpp>

#define DO_NOT_CNT_TIME
#include <log.h>
#include <opencv2/dnn/dnn.hpp>
#include <fstream>

// 判断两个灯条的角度差
static bool angelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    return abs(angle_i - angle_j) < 20;
}

// 判断两个灯条的高度差
static bool heightJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    return abs(centers.y) < 30;
}

// 判断两个灯条的间距
static bool lengthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    return (side_length / light_blob_i.length < 10 && side_length / light_blob_i.length > 0.5);
}

// 判断两个灯条的长度比
static bool lengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    return (light_blob_i.length / light_blob_j.length < 2.7
            && light_blob_i.length / light_blob_j.length > 0.4);
}

// 判断两个灯条的错位度
static bool CuoWeiDuJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
    if (abs(angle_i - angle_j) > 90) {
        angle += 3.14159265459 / 2;
    }
    Vector2f orientation(cos(angle), sin(angle));
    Vector2f p2p(light_blob_j.rect.center.x - light_blob_i.rect.center.x,
                 light_blob_j.rect.center.y - light_blob_i.rect.center.y);
    return abs(orientation.dot(p2p)) < 25;
}

// 判断装甲板方向
static bool boxAngleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0;
    if (abs(angle_i - angle_j) > 90) {
        angle += 90.0;
    }
    return (-120.0 < angle && angle < -60.0) || (60.0 < angle && angle < 120.0);
}

// 判断两个灯条是否可以匹配
static bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, uint8_t enemy_color) {
    return light_blob_i.blob_color == enemy_color &&
           light_blob_j.blob_color == enemy_color &&
           lengthRatioJudge(light_blob_i, light_blob_j) &&
           lengthJudge(light_blob_i, light_blob_j) &&
           //           heightJudge(light_blob_i, light_blob_j) &&
           angelJudge(light_blob_i, light_blob_j) &&
           boxAngleJudge(light_blob_i, light_blob_j) &&
           CuoWeiDuJudge(light_blob_i, light_blob_j);

}

// 匹配所有灯条，得出装甲板候选区
bool ArmorFinder::matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes) {
    armor_boxes.clear();
    for (int i = 0; i < light_blobs.size() - 1; ++i) {
        for (int j = i + 1; j < light_blobs.size(); ++j) {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j), enemy_color)) {
                continue;
            }
            cv::Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(i)).rect.boundingRect();
            cv::Rect2d rect_right = light_blobs.at(static_cast<unsigned long>(j)).rect.boundingRect();
            double min_x, min_y, max_x, max_y;
            min_x = fmin(rect_left.x, rect_right.x) - 4;
            max_x = fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
            min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
            max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
                    0.5 * (rect_left.height + rect_right.height) / 2.0;
            if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows) {
                continue;
            }
            if (state == SEARCHING_STATE && (max_y + min_y) / 2 < 1) continue;  // 值越小,检测范围上边界越往上
            if ((max_x - min_x) / (max_y - min_y) < 0.8) continue;
            LightBlobs pair_blobs = {light_blobs.at(i), light_blobs.at(j)};
            armor_boxes.emplace_back(
                    cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y),
                    pair_blobs,
                    enemy_color
            );
        }
    }
    return !armor_boxes.empty();
}

void ArmorFinder::InitClass(const std::string & model_path, const std::string & label_path, const double thre)
{
    net_ = cv::dnn::readNetFromONNX(model_path);

    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line)) {
        class_names_.push_back(line[0]);
    }
}



int ArmorFinder::doClassify(cv::Mat &image)
{
        // Normalize
        image = image / 255.0;

        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1., cv::Size(28, 20));

        // Set the input blob for the neural network
        net_.setInput(blob);
        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward();

        // Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;
        return label_id;
//        armor.number = class_names_[label_id];
//
//        std::stringstream result_ss;
//        result_ss << armor.number << ":_" << std::fixed << std::setprecision(1)
//                  << armor.confidence * 100.0 << "%";
//        armor.classfication_result = result_ss.str();
//
//    armors.erase(
//            std::remove_if(
//                    armors.begin(), armors.end(),
//                    [this](const Armor & armor) {
//                        if (armor.confidence < threshold || armor.number == 'N') {
//                            return true;
//                        }
//
//                        bool mismatch = false;
//                        if (armor.armor_type == LARGE) {
//                            mismatch = armor.number == 'O' || armor.number == '2' || armor.number == '3' ||
//                                       armor.number == '4' || armor.number == '5';
//                        } else if (armor.armor_type == SMALL) {
//                            mismatch = armor.number == '1' || armor.number == 'B' || armor.number == 'G';
//                        }
//                        return mismatch;
//                    }),
//            armors.end());
}

// 在给定的图像上寻找装甲板
bool ArmorFinder::findArmorBox(const cv::Mat &src, ArmorBox &box) {
    LightBlobs light_blobs; // 存储所有可能的灯条
    ArmorBoxes armor_boxes; // 装甲板候选区

    box.rect = cv::Rect2d(0, 0, 0, 0);
    box.id = -1;
// 寻找所有可能的灯条
    CNT_TIME("blob", {
        if (!findLightBlobs(src, light_blobs)) {
            return false;
        }
    });

    if (show_light_blobs && state==SEARCHING_STATE) {
        showLightBlobs("light_blobs", src, light_blobs);
        cv::waitKey(1);
    }
// 对灯条进行匹配得出装甲板候选区
    CNT_TIME("boxes", {
        if (!matchArmorBoxes(src, light_blobs, armor_boxes)) {
            return false;
        }
    });
    if (show_armor_boxes && state==SEARCHING_STATE) {
        showArmorBoxes("boxes", src, armor_boxes);
        cv::waitKey(1);
    }
// 如果分类器可用，则使用分类器对装甲板候选区进行筛选
    if (classifier) {
        const int light_length = 12;
        const int warp_height = 28;
        const int small_armor_width = 32;

        for (auto &armor_box : armor_boxes) {
            armor_box.getFourPoint(armor_box);
            cv::Point lb = cv::Point(armor_box.four_point[1]); // 左下
            cv::Point lt = cv::Point(armor_box.four_point[0]); // 左上
            cv::Point rt = cv::Point(armor_box.four_point[3]); // 右上
            cv::Point rb = cv::Point(armor_box.four_point[2]); // 右下
            // Warp perspective transform

            cv::Point2f lights_vertices[4] = {rb,rt,lt,lb};

            const int top_light_y = (warp_height - light_length) / 2 - 1;
            const int bottom_light_y = top_light_y + light_length;
            const int warp_width = small_armor_width;
            cv::Point2f target_vertices[4] = {
                    cv::Point(0, bottom_light_y),
                    cv::Point(0, top_light_y),
                    cv::Point(warp_width - 1, top_light_y),
                    cv::Point(warp_width - 1, bottom_light_y),
            };
            cv::Mat number_image;
            auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
            cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
            cv::resize(number_image, number_image, cv::Size(48, 36));
            cv::imshow("warp", number_image);

            int c = classifier(number_image);
            if (c == RED2 || c == BLUE2){ // TODO : Warning  这里强行不识别工程
                c = 0;
            }
            armor_box.id = c;
        }

        // 按照优先级对装甲板进行排序
        sort(armor_boxes.begin(), armor_boxes.end(), [&](const ArmorBox &a, const ArmorBox &b) {
            if (last_box.rect != cv::Rect2d()) {
                return getPointLength(a.getCenter() - last_box.getCenter()) <
                       getPointLength(b.getCenter() - last_box.getCenter());
            } else {
                return a < b;
            }
        });
        for (auto &one_box : armor_boxes) {
            if (one_box.id != 0) {
                box = one_box;
                break;
            }
        }

        if (box.rect == cv::Rect2d(0, 0, 0, 0)) {
            return false;
        }
        if (show_armor_boxes && state==SEARCHING_STATE) {
            showArmorBoxesClass("class", src, armor_boxes);
        }
    }
    else { // 如果分类器不可用，则直接选取候选区中的第一个区域作为目标(往往会误识别)
        box = armor_boxes[0];
    }

    // 保存有效的装甲板图作为标签(需要去others/src/options.cpp 修改设置开启)
    if (save_labelled_boxes) {
        for (const auto &one_box : armor_boxes) {
            char filename[100];
//                sprintf(filename, PROJECT_DIR"/armor_save_box/%s_%d.jpg", id2name[one_box.id].data(),
//                        time(nullptr) + clock());
            sprintf(filename, PROJECT_DIR"/armor_save_box/NO/%d.jpg", time(nullptr) + clock());

            auto box_roi = src(one_box.rect);
            cv::resize(box_roi, box_roi, cv::Size(48, 36));
            cv::imwrite(filename, box_roi);
        }
    }
    return true;
}

// 获取装甲板四点（用于PNP解算）
void ArmorBox::getFourPoint(ArmorBox &box) {
    cv::Point2f vertices[4], pt[4], st[4];
    int _temp = 0; // 计数器
    for(auto &light_blob :box.light_blobs){
        light_blob.rect.points(vertices);  // rRect.points有顺序的，y最小的点是0,顺时针1 2 3
        float anglei = fabs(light_blob.rect.angle);
        // 往右斜的长灯条
        if(anglei >= 45.0){
            pt[0] = vertices[3];
            pt[1] = vertices[0];
            pt[2] = vertices[2];
            pt[3] = vertices[1];
        }
        // 往左斜的长灯条
        else{
            pt[0] = vertices[2];
            pt[1] = vertices[3];
            pt[2] = vertices[1];
            pt[3] = vertices[0];
        }
        cv::Point2f _blob_top = cv::Point2i((pt[0].x + pt[2].x)/2 , (pt[0].y + pt[2].y)/2);    // 灯条顶部中心
        cv::Point2f _blob_bottom = cv::Point2i((pt[1].x + pt[3].x)/2 , (pt[1].y + pt[3].y)/2); // 灯条底部中心
        st[_temp] = _blob_top;
        st[_temp+1] = _blob_bottom;
        _temp += 2;
    }
    if(st[0].x >= st[3].x) { // Warning: 因为横着的装甲板不被识别，所以这里用了比较偷懒的做法，直接比较灯条顶点的x值大小来决定灯条的左右顺序
        four_point = {st[0], st[1], st[3], st[2]};
    }
    else{
        four_point = {st[2], st[3], st[1], st[0]};
    }
}


