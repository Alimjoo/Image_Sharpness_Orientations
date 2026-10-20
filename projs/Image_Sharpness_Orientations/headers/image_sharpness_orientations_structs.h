#pragma once
#ifndef IMAGE_SHARPNESS_ORIENTATIONS_STRUCTS_H
#define IMAGE_SHARPNESS_ORIENTATIONS_STRUCTS_H

#include <opencv2/core.hpp>
#include <vector>

// 与 Alg_CH 保持相同的图像、点和轮廓类型。
using CHImage = cv::Mat;
using CHPointF = cv::Point2f;
using CHContour = std::vector<CHPointF>;
using CHRect = cv::Rect;

enum Image_Sharpness_Orientations_Error_Codes {
    ISO_OK = 0,                   // 成功；检测是否全部匹配请检查 Detect_Result::all_matched
    ISO_ImageIsEmpty = 3,         // 输入图像为空
    ISO_InvalidParameter = 5,     // 参数错误
    ISO_UnsupportedImage = 6,     // 仅支持二维 8UC1 / 8UC3(BGR) / 8UC4(BGRA)
    ISO_BaseNotRegistered = 7,    // 尚未注册基准
    ISO_NoShapes = 8,             // 没有找到符合条件的图形
    ISO_Fail = -1                 // 处理失败
};

struct Image_Sharpness_Orientations_Params {
    bool detect_dark_area = true; // true: 检测亮底上的暗色图形；false: 检测暗底上的白色/亮色图形
    int threshold = -1;           // -1: Otsu自动阈值；0~255: 固定阈值，暗色<=阈值，亮色>阈值
    double min_area = 100.0;      // 最小外轮廓面积，像素平方，必须为有限正数
};

struct Image_Sharpness_Orientations_Shape {
    int id = -1;                         // 基准编号从1开始；未匹配图形为-1，不代表一定是新增图形
    bool matched = false;                // 仅检测结果使用：是否匹配到了基准
    CHContour contour;                   // 外轮廓，原图像素坐标，不包含内孔轮廓
    CHPointF center;                      // 原图中外轮廓围成区域的重心；匹配归一化不改变测量坐标
    CHRect bounds;                       // 正外接矩形
    double area = 0.0;                   // 外轮廓包围面积，像素平方，包含内孔
    double orientation = 0.0;            // 图形主轴绝对角度，[0,180)，图像坐标系顺时针为正
    bool orientation_valid = false;      // 接近圆形/对称形状时主轴不可靠
    double angle = 0.0;                  // 仅检测结果使用：相对基准主轴角度，[-90,90)，顺时针为正
    bool angle_valid = false;            // 仅检测结果使用：成功匹配且两个主轴均可靠时为true
    // 图形前景平均亮度，范围0~255，排除内孔和其他独立图形
    float mean_brightness = 0.0f;
    // 图形清晰度，外轮廓带内Sobel/Tenengrad平均梯度能量 mean(Gx² + Gy²)，越大越清晰
    float sharpness = 0.0f;
    // 前景与局部背景平均亮度差的绝对值，范围0~255；没有可用局部背景时为0
    float contrast = 0.0f;
    double match_cost = -1.0;            // 仅检测结果使用：位置/顺序为主、形状为辅的匹配代价，非变形量；未匹配为-1
};

using Image_Sharpness_Orientations_Shapes = std::vector<Image_Sharpness_Orientations_Shape>;

// RegisterBase专用输出：预览已注册的基准图形，不包含检测匹配汇总。
// shapes中的matched、angle、angle_valid、match_cost仅供Detect使用，注册时保持默认值。
struct Image_Sharpness_Orientations_Register_Result {
    cv::Size image_size;                       // 基准图尺寸
    Image_Sharpness_Orientations_Shapes shapes; // 基准图形，按id排列；可通过shapes.size()获取数量
};

// Detect专用输出：当前图形测量值，以及相对已注册基准的匹配结果。
struct Image_Sharpness_Orientations_Detect_Result {
    cv::Size image_size;
    Image_Sharpness_Orientations_Shapes shapes; // 匹配项按基准id排列，其后为未匹配项
    std::vector<int> missing_ids;          // 本次没有找到的基准编号
    int base_shape_count = 0;
    int matched_count = 0;
    bool all_matched = false;             // 无缺失且无多余图形
    double angle = 0.0;                   // 整体相对基准旋转角，[-180,180)，顺时针为正
    bool angle_valid = false;            // 至少两个匹配中心可确定整体角度；单个则使用其主轴
};

#endif // IMAGE_SHARPNESS_ORIENTATIONS_STRUCTS_H
