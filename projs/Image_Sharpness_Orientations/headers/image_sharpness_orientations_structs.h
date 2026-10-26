#pragma once
#ifndef IMAGE_SHARPNESS_ORIENTATIONS_STRUCTS_H
#define IMAGE_SHARPNESS_ORIENTATIONS_STRUCTS_H

#include <opencv2/core.hpp>
#include <array>
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
    ISO_OuterEdgeClipped = 9,     // 最大外轮廓接触图像边界，无法测量完整投影
    ISO_EllipseFitFailed = 10,    // 外轮廓不能拟合为有效椭圆
    ISO_PartialMeasurement = 11,  // 九点测量部分/全部点无效；保留各点状态和有效测量值
    ISO_Fail = -1                 // 处理失败
};

struct Image_Sharpness_Orientations_Params {
    bool detect_dark_area = true; // true: 检测亮底上的暗色图形；false: 检测暗底上的白色/亮色图形
    int threshold = -1;           // -1: Otsu自动阈值；0~255: 固定阈值，暗色<=阈值，亮色>阈值
    double min_area = 100.0;      // 最小外轮廓面积，像素平方，必须为有限正数
};

// 独立外边缘检测参数；每次调用传入，不依赖RegisterBase保存的参数。
struct Image_Sharpness_Orientations_Outer_Edge_Params {
    bool detect_bright_area = true; // true: 黑底上的亮投影；false: 亮底上的暗投影
    int threshold = -1;             // -1: Otsu；0~255: 固定阈值，亮区>阈值，暗区<=阈值
    double min_area = 100.0;         // 最小外轮廓面积，像素平方，必须为有限正数
};

// DetectOuterEdge专用输出；仅ISO_OK时有效，失败时所有字段清空。
struct Image_Sharpness_Orientations_Outer_Edge_Result {
    cv::Size image_size;
    CHContour contour;                 // 检测到的真实外轮廓，原图像素坐标，不是拟合椭圆
    CHRect bounds;                     // 真实外轮廓的正外接矩形
    double area = 0.0;                 // 真实外轮廓包围面积，像素平方，包含内部图案/孔洞
    CHPointF center;                    // 拟合椭圆中心，原图像素坐标
    double a = 0.0;                    // 长半轴，像素；完整长轴长度为2*a
    double b = 0.0;                    // 短半轴，像素；a>=b>0，完整短轴长度为2*b
    double orientation = 0.0;          // 长轴方向，[0,180)度，图像坐标系顺时针为正
    bool orientation_valid = false;    // 接近圆形时方向不可靠，a/b/中心/清晰度仍有效
    double axis_ratio = 0.0;           // b/a，(0,1]；越接近1越接近圆形
    // 椭圆坐标中归一化半径误差的RMS，理想椭圆为0，无量纲，不是像素距离
    double fit_error = 0.0;
    float mean_brightness = 0.0f;      // 外轮廓内部原图平均灰度0~255，包含内部图案/孔洞
    float sharpness = 0.0f;            // 真实外轮廓带内Sobel/Tenengrad平均梯度能量
    float contrast = 0.0f;             // 外边缘内、外两侧局部平均灰度差的绝对值，0~255
};

enum class Image_Sharpness_Orientations_Brightness_Mode {
    Auto,       // 椭圆中心+8个方向，直接测量指定圆盘，允许包含黑色像素
    AvoidBlack, // 自动布点，整块圆盘必须位于真实投影内且所有像素高于黑色阈值；必要时移动
    Manual      // 按用户提供的9个点测量，不拟合外轮廓、不自动移动
};

enum class Image_Sharpness_Orientations_Brightness_Point_Status {
    NotMeasured,
    Valid,
    OutsideImage, // 圆盘超出输入图像，不裁剪采样面积
    NoPixels,     // 极小的亚像素圆盘内没有像素中心
    NoBrightArea  // 搜索半径内找不到能容纳完整圆盘的亮区
};

struct Image_Sharpness_Orientations_Brightness_Params {
    Image_Sharpness_Orientations_Brightness_Mode mode = Image_Sharpness_Orientations_Brightness_Mode::Auto;
    int inset_px = 20;          // 自动模式：从椭圆边界沿射线向中心缩进到采样中心的距离，>=0
    int spot_diameter_px = 11;  // 每个采样圆盘的直径，>=1；半径=直径/2.0，三个模式均使用
    int black_threshold = -1;  // 仅避黑模式使用，其他模式忽略：原始灰度<=阈值为黑；-1为Otsu，0~255固定阈值
    int search_radius_px = 50; // 仅避黑模式使用，其他模式忽略：新中心离原中心的最大距离；0禁止移动
    std::vector<CHPointF> points; // 手动模式必须恰好9点，按输入顺序输出；其他模式忽略
    Image_Sharpness_Orientations_Outer_Edge_Params outer_edge; // 自动/避黑模式的轮廓提取参数
};

struct Image_Sharpness_Orientations_Brightness_Point {
    int id = 0;                       // 1~9；自动模式1为中心，2从正上方开始每45度顺时针排列
    CHPointF requested_center;          // 自动布点或用户给定的原始坐标
    CHPointF center;                    // 实际测量坐标；无有效替代点时保留原始坐标
    bool moved = false;               // 是否由避黑模式移动；移动后的中心位于整数像素坐标
    bool valid = false;
    Image_Sharpness_Orientations_Brightness_Point_Status status = Image_Sharpness_Orientations_Brightness_Point_Status::NotMeasured;
    int pixel_count = 0;              // 圆盘内参与平均的像素数；无效时为0
    float mean_brightness = 0.0f;      // 此圆盘的原始灰度均值0~255；不是九点平均值，不是标定后的lux
};

struct Image_Sharpness_Orientations_Brightness_Result {
    cv::Size image_size;
    std::array<Image_Sharpness_Orientations_Brightness_Point, 9> points{};
    int spot_diameter_px = 0;
    int valid_count = 0;
    bool all_valid = false;
    int effective_black_threshold = -1; // 避黑模式实际使用的阈值；其他模式为-1
    bool ellipse_valid = false;         // 手动模式为false；以下几何量用于显示自动布点依据
    CHPointF ellipse_center;
    double ellipse_a = 0.0;             // 长半轴，像素
    double ellipse_b = 0.0;             // 短半轴，像素
    double ellipse_orientation = 0.0;   // 长轴方向，[0,180)度，顺时针
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
