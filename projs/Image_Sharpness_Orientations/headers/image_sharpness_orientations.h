/*
ChangHua Technology Co., Ltd. (CH Tech)
Image_Sharpness_Orientations API Public Header File
Author: ALIMUJIANG YUEMAIER
Dependencies:
    image_sharpness_orientations_structs.h
    opencv 4.x (当前构建版本 4.12.0)
*/
#pragma once
#ifndef IMAGE_SHARPNESS_ORIENTATIONS_H
#define IMAGE_SHARPNESS_ORIENTATIONS_H

#if defined(IMAGE_SHARPNESS_ORIENTATIONS_STATIC)
  #define IMAGE_SHARPNESS_ORIENTATIONS_API
#elif defined(_WIN32) || defined(__CYGWIN__)
  #ifdef IMAGE_SHARPNESS_ORIENTATIONS_DLL_EXPORTS
    #define IMAGE_SHARPNESS_ORIENTATIONS_API __declspec(dllexport)
  #else
    #define IMAGE_SHARPNESS_ORIENTATIONS_API __declspec(dllimport)
  #endif
#elif defined(__GNUC__) && __GNUC__ >= 4
  #define IMAGE_SHARPNESS_ORIENTATIONS_API __attribute__((visibility("default")))
#else
  #define IMAGE_SHARPNESS_ORIENTATIONS_API
#endif

#include "image_sharpness_orientations_structs.h"

/// @brief 测试接口是否可用
/// @return 0表示可用
extern "C" IMAGE_SHARPNESS_ORIENTATIONS_API int test_IMAGE_SHARPNESS_ORIENTATIONS_API();

#pragma region 图像清晰度和方向检测

class IMAGE_SHARPNESS_ORIENTATIONS_API CH_Image_Sharpness_Orientations {
public:
    CH_Image_Sharpness_Orientations();
    ~CH_Image_Sharpness_Orientations();

    /// @brief 注册基准图，按detect_dark_area提取暗色或亮色图形的外轮廓并分配编号
    /// @param src 基准图，8位灰度/BGR/BGRA；图形与产品底色相反，接触图像边界的区域不作为图形
    /// @param result 输出基准图形、编号、面积、主轴角度、平均亮度、Tenengrad清晰度和对比度
    /// @param params 明暗选择、二值化阈值和最小面积；成功注册后保存在当前对象内存中供Detect使用
    /// @return from Image_Sharpness_Orientations_Error_Codes；注册失败保留之前的基准
    int RegisterBase(const CHImage& src, Image_Sharpness_Orientations_Register_Result& result,
        Image_Sharpness_Orientations_Params params = Image_Sharpness_Orientations_Params());

    /// @brief 以全局位置和相对顺序为主、形状为辅进行一对一匹配，输出编号及原图测量值
    /// @param src 待检测图，应为相同布局；允许不同分辨率、非等比缩放、剪切及适度透视/局部变形
    /// @param result 输出轮廓、编号、角度、面积、平均亮度、Tenengrad清晰度、对比度以及缺失编号
    /// @return from Image_Sharpness_Orientations_Error_Codes；ISO_OK不代表全部匹配
    int Detect(const CHImage& src, Image_Sharpness_Orientations_Detect_Result& result) const;

    /// @brief 独立检测最大投影外边缘，拟合椭圆并测量外边缘清晰度；无需注册基准
    /// @param src 8位灰度/BGR/BGRA图；完整投影必须位于图像内，内部图案不参与椭圆拟合
    /// @param result 输出真实轮廓、椭圆中心/半轴a和b/方向/轴比/拟合误差、亮度、清晰度和对比度
    /// @param params 投影明暗选择、二值化阈值和最小面积；本次调用使用，不修改已有基准
    /// @return ISO_OK表示测量成功，不代表投影合格；失败时清空result
    int DetectOuterEdge(const CHImage& src, Image_Sharpness_Orientations_Outer_Edge_Result& result,
        Image_Sharpness_Orientations_Outer_Edge_Params params = Image_Sharpness_Orientations_Outer_Edge_Params()) const;

    /// @brief 独立九点亮度检测：自动布点、整圆盘避黑或用户指定9点；无需注册基准
    /// @param src 8位灰度/BGR/BGRA图；亮度测量始终使用原始灰度像素
    /// @param result 每个点分别输出原坐标、实际坐标、移动标记、状态、采样像素数及平均灰度
    /// @param params 模式、缩进量、圆盘直径、避黑阈值/搜索半径、手动坐标及轮廓参数
    /// @return ISO_OK: 9点均有效；ISO_PartialMeasurement: 保留9点结果，检查各点valid/status；其他错误清空result
    int DetectBrightness(const CHImage& src, Image_Sharpness_Orientations_Brightness_Result& result,
        Image_Sharpness_Orientations_Brightness_Params params = Image_Sharpness_Orientations_Brightness_Params()) const;

    // 每个实例独立保存一套基准，不能复制；注册和检测请勿在同一实例上并发执行。
    CH_Image_Sharpness_Orientations(const CH_Image_Sharpness_Orientations&) = delete;
    CH_Image_Sharpness_Orientations& operator=(const CH_Image_Sharpness_Orientations&) = delete;

private:
    class Impl;
    Impl* impl_;
};

#pragma endregion

#endif // IMAGE_SHARPNESS_ORIENTATIONS_H
