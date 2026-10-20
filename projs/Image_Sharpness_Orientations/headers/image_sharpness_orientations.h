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

    // 每个实例独立保存一套基准，不能复制；注册和检测请勿在同一实例上并发执行。
    CH_Image_Sharpness_Orientations(const CH_Image_Sharpness_Orientations&) = delete;
    CH_Image_Sharpness_Orientations& operator=(const CH_Image_Sharpness_Orientations&) = delete;

private:
    class Impl;
    Impl* impl_;
};

#pragma endregion

#endif // IMAGE_SHARPNESS_ORIENTATIONS_H
