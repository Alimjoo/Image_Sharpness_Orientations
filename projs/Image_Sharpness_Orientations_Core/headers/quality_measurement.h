#pragma once

#include "image_sharpness_orientations_structs.h"

namespace iso_detail {

void measure_quality(const cv::Mat& gray, const cv::Mat& foreground,
    const std::vector<cv::Point>& contour, const std::vector<cv::Point>& product_contour,
    Image_Sharpness_Orientations_Shape& shape);

void measure_outer_quality(const cv::Mat& gray, const std::vector<cv::Point>& contour,
    Image_Sharpness_Orientations_Outer_Edge_Result& result);

} // namespace iso_detail
