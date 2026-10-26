#pragma once

#include "image_sharpness_orientations_structs.h"

namespace iso_detail {

void measure_quality(const cv::Mat& gray, const cv::Mat& foreground,
    const std::vector<cv::Point>& contour, const std::vector<cv::Point>& product_contour,
    Image_Sharpness_Orientations_Shape& shape);

} // namespace iso_detail
