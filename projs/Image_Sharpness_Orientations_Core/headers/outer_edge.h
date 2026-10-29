#pragma once

#include "image_sharpness_orientations_structs.h"

namespace iso_detail {

int extract_outer_contour(const CHImage& src, const Image_Sharpness_Orientations_Outer_Edge_Params& params,
    std::vector<cv::Point>& contour, cv::Mat& gray);

// Geometry-only path shared with nine-point placement; returns the original gray image.
int fit_outer_edge(const CHImage& src, const Image_Sharpness_Orientations_Outer_Edge_Params& params,
    Image_Sharpness_Orientations_Outer_Edge_Result& result, cv::Mat& gray);

int detect_outer_edge(const CHImage& src, const Image_Sharpness_Orientations_Outer_Edge_Params& params,
    Image_Sharpness_Orientations_Outer_Edge_Result& result);

} // namespace iso_detail
