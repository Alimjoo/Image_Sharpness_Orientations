#pragma once

#include "image_sharpness_orientations_structs.h"

// Internal interface between the API wrapper and the algorithm implementation.
// These declarations are not exported or included in the public package.
namespace iso_detail {

inline constexpr double kPi = 3.14159265358979323846;

struct Product_Frame {
    cv::Point2d center;
    double area = 0.0;
    cv::Matx22d normalization = cv::Matx22d::eye();
};

bool valid_params(const Image_Sharpness_Orientations_Params& params);

int extract_shapes(const CHImage& src, const Image_Sharpness_Orientations_Params& params,
    cv::Size& image_size, Image_Sharpness_Orientations_Shapes& shapes, Product_Frame& frame);

void match_shapes(const Image_Sharpness_Orientations_Register_Result& base,
    const Product_Frame& base_frame, const Product_Frame& current_frame,
    Image_Sharpness_Orientations_Detect_Result& result);

} // namespace iso_detail
