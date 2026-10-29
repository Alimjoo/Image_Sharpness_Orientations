#pragma once

#include "image_sharpness_orientations_structs.h"

namespace iso_detail {

bool valid_detect_params(const Image_Sharpness_Orientations_Detect_Params& params,
    const Image_Sharpness_Orientations_Register_Result& base);

void measure_pose(const Image_Sharpness_Orientations_Register_Result& base,
    const Image_Sharpness_Orientations_Detect_Params& params,
    Image_Sharpness_Orientations_Detect_Result& result);

} // namespace iso_detail
