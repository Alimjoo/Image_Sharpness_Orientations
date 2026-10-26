#pragma once

#include "image_sharpness_orientations_structs.h"

namespace iso_detail {

int detect_brightness(const CHImage& src, const Image_Sharpness_Orientations_Brightness_Params& params,
    Image_Sharpness_Orientations_Brightness_Result& result);

} // namespace iso_detail
