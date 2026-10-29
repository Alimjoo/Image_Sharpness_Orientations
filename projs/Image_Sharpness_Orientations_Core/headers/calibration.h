#pragma once

#include "image_sharpness_orientations_structs.h"

namespace iso_detail {

int calibrate_image(const CHImage& src, const Image_Calibration_Params& params,
    Image_Calibration_Result& result);

} // namespace iso_detail
