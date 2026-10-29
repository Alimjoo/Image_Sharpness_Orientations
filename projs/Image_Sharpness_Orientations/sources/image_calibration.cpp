#include "image_sharpness_orientations.h"
#include "calibration.h"

#include <exception>
#include <utility>

int CH_Image_Calibration::Calibrate(const CHImage& src, Image_Calibration_Result& result,
    Image_Calibration_Params params) const {
    result = {};
    try {
        Image_Calibration_Result next;
        const int status = iso_detail::calibrate_image(src, params, next);
        if (status == ISO_OK) result = std::move(next);
        return status;
    } catch (const std::exception&) {
        result = {};
        return ISO_Fail;
    }
}
