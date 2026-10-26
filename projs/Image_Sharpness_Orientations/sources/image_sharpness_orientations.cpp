#include "image_sharpness_orientations.h"
#include "image_sharpness_orientations_core.h"

#include <exception>
#include <memory>

class CH_Image_Sharpness_Orientations::Impl {
public:
    Image_Sharpness_Orientations_Params params;
    Image_Sharpness_Orientations_Register_Result base;
    iso_detail::Product_Frame frame;
};

extern "C" int test_IMAGE_SHARPNESS_ORIENTATIONS_API() { return ISO_OK; }

CH_Image_Sharpness_Orientations::CH_Image_Sharpness_Orientations() : impl_(nullptr) {}
CH_Image_Sharpness_Orientations::~CH_Image_Sharpness_Orientations() { delete impl_; }

int CH_Image_Sharpness_Orientations::RegisterBase(const CHImage& src,
    Image_Sharpness_Orientations_Register_Result& result, Image_Sharpness_Orientations_Params params) {
    result = {};
    if (!iso_detail::valid_params(params)) return ISO_InvalidParameter;
    try {
        auto next = std::make_unique<Impl>();
        next->params = params;
        const int status = iso_detail::extract_shapes(src, params, next->base.image_size, next->base.shapes, next->frame);
        if (status != ISO_OK) return status;
        for (size_t i = 0; i < next->base.shapes.size(); ++i) {
            next->base.shapes[i].id = static_cast<int>(i) + 1;
        }
        result = next->base;
        // Commit only after extraction and all output allocations succeed.
        delete impl_;
        impl_ = next.release();
        return ISO_OK;
    } catch (const std::exception&) {
        result = {};
        return ISO_Fail;
    }
}

int CH_Image_Sharpness_Orientations::Detect(const CHImage& src,
    Image_Sharpness_Orientations_Detect_Result& result) const {
    result = {};
    if (!impl_) return ISO_BaseNotRegistered;
    try {
        iso_detail::Product_Frame frame;
        const int status = iso_detail::extract_shapes(src, impl_->params, result.image_size, result.shapes, frame);
        if (status == ISO_NoShapes) {
            result.base_shape_count = static_cast<int>(impl_->base.shapes.size());
            for (const auto& shape : impl_->base.shapes) result.missing_ids.push_back(shape.id);
            return status;
        }
        if (status != ISO_OK) return status;
        iso_detail::match_shapes(impl_->base, impl_->frame, frame, result);
        return ISO_OK;
    } catch (const std::exception&) {
        result = {};
        return ISO_Fail;
    }
}
