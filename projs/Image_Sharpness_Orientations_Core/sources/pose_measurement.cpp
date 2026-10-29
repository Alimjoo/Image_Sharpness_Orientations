#include "pose_measurement.h"
#include "image_sharpness_orientations_core.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace iso_detail {
namespace {

using Shape = Image_Sharpness_Orientations_Shape;

struct Correspondence {
    const Shape* base;
    const Shape* current;
};

} // namespace

bool valid_detect_params(const Image_Sharpness_Orientations_Detect_Params& params,
    const Image_Sharpness_Orientations_Register_Result& base) {
    const auto& ids = params.feature_ids;
    if (ids == std::array<int, 2>{0, 0}) return true;
    if (ids[0] <= 0 || ids[1] <= 0 || ids[0] == ids[1]) return false;
    for (int id : ids) {
        if (std::none_of(base.shapes.begin(), base.shapes.end(),
            [id](const auto& shape) { return shape.id == id; })) return false;
    }
    return true;
}

void measure_pose(const Image_Sharpness_Orientations_Register_Result& base,
    const Image_Sharpness_Orientations_Detect_Params& params,
    Image_Sharpness_Orientations_Detect_Result& result) {
    result.angle = result.offset_x = result.offset_y = 0.0;
    result.angle_valid = result.reference_valid = result.offset_valid = false;
    result.base_image_size = base.image_size;
    result.base_reference_center = result.reference_center = cv::Point2d();
    result.reference_ids.clear();
    result.using_feature_pair = params.feature_ids != std::array<int, 2>{0, 0};
    const bool same_size = result.image_size == base.image_size;
    std::vector<Correspondence> matches;
    for (auto& current : result.shapes) {
        current.base_center = CHPointF();
        current.offset_x = current.offset_y = 0.0;
        current.offset_valid = false;
        if (!current.matched) continue;
        const auto model = std::find_if(base.shapes.begin(), base.shapes.end(),
            [&](const auto& shape) { return shape.id == current.id; });
        if (model == base.shapes.end()) continue;
        current.base_center = model->center;
        if (same_size) {
            current.offset_x = static_cast<double>(current.center.x) - model->center.x;
            current.offset_y = static_cast<double>(current.center.y) - model->center.y;
            current.offset_valid = true;
        }
        matches.push_back({&*model, &current});
    }

    if (result.using_feature_pair) {
        result.reference_ids.assign(params.feature_ids.begin(), params.feature_ids.end());
        std::vector<Correspondence> selected;
        for (int id : params.feature_ids) {
            const auto found = std::find_if(matches.begin(), matches.end(),
                [id](const auto& match) { return match.base->id == id; });
            if (found == matches.end()) return; // An explicit pair never falls back to other features.
            selected.push_back(*found);
        }
        const auto base_delta = cv::Point2d(selected[1].base->center) - cv::Point2d(selected[0].base->center);
        const auto current_delta = cv::Point2d(selected[1].current->center) - cv::Point2d(selected[0].current->center);
        if (cv::norm(base_delta) <= 1e-6 || cv::norm(current_delta) <= 1e-6) return;
        matches = std::move(selected);
    } else {
        for (const auto& match : matches) result.reference_ids.push_back(match.base->id);
    }
    if (matches.empty()) return;

    cv::Point2d base_mean(0.0, 0.0), current_mean(0.0, 0.0);
    for (const auto& match : matches) {
        base_mean += cv::Point2d(match.base->center);
        current_mean += cv::Point2d(match.current->center);
    }
    base_mean *= 1.0 / static_cast<double>(matches.size());
    current_mean *= 1.0 / static_cast<double>(matches.size());
    result.base_reference_center = base_mean;
    result.reference_center = current_mean;
    result.reference_valid = true;
    if (same_size) {
        result.offset_x = current_mean.x - base_mean.x;
        result.offset_y = current_mean.y - base_mean.y;
        result.offset_valid = true;
    }

    // Preserve the automatic least-squares rotation. For a selected pair this is
    // exactly the signed difference between the directed base/current feature lines.
    if (matches.size() >= 2) {
        double dot = 0.0, cross = 0.0;
        for (const auto& match : matches) {
            const auto a = cv::Point2d(match.base->center) - base_mean;
            const auto b = cv::Point2d(match.current->center) - current_mean;
            dot += a.dot(b);
            cross += a.x * b.y - a.y * b.x;
        }
        result.angle_valid = std::hypot(dot, cross) > 1e-9;
        if (result.angle_valid) {
            result.angle = std::atan2(cross, dot) * 180.0 / kPi;
            if (result.angle >= 180.0) result.angle -= 360.0;
        }
    } else {
        result.angle_valid = matches[0].current->angle_valid;
        result.angle = matches[0].current->angle;
    }
}

} // namespace iso_detail
