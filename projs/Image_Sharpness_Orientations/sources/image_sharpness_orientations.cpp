#include "image_sharpness_orientations.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <numeric>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Infrequent tuning stays private; these preserve the original default behavior.
constexpr int kBlurSize = 3;
constexpr double kMaxAreaRatio = 0.25;
constexpr int kContourBand = 2;
constexpr int kBackgroundRadius = 5;
constexpr double kMaxMatchDistance = 0.12;
constexpr double kMaxAreaChange = 2.0;
constexpr double kMaxShapeDistance = 1.0;
constexpr double kMinOrientationAnisotropy = 0.08;

struct Product_Frame {
    cv::Point2d center;
    double area = 0.0;
};

double axis_difference(double angle) {
    while (angle >= 90.0) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;
    return angle;
}

void measure_axis(const cv::Moments& m, double minimum_anisotropy,
    double& angle, bool& valid) {
    const double spread = m.mu20 + m.mu02;
    const double difference = std::hypot(m.mu20 - m.mu02, 2.0 * m.mu11);
    valid = spread > 1e-9 && difference / spread >= minimum_anisotropy;
    angle = 0.0;
    if (valid) {
        angle = 0.5 * std::atan2(2.0 * m.mu11, m.mu20 - m.mu02) * 180.0 / kPi;
        if (angle < 0.0) angle += 180.0;
    }
}

bool valid_params(const Image_Sharpness_Orientations_Params& p) {
    return p.threshold >= -1 && p.threshold <= 255 &&
        std::isfinite(p.min_area) && p.min_area > 0.0;
}

// Segmentation identifies the pixels, but all measurements use the original gray image.
void measure_quality(const cv::Mat& gray, const cv::Mat& foreground,
    const std::vector<cv::Point>& contour, const std::vector<cv::Point>& product_contour,
    Image_Sharpness_Orientations_Shape& shape) {
    const int margin = std::max(kContourBand, kBackgroundRadius) + 3;
    const auto& bounds = shape.bounds;
    const int left = std::max(0, bounds.x - margin);
    const int top = std::max(0, bounds.y - margin);
    const int right = std::min(gray.cols, bounds.x + bounds.width + margin);
    const int bottom = std::min(gray.rows, bounds.y + bounds.height + margin);
    const cv::Rect roi(left, top, right - left, bottom - top);
    const cv::Mat gray_roi = gray(roi);

    // Isolate this connected part, excluding holes and disconnected parts nested inside.
    // Mask-only flood fill leaves both the image and the segmentation unchanged.
    cv::Mat foreground_roi = foreground(roi);
    cv::Mat flood_mask = cv::Mat::zeros(roi.height + 2, roi.width + 2, CV_8UC1);
    cv::floodFill(foreground_roi, flood_mask, contour.front() - roi.tl(), cv::Scalar(0),
        nullptr, cv::Scalar(0), cv::Scalar(0), 8 | cv::FLOODFILL_MASK_ONLY | (255 << 8));
    const cv::Mat component_mask = flood_mask(cv::Rect(1, 1, roi.width, roi.height));
    const double foreground_mean = cv::mean(gray_roi, component_mask)[0];
    shape.mean_brightness = static_cast<float>(foreground_mean);

    // Sample nearby background around the part (including hole boundaries).
    // Exclude every selected foreground region, as well as pixels outside the product.
    cv::Mat background_mask, opposite_mask, product_region = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::dilate(component_mask, background_mask, cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * kBackgroundRadius + 1, 2 * kBackgroundRadius + 1)),
        cv::Point(-1, -1), 1, cv::BORDER_CONSTANT | cv::BORDER_ISOLATED, cv::Scalar(0));
    cv::bitwise_not(foreground_roi, opposite_mask);
    cv::bitwise_and(background_mask, opposite_mask, background_mask);
    cv::drawContours(product_region, std::vector<std::vector<cv::Point>>{product_contour},
        0, cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), 0, -roi.tl());
    cv::bitwise_and(background_mask, product_region, background_mask);
    if (cv::countNonZero(background_mask) > 0) {
        const double background_mean = cv::mean(gray_roi, background_mask)[0];
        shape.contrast = static_cast<float>(std::abs(foreground_mean - background_mean));
    }

    // Same unscaled 3x3 Sobel/Tenengrad energy formula as Alg_CH, sampled on our contour band.
    cv::Mat dx, dy;
    cv::Sobel(gray_roi, dx, CV_32F, 1, 0, 3);
    cv::Sobel(gray_roi, dy, CV_32F, 0, 1, 3);
    const cv::Mat energy = dx.mul(dx) + dy.mul(dy);
    cv::Mat band_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::drawContours(band_mask, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(255),
        2 * kContourBand + 1, cv::LINE_8, cv::noArray(), 0, -roi.tl());
    shape.sharpness = static_cast<float>(cv::mean(energy, band_mask)[0]);
}

int extract_shapes(const CHImage& src, const Image_Sharpness_Orientations_Params& params,
    cv::Size& image_size, Image_Sharpness_Orientations_Shapes& shapes, Product_Frame& frame) {
    if (src.empty()) return ISO_ImageIsEmpty;
    if (src.dims != 2 || src.depth() != CV_8U ||
        (src.channels() != 1 && src.channels() != 3 && src.channels() != 4)) {
        return ISO_UnsupportedImage;
    }
    image_size = src.size();

    cv::Mat gray;
    if (src.channels() == 1) gray = src;
    else cv::cvtColor(src, gray, src.channels() == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);

    cv::Mat filtered, foreground;
    cv::GaussianBlur(gray, filtered, cv::Size(kBlurSize, kBlurSize), 0.0);
    const int threshold_type = params.detect_dark_area ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    cv::threshold(filtered, foreground, std::max(params.threshold, 0), 255,
        threshold_type | (params.threshold < 0 ? cv::THRESH_OTSU : 0));

    // The product has the opposite polarity to its marks: bright for dark marks,
    // dark for light marks. Its largest silhouette provides position and scale.
    cv::Mat product_mask;
    cv::bitwise_not(foreground, product_mask);
    std::vector<std::vector<cv::Point>> product_contours;
    cv::findContours(product_mask, product_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (product_contours.empty()) return ISO_NoShapes;
    const auto product_it = std::max_element(product_contours.begin(), product_contours.end(),
        [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
    const cv::Moments product_moments = cv::moments(*product_it);
    if (product_moments.m00 <= 0.0) return ISO_NoShapes;
    frame.area = product_moments.m00;
    frame.center = {product_moments.m10 / frame.area, product_moments.m01 / frame.area};

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // CCOMP gives each selected connected component its own outer contour.
    // Opposite-polarity holes remain children; border-connected background is skipped.
    cv::findContours(foreground, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    const double maximum_area = kMaxAreaRatio * static_cast<double>(src.total());
    for (size_t i = 0; i < contours.size(); ++i) {
        if (hierarchy[i][3] >= 0) continue; // Do not count holes as separate parts.
        const auto& contour = contours[i];
        const cv::Rect bounds = cv::boundingRect(contour);
        if (bounds.x <= 0 || bounds.y <= 0 ||
            bounds.x + bounds.width >= src.cols || bounds.y + bounds.height >= src.rows) continue;
        const cv::Moments m = cv::moments(contour);
        if (m.m00 < params.min_area || m.m00 > maximum_area) continue;
        const cv::Point2f center(static_cast<float>(m.m10 / m.m00),
            static_cast<float>(m.m01 / m.m00));
        if (cv::pointPolygonTest(*product_it, center, false) < 0.0) continue;

        Image_Sharpness_Orientations_Shape shape;
        shape.center = center;
        shape.bounds = bounds;
        shape.area = m.m00;
        shape.contour.reserve(contour.size());
        for (const auto& point : contour) shape.contour.emplace_back(point);
        measure_axis(m, kMinOrientationAnisotropy,
            shape.orientation, shape.orientation_valid);
        measure_quality(gray, foreground, contour, *product_it, shape);
        shapes.push_back(std::move(shape));
    }
    std::sort(shapes.begin(), shapes.end(), [](const auto& a, const auto& b) {
        if (a.center.x != b.center.x) return a.center.x < b.center.x;
        return a.center.y < b.center.y;
    });
    return shapes.empty() ? ISO_NoShapes : ISO_OK;
}

cv::Point2d normalized_center(const Image_Sharpness_Orientations_Shape& shape,
    const Product_Frame& frame, double rotation) {
    const cv::Point2d point = (cv::Point2d(shape.center) - frame.center) / std::sqrt(frame.area);
    const double radians = rotation * kPi / 180.0;
    const double c = std::cos(radians), s = std::sin(radians);
    return {c * point.x - s * point.y, s * point.x + c * point.y};
}

// Compactness and moment anisotropy remain stable for thin, slightly blurred strokes.
// Log compactness is scale/rotation invariant; anisotropy is bounded in [0, 1].
cv::Vec2d shape_descriptor(const CHContour& contour) {
    const auto m = cv::moments(contour);
    const double perimeter = cv::arcLength(contour, true);
    return {std::log(perimeter * perimeter / m.m00),
        std::hypot(m.mu20 - m.mu02, 2.0 * m.mu11) / (m.mu20 + m.mu02)};
}

void match_shapes(const Image_Sharpness_Orientations_Register_Result& base, const Product_Frame& base_frame,
    const Product_Frame& current_frame, Image_Sharpness_Orientations_Detect_Result& result, double rotation) {
    result.missing_ids.clear();
    result.angle = 0.0;
    result.angle_valid = false;
    for (auto& shape : result.shapes) {
        shape.id = -1;
        shape.matched = false;
        shape.match_cost = -1.0;
        shape.angle = 0.0;
        shape.angle_valid = false;
    }
    struct Candidate { double cost; size_t base_index; size_t current_index; };
    std::vector<Candidate> candidates;
    std::vector<cv::Vec2d> base_descriptors, current_descriptors;
    for (const auto& shape : base.shapes) base_descriptors.push_back(shape_descriptor(shape.contour));
    for (const auto& shape : result.shapes) current_descriptors.push_back(shape_descriptor(shape.contour));
    for (size_t i = 0; i < base.shapes.size(); ++i) {
        const auto& model = base.shapes[i];
        const auto model_center = normalized_center(model, base_frame, 0.0);
        for (size_t j = 0; j < result.shapes.size(); ++j) {
            const auto& current = result.shapes[j];
            const double distance = cv::norm(
                model_center - normalized_center(current, current_frame, rotation));
            if (distance > kMaxMatchDistance) continue;
            const double area_ratio = (current.area / current_frame.area) /
                (model.area / base_frame.area);
            if (area_ratio > kMaxAreaChange || area_ratio < 1.0 / kMaxAreaChange) continue;
            const double shape_distance = std::abs(base_descriptors[i][0] - current_descriptors[j][0]) +
                std::abs(base_descriptors[i][1] - current_descriptors[j][1]);
            if (!std::isfinite(shape_distance) || shape_distance > kMaxShapeDistance) continue;
            const double cost = distance / kMaxMatchDistance +
                0.35 * std::abs(std::log(area_ratio)) / std::log(kMaxAreaChange) +
                0.20 * shape_distance / kMaxShapeDistance;
            candidates.push_back({cost, i, j});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.cost != b.cost) return a.cost < b.cost;
        if (a.base_index != b.base_index) return a.base_index < b.base_index;
        return a.current_index < b.current_index;
    });
    std::vector<bool> base_used(base.shapes.size(), false);
    std::vector<bool> current_used(result.shapes.size(), false);
    std::vector<std::pair<size_t, size_t>> matches;
    for (const auto& candidate : candidates) {
        if (base_used[candidate.base_index] || current_used[candidate.current_index]) continue;
        base_used[candidate.base_index] = true;
        current_used[candidate.current_index] = true;
        const auto& model = base.shapes[candidate.base_index];
        auto& shape = result.shapes[candidate.current_index];
        shape.id = model.id;
        shape.matched = true;
        shape.match_cost = candidate.cost;
        shape.angle_valid = model.orientation_valid && shape.orientation_valid;
        if (shape.angle_valid) shape.angle = axis_difference(shape.orientation - model.orientation);
        matches.emplace_back(candidate.base_index, candidate.current_index);
    }
    result.base_shape_count = static_cast<int>(base.shapes.size());
    result.matched_count = static_cast<int>(matches.size());
    for (size_t i = 0; i < base_used.size(); ++i) {
        if (!base_used[i]) result.missing_ids.push_back(base.shapes[i].id);
    }
    result.all_matched = result.missing_ids.empty() && matches.size() == result.shapes.size();

    // Least-squares rotation of matched centers; translation and uniform scale cancel.
    if (matches.size() >= 2) {
        cv::Point2d base_mean(0.0, 0.0), current_mean(0.0, 0.0);
        for (const auto& pair : matches) {
            base_mean += cv::Point2d(base.shapes[pair.first].center);
            current_mean += cv::Point2d(result.shapes[pair.second].center);
        }
        base_mean *= 1.0 / static_cast<double>(matches.size());
        current_mean *= 1.0 / static_cast<double>(matches.size());
        double dot = 0.0, cross = 0.0;
        for (const auto& pair : matches) {
            const auto a = cv::Point2d(base.shapes[pair.first].center) - base_mean;
            const auto b = cv::Point2d(result.shapes[pair.second].center) - current_mean;
            dot += a.dot(b);
            cross += a.x * b.y - a.y * b.x;
        }
        result.angle_valid = std::hypot(dot, cross) > 1e-9;
        if (result.angle_valid) {
            result.angle = std::atan2(cross, dot) * 180.0 / kPi;
            if (result.angle >= 180.0) result.angle -= 360.0;
        }
    } else if (matches.size() == 1) {
        const auto& shape = result.shapes[matches[0].second];
        result.angle_valid = shape.angle_valid;
        result.angle = shape.angle;
    }
    std::stable_sort(result.shapes.begin(), result.shapes.end(), [](const auto& a, const auto& b) {
        if (a.matched != b.matched) return a.matched;
        return a.matched && a.id < b.id;
    });
}

} // namespace

class CH_Image_Sharpness_Orientations::Impl {
public:
    Image_Sharpness_Orientations_Params params;
    Image_Sharpness_Orientations_Register_Result base;
    Product_Frame frame;
};

extern "C" int test_IMAGE_SHARPNESS_ORIENTATIONS_API() { return ISO_OK; }

CH_Image_Sharpness_Orientations::CH_Image_Sharpness_Orientations() : impl_(nullptr) {}
CH_Image_Sharpness_Orientations::~CH_Image_Sharpness_Orientations() { delete impl_; }

int CH_Image_Sharpness_Orientations::RegisterBase(const CHImage& src,
    Image_Sharpness_Orientations_Register_Result& result, Image_Sharpness_Orientations_Params params) {
    result = {};
    if (!valid_params(params)) return ISO_InvalidParameter;
    try {
        auto next = std::make_unique<Impl>();
        next->params = params;
        const int status = extract_shapes(src, params, next->base.image_size, next->base.shapes, next->frame);
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
        Product_Frame frame;
        const int status = extract_shapes(src, impl_->params, result.image_size, result.shapes, frame);
        if (status == ISO_NoShapes) {
            result.base_shape_count = static_cast<int>(impl_->base.shapes.size());
            for (const auto& shape : impl_->base.shapes) result.missing_ids.push_back(shape.id);
            return status;
        }
        if (status != ISO_OK) return status;
        // Start from nearby normalized positions; then refine using matched centers.
        // The product silhouette itself can change principal axis under perspective.
        match_shapes(impl_->base, impl_->frame, frame, result, 0.0);
        if (result.matched_count >= 2 && result.angle_valid) {
            auto refined = result;
            match_shapes(impl_->base, impl_->frame, frame, refined, -result.angle);
            const auto total_cost = [](const Image_Sharpness_Orientations_Detect_Result& candidate) {
                double cost = 0.0;
                for (const auto& shape : candidate.shapes) if (shape.matched) cost += shape.match_cost;
                return cost;
            };
            if (refined.matched_count > result.matched_count ||
                (refined.matched_count == result.matched_count && total_cost(refined) < total_cost(result))) {
                result = std::move(refined);
            }
        }
        return ISO_OK;
    } catch (const std::exception&) {
        result = {};
        return ISO_Fail;
    }
}
