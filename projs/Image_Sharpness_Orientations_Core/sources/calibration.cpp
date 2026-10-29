#include "calibration.h"
#include "image_sharpness_orientations_core.h"
#include "outer_edge.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace iso_detail {
namespace {

bool positive_finite(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool valid_calibration_params(const Image_Calibration_Params& p) {
    if (p.threshold < -1 || p.threshold > 255 || !positive_finite(p.min_area) ||
        !positive_finite(p.max_relative_error) || p.max_relative_error > 0.25) return false;
    if (p.shape == Image_Calibration_Shape::Circle) return positive_finite(p.diameter_mm);
    if (p.shape == Image_Calibration_Shape::Rectangle) {
        return positive_finite(p.width_mm) && positive_finite(p.height_mm);
    }
    return false;
}

double cross(const cv::Point2d& a, const cv::Point2d& b) {
    return a.x * b.y - a.y * b.x;
}

struct Edge_Line {
    cv::Point2d origin;
    cv::Point2d direction;
};

bool fit_rectangle(const std::vector<cv::Point>& contour, const Image_Calibration_Params& params,
    Image_Calibration_Result& result) {
    std::vector<cv::Point> polygon;
    cv::approxPolyDP(contour, polygon, 0.015 * cv::arcLength(contour, true), true);
    if (polygon.size() != 4 || !cv::isContourConvex(polygon)) return false;
    if (cv::contourArea(polygon, true) < 0.0) std::reverse(polygon.begin(), polygon.end());
    // Give square targets a repeatable starting corner; physical corner identities remain ambiguous.
    const auto first = std::min_element(polygon.begin(), polygon.end(), [](const auto& a, const auto& b) {
        return a.y != b.y ? a.y < b.y : a.x < b.x;
    });
    std::rotate(polygon.begin(), first, polygon.end());

    std::array<std::vector<cv::Point2f>, 4> groups;
    for (const auto& point : contour) {
        double nearest = std::numeric_limits<double>::infinity();
        double nearest_t = 0.0;
        size_t side = 0;
        for (size_t i = 0; i < 4; ++i) {
            const cv::Point2d origin(polygon[i]);
            const cv::Point2d delta = cv::Point2d(polygon[(i + 1) % 4]) - origin;
            const double length_squared = delta.dot(delta);
            if (length_squared < 1.0) return false;
            const double t = (cv::Point2d(point) - origin).dot(delta) / length_squared;
            const double distance = cv::norm(cv::Point2d(point) - (origin + std::clamp(t, 0.0, 1.0) * delta));
            if (distance < nearest) { nearest = distance; nearest_t = t; side = i; }
        }
        // Keep corner rounding out of the line fit; intersect the fitted straight edges afterwards.
        if (nearest_t >= 0.05 && nearest_t <= 0.95) groups[side].emplace_back(point);
    }

    std::array<Edge_Line, 4> lines;
    for (size_t i = 0; i < 4; ++i) {
        if (groups[i].size() < 5) return false;
        cv::Vec4f fitted;
        cv::fitLine(groups[i], fitted, cv::DIST_L2, 0.0, 0.01, 0.01);
        const double norm = std::hypot(fitted[0], fitted[1]);
        if (!positive_finite(norm) || !std::isfinite(fitted[2]) || !std::isfinite(fitted[3])) return false;
        lines[i] = {{fitted[2], fitted[3]}, {fitted[0] / norm, fitted[1] / norm}};
    }
    std::array<cv::Point2d, 4> corners;
    for (size_t i = 0; i < 4; ++i) {
        const auto& current = lines[i];
        const auto& previous = lines[(i + 3) % 4];
        // A uniform scale requires approximately perpendicular and parallel edge pairs.
        if (std::abs(current.direction.dot(previous.direction)) > params.max_relative_error ||
            std::abs(cross(current.direction, lines[(i + 2) % 4].direction)) > params.max_relative_error) return false;
        const double denominator = cross(previous.direction, current.direction);
        if (std::abs(denominator) < 1e-9) return false;
        const double t = cross(current.origin - previous.origin, current.direction) / denominator;
        corners[i] = previous.origin + t * previous.direction;
        if (!std::isfinite(corners[i].x) || !std::isfinite(corners[i].y) || corners[i].x < 0.0 || corners[i].y < 0.0 ||
            corners[i].x >= result.image_size.width || corners[i].y >= result.image_size.height) return false;
    }
    std::array<double, 4> lengths;
    for (size_t i = 0; i < 4; ++i) {
        lengths[i] = cv::norm(corners[(i + 1) % 4] - corners[i]);
        if (!positive_finite(lengths[i])) return false;
    }
    for (size_t i = 0; i < 2; ++i) {
        if (std::abs(lengths[i] - lengths[i + 2]) / (0.5 * lengths[i] + 0.5 * lengths[i + 2]) >
            params.max_relative_error) return false;
    }
    double width = 0.5 * (lengths[0] + lengths[2]);
    double height = 0.5 * (lengths[1] + lengths[3]);
    const double observed_ratio = std::log(width) - std::log(height);
    const double physical_ratio = std::log(params.width_mm) - std::log(params.height_mm);
    // Match the unlabeled side pair to the caller's dimensions, regardless of rotation or width<height.
    if (std::abs(-observed_ratio - physical_ratio) < std::abs(observed_ratio - physical_ratio)) {
        std::swap(width, height);
        std::rotate(corners.begin(), corners.begin() + 1, corners.end());
    }

    double squared_error = 0.0;
    for (const auto& point : contour) {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto& line : lines) {
            nearest = std::min(nearest, std::abs(cross(line.direction, cv::Point2d(point) - line.origin)));
        }
        squared_error += nearest * nearest;
        result.contour.emplace_back(point);
    }
    result.fit_error = std::sqrt(squared_error / contour.size()) / (0.5 * width + 0.5 * height);
    result.width_px = width;
    result.height_px = height;
    result.width_mm = params.width_mm;
    result.height_mm = params.height_mm;
    cv::Point2d center(0.0, 0.0);
    for (size_t i = 0; i < 4; ++i) {
        result.corners[i] = cv::Point2f(corners[i]);
        center += 0.25 * corners[i];
    }
    result.center = cv::Point2f(center);
    const auto width_direction = corners[1] - corners[0];
    result.orientation = std::atan2(width_direction.y, width_direction.x) * 180.0 / kPi;
    if (result.orientation < 0.0) result.orientation += 180.0;
    if (result.orientation >= 180.0) result.orientation -= 180.0;
    return std::isfinite(result.fit_error) && result.fit_error <= params.max_relative_error;
}

} // namespace

int calibrate_image(const CHImage& src, const Image_Calibration_Params& params,
    Image_Calibration_Result& result) {
    if (!valid_calibration_params(params)) return ISO_InvalidParameter;
    Image_Sharpness_Orientations_Outer_Edge_Params detection;
    detection.detect_bright_area = params.detect_white;
    detection.threshold = params.threshold;
    detection.min_area = params.min_area;
    cv::Mat gray;
    result.shape = params.shape;
    if (params.shape == Image_Calibration_Shape::Circle) {
        Image_Sharpness_Orientations_Outer_Edge_Result ellipse;
        const int status = fit_outer_edge(src, detection, ellipse, gray);
        if (status != ISO_OK) return status;
        if (ellipse.fit_error > params.max_relative_error) return ISO_CalibrationTargetInvalid;
        result.image_size = ellipse.image_size;
        result.contour = std::move(ellipse.contour);
        result.center = ellipse.center;
        result.orientation = ellipse.orientation;
        result.width_px = 2.0 * ellipse.a;
        result.height_px = 2.0 * ellipse.b;
        result.width_mm = result.height_mm = params.diameter_mm;
        result.fit_error = ellipse.fit_error;
    } else {
        std::vector<cv::Point> contour;
        const int status = extract_outer_contour(src, detection, contour, gray);
        if (status != ISO_OK) return status;
        result.image_size = src.size();
        if (!fit_rectangle(contour, params, result)) return ISO_CalibrationTargetInvalid;
    }

    const double sw = result.width_mm / result.width_px;
    const double sh = result.height_mm / result.height_px;
    if (!positive_finite(sw) || !positive_finite(sh)) return ISO_InvalidParameter;
    result.mm_per_pixel_width = sw;
    result.mm_per_pixel_height = sh;
    result.scale_disagreement = std::abs(sw - sh) / (0.5 * sw + 0.5 * sh);
    if (result.scale_disagreement > params.max_relative_error) return ISO_CalibrationTargetInvalid;
    // Least-squares uniform length scale, computed without squaring large physical dimensions.
    const double diagonal = std::hypot(result.width_px, result.height_px);
    const double weight = result.width_px / diagonal;
    result.mm_per_pixel = sw * weight * weight + sh * (1.0 - weight * weight);
    result.pixels_per_mm = 1.0 / result.mm_per_pixel;
    if (!positive_finite(result.mm_per_pixel) || !positive_finite(result.pixels_per_mm)) return ISO_InvalidParameter;
    result.valid = true;
    return ISO_OK;
}

} // namespace iso_detail
