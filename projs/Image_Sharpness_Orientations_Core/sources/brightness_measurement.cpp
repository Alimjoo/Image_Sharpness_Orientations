#include "brightness_measurement.h"
#include "image_sharpness_orientations_core.h"
#include "outer_edge.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace iso_detail {
namespace {

using Mode = Image_Sharpness_Orientations_Brightness_Mode;
using PointStatus = Image_Sharpness_Orientations_Brightness_Point_Status;
using Params = Image_Sharpness_Orientations_Brightness_Params;
using PointResult = Image_Sharpness_Orientations_Brightness_Point;

bool valid_brightness_params(const Params& params) {
    if (params.spot_diameter_px < 1) return false;
    if (params.mode == Mode::Manual) {
        if (params.points.size() != 9) return false;
        for (const auto& point : params.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
        }
        return true;
    }
    if (params.mode != Mode::Auto && params.mode != Mode::AvoidBlack) return false;
    if (params.inset_px < 0) return false;
    return params.mode != Mode::AvoidBlack ||
        (params.black_threshold >= -1 && params.black_threshold <= 255 && params.search_radius_px >= 0);
}

PointStatus sample_disk(const cv::Mat& gray, const cv::Mat& allowed, const CHPointF& center,
    double radius, float& brightness, int& count) {
    const double cx = center.x, cy = center.y;
    // Reject rather than clip, so all nine measurements retain the requested footprint.
    if (cx - radius < 0.0 || cy - radius < 0.0 ||
        cx + radius > gray.cols - 1.0 || cy + radius > gray.rows - 1.0) return PointStatus::OutsideImage;
    double sum = 0.0;
    int pixels = 0;
    const double radius_squared = radius * radius;
    for (int y = static_cast<int>(std::ceil(cy - radius)); y <= static_cast<int>(std::floor(cy + radius)); ++y) {
        const auto* row = gray.ptr<unsigned char>(y);
        const auto* permitted = allowed.empty() ? nullptr : allowed.ptr<unsigned char>(y);
        for (int x = static_cast<int>(std::ceil(cx - radius)); x <= static_cast<int>(std::floor(cx + radius)); ++x) {
            const double dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy > radius_squared) continue;
            if (permitted && permitted[x] == 0) return PointStatus::NoBrightArea;
            sum += row[x];
            ++pixels;
        }
    }
    if (pixels == 0) return PointStatus::NoPixels;
    brightness = static_cast<float>(sum / pixels);
    count = pixels;
    return PointStatus::Valid;
}

bool find_nearest_bright_disk(const cv::Mat& gray, const cv::Mat& allowed,
    const Params& params, PointResult& point) {
    if (params.search_radius_px == 0) return false;
    const double radius = params.spot_diameter_px / 2.0;
    const double search = params.search_radius_px;
    const auto& requested = point.requested_center;
    const double left = std::ceil(std::max(radius, requested.x - search));
    const double top = std::ceil(std::max(radius, requested.y - search));
    const double right = std::floor(std::min(gray.cols - 1.0 - radius, requested.x + search));
    const double bottom = std::floor(std::min(gray.rows - 1.0 - radius, requested.y + search));
    if (left > right || top > bottom) return false;

    // Limit the distance transform to this point's search neighborhood plus its disk.
    // Zero padding also treats pixels beyond the image as unavailable.
    const int x0 = static_cast<int>(std::max(0.0, left - std::ceil(radius) - 1));
    const int y0 = static_cast<int>(std::max(0.0, top - std::ceil(radius) - 1));
    const int x1 = static_cast<int>(std::min(static_cast<double>(gray.cols), right + std::ceil(radius) + 2));
    const int y1 = static_cast<int>(std::min(static_cast<double>(gray.rows), bottom + std::ceil(radius) + 2));
    const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
    cv::Mat padded, clearance;
    cv::copyMakeBorder(allowed(roi), padded, 1, 1, 1, 1,
        cv::BORDER_CONSTANT | cv::BORDER_ISOLATED, cv::Scalar(0));
    cv::distanceTransform(padded, clearance, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    double best_distance = std::numeric_limits<double>::infinity();
    for (int y = static_cast<int>(top); y <= static_cast<int>(bottom); ++y) {
        for (int x = static_cast<int>(left); x <= static_cast<int>(right); ++x) {
            const double dx = x - static_cast<double>(requested.x), dy = y - static_cast<double>(requested.y);
            const double distance = dx * dx + dy * dy;
            if (distance > search * search || distance >= best_distance) continue;
            if (clearance.at<float>(y - y0 + 1, x - x0 + 1) <= radius) continue;
            const CHPointF candidate(static_cast<float>(x), static_cast<float>(y));
            float brightness = 0.0f;
            int count = 0;
            // Verify the exact sampling pixels after the fast clearance test.
            if (sample_disk(gray, allowed, candidate, radius, brightness, count) != PointStatus::Valid) continue;
            best_distance = distance;
            point.center = candidate;
            point.mean_brightness = brightness;
            point.pixel_count = count;
        }
    }
    if (!std::isfinite(best_distance)) return false;
    point.moved = point.center != point.requested_center;
    return true;
}

} // namespace

int detect_brightness(const CHImage& src, const Params& params,
    Image_Sharpness_Orientations_Brightness_Result& result) {
    if (!valid_brightness_params(params)) return ISO_InvalidParameter;
    if (src.empty()) return ISO_ImageIsEmpty;
    if (src.dims != 2 || src.depth() != CV_8U ||
        (src.channels() != 1 && src.channels() != 3 && src.channels() != 4)) return ISO_UnsupportedImage;

    cv::Mat gray, allowed;
    std::array<CHPointF, 9> centers{};
    if (params.mode == Mode::Manual) {
        if (src.channels() == 1) gray = src;
        else cv::cvtColor(src, gray, src.channels() == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);
        std::copy(params.points.begin(), params.points.end(), centers.begin());
    } else {
        Image_Sharpness_Orientations_Outer_Edge_Result ellipse;
        const int status = fit_outer_edge(src, params.outer_edge, ellipse, gray);
        if (status != ISO_OK) return status;
        result.ellipse_valid = true;
        result.ellipse_center = ellipse.center;
        result.ellipse_a = ellipse.a;
        result.ellipse_b = ellipse.b;
        result.ellipse_orientation = ellipse.orientation;
        centers[0] = ellipse.center;
        const double radians = ellipse.orientation * kPi / 180.0;
        const double c = std::cos(radians), s = std::sin(radians);
        // Fixed image directions keep point IDs stable even when a circle's axis is arbitrary.
        for (int i = 0; i < 8; ++i) {
            const double direction = (-90.0 + i * 45.0) * kPi / 180.0;
            const double dx = std::cos(direction), dy = std::sin(direction);
            const double u = (c * dx + s * dy) / ellipse.a;
            const double v = (-s * dx + c * dy) / ellipse.b;
            const double boundary_distance = 1.0 / std::hypot(u, v);
            if (params.inset_px >= boundary_distance) return ISO_InvalidParameter;
            const double distance = boundary_distance - params.inset_px;
            centers[i + 1] = {static_cast<float>(ellipse.center.x + distance * dx),
                static_cast<float>(ellipse.center.y + distance * dy)};
        }
        if (params.mode == Mode::AvoidBlack) {
            result.effective_black_threshold = static_cast<int>(cv::threshold(gray, allowed,
                std::max(params.black_threshold, 0), 255,
                cv::THRESH_BINARY | (params.black_threshold < 0 ? cv::THRESH_OTSU : 0)));
            std::vector<cv::Point> contour;
            for (const auto& point : ellipse.contour) contour.emplace_back(point);
            cv::Mat silhouette = cv::Mat::zeros(gray.size(), CV_8UC1);
            cv::drawContours(silhouette, std::vector<std::vector<cv::Point>>{contour}, 0,
                cv::Scalar(255), cv::FILLED);
            cv::bitwise_and(allowed, silhouette, allowed);
        }
    }

    result.image_size = src.size();
    result.spot_diameter_px = params.spot_diameter_px;
    for (size_t i = 0; i < centers.size(); ++i) {
        auto& point = result.points[i];
        point.id = static_cast<int>(i) + 1;
        point.requested_center = point.center = centers[i];
        point.status = sample_disk(gray, allowed, point.center, params.spot_diameter_px / 2.0,
            point.mean_brightness, point.pixel_count);
        if (params.mode == Mode::AvoidBlack && point.status != PointStatus::Valid) {
            point.status = find_nearest_bright_disk(gray, allowed, params, point)
                ? PointStatus::Valid : PointStatus::NoBrightArea;
        }
        point.valid = point.status == PointStatus::Valid;
        if (point.valid) ++result.valid_count;
    }
    result.all_valid = result.valid_count == 9;
    return result.all_valid ? ISO_OK : ISO_PartialMeasurement;
}

} // namespace iso_detail
