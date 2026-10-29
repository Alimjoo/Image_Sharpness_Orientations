#include "outer_edge.h"
#include "image_sharpness_orientations_core.h"
#include "quality_measurement.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace iso_detail {

int extract_outer_contour(const CHImage& src, const Image_Sharpness_Orientations_Outer_Edge_Params& params,
    std::vector<cv::Point>& contour, cv::Mat& gray) {
    if (params.threshold < -1 || params.threshold > 255 ||
        !std::isfinite(params.min_area) || params.min_area <= 0.0) return ISO_InvalidParameter;
    if (src.empty()) return ISO_ImageIsEmpty;
    if (src.dims != 2 || src.depth() != CV_8U ||
        (src.channels() != 1 && src.channels() != 3 && src.channels() != 4)) {
        return ISO_UnsupportedImage;
    }

    if (src.channels() == 1) gray = src;
    else cv::cvtColor(src, gray, src.channels() == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);
    double minimum = 0.0, maximum = 0.0;
    cv::minMaxLoc(gray, &minimum, &maximum);
    if (minimum == maximum) return ISO_NoShapes;

    // Smooth for segmentation only. Internal holes/marks do not contribute fit points.
    cv::Mat filtered, mask;
    cv::GaussianBlur(gray, filtered, cv::Size(3, 3), 0.0);
    const int threshold_type = params.detect_bright_area ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
    cv::threshold(filtered, mask, std::max(params.threshold, 0), 255,
        threshold_type | (params.threshold < 0 ? cv::THRESH_OTSU : 0));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty()) return ISO_NoShapes;
    const auto largest = std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
    const double area = cv::contourArea(*largest);
    if (area < params.min_area) return ISO_NoShapes;
    const cv::Rect bounds = cv::boundingRect(*largest);
    // Do not fall back to smaller objects when the main projection is cropped.
    if (bounds.x <= 0 || bounds.y <= 0 ||
        bounds.x + bounds.width >= gray.cols || bounds.y + bounds.height >= gray.rows) {
        return ISO_OuterEdgeClipped;
    }
    contour = *largest;
    return ISO_OK;
}

int fit_outer_edge(const CHImage& src, const Image_Sharpness_Orientations_Outer_Edge_Params& params,
    Image_Sharpness_Orientations_Outer_Edge_Result& result, cv::Mat& gray) {
    std::vector<cv::Point> contour;
    const int status = extract_outer_contour(src, params, contour, gray);
    if (status != ISO_OK) return status;
    if (contour.size() < 5) return ISO_EllipseFitFailed;

    cv::RotatedRect ellipse;
    try {
        ellipse = cv::fitEllipse(contour);
    } catch (const cv::Exception&) {
        return ISO_EllipseFitFailed;
    }
    if (!std::isfinite(ellipse.center.x) || !std::isfinite(ellipse.center.y) ||
        !std::isfinite(ellipse.size.width) || !std::isfinite(ellipse.size.height) ||
        !std::isfinite(ellipse.angle) || ellipse.size.width <= 0.0f || ellipse.size.height <= 0.0f) {
        return ISO_EllipseFitFailed;
    }

    result.image_size = src.size();
    result.bounds = cv::boundingRect(contour);
    result.area = cv::contourArea(contour);
    result.center = ellipse.center;
    result.a = 0.5 * std::max(ellipse.size.width, ellipse.size.height);
    result.b = 0.5 * std::min(ellipse.size.width, ellipse.size.height);
    // OpenCV's angle refers to the rectangle's width axis; expose the major axis.
    result.orientation = std::fmod(ellipse.angle +
        (ellipse.size.height > ellipse.size.width ? 90.0 : 0.0), 180.0);
    if (result.orientation < 0.0) result.orientation += 180.0;
    result.axis_ratio = result.b / result.a;
    const double ratio_squared = result.axis_ratio * result.axis_ratio;
    result.orientation_valid = (1.0 - ratio_squared) / (1.0 + ratio_squared) >= 0.08;

    const double radians = result.orientation * kPi / 180.0;
    const double c = std::cos(radians), s = std::sin(radians);
    double squared_error = 0.0;
    result.contour.reserve(contour.size());
    for (const auto& point : contour) {
        result.contour.emplace_back(point);
        const double dx = point.x - static_cast<double>(result.center.x);
        const double dy = point.y - static_cast<double>(result.center.y);
        const double x = (c * dx + s * dy) / result.a;
        const double y = (-s * dx + c * dy) / result.b;
        const double error = std::hypot(x, y) - 1.0;
        squared_error += error * error;
    }
    result.fit_error = std::sqrt(squared_error / static_cast<double>(contour.size()));
    return ISO_OK;
}

int detect_outer_edge(const CHImage& src, const Image_Sharpness_Orientations_Outer_Edge_Params& params,
    Image_Sharpness_Orientations_Outer_Edge_Result& result) {
    cv::Mat gray;
    const int status = fit_outer_edge(src, params, result, gray);
    if (status != ISO_OK) return status;
    std::vector<cv::Point> contour;
    contour.reserve(result.contour.size());
    for (const auto& point : result.contour) contour.emplace_back(point);
    measure_outer_quality(gray, contour, result);
    return ISO_OK;
}

} // namespace iso_detail
