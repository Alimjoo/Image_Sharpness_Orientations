#include "image_sharpness_orientations_core.h"
#include "quality_measurement.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <utility>

namespace iso_detail {
namespace {

constexpr int kBlurSize = 3;
constexpr double kMaxAreaRatio = 0.25;
constexpr double kMinOrientationAnisotropy = 0.08;

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

} // namespace

bool valid_params(const Image_Sharpness_Orientations_Params& p) {
    return p.threshold >= -1 && p.threshold <= 255 &&
        std::isfinite(p.min_area) && p.min_area > 0.0;
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
    // Whiten the silhouette covariance: remove independent x/y scale and shear,
    // leaving only a rotation between affine views of the same product. Using
    // the outer silhouette makes this independent of missing or extra marks.
    const double xx = product_moments.mu20 / frame.area;
    const double xy = product_moments.mu11 / frame.area;
    const double yy = product_moments.mu02 / frame.area;
    const double root_det = std::sqrt(std::max(0.0, xx * yy - xy * xy));
    if (root_det > 1e-9) {
        frame.normalization = cv::Matx22d(yy + root_det, -xy, -xy, xx + root_det) *
            (1.0 / (2.0 * std::sqrt(kPi) * root_det * std::sqrt(xx + yy + 2.0 * root_det)));
    } else {
        frame.normalization = cv::Matx22d::eye() * (1.0 / std::sqrt(frame.area));
    }

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

} // namespace iso_detail
