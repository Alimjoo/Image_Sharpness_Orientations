#include "quality_measurement.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace iso_detail {
namespace {

constexpr int kContourBand = 2;
constexpr int kBackgroundRadius = 5;

} // namespace

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

} // namespace iso_detail
