#include "main.h"
#include "image_sharpness_orientations.h"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using Detector = CH_Image_Sharpness_Orientations;
using Params = Image_Sharpness_Orientations_Brightness_Params;
using Result = Image_Sharpness_Orientations_Brightness_Result;
using Mode = Image_Sharpness_Orientations_Brightness_Mode;
using Status = Image_Sharpness_Orientations_Brightness_Point_Status;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error("brightness: " + message);
}

CHImage projection(double angle = 25.0) {
    CHImage image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::ellipse(image, {320, 240}, {240, 160}, angle, 0, 360, cv::Scalar(200), cv::FILLED);
    return image;
}

// Independent reference over all pixel centers in the requested disk.
bool bright_disk(const CHImage& image, CHPointF center, double radius, int threshold) {
    if (center.x - radius < 0 || center.y - radius < 0 ||
        center.x + radius > image.cols - 1 || center.y + radius > image.rows - 1) return false;
    for (int y = static_cast<int>(std::floor(center.y - radius)); y <= std::ceil(center.y + radius); ++y) {
        for (int x = static_cast<int>(std::floor(center.x - radius)); x <= std::ceil(center.x + radius); ++x) {
            if (cv::norm(CHPointF(static_cast<float>(x), static_cast<float>(y)) - center) <= radius &&
                image.at<unsigned char>(y, x) <= threshold) return false;
        }
    }
    return true;
}

void test_automatic_placement() {
    const Detector detector;
    Params params;
    params.inset_px = 30;
    params.spot_diameter_px = 15;
    Result result;
    const auto image = projection();
    require(detector.DetectBrightness(image, result, params) == ISO_OK && result.all_valid && result.valid_count == 9,
        "automatic nine-point measurements without registration");
    require(result.ellipse_valid && result.spot_diameter_px == 15 && result.effective_black_threshold == -1,
        "placement geometry and footprint are reported");
    const double angle = 25.0 * CV_PI / 180.0;
    for (size_t i = 0; i < result.points.size(); ++i) {
        const auto& point = result.points[i];
        require(point.id == static_cast<int>(i) + 1 && point.valid && !point.moved &&
            point.status == Status::Valid && point.center == point.requested_center && point.pixel_count > 0,
            "nine individual stable IDs and valid measurements");
        require(std::abs(point.mean_brightness - 200.0f) < 0.01f, "each white disk has known brightness");
        CHPointF expected(320, 240);
        if (i != 0) {
            const double direction = (-90.0 + (i - 1) * 45.0) * CV_PI / 180.0;
            const double dx = std::cos(direction), dy = std::sin(direction);
            const double u = (std::cos(angle) * dx + std::sin(angle) * dy) / 240.0;
            const double v = (-std::sin(angle) * dx + std::cos(angle) * dy) / 160.0;
            const double distance = 1.0 / std::hypot(u, v) - 30;
            expected += CHPointF(static_cast<float>(distance * dx), static_cast<float>(distance * dy));
        }
        require(cv::norm(point.center - expected) < 1.0, "rotated ellipse ray intersections, clockwise from top");
    }
    const auto first = result;
    params.inset_px += 10;
    require(detector.DetectBrightness(image, result, params) == ISO_OK, "increased inset");
    require(result.points[0].center == first.points[0].center, "inset does not move center point");
    for (size_t i = 1; i < result.points.size(); ++i) {
        require(std::abs(cv::norm(result.points[i].center - first.points[i].center) - 10.0) < 0.001,
            "inset is a pixel distance toward the center");
    }
    CHImage circle(image.size(), CV_8UC1, cv::Scalar(0));
    cv::circle(circle, {320, 240}, 180, cv::Scalar(200), cv::FILLED);
    require(detector.DetectBrightness(circle, result, params) == ISO_OK &&
        std::abs(result.points[1].center.x - 320.0f) < 0.1 && result.points[1].center.y < 240 &&
        std::abs(result.points[3].center.y - 240.0f) < 0.1 && result.points[3].center.x > 320,
        "circle numbering ignores its arbitrary fitted principal-axis angle");
}

void test_manual_values_and_formats() {
    Detector detector;
    Params params;
    params.mode = Mode::Manual;
    params.spot_diameter_px = 12;
    for (int i = 0; i < 9; ++i) params.points.emplace_back(20.0f + 24.0f * i, 100.0f);
    CHImage image(200, 256, CV_8UC1);
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) image.at<unsigned char>(y, x) = static_cast<unsigned char>(x);
    }
    const auto original = image.clone();
    Result reference;
    require(detector.DetectBrightness(image, reference, params) == ISO_OK && !reference.ellipse_valid,
        "manual mode works on images without a projection silhouette");
    for (size_t i = 0; i < params.points.size(); ++i) {
        require(reference.points[i].center == params.points[i] && !reference.points[i].moved &&
            reference.points[i].mean_brightness == params.points[i].x,
            "manual coordinates and nine distinct brightness values are preserved");
    }
    require(cv::norm(image, original, cv::NORM_INF) == 0, "input pixels are preserved");
    for (int conversion : {cv::COLOR_GRAY2BGR, cv::COLOR_GRAY2BGRA}) {
        CHImage color;
        cv::cvtColor(image, color, conversion);
        Result result;
        require(detector.DetectBrightness(color, result, params) == ISO_OK, "color image input");
        for (size_t i = 0; i < result.points.size(); ++i) {
            require(result.points[i].mean_brightness == reference.points[i].mean_brightness,
                "color conversion preserves grayscale brightness");
        }
    }
    CHImage padded(250, 300, CV_8UC1, cv::Scalar(0));
    auto roi = padded(cv::Rect(15, 20, image.cols, image.rows));
    image.copyTo(roi);
    Result result;
    require(!roi.isContinuous() && detector.DetectBrightness(roi, result, params) == ISO_OK &&
        result.points[7].mean_brightness == reference.points[7].mean_brightness, "non-contiguous input");

    // A mixed disk must include dark pixels in manual mode, never silently mask them out.
    CHImage split(200, 256, CV_8UC1, cv::Scalar(40));
    split(cv::Rect(101, 0, 155, 200)).setTo(200);
    params.points.assign(9, CHPointF(100.5f, 100));
    require(detector.DetectBrightness(split, result, params) == ISO_OK && result.points[0].mean_brightness == 120.0f,
        "full subpixel disk is averaged, including its dark half");

    params.points[0] = {2, 100};
    require(detector.DetectBrightness(image, result, params) == ISO_PartialMeasurement &&
        result.valid_count == 8 && !result.all_valid && !result.points[0].valid &&
        result.points[0].status == Status::OutsideImage && result.points[0].pixel_count == 0,
        "out-of-image disk is reported without clipping or losing the other eight values");
    params.points[0] = {1e30f, 100};
    require(detector.DetectBrightness(image, result, params) == ISO_PartialMeasurement,
        "large finite manual coordinates are handled without integer overflow");
    params.spot_diameter_px = 1;
    params.points.assign(9, CHPointF(100, 100));
    params.points[0] = {100.5f, 100.5f};
    require(detector.DetectBrightness(image, result, params) == ISO_PartialMeasurement &&
        result.points[0].status == Status::NoPixels && result.valid_count == 8,
        "empty subpixel footprint has an explicit status");
}

void test_whole_disk_avoidance() {
    Detector detector;
    auto image = projection();
    // The center remains white, but a single dark pixel lies near the sampling disk's rim.
    image.at<unsigned char>(240, 328) = 0;
    Params params;
    params.spot_diameter_px = 21;
    params.black_threshold = 100;
    params.search_radius_px = 30;
    Result automatic, avoided;
    require(detector.DetectBrightness(image, automatic, params) == ISO_OK &&
        automatic.points[0].mean_brightness < 200.0f &&
        image.at<unsigned char>(cvRound(automatic.points[0].center.y), cvRound(automatic.points[0].center.x)) == 200,
        "automatic disk contains a black pixel despite a white center");
    params.mode = Mode::AvoidBlack;
    const auto original = image.clone();
    require(detector.DetectBrightness(image, avoided, params) == ISO_OK && avoided.points[0].moved &&
        avoided.effective_black_threshold == 100, "whole-disk overlap triggers relocation");
    for (size_t i = 0; i < avoided.points.size(); ++i) {
        const auto& point = avoided.points[i];
        require(point.requested_center == automatic.points[i].center &&
            cv::norm(point.center - point.requested_center) <= params.search_radius_px &&
            bright_disk(image, point.center, 10.5, 100) && point.mean_brightness == 200.0f,
            "every pixel in every final disk is bright and movement is bounded");
        if (i != 0) require(!point.moved, "unaffected disks stay at their exact original centers");
    }
    const auto& moved = avoided.points[0];
    const double chosen_distance = cv::norm(moved.center - moved.requested_center);
    for (int y = 210; y <= 270; ++y) {
        for (int x = 290; x <= 350; ++x) {
            const CHPointF candidate(static_cast<float>(x), static_cast<float>(y));
            if (cv::norm(candidate - moved.requested_center) < chosen_distance - 1e-6) {
                require(!bright_disk(image, candidate, 10.5, 100), "replacement is the nearest safe integer center");
            }
        }
    }
    require(cv::norm(image, original, cv::NORM_INF) == 0, "avoidance does not modify source pixels");
    params.search_radius_px = 0;
    require(detector.DetectBrightness(image, avoided, params) == ISO_PartialMeasurement &&
        avoided.valid_count == 8 && avoided.points[0].status == Status::NoBrightArea &&
        !avoided.points[0].moved && avoided.points[0].center == automatic.points[0].center &&
        avoided.points[0].mean_brightness == 0.0f && avoided.points[0].pixel_count == 0,
        "no safe replacement is reported without returning a contaminated measurement");

    // A small white island can fit a point but cannot fit the requested disk.
    cv::circle(image, {320, 240}, 45, cv::Scalar(0), cv::FILLED);
    cv::circle(image, {320, 240}, 4, cv::Scalar(200), cv::FILLED);
    params.search_radius_px = 20;
    require(detector.DetectBrightness(image, avoided, params) == ISO_PartialMeasurement &&
        !avoided.points[0].valid && avoided.valid_count == 8, "bright island smaller than spot diameter is rejected");
    params.search_radius_px = 70;
    params.black_threshold = -1;
    require(detector.DetectBrightness(image, avoided, params) == ISO_OK && avoided.points[0].moved &&
        avoided.effective_black_threshold >= 0 && avoided.effective_black_threshold < 200,
        "larger neighborhood finds white area with automatic black threshold");

    Image_Sharpness_Orientations_Register_Result base;
    Image_Sharpness_Orientations_Detect_Result detected;
    require(detector.RegisterBase(image, base) == ISO_OK, "register state for independence check");
    require(detector.DetectBrightness(image, avoided, params) == ISO_OK &&
        detector.Detect(image, detected) == ISO_OK && detected.all_matched,
        "brightness measurement preserves the registered base");
}

void test_irregular_outline_and_errors() {
    const Detector detector;
    CHImage image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::circle(image, {320, 240}, 180, cv::Scalar(200), cv::FILLED);
    cv::rectangle(image, {300, 50, 40, 70}, cv::Scalar(0), cv::FILLED);
    Params params;
    params.spot_diameter_px = 21;
    params.black_threshold = 100;
    params.search_radius_px = 60;
    Result automatic, result;
    require(detector.DetectBrightness(image, automatic, params) == ISO_OK &&
        automatic.points[1].mean_brightness < 200.0f, "fitted ellipse can place a disk in an actual boundary notch");
    params.mode = Mode::AvoidBlack;
    require(detector.DetectBrightness(image, result, params) == ISO_OK && result.points[1].moved,
        "avoidance moves away from an irregular outer boundary");
    for (const auto& point : result.points) {
        require(bright_disk(image, point.center, 10.5, 100), "notched projection contains every final disk");
    }

    CHImage dark(200, 200, CV_8UC1, cv::Scalar(220));
    cv::circle(dark, {100, 100}, 65, cv::Scalar(30), cv::FILLED);
    params.outer_edge.detect_bright_area = false;
    params.search_radius_px = 200;
    require(detector.DetectBrightness(dark, result, params) == ISO_PartialMeasurement && result.valid_count == 0,
        "bright background outside the selected projection cannot be used as a replacement");

    const auto reject = [&](const CHImage& input, const Params& invalid, int expected) {
        result = automatic;
        require(detector.DetectBrightness(input, result, invalid) == expected && result.image_size.empty() &&
            !result.ellipse_valid && result.valid_count == 0 && result.points[0].id == 0 &&
            result.points[0].status == Status::NotMeasured, "fatal errors clear stale output");
    };
    reject(CHImage(), {}, ISO_ImageIsEmpty);
    reject(CHImage(200, 200, CV_16UC1), {}, ISO_UnsupportedImage);
    reject(CHImage(200, 200, CV_8UC2), {}, ISO_UnsupportedImage);
    reject(CHImage(200, 200, CV_8UC1, cv::Scalar(180)), {}, ISO_NoShapes);
    params = {};
    params.spot_diameter_px = 0;
    reject(image, params, ISO_InvalidParameter);
    params = {};
    params.inset_px = 10000;
    reject(image, params, ISO_InvalidParameter);
    params.inset_px = -1;
    reject(image, params, ISO_InvalidParameter);
    params = {};
    params.mode = static_cast<Mode>(99);
    reject(image, params, ISO_InvalidParameter);
    params.mode = Mode::AvoidBlack;
    params.black_threshold = 256;
    reject(image, params, ISO_InvalidParameter);
    params.black_threshold = -1;
    params.search_radius_px = -1;
    reject(image, params, ISO_InvalidParameter);
    params = {};
    params.mode = Mode::Manual;
    params.points.assign(8, CHPointF(100, 100));
    reject(image, params, ISO_InvalidParameter);
    params.points.assign(10, CHPointF(100, 100));
    reject(image, params, ISO_InvalidParameter);
    params.points.assign(9, CHPointF(100, 100));
    params.points[0].x = std::numeric_limits<float>::quiet_NaN();
    reject(image, params, ISO_InvalidParameter);
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

void test_brightness() {
    test_automatic_placement();
    test_manual_values_and_formats();
    test_whole_disk_avoidance();
    test_irregular_outline_and_errors();
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
