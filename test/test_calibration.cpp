#include "main.h"
#include "image_sharpness_orientations.h"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using Calibrator = CH_Image_Calibration;
using Params = Image_Calibration_Params;
using Result = Image_Calibration_Result;
using Shape = Image_Calibration_Shape;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error("calibration: " + message);
}

CHImage circle() {
    CHImage image(480, 640, CV_8UC1, cv::Scalar(20));
    cv::circle(image, {320, 240}, 160, cv::Scalar(220), cv::FILLED);
    return image;
}

CHImage rectangle(cv::Size2f size = {320, 180}, float angle = 0.0f) {
    CHImage image(480, 640, CV_8UC1, cv::Scalar(20));
    cv::Point2f corners[4];
    cv::RotatedRect({320, 240}, size, angle).points(corners);
    std::vector<cv::Point> polygon;
    for (const auto& point : corners) polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    cv::fillConvexPoly(image, polygon, cv::Scalar(220));
    return image;
}

void check_scale(const Result& result, double width_px, double height_px) {
    require(result.valid && result.image_size == cv::Size(640, 480) && !result.contour.empty(),
        "valid calibration and target contour");
    require(cv::norm(result.center - CHPointF(320, 240)) < 1.0, "target center");
    require(std::abs(result.width_px - width_px) < 1.5 && std::abs(result.height_px - height_px) < 1.5,
        "measured diameters or side lengths");
    require(std::abs(result.mm_per_pixel / 0.25 - 1.0) < 0.005 &&
        std::abs(result.pixels_per_mm / 4.0 - 1.0) < 0.005, "known 0.25mm/px calibration");
    require(std::abs(result.mm_per_pixel * result.pixels_per_mm - 1.0) < 1e-12,
        "pixel/mm and mm/pixel are reciprocal");
    require(result.scale_disagreement < 0.01 && result.fit_error < 0.01, "consistent dimensions and good shape fit");
}

void test_circle_calibration() {
    const Calibrator calibrator;
    Params params;
    params.diameter_mm = 80;
    const auto image = circle();
    const auto original = image.clone();
    Result reference, result;
    require(calibrator.Calibrate(image, reference, params) == ISO_OK, "white circle without any registration");
    check_scale(reference, 320, 320);
    require(reference.shape == Shape::Circle && reference.width_mm == 80 && reference.height_mm == 80,
        "diameter is the physical length for both circle directions");
    require(cv::norm(image, original, cv::NORM_INF) == 0, "calibration preserves input pixels");
    for (int threshold : {-1, 120}) {
        CHImage inverted;
        cv::bitwise_not(image, inverted);
        params.detect_white = false;
        params.threshold = threshold;
        require(calibrator.Calibrate(inverted, result, params) == ISO_OK, "black circle, auto/fixed threshold");
        check_scale(result, 320, 320);
        require(std::abs(result.mm_per_pixel - reference.mm_per_pixel) < 0.0001, "polarity independent scale");
    }
    params.detect_white = true;
    params.threshold = -1;
    params.width_mm = std::numeric_limits<double>::quiet_NaN();
    params.height_mm = -1;
    require(calibrator.Calibrate(image, result, params) == ISO_OK, "circle ignores rectangle dimensions");
    CHImage bigger;
    cv::resize(image, bigger, {}, 2.0, 2.0, cv::INTER_NEAREST);
    require(calibrator.Calibrate(bigger, result, params) == ISO_OK &&
        std::abs(result.mm_per_pixel / reference.mm_per_pixel - 0.5) < 0.003,
        "doubling image resolution halves mm/pixel on recalibration");
}

void test_rectangle_and_square_calibration() {
    const Calibrator calibrator;
    Params params;
    params.shape = Shape::Rectangle;
    params.width_mm = 80;
    params.height_mm = 45;
    for (float angle : {0.0f, 22.0f, 90.0f, 140.0f}) {
        const auto image = rectangle({320, 180}, angle);
        for (bool white : {true, false}) {
            CHImage input;
            if (white) input = image;
            else cv::bitwise_not(image, input);
            params.detect_white = white;
            Result result;
            require(calibrator.Calibrate(input, result, params) == ISO_OK, "rotated white/black rectangle");
            check_scale(result, 320, 180);
            require(result.shape == Shape::Rectangle &&
                std::abs(std::remainder(result.orientation - angle, 180.0)) < 0.5,
                "rectangle width direction follows known rotation");
            for (size_t i = 0; i < 4; ++i) {
                const double expected = i % 2 == 0 ? 320 : 180;
                require(std::abs(cv::norm(result.corners[(i + 1) % 4] - result.corners[i]) - expected) < 1.5,
                    "fitted edge intersections correspond to physical width and height");
            }
        }
    }
    params.detect_white = true;
    params.width_mm = 45;
    params.height_mm = 80;
    Result result;
    require(calibrator.Calibrate(rectangle({320, 180}, 33), result, params) == ISO_OK,
        "caller can supply width smaller than height");
    check_scale(result, 180, 320);
    params.width_mm = params.height_mm = 60;
    params.diameter_mm = std::numeric_limits<double>::quiet_NaN();
    for (float angle : {0.0f, 35.0f, 87.0f}) {
        require(calibrator.Calibrate(rectangle({240, 240}, angle), result, params) == ISO_OK,
            "square uses rectangle mode with equal physical sides");
        check_scale(result, 240, 240);
    }
}

void test_formats_and_failures() {
    const Calibrator calibrator;
    Params params;
    params.shape = Shape::Rectangle;
    params.width_mm = 80;
    params.height_mm = 45;
    const auto image = rectangle({320, 180}, 25);
    Result valid, result;
    require(calibrator.Calibrate(image, valid, params) == ISO_OK, "reference rectangle calibration");
    for (int conversion : {cv::COLOR_GRAY2BGR, cv::COLOR_GRAY2BGRA}) {
        CHImage color;
        cv::cvtColor(image, color, conversion);
        require(calibrator.Calibrate(color, result, params) == ISO_OK &&
            result.mm_per_pixel == valid.mm_per_pixel, "color calibration matches grayscale");
    }
    CHImage padded(520, 700, CV_8UC1, cv::Scalar(0));
    auto roi = padded(cv::Rect(30, 20, image.cols, image.rows));
    image.copyTo(roi);
    require(!roi.isContinuous() && calibrator.Calibrate(roi, result, params) == ISO_OK &&
        result.mm_per_pixel == valid.mm_per_pixel, "non-contiguous image input");

    const auto reject = [&](const CHImage& input, const Params& settings, int expected) {
        result = valid;
        require(calibrator.Calibrate(input, result, settings) == expected && !result.valid &&
            result.contour.empty() && result.image_size.empty() && result.mm_per_pixel == 0.0 &&
            result.pixels_per_mm == 0.0 && result.corners[0] == CHPointF(), "failed calibration clears prior output");
    };
    reject(CHImage(), params, ISO_ImageIsEmpty);
    reject(CHImage(200, 200, CV_16UC1), params, ISO_UnsupportedImage);
    reject(CHImage(200, 200, CV_8UC2), params, ISO_UnsupportedImage);
    for (int level : {0, 127, 255}) reject(CHImage(200, 200, CV_8UC1, cv::Scalar(level)), params, ISO_NoShapes);
    reject(circle(), params, ISO_CalibrationTargetInvalid);
    Params wrong = params;
    wrong.height_mm = 80;
    reject(image, wrong, ISO_CalibrationTargetInvalid);
    wrong = {};
    wrong.diameter_mm = 80;
    reject(rectangle({240, 240}), wrong, ISO_CalibrationTargetInvalid);
    CHImage ellipse(image.size(), CV_8UC1, cv::Scalar(20));
    cv::ellipse(ellipse, {320, 240}, {180, 120}, 30, 0, 360, cv::Scalar(220), cv::FILLED);
    reject(ellipse, wrong, ISO_CalibrationTargetInvalid);
    for (const auto& polygon : std::vector<std::vector<cv::Point>>{
        {{200, 140}, {420, 140}, {470, 340}, {150, 340}}, // trapezoid
        {{200, 140}, {450, 140}, {400, 340}, {150, 340}}, // parallelogram
        {{320, 100}, {480, 340}, {160, 340}}}) {
        CHImage skew(image.size(), CV_8UC1, cv::Scalar(20));
        cv::fillConvexPoly(skew, polygon, cv::Scalar(220));
        reject(skew, params, ISO_CalibrationTargetInvalid);
    }
    CHImage clipped(image.size(), CV_8UC1, cv::Scalar(20));
    cv::rectangle(clipped, {0, 80, 350, 250}, cv::Scalar(220), cv::FILLED);
    cv::rectangle(clipped, {450, 200, 100, 70}, cv::Scalar(220), cv::FILLED);
    reject(clipped, params, ISO_OuterEdgeClipped);
    wrong = params;
    wrong.detect_white = false;
    reject(image, wrong, ISO_OuterEdgeClipped);
    for (double value : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        wrong = params; wrong.width_mm = value;
        reject(image, wrong, ISO_InvalidParameter);
        wrong = params; wrong.height_mm = value;
        reject(image, wrong, ISO_InvalidParameter);
        wrong = {}; wrong.diameter_mm = value;
        reject(circle(), wrong, ISO_InvalidParameter);
        wrong = params; wrong.min_area = value;
        reject(image, wrong, ISO_InvalidParameter);
        wrong = params; wrong.max_relative_error = value;
        reject(image, wrong, ISO_InvalidParameter);
    }
    for (int value : {-2, 256}) {
        wrong = params; wrong.threshold = value;
        reject(image, wrong, ISO_InvalidParameter);
    }
    wrong = params; wrong.shape = static_cast<Shape>(99);
    reject(image, wrong, ISO_InvalidParameter);
    wrong = params; wrong.max_relative_error = 0.3;
    reject(image, wrong, ISO_InvalidParameter);
    wrong = params; wrong.min_area = 1e9;
    reject(image, wrong, ISO_NoShapes);
    require(calibrator.Calibrate(image, result, params) == ISO_OK && result.mm_per_pixel == valid.mm_per_pixel,
        "calibrator remains usable after failures");
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

void test_calibration() {
    test_circle_calibration();
    test_rectangle_and_square_calibration();
    test_formats_and_failures();
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
