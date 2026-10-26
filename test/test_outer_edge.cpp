#include "main.h"
#include "image_sharpness_orientations.h"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using Detector = CH_Image_Sharpness_Orientations;
using Params = Image_Sharpness_Orientations_Outer_Edge_Params;
using Result = Image_Sharpness_Orientations_Outer_Edge_Result;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error("outer edge: " + message);
}

CHImage make_projection(cv::Size axes = {220, 140}, double angle = 31.0,
    int foreground = 200, int background = 20) {
    CHImage image(600, 800, CV_8UC1, cv::Scalar(background));
    cv::ellipse(image, {400, 300}, axes, angle, 0, 360, cv::Scalar(foreground), cv::FILLED);
    return image;
}

void check_geometry(const Result& result, cv::Size axes, double angle) {
    require(cv::norm(result.center - CHPointF(400, 300)) < 0.6, "fitted center");
    require(std::abs(result.a - axes.width) < 1.0 && std::abs(result.b - axes.height) < 1.0,
        "a and b are major/minor semi-axis lengths in pixels");
    require(result.a >= result.b && result.b > 0.0, "ordered positive axes");
    require(result.orientation >= 0.0 && result.orientation < 180.0, "angle range");
    require(result.fit_error < 0.01, "low residual for a synthetic ellipse");
    require(std::abs(result.axis_ratio - static_cast<double>(axes.height) / axes.width) < 0.01,
        "axis ratio measures ellipse distortion");
    if (axes.width != axes.height) {
        require(result.orientation_valid && std::abs(std::remainder(result.orientation - angle, 180.0)) < 0.5,
            "major-axis angle is clockwise, including vertical ellipses");
    } else {
        require(!result.orientation_valid, "circle has no reliable orientation");
    }
}

void test_geometry_and_quality() {
    const Detector detector;
    Result result;
    for (double angle : {0.0, 31.0, 90.0, 155.0, 179.0}) {
        const auto image = make_projection({220, 140}, angle);
        require(detector.DetectOuterEdge(image, result) == ISO_OK, "unregistered ellipse detection");
        check_geometry(result, {220, 140}, angle);
        require(result.image_size == image.size() && result.contour.size() > 500,
            "returns image dimensions and dense measured outer contour");
        require(std::abs(result.area / (3.14159265358979323846 * 220 * 140) - 1.0) < 0.01,
            "raw contour area follows ellipse area");
        require(std::abs(result.mean_brightness - 200.0f) < 1.0f &&
            std::abs(result.contrast - 180.0f) < 5.0f, "original-pixel brightness and outer-edge contrast");
        require(std::isfinite(result.sharpness) && result.sharpness > 0.0f, "positive sharpness");
    }

    const auto circle = make_projection({170, 170}, 0.0);
    require(detector.DetectOuterEdge(circle, result) == ISO_OK, "circle detection");
    check_geometry(result, {170, 170}, 0.0);

    const auto large = make_projection({360, 260}, 0.0);
    require(detector.DetectOuterEdge(large, result) == ISO_OK && result.area > 0.5 * large.total(),
        "large projection is not limited by the internal-part area filter");
    check_geometry(result, {360, 260}, 0.0);

    const auto plain = make_projection();
    Result reference;
    require(detector.DetectOuterEdge(plain, reference) == ISO_OK, "plain ellipse");
    auto marked = plain.clone();
    cv::rectangle(marked, {330, 250, 35, 80}, cv::Scalar(20), cv::FILLED);
    cv::circle(marked, {440, 310}, 30, cv::Scalar(20), cv::FILLED);
    cv::circle(marked, {440, 310}, 10, cv::Scalar(200), cv::FILLED);
    const auto original = marked.clone();
    require(detector.DetectOuterEdge(marked, result) == ISO_OK, "internal marks and nested islands");
    require(result.contour == reference.contour && result.a == reference.a && result.b == reference.b,
        "internal edges never enter the outer ellipse fit");
    require(result.sharpness == reference.sharpness && result.contrast == reference.contrast,
        "internal mark sharpness and contrast do not affect outer-edge measurements");
    require(result.mean_brightness < reference.mean_brightness,
        "projection brightness includes internal marks");
    require(cv::norm(marked, original, cv::NORM_INF) == 0, "input pixels are preserved");

    CHImage blurred;
    cv::GaussianBlur(plain, blurred, {0, 0}, 4.0);
    require(detector.DetectOuterEdge(blurred, result) == ISO_OK, "blurred ellipse detection");
    require(result.sharpness < 0.8f * reference.sharpness, "outer-edge sharpness decreases with blur");
    require(std::abs(result.a - reference.a) < 2.0 && std::abs(result.b - reference.b) < 2.0,
        "moderate blur preserves ellipse dimensions");

    auto multiple = plain.clone();
    cv::circle(multiple, {65, 65}, 30, cv::Scalar(220), cv::FILLED);
    require(detector.DetectOuterEdge(multiple, result) == ISO_OK && result.contour == reference.contour,
        "largest external contour is selected");

    CHImage rectangle(600, 800, CV_8UC1, cv::Scalar(20));
    cv::rectangle(rectangle, {220, 120, 360, 360}, cv::Scalar(200), cv::FILLED);
    require(detector.DetectOuterEdge(rectangle, result) == ISO_OK && result.fit_error > 0.05,
        "fit error reveals a non-elliptical boundary even when its axis ratio is near one");

    Result weaker, stronger;
    Params params;
    params.threshold = 110;
    require(detector.DetectOuterEdge(make_projection({220, 140}, 31, 140, 80), weaker, params) == ISO_OK,
        "known lower-contrast edge");
    params.threshold = 140;
    require(detector.DetectOuterEdge(make_projection({220, 140}, 31, 200, 80), stronger, params) == ISO_OK,
        "known higher-contrast edge");
    require(std::abs(stronger.sharpness / weaker.sharpness - 4.0f) < 0.01f,
        "doubling contrast quadruples Tenengrad energy");
}

void test_polarity_formats_and_state() {
    Detector detector;
    auto image = make_projection();
    cv::rectangle(image, {330, 250, 35, 80}, cv::Scalar(20), cv::FILLED);
    cv::circle(image, {440, 310}, 30, cv::Scalar(20), cv::FILLED);
    Result bright, current;
    require(detector.DetectOuterEdge(image, bright) == ISO_OK, "outer detection before registration");
    Image_Sharpness_Orientations_Detect_Result detected;
    require(detector.Detect(image, detected) == ISO_BaseNotRegistered,
        "outer detection does not implicitly register a base");

    Image_Sharpness_Orientations_Register_Result base;
    require(detector.RegisterBase(image, base) == ISO_OK && base.shapes.size() == 2,
        "register internal marks");
    CHImage inverted;
    cv::bitwise_not(image, inverted);
    Params dark;
    dark.detect_bright_area = false;
    require(detector.DetectOuterEdge(inverted, current, dark) == ISO_OK, "dark projection on bright background");
    require(cv::norm(current.center - bright.center) < 0.1 && std::abs(current.a - bright.a) < 0.1 &&
        std::abs(current.b - bright.b) < 0.1, "inversion preserves fitted geometry");
    require(std::abs(current.mean_brightness + bright.mean_brightness - 255.0f) < 0.1f &&
        std::abs(current.contrast - bright.contrast) < 0.1f &&
        std::abs(current.sharpness / bright.sharpness - 1.0f) < 0.01f,
        "brightness inverts and edge quality is polarity independent");
    dark.threshold = 100;
    require(detector.DetectOuterEdge(inverted, current, dark) == ISO_OK, "fixed threshold for dark projection");

    for (int conversion : {cv::COLOR_GRAY2BGR, cv::COLOR_GRAY2BGRA}) {
        CHImage color;
        cv::cvtColor(image, color, conversion);
        require(detector.DetectOuterEdge(color, current) == ISO_OK && current.a == bright.a &&
            current.sharpness == bright.sharpness, "BGR/BGRA match grayscale measurements");
    }
    CHImage padded(image.rows + 60, image.cols + 70, CV_8UC1, cv::Scalar(0));
    auto roi = padded(cv::Rect(30, 25, image.cols, image.rows));
    image.copyTo(roi);
    require(!roi.isContinuous() && detector.DetectOuterEdge(roi, current) == ISO_OK &&
        current.contour == bright.contour && current.sharpness == bright.sharpness,
        "non-contiguous input uses coordinates relative to the supplied image");

    require(detector.DetectOuterEdge(CHImage(), current) == ISO_ImageIsEmpty, "failed outer detection");
    require(detector.Detect(image, detected) == ISO_OK && detected.all_matched && detected.matched_count == 2,
        "successful and failed outer calls leave the registered base and polarity unchanged");
    for (size_t i = 0; i < base.shapes.size(); ++i) {
        require(detected.shapes[i].id == base.shapes[i].id &&
            detected.shapes[i].sharpness == base.shapes[i].sharpness, "existing part measurements are unchanged");
    }
}

void test_failures() {
    const Detector detector;
    Result valid;
    const auto image = make_projection();
    require(detector.DetectOuterEdge(image, valid) == ISO_OK, "seed successful output");
    const auto reject = [&](const CHImage& input, const Params& params, int expected) {
        auto result = valid;
        require(detector.DetectOuterEdge(input, result, params) == expected, "expected error code");
        require(result.contour.empty() && result.image_size.empty() && result.a == 0.0 && result.b == 0.0 &&
            result.center == CHPointF() && result.area == 0.0 && result.sharpness == 0.0f &&
            !result.orientation_valid, "failure clears stale geometry and quality");
    };
    reject(CHImage(), {}, ISO_ImageIsEmpty);
    CHImage image16;
    image.convertTo(image16, CV_16U);
    reject(image16, {}, ISO_UnsupportedImage);
    reject(CHImage(100, 100, CV_8UC2), {}, ISO_UnsupportedImage);
    const int sizes[] = {3, 10, 10};
    reject(CHImage(3, sizes, CV_8UC1), {}, ISO_UnsupportedImage);
    for (int threshold : {-2, 256}) {
        Params params;
        params.threshold = threshold;
        reject(image, params, ISO_InvalidParameter);
    }
    for (double area : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity()}) {
        Params params;
        params.min_area = area;
        reject(image, params, ISO_InvalidParameter);
    }
    for (int level : {0, 127, 255}) {
        reject(CHImage(image.size(), CV_8UC1, cv::Scalar(level)), {}, ISO_NoShapes);
    }
    Params params;
    params.min_area = 1e9;
    reject(image, params, ISO_NoShapes);
    params = {};
    params.threshold = 255;
    reject(image, params, ISO_NoShapes);

    CHImage clipped(image.size(), CV_8UC1, cv::Scalar(20));
    cv::ellipse(clipped, {20, 300}, {220, 140}, 0, 0, 360, cv::Scalar(200), cv::FILLED);
    cv::circle(clipped, {600, 300}, 30, cv::Scalar(200), cv::FILLED);
    reject(clipped, {}, ISO_OuterEdgeClipped);
    params = {};
    params.detect_bright_area = false;
    reject(image, params, ISO_OuterEdgeClipped);
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

void test_outer_edge() {
    test_geometry_and_quality();
    test_polarity_formats_and_state();
    test_failures();
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
