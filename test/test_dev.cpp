#include "main.h"
#include "image_sharpness_orientations.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {

using RegisterResult = Image_Sharpness_Orientations_Register_Result;
using DetectResult = Image_Sharpness_Orientations_Detect_Result;
using Params = Image_Sharpness_Orientations_Params;
using Detector = CH_Image_Sharpness_Orientations;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

CHImage make_product() {
    CHImage image(640, 900, CV_8UC1, cv::Scalar(0));
    cv::ellipse(image, {450, 320}, {415, 290}, 0, 0, 360, cv::Scalar(185), cv::FILLED);
    for (int i = 0; i < 3; ++i) {
        const cv::Size axes(200 - 55 * i, 155 - 35 * i);
        cv::ellipse(image, {360, 300}, axes, 0, 135, 225, cv::Scalar(10), 13);
        cv::ellipse(image, {540, 300}, axes, 0, -45, 45, cv::Scalar(10), 13);
    }
    cv::ellipse(image, {450, 225}, {57, 100}, 15, 0, 360, cv::Scalar(10), 15);
    cv::ellipse(image, {450, 445}, {57, 43}, 0, 0, 360, cv::Scalar(10), 15);
    return image;
}

double average_sharpness(const Image_Sharpness_Orientations_Shapes& shapes) {
    double sum = 0.0;
    for (const auto& shape : shapes) sum += shape.sharpness;
    return sum / static_cast<double>(shapes.size());
}

void test_registration_and_motion() {
    Detector detector;
    RegisterResult base;
    DetectResult current;
    const auto image = make_product();
    const auto original = image.clone();
    require(detector.RegisterBase(image, base) == ISO_OK, "register synthetic base");
    require(base.shapes.size() == 8, "exactly eight dark parts; exclude holes/background");
    require(cv::norm(image, original, cv::NORM_INF) == 0, "registration must not modify input");
    require(detector.Detect(image, current) == ISO_OK && current.all_matched, "identical image matches");
    for (size_t i = 0; i < base.shapes.size(); ++i) {
        require(base.shapes[i].id == static_cast<int>(i) + 1, "base IDs start at one");
        require(current.shapes[i].id == base.shapes[i].id, "identical IDs");
        require(std::abs(current.shapes[i].area - base.shapes[i].area) < 1e-8, "identical contour area");
        require(base.shapes[i].sharpness > 0.0, "positive edge sharpness");
    }

    const double clockwise_angle = 8.0;
    const double scale = 1.04;
    auto transform = cv::getRotationMatrix2D({450.0f, 320.0f}, -clockwise_angle, scale);
    transform.at<double>(0, 2) += 18.0;
    transform.at<double>(1, 2) -= 12.0;
    CHImage transformed;
    cv::warpAffine(image, transformed, transform, image.size(), cv::INTER_LINEAR);
    const int motion_status = detector.Detect(transformed, current);
    if (motion_status != ISO_OK || !current.all_matched) {
        std::cerr << "Motion status=" << motion_status << " matched=" << current.matched_count
            << " parts=" << current.shapes.size() << '\n';
        for (const auto& shape : current.shapes) {
            std::cerr << "id=" << shape.id << " center=" << shape.center << " area=" << shape.area
                << " axis=" << shape.orientation << " cost=" << shape.match_cost << '\n';
        }
    }
    require(motion_status == ISO_OK && current.all_matched,
        "translated, scaled, rotated parts keep their IDs");
    require(current.angle_valid && std::abs(current.angle - clockwise_angle) < 0.5,
        "whole-pattern clockwise angle");
    for (size_t i = 0; i < base.shapes.size(); ++i) {
        const auto& source = base.shapes[i];
        const auto& found = current.shapes[i];
        const cv::Point2d expected(
            transform.at<double>(0, 0) * source.center.x + transform.at<double>(0, 1) * source.center.y + transform.at<double>(0, 2),
            transform.at<double>(1, 0) * source.center.x + transform.at<double>(1, 1) * source.center.y + transform.at<double>(1, 2));
        require(found.id == source.id && cv::norm(expected - cv::Point2d(found.center)) < 2.0,
            "each ID follows its own transformed shape");
        require(std::abs(found.area / source.area - scale * scale) < 0.07, "area follows scale squared");
        require(found.angle_valid && std::abs(found.angle - clockwise_angle) < 2.0, "each shape rotation");
    }

    // A changed product outline must not rotate the printed-part identities.
    CHImage changed_outline(image.size(), CV_8UC1, cv::Scalar(0));
    cv::ellipse(changed_outline, {450, 320}, {380, 330}, 45, 0, 360, cv::Scalar(185), cv::FILLED);
    cv::Mat part_mask = cv::Mat::zeros(image.size(), CV_8UC1);
    for (const auto& shape : base.shapes) {
        std::vector<cv::Point> contour;
        for (const auto& point : shape.contour) contour.emplace_back(point);
        cv::fillPoly(part_mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));
    }
    cv::dilate(part_mask, part_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5}));
    image.copyTo(changed_outline, part_mask);
    require(detector.Detect(changed_outline, current) == ISO_OK && current.all_matched &&
        current.angle_valid && std::abs(current.angle) < 0.1,
        "changed product silhouette does not dictate printed-part rotation");

    CHImage blurred;
    cv::GaussianBlur(image, blurred, {0, 0}, 4.0);
    require(detector.Detect(blurred, current) == ISO_OK && current.all_matched, "blurred parts still match");
    require(average_sharpness(current.shapes) < 0.80 * average_sharpness(base.shapes), "blur decreases edge sharpness");

    CHImage changed = image.clone();
    std::vector<cv::Point> removed_contour;
    for (const auto& point : base.shapes.front().contour) removed_contour.emplace_back(point);
    cv::Mat removal_mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::fillPoly(removal_mask, std::vector<std::vector<cv::Point>>{removed_contour}, cv::Scalar(255));
    cv::dilate(removal_mask, removal_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7}));
    changed.setTo(cv::Scalar(185), removal_mask);
    cv::circle(changed, {500, 565}, 12, cv::Scalar(10), cv::FILLED);
    require(detector.Detect(changed, current) == ISO_OK, "missing/extra image is analyzable");
    require(!current.all_matched && current.matched_count == 7 && current.missing_ids == std::vector<int>{1},
        "report missing base ID");
    require(current.shapes.size() == 8 && current.shapes.back().id == -1 &&
        !current.shapes.back().matched && !current.shapes.back().angle_valid, "report extra unmatched part");

    base.shapes.clear();
    require(detector.Detect(image, current) == ISO_OK && current.shapes.size() == 8,
        "caller result does not own internal base state");
}

void test_dark_and_light_parts() {
    const auto dark_image = make_product();
    CHImage light_image;
    cv::bitwise_not(dark_image, light_image);
    const auto original_light = light_image.clone();

    // Test both Otsu and explicit thresholds with opposite-polarity product/exterior.
    for (int threshold : {-1, 100}) {
        Detector dark_detector, light_detector;
        Params dark_params, light_params;
        dark_params.threshold = threshold;
        light_params.detect_dark_area = false;
        // For 8-bit inversion, x <= T is equivalent to (255 - x) > (254 - T).
        light_params.threshold = threshold < 0 ? -1 : 254 - threshold;

        RegisterResult dark_base, light_base;
        DetectResult dark_current, light_current;
        require(dark_detector.RegisterBase(dark_image, dark_base, dark_params) == ISO_OK &&
            dark_base.shapes.size() == 8, "register dark parts with selected threshold");
        require(light_detector.RegisterBase(light_image, light_base, light_params) == ISO_OK &&
            light_base.shapes.size() == 8, "register light parts; exclude bright exterior and dark holes");
        require(cv::norm(light_image, original_light, cv::NORM_INF) == 0, "light registration preserves input");
        for (size_t i = 0; i < dark_base.shapes.size(); ++i) {
            const auto& a = dark_base.shapes[i];
            const auto& b = light_base.shapes[i];
            require(a.id == b.id && cv::norm(a.center - b.center) < 0.5, "polarity preserves part identity");
            require(std::abs(a.area / b.area - 1.0) < 0.01, "polarity preserves outer contour area");
            require(std::abs(a.mean_brightness + b.mean_brightness - 255.0f) < 0.5f,
                "inverting the image inverts foreground brightness");
            require(std::abs(a.contrast - b.contrast) < 0.5f, "contrast is independent of polarity");
            require(std::abs(a.sharpness - b.sharpness) < 0.02f * std::max(a.sharpness, b.sharpness),
                "Tenengrad energy is independent of polarity");
        }

        // Caller edits after registration must not change the stored detection polarity.
        light_params.detect_dark_area = true;
        light_params.min_area = 1e9;
        require(light_detector.Detect(light_image, light_current) == ISO_OK && light_current.all_matched,
            "Detect reuses the registered light-area parameters");

        const double clockwise_angle = -6.0;
        auto transform = cv::getRotationMatrix2D({450.0f, 320.0f}, -clockwise_angle, 1.0);
        transform.at<double>(0, 2) += 15.0;
        CHImage dark_moved, light_moved;
        cv::warpAffine(dark_image, dark_moved, transform, dark_image.size(), cv::INTER_LINEAR);
        cv::bitwise_not(dark_moved, light_moved);
        require(dark_detector.Detect(dark_moved, dark_current) == ISO_OK && dark_current.all_matched,
            "dark parts match after motion");
        require(light_detector.Detect(light_moved, light_current) == ISO_OK && light_current.all_matched &&
            light_current.matched_count == 8, "light parts match after motion");
        require(light_current.angle_valid && std::abs(light_current.angle - clockwise_angle) < 0.5,
            "light-part whole-pattern angle");
        for (size_t i = 0; i < dark_current.shapes.size(); ++i) {
            require(dark_current.shapes[i].id == light_current.shapes[i].id &&
                cv::norm(dark_current.shapes[i].center - light_current.shapes[i].center) < 0.5,
                "light mode follows the same transformed part IDs");
            require(light_current.shapes[i].angle_valid &&
                std::abs(light_current.shapes[i].angle - clockwise_angle) < 2.0, "light-part relative angle");
        }

        CHImage light_blurred;
        cv::GaussianBlur(light_image, light_blurred, {0, 0}, 4.0);
        require(light_detector.Detect(light_blurred, light_current) == ISO_OK && light_current.all_matched,
            "blurred light parts still match");
        require(average_sharpness(light_current.shapes) < 0.80 * average_sharpness(light_base.shapes),
            "blur reduces light-part sharpness");

        CHImage light_color;
        cv::cvtColor(light_image, light_color, cv::COLOR_GRAY2BGRA);
        require(light_detector.Detect(light_color, light_current) == ISO_OK && light_current.all_matched,
            "light mode supports color input");
        for (int level : {0, 255}) {
            require(light_detector.Detect(CHImage(light_image.size(), CV_8UC1, cv::Scalar(level)), light_current)
                == ISO_NoShapes && light_current.missing_ids.size() == 8,
                "uniform image reports all light parts missing");
        }
    }
}

void test_quality_measurements() {
    const auto rectangle_quality = [](int foreground, int background) {
        CHImage image(240, 320, CV_8UC1, cv::Scalar(background));
        cv::rectangle(image, cv::Rect(80, 70, 100, 60), cv::Scalar(foreground), cv::FILLED);
        Params params;
        params.threshold = (foreground + background) / 2;
        Detector detector;
        RegisterResult base;
        DetectResult current;
        require(detector.RegisterBase(image, base, params) == ISO_OK && base.shapes.size() == 1,
            "register known-intensity rectangle");
        require(detector.Detect(image, current) == ISO_OK && current.all_matched,
            "detect known-intensity rectangle");
        const auto& shape = current.shapes.front();
        require(std::abs(shape.mean_brightness - foreground) < 0.01,
            "foreground brightness matches known pixel value");
        require(std::abs(shape.contrast - std::abs(background - foreground)) < 0.01,
            "contrast matches known foreground/background difference");
        require(std::isfinite(shape.sharpness) && shape.sharpness > 0.0f,
            "finite positive Tenengrad energy");
        require(shape.mean_brightness == base.shapes.front().mean_brightness &&
            shape.contrast == base.shapes.front().contrast && shape.sharpness == base.shapes.front().sharpness,
            "registration and detection report the same quality measurements");
        return shape;
    };

    const auto normal = rectangle_quality(80, 140);
    const auto brighter = rectangle_quality(120, 180);
    const auto stronger = rectangle_quality(80, 200);
    require(std::abs(brighter.sharpness / normal.sharpness - 1.0f) < 0.001f,
        "constant brightness offset preserves Tenengrad energy");
    require(std::abs(stronger.sharpness / normal.sharpness - 4.0f) < 0.01f,
        "doubling edge contrast quadruples gradient energy, rather than gradient magnitude");

    // A separate part inside a hole must not contribute to the enclosing part's mean.
    // It is also close enough to lie in the enclosing part's local background ring.
    CHImage nested(400, 600, CV_8UC1, cv::Scalar(200));
    cv::rectangle(nested, cv::Rect(120, 80, 200, 200), cv::Scalar(30), cv::FILLED);
    cv::rectangle(nested, cv::Rect(140, 100, 160, 160), cv::Scalar(200), cv::FILLED);
    cv::rectangle(nested, cv::Rect(143, 130, 25, 60), cv::Scalar(0), cv::FILLED);
    const auto original = nested.clone();
    Params params;
    params.threshold = 100;
    Detector detector;
    RegisterResult base;
    DetectResult current;
    require(detector.RegisterBase(nested, base, params) == ISO_OK && base.shapes.size() == 2,
        "register ring and disconnected nested part");
    require(cv::norm(nested, original, cv::NORM_INF) == 0, "quality measurement preserves input pixels");
    for (const auto& shape : base.shapes) {
        const bool is_ring = shape.area > 20000.0;
        const double expected_brightness = is_ring ? 30.0 : 0.0;
        const double expected_contrast = is_ring ? 170.0 : 200.0;
        require(std::abs(shape.mean_brightness - expected_brightness) < 0.01,
            "brightness excludes holes and other connected components");
        require(std::abs(shape.contrast - expected_contrast) < 0.5,
            "local background excludes nearby and nested foreground");
    }

    CHImage remote_change(nested.size(), CV_8UC1, cv::Scalar(240));
    const cv::Rect local_region(100, 60, 240, 240);
    nested(local_region).copyTo(remote_change(local_region));
    require(detector.Detect(remote_change, current) == ISO_OK && current.all_matched,
        "remote background change does not affect matching");
    for (size_t i = 0; i < base.shapes.size(); ++i) {
        require(current.shapes[i].mean_brightness == base.shapes[i].mean_brightness &&
            current.shapes[i].contrast == base.shapes[i].contrast &&
            current.shapes[i].sharpness == base.shapes[i].sharpness,
            "quality uses local pixels, not the whole image background");
    }

    CHImage color;
    cv::cvtColor(nested, color, cv::COLOR_GRAY2BGRA);
    require(detector.Detect(color, current) == ISO_OK && current.all_matched, "quality supports BGRA input");
    for (size_t i = 0; i < base.shapes.size(); ++i) {
        require(current.shapes[i].mean_brightness == base.shapes[i].mean_brightness &&
            current.shapes[i].contrast == base.shapes[i].contrast,
            "BGRA conversion preserves brightness and contrast");
    }
}

void test_errors_and_formats() {
    Detector detector;
    RegisterResult base_result;
    DetectResult result;
    const auto image = make_product();
    require(detector.Detect(image, result) == ISO_BaseNotRegistered, "detect requires base");
    require(detector.RegisterBase(CHImage(), base_result) == ISO_ImageIsEmpty, "empty base");
    Params params;
    for (int threshold : {-2, 256}) {
        params.threshold = threshold;
        require(detector.RegisterBase(image, base_result, params) == ISO_InvalidParameter, "reject invalid threshold");
    }
    params = {};
    for (double area : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity()}) {
        params.min_area = area;
        require(detector.RegisterBase(image, base_result, params) == ISO_InvalidParameter, "reject invalid minimum area");
    }
    require(detector.RegisterBase(image, base_result) == ISO_OK, "valid base");
    require(detector.RegisterBase(CHImage(), base_result) == ISO_ImageIsEmpty && base_result.shapes.empty(),
        "failed registration clears output");
    require(detector.Detect(image, result) == ISO_OK && result.all_matched, "failed registration preserves base");

    CHImage image16;
    image.convertTo(image16, CV_16U);
    require(detector.Detect(image16, result) == ISO_UnsupportedImage && result.shapes.empty(), "reject 16-bit input");
    require(detector.Detect(CHImage(), result) == ISO_ImageIsEmpty && result.shapes.empty(), "empty detect clears output");
    require(detector.Detect(CHImage(image.size(), CV_8UC1, cv::Scalar(185)), result) == ISO_NoShapes &&
        result.missing_ids.size() == 8 && result.matched_count == 0, "no parts reports every missing ID");
    require(detector.Detect(CHImage(image.size(), CV_8UC1, cv::Scalar(0)), result) == ISO_NoShapes,
        "dark background alone is not a part");

    CHImage color;
    cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
    require(detector.Detect(color, result) == ISO_OK && result.all_matched, "BGR input");
    cv::cvtColor(image, color, cv::COLOR_GRAY2BGRA);
    require(detector.Detect(color, result) == ISO_OK && result.all_matched, "BGRA input");
    require(detector.Detect(CHImage(image.size(), CV_8UC2), result) == ISO_UnsupportedImage, "reject two channels");
    Detector independent;
    require(independent.Detect(image, result) == ISO_BaseNotRegistered, "instance state is independent");

    CHImage circle(300, 400, CV_8UC1, cv::Scalar(180));
    cv::circle(circle, {200, 150}, 45, cv::Scalar(10), cv::FILLED);
    require(independent.RegisterBase(circle, base_result) == ISO_OK && base_result.shapes.size() == 1, "single circular part");
    require(!base_result.shapes[0].orientation_valid, "circle axis is not reliable");
    require(independent.Detect(circle, result) == ISO_OK && result.all_matched &&
        !result.shapes[0].angle_valid && !result.angle_valid, "matched circle has no fabricated rotation");
    const double expected_area = 3.14159265358979323846 * 45.0 * 45.0;
    require(std::abs(result.shapes[0].area / expected_area - 1.0) < 0.05, "circle contour area");
}

CHImage read_image(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    require(stream.good(), "cannot open input image");
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(stream)), {});
    auto image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
    require(!image.empty(), "cannot decode input image");
    return image;
}

void save_overlay_impl(const std::filesystem::path& path, const CHImage& image,
    const Image_Sharpness_Orientations_Shapes& shapes, const std::string& caption, bool registration) {
    CHImage color;
    if (image.channels() == 1) cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
    else if (image.channels() == 4) cv::cvtColor(image, color, cv::COLOR_BGRA2BGR);
    else color = image.clone();
    const double scale = std::min(1.0, 1500.0 / color.cols);
    cv::resize(color, color, {}, scale, scale, cv::INTER_AREA);
    for (const auto& shape : shapes) {
        std::vector<cv::Point> contour;
        for (const auto& point : shape.contour) contour.emplace_back(cvRound(point.x * scale), cvRound(point.y * scale));
        const cv::Scalar ink = registration || shape.matched ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::polylines(color, std::vector<std::vector<cv::Point>>{contour}, true, ink, 2, cv::LINE_AA);
        const cv::Point center(cvRound(shape.center.x * scale), cvRound(shape.center.y * scale));
        const std::string label = shape.id < 0 ? "new" : std::to_string(shape.id);
        cv::putText(color, label, center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        cv::putText(color, label, center, cv::FONT_HERSHEY_SIMPLEX, 0.8, ink, 2, cv::LINE_AA);
    }
    cv::putText(color, caption, {20, 35}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    std::vector<unsigned char> bytes;
    require(cv::imencode(".png", color, bytes), "cannot encode overlay");
    std::ofstream stream(path, std::ios::binary);
    stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    require(stream.good(), "cannot save overlay");
}

void save_overlay(const std::filesystem::path& path, const CHImage& image, const RegisterResult& result) {
    const std::string caption = "Registered " + std::to_string(result.shapes.size()) + " base parts";
    save_overlay_impl(path, image, result.shapes, caption, true);
}

void save_overlay(const std::filesystem::path& path, const CHImage& image, const DetectResult& result) {
    std::ostringstream caption;
    caption << "matched " << result.matched_count << "/" << result.base_shape_count
            << "  parts " << result.shapes.size() << "  angle ";
    if (result.angle_valid) caption << std::fixed << std::setprecision(2) << result.angle << " deg CW";
    else caption << "unavailable";
    save_overlay_impl(path, image, result.shapes, caption.str(), false);
}

std::string csv_name(const std::filesystem::path& path) {
    std::string escaped = "\"";
    for (char c : path.filename().u8string()) {
        if (c == '"') escaped += '"';
        escaped += c;
    }
    return escaped + "\"";
}

void write_measurements(std::ofstream& measurements, const std::filesystem::path& path,
    const Image_Sharpness_Orientations_Shapes& shapes, bool registration) {
    for (const auto& shape : shapes) {
        measurements << csv_name(path) << ',' << shape.id << ',';
        if (!registration) measurements << shape.matched;
        measurements << ',' << shape.center.x << ',' << shape.center.y << ',' << shape.area
            << ',' << shape.mean_brightness << ',' << shape.sharpness << ',' << shape.contrast
            << ',' << shape.orientation << ',' << shape.orientation_valid << ',';
        if (!registration) measurements << shape.angle;
        measurements << ',';
        if (!registration) measurements << shape.angle_valid;
        measurements << ',';
        if (!registration) measurements << shape.match_cost;
        measurements << ',' << (registration ? "register" : "detect") << '\n';
    }
}

void write_results(std::ofstream& summary, std::ofstream& measurements,
    const std::filesystem::path& path, int status, const RegisterResult& result) {
    summary << csv_name(path) << ',' << status << ',' << result.shapes.size() << ','
            << result.shapes.size() << ",,,,,,register\n";
    write_measurements(measurements, path, result.shapes, true);
}

void write_results(std::ofstream& summary, std::ofstream& measurements,
    const std::filesystem::path& path, int status, const DetectResult& result) {
    summary << csv_name(path) << ',' << status << ',' << result.shapes.size() << ','
            << result.base_shape_count << ',' << result.matched_count << ',' << result.all_matched
            << ',' << result.angle << ',' << result.angle_valid << ",\"";
    for (int id : result.missing_ids) summary << id << ' ';
    summary << "\",detect\n";
    write_measurements(measurements, path, result.shapes, false);
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

int dev() {
    try {
        require(test_IMAGE_SHARPNESS_ORIENTATIONS_API() == ISO_OK, "DLL linkage");
        test_registration_and_motion();
        test_dark_and_light_parts();
        test_quality_measurements();
        test_errors_and_formats();
        std::cout << "All registration, matching, angle, area, brightness, Tenengrad, contrast, and error checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

int run_images(const std::vector<std::filesystem::path>& args) {
    if (args.size() < 4 || args[0] != "--images") {
        std::cerr << "Usage: Image_Sharpness_Orientations_test --images BASE OUTPUT_DIR IMAGE [IMAGE ...]\n";
        return 2;
    }
    try {
        std::filesystem::create_directories(args[2]);
        std::ofstream summary(args[2] / "summary.csv");
        std::ofstream measurements(args[2] / "measurements.csv");
        require(summary.good() && measurements.good(), "cannot create CSV outputs");
        summary << "image,status,parts,base_parts,matched,all_matched,angle_deg,angle_valid,missing_ids,stage\n";
        measurements << "image,id,matched,center_x,center_y,area_px2,mean_brightness,sharpness,contrast,orientation_deg,orientation_valid,angle_deg,angle_valid,match_cost,stage\n";
        measurements << std::setprecision(10);
        Detector detector;
        RegisterResult base_result;
        DetectResult result;
        const auto base = read_image(args[1]);
        const int registered = detector.RegisterBase(base, base_result);
        require(registered == ISO_OK, "base registration failed: " + std::to_string(registered));
        write_results(summary, measurements, args[1], registered, base_result);
        save_overlay(args[2] / "base.png", base, base_result);
        std::cout << "Registered " << base_result.shapes.size() << " parts.\n";
        bool success = true;
        for (size_t i = 3; i < args.size(); ++i) {
            const auto image = read_image(args[i]);
            const int status = detector.Detect(image, result);
            write_results(summary, measurements, args[i], status, result);
            save_overlay(args[2] / (args[i].stem().u8string() + ".png"), image, result);
            std::cout << args[i].filename().u8string() << ": status=" << status
                << ", parts=" << result.shapes.size() << ", matched=" << result.matched_count
                << "/" << result.base_shape_count << ", angle=" << result.angle << '\n';
            if (status != ISO_OK || !result.all_matched) success = false;
        }
        require(summary.good() && measurements.good(), "cannot write CSV outputs");
        return success ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
