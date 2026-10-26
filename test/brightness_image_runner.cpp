#include "main.h"
#include "image_sharpness_orientations.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

using Mode = Image_Sharpness_Orientations_Brightness_Mode;
using Status = Image_Sharpness_Orientations_Brightness_Point_Status;
using Result = Image_Sharpness_Orientations_Brightness_Result;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

const char* status_name(Status status) {
    switch (status) {
    case Status::Valid: return "valid";
    case Status::OutsideImage: return "outside_image";
    case Status::NoPixels: return "no_pixels";
    case Status::NoBrightArea: return "no_bright_area";
    default: return "not_measured";
    }
}

void save_brightness_overlay(const std::filesystem::path& path, const cv::Mat& image, const Result& result) {
    cv::Mat color;
    if (image.channels() == 1) cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
    else if (image.channels() == 4) cv::cvtColor(image, color, cv::COLOR_BGRA2BGR);
    else color = image.clone();
    const double scale = std::min(1.0, 1500.0 / image.cols);
    cv::resize(color, color, {}, scale, scale, cv::INTER_AREA);
    if (result.ellipse_valid) {
        cv::ellipse(color, cv::RotatedRect(result.ellipse_center * static_cast<float>(scale),
            cv::Size2f(static_cast<float>(2 * result.ellipse_a * scale), static_cast<float>(2 * result.ellipse_b * scale)),
            static_cast<float>(result.ellipse_orientation)), {255, 150, 0}, 1, cv::LINE_AA);
    }
    for (const auto& point : result.points) {
        // Very distant manual points are retained in the CSV but cannot be drawn in the image.
        const auto visible = [&](CHPointF p) {
            return p.x >= 0 && p.y >= 0 && p.x < image.cols && p.y < image.rows;
        };
        if (!visible(point.center)) continue;
        const cv::Point center(cvRound(point.center.x * scale), cvRound(point.center.y * scale));
        const cv::Scalar ink = point.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        const double radius = result.spot_diameter_px * 0.5 * scale;
        // Bound drawing coordinates for invalid oversized disks; the CSV retains the true diameter.
        if (radius <= std::max(color.cols, color.rows)) {
            cv::circle(color, center, std::max(1, cvRound(radius)), ink, 2, cv::LINE_AA);
        } else cv::drawMarker(color, center, ink, cv::MARKER_CROSS, 12, 2, cv::LINE_AA);
        if (point.moved && visible(point.requested_center)) {
            const cv::Point requested(cvRound(point.requested_center.x * scale), cvRound(point.requested_center.y * scale));
            cv::drawMarker(color, requested, {0, 255, 255}, cv::MARKER_CROSS, 10, 1, cv::LINE_AA);
            cv::arrowedLine(color, requested, center, {0, 255, 255}, 1, cv::LINE_AA, 0, 0.2);
        }
        std::ostringstream text;
        text << point.id << ":";
        if (point.valid) text << std::fixed << std::setprecision(1) << point.mean_brightness;
        else text << "invalid";
        const auto label = text.str();
        int baseline = 0;
        const auto text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const cv::Point label_at(std::max(0, std::min(center.x + 8, color.cols - text_size.width - 2)),
            std::max(text_size.height, std::min(center.y - 8, color.rows - baseline - 2)));
        cv::putText(color, label, label_at, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 3, cv::LINE_AA);
        cv::putText(color, label, label_at, cv::FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv::LINE_AA);
    }
    cv::copyMakeBorder(color, color, 60, 0, 0, std::max(0, 800 - color.cols), cv::BORDER_CONSTANT, {0, 0, 0});
    cv::putText(color, "Green: valid disk   Red: invalid   Yellow: original point -> replacement",
        {12, 23}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
    cv::putText(color, "Valid: " + std::to_string(result.valid_count) + "/9   Labels: point ID : mean gray (0-255)",
        {12, 48}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
    std::vector<unsigned char> encoded;
    require(cv::imencode(".png", color, encoded), "cannot encode brightness overlay");
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
    output.close();
    require(output.good(), "cannot write brightness overlay");
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

int run_brightness_image(const std::vector<std::filesystem::path>& args) {
    const auto usage = [] {
        std::cerr << "Usage: Image_Sharpness_Orientations_test --brightness IMAGE OUTPUT_DIR\n"
            "  [--mode auto|avoid-black|manual] [--inset N] [--diameter N]\n"
            "  [--points FILE] [--threshold N] [--dark]\n"
            "  avoid-black only: [--black-threshold N] [--search-radius N]\n"
            "Manual points file: exactly 9 pairs of x y coordinates (whitespace separated).\n";
    };
    if (args.size() < 3) { usage(); return 2; }
    Image_Sharpness_Orientations_Brightness_Params params;
    std::filesystem::path points_file;
    bool black_threshold_provided = false;
    bool search_radius_provided = false;
    try {
        for (size_t i = 3; i < args.size(); ++i) {
            const auto option = args[i].u8string();
            if (option == "--dark") { params.outer_edge.detect_bright_area = false; continue; }
            require(i + 1 < args.size(), "missing value for " + option);
            const auto value_path = args[++i];
            const auto value = value_path.u8string();
            if (option == "--mode") {
                if (value == "auto") params.mode = Mode::Auto;
                else if (value == "avoid-black") params.mode = Mode::AvoidBlack;
                else if (value == "manual") params.mode = Mode::Manual;
                else throw std::runtime_error("unknown brightness mode: " + value);
            } else if (option == "--points") points_file = value_path;
            else {
                size_t consumed = 0;
                const int number = std::stoi(value, &consumed);
                require(consumed == value.size(), "invalid integer: " + value);
                if (option == "--inset") params.inset_px = number;
                else if (option == "--diameter") params.spot_diameter_px = number;
                else if (option == "--black-threshold") {
                    params.black_threshold = number;
                    black_threshold_provided = true;
                } else if (option == "--search-radius") {
                    params.search_radius_px = number;
                    search_radius_provided = true;
                } else if (option == "--threshold") params.outer_edge.threshold = number;
                else throw std::runtime_error("unknown option: " + option);
            }
        }
        require(!black_threshold_provided || params.mode == Mode::AvoidBlack,
            "--black-threshold requires --mode avoid-black");
        require(!search_radius_provided || params.mode == Mode::AvoidBlack,
            "--search-radius requires --mode avoid-black");
        if (params.mode == Mode::Manual) {
            require(!points_file.empty(), "manual mode requires --points FILE");
            std::ifstream points(points_file);
            require(points.good(), "cannot open manual points file");
            float x = 0.0f, y = 0.0f;
            for (int i = 0; i < 9; ++i) {
                require(static_cast<bool>(points >> x >> y), "expected 9 finite x y pairs");
                params.points.emplace_back(x, y);
            }
            std::string extra;
            require(!(points >> extra), "expected exactly 9 point pairs");
        } else require(points_file.empty(), "--points requires --mode manual");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; usage(); return 2;
    }

    try {
        std::ifstream input(args[1], std::ios::binary);
        require(input.good(), "cannot open input image: " + args[1].u8string());
        const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)), {});
        require(!bytes.empty(), "empty input file");
        const auto image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
        require(!image.empty(), "cannot decode input image");
        CH_Image_Sharpness_Orientations detector;
        Result result;
        const int status = detector.DetectBrightness(image, result, params);
        if (status != ISO_OK && status != ISO_PartialMeasurement) {
            std::cerr << "Brightness detection failed (status=" << status
                << "). Check image format, parameters, and complete outer boundary for auto modes. No files updated.\n";
            return 1;
        }
        const auto overlay = args[2] / "brightness.png";
        const auto csv = args[2] / "brightness.csv";
        for (const auto& source_path : {args[1], points_file}) {
            if (source_path.empty()) continue;
            const auto source = std::filesystem::weakly_canonical(source_path);
            require(source != std::filesystem::weakly_canonical(overlay) &&
                source != std::filesystem::weakly_canonical(csv), "output would overwrite an input; choose another directory");
        }
        std::filesystem::create_directories(args[2]);
        save_brightness_overlay(overlay, image, result);
        std::ofstream output(csv);
        output << "id,requested_x,requested_y,center_x,center_y,moved,valid,status,diameter_px,pixel_count,mean_brightness,black_threshold\n"
            << std::setprecision(12);
        for (const auto& point : result.points) {
            output << point.id << ',' << point.requested_center.x << ',' << point.requested_center.y
                << ',' << point.center.x << ',' << point.center.y << ',' << point.moved << ',' << point.valid
                << ',' << status_name(point.status) << ',' << result.spot_diameter_px << ',' << point.pixel_count << ',';
            if (point.valid) output << point.mean_brightness;
            output << ',';
            if (result.effective_black_threshold >= 0) output << result.effective_black_threshold;
            output << '\n';
            std::cout << point.id << ": " << point.requested_center << " -> " << point.center
                << "  " << status_name(point.status);
            if (point.valid) std::cout << "  brightness=" << point.mean_brightness;
            std::cout << '\n';
        }
        output.close();
        require(output.good(), "cannot write brightness CSV");
        std::cout << "Valid points: " << result.valid_count << "/9\nOverlay: " << overlay.u8string()
            << "\nMeasurements: " << csv.u8string() << '\n';
        return result.all_valid ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
