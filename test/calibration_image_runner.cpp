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

using Shape = Image_Calibration_Shape;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void save_calibration_overlay(const std::filesystem::path& path, const CHImage& image,
    const Image_Calibration_Result& result) {
    cv::Mat color;
    if (image.channels() == 1) cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
    else if (image.channels() == 4) cv::cvtColor(image, color, cv::COLOR_BGRA2BGR);
    else color = image.clone();
    const double scale = std::min(1.0, 1500.0 / image.cols);
    cv::resize(color, color, {}, scale, scale, cv::INTER_AREA);
    const auto display_point = [&](CHPointF p) { return cv::Point(cvRound(p.x * scale), cvRound(p.y * scale)); };
    std::vector<cv::Point> contour;
    for (const auto& point : result.contour) contour.push_back(display_point(point));
    cv::polylines(color, std::vector<std::vector<cv::Point>>{contour}, true, {0, 255, 0}, 3, cv::LINE_AA);
    if (result.shape == Shape::Circle) {
        cv::ellipse(color, cv::RotatedRect(result.center * static_cast<float>(scale),
            cv::Size2f(static_cast<float>(result.width_px * scale), static_cast<float>(result.height_px * scale)),
            static_cast<float>(result.orientation)), {0, 0, 255}, 1, cv::LINE_AA);
    } else {
        std::vector<cv::Point> corners;
        for (const auto& point : result.corners) corners.push_back(display_point(point));
        cv::polylines(color, std::vector<std::vector<cv::Point>>{corners}, true, {0, 0, 255}, 1, cv::LINE_AA);
        for (size_t i = 0; i < 4; ++i) {
            cv::circle(color, corners[i], 4, {0, 255, 255}, cv::FILLED, cv::LINE_AA);
            cv::putText(color, std::to_string(i), corners[i] + cv::Point(6, -6),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2, cv::LINE_AA);
        }
    }
    cv::drawMarker(color, display_point(result.center), {255, 255, 0}, cv::MARKER_CROSS, 15, 2, cv::LINE_AA);
    cv::copyMakeBorder(color, color, 75, 0, 0, std::max(0, 850 - color.cols), cv::BORDER_CONSTANT, {0, 0, 0});
    cv::putText(color, "Green: measured contour   Red: fitted target   Yellow: rectangle corners",
        {12, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
    std::ostringstream caption;
    caption << std::fixed << std::setprecision(6) << result.mm_per_pixel << " mm/px   "
        << result.pixels_per_mm << " px/mm   scale disagreement="
        << std::setprecision(2) << result.scale_disagreement * 100.0 << "%";
    cv::putText(color, caption.str(), {12, 55}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
        {255, 255, 255}, 1, cv::LINE_AA);
    std::vector<unsigned char> encoded;
    require(cv::imencode(".png", color, encoded), "cannot encode calibration overlay");
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
    output.close();
    require(output.good(), "cannot save calibration overlay");
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

int run_calibration_image(const std::vector<std::filesystem::path>& args) {
    const auto usage = [] {
        std::cerr << "Usage: Image_Sharpness_Orientations_test --calibrate IMAGE OUTPUT_DIR\n"
            "  --shape circle --diameter-mm D\n"
            "  --shape rectangle --width-mm W --height-mm H  (square: W=H)\n"
            "  [--white | --black] [--threshold N] [--min-area N] [--max-error 0.03]\n";
    };
    if (args.size() < 3) { usage(); return 2; }
    Image_Calibration_Params params;
    bool diameter_provided = false, width_provided = false, height_provided = false;
    try {
        for (size_t i = 3; i < args.size(); ++i) {
            const auto option = args[i].u8string();
            if (option == "--white") { params.detect_white = true; continue; }
            if (option == "--black") { params.detect_white = false; continue; }
            require(i + 1 < args.size(), "missing value for " + option);
            const auto value = args[++i].u8string();
            if (option == "--shape") {
                if (value == "circle") params.shape = Shape::Circle;
                else if (value == "rectangle") params.shape = Shape::Rectangle;
                else throw std::runtime_error("shape must be circle or rectangle; squares use rectangle");
                continue;
            }
            size_t consumed = 0;
            if (option == "--threshold") params.threshold = std::stoi(value, &consumed);
            else {
                const double number = std::stod(value, &consumed);
                if (option == "--diameter-mm") { params.diameter_mm = number; diameter_provided = true; }
                else if (option == "--width-mm") { params.width_mm = number; width_provided = true; }
                else if (option == "--height-mm") { params.height_mm = number; height_provided = true; }
                else if (option == "--min-area") params.min_area = number;
                else if (option == "--max-error") params.max_relative_error = number;
                else throw std::runtime_error("unknown option: " + option);
            }
            require(consumed == value.size(), "invalid numeric value: " + value);
        }
        if (params.shape == Shape::Circle) {
            require(diameter_provided, "circle requires --diameter-mm");
            require(!width_provided && !height_provided, "--width-mm and --height-mm require --shape rectangle");
        } else {
            require(width_provided && height_provided, "rectangle requires --width-mm and --height-mm");
            require(!diameter_provided, "--diameter-mm requires --shape circle");
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; usage(); return 2;
    }
    try {
        std::ifstream input(args[1], std::ios::binary);
        require(input.good(), "cannot open image: " + args[1].u8string());
        const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)), {});
        require(!bytes.empty(), "empty input image file");
        const auto image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
        require(!image.empty(), "cannot decode input image");
        CH_Image_Calibration calibrator;
        Image_Calibration_Result result;
        const int status = calibrator.Calibrate(image, result, params);
        if (status != ISO_OK) {
            std::cerr << "Calibration failed (status=" << status << "): ";
            if (status == ISO_CalibrationTargetInvalid) std::cerr << "target shape, view, or physical aspect ratio is inconsistent with a uniform scale";
            else if (status == ISO_OuterEdgeClipped) std::cerr << "largest target touches the image border; check framing and polarity";
            else if (status == ISO_InvalidParameter) std::cerr << "invalid physical dimensions, threshold, area, or tolerance";
            else if (status == ISO_NoShapes) std::cerr << "no qualifying target found";
            else std::cerr << "unsupported image or processing error";
            std::cerr << ". No output files updated.\n";
            return 1;
        }
        const auto overlay = args[2] / "calibration.png";
        const auto csv = args[2] / "calibration.csv";
        const auto source = std::filesystem::weakly_canonical(args[1]);
        require(source != std::filesystem::weakly_canonical(overlay) &&
            source != std::filesystem::weakly_canonical(csv), "output would overwrite input; choose another directory");
        std::filesystem::create_directories(args[2]);
        save_calibration_overlay(overlay, image, result);
        std::ofstream output(csv);
        output << "shape,image_width,image_height,center_x,center_y,orientation_deg,width_px,height_px,width_mm,height_mm,mm_per_pixel,pixels_per_mm,mm_per_pixel_width,mm_per_pixel_height,scale_disagreement,fit_error";
        for (int i = 0; i < 4; ++i) output << ",corner" << i << "_x,corner" << i << "_y";
        output << '\n' << std::setprecision(12) << (result.shape == Shape::Circle ? "circle" : "rectangle")
            << ',' << result.image_size.width << ',' << result.image_size.height << ',' << result.center.x << ',' << result.center.y
            << ',' << result.orientation << ',' << result.width_px << ',' << result.height_px
            << ',' << result.width_mm << ',' << result.height_mm << ',' << result.mm_per_pixel << ',' << result.pixels_per_mm
            << ',' << result.mm_per_pixel_width << ',' << result.mm_per_pixel_height
            << ',' << result.scale_disagreement << ',' << result.fit_error;
        for (const auto& point : result.corners) {
            if (result.shape == Shape::Rectangle) output << ',' << point.x << ',' << point.y;
            else output << ",,";
        }
        output << '\n';
        output.close();
        require(output.good(), "cannot save calibration CSV");
        std::cout << std::setprecision(10) << "Measured: " << result.width_px << " x " << result.height_px
            << " px; known: " << result.width_mm << " x " << result.height_mm << " mm\n"
            << "Scale: " << result.mm_per_pixel << " mm/pixel; " << result.pixels_per_mm << " pixels/mm\n"
            << "Dimension scale disagreement: " << result.scale_disagreement * 100.0 << "%\n"
            << "Overlay: " << overlay.u8string() << "\nCalibration: " << csv.u8string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
