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

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

const char* error_message(int status) {
    switch (status) {
    case ISO_ImageIsEmpty: return "empty image";
    case ISO_InvalidParameter: return "invalid threshold or minimum area";
    case ISO_UnsupportedImage: return "expected an 8-bit grayscale, BGR, or BGRA image";
    case ISO_NoShapes: return "no qualifying projection found";
    case ISO_OuterEdgeClipped: return "largest boundary touches the image border; check framing and polarity (--dark)";
    case ISO_EllipseFitFailed: return "could not fit a valid ellipse";
    default: return "processing failed";
    }
}

void save_overlay(const std::filesystem::path& path, const CHImage& image,
    const Image_Sharpness_Orientations_Outer_Edge_Result& edge) {
    cv::Mat color;
    if (image.channels() == 1) cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
    else if (image.channels() == 4) cv::cvtColor(image, color, cv::COLOR_BGRA2BGR);
    else color = image.clone();
    const double scale = std::min(1.0, 1500.0 / image.cols);
    cv::resize(color, color, {}, scale, scale, cv::INTER_AREA);
    std::vector<cv::Point> contour;
    for (const auto& point : edge.contour) {
        contour.emplace_back(cvRound(point.x * scale), cvRound(point.y * scale));
    }
    cv::polylines(color, std::vector<std::vector<cv::Point>>{contour}, true, {0, 255, 0}, 3, cv::LINE_AA);
    const cv::Point2f center = edge.center * static_cast<float>(scale);
    const cv::RotatedRect ellipse(center,
        cv::Size2f(static_cast<float>(2.0 * edge.a * scale), static_cast<float>(2.0 * edge.b * scale)),
        static_cast<float>(edge.orientation));
    cv::ellipse(color, ellipse, {0, 0, 255}, 1, cv::LINE_AA);
    cv::drawMarker(color, center, {255, 255, 0}, cv::MARKER_CROSS, 16, 2, cv::LINE_AA);
    cv::copyMakeBorder(color, color, 75, 0, 0, std::max(0, 680 - color.cols), cv::BORDER_CONSTANT, {0, 0, 0});
    cv::putText(color, "Green: measured edge   Red: fitted ellipse   Cyan: center",
        {12, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);
    std::ostringstream caption;
    caption << std::fixed << std::setprecision(2) << "a=" << edge.a << "px  b=" << edge.b
            << "px  b/a=" << std::setprecision(4) << edge.axis_ratio
            << "  sharpness=" << std::setprecision(1) << edge.sharpness;
    cv::putText(color, caption.str(), {12, 55}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
        {255, 255, 255}, 1, cv::LINE_AA);
    std::vector<unsigned char> bytes;
    require(cv::imencode(".png", color, bytes), "cannot encode overlay");
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    require(output.good(), "cannot write overlay");
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

int run_outer_edge_image(const std::vector<std::filesystem::path>& args) {
    const auto usage = [] {
        std::cerr << "Usage: Image_Sharpness_Orientations_test --outer-edge IMAGE OUTPUT_DIR"
                     " [--dark] [--threshold N] [--min-area N]\n";
    };
    if (args.size() < 3) {
        usage();
        return 2;
    }
    Image_Sharpness_Orientations_Outer_Edge_Params params;
    try {
        for (size_t i = 3; i < args.size(); ++i) {
            const auto option = args[i].u8string();
            if (option == "--dark") params.detect_bright_area = false;
            else if ((option == "--threshold" || option == "--min-area") && i + 1 < args.size()) {
                const auto value = args[++i].u8string();
                size_t consumed = 0;
                if (option == "--threshold") params.threshold = std::stoi(value, &consumed);
                else params.min_area = std::stod(value, &consumed);
                require(consumed == value.size(), "invalid numeric option: " + value);
            } else throw std::runtime_error("unknown or incomplete option: " + option);
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        usage();
        return 2;
    }
    try {
        // Decode from bytes to support Windows Unicode paths as well as macOS/Linux.
        std::ifstream input(args[1], std::ios::binary);
        require(input.good(), "cannot open input image: " + args[1].u8string());
        const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)), {});
        require(!bytes.empty(), "input file is empty");
        const auto image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
        require(!image.empty(), "cannot decode input image");

        CH_Image_Sharpness_Orientations detector;
        Image_Sharpness_Orientations_Outer_Edge_Result edge;
        const int status = detector.DetectOuterEdge(image, edge, params);
        if (status != ISO_OK) {
            std::cerr << "Detection failed (status=" << status << "): " << error_message(status)
                      << ". No output files updated.\n";
            return 1;
        }

        const auto overlay = args[2] / "outer_edge.png";
        const auto csv = args[2] / "outer_edge.csv";
        const auto source = std::filesystem::weakly_canonical(args[1]);
        require(source != std::filesystem::weakly_canonical(overlay) &&
            source != std::filesystem::weakly_canonical(csv), "output would overwrite the input image; choose another output directory");
        std::filesystem::create_directories(args[2]);
        save_overlay(overlay, image, edge);
        std::ofstream measurements(csv);
        measurements << "status,image_width,image_height,center_x,center_y,a_px,b_px,orientation_deg,orientation_valid,axis_ratio,fit_error,area_px2,mean_brightness,sharpness,contrast\n"
            << std::setprecision(12) << status << ',' << edge.image_size.width << ',' << edge.image_size.height
            << ',' << edge.center.x << ',' << edge.center.y << ',' << edge.a << ',' << edge.b
            << ',' << edge.orientation << ',' << edge.orientation_valid << ',' << edge.axis_ratio
            << ',' << edge.fit_error << ',' << edge.area << ',' << edge.mean_brightness
            << ',' << edge.sharpness << ',' << edge.contrast << '\n';
        measurements.close();
        require(measurements.good(), "cannot write measurements CSV");
        std::cout << std::fixed << std::setprecision(4)
            << "center=" << edge.center << "  a=" << edge.a << "px  b=" << edge.b << "px\n"
            << "orientation=" << edge.orientation << " deg CW  orientation_valid=" << edge.orientation_valid
            << "  b/a=" << edge.axis_ratio << "  fit_error=" << edge.fit_error << '\n'
            << "sharpness=" << edge.sharpness << "  brightness=" << edge.mean_brightness
            << "  contrast=" << edge.contrast << '\n'
            << "Overlay: " << overlay.u8string() << "\nMeasurements: " << csv.u8string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
