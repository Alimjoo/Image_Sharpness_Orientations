#include "main.h"

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[]) {
#else
int main(int argc, char* argv[]) {
#endif
    if (argc == 1) return TEST_IMAGE_SHARPNESS_ORIENTATIONS::dev();
    std::vector<std::filesystem::path> args;
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
    if (args[0] == "--outer-edge") return TEST_IMAGE_SHARPNESS_ORIENTATIONS::run_outer_edge_image(args);
    if (args[0] == "--brightness") return TEST_IMAGE_SHARPNESS_ORIENTATIONS::run_brightness_image(args);
    if (args[0] == "--calibrate") return TEST_IMAGE_SHARPNESS_ORIENTATIONS::run_calibration_image(args);
    return TEST_IMAGE_SHARPNESS_ORIENTATIONS::run_images(args);
}
