#ifndef IMAGE_SHARPNESS_ORIENTATIONS_TEST_MAIN_H
#define IMAGE_SHARPNESS_ORIENTATIONS_TEST_MAIN_H

#include <filesystem>
#include <vector>

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {
    int dev();
    int run_images(const std::vector<std::filesystem::path>& args);
}

#endif // IMAGE_SHARPNESS_ORIENTATIONS_TEST_MAIN_H
