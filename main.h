#ifndef IMAGE_SHARPNESS_ORIENTATIONS_TEST_MAIN_H
#define IMAGE_SHARPNESS_ORIENTATIONS_TEST_MAIN_H

#include <filesystem>
#include <vector>

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {
    int dev();
    void test_outer_edge();
    void test_brightness();
    int run_images(const std::vector<std::filesystem::path>& args);
    int run_outer_edge_image(const std::vector<std::filesystem::path>& args);
    int run_brightness_image(const std::vector<std::filesystem::path>& args);
}

#endif // IMAGE_SHARPNESS_ORIENTATIONS_TEST_MAIN_H
