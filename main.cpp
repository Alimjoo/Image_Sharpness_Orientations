#include "main.h"

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[]) {
#else
int main(int argc, char* argv[]) {
#endif
    if (argc == 1) return TEST_IMAGE_SHARPNESS_ORIENTATIONS::dev();
    std::vector<std::filesystem::path> args;
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
    return TEST_IMAGE_SHARPNESS_ORIENTATIONS::run_images(args);
}
