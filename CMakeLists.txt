cmake_minimum_required(VERSION 3.18)
project(Image_Sharpness_Orientations VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

include(CTest)

# Same local OpenCV convention as Alg_CH; -DOpenCV_DIR always takes precedence.
if(WIN32 AND NOT OpenCV_DIR AND EXISTS "$ENV{USERPROFILE}/Documents/opencv/build/OpenCVConfig.cmake")
    set(OpenCV_DIR "$ENV{USERPROFILE}/Documents/opencv/build")
endif()
set(IMAGE_SHARPNESS_ORIENTATIONS_OPENCV_COMPONENTS core imgproc)
if(BUILD_TESTING)
    list(APPEND IMAGE_SHARPNESS_ORIENTATIONS_OPENCV_COMPONENTS imgcodecs)
endif()
find_package(OpenCV 4 REQUIRED COMPONENTS ${IMAGE_SHARPNESS_ORIENTATIONS_OPENCV_COMPONENTS})

# Runtime copies are only for the local test executable, not the public package.
# The official Windows OpenCV distribution combines components into opencv_world.
set(IMAGE_SHARPNESS_ORIENTATIONS_OPENCV_RUNTIME_TARGETS)
if(WIN32 AND OpenCV_SHARED)
    if(TARGET opencv_world)
        set(IMAGE_SHARPNESS_ORIENTATIONS_OPENCV_RUNTIME_TARGETS opencv_world)
    else()
        set(IMAGE_SHARPNESS_ORIENTATIONS_OPENCV_RUNTIME_TARGETS ${OpenCV_LIBS})
    endif()
endif()

add_subdirectory(projs/Image_Sharpness_Orientations_Core)
add_subdirectory(projs/Image_Sharpness_Orientations)

if(BUILD_TESTING)
    add_executable(Image_Sharpness_Orientations_test
        main.cpp
        main.h
        test/test_dev.cpp
        test/test_outer_edge.cpp
        test/test_brightness.cpp
        test/test_calibration.cpp
        test/test_pose.cpp
        test/outer_edge_image_runner.cpp
        test/brightness_image_runner.cpp
        test/calibration_image_runner.cpp
    )
    target_include_directories(Image_Sharpness_Orientations_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(Image_Sharpness_Orientations_test PRIVATE
        Image_Sharpness_Orientations opencv_imgcodecs
    )

    if(WIN32)
        add_custom_command(
            TARGET Image_Sharpness_Orientations_test POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:Image_Sharpness_Orientations>"
                "$<TARGET_FILE_DIR:Image_Sharpness_Orientations_test>"
            VERBATIM
            COMMENT "Copying Image_Sharpness_Orientations DLL to the test directory"
        )
        foreach(opencv_target IN LISTS IMAGE_SHARPNESS_ORIENTATIONS_OPENCV_RUNTIME_TARGETS)
            add_custom_command(
                TARGET Image_Sharpness_Orientations_test POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${opencv_target}>"
                    "$<TARGET_FILE_DIR:Image_Sharpness_Orientations_test>"
                VERBATIM
            )
        endforeach()
    endif()

    add_test(NAME Image_Sharpness_Orientations_API COMMAND Image_Sharpness_Orientations_test)
endif()
