# Internal implementation, linked into the final API library.
add_library(Image_Sharpness_Orientations_Core STATIC
    headers/image_sharpness_orientations_core.h
    headers/quality_measurement.h
    headers/assignment.h
    headers/outer_edge.h
    headers/brightness_measurement.h
    sources/shape_extraction.cpp
    sources/quality_measurement.cpp
    sources/assignment.cpp
    sources/layout_matching.cpp
    sources/outer_edge.cpp
    sources/brightness_measurement.cpp
)

set_target_properties(Image_Sharpness_Orientations_Core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)
target_include_directories(Image_Sharpness_Orientations_Core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/headers"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../Image_Sharpness_Orientations/headers"
)
target_include_directories(Image_Sharpness_Orientations_Core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Image_Sharpness_Orientations_Core PUBLIC opencv_core opencv_imgproc)
