add_library(Image_Sharpness_Orientations SHARED
    headers/image_sharpness_orientations.h
    headers/image_sharpness_orientations_structs.h
    sources/image_sharpness_orientations.cpp
    sources/image_calibration.cpp
)

set(IMAGE_SHARPNESS_ORIENTATIONS_CONFIG "$<IF:$<BOOL:$<CONFIG>>,$<CONFIG>,Release>")
set(IMAGE_SHARPNESS_ORIENTATIONS_OUTPUT_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/Image_Sharpness_Orientations_${IMAGE_SHARPNESS_ORIENTATIONS_CONFIG}"
)
set(IMAGE_SHARPNESS_ORIENTATIONS_PACKAGE_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/Image_Sharpness_Orientations_Public_Release"
)

set_target_properties(Image_Sharpness_Orientations PROPERTIES
    DEBUG_POSTFIX "d"
    RUNTIME_OUTPUT_DIRECTORY "${IMAGE_SHARPNESS_ORIENTATIONS_OUTPUT_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${IMAGE_SHARPNESS_ORIENTATIONS_OUTPUT_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${IMAGE_SHARPNESS_ORIENTATIONS_OUTPUT_DIR}"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)
target_include_directories(Image_Sharpness_Orientations PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/headers")
target_include_directories(Image_Sharpness_Orientations SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Image_Sharpness_Orientations
    PUBLIC opencv_core opencv_imgproc
    PRIVATE Image_Sharpness_Orientations_Core
)

if(WIN32)
    target_compile_definitions(Image_Sharpness_Orientations PRIVATE IMAGE_SHARPNESS_ORIENTATIONS_DLL_EXPORTS)
    set_target_properties(Image_Sharpness_Orientations PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS OFF)
endif()

add_custom_command(
    TARGET Image_Sharpness_Orientations POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${IMAGE_SHARPNESS_ORIENTATIONS_OUTPUT_DIR}"
        "${IMAGE_SHARPNESS_ORIENTATIONS_PACKAGE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:Image_Sharpness_Orientations>"
        "${IMAGE_SHARPNESS_ORIENTATIONS_PACKAGE_DIR}/$<TARGET_FILE_NAME:Image_Sharpness_Orientations>"
    VERBATIM
    COMMENT "Packaging Image_Sharpness_Orientations"
)

foreach(public_header image_sharpness_orientations.h image_sharpness_orientations_structs.h)
    add_custom_command(
        TARGET Image_Sharpness_Orientations POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/headers/${public_header}"
            "${IMAGE_SHARPNESS_ORIENTATIONS_OUTPUT_DIR}/${public_header}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/headers/${public_header}"
            "${IMAGE_SHARPNESS_ORIENTATIONS_PACKAGE_DIR}/${public_header}"
        VERBATIM
    )
endforeach()

if(WIN32)
    add_custom_command(
        TARGET Image_Sharpness_Orientations POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_LINKER_FILE:Image_Sharpness_Orientations>"
            "${IMAGE_SHARPNESS_ORIENTATIONS_PACKAGE_DIR}/$<TARGET_LINKER_FILE_NAME:Image_Sharpness_Orientations>"
        VERBATIM
    )
endif()
