#include "main.h"
#include "image_sharpness_orientations.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

using Detector = CH_Image_Sharpness_Orientations;
using RegisterResult = Image_Sharpness_Orientations_Register_Result;
using DetectResult = Image_Sharpness_Orientations_Detect_Result;
using Options = Image_Sharpness_Orientations_Detect_Params;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error("pose: " + message);
}

CHImage product(bool marks = true) {
    CHImage image(600, 800, CV_8UC1, cv::Scalar(0));
    cv::ellipse(image, {400, 300}, {360, 260}, 0, 0, 360, cv::Scalar(185), cv::FILLED);
    if (marks) {
        for (const auto& point : std::vector<cv::Point>{{200, 190}, {360, 170}, {470, 280}, {280, 400}}) {
            cv::circle(image, point, 18, cv::Scalar(10), cv::FILLED);
        }
    }
    return image;
}

const Image_Sharpness_Orientations_Shape& by_id(const Image_Sharpness_Orientations_Shapes& shapes, int id) {
    const auto found = std::find_if(shapes.begin(), shapes.end(),
        [id](const auto& shape) { return shape.id == id; });
    require(found != shapes.end(), "expected shape ID " + std::to_string(id));
    return *found;
}

cv::Point2d transform_point(cv::Point2d point, const cv::Mat& transform) {
    return {transform.at<double>(0, 0) * point.x + transform.at<double>(0, 1) * point.y + transform.at<double>(0, 2),
        transform.at<double>(1, 0) * point.x + transform.at<double>(1, 1) * point.y + transform.at<double>(1, 2)};
}

void test_known_motion() {
    Detector detector;
    RegisterResult base;
    DetectResult current;
    const auto image = product();
    require(detector.RegisterBase(image, base) == ISO_OK && base.shapes.size() == 4, "register four circular features");
    for (const auto& shape : base.shapes) {
        require(!shape.offset_valid && shape.offset_x == 0.0 && shape.offset_y == 0.0 && shape.base_center == CHPointF(),
            "registration leaves detection-only offset fields at defaults");
    }
    require(detector.Detect(image, current) == ISO_OK && current.all_matched &&
        current.reference_valid && current.offset_valid && !current.using_feature_pair &&
        current.reference_ids == std::vector<int>({1, 2, 3, 4}) && current.offset_x == 0.0 && current.offset_y == 0.0,
        "default Detect returns zero overall displacement for identical input");
    const auto base_reference = current.base_reference_center;
    const auto original = image.clone();
    auto transform = cv::getRotationMatrix2D({400, 300}, -13.0, 1.0);
    transform.at<double>(0, 2) += 23;
    transform.at<double>(1, 2) -= 17;
    CHImage moved;
    cv::warpAffine(image, moved, transform, image.size());
    require(detector.Detect(moved, current) == ISO_OK && current.all_matched && current.angle_valid &&
        std::abs(current.angle - 13.0) < 0.3 && current.offset_valid,
        "rotation above 2 degrees remains a valid measurement without a product acceptance limit");
    const auto expected_reference = transform_point(base_reference, transform);
    require(cv::norm(current.reference_center - expected_reference) < 0.5 &&
        cv::norm(cv::Point2d(current.offset_x, current.offset_y) - (expected_reference - base_reference)) < 0.5,
        "overall displacement is measured at the reported reference, not the affine origin");
    for (const auto& shape : current.shapes) {
        const auto& source = by_id(base.shapes, shape.id);
        const auto expected = transform_point(source.center, transform) - cv::Point2d(source.center);
        require(shape.offset_valid && shape.base_center == source.center &&
            cv::norm(cv::Point2d(shape.offset_x, shape.offset_y) - expected) < 0.5,
            "each matched part reports its own base center and displacement");
    }

    Options options;
    options.feature_ids = {1, 4};
    require(detector.Detect(moved, current, options) == ISO_OK && current.using_feature_pair &&
        current.reference_ids == std::vector<int>({1, 4}) && current.angle_valid && current.offset_valid &&
        std::abs(current.angle - 13.0) < 0.3, "selected pair line determines overall rotation");
    const auto midpoint = (cv::Point2d(by_id(base.shapes, 1).center) + cv::Point2d(by_id(base.shapes, 4).center)) * 0.5;
    const auto moved_midpoint = transform_point(midpoint, transform);
    require(cv::norm(current.base_reference_center - midpoint) < 1e-9 &&
        cv::norm(current.reference_center - moved_midpoint) < 0.5 &&
        cv::norm(cv::Point2d(current.offset_x, current.offset_y) - (moved_midpoint - midpoint)) < 0.5,
        "selected pair midpoint supplies the position reference");
    const auto forward = current;
    options.feature_ids = {4, 1};
    require(detector.Detect(moved, current, options) == ISO_OK && current.reference_ids == std::vector<int>({4, 1}) &&
        std::abs(current.angle - forward.angle) < 1e-9 && current.offset_x == forward.offset_x &&
        current.offset_y == forward.offset_y, "reversing both endpoints preserves relative angle and midpoint");
    require(cv::norm(image, original, cv::NORM_INF) == 0, "pose measurements preserve the base image");

    const cv::Mat translation = (cv::Mat_<double>(2, 3) << 1, 0, -19, 0, 1, 12);
    cv::warpAffine(image, moved, translation, image.size());
    require(detector.Detect(moved, current) == ISO_OK && current.offset_valid &&
        std::abs(current.offset_x + 19) < 0.1 && std::abs(current.offset_y - 12) < 0.1,
        "translation signs are right/down positive");
}

void test_selection_and_missing_features() {
    Detector detector;
    RegisterResult base;
    DetectResult automatic, selected;
    const auto image = product();
    require(detector.RegisterBase(image, base) == ISO_OK, "register pair selection fixture");
    Options options;
    options.feature_ids = {1, 4};
    auto changed = image.clone();
    cv::circle(changed, {280, 400}, 23, cv::Scalar(185), cv::FILLED);
    cv::circle(changed, {280, 430}, 18, cv::Scalar(10), cv::FILLED);
    require(detector.Detect(changed, automatic) == ISO_OK && automatic.all_matched && automatic.angle_valid &&
        std::abs(automatic.angle) > 0.2, "an unrelated shifted feature affects the automatic all-feature angle");
    require(detector.Detect(changed, selected, options) == ISO_OK && selected.all_matched && selected.angle_valid &&
        std::abs(selected.angle) < 0.01 && selected.offset_valid && selected.offset_x == 0.0 && selected.offset_y == 0.0,
        "selected pair ignores movement of unrelated features");
    require(by_id(selected.shapes, 2).offset_valid && std::abs(by_id(selected.shapes, 2).offset_y - 30.0) < 0.1,
        "pair selection still returns each unrelated shape's displacement");

    changed = image.clone();
    cv::circle(changed, {280, 400}, 23, cv::Scalar(185), cv::FILLED);
    cv::circle(changed, {630, 400}, 12, cv::Scalar(10), cv::FILLED);
    require(detector.Detect(changed, automatic) == ISO_OK && automatic.missing_ids == std::vector<int>({2}) &&
        automatic.reference_ids == std::vector<int>({1, 3, 4}) && automatic.offset_valid &&
        automatic.offset_x == 0.0 && automatic.offset_y == 0.0,
        "missing and extra parts alone do not create false offsets from different centroid subsets");
    const auto& extra = automatic.shapes.back();
    require(!extra.matched && !extra.offset_valid && extra.base_center == CHPointF() &&
        extra.offset_x == 0.0 && extra.offset_y == 0.0, "unmatched part has no fabricated offset");
    require(detector.Detect(changed, selected, options) == ISO_OK && selected.angle_valid && selected.offset_valid,
        "missing unrelated feature does not invalidate selected pair");

    changed = image.clone();
    cv::circle(changed, {200, 190}, 23, cv::Scalar(185), cv::FILLED);
    require(detector.Detect(changed, selected, options) == ISO_OK && selected.matched_count == 3 &&
        selected.using_feature_pair && selected.reference_ids == std::vector<int>({1, 4}) &&
        !selected.angle_valid && !selected.reference_valid && !selected.offset_valid &&
        selected.angle == 0.0 && selected.offset_x == 0.0 && selected.offset_y == 0.0,
        "missing selected feature invalidates overall pose without silently using other parts");
    require(by_id(selected.shapes, 4).offset_valid, "surviving matched part retains its own valid displacement");
    require(detector.Detect(changed, automatic) == ISO_OK && automatic.angle_valid && automatic.offset_valid &&
        !automatic.using_feature_pair, "pair choice is per-call and does not change default detection");

    require(detector.Detect(CHImage(image.size(), CV_8UC1, cv::Scalar(185)), selected, options) == ISO_NoShapes &&
        selected.missing_ids.size() == 4 && selected.reference_ids == std::vector<int>({1, 4}) &&
        selected.base_image_size == image.size() && !selected.reference_valid && !selected.offset_valid &&
        !selected.angle_valid, "no-shapes result reports unavailable pose and requested pair");
}

void test_validity_and_parameters() {
    Detector detector;
    RegisterResult base;
    DetectResult result;
    const auto image = product();
    require(detector.Detect(image, result, Options()) == ISO_BaseNotRegistered, "new overload requires registration");
    require(detector.RegisterBase(image, base) == ISO_OK, "register validation fixture");
    for (const auto& ids : std::vector<std::array<int, 2>>{{0, 1}, {1, 0}, {-1, 1}, {1, 1}, {1, 99}, {99, 100}}) {
        require(detector.Detect(image, result) == ISO_OK, "seed valid output");
        Options options;
        options.feature_ids = ids;
        require(detector.Detect(image, result, options) == ISO_InvalidParameter && result.shapes.empty() &&
            result.reference_ids.empty() && !result.angle_valid && !result.offset_valid && !result.reference_valid,
            "invalid pair IDs clear output and are rejected");
    }
    require(detector.Detect(image, result) == ISO_OK && result.all_matched, "invalid options leave base unchanged");
    Options options;
    options.feature_ids = {1, 4};
    CHImage resized;
    cv::resize(image, resized, {}, 0.75, 0.75, cv::INTER_AREA);
    for (const auto& choice : {Options(), options}) {
        require(detector.Detect(resized, result, choice) == ISO_OK && result.all_matched && result.angle_valid &&
            result.reference_valid && !result.offset_valid && result.offset_x == 0.0 && result.offset_y == 0.0 &&
            result.base_image_size == image.size() && result.image_size == resized.size(),
            "different resolutions retain matching/angles/reference coordinates without claiming valid displacement");
        for (const auto& part : result.shapes) {
            require(!part.offset_valid && part.offset_x == 0.0 && part.offset_y == 0.0 &&
                part.base_center == by_id(base.shapes, part.id).center, "per-part offsets follow coordinate compatibility");
        }
    }
    require(detector.Detect(CHImage(), result, options) == ISO_ImageIsEmpty && result.reference_ids.empty() &&
        !result.using_feature_pair && !result.offset_valid && !result.reference_valid, "failed image clears prior pose");

    auto single = product(false);
    cv::circle(single, {400, 300}, 18, cv::Scalar(10), cv::FILLED);
    require(detector.RegisterBase(single, base) == ISO_OK && base.shapes.size() == 1, "single circular feature");
    require(detector.Detect(single, result) == ISO_OK && result.offset_valid && result.reference_valid &&
        !result.angle_valid && result.reference_ids == std::vector<int>({1}),
        "one circular feature defines position even though its angle is unavailable");

    auto concentric = product(false);
    cv::circle(concentric, {400, 300}, 70, cv::Scalar(10), cv::FILLED);
    cv::circle(concentric, {400, 300}, 50, cv::Scalar(185), cv::FILLED);
    cv::circle(concentric, {400, 300}, 18, cv::Scalar(10), cv::FILLED);
    require(detector.RegisterBase(concentric, base) == ISO_OK && base.shapes.size() == 2 &&
        cv::norm(base.shapes[0].center - base.shapes[1].center) < 1e-6, "two distinct features with coincident centers");
    options.feature_ids = {1, 2};
    require(detector.Detect(concentric, result, options) == ISO_OK && result.all_matched &&
        !result.angle_valid && !result.reference_valid && !result.offset_valid,
        "coincident selected points yield unavailable pose, without product rejection");
    require(detector.Detect(concentric, result) == ISO_OK && result.reference_valid && result.offset_valid &&
        !result.angle_valid, "automatic position remains defined for coincident feature centers");
}

} // namespace

namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS {

void test_pose() {
    test_known_motion();
    test_selection_and_missing_features();
    test_validity_and_parameters();
}

} // namespace TEST_IMAGE_SHARPNESS_ORIENTATIONS
