#include "image_sharpness_orientations.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Segmentation and matching defaults stay private.
constexpr int kBlurSize = 3;
constexpr double kMaxAreaRatio = 0.25;
constexpr int kContourBand = 2;
constexpr int kBackgroundRadius = 5;
constexpr double kMaxMatchDistance = 0.12;
constexpr double kUnmatchedCost = 0.5;
constexpr double kMinOrientationAnisotropy = 0.08;

struct Product_Frame {
    cv::Point2d center;
    double area = 0.0;
    cv::Matx22d normalization = cv::Matx22d::eye();
};

double axis_difference(double angle) {
    while (angle >= 90.0) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;
    return angle;
}

void measure_axis(const cv::Moments& m, double minimum_anisotropy,
    double& angle, bool& valid) {
    const double spread = m.mu20 + m.mu02;
    const double difference = std::hypot(m.mu20 - m.mu02, 2.0 * m.mu11);
    valid = spread > 1e-9 && difference / spread >= minimum_anisotropy;
    angle = 0.0;
    if (valid) {
        angle = 0.5 * std::atan2(2.0 * m.mu11, m.mu20 - m.mu02) * 180.0 / kPi;
        if (angle < 0.0) angle += 180.0;
    }
}

bool valid_params(const Image_Sharpness_Orientations_Params& p) {
    return p.threshold >= -1 && p.threshold <= 255 &&
        std::isfinite(p.min_area) && p.min_area > 0.0;
}

// Segmentation identifies the pixels, but all measurements use the original gray image.
void measure_quality(const cv::Mat& gray, const cv::Mat& foreground,
    const std::vector<cv::Point>& contour, const std::vector<cv::Point>& product_contour,
    Image_Sharpness_Orientations_Shape& shape) {
    const int margin = std::max(kContourBand, kBackgroundRadius) + 3;
    const auto& bounds = shape.bounds;
    const int left = std::max(0, bounds.x - margin);
    const int top = std::max(0, bounds.y - margin);
    const int right = std::min(gray.cols, bounds.x + bounds.width + margin);
    const int bottom = std::min(gray.rows, bounds.y + bounds.height + margin);
    const cv::Rect roi(left, top, right - left, bottom - top);
    const cv::Mat gray_roi = gray(roi);

    // Isolate this connected part, excluding holes and disconnected parts nested inside.
    // Mask-only flood fill leaves both the image and the segmentation unchanged.
    cv::Mat foreground_roi = foreground(roi);
    cv::Mat flood_mask = cv::Mat::zeros(roi.height + 2, roi.width + 2, CV_8UC1);
    cv::floodFill(foreground_roi, flood_mask, contour.front() - roi.tl(), cv::Scalar(0),
        nullptr, cv::Scalar(0), cv::Scalar(0), 8 | cv::FLOODFILL_MASK_ONLY | (255 << 8));
    const cv::Mat component_mask = flood_mask(cv::Rect(1, 1, roi.width, roi.height));
    const double foreground_mean = cv::mean(gray_roi, component_mask)[0];
    shape.mean_brightness = static_cast<float>(foreground_mean);

    // Sample nearby background around the part (including hole boundaries).
    // Exclude every selected foreground region, as well as pixels outside the product.
    cv::Mat background_mask, opposite_mask, product_region = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::dilate(component_mask, background_mask, cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * kBackgroundRadius + 1, 2 * kBackgroundRadius + 1)),
        cv::Point(-1, -1), 1, cv::BORDER_CONSTANT | cv::BORDER_ISOLATED, cv::Scalar(0));
    cv::bitwise_not(foreground_roi, opposite_mask);
    cv::bitwise_and(background_mask, opposite_mask, background_mask);
    cv::drawContours(product_region, std::vector<std::vector<cv::Point>>{product_contour},
        0, cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), 0, -roi.tl());
    cv::bitwise_and(background_mask, product_region, background_mask);
    if (cv::countNonZero(background_mask) > 0) {
        const double background_mean = cv::mean(gray_roi, background_mask)[0];
        shape.contrast = static_cast<float>(std::abs(foreground_mean - background_mean));
    }

    // Same unscaled 3x3 Sobel/Tenengrad energy formula as Alg_CH, sampled on our contour band.
    cv::Mat dx, dy;
    cv::Sobel(gray_roi, dx, CV_32F, 1, 0, 3);
    cv::Sobel(gray_roi, dy, CV_32F, 0, 1, 3);
    const cv::Mat energy = dx.mul(dx) + dy.mul(dy);
    cv::Mat band_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::drawContours(band_mask, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(255),
        2 * kContourBand + 1, cv::LINE_8, cv::noArray(), 0, -roi.tl());
    shape.sharpness = static_cast<float>(cv::mean(energy, band_mask)[0]);
}

int extract_shapes(const CHImage& src, const Image_Sharpness_Orientations_Params& params,
    cv::Size& image_size, Image_Sharpness_Orientations_Shapes& shapes, Product_Frame& frame) {
    if (src.empty()) return ISO_ImageIsEmpty;
    if (src.dims != 2 || src.depth() != CV_8U ||
        (src.channels() != 1 && src.channels() != 3 && src.channels() != 4)) {
        return ISO_UnsupportedImage;
    }
    image_size = src.size();

    cv::Mat gray;
    if (src.channels() == 1) gray = src;
    else cv::cvtColor(src, gray, src.channels() == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);

    cv::Mat filtered, foreground;
    cv::GaussianBlur(gray, filtered, cv::Size(kBlurSize, kBlurSize), 0.0);
    const int threshold_type = params.detect_dark_area ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    cv::threshold(filtered, foreground, std::max(params.threshold, 0), 255,
        threshold_type | (params.threshold < 0 ? cv::THRESH_OTSU : 0));

    // The product has the opposite polarity to its marks: bright for dark marks,
    // dark for light marks. Its largest silhouette provides position and scale.
    cv::Mat product_mask;
    cv::bitwise_not(foreground, product_mask);
    std::vector<std::vector<cv::Point>> product_contours;
    cv::findContours(product_mask, product_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (product_contours.empty()) return ISO_NoShapes;
    const auto product_it = std::max_element(product_contours.begin(), product_contours.end(),
        [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
    const cv::Moments product_moments = cv::moments(*product_it);
    if (product_moments.m00 <= 0.0) return ISO_NoShapes;
    frame.area = product_moments.m00;
    frame.center = {product_moments.m10 / frame.area, product_moments.m01 / frame.area};
    // Whiten the silhouette covariance: remove independent x/y scale and shear,
    // leaving only a rotation between affine views of the same product. Using
    // the outer silhouette makes this independent of missing or extra marks.
    const double xx = product_moments.mu20 / frame.area;
    const double xy = product_moments.mu11 / frame.area;
    const double yy = product_moments.mu02 / frame.area;
    const double root_det = std::sqrt(std::max(0.0, xx * yy - xy * xy));
    if (root_det > 1e-9) {
        frame.normalization = cv::Matx22d(yy + root_det, -xy, -xy, xx + root_det) *
            (1.0 / (2.0 * std::sqrt(kPi) * root_det * std::sqrt(xx + yy + 2.0 * root_det)));
    } else {
        frame.normalization = cv::Matx22d::eye() * (1.0 / std::sqrt(frame.area));
    }

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // CCOMP gives each selected connected component its own outer contour.
    // Opposite-polarity holes remain children; border-connected background is skipped.
    cv::findContours(foreground, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    const double maximum_area = kMaxAreaRatio * static_cast<double>(src.total());
    for (size_t i = 0; i < contours.size(); ++i) {
        if (hierarchy[i][3] >= 0) continue; // Do not count holes as separate parts.
        const auto& contour = contours[i];
        const cv::Rect bounds = cv::boundingRect(contour);
        if (bounds.x <= 0 || bounds.y <= 0 ||
            bounds.x + bounds.width >= src.cols || bounds.y + bounds.height >= src.rows) continue;
        const cv::Moments m = cv::moments(contour);
        if (m.m00 < params.min_area || m.m00 > maximum_area) continue;
        const cv::Point2f center(static_cast<float>(m.m10 / m.m00),
            static_cast<float>(m.m01 / m.m00));
        if (cv::pointPolygonTest(*product_it, center, false) < 0.0) continue;

        Image_Sharpness_Orientations_Shape shape;
        shape.center = center;
        shape.bounds = bounds;
        shape.area = m.m00;
        shape.contour.reserve(contour.size());
        for (const auto& point : contour) shape.contour.emplace_back(point);
        measure_axis(m, kMinOrientationAnisotropy,
            shape.orientation, shape.orientation_valid);
        measure_quality(gray, foreground, contour, *product_it, shape);
        shapes.push_back(std::move(shape));
    }
    std::sort(shapes.begin(), shapes.end(), [](const auto& a, const auto& b) {
        if (a.center.x != b.center.x) return a.center.x < b.center.x;
        return a.center.y < b.center.y;
    });
    return shapes.empty() ? ISO_NoShapes : ISO_OK;
}

cv::Point2d normalized_center(const Image_Sharpness_Orientations_Shape& shape,
    const Product_Frame& frame) {
    const cv::Point2d offset = cv::Point2d(shape.center) - frame.center;
    const auto point = frame.normalization * cv::Vec2d(offset.x, offset.y);
    return {point[0], point[1]};
}

cv::Point2d rotate_point(const cv::Point2d& point, double rotation) {
    const double radians = rotation * kPi / 180.0;
    const double c = std::cos(radians), s = std::sin(radians);
    return {c * point.x - s * point.y, s * point.x + c * point.y};
}

// Compactness and moment anisotropy remain stable for thin, slightly blurred strokes.
// Log compactness is scale/rotation invariant; anisotropy is bounded in [0, 1].
cv::Vec2d shape_descriptor(const CHContour& contour) {
    const auto m = cv::moments(contour);
    const double perimeter = cv::arcLength(contour, true);
    return {std::log(perimeter * perimeter / m.m00),
        std::hypot(m.mu20 - m.mu02, 2.0 * m.mu11) / (m.mu20 + m.mu02)};
}

// Soft horizontal/vertical ranks describe the whole layout. Near-equal x or y
// coordinates are ties, so two vertically stacked parts need not keep x order.
// Fractions, rather than absolute ranks, tolerate missing and extra parts.
std::vector<cv::Point2d> layout_order(const std::vector<cv::Point2d>& points) {
    std::vector<cv::Point2d> order(points.size());
    if (points.size() < 2) return order;
    const double denominator = static_cast<double>(points.size() - 1);
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j) continue;
            const auto difference = (points[i] - points[j]) * (1.0 / 0.04);
            order[i].x += std::clamp(difference.x, -1.0, 1.0) / denominator;
            order[i].y += std::clamp(difference.y, -1.0, 1.0) / denominator;
        }
    }
    return order;
}

// Rectangular Hungarian assignment (rows <= columns). Dummy columns allow
// missing base parts; each real column can be used at most once. Unlike greedy
// pairs, this minimizes the cost of the complete assignment.
std::vector<size_t> minimum_assignment(const std::vector<std::vector<double>>& costs) {
    const size_t rows = costs.size(), columns = costs.front().size();
    std::vector<double> u(rows + 1), v(columns + 1);
    std::vector<size_t> owner(columns + 1), previous(columns + 1);
    for (size_t row = 1; row <= rows; ++row) {
        owner[0] = row;
        size_t column = 0;
        std::vector<double> minimum(columns + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(columns + 1, false);
        do {
            used[column] = true;
            const size_t active_row = owner[column];
            double delta = std::numeric_limits<double>::infinity();
            size_t next = 0;
            for (size_t j = 1; j <= columns; ++j) {
                if (used[j]) continue;
                const double reduced = costs[active_row - 1][j - 1] - u[active_row] - v[j];
                if (reduced < minimum[j]) {
                    minimum[j] = reduced;
                    previous[j] = column;
                }
                if (minimum[j] < delta) {
                    delta = minimum[j];
                    next = j;
                }
            }
            for (size_t j = 0; j <= columns; ++j) {
                if (used[j]) {
                    u[owner[j]] += delta;
                    v[j] -= delta;
                } else minimum[j] -= delta;
            }
            column = next;
        } while (owner[column] != 0);
        do {
            const size_t next = previous[column];
            owner[column] = owner[next];
            column = next;
        } while (column != 0);
    }
    std::vector<size_t> assignment(rows);
    for (size_t j = 1; j <= columns; ++j) {
        if (owner[j] != 0) assignment[owner[j] - 1] = j - 1;
    }
    return assignment;
}

struct Layout_Match {
    std::vector<size_t> assignment;
    std::vector<double> costs;
    double score = std::numeric_limits<double>::infinity();
    double rotation = 0.0;
};

Layout_Match match_layout(const Image_Sharpness_Orientations_Register_Result& base,
    const Product_Frame& base_frame, const Product_Frame& current_frame,
    const Image_Sharpness_Orientations_Shapes& current) {
    std::vector<cv::Point2d> base_points, current_points;
    std::vector<cv::Vec2d> base_descriptors, current_descriptors;
    for (const auto& part : base.shapes) {
        base_points.push_back(normalized_center(part, base_frame));
        base_descriptors.push_back(shape_descriptor(part.contour));
    }
    for (const auto& part : current) {
        current_points.push_back(normalized_center(part, current_frame));
        current_descriptors.push_back(shape_descriptor(part.contour));
    }
    const auto base_order = layout_order(base_points);
    const auto evaluate = [&](double rotation) {
        std::vector<cv::Point2d> aligned;
        for (const auto& point : current_points) aligned.push_back(rotate_point(point, rotation));
        const auto current_order = layout_order(aligned);
        std::vector<std::vector<double>> costs(base.shapes.size(),
            std::vector<double>(current.size() + base.shapes.size(), kUnmatchedCost));
        for (size_t i = 0; i < base.shapes.size(); ++i) {
            for (size_t j = 0; j < current.size(); ++j) {
                const double distance = cv::norm(base_points[i] - aligned[j]);
                if (distance > kMaxMatchDistance) {
                    costs[i][j] = 1e6;
                    continue;
                }
                const double area_ratio = (current[j].area / current_frame.area) /
                    (base.shapes[i].area / base_frame.area);
                const auto descriptors = base_descriptors[i] - current_descriptors[j];
                const double shape_distance = std::abs(descriptors[0]) + std::abs(descriptors[1]);
                const auto order = base_order[i] - current_order[j];
                // Geometry and order dominate. Appearance penalties are bounded:
                // a deformed part must remain identifiable for later measurement.
                const double cost = distance * distance / (kMaxMatchDistance * kMaxMatchDistance) +
                    0.15 * (std::abs(order.x) + std::abs(order.y)) +
                    0.08 * std::min(1.0, std::abs(std::log(area_ratio)) / std::log(2.0)) +
                    0.04 * std::min(1.0, shape_distance);
                // Matching also removes the penalty for an extra current part.
                costs[i][j] = cost - kUnmatchedCost;
            }
        }
        Layout_Match match;
        match.rotation = rotation;
        match.assignment = minimum_assignment(costs);
        match.score = kUnmatchedCost * static_cast<double>(base.shapes.size() + current.size());
        for (size_t i = 0; i < match.assignment.size(); ++i) {
            const size_t j = match.assignment[i];
            const double cost = j < current.size() ? costs[i][j] + kUnmatchedCost : 0.0;
            match.costs.push_back(cost);
            if (j < current.size()) match.score += cost - 2.0 * kUnmatchedCost;
        }
        return match;
    };

    // Silhouette whitening leaves a rotation, which is chosen using the complete
    // mark layout rather than the product's principal axis or a few greedy pairs.
    auto best = evaluate(0.0);
    const auto consider = [&](double rotation) {
        auto candidate = evaluate(rotation);
        if (candidate.score < best.score - 1e-12) best = std::move(candidate);
    };
    for (int angle = -90; angle <= 90; angle += 5) consider(static_cast<double>(angle));
    for (double step : {1.0, 0.2}) {
        const double center = best.rotation;
        for (int offset = -4; offset <= 4; ++offset) {
            const double angle = center + step * offset;
            if (std::abs(angle) <= 90.0) consider(angle);
        }
    }
    return best;
}

void match_shapes(const Image_Sharpness_Orientations_Register_Result& base, const Product_Frame& base_frame,
    const Product_Frame& current_frame, Image_Sharpness_Orientations_Detect_Result& result) {
    result.missing_ids.clear();
    result.angle = 0.0;
    result.angle_valid = false;
    for (auto& shape : result.shapes) {
        shape.id = -1;
        shape.matched = false;
        shape.match_cost = -1.0;
        shape.angle = 0.0;
        shape.angle_valid = false;
    }
    const auto assignment = match_layout(base, base_frame, current_frame, result.shapes);
    std::vector<bool> base_used(base.shapes.size(), false);
    std::vector<std::pair<size_t, size_t>> matches;
    for (size_t i = 0; i < assignment.assignment.size(); ++i) {
        const size_t j = assignment.assignment[i];
        if (j >= result.shapes.size()) continue;
        base_used[i] = true;
        const auto& model = base.shapes[i];
        auto& shape = result.shapes[j];
        shape.id = model.id;
        shape.matched = true;
        shape.match_cost = assignment.costs[i];
        shape.angle_valid = model.orientation_valid && shape.orientation_valid;
        if (shape.angle_valid) shape.angle = axis_difference(shape.orientation - model.orientation);
        matches.emplace_back(i, j);
    }
    result.base_shape_count = static_cast<int>(base.shapes.size());
    result.matched_count = static_cast<int>(matches.size());
    for (size_t i = 0; i < base_used.size(); ++i) {
        if (!base_used[i]) result.missing_ids.push_back(base.shapes[i].id);
    }
    result.all_matched = result.missing_ids.empty() && matches.size() == result.shapes.size();

    // Least-squares rotation of matched centers; translation and uniform scale cancel.
    if (matches.size() >= 2) {
        cv::Point2d base_mean(0.0, 0.0), current_mean(0.0, 0.0);
        for (const auto& pair : matches) {
            base_mean += cv::Point2d(base.shapes[pair.first].center);
            current_mean += cv::Point2d(result.shapes[pair.second].center);
        }
        base_mean *= 1.0 / static_cast<double>(matches.size());
        current_mean *= 1.0 / static_cast<double>(matches.size());
        double dot = 0.0, cross = 0.0;
        for (const auto& pair : matches) {
            const auto a = cv::Point2d(base.shapes[pair.first].center) - base_mean;
            const auto b = cv::Point2d(result.shapes[pair.second].center) - current_mean;
            dot += a.dot(b);
            cross += a.x * b.y - a.y * b.x;
        }
        result.angle_valid = std::hypot(dot, cross) > 1e-9;
        if (result.angle_valid) {
            result.angle = std::atan2(cross, dot) * 180.0 / kPi;
            if (result.angle >= 180.0) result.angle -= 360.0;
        }
    } else if (matches.size() == 1) {
        const auto& shape = result.shapes[matches[0].second];
        result.angle_valid = shape.angle_valid;
        result.angle = shape.angle;
    }
    std::stable_sort(result.shapes.begin(), result.shapes.end(), [](const auto& a, const auto& b) {
        if (a.matched != b.matched) return a.matched;
        return a.matched && a.id < b.id;
    });
}

} // namespace

class CH_Image_Sharpness_Orientations::Impl {
public:
    Image_Sharpness_Orientations_Params params;
    Image_Sharpness_Orientations_Register_Result base;
    Product_Frame frame;
};

extern "C" int test_IMAGE_SHARPNESS_ORIENTATIONS_API() { return ISO_OK; }

CH_Image_Sharpness_Orientations::CH_Image_Sharpness_Orientations() : impl_(nullptr) {}
CH_Image_Sharpness_Orientations::~CH_Image_Sharpness_Orientations() { delete impl_; }

int CH_Image_Sharpness_Orientations::RegisterBase(const CHImage& src,
    Image_Sharpness_Orientations_Register_Result& result, Image_Sharpness_Orientations_Params params) {
    result = {};
    if (!valid_params(params)) return ISO_InvalidParameter;
    try {
        auto next = std::make_unique<Impl>();
        next->params = params;
        const int status = extract_shapes(src, params, next->base.image_size, next->base.shapes, next->frame);
        if (status != ISO_OK) return status;
        for (size_t i = 0; i < next->base.shapes.size(); ++i) {
            next->base.shapes[i].id = static_cast<int>(i) + 1;
        }
        result = next->base;
        // Commit only after extraction and all output allocations succeed.
        delete impl_;
        impl_ = next.release();
        return ISO_OK;
    } catch (const std::exception&) {
        result = {};
        return ISO_Fail;
    }
}

int CH_Image_Sharpness_Orientations::Detect(const CHImage& src,
    Image_Sharpness_Orientations_Detect_Result& result) const {
    result = {};
    if (!impl_) return ISO_BaseNotRegistered;
    try {
        Product_Frame frame;
        const int status = extract_shapes(src, impl_->params, result.image_size, result.shapes, frame);
        if (status == ISO_NoShapes) {
            result.base_shape_count = static_cast<int>(impl_->base.shapes.size());
            for (const auto& shape : impl_->base.shapes) result.missing_ids.push_back(shape.id);
            return status;
        }
        if (status != ISO_OK) return status;
        match_shapes(impl_->base, impl_->frame, frame, result);
        return ISO_OK;
    } catch (const std::exception&) {
        result = {};
        return ISO_Fail;
    }
}
