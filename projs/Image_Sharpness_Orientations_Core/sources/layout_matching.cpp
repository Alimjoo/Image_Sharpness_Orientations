#include "image_sharpness_orientations_core.h"
#include "assignment.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace iso_detail {
namespace {

constexpr double kMaxMatchDistance = 0.12;
constexpr double kUnmatchedCost = 0.5;

double axis_difference(double angle) {
    while (angle >= 90.0) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;
    return angle;
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

} // namespace

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

} // namespace iso_detail
