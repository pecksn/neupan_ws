// Module contract: Geometry-only simplification for exported cloud paths. Preserve
// endpoint/order and tolerance semantics; the local tracking path is not replaced by
// this helper.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace osm_map_handler {

// Return squared planar point-to-segment distance in square metres, including
// degenerate-segment handling.
template <typename Point>
double cloudPointSegmentDistanceSquared(const Point &p, const Point &a,
                                        const Point &b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double t =
        length_squared > 0.0
            ? std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) /
                                              length_squared))
            : 0.0;
    const double ex = p.x - (a.x + t * dx);
    const double ey = p.y - (a.y + t * dy);
    return ex * ex + ey * ey;
}

// Display-only Ramer-Douglas-Peucker simplification, in map meters. Keep
// endpoints, order and direction reversals, including collinear out-and-back.
// Iterative subdivision avoids recursion overflow on long routes.
// Simplify within tolerance_m while retaining endpoints and route ordering;
// invalid/nonpositive tolerance leaves the source path unchanged.
template <typename Point>
std::vector<Point> simplifyCloudPath(const std::vector<Point> &path,
                                     double tolerance_m) {
    if (path.size() < 3 || !std::isfinite(tolerance_m) || tolerance_m <= 0.0) {
        return path;
    }
    std::vector<Point> points;
    points.reserve(path.size());
    for (const auto &point : path) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return path;
        }
        if (points.empty() || point.x != points.back().x ||
            point.y != points.back().y) {
            points.push_back(point);
        }
    }
    if (points.size() < 2) {
        return {path.front(), path.back()};
    }

    std::vector<bool> keep(points.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::size_t> anchors{0};
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const double dot =
            (points[i].x - points[i - 1].x) * (points[i + 1].x - points[i].x) +
            (points[i].y - points[i - 1].y) * (points[i + 1].y - points[i].y);
        if (dot < 0.0) {
            keep[i] = true;
            anchors.push_back(i);
        }
    }
    anchors.push_back(points.size() - 1);
    std::vector<std::pair<std::size_t, std::size_t>> pending;
    for (std::size_t i = 1; i < anchors.size(); ++i) {
        pending.emplace_back(anchors[i - 1], anchors[i]);
    }
    const double limit_squared = tolerance_m * tolerance_m;
    while (!pending.empty()) {
        const auto range = pending.back();
        pending.pop_back();
        double max_squared = limit_squared;
        std::size_t split = range.first;
        for (std::size_t i = range.first + 1; i < range.second; ++i) {
            const double distance = cloudPointSegmentDistanceSquared(
                points[i], points[range.first], points[range.second]);
            if (distance > max_squared) {
                max_squared = distance;
                split = i;
            }
        }
        if (split != range.first) {
            keep[split] = true;
            pending.emplace_back(range.first, split);
            pending.emplace_back(split, range.second);
        }
    }
    std::vector<Point> result;
    result.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (keep[i]) {
            result.push_back(points[i]);
        }
    }
    return result;
}

} // namespace osm_map_handler
