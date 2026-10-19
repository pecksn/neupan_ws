#include "../src/cloud_path_simplification.h"

#include <iostream>
#include <limits>
#include <stdexcept>

struct Point {
    double x;
    double y;
};

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool equal(const Point& a, const Point& b) {
    return a.x == b.x && a.y == b.y;
}

// Every retained point must remain in original order; every removed point
// must be within tolerance of its replacement segment.
void verify(const std::vector<Point>& source, const std::vector<Point>& simplified, double tolerance) {
    require(equal(source.front(), simplified.front()), "Start changed");
    require(equal(source.back(), simplified.back()), "End changed");
    std::size_t begin = 0;
    for (std::size_t k = 1; k < simplified.size(); ++k) {
        std::size_t end = begin + 1;
        while (end < source.size() && !equal(source[end], simplified[k])) {
            ++end;
        }
        require(end < source.size(), "Point order changed");
        for (std::size_t i = begin; i <= end; ++i) {
            require(osm_map_handler::cloudPointSegmentDistanceSquared(source[i], source[begin], source[end])
                    <= tolerance * tolerance + 1e-10, "Deviation exceeds tolerance");
        }
        begin = end;
    }
}

int main() {
    using osm_map_handler::simplifyCloudPath;
    std::vector<Point> straight;
    for (int i = 0; i <= 40; ++i) {
        straight.push_back({i * 2.5, 0.0});
    }
    const auto saved = straight;
    const auto line = simplifyCloudPath(straight, 0.10);
    require(line.size() == 2, "Straight must have exactly two endpoints");
    verify(straight, line, 0.10);
    for (std::size_t i = 0; i < straight.size(); ++i) {
        require(equal(straight[i], saved[i]), "Local source path mutated");
    }
    std::cout << "100m straight: " << straight.size() << " -> " << line.size() << '\n';

    auto corner = straight;
    for (int i = 1; i <= 40; ++i) {
        corner.push_back({100.0, i * 2.5});
    }
    const auto turn = simplifyCloudPath(corner, 0.10);
    require(turn.size() == 3 && equal(turn[1], {100, 0}), "Right-angle corner lost");
    verify(corner, turn, 0.10);
    std::cout << "Right-angle turn: " << corner.size() << " -> " << turn.size() << '\n';

    std::vector<Point> curve;
    for (int i = 0; i <= 40; ++i) {
        const double angle = (std::acos(-1.0) / 2.0) * i / 40;
        curve.push_back({5 * std::cos(angle), 5 * std::sin(angle)});
    }
    const auto arc = simplifyCloudPath(curve, 0.10);
    require(arc.size() > 3 && arc.size() < curve.size(), "Curve detail not preserved");
    verify(curve, arc, 0.10);
    std::cout << "5m-radius curve: " << curve.size() << " -> " << arc.size() << '\n';

    const std::vector<Point> backtrack{{0, 0}, {5, 0}, {2, 0}, {10, 0}};
    require(simplifyCloudPath(backtrack, 0.10).size() == 4, "Collinear reversals lost");
    const std::vector<Point> duplicate_turn{{0, 0}, {5, 0}, {5, 0}, {2, 0}, {10, 0}};
    require(simplifyCloudPath(duplicate_turn, 0.10).size() == 4, "Repeated point hid reversal");
    const std::vector<Point> closed{{0, 0}, {5, 0}, {5, 5}, {0, 5}, {0, 0}};
    verify(closed, simplifyCloudPath(closed, 0.10), 0.10);
    require(simplifyCloudPath(std::vector<Point>{}, 0.10).empty(), "Empty input changed");
    require(simplifyCloudPath(std::vector<Point>{{0, 0}}, 0.10).size() == 1, "Singleton changed");
    require(simplifyCloudPath(std::vector<Point>{{0, 0}, {1, 0}}, 0.10).size() == 2, "Two-point input changed");
    require(simplifyCloudPath(std::vector<Point>{{1, 1}, {1, 1}, {1, 1}}, 0.10).size() == 2, "Degenerate endpoints lost");
    require(simplifyCloudPath(straight, 0).size() == straight.size(), "Zero tolerance must disable filtering");
    require(simplifyCloudPath(straight, std::numeric_limits<double>::quiet_NaN()).size() == straight.size(),
            "Invalid tolerance not handled");
    std::cout << "Endpoints, order, error bound, reversals, duplicates, disable and edge cases passed\n";
}
