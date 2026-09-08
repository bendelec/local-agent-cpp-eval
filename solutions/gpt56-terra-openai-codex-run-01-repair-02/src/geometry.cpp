#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace vwmini {
namespace {

constexpr double kSignedAreaEpsilon = 1.0e-8;
constexpr double kTwiceAreaEpsilon = 2.0 * kSignedAreaEpsilon;

struct DVec {
    double x{};
    double y{};
};

[[nodiscard]] DVec difference(Vec2 left, Vec2 right) noexcept {
    return {static_cast<double>(left.x) - static_cast<double>(right.x),
            static_cast<double>(left.y) - static_cast<double>(right.y)};
}

[[nodiscard]] double cross(DVec left, DVec right) noexcept {
    return left.x * right.y - left.y * right.x;
}

[[nodiscard]] bool finite(Vec2 value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

// The result is twice the signed triangle area. All predicate operands are widened before
// subtraction so finite float coordinates cannot overflow a geometric decision.
[[nodiscard]] double orient(Vec2 a, Vec2 b, Vec2 c) noexcept {
    return cross(difference(b, a), difference(c, a));
}

[[nodiscard]] double signed_area(const std::vector<Vec2> &vertices) noexcept {
    double twice_area = 0.0;
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const Vec2 a = vertices[index];
        const Vec2 b = vertices[(index + 1) % vertices.size()];
        twice_area += static_cast<double>(a.x) * static_cast<double>(b.y) -
                      static_cast<double>(a.y) * static_cast<double>(b.x);
    }
    return twice_area * 0.5;
}

[[nodiscard]] bool on_closed_segment(Vec2 point, Vec2 a, Vec2 b) noexcept {
    const DVec point_offset = difference(point, a);
    const DVec edge = difference(b, a);
    return cross(edge, point_offset) == 0.0 &&
           static_cast<double>(point.x) >=
               std::min(static_cast<double>(a.x), static_cast<double>(b.x)) &&
           static_cast<double>(point.x) <=
               std::max(static_cast<double>(a.x), static_cast<double>(b.x)) &&
           static_cast<double>(point.y) >=
               std::min(static_cast<double>(a.y), static_cast<double>(b.y)) &&
           static_cast<double>(point.y) <=
               std::max(static_cast<double>(a.y), static_cast<double>(b.y));
}

[[nodiscard]] int orientation_sign(Vec2 a, Vec2 b, Vec2 c) noexcept {
    const double value = orient(a, b, c);
    return (value > 0.0) - (value < 0.0);
}

[[nodiscard]] bool segments_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept {
    const int ab_c = orientation_sign(a, b, c);
    const int ab_d = orientation_sign(a, b, d);
    const int cd_a = orientation_sign(c, d, a);
    const int cd_b = orientation_sign(c, d, b);
    if (ab_c != ab_d && cd_a != cd_b) {
        return true;
    }
    return (ab_c == 0 && on_closed_segment(c, a, b)) || (ab_d == 0 && on_closed_segment(d, a, b)) ||
           (cd_a == 0 && on_closed_segment(a, c, d)) || (cd_b == 0 && on_closed_segment(b, c, d));
}

[[nodiscard]] bool strictly_in_triangle(Vec2 point, Vec2 a, Vec2 b, Vec2 c) noexcept {
    return orient(a, b, point) > 0.0 && orient(b, c, point) > 0.0 && orient(c, a, point) > 0.0;
}

[[nodiscard]] Error invalid_argument(const char *message) {
    return {ErrorCode::InvalidArgument, message};
}

[[nodiscard]] Error invalid_mesh(const char *message) {
    return {ErrorCode::InvalidMesh, message};
}

} // namespace

float length(Vec2 value) noexcept {
    const double value_length =
        std::hypot(static_cast<double>(value.x), static_cast<double>(value.y));
    // Vec2 stores floats; saturating is the only finite float representation for a longer finite
    // vector. Internal predicates and normalization never consume this saturated public result.
    return value_length > static_cast<double>(std::numeric_limits<float>::max())
               ? std::numeric_limits<float>::max()
               : static_cast<float>(value_length);
}

Vec2 normalized(Vec2 value) noexcept {
    const double value_length =
        std::hypot(static_cast<double>(value.x), static_cast<double>(value.y));
    if (value_length == 0.0) {
        return {0.0F, 0.0F};
    }
    return {static_cast<float>(static_cast<double>(value.x) / value_length),
            static_cast<float>(static_cast<double>(value.y) / value_length)};
}

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon &polygon) {
    const std::vector<Vec2> &vertices = polygon.vertices;
    for (const Vec2 vertex : vertices) {
        if (!finite(vertex)) {
            return std::unexpected(invalid_argument("polygon vertex is not finite"));
        }
    }
    if (vertices.size() < 3) {
        return std::unexpected(invalid_mesh("polygon has fewer than three vertices"));
    }
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        if (vertices[index] == vertices[(index + 1) % vertices.size()]) {
            return std::unexpected(invalid_mesh("polygon has consecutive duplicate vertices"));
        }
    }
    for (std::size_t first = 0; first < vertices.size(); ++first) {
        const std::size_t first_next = (first + 1) % vertices.size();
        for (std::size_t second = first + 1; second < vertices.size(); ++second) {
            const std::size_t second_next = (second + 1) % vertices.size();
            const bool adjacent = first == second || first_next == second || second_next == first;
            if (!adjacent && segments_intersect(vertices[first], vertices[first_next],
                                                vertices[second], vertices[second_next])) {
                return std::unexpected(invalid_mesh("polygon self-intersects"));
            }
        }
    }

    if (signed_area(vertices) <= kSignedAreaEpsilon) {
        return std::unexpected(invalid_mesh("polygon is clockwise or degenerate"));
    }

    std::vector<std::size_t> remaining(vertices.size());
    for (std::size_t index = 0; index < remaining.size(); ++index) {
        remaining[index] = index;
    }
    std::vector<Polygon> result;
    result.reserve(vertices.size() - 2);

    while (remaining.size() > 3) {
        bool clipped = false;
        for (std::size_t index = 0; index < remaining.size(); ++index) {
            const std::size_t previous =
                remaining[(index + remaining.size() - 1) % remaining.size()];
            const std::size_t current = remaining[index];
            const std::size_t next = remaining[(index + 1) % remaining.size()];
            if (orient(vertices[previous], vertices[current], vertices[next]) <=
                kTwiceAreaEpsilon) {
                continue;
            }
            bool has_interior_vertex = false;
            for (const std::size_t candidate : remaining) {
                if (candidate != previous && candidate != current && candidate != next &&
                    strictly_in_triangle(vertices[candidate], vertices[previous], vertices[current],
                                         vertices[next])) {
                    has_interior_vertex = true;
                    break;
                }
            }
            if (has_interior_vertex) {
                continue;
            }
            result.push_back({{{vertices[previous], vertices[current], vertices[next]}}});
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(index));
            clipped = true;
            break;
        }
        if (!clipped) {
            return std::unexpected(invalid_mesh("polygon cannot be triangulated"));
        }
    }
    if (orient(vertices[remaining[0]], vertices[remaining[1]], vertices[remaining[2]]) <=
        kTwiceAreaEpsilon) {
        return std::unexpected(invalid_mesh("polygon triangulation has a degenerate triangle"));
    }
    result.push_back({{{vertices[remaining[0]], vertices[remaining[1]], vertices[remaining[2]]}}});
    return result;
}

} // namespace vwmini
