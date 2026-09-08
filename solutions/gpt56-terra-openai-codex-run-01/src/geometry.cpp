#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace vwmini {
namespace {

constexpr double kAreaEpsilon = 1.0e-8;

[[nodiscard]] bool finite(Vec2 value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y);
}

[[nodiscard]] double orient(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y)
        - (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
}

[[nodiscard]] double signed_area(const std::vector<Vec2>& vertices) noexcept
{
    double twice_area = 0.0;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const Vec2 a = vertices[i];
        const Vec2 b = vertices[(i + 1) % vertices.size()];
        twice_area += static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
    }
    return twice_area * 0.5;
}

[[nodiscard]] bool on_closed_segment(Vec2 point, Vec2 a, Vec2 b) noexcept
{
    return orient(a, b, point) == 0.0
        && point.x >= std::min(a.x, b.x) && point.x <= std::max(a.x, b.x)
        && point.y >= std::min(a.y, b.y) && point.y <= std::max(a.y, b.y);
}

[[nodiscard]] int orientation_sign(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    const double value = orient(a, b, c);
    return (value > 0.0) - (value < 0.0);
}

[[nodiscard]] bool segments_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const int ab_c = orientation_sign(a, b, c);
    const int ab_d = orientation_sign(a, b, d);
    const int cd_a = orientation_sign(c, d, a);
    const int cd_b = orientation_sign(c, d, b);
    if (ab_c != ab_d && cd_a != cd_b) {
        return true;
    }
    return (ab_c == 0 && on_closed_segment(c, a, b))
        || (ab_d == 0 && on_closed_segment(d, a, b))
        || (cd_a == 0 && on_closed_segment(a, c, d))
        || (cd_b == 0 && on_closed_segment(b, c, d));
}

[[nodiscard]] bool strictly_in_triangle(Vec2 point, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return orient(a, b, point) > 0.0 && orient(b, c, point) > 0.0
        && orient(c, a, point) > 0.0;
}

[[nodiscard]] Error invalid_argument(const char* message)
{
    return {ErrorCode::InvalidArgument, message};
}

[[nodiscard]] Error invalid_mesh(const char* message)
{
    return {ErrorCode::InvalidMesh, message};
}

} // namespace

float length(Vec2 value) noexcept
{
    return std::hypot(value.x, value.y);
}

Vec2 normalized(Vec2 value) noexcept
{
    const float value_length = length(value);
    if (value_length == 0.0F || !std::isfinite(value_length)) {
        return {0.0F, 0.0F};
    }
    return value * (1.0F / value_length);
}

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon)
{
    const std::vector<Vec2>& vertices = polygon.vertices;
    for (const Vec2 vertex : vertices) {
        if (!finite(vertex)) {
            return std::unexpected(invalid_argument("polygon vertex is not finite"));
        }
    }
    if (vertices.size() < 3) {
        return std::unexpected(invalid_mesh("polygon has fewer than three vertices"));
    }
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        if (vertices[i] == vertices[(i + 1) % vertices.size()]) {
            return std::unexpected(invalid_mesh("polygon has consecutive duplicate vertices"));
        }
    }
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const std::size_t next_i = (i + 1) % vertices.size();
        for (std::size_t j = i + 1; j < vertices.size(); ++j) {
            const std::size_t next_j = (j + 1) % vertices.size();
            const bool adjacent = i == j || next_i == j || next_j == i;
            if (!adjacent
                && segments_intersect(vertices[i], vertices[next_i], vertices[j], vertices[next_j])) {
                return std::unexpected(invalid_mesh("polygon self-intersects"));
            }
        }
    }

    const double area = signed_area(vertices);
    if (area <= kAreaEpsilon) {
        return std::unexpected(invalid_mesh("polygon is clockwise or degenerate"));
    }

    std::vector<std::size_t> remaining(vertices.size());
    for (std::size_t i = 0; i < remaining.size(); ++i) {
        remaining[i] = i;
    }
    std::vector<Polygon> result;
    result.reserve(vertices.size() - 2);

    while (remaining.size() > 3) {
        bool clipped = false;
        for (std::size_t i = 0; i < remaining.size(); ++i) {
            const std::size_t previous = remaining[(i + remaining.size() - 1) % remaining.size()];
            const std::size_t current = remaining[i];
            const std::size_t next = remaining[(i + 1) % remaining.size()];
            if (orient(vertices[previous], vertices[current], vertices[next]) <= kAreaEpsilon) {
                continue;
            }
            bool has_interior_vertex = false;
            for (const std::size_t candidate : remaining) {
                if (candidate != previous && candidate != current && candidate != next
                    && strictly_in_triangle(vertices[candidate], vertices[previous], vertices[current],
                                            vertices[next])) {
                    has_interior_vertex = true;
                    break;
                }
            }
            if (has_interior_vertex) {
                continue;
            }
            result.push_back({{{vertices[previous], vertices[current], vertices[next]}}});
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            return std::unexpected(invalid_mesh("polygon cannot be triangulated"));
        }
    }
    if (orient(vertices[remaining[0]], vertices[remaining[1]], vertices[remaining[2]])
        <= kAreaEpsilon) {
        return std::unexpected(invalid_mesh("polygon triangulation has a degenerate triangle"));
    }
    result.push_back({{{vertices[remaining[0]], vertices[remaining[1]], vertices[remaining[2]]}}});
    return result;
}

} // namespace vwmini
