#include <vwmini/geometry.hpp>

#include "vwmini/internal/errors.hpp"
#include "vwmini/internal/predicates.hpp"

#include <algorithm>
#include <cstddef>
#include <expected>
#include <span>
#include <utility>
#include <vector>

namespace vwmini {
namespace {

using internal::is_finite;
using internal::kMinArea;
using internal::make_error;
using internal::near;
using internal::non_degenerate;
using internal::orient;
using internal::point_in_triangle_strict;
using internal::segments_intersect;

// -- Validation (MSH-002) ---------------------------------------------------

[[nodiscard]] std::expected<void, Error> validate_outline(std::span<const Vec2> v)
{
    if (std::ranges::any_of(v, [](Vec2 p) { return !is_finite(p); })) {
        return make_error(ErrorCode::InvalidArgument, "outline has a non-finite coordinate");
    }
    if (v.size() < 3) {
        return make_error(ErrorCode::InvalidMesh, "outline needs at least three vertices");
    }

    const std::size_t n = v.size();
    // Cyclic consecutive duplicates (including equal first and last) are rejected.
    for (std::size_t i = 0; i < n; ++i) {
        if (near(v[i], v[(i + 1) % n])) {
            return make_error(ErrorCode::InvalidMesh, "outline has consecutive duplicate vertices");
        }
    }

    // Counter-clockwise and non-degenerate area (signed shoelace double area).
    double double_area = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1) % n];
        double_area += double(a.x) * double(b.y) - double(b.x) * double(a.y);
    }
    if (double_area <= kMinArea) {
        return make_error(ErrorCode::InvalidMesh, "outline is clockwise or degenerate");
    }

    // No self-intersection between non-adjacent edges.
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const bool adjacent = (j == (i + 1) % n) || (i == (j + 1) % n);
            if (adjacent) {
                continue;
            }
            if (segments_intersect(v[i], v[(i + 1) % n], v[j], v[(j + 1) % n])) {
                return make_error(ErrorCode::InvalidMesh, "outline self-intersects");
            }
        }
    }
    return {};
}

// Removes vertices that are collinear with their neighbors. A simple polygon with
// positive area always retains at least three non-collinear vertices, so this cannot
// reduce a valid outline below a triangle; removing zero-area corners preserves area.
[[nodiscard]] std::vector<Vec2> drop_collinear(std::vector<Vec2> poly)
{
    bool changed = true;
    while (changed && poly.size() > 3) {
        changed = false;
        std::vector<Vec2> kept;
        kept.reserve(poly.size());
        const std::size_t m = poly.size();
        for (std::size_t i = 0; i < m; ++i) {
            const Vec2 prev = poly[(i + m - 1) % m];
            const Vec2 cur = poly[i];
            const Vec2 next = poly[(i + 1) % m];
            if (non_degenerate(prev, cur, next)) {
                kept.push_back(cur);
            } else {
                changed = true;
            }
        }
        if (kept.size() >= 3) {
            poly = std::move(kept);
        } else {
            break;
        }
    }
    return poly;
}

// -- Ear clipping (MSH-003) --------------------------------------------------

[[nodiscard]] bool is_ear(std::span<const Vec2> poly, std::size_t i)
{
    const std::size_t m = poly.size();
    const Vec2 a = poly[(i + m - 1) % m];
    const Vec2 b = poly[i];
    const Vec2 c = poly[(i + 1) % m];
    if (orient(a, b, c) <= kMinArea) {
        return false; // reflex or degenerate corner: not a clippable ear
    }
    for (std::size_t k = 0; k < m; ++k) {
        if (k == (i + m - 1) % m || k == i || k == (i + 1) % m) {
            continue;
        }
        if (point_in_triangle_strict(poly[k], a, b, c)) {
            return false; // another vertex inside: not an ear
        }
    }
    return true;
}

} // namespace

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon)
{
    if (auto valid = validate_outline(polygon.vertices); !valid) {
        return std::unexpected(std::move(valid).error());
    }

    std::vector<Vec2> poly = drop_collinear(std::vector<Vec2>(polygon.vertices));
    std::vector<Polygon> triangles;
    triangles.reserve(poly.size() - 2);

    while (poly.size() > 3) {
        bool clipped = false;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            if (!is_ear(poly, i)) {
                continue;
            }
            const std::size_t m = poly.size();
            triangles.push_back(Polygon{{poly[(i + m - 1) % m], poly[i], poly[(i + 1) % m]}});
            poly.erase(poly.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            // Unreachable for a validated simple polygon; guards against a hang.
            return make_error(ErrorCode::InvalidMesh, "triangulation stalled");
        }
    }

    if (orient(poly[0], poly[1], poly[2]) <= kMinArea) {
        return make_error(ErrorCode::InvalidMesh, "triangulation produced a degenerate triangle");
    }
    triangles.push_back(Polygon{{poly[0], poly[1], poly[2]}});
    return triangles;
}

} // namespace vwmini
