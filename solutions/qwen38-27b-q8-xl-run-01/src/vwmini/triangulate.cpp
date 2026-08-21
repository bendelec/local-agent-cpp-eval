#include "vwmini/triangulate.hpp"

#include "vwmini/geometry_priv.hpp"

#include <numeric>
#include <optional>

namespace vwmini::detail {

namespace {

bool is_cyclic_neighbor(std::size_t i, std::size_t j, std::size_t n) noexcept
{
    return i == (j + 1) % n || j == (i + 1) % n;
}

bool on_triangle_boundary(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return point_on_segment_exact(p, a, b) || point_on_segment_exact(p, b, c) ||
           point_on_segment_exact(p, c, a);
}

// MSH-002 validation; returns nullopt on success, otherwise the error.
std::optional<Error> validate_outline(const Polygon& p)
{
    const auto& v = p.vertices;
    const std::size_t n = v.size();
    for (const Vec2& pt : v)
        if (!is_finite(pt))
            return Error{ErrorCode::InvalidArgument, "non-finite outline coordinate"};
    if (n < 3)
        return Error{ErrorCode::InvalidMesh, "outline has fewer than three vertices"};
    for (std::size_t i = 0; i < n; ++i)
        if (v[i] == v[(i + 1) % n])
            return Error{ErrorCode::InvalidMesh, "consecutive duplicate outline vertex"};

    // Self-intersection and boundary self-touching (exact float predicates;
    // sub-epsilon near-touches are numerically ambiguous and accepted).
    for (std::size_t i = 0; i < n; ++i)
    {
        const std::size_t i2 = (i + 1) % n;
        for (std::size_t j = i + 1; j < n; ++j)
        {
            if (is_cyclic_neighbor(i, j, n) || is_cyclic_neighbor(i2, j, n))
                continue;
            const std::size_t j2 = (j + 1) % n;
            if (segments_properly_intersect(v[i], v[i2], v[j], v[j2]) ||
                segments_collinear_overlap(v[i], v[i2], v[j], v[j2]))
                return Error{ErrorCode::InvalidMesh, "self-intersecting outline"};
            // i and j are not cyclic neighbours here (see continue), so v[j] is
            // no endpoint of [v[i], v[i2]]; for the symmetric test, i ~ j2 would
            // make v[i] an endpoint of [v[j], v[j2]] and must be excluded.
            if (point_on_segment_exact(v[j], v[i], v[i2]) ||
                (!is_cyclic_neighbor(j2, i, n) && point_on_segment_exact(v[i], v[j], v[j2])))
                return Error{ErrorCode::InvalidMesh, "self-touching outline"};
        }
    }
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 2; j < n; ++j)
        {
            // (0, n-1) are cyclic neighbours: an equal first/last pair is
            // already rejected as a consecutive duplicate.
            if (!(i == 0 && j == n - 1) && v[i] == v[j])
                return Error{ErrorCode::InvalidMesh, "repeated outline vertex"};
        }

    const float area2 = 2.0f * polygon_area(p);
    if (area2 < 0.0f)
        return Error{ErrorCode::InvalidMesh, "clockwise outline winding"};
    if (area2 <= epsilon2)
        return Error{ErrorCode::InvalidMesh, "degenerate outline area"};
    return std::nullopt;
}

// First ring position whose ear triangle passes the area requirement and
// contains no other ring vertex (strictly interior or on the boundary).
std::size_t find_ear(const std::vector<Vec2>& v, const std::vector<std::size_t>& ring,
                     bool require_area) noexcept
{
    const std::size_t m = ring.size();
    for (std::size_t i = 0; i < m; ++i)
    {
        const std::size_t a = ring[(i + m - 1) % m];
        const std::size_t b = ring[i];
        const std::size_t c = ring[(i + 1) % m];
        const float a2 = triangle_area2(v[a], v[b], v[c]);
        if (a2 <= (require_area ? epsilon2 : 0.0f))
            continue;
        bool blocked = false;
        for (std::size_t k = 0; k < m && !blocked; ++k)
        {
            if (k == i || k == (i + m - 1) % m || k == (i + 1) % m)
                continue;
            const Vec2 p = v[ring[k]];
            if (strictly_inside(p, v[a], v[b], v[c]) ||
                on_triangle_boundary(p, v[a], v[b], v[c]))
                blocked = true;
        }
        if (!blocked)
            return i;
    }
    return m; // sentinel: none found
}

} // namespace

bool clip_ring(const std::vector<Vec2>& v, std::vector<std::size_t>& ring,
               std::vector<Polygon>& out)
{
    while (ring.size() > 3)
    {
        std::size_t i = find_ear(v, ring, true);
        if (i == ring.size())
            i = find_ear(v, ring, false);
        if (i == ring.size())
            return false;
        const std::size_t m = ring.size();
        out.push_back(Polygon{{v[ring[(i + m - 1) % m]], v[ring[i]], v[ring[(i + 1) % m]]}});
        ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
    }
    const float a2 = triangle_area2(v[ring[0]], v[ring[1]], v[ring[2]]);
    if (a2 <= 0.0f)
        return false;
    out.push_back(Polygon{{v[ring[0]], v[ring[1]], v[ring[2]]}});
    return true;
}

Result<std::vector<Polygon>> triangulate_polygon(const Polygon& polygon)
{
    if (auto err = validate_outline(polygon))
        return std::unexpected(*err);

    std::vector<std::size_t> ring(polygon.vertices.size());
    std::iota(ring.begin(), ring.end(), std::size_t{0});
    std::vector<Polygon> triangles;
    triangles.reserve(ring.size() - 2);
    if (!clip_ring(polygon.vertices, ring, triangles))
        return fail(ErrorCode::InvalidMesh, "outline could not be triangulated");
    return triangles;
}

} // namespace vwmini::detail
