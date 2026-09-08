#include "triangulation.hpp"

#include "error.hpp"
#include "predicates.hpp"

#include <cstddef>
#include <expected>
#include <optional>
#include <vector>

namespace vwmini::detail
{
namespace
{

/**
 * Shoelace signed double area of a closed ring; positive when counter-clockwise.
 *
 * Measured from the ring's first vertex. Translating a ring leaves its area unchanged in exact
 * arithmetic, so the pivot costs nothing and keeps the accumulation away from cancellation for
 * rings that sit far from the origin, where raw shoelace terms swamp each other.
 */
[[nodiscard]] double ring_double_area(const std::vector<Vec2> &vertices)
{
    const std::size_t count = vertices.size();
    const Vec2d pivot = Vec2d(vertices.front());
    double area = 0.0;
    for (std::size_t i = 0; i < count; ++i)
    {
        area += cross_d(Vec2d(vertices[i]) - pivot, Vec2d(vertices[(i + 1) % count]) - pivot);
    }
    return area;
}

/// True when two non-adjacent outline edges cross, touch, or a vertex rests on a far edge.
[[nodiscard]] bool has_self_intersection(const std::vector<Vec2> &vertices)
{
    const std::size_t count = vertices.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        const Vec2 a1 = vertices[i];
        const Vec2 a2 = vertices[(i + 1) % count];
        for (std::size_t j = i + 1; j < count; ++j)
        {
            const bool adjacent = (j == i + 1) || (i == 0 && j + 1 == count);
            if (adjacent)
            {
                continue;
            }
            const Vec2 b1 = vertices[j];
            const Vec2 b2 = vertices[(j + 1) % count];
            if (segments_cross_internally(a1, a2, b1, b2))
            {
                return true;
            }
            if (point_on_segment(b1, a1, a2, kEpsilonLength) ||
                point_on_segment(b2, a1, a2, kEpsilonLength))
            {
                return true;
            }
        }
    }
    return false;
}

/// Greedy, deterministic ear clipping of a validated simple counter-clockwise ring.
class EarClipper
{
public:
    explicit EarClipper(const std::vector<Vec2> &vertices)
        : m_vertices(vertices), m_ring(vertices.size())
    {
        for (std::size_t i = 0; i < m_ring.size(); ++i)
        {
            m_ring[i] = i;
        }
    }

    /// Clips every ear; false when the ring leaves a degenerate remainder.
    [[nodiscard]] bool run(std::vector<Polygon> &triangles)
    {
        while (m_ring.size() > 3)
        {
            const auto ear = first_ear();
            if (!ear.has_value())
            {
                return false;
            }
            emit_ear(*ear, triangles);
        }
        const Vec2 a = m_vertices[m_ring[0]];
        const Vec2 b = m_vertices[m_ring[1]];
        const Vec2 c = m_vertices[m_ring[2]];
        if (signed_double_area(a, b, c) <= kMinNonDegenerateDoubleArea)
        {
            return false;
        }
        triangles.push_back(Polygon{{a, b, c}});
        return true;
    }

private:
    [[nodiscard]] std::size_t previous_of(std::size_t position) const noexcept
    {
        return (position + m_ring.size() - 1) % m_ring.size();
    }

    [[nodiscard]] std::size_t next_of(std::size_t position) const noexcept
    {
        return (position + 1) % m_ring.size();
    }

    /// First ring position, scanning from zero, that forms an ear safe to clip.
    [[nodiscard]] std::optional<std::size_t> first_ear() const
    {
        for (std::size_t position = 0; position < m_ring.size(); ++position)
        {
            if (is_ear(position))
            {
                return position;
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] bool is_ear(std::size_t position) const
    {
        const Vec2 a = m_vertices[m_ring[previous_of(position)]];
        const Vec2 b = m_vertices[m_ring[position]];
        const Vec2 c = m_vertices[m_ring[next_of(position)]];
        if (signed_double_area(a, b, c) <= kMinNonDegenerateDoubleArea)
        {
            return false;
        }
        return !contains_other_vertex(a, b, c, position);
    }

    /// True when a remaining vertex that is not an ear corner lies strictly inside `abc`.
    [[nodiscard]] bool contains_other_vertex(Vec2 a, Vec2 b, Vec2 c, std::size_t position) const
    {
        const std::size_t previous = previous_of(position);
        const std::size_t following = next_of(position);
        for (std::size_t candidate = 0; candidate < m_ring.size(); ++candidate)
        {
            if (candidate == position || candidate == previous || candidate == following)
            {
                continue;
            }
            if (strictly_inside_triangle(m_vertices[m_ring[candidate]], a, b, c))
            {
                return true;
            }
        }
        return false;
    }

    void emit_ear(std::size_t position, std::vector<Polygon> &triangles)
    {
        triangles.push_back(
            Polygon{{m_vertices[m_ring[previous_of(position)]], m_vertices[m_ring[position]],
                     m_vertices[m_ring[next_of(position)]]}});
        m_ring.erase(m_ring.begin() + static_cast<std::ptrdiff_t>(position));
    }

    const std::vector<Vec2> &m_vertices;
    std::vector<std::size_t> m_ring;
};

} // namespace

Result<std::vector<Polygon>> triangulate_outline(const Polygon &polygon)
{
    const std::vector<Vec2> &vertices = polygon.vertices;

    for (const Vec2 &vertex : vertices)
    {
        if (!is_finite(vertex))
        {
            return std::unexpected(
                make_error(ErrorCode::InvalidArgument, "polygon has a non-finite coordinate"));
        }
    }
    if (vertices.size() < 3)
    {
        return std::unexpected(
            make_error(ErrorCode::InvalidMesh, "polygon needs at least three vertices"));
    }
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        if (vertices[i] == vertices[(i + 1) % vertices.size()])
        {
            return std::unexpected(
                make_error(ErrorCode::InvalidMesh, "polygon has consecutive duplicate vertices"));
        }
    }
    if (has_self_intersection(vertices))
    {
        return std::unexpected(make_error(ErrorCode::InvalidMesh, "polygon is not simple"));
    }
    if (ring_double_area(vertices) <= kMinNonDegenerateDoubleArea)
    {
        return std::unexpected(
            make_error(ErrorCode::InvalidMesh, "polygon is clockwise or has degenerate area"));
    }

    std::vector<Polygon> triangles;
    EarClipper clipper(vertices);
    if (!clipper.run(triangles))
    {
        return std::unexpected(
            make_error(ErrorCode::InvalidMesh, "polygon could not be triangulated safely"));
    }
    return triangles;
}

} // namespace vwmini::detail

namespace vwmini
{

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon &polygon)
{
    return detail::triangulate_outline(polygon);
}

} // namespace vwmini
