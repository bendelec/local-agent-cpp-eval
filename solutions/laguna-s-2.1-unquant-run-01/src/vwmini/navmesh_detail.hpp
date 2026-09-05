#pragma once

// Private internal definition of NavMesh::Impl. Included only by .cpp files in
// this translation unit. The definition is intentionally out-of-line so that
// find_path (a friend) and NavMesh::create can share the layout without
// exposing it in the public header.

#include <array>
#include <cstddef>
#include <vector>

#include <vwmini/geometry.hpp>

namespace vwmini {

struct NavMesh::Impl {
    std::vector<std::array<Vec2, 3>> triangles;
    // neighbours[i][e] = index of triangle adjacent across edge e of triangle i,
    // or SIZE_MAX when that edge is a boundary edge.
    std::vector<std::array<std::size_t, 3>> neighbours;

    static constexpr std::size_t no_cell = static_cast<std::size_t>(-1);

    [[nodiscard]] std::size_t cell_count() const noexcept { return triangles.size(); }

    // Centroid of triangle index `c`.
    [[nodiscard]] static Vec2 centroid(const Impl& m, std::size_t c) noexcept
    {
        const auto& t = m.triangles[c];
        return Vec2{
            (t[0].x + t[1].x + t[2].x) / 3.0f,
            (t[0].y + t[1].y + t[2].y) / 3.0f,
        };
    }

    // The shared edge between adjacent triangles `a` and `b`, returned in triangle
    // `a`'s directed edge order. Precondition: a and b are adjacent.
    [[nodiscard]] static std::array<Vec2, 2> shared_edge(const Impl& m,
                                                         std::size_t a,
                                                         std::size_t b) noexcept
    {
        const auto& t = m.triangles[a];
        for (int e = 0; e < 3; ++e) {
            if (m.neighbours[a][e] == b) {
                return {t[e], t[(e + 1) % 3]};
            }
        }
        return {t[0], t[1]}; // unreachable for adjacent cells
    }

    // Constructed once via create(); stored as shared_ptr<const Impl>.
    Impl(std::vector<std::array<Vec2, 3>> tris,
         std::vector<std::array<std::size_t, 3>> neigh)
        : triangles(std::move(tris)), neighbours(std::move(neigh)) {}
};

} // namespace vwmini
