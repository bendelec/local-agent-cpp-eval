// Ear-clipping triangulation of one simple CCW hole-free polygon (MSH-002/003).
#pragma once

#include <vwmini/geometry.hpp>

#include <cstddef>
#include <vector>

namespace vwmini::detail {

// Validates per MSH-002 and clips ears. Returns the triangles (each a
// 3-vertex CCW Polygon) in deterministic order.
[[nodiscard]] Result<std::vector<Polygon>> triangulate_polygon(const Polygon& polygon);

// Strict ear-clipping core; assumes `v` passed MSH-002 validation and
// `ring` holds a simple CCW subsequence of its indices.
[[nodiscard]] bool clip_ring(const std::vector<Vec2>& v, std::vector<std::size_t>& ring,
                             std::vector<Polygon>& out);

} // namespace vwmini::detail
