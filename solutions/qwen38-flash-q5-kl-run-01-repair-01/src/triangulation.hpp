#pragma once

#include <vwmini/geometry.hpp>

#include <vector>

namespace vwmini::detail
{

/**
 * Validates and triangulates one simple, counter-clockwise, hole-free outline (MSH-002).
 *
 * \return CCW triangles covering the outline, or an error: `InvalidArgument` for
 *         non-finite coordinates, `InvalidMesh` for any other invalid outline (MSH-002).
 */
[[nodiscard]] Result<std::vector<Polygon>> triangulate_outline(const Polygon &polygon);

} // namespace vwmini::detail
