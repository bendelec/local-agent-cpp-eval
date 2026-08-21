#include <vwmini/geometry.hpp>

#include "vwmini/triangulate.hpp"

#include <cmath>

namespace vwmini {

float length(Vec2 value) noexcept
{
    return std::sqrt(dot(value, value));
}

Vec2 normalized(Vec2 value) noexcept
{
    const float len = length(value);
    if (len == 0.0f || !std::isfinite(len))
        return Vec2{};
    return value * (1.0f / len);
}

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon)
{
    return detail::triangulate_polygon(polygon);
}

} // namespace vwmini
