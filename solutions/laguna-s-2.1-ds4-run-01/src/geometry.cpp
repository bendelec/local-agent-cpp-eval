// VWmini geometry implementation. Pure functions only.
#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace vwmini {

float length(Vec2 value) noexcept
{
    return std::sqrt(dot(value, value));
}

Vec2 normalized(Vec2 value) noexcept
{
    const float len = length(value);
    // Normalize-by-zero yields {0,0}; also guards against sub-normal/inf paths.
    if (len == 0.0f || !std::isfinite(len)) {
        return Vec2{};
    }
    return value * (1.0f / len);
}

} // namespace vwmini
