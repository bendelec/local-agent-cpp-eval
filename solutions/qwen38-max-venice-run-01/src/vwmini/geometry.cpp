#include <vwmini/geometry.hpp>

#include <cmath>

namespace vwmini {

float length(Vec2 value) noexcept
{
    // hypot avoids the overflow (x*x -> inf) and underflow (x*x -> 0) of the naive
    // sum of squares, honouring the contract that finite input yields finite output.
    return std::hypot(value.x, value.y);
}

Vec2 normalized(Vec2 value) noexcept
{
    const float len = length(value);
    if (len == 0.0f) {
        return {0.0f, 0.0f};
    }
    return {value.x / len, value.y / len};
}

} // namespace vwmini
