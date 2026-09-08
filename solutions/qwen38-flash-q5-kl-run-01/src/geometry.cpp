#include <vwmini/geometry.hpp>

#include <cmath>

namespace vwmini
{

float length(Vec2 value) noexcept
{
    // std::hypot avoids the intermediate overflow/underflow of sqrt(x*x + y*y).
    return std::hypot(value.x, value.y);
}

Vec2 normalized(Vec2 value) noexcept
{
    const float len = length(value);
    if (len == 0.0f)
    {
        return {0.0f, 0.0f};
    }
    const float inverse = 1.0f / len;
    return {value.x * inverse, value.y * inverse};
}

} // namespace vwmini
