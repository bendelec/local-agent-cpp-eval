#include <vwmini/geometry.hpp>

#include <vwmini/detail.hpp>

namespace vwmini {

float length(Vec2 value) noexcept
{
    return std::sqrt(dot(value, value));
}

Vec2 normalized(Vec2 value) noexcept
{
    const float len = length(value);
    if (len == 0.0f || !std::isfinite(len)) {
        return Vec2{0.0f, 0.0f};
    }
    return value * (1.0f / len);
}

} // namespace vwmini
