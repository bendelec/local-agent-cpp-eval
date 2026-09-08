#include <vwmini/geometry.hpp>

#include <cmath>
#include <limits>

namespace vwmini
{

float length(Vec2 value) noexcept
{
    // Computed in double and clamped, so a finite vector never yields infinity: the true norm
    // of (FLT_MAX, FLT_MAX) is not representable as a float, and the largest finite float is
    // the closest value that keeps the documented "finite input -> finite output" promise.
    const double x = static_cast<double>(value.x);
    const double y = static_cast<double>(value.y);
    const double magnitude = std::hypot(x, y);
    constexpr double kFloatLimit = static_cast<double>(std::numeric_limits<float>::max());
    if (std::isfinite(magnitude) && magnitude > kFloatLimit)
    {
        return std::numeric_limits<float>::max();
    }
    // A non-finite norm comes from non-finite input and is reported unchanged.
    return static_cast<float>(magnitude);
}

Vec2 normalized(Vec2 value) noexcept
{
    // Scaling by the largest component first keeps every intermediate inside the float range,
    // so a huge finite vector normalizes instead of collapsing to zero, and a denormal one does
    // not underflow. Non-finite or exactly-zero input yields the zero vector.
    const double x = static_cast<double>(value.x);
    const double y = static_cast<double>(value.y);
    const double scale = std::max(std::abs(x), std::abs(y));
    if (scale == 0.0 || !std::isfinite(scale))
    {
        return {0.0f, 0.0f};
    }
    const Vec2 unit{static_cast<float>(x / scale), static_cast<float>(y / scale)};
    const double magnitude = std::hypot(static_cast<double>(unit.x), static_cast<double>(unit.y));
    if (!std::isfinite(magnitude) || magnitude == 0.0)
    {
        return {0.0f, 0.0f};
    }
    const float inverse = static_cast<float>(1.0 / magnitude);
    return Vec2{unit.x * inverse, unit.y * inverse};
}

} // namespace vwmini
