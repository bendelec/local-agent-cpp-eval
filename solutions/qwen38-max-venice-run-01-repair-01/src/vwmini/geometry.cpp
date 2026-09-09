#include <vwmini/geometry.hpp>

#include <cmath>
#include <limits>

namespace vwmini {

float length(Vec2 value) noexcept
{
    // hypot in double avoids both the intermediate overflow (float x*x -> inf)
    // and the underflow of the naive sum of squares. Finite inputs whose true
    // magnitude exceeds the float range (e.g. {FLT_MAX, FLT_MAX}, whose length
    // is ~4.8e38) saturate at FLT_MAX: the contract promises finite output for
    // finite input, so returning inf would break it.
    const double len = std::hypot(static_cast<double>(value.x), static_cast<double>(value.y));
    constexpr double kFloatMax = static_cast<double>(std::numeric_limits<float>::max());
    if (!(len <= kFloatMax)) { // Inf, NaN, or beyond the float range.
        return std::numeric_limits<float>::max();
    }
    return static_cast<float>(len);
}

Vec2 normalized(Vec2 value) noexcept
{
    // Divide in double: every float magnitude is representable in double, so
    // the division never overflows, and each result component has magnitude
    // <= 1, so the cast back to float never overflows either. This keeps the
    // unit-vector contract for finite inputs whose float length would overflow
    // (e.g. {FLT_MAX, FLT_MAX}); the float-only computation used to divide by
    // inf and collapse such vectors to {0, 0}.
    const double x = static_cast<double>(value.x);
    const double y = static_cast<double>(value.y);
    const double len = std::hypot(x, y);
    if (!(len > 0.0)) { // Zero gives the header-documented {0, 0}; NaN takes the
                        // same safe path (non-finite inputs are outside the contract).
        return {0.0f, 0.0f};
    }
    return {static_cast<float>(x / len), static_cast<float>(y / len)};
}

} // namespace vwmini
