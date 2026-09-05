// Geometry primitive tests.
#include "testing.hpp"

#include <vwmini/geometry.hpp>

using namespace vwmini;

namespace {
Vec2 vsum(const Vec2& p, const Vec2& q) { return operator+(p, q); }
Vec2 vdiff(const Vec2& p, const Vec2& q) { return operator-(p, q); }
} // namespace

int run_geometry_tests()
{
    const Vec2 a{1.0f, 2.0f};
    const Vec2 b{1.0f, 2.0f + 1e-5f}; // large enough to differ from 2.0f in float
    VWM_REQUIRE((Vec2{1, 2} == Vec2{1, 2}));
    VWM_REQUIRE(!(a == b));

    const Vec2 p1{1.0f, 2.0f};
    const Vec2 p2{3.0f, 4.0f};
    VWM_CHECK_EQ(vsum(p1, p2), (Vec2{4, 6}));
    VWM_CHECK_EQ(vdiff(Vec2{5, 7}, Vec2{2, 3}), (Vec2{3, 4}));
    VWM_CHECK_EQ(p2 * 2.0f, (Vec2{6, 8}));
    VWM_CHECK_EQ(2.0f * p2, (Vec2{6, 8}));

    const float d = dot(p1, p2);
    VWM_CHECK_EQ(d, 11.0f);
    const float c = cross(Vec2{1, 0}, Vec2{0, 1});
    VWM_CHECK_EQ(c, 1.0f);
    VWM_CHECK_FLOAT(length(Vec2{3, 4}), 5.0f);
    VWM_CHECK_EQ(normalized(Vec2{0, 0}), (Vec2{0, 0}));
    VWM_CHECK_FLOAT(length(normalized(Vec2{3, 4})), 1.0f);

    return ::vwmtest::counters().failed;
}
