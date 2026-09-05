// Minimal deterministic test helpers for VWmini. No external dependencies.
// Each focused test file exposes `int run_<name>_tests()` called explicitly in order
// from tests/main.cpp, so execution order never depends on static init.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace vwmtest {

struct Counters {
    int passed = 0;
    int failed = 0;
};

inline Counters& counters()
{
    static Counters c{};
    return c;
}

inline void reset_counters() noexcept { counters() = Counters{}; }

inline bool approx(float a, float b, float eps = 1e-4f) noexcept
{
    const float diff = std::fabs(a - b);
    if (diff <= eps) {
        return true;
    }
    const float ma = std::fabs(a);
    const float mb = std::fabs(b);
    const float largest = ma > mb ? ma : mb;
    return diff <= eps * largest;
}

namespace detail {
inline void check_fail(const char* file, int line, const char* expr) noexcept
{
    ++::vwmtest::counters().failed;
    std::fprintf(stderr, "FAIL %s:%d: CHECK(%s)\n", file, line, expr);
}
} // namespace detail

// Fatal precondition checks: abort the current test group on failure.
#define VWM_REQUIRE(cond)                                                        \
    do {                                                                         \
        if (!(cond)) {                                                           \
            ++::vwmtest::counters().failed;                                      \
            std::fprintf(stderr, "FAIL %s:%d: REQUIRE(%s)\n", __FILE__,          \
                         __LINE__, #cond);                                       \
            return ::vwmtest::counters().failed;                                 \
        }                                                                        \
        ++::vwmtest::counters().passed;                                          \
    } while (0)

// Non-fatal expectation checks: record failures without aborting the group.
#define VWM_CHECK(cond)                                                          \
    do {                                                                         \
        if (!(cond)) {                                                           \
            ::vwmtest::detail::check_fail(__FILE__, __LINE__, #cond);            \
        } else {                                                                 \
            ++::vwmtest::counters().passed;                                      \
        }                                                                        \
    } while (0)

#define VWM_CHECK_EQ(actual, expected) VWM_CHECK(((actual) == (expected)))
#define VWM_CHECK_NE(a, b) VWM_CHECK(!((a) == (b)))
#define VWM_CHECK_FLOAT(a, b) VWM_CHECK(::vwmtest::approx((a), (b)))
#define VWM_CHECK_GE(a, b) VWM_CHECK(((a) >= (b)))
#define VWM_CHECK_LE(a, b) VWM_CHECK(((a) <= (b)))
#define VWM_FAIL(msg)                                                            \
    do {                                                                         \
        ++::vwmtest::counters().failed;                                          \
        std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);       \
    } while (0)

} // namespace vwmtest
