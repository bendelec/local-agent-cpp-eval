#pragma once

// Minimal deterministic test harness: no external dependencies. Each TEST_CASE is
// self-registered; main() runs every registered case and reports failures.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace vwmini::test {

struct TestCase {
    const char* name;
    void (*fn)();
};

class Registry {
public:
    static Registry& instance()
    {
        static Registry registry;
        return registry;
    }

    void add(const char* name, void (*fn)())
    {
        cases_.push_back(TestCase{name, fn});
    }

    [[nodiscard]] std::size_t case_count() const { return cases_.size(); }

    int run()
    {
        int failed_cases = 0;
        for (const TestCase& tc : cases_) {
            failures_ = 0;
            current_ = tc.name;
            tc.fn();
            if (failures_ > 0) {
                ++failed_cases;
                std::printf("[FAIL] %s (%d assertion(s) failed)\n", tc.name, failures_);
            } else {
                std::printf("[ OK ] %s\n", tc.name);
            }
        }
        std::printf("%zu test case(s), %d failed\n", cases_.size(), failed_cases);
        return failed_cases == 0 ? 0 : 1;
    }

    void fail(const char* file, int line, const std::string& message)
    {
        ++failures_;
        std::printf("  %s:%d [%s] %s\n", file, line, current_, message.c_str());
    }

private:
    std::vector<TestCase> cases_;
    int failures_ = 0;
    const char* current_ = "";
};

inline void fail_impl(const char* file, int line, const std::string& message)
{
    Registry::instance().fail(file, line, message);
}

template <class A, class B>
void check_eq(const char* file, int line, const char* ea, const char* eb, const A& a, const B& b)
{
    if (!(a == b)) {
        std::string message = std::string("CHECK_EQ(") + ea + ", " + eb + ") failed";
        fail_impl(file, line, message);
    }
}

} // namespace vwmini::test

#define TEST_CASE(name)                                                                    \
    static void name();                                                                    \
    static bool vwmini_test_reg_##name = (vwmini::test::Registry::instance().add(#name, &name), true); \
    static void name()

#define CHECK(cond)                                                                        \
    do {                                                                                   \
        if (!(cond)) {                                                                     \
            vwmini::test::fail_impl(__FILE__, __LINE__, "CHECK(" #cond ") failed");        \
        }                                                                                  \
    } while (0)

#define CHECK_EQ(a, b) vwmini::test::check_eq(__FILE__, __LINE__, #a, #b, (a), (b))

#define CHECK_NEAR(a, b, tol)                                                              \
    do {                                                                                   \
        const double va_ = static_cast<double>(a);                                         \
        const double vb_ = static_cast<double>(b);                                         \
        if (std::abs(va_ - vb_) > static_cast<double>(tol)) {                              \
            vwmini::test::fail_impl(__FILE__, __LINE__,                                    \
                                    "CHECK_NEAR(" #a ", " #b ") failed");                  \
        }                                                                                  \
    } while (0)

#define CHECK_ERROR(result, expected_code)                                                \
    do {                                                                                   \
        const auto& res_ = (result);                                                       \
        if (res_.has_value()) {                                                            \
            vwmini::test::fail_impl(__FILE__, __LINE__,                                    \
                                    "expected error " #expected_code " but call succeeded"); \
        } else if (res_.error().code != (expected_code)) {                                 \
            vwmini::test::fail_impl(__FILE__, __LINE__, "unexpected error code");          \
        }                                                                                  \
    } while (0)
