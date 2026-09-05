#include "testing.hpp"

int run_geometry_tests();
int run_triangulator_tests();
int run_navmesh_tests();
int run_path_tests();
int run_simulation_tests();

int main()
{
    struct Entry {
        const char* name;
        int (*fn)();
    };
    const Entry entries[] = {
        {"geometry", &run_geometry_tests},
        {"triangulator", &run_triangulator_tests},
        {"navmesh", &run_navmesh_tests},
        {"paths", &run_path_tests},
        {"simulation", &run_simulation_tests},
    };

    int total_failures = 0;
    for (const auto& e : entries) {
        ::vwmtest::reset_counters();
        const int failures = e.fn();
        total_failures += failures;
        std::printf("[%-12s] failures=%d\n", e.name, failures);
    }
    std::printf("TOTAL FAILURES=%d\n", total_failures);
    return total_failures == 0 ? 0 : 1;
}
