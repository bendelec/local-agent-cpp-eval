// Pathfinding tests (WP4): endpoint validation, direct visibility, corridor routing,
// determinism, disconnected components, and shared-edge adjacency handling.
#include "testing.hpp"

#include <vwmini/nav_mesh.hpp>

using namespace vwmini;

namespace {
inline constexpr float kEps = 1e-4f;

Polygon Tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
}

Result<NavMesh> make_two_adjacent_triangles()
{
    // Two CCW triangles sharing edge (0,0)-(1,0), forming a diamond-ish quad split.
    std::vector<Polygon> polys{
        Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),   // left triangle
        Tri(Vec2{1, 0}, Vec2{0, 0}, Vec2{1, -1}),  // right triangle (reversed edge)
    };
    return NavMesh::create(polys);
}
} // namespace

int run_path_tests()
{
    auto mesh_res = make_two_adjacent_triangles();
    VWM_REQUIRE(mesh_res.has_value());
    const NavMesh& mesh = *mesh_res;

    // --- Non-finite endpoints -> InvalidArgument --------------------------------
    {
        auto r = find_path(mesh, Vec2{NAN, 0}, Vec2{0.5f, 0.0f});
        VWM_CHECK_EQ(r.error().code, ErrorCode::InvalidArgument);
        auto r2 = find_path(mesh, Vec2{0.5f, 0.0f}, Vec2{0.5f, INFINITY});
        VWM_CHECK_EQ(r2.error().code, ErrorCode::InvalidArgument);
    }

    // --- Outside-mesh endpoints -> OutsideMesh ----------------------------------
    {
        auto r = find_path(mesh, Vec2{5, 5}, Vec2{0.5f, 0.0f});
        VWM_CHECK_EQ(r.error().code, ErrorCode::OutsideMesh);
        auto r2 = find_path(mesh, Vec2{0.5f, 0.0f}, Vec2{-5, -5});
        VWM_CHECK_EQ(r2.error().code, ErrorCode::OutsideMesh);
    }

    // --- Identical endpoints collapse to single-point path ----------------------
    {
        auto r = find_path(mesh, Vec2{0.25f, 0.25f}, Vec2{0.25f, 0.25f});
        VWM_REQUIRE(r.has_value());
        VWM_CHECK_EQ(static_cast<int>(r->points.size()), 1);
        const Vec2 p{0.25f, 0.25f};
        VWM_CHECK_EQ(r->points[0], p);
    }

    // --- Direct line-of-sight yields exactly [start, goal] ----------------------
    {
        const Vec2 start{0.25f, 0.25f};
        const Vec2 goal{0.75f, -0.25f};
        auto r = find_path(mesh, start, goal);
        VWM_REQUIRE(r.has_value());
        VWM_CHECK_EQ(static_cast<int>(r->points.size()), 2);
        VWM_CHECK_EQ(r->points.front(), start);
        VWM_CHECK_EQ(r->points.back(), goal);
    }

    // --- Corridor routing produces a valid contained polyline -------------------
    {
        // Two triangles sharing edge (0,0)-(1,0); their union is a convex rhombus,
        // so a route exists and every consecutive waypoint pair must stay inside.
        const Vec2 start{0.1f, 0.8f};
        const Vec2 goal{0.9f, -0.8f};
        auto r = find_path(mesh, start, goal);
        VWM_REQUIRE(r.has_value());
        VWM_CHECK_EQ(r->points.front(), start);
        VWM_CHECK_EQ(r->points.back(), goal);
        VWM_CHECK_GE(static_cast<int>(r->points.size()), 2);
        // Every consecutive pair of waypoints must be contained by the mesh.
        for (std::size_t i = 1; i < r->points.size(); ++i) {
            const Vec2 mid{(r->points[i - 1].x + r->points[i].x) / 2.0f,
                           (r->points[i - 1].y + r->points[i].y) / 2.0f};
            VWM_REQUIRE(mesh.contains(mid));
        }
    }

    // --- Disconnected components -> NoPath -------------------------------------
    {
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),
            Tri(Vec2{5, 5}, Vec2{6, 5}, Vec2{5, 6}),
        };
        auto m = NavMesh::create(polys);
        VWM_REQUIRE(m.has_value());
        auto r = find_path(*m, Vec2{0.25f, 0.25f}, Vec2{5.25f, 5.25f});
        VWM_CHECK_EQ(r.error().code, ErrorCode::NoPath);
    }

    // --- Determinism: identical input produces identical output ------------------
    {
        const Vec2 s{0.1f, 0.8f};
        const Vec2 g{0.9f, -0.8f};
        auto a = find_path(mesh, s, g);
        auto b = find_path(mesh, s, g);
        VWM_REQUIRE(a.has_value() && b.has_value());
        VWM_CHECK_EQ(a->points.size(), b->points.size());
        for (std::size_t i = 0; i < a->points.size(); ++i) {
            VWM_CHECK_EQ(a->points[i], b->points[i]);
        }
    }

    return ::vwmtest::counters().failed;
}
