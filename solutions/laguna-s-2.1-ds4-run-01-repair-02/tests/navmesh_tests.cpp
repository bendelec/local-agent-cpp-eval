// Navigation mesh construction & containment tests (MSH-004 / MSH-005 / MSH-006).
#include "testing.hpp"

#include <vwmini/nav_mesh.hpp>

using namespace vwmini;

namespace {
inline constexpr float kEps = 1e-4f;

Polygon Tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
}
} // namespace

int run_navmesh_tests()
{
    // --- Single valid triangle ------------------------------------------------
    {
        std::vector<Polygon> polys{Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1})};
        auto res = NavMesh::create(polys);
        VWM_REQUIRE(res.has_value());
        VWM_CHECK_EQ(static_cast<int>(res->cell_count()), 1);
        VWM_REQUIRE(res->contains(Vec2{0.25f, 0.25f}));                 // interior
        VWM_REQUIRE(res->contains(Vec2{kEps * 0.5f, 0.0f}));            // within boundary tolerance
        VWM_REQUIRE(!res->contains(Vec2{2.0f, 2.0f}));                  // far outside
        VWM_REQUIRE(!res->contains(Vec2{-kEps * 2, 0.0f}));             // beyond tolerance
        VWM_REQUIRE(!res->contains(Vec2{INFINITY, 0.0f}));              // non-finite -> false
    }

    // --- Two triangles sharing a complete edge are adjacent & accepted --------
    {
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),   // edge (0,0)->(1,0), apex above
            Tri(Vec2{1, 0}, Vec2{0, 0}, Vec2{1, -1}),  // same edge reversed -> CCW, apex below
        };
        auto res = NavMesh::create(polys);
        VWM_REQUIRE(res.has_value());
        VWM_CHECK_EQ(static_cast<int>(res->cell_count()), 2);
    }

    // --- Disjoint components allowed -----------------------------------------
    {
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),
            Tri(Vec2{5, 5}, Vec2{6, 5}, Vec2{5, 6}),
        };
        auto res = NavMesh::create(polys);
        VWM_REQUIRE(res.has_value());
        VWM_CHECK_EQ(static_cast<int>(res->cell_count()), 2);
    }

    // --- Vertex-only touch is allowed (no adjacency created) ------------------
    {
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),
            Tri(Vec2{0, 0}, Vec2{-1, 0}, Vec2{0, -1}),
        };
        auto res = NavMesh::create(polys);
        VWM_REQUIRE(res.has_value());
        VWM_CHECK_EQ(static_cast<int>(res->cell_count()), 2);
    }

    // --- Empty list -> InvalidMesh -------------------------------------------
    {
        auto res = NavMesh::create({});
        VWM_REQUIRE(!res.has_value());
        VWM_CHECK_EQ(res.error().code, ErrorCode::InvalidMesh);
    }

    // --- Non-triangle polygon -> InvalidMesh ----------------------------------
    {
        std::vector<Polygon> polys{Polygon{{Vec2{0, 0}, Vec2{1, 0}}}};
        auto res = NavMesh::create(polys);
        VWM_REQUIRE(!res.has_value());
        VWM_CHECK_EQ(res.error().code, ErrorCode::InvalidMesh);
    }

    // --- Non-finite vertex -> InvalidArgument -------------------------------
    {
        std::vector<Polygon> polys{Tri(Vec2{0, 0}, Vec2{NAN, 0}, Vec2{0, 1})};
        auto res = NavMesh::create(polys);
        VWM_REQUIRE(!res.has_value());
        VWM_CHECK_EQ(res.error().code, ErrorCode::InvalidArgument);
    }

    // --- Clockwise / degenerate triangle -> InvalidMesh ----------------------
    {
        std::vector<Polygon> cw{Tri(Vec2{0, 0}, Vec2{0, 1}, Vec2{1, 0})}; // CW for y-up
        VWM_CHECK_EQ(NavMesh::create(cw).error().code, ErrorCode::InvalidMesh);
    }
    {
        std::vector<Polygon> flat{Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{2, 0})}; // collinear
        VWM_CHECK_EQ(NavMesh::create(flat).error().code, ErrorCode::InvalidMesh);
    }

    // --- Overlapping interiors -> InvalidMesh ---------------------------------
    {
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{4, 0}, Vec2{0, 4}),      // large
            Tri(Vec2{1, 1}, Vec2{2, 1}, Vec2{1, 2}),      // fully inside the large one
        };
        VWM_CHECK_EQ(NavMesh::create(polys).error().code, ErrorCode::InvalidMesh);
    }

    // --- T-junction: a vertex lies on another's edge interior ---------------
    {
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{2, 0}, Vec2{0, 2}),      // hypotenuse passes through (1,1)
            Tri(Vec2{1, 1}, Vec2{3, 1}, Vec2{1, 3}),      // vertex (1,1) on that edge interior
        };
        VWM_CHECK_EQ(NavMesh::create(polys).error().code, ErrorCode::InvalidMesh);
    }

    // --- Non-manifold edge (>2 triangles share an edge, two on same side) ----
    {
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{2, 0}, Vec2{1, 1}),   // above shared base edge (0,0)->(2,0)
            Tri(Vec2{2, 0}, Vec2{0, 0}, Vec2{1, -1}),  // below, reversed edge -> valid adjacency
            Tri(Vec2{2, 0}, Vec2{0, 0}, Vec2{1, 2}),   // also above -> overlaps previous
        };
        VWM_CHECK_EQ(NavMesh::create(polys).error().code, ErrorCode::InvalidMesh);
    }

    return ::vwmtest::counters().failed;
}
