#include <vwmini/nav_mesh.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

using namespace vwmini;

namespace {

int failures = 0;

#define CHECK(...)                                                                                                     \
    do {                                                                                                               \
        if (!(__VA_ARGS__)) {                                                                                          \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__);                                \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

Polygon tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
}

// Unit square split into two triangles sharing the diagonal (0,0)-(1,1).
std::vector<Polygon> squareMesh()
{
    return {tri({0, 0}, {1, 0}, {1, 1}), tri({0, 0}, {1, 1}, {0, 1})};
}

// L-shaped region (a simple polygon triangulated by hand into 4 triangles).
std::vector<Polygon> lMesh()
{
    // vertices: (0,0),(3,0),(3,1),(1,1),(1,3),(0,3)
    // triangulation into 4 CCW triangles.
    return {tri({0, 0}, {3, 0}, {3, 1}), tri({0, 0}, {3, 1}, {1, 1}), tri({0, 0}, {1, 1}, {1, 3}),
            tri({0, 0}, {1, 3}, {0, 3})};
}

} // namespace

static void test_create_valid()
{
    auto r = NavMesh::create(squareMesh());
    CHECK(r.has_value());
    if (r) {
        CHECK(r->cell_count() == 2);
        CHECK(r->contains({0.5f, 0.5f}));
        CHECK(r->contains({0.5f, 0.2f}));
    }
}

static void test_create_invalid()
{
    // Empty list -> InvalidMesh.
    CHECK(NavMesh::create({}).error().code == ErrorCode::InvalidMesh);
    // Non-triangle -> InvalidMesh.
    CHECK(NavMesh::create({Polygon{{Vec2{0, 0}, Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}}}}).error().code ==
          ErrorCode::InvalidMesh);
    // Non-finite vertex -> InvalidArgument.
    CHECK(NavMesh::create({tri({0, 0}, {std::nanf(""), 0}, {1, 1})}).error().code == ErrorCode::InvalidArgument);
    // Clockwise triangle -> InvalidMesh.
    CHECK(NavMesh::create({tri({0, 0}, {0, 1}, {1, 0})}).error().code == ErrorCode::InvalidMesh);
    // Degenerate triangle -> InvalidMesh.
    CHECK(NavMesh::create({tri({0, 0}, {1, 0}, {2, 0})}).error().code == ErrorCode::InvalidMesh);
    // Overlapping interiors -> InvalidMesh.
    CHECK(NavMesh::create({tri({0, 0}, {2, 0}, {0, 2}), tri({0, 0}, {1, 0}, {0, 1})}).error().code ==
          ErrorCode::InvalidMesh);
    // Non-manifold edge (three triangles sharing one edge).
    CHECK(NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}), tri({0, 0}, {1, 0}, {1, -1}), tri({0, 0}, {1, 0}, {0, -1})})
              .error()
              .code == ErrorCode::InvalidMesh);
    // T-junction: a vertex lying in the middle of another triangle's edge.
    // Triangle A = (0,0),(2,0),(0,2). Triangle B has a vertex at (1,0) on A's edge.
    CHECK(NavMesh::create({tri({0, 0}, {2, 0}, {0, 2}), tri({1, 0}, {2, 0}, {1, 1})}).error().code ==
          ErrorCode::InvalidMesh);
}

static void test_contains_boundary()
{
    auto r = NavMesh::create(squareMesh());
    CHECK(r.has_value());
    if (r) {
        const NavMesh& m = *r;
        // Non-finite -> false.
        CHECK(m.contains({std::nanf(""), 0}) == false);
        // Strictly inside -> true.
        CHECK(m.contains({0.25f, 0.25f}));
        // On a triangle edge (distance 0) -> true.
        CHECK(m.contains({0.0f, 0.5f}));
        CHECK(m.contains({1.0f, 0.5f}));
        // Just outside a boundary edge within epsilon -> true.
        CHECK(m.contains({0.0f, -0.5e-4f}));
        // Clearly outside -> false.
        CHECK(m.contains({2.0f, 2.0f}) == false);
    }
}

static void test_find_path_direct()
{
    auto r = NavMesh::create(squareMesh());
    CHECK(r.has_value());
    if (r) {
        const NavMesh& m = *r;
        // Non-finite -> InvalidArgument.
        CHECK(find_path(m, {std::nanf(""), 0}, {0.5f, 0.5f}).error().code == ErrorCode::InvalidArgument);
        // Start outside -> OutsideMesh.
        CHECK(find_path(m, {2.0f, 2.0f}, {0.5f, 0.5f}).error().code == ErrorCode::OutsideMesh);
        // Goal outside -> OutsideMesh.
        CHECK(find_path(m, {0.5f, 0.5f}, {2.0f, 2.0f}).error().code == ErrorCode::OutsideMesh);
        // Equal endpoints -> [start].
        auto e = find_path(m, {0.5f, 0.5f}, {0.5f, 0.5f});
        CHECK(e.has_value());
        if (e) {
            CHECK(e->points.size() == 1);
            CHECK(e->points[0] == Vec2{0.5f, 0.5f});
        }
        // Direct segment fully inside -> [start, goal].
        auto d = find_path(m, {0.2f, 0.2f}, {0.8f, 0.8f});
        CHECK(d.has_value());
        if (d) {
            CHECK(d->points.size() == 2);
            CHECK(d->points[0] == Vec2{0.2f, 0.2f});
            CHECK(d->points[1] == Vec2{0.8f, 0.8f});
        }
    }
}

static void test_find_path_disconnected()
{
    // Two disjoint triangles (touching only at a vertex, not an edge).
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}), tri({2, 2}, {3, 2}, {2, 3})});
    CHECK(r.has_value());
    if (r) {
        const NavMesh& m = *r;
        CHECK(find_path(m, {0.2f, 0.2f}, {2.2f, 2.2f}).error().code == ErrorCode::NoPath);
    }
}

static void test_find_path_direct_boundary_tolerance()
{
    // Issue 1: a direct segment that crosses a boundary edge within the tolerance
    // zone (endpoints within epsilon of the edge) is contained per MSH-006, so
    // find_path must return exactly the two supplied endpoints, not a corridor
    // fallback.
    auto r = NavMesh::create(squareMesh());
    CHECK(r.has_value());
    if (r) {
        const NavMesh& m = *r;
        // Vertical segment along x=0.5 crossing the bottom edge (0,0)-(1,0);
        // the goal (0.5,-0.5e-4) is within epsilon of that edge.
        auto p = find_path(m, {0.5f, 0.5f}, {0.5f, -0.5e-4f});
        CHECK(p.has_value());
        if (p) {
            CHECK(p->points.size() == 2);
            CHECK(p->points[0] == Vec2{0.5f, 0.5f});
            CHECK(p->points[1] == Vec2{0.5f, -0.5e-4f});
        }
    }
}

static void test_find_path_segments_contained()
{
    // Issue 2: every segment of a route through the L-shape must be contained in
    // the mesh (dense sampling), including the specific (2.9,0.9)->(0.9,2.9)
    // diagonal that used to cut through the non-walkable notch.
    auto r = NavMesh::create(lMesh());
    CHECK(r.has_value());
    if (r) {
        const NavMesh& m = *r;
        auto p = find_path(m, {2.9f, 0.9f}, {0.9f, 2.9f});
        CHECK(p.has_value());
        if (p) {
            const std::vector<Vec2>& pts = p->points;
            CHECK(pts.front() == Vec2{2.9f, 0.9f});
            CHECK(pts.back() == Vec2{0.9f, 2.9f});
            for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                const Vec2 a = pts[i];
                const Vec2 b = pts[i + 1];
                for (int k = 0; k <= 256; ++k) {
                    const Vec2 q = a + (b - a) * (static_cast<float>(k) / 256.0f);
                    CHECK(m.contains(q));
                }
            }
        }
    }
}

static void test_create_identical_triangles()
{
    // Issue 5: two geometrically identical CCW triangles with fully overlapping
    // interiors must be rejected as InvalidMesh.
    CHECK(NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}), tri({0, 0}, {1, 0}, {0, 1})}).error().code ==
          ErrorCode::InvalidMesh);
    // A triangle fully contained inside another is also overlapping.
    CHECK(NavMesh::create({tri({0, 0}, {4, 0}, {0, 4}), tri({0, 0}, {1, 0}, {0, 1})}).error().code ==
          ErrorCode::InvalidMesh);
}

static void test_find_path_l_bend()
{
    auto r = NavMesh::create(lMesh());
    CHECK(r.has_value());
    if (r) {
        const NavMesh& m = *r;
        // Start in the bottom leg, goal in the top leg; direct segment exits the notch.
        auto p = find_path(m, {2.5f, 0.5f}, {0.5f, 2.5f});
        CHECK(p.has_value());
        if (p) {
            const std::vector<Vec2>& pts = p->points;
            CHECK(pts.size() >= 3);
            CHECK(pts.front() == Vec2{2.5f, 0.5f});
            CHECK(pts.back() == Vec2{0.5f, 2.5f});
            // Every point is contained by the mesh.
            for (const Vec2& q : pts) {
                CHECK(m.contains(q));
            }
            // Length close to the shortest path around the inner corner (~3.162).
            float len = 0.0f;
            for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                len += length(pts[i + 1] - pts[i]);
            }
            CHECK(std::fabs(len - 3.162f) < 0.15f);
        }
        // Determinism.
        auto p2 = find_path(m, {2.5f, 0.5f}, {0.5f, 2.5f});
        CHECK(p2.has_value());
        if (p2) {
            CHECK(p2->points.size() == p->points.size());
            for (std::size_t i = 0; i < p->points.size(); ++i) {
                CHECK(p2->points[i] == p->points[i]);
            }
        }
    }
}

int main()
{
    test_create_valid();
    test_create_invalid();
    test_contains_boundary();
    test_find_path_direct();
    test_find_path_direct_boundary_tolerance();
    test_find_path_segments_contained();
    test_find_path_disconnected();
    test_find_path_l_bend();
    test_create_identical_triangles();
    if (failures == 0) {
        std::printf("nav_mesh_test: OK\n");
        return 0;
    }
    std::printf("nav_mesh_test: %d failures\n", failures);
    return 1;
}
