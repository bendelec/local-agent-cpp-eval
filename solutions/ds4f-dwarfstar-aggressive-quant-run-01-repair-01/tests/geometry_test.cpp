#include <vwmini/geometry.hpp>

using namespace vwmini;

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

#define CHECK(...)                                                                                                     \
    do {                                                                                                               \
        if (!(__VA_ARGS__)) {                                                                                          \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__);                                \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

} // namespace

static void test_vec2()
{
    CHECK((Vec2{1, 2} + Vec2{3, 4}) == Vec2{4, 6});
    CHECK((Vec2{1, 2} - Vec2{3, 4}) == Vec2{-2, -2});
    CHECK((Vec2{1, 2} * 3.0f) == Vec2{3, 6});
    CHECK((3.0f * Vec2{1, 2}) == Vec2{3, 6});
    CHECK(dot(Vec2{1, 2}, Vec2{3, 4}) == 11.0f);
    CHECK(cross(Vec2{1, 0}, Vec2{0, 1}) == 1.0f);
    CHECK(length(Vec2{3, 4}) == 5.0f);
    CHECK(normalized(Vec2{3, 4}) == Vec2{0.6f, 0.8f});
    CHECK(normalized(Vec2{0, 0}) == Vec2{0, 0});
    CHECK((Vec2{1, 1} == Vec2{1, 1}) == true);
    CHECK((Vec2{1, 1} == Vec2{1, 2}) == false);
}

static void test_triangulate_square()
{
    // CCW unit square.
    Polygon p{{Vec2{0, 0}, Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}}};
    auto r = triangulate_simple_polygon(p);
    CHECK(r.has_value());
    if (r) {
        CHECK(r->size() == 2);
        float area = 0.0f;
        for (const auto& t : *r) {
            CHECK(t.vertices.size() == 3);
            area += 0.5f * std::abs(cross(t.vertices[1] - t.vertices[0], t.vertices[2] - t.vertices[0]));
        }
        CHECK(std::fabs(area - 1.0f) < 1e-3f);
    }
}

static void test_triangulate_invalid()
{
    // Fewer than three vertices.
    CHECK(triangulate_simple_polygon(Polygon{{Vec2{0, 0}}}).error().code == ErrorCode::InvalidMesh);
    // Consecutive duplicate.
    Polygon dup{{Vec2{0, 0}, Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}}};
    CHECK(triangulate_simple_polygon(dup).error().code == ErrorCode::InvalidMesh);
    // Clockwise winding (negative area).
    Polygon cw{{Vec2{0, 0}, Vec2{0, 1}, Vec2{1, 0}}};
    CHECK(triangulate_simple_polygon(cw).error().code == ErrorCode::InvalidMesh);
    // Non-finite coordinate.
    Polygon nf{{Vec2{0, 0}, Vec2{1, 0}, Vec2{std::nanf(""), 1}}};
    CHECK(triangulate_simple_polygon(nf).error().code == ErrorCode::InvalidArgument);
    // Self-intersecting bow-tie.
    Polygon bow{{Vec2{0, 0}, Vec2{1, 1}, Vec2{1, 0}, Vec2{0, 1}}};
    CHECK(triangulate_simple_polygon(bow).error().code == ErrorCode::InvalidMesh);
}

static void test_triangulate_self_touch()
{
    // Issue 6a: a non-consecutive vertex lying on a non-adjacent edge is a
    // self-touching outline, not a simple polygon.
    Polygon selfTouch{{Vec2{0, 0}, Vec2{2, 0}, Vec2{1, 0}, Vec2{0, 2}}};
    CHECK(triangulate_simple_polygon(selfTouch).error().code == ErrorCode::InvalidMesh);
    // A repeated non-consecutive vertex (pinch) is also a self-touch.
    Polygon pinch{{Vec2{0, 0}, Vec2{2, 0}, Vec2{2, 2}, Vec2{0, 0}, Vec2{0, 2}}};
    CHECK(triangulate_simple_polygon(pinch).error().code == ErrorCode::InvalidMesh);
}

static void test_triangulate_collinear_overlap()
{
    // Issue 6b: overlapping non-adjacent collinear edges are not simple.
    // Edge (2,2)-(1,0) touches edge (0,0)-(2,0) at vertex (1,0).
    Polygon coll{{Vec2{0, 0}, Vec2{2, 0}, Vec2{2, 2}, Vec2{1, 0}, Vec2{0, 1}}};
    CHECK(triangulate_simple_polygon(coll).error().code == ErrorCode::InvalidMesh);
}

static void test_triangulate_pentagon()
{
    // CCW convex pentagon.
    Polygon p{{Vec2{0, 0}, Vec2{2, 0}, Vec2{2, 1}, Vec2{1, 2}, Vec2{0, 1}}};
    auto r = triangulate_simple_polygon(p);
    CHECK(r.has_value());
    if (r) {
        CHECK(r->size() == 3);
        // Determinism: identical input produces identical output.
        auto r2 = triangulate_simple_polygon(p);
        CHECK(r2.has_value());
        if (r2) {
            CHECK(r->size() == r2->size());
            if (r->size() == r2->size()) {
                for (std::size_t i = 0; i < r->size(); ++i) {
                    CHECK((*r)[i].vertices == (*r2)[i].vertices);
                }
            }
        }
    }
}

int main()
{
    test_vec2();
    test_triangulate_square();
    test_triangulate_invalid();
    test_triangulate_self_touch();
    test_triangulate_collinear_overlap();
    test_triangulate_pentagon();
    if (failures == 0) {
        std::printf("geometry_test: OK\n");
        return 0;
    }
    std::printf("geometry_test: %d failures\n", failures);
    return 1;
}
