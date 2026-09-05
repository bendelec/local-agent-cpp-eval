// Triangulator tests (MSH-002 / MSH-003).
#include "testing.hpp"

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>

using namespace vwmini;

namespace {
inline constexpr float kEpsSq = 1e-4f * 1e-4f;

// CCW unit square in a y-up plane (+x right, +y up): bottom-left -> bottom-right -> top-right -> top-left.
Polygon ccw_square()
{
    return Polygon{{Vec2{0, 0}, Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}}};
}
// Clockwise square (reverse order).
Polygon cw_square()
{
    return Polygon{{Vec2{0, 0}, Vec2{0, 1}, Vec2{1, 1}, Vec2{1, 0}}};
}
bool tris_valid(const std::vector<Polygon>& out)
{
    if (out.empty()) {
        return false;
    }
    for (const auto& tri : out) {
        if (tri.vertices.size() != 3) {
            return false;
        }
        const float darea = cross(tri.vertices[1] - tri.vertices[0],
                                  tri.vertices[2] - tri.vertices[0]);
        if (!(darea > kEpsSq)) {
            return false;
        }
    }
    return true;
}
double signed_double_area(const Polygon& p)
{
    double s = 0;
    const int n = static_cast<int>(p.vertices.size());
    for (int i = 0; i < n; ++i) {
        const Vec2 a = p.vertices[i];
        const Vec2 b = p.vertices[(i + 1) % n];
        s += static_cast<double>(cross(a, b));
    }
    return s;
}
double total_tri_area(const std::vector<Polygon>& out)
{
    double s = 0;
    for (const auto& t : out) {
        s += static_cast<double>(cross(t.vertices[1] - t.vertices[0],
                                       t.vertices[2] - t.vertices[0]));
    }
    return s;
}
} // namespace

int run_triangulator_tests()
{
    // --- Valid outlines ------------------------------------------------------
    {
        const auto r = triangulate_simple_polygon(ccw_square());
        VWM_REQUIRE(r.has_value());
        VWM_CHECK_EQ(static_cast<int>(r->size()), 2);
        VWM_REQUIRE(tris_valid(*r));
    }
    {
        const Polygon tri{{Vec2{0, 0}, Vec2{2, 0}, Vec2{0, 2}}};
        const auto r = triangulate_simple_polygon(tri);
        VWM_REQUIRE(r.has_value());
        VWM_CHECK_EQ(static_cast<int>(r->size()), 1);
    }
    {
        // Convex pentagon (CCW) -> 3 valid triangles, area preserved within tolerance.
        const Polygon pent{{Vec2{0, 0}, Vec2{2, 0}, Vec2{3, 1}, Vec2{1.5f, 2.5f}, Vec2{-0.5f, 1}}};
        const auto r = triangulate_simple_polygon(pent);
        VWM_REQUIRE(r.has_value());
        VWM_REQUIRE(tris_valid(*r));
        const double inp = signed_double_area(pent);
        const double got = total_tri_area(*r);
        VWM_REQUIRE(std::fabs(got - inp) <= 1e-3 * std::max(1.0, std::fabs(inp)));
    }
    {
        // Collinear-but-distinct boundary vertex is permitted.
        const Polygon colok{{Vec2{0, 0}, Vec2{0.5f, 0}, Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}}};
        const auto r = triangulate_simple_polygon(colok);
        VWM_REQUIRE(r.has_value());
        VWM_REQUIRE(tris_valid(*r));
    }
    {
        // Determinism: identical input yields identical output sequence.
        const auto a = triangulate_simple_polygon(ccw_square());
        const auto b = triangulate_simple_polygon(ccw_square());
        VWM_REQUIRE(a.has_value() && b.has_value());
        VWM_CHECK_EQ(static_cast<int>(a->size()), static_cast<int>(b->size()));
        bool same = true;
        for (std::size_t i = 0; i < a->size(); ++i) {
            if ((*a)[i].vertices != (*b)[i].vertices) {
                same = false;
            }
        }
        VWM_REQUIRE(same);
    }
    {
        // Every emitted triangle vertex must be an input vertex (MSH-003).
        const Polygon pent{{Vec2{0, 0}, Vec2{2, 0}, Vec2{3, 1}, Vec2{1.5f, 2.5f}, Vec2{-0.5f, 1}}};
        const auto r = triangulate_simple_polygon(pent);
        VWM_REQUIRE(r.has_value());
        bool all_input = true;
        for (const auto& t : *r) {
            for (const Vec2 v : t.vertices) {
                bool found = false;
                for (const Vec2 pv : pent.vertices) {
                    if (v == pv) {
                        found = true;
                    }
                }
                if (!found) {
                    all_input = false;
                }
            }
        }
        VWM_REQUIRE(all_input);
    }

    // --- InvalidArgument: non-finite coordinate ------------------------------
    {
        const Polygon nf{{Vec2{0, 0}, Vec2{INFINITY, 0}, Vec2{0, 1}}};
        const auto r = triangulate_simple_polygon(nf);
        VWM_REQUIRE(!r.has_value());
        VWM_CHECK_EQ(r.error().code, ErrorCode::InvalidArgument);
    }

    // --- InvalidMesh cases ---------------------------------------------------
    {
        const Polygon too_small{{Vec2{0, 0}, Vec2{1, 0}}};
        VWM_CHECK_EQ(triangulate_simple_polygon(too_small).error().code, ErrorCode::InvalidMesh);
    }
    {
        // Cyclic consecutive duplicate including first==last.
        const Polygon dup{{Vec2{0, 0}, Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 0}}};
        VWM_CHECK_EQ(triangulate_simple_polygon(dup).error().code, ErrorCode::InvalidMesh);
    }
    {
        // Clockwise winding.
        VWM_CHECK_EQ(triangulate_simple_polygon(cw_square()).error().code, ErrorCode::InvalidMesh);
    }
    {
        // Degenerate (all collinear): zero area.
        const Polygon flat{{Vec2{0, 0}, Vec2{1, 0}, Vec2{2, 0}}};
        VWM_CHECK_EQ(triangulate_simple_polygon(flat).error().code, ErrorCode::InvalidMesh);
    }
    {
        // Self-intersecting bowtie.
        const Polygon bowtie{{Vec2{0, 0}, Vec2{2, 2}, Vec2{2, 0}, Vec2{0, 2}}};
        VWM_CHECK_EQ(triangulate_simple_polygon(bowtie).error().code, ErrorCode::InvalidMesh);
    }

    return ::vwmtest::counters().failed;
}
