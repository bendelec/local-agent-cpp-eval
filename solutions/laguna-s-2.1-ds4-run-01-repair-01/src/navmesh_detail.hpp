// Private implementation details for vwmini::NavMesh.
// Included ONLY by src/nav_mesh.cpp. Not part of the public API.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>
#include <vwmini/geometry.hpp>

namespace vwmini {

inline constexpr float kEpsilon = 1e-4f;
inline constexpr float kEpsSq = kEpsilon * kEpsilon;

namespace detail {

// --- Finite / scalar validation -------------------------------------------------
bool finite_vec(Vec2 p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

// Twice signed area of triangle abc in double precision (>= contract threshold).
double signed_double_area(Vec2 a, Vec2 b, Vec2 c) noexcept {
    const double abx = static_cast<double>(b.x) - a.x;
    const double aby = static_cast<double>(b.y) - a.y;
    const double acx = static_cast<double>(c.x) - a.x;
    const double acy = static_cast<double>(c.y) - a.y;
    return abx * acy - aby * acx;
}

enum class Orient { Pos, Neg, Zero };
Orient orient_strict(Vec2 a, Vec2 b, Vec2 c) noexcept {
    const double cr = signed_double_area(a, b, c);
    if (cr > kEpsilon) {
        return Orient::Pos;
    }
    if (cr < -kEpsilon) {
        return Orient::Neg;
    }
    return Orient::Zero;
}

float dot_v(Vec2 a, Vec2 b) noexcept {
    return a.x * b.x + a.y * b.y;
}

float dist_sq_point_seg(Vec2 p, Vec2 a, Vec2 b) noexcept {
    const Vec2 ba = b - a;
    const float len2 = dot_v(ba, ba);
    if (len2 <= kEpsSq) {
        return dot_v(p - a, p - a);
    }
    float t = dot_v(p - a, ba) / len2;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    const Vec2 closest = a + ba * t;
    return dot_v(p - closest, p - closest);
}

bool endpoints_near(Vec2 a, Vec2 b) noexcept {
    return dot_v(a - b, a - b) <= kEpsSq;
}

// Strictly inside a CCW triangle (all orientations strictly positive beyond epsilon).
bool strictly_inside(const Vec2 p, const Vec2* v) noexcept {
    return orient_strict(v[0], v[1], p) == Orient::Pos &&
           orient_strict(v[1], v[2], p) == Orient::Pos &&
           orient_strict(v[2], v[0], p) == Orient::Pos;
}

// Closed-triangle membership: strict interior OR within epsilon of any edge/vertex.
bool covered_by_closed_triangle(Vec2 p, const Vec2* v) noexcept {
    if (strictly_inside(p, v)) {
        return true;
    }
    if (dist_sq_point_seg(p, v[0], v[1]) <= kEpsSq) {
        return true;
    }
    if (dist_sq_point_seg(p, v[1], v[2]) <= kEpsSq) {
        return true;
    }
    if (dist_sq_point_seg(p, v[2], v[0]) <= kEpsSq) {
        return true;
    }
    return false;
}

// True when point p lies on closed segment rs at an interior location (not near r or s).
bool vertex_on_edge_interior(Vec2 p, Vec2 r, Vec2 s) noexcept {
    if (dist_sq_point_seg(p, r, s) > kEpsSq) {
        return false;
    }
    if (endpoints_near(p, r) || endpoints_near(p, s)) {
        return false;
    }
    return true;
}

// Improper contact between two edges: a transverse proper crossing away from any shared
// endpoint, or a collinear/T-junction contact landing on a NON-endpoint interior of one
// edge. Mere coincident-vertex touch is NOT improper here.
bool edge_improper_contact(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept {
    const bool shares_endpoint = endpoints_near(a, c) || endpoints_near(a, d) ||
                                 endpoints_near(b, c) || endpoints_near(b, d);
    auto opposite = [](Orient x, Orient y) -> bool {
        return (x == Orient::Pos && y == Orient::Neg) || (x == Orient::Neg && y == Orient::Pos);
    };
    const Orient oa = orient_strict(a, b, c);
    const Orient ob = orient_strict(a, b, d);
    const Orient oc = orient_strict(c, d, a);
    const Orient od = orient_strict(c, d, b);
    if (!shares_endpoint && opposite(oa, ob) && opposite(oc, od)) {
        return true; // transverse proper crossing away from any shared endpoint
    }
    if (!shares_endpoint) {
        if (oa == Orient::Zero && vertex_on_edge_interior(c, a, b)) {
            return true;
        }
        if (ob == Orient::Zero && vertex_on_edge_interior(d, a, b)) {
            return true;
        }
        if (oc == Orient::Zero && vertex_on_edge_interior(a, c, d)) {
            return true;
        }
        if (od == Orient::Zero && vertex_on_edge_interior(b, c, d)) {
            return true;
        }
    }
    return false;
}

// A single accepted CCW triangle stored as three ordered vertices.
struct Tri {
    Vec2 v[3];
};

// Parameter interval [lo,hi] of segment P(t)=A+t*D (t in [0,1]) lying inside closed tri.
// Computed analytically by clipping against each closed half-plane with an epsilon margin.
std::pair<double, double> triangle_interval(const Tri& t, Vec2 a, Vec2 d) noexcept {
    double lo = 0.0;
    double hi = 1.0;
    for (int e = 0; e < 3; ++e) {
        const Vec2 va = t.v[e];
        const Vec2 vb = t.v[(e + 1) % 3];
        const Vec2 edge = vb - va; // directed edge of the CCW triangle
        // f(t) = cross(edge, P(t)-va) ; >= -eps on the closed interior side.
        const double base = static_cast<double>(edge.x) * (static_cast<double>(a.y) - va.y) -
                            static_cast<double>(edge.y) * (static_cast<double>(a.x) - va.x);
        const double slope = static_cast<double>(edge.x) * static_cast<double>(d.y) -
                             static_cast<double>(edge.y) * static_cast<double>(d.x);
        const double tol = kEpsilon;
        if (slope == 0.0) {
            if (base < -tol) {
                return {1.0, 0.0}; // outside this half-plane entirely
            }
            continue;
        }
        const double bound = (-tol - base) / slope;
        if (slope > 0.0) {
            lo = std::max(lo, bound);
        } else {
            hi = std::min(hi, bound);
        }
        if (lo > hi) {
            return {1.0, 0.0};
        }
    }
    return {lo, hi};
}

// True when every point of segment AB is contained by the union of the given triangles.
bool segment_covered(const std::vector<Tri>& tris, Vec2 a, Vec2 b) noexcept {
    const Vec2 d = b - a;
    const double seg_len = std::hypot(static_cast<double>(d.x), static_cast<double>(d.y));
    if (seg_len <= kEpsilon) {
        return true; // degenerate/coincident endpoints assumed already contained
    }
    struct Iv {
        double lo, hi;
    };
    std::vector<Iv> ivs;
    ivs.reserve(tris.size());
    for (const auto& t : tris) {
        const auto [lo, hi] = triangle_interval(t, a, d);
        if (hi >= lo) {
            ivs.push_back({std::clamp(lo, 0.0, 1.0), std::clamp(hi, 0.0, 1.0)});
        }
    }
    if (ivs.empty()) {
        return false;
    }
    std::sort(ivs.begin(), ivs.end(), [](const Iv& x, const Iv& y) { return x.lo < y.lo; });
    double cur = 0.0;
    for (const auto& iv : ivs) {
        if (iv.lo > cur + kEpsilon) {
            return false; // gap before this interval starts
        }
        cur = std::max(cur, iv.hi);
    }
    return cur >= 1.0 - kEpsilon;
}

// Index of first triangle whose closure contains p, or SIZE_MAX.
std::size_t containing_index(const std::vector<Tri>& tris, Vec2 p) noexcept {
    for (std::size_t i = 0; i < tris.size(); ++i) {
        if (covered_by_closed_triangle(p, tris[i].v)) {
            return i;
        }
    }
    return static_cast<std::size_t>(-1);
}

} // namespace detail

struct NavMesh::Impl {
    std::vector<detail::Tri> tris;       // accepted input order preserved
    std::vector<std::array<int, 3>> adj; // per-cell neighbour index per edge (-1=boundary)
};

} // namespace vwmini
