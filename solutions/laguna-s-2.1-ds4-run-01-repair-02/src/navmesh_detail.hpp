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

bool finite_vec(Vec2 p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

// Twice the signed area of triangle (a,b,c) in double precision (positive iff CCW).
double signed_double_area(Vec2 a, Vec2 b, Vec2 c) noexcept {
    const double abx = static_cast<double>(b.x) - a.x;
    const double aby = static_cast<double>(b.y) - a.y;
    const double acx = static_cast<double>(c.x) - a.x;
    const double acy = static_cast<double>(c.y) - a.y;
    return abx * acy - aby * acx;
}

// True when the triangle has strictly positive (CCW) signed area beyond epsilon^2.
bool valid_triangle(Vec2 a, Vec2 b, Vec2 c) noexcept {
    return signed_double_area(a, b, c) > static_cast<double>(kEpsSq);
}

// Strict orientation sign using exact double-precision comparison (> 0 / < 0 / == 0).
enum class Orient { Pos, Neg, Zero };
Orient orient_sign(Vec2 a, Vec2 b, Vec2 c) noexcept {
    const double cr = signed_double_area(a, b, c);
    if (cr > 0.0) {
        return Orient::Pos;
    }
    if (cr < 0.0) {
        return Orient::Neg;
    }
    return Orient::Zero;
}

// Squared Euclidean distance from p to closed segment [a, b] (scale-safe doubles).
double dist_sq_point_seg(Vec2 p, Vec2 a, Vec2 b) noexcept {
    const double ax = a.x, ay = a.y, bx = b.x, by = b.y, px = p.x, py = p.y;
    const double abx = bx - ax, aby = by - ay;
    const double ab_len_sq = abx * abx + aby * aby;
    if (ab_len_sq <= 0.0) {
        const double dpx = px - ax, dpy = py - ay;
        return dpx * dpx + dpy * dpy;
    }
    double t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq;
    if (t < 0.0) {
        t = 0.0;
    } else if (t > 1.0) {
        t = 1.0;
    }
    const double projx = ax + abx * t, projy = ay + aby * t;
    const double dx = px - projx, dy = py - projy;
    return dx * dx + dy * dy;
}

bool endpoints_near(Vec2 a, Vec2 b) noexcept {
    return dot(a - b, a - b) <= kEpsSq;
}

// Two directed edges share the same unordered endpoints within epsilon.
bool edges_match(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept {
    return (endpoints_near(a, c) && endpoints_near(b, d)) ||
           (endpoints_near(a, d) && endpoints_near(b, c));
}

// Strict point-in-triangle: boundary excluded (all edge orientations strictly positive).
bool point_in_triangle_strict(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept {
    const auto o = [](Vec2 x, Vec2 y, Vec2 z) -> double { return signed_double_area(x, y, z); };
    return o(a, b, p) > 0.0 && o(b, c, p) > 0.0 && o(c, a, p) > 0.0;
}

// Closed membership per MSH-006: strict interior OR within epsilon of any closed edge
// (Euclidean edge-distance rule only — never an orientation or parameter-space slack).
bool contains_point_triangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept {
    if (point_in_triangle_strict(p, a, b, c)) {
        return true;
    }
    if (dist_sq_point_seg(p, a, b) <= static_cast<double>(kEpsSq)) {
        return true;
    }
    if (dist_sq_point_seg(p, b, c) <= static_cast<double>(kEpsSq)) {
        return true;
    }
    if (dist_sq_point_seg(p, c, a) <= static_cast<double>(kEpsSq)) {
        return true;
    }
    return false;
}

// A vertex lying on a NON-endpoint interior of segment (a,b): T-junction marker.
bool vertex_in_segment_interior(Vec2 p, Vec2 a, Vec2 b) noexcept {
    if (dist_sq_point_seg(p, a, b) > static_cast<double>(kEpsSq)) {
        return false;
    }
    if (dot(p - a, p - a) < kEpsSq) {
        return false; // near endpoint a
    }
    if (dot(p - b, p - b) < kEpsSq) {
        return false; // near endpoint b
    }
    return true;
}

// Proper transverse intersection of segments (a,b) and (c,d), excluding shared-endpoint
// touches. Used to reject overlapping triangle interiors during create().
bool proper_segment_intersection(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept {
    const Vec2 r = b - a;
    const Vec2 s = d - c;
    const float rs = cross(r, s);
    if (std::fabs(rs) <= kEpsilon) {
        return false; // parallel/degenerate
    }
    const Vec2 qp = c - a;
    const float t = cross(qp, s) / rs;
    const float u = cross(qp, r) / rs;
    return t > 0.0f && t < 1.0f && u > 0.0f && u < 1.0f;
}

} // namespace detail

struct NavMesh::Impl {
    std::vector<std::array<Vec2, 3>> triangles;         // accepted input order preserved
    std::vector<std::array<std::size_t, 3>> neighbours; // SIZE_MAX => boundary edge
    static constexpr std::size_t no_cell = static_cast<std::size_t>(-1);

    [[nodiscard]] std::size_t cell_count() const noexcept { return triangles.size(); }

    [[nodiscard]] static Vec2 centroid(const Impl& m, std::size_t c) noexcept {
        const auto& t = m.triangles[c];
        return Vec2{(t[0].x + t[1].x + t[2].x) / 3.0f, (t[0].y + t[1].y + t[2].y) / 3.0f};
    }

    // Shared edge between adjacent cells a and b in a's directed-edge order.
    [[nodiscard]] static std::array<Vec2, 2> shared_edge(const Impl& m, std::size_t a,
                                                         std::size_t b) noexcept {
        const auto& t = m.triangles[a];
        for (int e = 0; e < 3; ++e) {
            if (m.neighbours[a][e] == b) {
                return {t[e], t[(e + 1) % 3]};
            }
        }
        return {t[0], t[1]}; // unreachable for adjacent cells
    }

    Impl(std::vector<std::array<Vec2, 3>> tris, std::vector<std::array<std::size_t, 3>> neigh)
        : triangles(std::move(tris)), neighbours(std::move(neigh)) {}
};

} // namespace vwmini
