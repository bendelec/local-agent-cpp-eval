// VWmini navigation mesh implementation (opaque storage + queries).
#include <vwmini/nav_mesh.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace vwmini {

inline constexpr float kEpsilon = 1e-4f;
inline constexpr float kEpsSq = kEpsilon * kEpsilon;

// ---------------------------------------------------------------------------
// Private mesh representation
// ---------------------------------------------------------------------------
struct NavMesh::Impl {
    std::vector<Triangle> tris;                       // accepted input order preserved
    std::vector<std::array<int, 3>> adj;              // per-cell neighbour index per edge (-1 = boundary)
};

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept
    : m_impl(std::move(impl))
{
}

std::span<const Triangle> NavMesh::triangles_view() const noexcept
{
    if (!m_impl) {
        return {};
    }
    return std::span<const Triangle>{m_impl->tris.data(), m_impl->tris.size()};
}

namespace {

// Orientation buckets tolerant to epsilon.
enum class Orient { Pos, Neg, Zero };
Orient orient_sign(const Vec2 a, const Vec2 b, const Vec2 c) noexcept
{
    const float cr = cross(b - a, c - a);
    if (cr > kEpsilon) {
        return Orient::Pos;
    }
    if (cr < -kEpsilon) {
        return Orient::Neg;
    }
    return Orient::Zero;
}

// Squared distance from point p to closed segment ab.
float dist_sq_point_seg(const Vec2 p, const Vec2 a, const Vec2 b) noexcept
{
    const Vec2 ba = b - a;
    const float len2 = dot(ba, ba);
    if (len2 <= kEpsSq) {
        return dot(p - a, p - a);
    }
    float t = dot(p - a, ba) / len2;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    const Vec2 closest = a + ba * t;
    return dot(p - closest, p - closest);
}

// True when point is strictly inside CCW triangle OR within eps of any edge segment.
bool covered_by_triangle(const Vec2 p, const Vec2 a, const Vec2 b, const Vec2 c) noexcept
{
    if (orient_sign(a, b, p) == Orient::Pos && orient_sign(b, c, p) == Orient::Pos &&
        orient_sign(c, a, p) == Orient::Pos) {
        return true;
    }
    if (dist_sq_point_seg(p, a, b) <= kEpsSq) {
        return true;
    }
    if (dist_sq_point_seg(p, b, c) <= kEpsSq) {
        return true;
    }
    if (dist_sq_point_seg(p, c, a) <= kEpsSq) {
        return true;
    }
    return false;
}

// Classify the geometric relationship between two distinct triangles' closures.
enum class PairCode { SharedEdge, Disjoint, Invalid };
struct PairInfo {
    PairCode code{PairCode::Disjoint};
    int edge_i{-1}; // matching edge index in tri i (0..2)
    int edge_j{-1}; // matching edge index in tri j (0..2)
};

int shared_edge_index(const Triangle& ti, const Triangle& tj) noexcept
{
    for (int e = 0; e < 3; ++e) {
        const Vec2 ai = ti.v[e];
        const Vec2 bi = ti.v[(e + 1) % 3];
        for (int f = 0; f < 3; ++f) {
            const Vec2 aj = tj.v[f];
            const Vec2 bj = tj.v[(f + 1) % 3];
            const bool same = (ai == aj && bi == bj);
            const bool rev = (ai == bj && bi == aj);
            if ((same || rev) && dot(bi - ai, bi - ai) > kEpsSq) {
                return e * 3 + f; // encode both indices
            }
        }
    }
    return -1;
}

// True when point p lies on closed segment rs at an interior location (not near r or s).
bool vertex_on_edge_interior(const Vec2 p, const Vec2 r, const Vec2 s) noexcept
{
    if (dist_sq_point_seg(p, r, s) > kEpsSq) {
        return false;
    }
    if (p == r || p == s) {
        return false;
    }
    return true;
}

// Strictly inside CCW triangle (all orientations strictly positive beyond eps).
bool strictly_inside_triangle(const Vec2 p, const Triangle& t) noexcept
{
    return orient_sign(t.v[0], t.v[1], p) == Orient::Pos &&
           orient_sign(t.v[1], t.v[2], p) == Orient::Pos &&
           orient_sign(t.v[2], t.v[0], p) == Orient::Pos;
}

// Improper contact between two edges: transverse crossing OR collinear/partial overlap OR
// T-junction (endpoint of one lying on the other's interior), but NOT mere coincident-shared-vertex touch.
bool edge_improper_contact(const Vec2 a, const Vec2 b, const Vec2 c, const Vec2 d) noexcept
{
    const bool shares_endpoint = (a == c) || (a == d) || (b == c) || (b == d);
    auto opposite = [](Orient x, Orient y) -> bool {
        return (x == Orient::Pos && y == Orient::Neg) || (x == Orient::Neg && y == Orient::Pos);
    };
    const Orient oa = orient_sign(a, b, c);
    const Orient ob = orient_sign(a, b, d);
    const Orient oc = orient_sign(c, d, a);
    const Orient od = orient_sign(c, d, b);
    if (!shares_endpoint && opposite(oa, ob) && opposite(oc, od)) {
        return true; // transverse proper crossing away from any shared endpoint
    }
    // Boundary/collinear cases: flag only contacts landing on a NON-endpoint interior.
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

PairInfo classify_pair(const Triangle& ti, const Triangle& tj) noexcept
{
    PairInfo info{};
    const int match = shared_edge_index(ti, tj);
    if (match >= 0) {
        const int ei = match / 3;
        const int ej = match % 3;
        // Verify apexes are on opposite sides of the shared line => manifold, no overlap.
        const Vec2 ap = ti.v[ei];
        const Vec2 bp = ti.v[(ei + 1) % 3];
        const Vec2 opp_i = ti.v[(ei + 2) % 3];
        const Vec2 opp_j = tj.v[(ej + 2) % 3];
        const Orient si = orient_sign(ap, bp, opp_i);
        const Orient sj = orient_sign(ap, bp, opp_j);
        if (si != sj && si != Orient::Zero && sj != Orient::Zero) {
            info.code = PairCode::SharedEdge;
            info.edge_i = ei;
            info.edge_j = ej;
        } else {
            info.code = PairCode::Invalid; // duplicate/folded/overlapping
        }
        return info;
    }

    // No full shared edge: reject improper contact while allowing isolated vertex touches.
    for (const Vec2 v : ti.v) {
        if (strictly_inside_triangle(v, tj)) {
            info.code = PairCode::Invalid;
            return info;
        }
    }
    for (const Vec2 v : tj.v) {
        if (strictly_inside_triangle(v, ti)) {
            info.code = PairCode::Invalid;
            return info;
        }
    }
    for (int e = 0; e < 3; ++e) {
        const Vec2 ae = ti.v[e];
        const Vec2 be = ti.v[(e + 1) % 3];
        for (int f = 0; f < 3; ++f) {
            const Vec2 cf = tj.v[f];
            const Vec2 df = tj.v[(f + 1) % 3];
            if (edge_improper_contact(ae, be, cf, df)) {
                info.code = PairCode::Invalid;
                return info;
            }
        }
    }

    info.code = PairCode::Disjoint;
    return info;
}

void link_adjacency(std::vector<std::array<int, 3>>& adj, int i, int j, int ei, int ej) noexcept
{
    adj[i][ei] = j;
    adj[j][ej] = i;
}

} // namespace

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles)
{
    auto impl = std::make_shared<Impl>();
    impl->tris.reserve(triangles.size());
    impl->adj.reserve(triangles.size());

    for (const Polygon& poly : triangles) {
        if (poly.vertices.size() != 3) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "triangle must have exactly three vertices"});
        }
        const Vec2 a = poly.vertices[0];
        const Vec2 b = poly.vertices[1];
        const Vec2 c = poly.vertices[2];
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) ||
            !std::isfinite(b.y) || !std::isfinite(c.x) || !std::isfinite(c.y)) {
            return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite vertex coordinate"});
        }
        const float darea = cross(b - a, c - a);
        if (!(darea > kEpsSq)) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "degenerate or non-CCW triangle"});
        }
        impl->tris.push_back(Triangle{a, b, c});
        impl->adj.push_back({-1, -1, -1});
    }

    if (impl->tris.empty()) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "empty triangle list"});
    }

    const int n = static_cast<int>(impl->tris.size());
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const PairInfo pi = classify_pair(impl->tris[i], impl->tris[j]);
            switch (pi.code) {
                case PairCode::SharedEdge:
                    link_adjacency(impl->adj, i, j, pi.edge_i, pi.edge_j);
                    break;
                case PairCode::Disjoint:
                    continue;
                default:
                    return std::unexpected(Error{ErrorCode::InvalidMesh, "overlapping or non-manifold triangles"});
            }
        }
    }

    return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept
{
    if (!m_impl || !std::isfinite(point.x) || !std::isfinite(point.y)) {
        return false;
    }
    for (const Triangle& t : m_impl->tris) {
        if (covered_by_triangle(point, t.v[0], t.v[1], t.v[2])) {
            return true;
        }
    }
    return false;
}

std::size_t NavMesh::cell_count() const noexcept
{
    return m_impl ? m_impl->tris.size() : 0;
}

} // namespace vwmini
