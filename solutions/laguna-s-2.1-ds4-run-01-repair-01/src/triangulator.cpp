// VWmini triangulation: validate + ear-clip one simple CCW hole-free outline.
#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace vwmini {

inline constexpr float kEpsilon = 1e-4f;
inline constexpr float kEpsSq = kEpsilon * kEpsilon;

namespace {

// Orientation sign of (a,b,c): cross(b-a, c-a). POS/NEG/ZERO buckets tolerate epsilon.
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

bool on_segment(const Vec2 p, const Vec2 q, const Vec2 r) noexcept
{
    // Assumes collinear(p,q,r); true when q lies within segment pr's bbox (+eps slack).
    const bool bx = q.x >= std::min(p.x, r.x) - kEpsilon && q.x <= std::max(p.x, r.x) + kEpsilon;
    const bool by = q.y >= std::min(p.y, r.y) - kEpsilon && q.y <= std::max(p.y, r.y) + kEpsilon;
    return bx && by;
}

// True if closed segments ab and cd share any point (touch/cross/collinear overlap),
// excluding cases where they merely meet at a shared endpoint is NOT excluded here --
// callers pass only non-adjacent edge pairs so any contact counts as self-intersection.
bool segments_contact(const Vec2 a, const Vec2 b, const Vec2 c, const Vec2 d) noexcept
{
    const Orient o1 = orient_sign(a, b, c);
    const Orient o2 = orient_sign(a, b, d);
    const Orient o3 = orient_sign(c, d, a);
    const Orient o4 = orient_sign(c, d, b);

    if (o1 != o2 && o3 != o4) {
        return true; // proper or general crossing
    }
    if (o1 == Orient::Zero && on_segment(a, c, b)) {
        return true;
    }
    if (o2 == Orient::Zero && on_segment(a, d, b)) {
        return true;
    }
    if (o3 == Orient::Zero && on_segment(c, a, d)) {
        return true;
    }
    if (o4 == Orient::Zero && on_segment(c, b, d)) {
        return true;
    }
    return false;
}

// Strictly interior to CCW triangle (A,B,C): all sub-crossings positive beyond eps.
bool strictly_inside_triangle(const Vec2 p, const Vec2 a, const Vec2 b, const Vec2 c) noexcept
{
    if (orient_sign(a, b, p) != Orient::Pos) {
        return false;
    }
    if (orient_sign(b, c, p) != Orient::Pos) {
        return false;
    }
    if (orient_sign(c, a, p) != Orient::Pos) {
        return false;
    }
    return true;
}

} // namespace

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon)
{
    const auto& verts = polygon.vertices;

    // --- Finiteness ----------------------------------------------------------
    for (const Vec2 v : verts) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
            return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite vertex coordinate"});
        }
    }

    const int n = static_cast<int>(verts.size());
    if (n < 3) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "polygon has fewer than three vertices"});
    }

    // --- Cyclic consecutive duplicates (includes first==last) ----------------
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        if (verts[i] == verts[j]) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "cyclic duplicate vertex"});
        }
    }

    // --- Signed double area: must be CCW (>0) and non-degenerate -------------
    double s = 0.0;
    for (int i = 0; i < n; ++i) {
        const Vec2 a = verts[i];
        const Vec2 b = verts[(i + 1) % n];
        s += static_cast<double>(cross(a, b));
    }
    if (!(s > static_cast<double>(kEpsSq))) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "degenerate or clockwise outline"});
    }

    // --- Self-intersection among non-adjacent edges --------------------------
    for (int i = 0; i < n; ++i) {
        const Vec2 a = verts[i];
        const Vec2 b = verts[(i + 1) % n];
        for (int j = i + 2; j < n; ++j) {
            // Skip the wrap-around adjacent pair (edge n-1 vs edge 0).
            if (i == 0 && j == n - 1) {
                continue;
            }
            const Vec2 c = verts[j];
            const Vec2 d = verts[(j + 1) % n];
            if (segments_contact(a, b, c, d)) {
                return std::unexpected(Error{ErrorCode::InvalidMesh, "self-intersecting outline"});
            }
        }
    }

    // --- Ear clipping --------------------------------------------------------
    std::vector<int> ring(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        ring[static_cast<std::size_t>(i)] = i;
    }

    std::vector<Polygon> out;
    out.reserve(n - 2);

    int guard = n * n + 4; // safety bound; simple polygons always terminate earlier
    while (ring.size() >= 3 && --guard > 0) {
        bool clipped = false;
        const int m = static_cast<int>(ring.size());
        for (int k = 0; k < m; ++k) {
            const int pi = ring[(k - 1 + m) % m];
            const int ci = ring[k];
            const int ni = ring[(k + 1) % m];
            const Vec2 prev = verts[pi];
            const Vec2 cur = verts[ci];
            const Vec2 nxt = verts[ni];

            const float cr = cross(cur - prev, nxt - cur);
            if (cr <= kEpsilon) {
                // Collinear ear tip: drop redundant vertex without emitting a triangle.
                ring.erase(ring.begin() + k);
                clipped = true;
                break;
            }
            // Convex candidate: valid ear iff no other vertex is strictly inside it.
            bool is_ear = true;
            for (int t = 0; t < m; ++t) {
                if (t == k || t == (k - 1 + m) % m || t == (k + 1) % m) {
                    continue;
                }
                if (strictly_inside_triangle(verts[ring[t]], prev, cur, nxt)) {
                    is_ear = false;
                    break;
                }
            }
            if (is_ear) {
                out.push_back(Polygon{{prev, cur, nxt}});
                ring.erase(ring.begin() + k);
                clipped = true;
                break;
            }
        }
        if (!clipped) {
            // No ear found despite remaining polygon: treat as invalid geometry.
            return std::unexpected(Error{ErrorCode::InvalidMesh, "triangulation failed"});
        }
    }

    if (out.empty()) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "no triangles produced"});
    }
    return out;
}

} // namespace vwmini
