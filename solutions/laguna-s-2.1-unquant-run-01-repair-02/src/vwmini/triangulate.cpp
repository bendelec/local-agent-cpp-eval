#include <vwmini/detail.hpp>
#include <vwmini/geometry.hpp>

#include <vector>

namespace vwmini {

namespace {

// True when outline vertices form a simple (non-self-intersecting) polygon after the
// cyclic consecutive-duplicate check. Collinear consecutive edges are allowed.
bool edges_self_intersect(const Polygon& polygon)
{
    const auto& v = polygon.vertices;
    const int n = static_cast<int>(v.size());
    // Non-adjacent edge pairs; edges sharing a vertex may only meet at that vertex.
    for (int i = 0; i < n; ++i) {
        const Vec2 a0 = v[i];
        const Vec2 a1 = v[(i + 1) % n];
        for (int j = i + 2; j < n; ++j) {
            // Skip edges that share vertex (i+1)%n == j (adjacent) etc.
            if (i == 0 && j == n - 1) {
                continue; // adjacent cyclically
            }
            const Vec2 b0 = v[j];
            const Vec2 b1 = v[(j + 1) % n];
            if (detail::proper_segment_intersection(a0, a1, b0, b1)) {
                return true;
            }
        }
    }
    return false;
}

} // namespace

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon)
{
    const auto& v = polygon.vertices;
    const int n = static_cast<int>(v.size());

    // Non-finite coordinates -> InvalidArgument.
    for (int i = 0; i < n; ++i) {
        if (!detail::finite(v[i])) {
            return std::unexpected(
                Error{ErrorCode::InvalidArgument, "non-finite vertex coordinate"});
        }
    }

    // Fewer than three vertices.
    if (n < 3) {
        return std::unexpected(
            Error{ErrorCode::InvalidMesh, "polygon has fewer than three vertices"});
    }

    // Cyclic consecutive duplicate vertices (including equal first and last).
    for (int i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1) % n];
        if (a == b) {
            return std::unexpected(
                Error{ErrorCode::InvalidMesh, "cyclic consecutive duplicate vertex"});
        }
    }

    // Clockwise or degenerate (area <= eps^2) -> InvalidMesh.
    const float area = detail::signed_polygon_area(polygon.vertices);
    if (area <= detail::epsilon_sq) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "polygon is clockwise or degenerate"});
    }

    // Self-intersections -> InvalidMesh.
    if (edges_self_intersect(polygon)) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "polygon is self-intersecting"});
    }

    // Ear clipping. Vertices kept on an active ring; collinear tips are skipped so
    // they never form degenerate ears. The first valid ear in index order is always
    // chosen, giving deterministic output.
    std::vector<int> idx;
    idx.reserve(n);
    for (int i = 0; i < n; ++i) {
        idx.push_back(i);
    }

    std::vector<Polygon> triangles;
    triangles.reserve(static_cast<std::size_t>(n) - 2);

    while (static_cast<int>(idx.size()) > 2) {
        const int m = static_cast<int>(idx.size());
        bool clipped = false;
        for (int i = 0; i < m; ++i) {
            const int pi = idx[(i - 1 + m) % m];
            const int ti = idx[i];
            const int ni = idx[(i + 1) % m];
            const Vec2 prev = v[pi];
            const Vec2 tip = v[ti];
            const Vec2 nxt = v[ni];

            // Ear tip must be a strict left turn (CCW convex vertex); collinear tips
            // are not ears and are skipped.
            if (cross(tip - prev, nxt - tip) <= 0.0f) {
                continue;
            }
            // Non-degenerate ear triangle.
            if (!detail::valid_triangle(prev, tip, nxt)) {
                continue;
            }
            // Ear is valid iff no other active vertex lies strictly inside it.
            bool ear_clear = true;
            for (int k = 0; k < m; ++k) {
                if (k == i) {
                    continue;
                }
                const int vk = idx[k];
                if (detail::point_in_triangle_strict(v[vk], prev, tip, nxt)) {
                    ear_clear = false;
                    break;
                }
            }
            if (!ear_clear) {
                continue;
            }

            triangles.push_back(Polygon{{prev, tip, nxt}});
            idx.erase(idx.begin() + i);
            clipped = true;
            break;
        }
        if (!clipped) {
            // No ear found despite passing validation: reject to stay safe/deterministic.
            return std::unexpected(
                Error{ErrorCode::InvalidMesh, "triangulation failed to find an ear"});
        }
    }

    return triangles;
}

} // namespace vwmini
