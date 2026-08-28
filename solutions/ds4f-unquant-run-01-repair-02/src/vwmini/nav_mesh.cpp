#include "nav_mesh_detail.hpp"

#include "internal/geometry_detail.hpp"

#include <string>
#include <utility>

namespace vwmini {

namespace {

[[nodiscard]] Error invalid_mesh(const char *reason) {
  return Error{ErrorCode::InvalidMesh, std::string(reason)};
}

// Exact-equality dedup of mesh vertices; returns the index of an equal point.
[[nodiscard]] std::size_t point_index(std::vector<Vec2> &points, Vec2 point) {
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (points[i] == point) {
      return i;
    }
  }
  points.push_back(point);
  return points.size() - 1;
}

} // namespace

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept
    : m_impl(std::move(impl)) {}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles) {
  if (triangles.empty()) {
    return std::unexpected(invalid_mesh("empty triangle list"));
  }
  std::vector<Vec2> points;
  std::vector<std::array<std::size_t, 3>> verts;
  verts.reserve(triangles.size());
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    Polygon &tri = triangles[i];
    if (tri.vertices.size() != 3) {
      return std::unexpected(
          invalid_mesh("a triangle does not have exactly three vertices"));
    }
    for (const Vec2 v : tri.vertices) {
      if (!detail::is_finite(v)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "non-finite mesh vertex"});
      }
    }
    const Vec2 a = tri.vertices[0];
    const Vec2 b = tri.vertices[1];
    const Vec2 c = tri.vertices[2];
    if (!detail::is_valid_ccw_triangle(a, b, c)) {
      return std::unexpected(invalid_mesh("clockwise or degenerate triangle"));
    }
    verts.push_back({point_index(points, a), point_index(points, b),
                     point_index(points, c)});
  }

  // MSH-004: triangle interiors must not overlap; touching at shared corner
  // points or along complete shared edges is allowed. A vertex lying on
  // another triangle's OPEN edge (within the endpoint tolerance) is an
  // invalid T-junction and is rejected; it would otherwise leave a crack.
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      const auto &ti = triangles[i].vertices;
      const auto &tj = triangles[j].vertices;
      // Vertex on open edge: within epsilon of the edge span, but away from
      // both endpoints (a vertex within epsilon of an endpoint is a corner
      // contact, which is allowed).
      const auto vertex_on_open_edge = [](Vec2 v, Vec2 a, Vec2 b) {
        return detail::dist_point_segment(v, a, a) > detail::kEpsilonD &&
               detail::dist_point_segment(v, b, b) > detail::kEpsilonD &&
               detail::dist_point_segment(v, a, b) <= detail::kEpsilonD;
      };
      for (std::size_t ei = 0; ei < 3; ++ei) {
        for (std::size_t ej = 0; ej < 3; ++ej) {
          if (detail::proper_cross(ti[ei], ti[(ei + 1) % 3], tj[ej],
                                   tj[(ej + 1) % 3])) {
            return std::unexpected(invalid_mesh("overlapping triangles"));
          }
        }
      }
      for (const Vec2 v : ti) {
        if (detail::point_in_triangle_strict(v, tj[0], tj[1], tj[2])) {
          return std::unexpected(invalid_mesh("overlapping triangles"));
        }
      }
      for (const Vec2 v : tj) {
        if (detail::point_in_triangle_strict(v, ti[0], ti[1], ti[2])) {
          return std::unexpected(invalid_mesh("overlapping triangles"));
        }
      }
      for (const Vec2 v : ti) {
        for (std::size_t ej = 0; ej < 3; ++ej) {
          if (vertex_on_open_edge(v, tj[ej], tj[(ej + 1) % 3])) {
            return std::unexpected(
                invalid_mesh("T-junction vertex on open edge"));
          }
        }
      }
      for (const Vec2 v : tj) {
        for (std::size_t ei = 0; ei < 3; ++ei) {
          if (vertex_on_open_edge(v, ti[ei], ti[(ei + 1) % 3])) {
            return std::unexpected(
                invalid_mesh("T-junction vertex on open edge"));
          }
        }
      }
      const Vec2 ci{(ti[0].x + ti[1].x + ti[2].x) / 3.0f,
                    (ti[0].y + ti[1].y + ti[2].y) / 3.0f};
      const Vec2 cj{(tj[0].x + tj[1].x + tj[2].x) / 3.0f,
                    (tj[0].y + tj[1].y + tj[2].y) / 3.0f};
      if (detail::point_in_triangle_strict(ci, tj[0], tj[1], tj[2]) ||
          detail::point_in_triangle_strict(cj, ti[0], ti[1], ti[2])) {
        return std::unexpected(invalid_mesh("overlapping triangles"));
      }
    }
  }

  // MSH-005: an edge is a shared boundary when all endpoints match exactly.
  // Count matching reverse edges; more than one means a non-manifold edge.
  auto impl = std::make_shared<Impl>();
  impl->triangles = std::move(triangles);
  impl->points = std::move(points);
  impl->verts = std::move(verts);
  impl->neighbours.assign(impl->triangles.size(), {-1, -1, -1});

  const std::size_t t = impl->triangles.size();
  for (std::size_t i = 0; i < t; ++i) {
    for (std::size_t slot = 0; slot < 3; ++slot) {
      const std::size_t from = impl->verts[i][slot];
      const std::size_t to = impl->verts[i][(slot + 1) % 3];
      std::size_t matches = 0;
      for (std::size_t j = 0; j < t; ++j) {
        if (j == i) {
          continue;
        }
        for (std::size_t k = 0; k < 3; ++k) {
          if (impl->verts[j][k] == to && impl->verts[j][(k + 1) % 3] == from) {
            ++matches;
            if (matches == 1) {
              impl->neighbours[i][slot] = static_cast<std::int32_t>(j);
            } else if (matches > 1) {
              return std::unexpected(invalid_mesh("non-manifold edge"));
            }
          }
        }
      }
    }
  }
  return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept {
  for (std::size_t i = 0; i < m_impl->triangles.size(); ++i) {
    const auto &tri = m_impl->triangles[i].vertices;
    if (detail::point_contained_by_triangle(point, tri[0], tri[1], tri[2])) {
      return true;
    }
  }
  return false;
}

std::size_t NavMesh::cell_count() const noexcept {
  return m_impl->triangles.size();
}

} // namespace vwmini
