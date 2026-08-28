#include "test_framework.hpp"

#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>

#include <limits>

using namespace vwmini;

namespace {

Polygon tri(Vec2 a, Vec2 b, Vec2 c) {
  Polygon p;
  p.vertices = {a, b, c};
  return p;
}

// The canonical two-cell square: (0,0)-(2,2) split by the (0,2)-(2,0) diagonal.
std::vector<Polygon> square_mesh() {
  return {tri({0, 0}, {2, 0}, {2, 2}), tri({0, 0}, {2, 2}, {0, 2})};
}

// Four triangles sharing the centre vertex (1,1): the 2x2 square split by both
// diagonals. Used to exercise exact through-vertex passes.
std::vector<Polygon> four_triangle_mesh() {
  return {tri({0, 0}, {2, 0}, {1, 1}),  // bottom
          tri({0, 0}, {1, 1}, {0, 2}),  // left
          tri({0, 2}, {1, 1}, {2, 2}),  // top
          tri({2, 2}, {1, 1}, {2, 0})}; // right
}

std::vector<Polygon> l_corridor_mesh() {
  Polygon l;
  l.vertices = {{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}};
  const auto tris = triangulate_simple_polygon(l);
  // Valid CCW simple polygon, so triangulation must succeed.
  if (!tris) {
    return {};
  }
  return *tris;
}

[[nodiscard]] bool polyline_contained(const NavMesh &mesh,
                                      const std::vector<Vec2> &points) {
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    for (int k = 0; k <= 16; ++k) {
      const float u = static_cast<float>(k) / 16.0f;
      const Vec2 sample = points[i] + (points[i + 1] - points[i]) * u;
      if (!mesh.contains(sample)) {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] float polyline_length(const std::vector<Vec2> &points) {
  float total = 0.0f;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    total += length(points[i + 1] - points[i]);
  }
  return total;
}

} // namespace

TEST_CASE(path_direct_within_one_cell) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  const auto path = find_path(*mesh, {0.2f, 0.2f}, {1.0f, 0.5f});
  CHECK(path.has_value());
  if (!path) {
    return;
  }
  CHECK_EQ(path->points.size(), std::size_t{2});
  CHECK((path->points[0] == Vec2{0.2f, 0.2f}));
  CHECK((path->points[1] == Vec2{1.0f, 0.5f})); // exact endpoints preserved
}

TEST_CASE(path_direct_across_shared_edge) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  // Start in the top-left cell, goal in the bottom-right cell; the straight
  // segment crosses the shared diagonal once, so no waypoints are needed.
  const auto path = find_path(*mesh, {0.2f, 0.5f}, {1.8f, 1.5f});
  CHECK(path.has_value());
  if (!path) {
    return;
  }
  CHECK_EQ(path->points.size(), std::size_t{2});
  CHECK((path->points[0] == Vec2{0.2f, 0.5f}));
  CHECK((path->points[1] == Vec2{1.8f, 1.5f}));
  CHECK(polyline_contained(*mesh, path->points));
}

TEST_CASE(path_bends_around_l_corner) {
  const auto mesh = NavMesh::create(l_corridor_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  CHECK_EQ(mesh->cell_count(), std::size_t{4});

  // The straight segment from (1.7,0.4) to (0.4,1.7) crosses the notch of the
  // L, so the shortest contained route bends through the inner corner (1,1).
  const Vec2 start{1.7f, 0.4f};
  const Vec2 goal{0.4f, 1.7f};
  const auto path = find_path(*mesh, start, goal);
  CHECK(path.has_value());
  if (!path) {
    return;
  }
  CHECK_EQ(path->points.size(), std::size_t{3});
  CHECK(path->points.front() == start);
  CHECK(path->points.back() == goal);
  CHECK_NEAR(path->points[1].x, 1.0f, 1e-3f); // inner corner (1,1)
  CHECK_NEAR(path->points[1].y, 1.0f, 1e-3f);
  CHECK(polyline_contained(*mesh, path->points));
  // Two equal legs of length sqrt(0.49 + 0.36).
  CHECK_NEAR(polyline_length(path->points), 1.8439f, 1e-3f);
}

TEST_CASE(path_straight_through_shared_vertex) {
  const auto mesh = NavMesh::create(four_triangle_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  // The segment runs exactly along two shared edges through the centre
  // vertex (1,1); it is fully contained, so the path stays a single segment.
  const auto path = find_path(*mesh, {1.5f, 0.5f}, {0.5f, 1.5f});
  CHECK(path.has_value());
  if (!path) {
    return;
  }
  CHECK_EQ(path->points.size(), std::size_t{2});
  CHECK((path->points[0] == Vec2{1.5f, 0.5f}));
  CHECK((path->points[1] == Vec2{0.5f, 1.5f}));
  CHECK(polyline_contained(*mesh, path->points));
}

TEST_CASE(path_start_equals_goal) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  const auto path = find_path(*mesh, {1.0f, 1.0f}, {1.0f, 1.0f});
  CHECK(path.has_value());
  if (!path) {
    return;
  }
  CHECK_EQ(path->points.size(), std::size_t{1});
  CHECK((path->points[0] == Vec2{1.0f, 1.0f}));
}

TEST_CASE(path_endpoints_on_boundary) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  // Along the shared diagonal, both endpoints are mesh vertices.
  CHECK(find_path(*mesh, {0.0f, 0.0f}, {2.0f, 2.0f}).has_value());
  // Along the bottom boundary edge.
  CHECK(find_path(*mesh, {0.0f, 0.0f}, {2.0f, 0.0f}).has_value());
  // Midpoint of a boundary edge to a strictly interior point.
  CHECK(find_path(*mesh, {1.0f, 0.0f}, {1.0f, 1.0f}).has_value());
}

TEST_CASE(path_error_taxonomy) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  CHECK_ERROR((find_path(*mesh, {nan, 0.0f}, {1.0f, 1.0f})),
              ErrorCode::InvalidArgument);
  CHECK_ERROR((find_path(*mesh, {0.0f, nan}, {1.0f, 1.0f})),
              ErrorCode::InvalidArgument);
  CHECK_ERROR((find_path(*mesh, {1.0f, 1.0f}, {nan, 0.0f})),
              ErrorCode::InvalidArgument);
  CHECK_ERROR((find_path(*mesh, {inf, 0.0f}, {1.0f, 1.0f})),
              ErrorCode::InvalidArgument);
  CHECK_ERROR((find_path(*mesh, {5.0f, 5.0f}, {1.0f, 1.0f})),
              ErrorCode::OutsideMesh);
  CHECK_ERROR((find_path(*mesh, {1.0f, 1.0f}, {-5.0f, 5.0f})),
              ErrorCode::OutsideMesh);
  CHECK_ERROR((find_path(*mesh, {5.0f, 5.0f}, {-5.0f, -5.0f})),
              ErrorCode::OutsideMesh);

  // Two disconnected triangles: no route between the components.
  const auto disconnected = NavMesh::create(
      {tri({0, 0}, {1, 0}, {0, 1}), tri({10, 10}, {11, 10}, {10, 11})});
  CHECK(disconnected.has_value());
  if (!disconnected) {
    return;
  }
  CHECK_ERROR((find_path(*disconnected, {0.1f, 0.1f}, {10.1f, 10.1f})),
              ErrorCode::NoPath);
}

TEST_CASE(path_band_endpoints_exact) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  // Points just outside the mesh boundary (within the containment band
  // tolerance) are contained; a direct segment between two such band points
  // must be reported as a direct path with *exact* band endpoints, never
  // NoPath.
  constexpr float eps = 1e-4f; // containment band half-width
  const struct {
    Vec2 start;
    Vec2 goal;
  } band_cases[] = {
      {{1.0f, -0.5f * eps}, {1.0f, 2.0f + 0.5f * eps}}, // vertical
      {{-0.5f * eps, 1.0f}, {2.0f + 0.5f * eps, 1.0f}}, // horizontal
      {{0.5f, -0.5f * eps}, {1.5f, 2.0f + 0.5f * eps}}, // diagonal
      {{-0.5f * eps, -0.5f * eps},
       {2.0f + 0.5f * eps, 2.0f + 0.5f * eps}}, // corner to corner
  };
  for (const auto &c : band_cases) {
    CHECK(mesh->contains(c.start));
    CHECK(mesh->contains(c.goal));
    const auto path = find_path(*mesh, c.start, c.goal);
    CHECK(path.has_value());
    if (!path) {
      continue;
    }
    CHECK_EQ(path->points.size(), std::size_t{2});
    if (path->points.size() == 2) {
      CHECK((path->points[0] == c.start));
      CHECK((path->points[1] == c.goal));
    }
  }
}

TEST_CASE(path_deterministic) {
  const auto mesh = NavMesh::create(l_corridor_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  const Vec2 start{1.7f, 0.4f};
  const Vec2 goal{0.4f, 1.7f};
  const auto first = find_path(*mesh, start, goal);
  const auto second = find_path(*mesh, start, goal);
  CHECK(first.has_value());
  CHECK(second.has_value());
  if (!first || !second) {
    return;
  }
  CHECK(first->points == second->points);
}
