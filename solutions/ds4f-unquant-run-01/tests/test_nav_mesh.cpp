#include "test_framework.hpp"

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

} // namespace

TEST_CASE(nav_mesh_create_valid) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  CHECK_EQ(mesh->cell_count(), std::size_t{2});
  NavMesh copy = *mesh; // regular value: shared immutable state
  CHECK_EQ(copy.cell_count(), std::size_t{2});

  const auto single = NavMesh::create({tri({0, 0}, {2, 0}, {0, 2})});
  CHECK(single.has_value());
  CHECK_EQ(single->cell_count(), std::size_t{1});
}

TEST_CASE(nav_mesh_create_msh004_invalid) {
  // empty
  CHECK_ERROR((NavMesh::create({})), ErrorCode::InvalidMesh);
  // not a triangle: too many vertices
  CHECK_ERROR((NavMesh::create({[] {
                Polygon p;
                p.vertices = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
                return p;
              }()})),
              ErrorCode::InvalidMesh);
  // not a triangle: too few vertices
  CHECK_ERROR((NavMesh::create({[] {
                Polygon p;
                p.vertices = {{0, 0}, {2, 0}};
                return p;
              }()})),
              ErrorCode::InvalidMesh);
  // clockwise
  CHECK_ERROR((NavMesh::create({tri({0, 0}, {0, 2}, {2, 0})})),
              ErrorCode::InvalidMesh);
  // degenerate (collinear)
  CHECK_ERROR((NavMesh::create({tri({0, 0}, {1, 0}, {2, 0})})),
              ErrorCode::InvalidMesh);
  // overlapping triangles (one contains the other)
  CHECK_ERROR((NavMesh::create(
                  {tri({0, 0}, {2, 0}, {1, 1}), tri({0, 0}, {2, 0}, {1, 2})})),
              ErrorCode::InvalidMesh);
  // overlapping triangles (partial chamber overlap, crossing edges)
  CHECK_ERROR((NavMesh::create({tri({0, 0}, {2, 0}, {1, 1}),
                                tri({1, 0}, {3, 0}, {0.5f, 1})})),
              ErrorCode::InvalidMesh);
  // non-manifold: one edge claimed by three triangles (overlap fires first)
  CHECK_ERROR((NavMesh::create({tri({0, 0}, {2, 0}, {1, 1}),
                                tri({2, 0}, {0, 0}, {1, -1}),
                                tri({0, 0}, {2, 0}, {1, 3})})),
              ErrorCode::InvalidMesh);
}

TEST_CASE(nav_mesh_create_non_finite) {
  Polygon bad = tri({0, 0}, {2, 0}, {0, 2});
  bad.vertices[1].x = std::numeric_limits<float>::infinity();
  CHECK_ERROR((NavMesh::create({bad})), ErrorCode::InvalidArgument);
  bad = tri({0, 0}, {2, 0}, {0, 2});
  bad.vertices[2].y = std::nanf("");
  CHECK_ERROR((NavMesh::create({bad})), ErrorCode::InvalidArgument);
}

TEST_CASE(nav_mesh_accepts_boundary_contacts) {
  // complete shared edge: valid adjacency
  CHECK(NavMesh::create(
            {tri({0, 0}, {2, 0}, {1, 1}), tri({2, 0}, {0, 0}, {1, -1})})
            .has_value());
  // T-junction/crack: the top triangle's base edge is split into two edges of
  // the two bottom triangles; no complete edge match exists, so all stay
  // boundary edges
  CHECK(NavMesh::create({tri({0, 0}, {2, 0}, {1, 1}),
                         tri({0, 0}, {1, -1}, {1, 0}),
                         tri({1, 0}, {1, -1}, {2, 0})})
            .has_value());
  // a triangle nested inside another triangle's interior is an overlap: invalid
  CHECK_ERROR((NavMesh::create(
                  {tri({0, 0}, {2, 0}, {1, 1}), tri({0, 0}, {1, 0}, {1, 1})})),
              ErrorCode::InvalidMesh);
  // point touch only
  CHECK(NavMesh::create(
            {tri({0, 0}, {1, 0}, {1, 1}), tri({2, 0}, {3, 0}, {2.5f, 1})})
            .has_value());
}

TEST_CASE(nav_mesh_contains) {
  const auto mesh = NavMesh::create(square_mesh());
  CHECK(mesh.has_value());
  if (!mesh) {
    return;
  }
  CHECK(mesh->contains({1.0f, 1.0f})); // diagonal interior
  CHECK(mesh->contains({0.5f, 1.8f}));
  CHECK(mesh->contains({0.0f, 0.0f})); // corner
  CHECK(mesh->contains({1.0f, 2.0f})); // boundary edge
  CHECK(mesh->contains({1.0f, 0.0f}));
  CHECK(!mesh->contains({3.0f, 3.0f}));
  CHECK(!mesh->contains({-1.0f, 0.0f}));

  const auto single = NavMesh::create({tri({0, 0}, {2, 0}, {0, 2})});
  CHECK(single.has_value());
  if (!single) {
    return;
  }
  // boundary tolerance: within epsilon of an edge counts as contained
  CHECK(single->contains({1.0f, -0.5e-4f}));
  CHECK(!single->contains({1.0f, -2.0e-4f}));
  // strict interior near the hypotenuse
  CHECK(single->contains({0.5f, 0.5f}));
  CHECK(!single->contains({1.5f, 1.5f}));
}

TEST_CASE(nav_mesh_subset_and_const_use) {
  // MSH-008: any subset of valid triangles is a valid mesh
  const auto diagonal = NavMesh::create({square_mesh()[1]});
  CHECK(diagonal.has_value());
  if (!diagonal) {
    return;
  }
  CHECK_EQ(diagonal->cell_count(), std::size_t{1});
  const NavMesh &ref = *diagonal;
  CHECK(ref.contains({0.5f, 1.5f}));
  CHECK(!ref.contains({1.5f, 0.5f}));
}
