#include "test_framework.hpp"

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <limits>

using namespace vwmini;

namespace {

Polygon make_poly(std::initializer_list<Vec2> vs) {
  Polygon p;
  p.vertices = vs;
  return p;
}

double polygon_signed_area_for_test(const Polygon &p) {
  double sum = 0.0;
  for (std::size_t i = 0; i < p.vertices.size(); ++i) {
    const Vec2 a = p.vertices[i];
    const Vec2 b = p.vertices[(i + 1) % p.vertices.size()];
    sum += static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
  }
  return 0.5 * sum;
}

bool same_point_seq(const std::vector<Polygon> &a,
                    const std::vector<Polygon> &b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].vertices != b[i].vertices) {
      return false;
    }
  }
  return true;
}

double tri_area(const Polygon &t) {
  return 0.5 *
         cross(t.vertices[1] - t.vertices[0], t.vertices[2] - t.vertices[0]);
}

} // namespace

TEST_CASE(vec2_operations) {
  const Vec2 a{3.0f, 4.0f};
  CHECK_EQ((a + Vec2{1.0f, -1.0f}), (Vec2{4.0f, 3.0f}));
  CHECK_EQ((a - Vec2{1.0f, 1.0f}), (Vec2{2.0f, 3.0f}));
  CHECK_EQ((a * 2.0f), (Vec2{6.0f, 8.0f}));
  CHECK_EQ((2.0f * a), (Vec2{6.0f, 8.0f}));
  CHECK_NEAR(dot(a, Vec2{1.0f, 2.0f}), 11.0, 1e-6);
  CHECK_NEAR(cross(a, Vec2{1.0f, 0.0f}), -4.0, 1e-6);
  CHECK_NEAR(length(a), 5.0, 1e-6);
  const Vec2 u = normalized(a);
  CHECK_NEAR(length(u), 1.0, 1e-6);
  CHECK_EQ(normalized((Vec2{0.0f, 0.0f})), (Vec2{0.0f, 0.0f}));
  CHECK((Vec2{1.0f, 2.0f} == Vec2{1.0f, 2.0f}));
  CHECK(!(Vec2{1.0f, 2.0f} == Vec2{1.0f, 2.0f + 1e-5f})); // exact equality
}

TEST_CASE(triangulate_square) {
  const Polygon square = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
  const auto result = triangulate_simple_polygon(square);
  CHECK(result.has_value());
  if (!result) {
    return;
  }
  CHECK_EQ(result->size(), std::size_t{2});
  double sum = 0.0;
  for (const Polygon &t : *result) {
    CHECK_EQ(t.vertices.size(), std::size_t{3});
    CHECK(tri_area(t) > 0.0); // CCW
    for (const Vec2 v : t.vertices) {
      const bool is_input = v == Vec2{0, 0} || v == Vec2{2, 0} ||
                            v == Vec2{2, 2} || v == Vec2{0, 2};
      CHECK(is_input);
    }
    sum += tri_area(t);
  }
  CHECK_NEAR(sum, 4.0, 1e-4);
}

TEST_CASE(triangulate_pentagon_with_collinear_vertex) {
  const Polygon p = make_poly({{0, 0}, {1, 0}, {2, 0}, {2, 2}, {0, 2}});
  const auto result = triangulate_simple_polygon(p);
  CHECK(result.has_value());
  if (!result) {
    return;
  }
  double sum = 0.0;
  for (const Polygon &t : *result) {
    CHECK_EQ(t.vertices.size(), std::size_t{3});
    CHECK(tri_area(t) > 0.0);
    sum += tri_area(t);
  }
  CHECK_NEAR(sum, 4.0, 1e-4);
}

TEST_CASE(triangulate_deterministic_and_area_conservation) {
  const Polygon star =
      make_poly({{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}});
  const auto a = triangulate_simple_polygon(star);
  const auto b = triangulate_simple_polygon(star);
  CHECK(a.has_value());
  CHECK(b.has_value());
  if (!a || !b) {
    return;
  }
  CHECK(same_point_seq(*a, *b));
  const double input_area = polygon_signed_area_for_test(star);
  double sum = 0.0;
  for (const Polygon &t : *a) {
    sum += tri_area(t);
  }
  const double budget = 1e-4 * std::max(1.0, std::abs(input_area));
  CHECK(std::abs(sum - input_area) <= budget);

  // non-adjacent edges of the input must not intersect; a self-touch is invalid
  const Polygon pinch = make_poly({{0, 0}, {2, 0}, {2, 2}, {0, 0}, {0, 2}});
  CHECK_ERROR(triangulate_simple_polygon(pinch), ErrorCode::InvalidMesh);
}

TEST_CASE(triangulate_invalid_inputs) {
  const auto invalid = [](std::initializer_list<Vec2> vs) {
    return triangulate_simple_polygon(make_poly(vs));
  };
  CHECK_ERROR(invalid({{0, 0}, {1, 0}}), ErrorCode::InvalidMesh);
  CHECK_ERROR(invalid({}), ErrorCode::InvalidMesh);
  CHECK_ERROR(invalid({{0, 0}, {1, 0}, {0, 0}}), ErrorCode::InvalidMesh);
  // cyclic duplicate: first == last
  CHECK_ERROR(invalid({{0, 0}, {1, 0}, {1, 1}, {0, 0}}),
              ErrorCode::InvalidMesh);
  // clockwise
  CHECK_ERROR(invalid({{0, 0}, {0, 2}, {2, 2}, {2, 0}}),
              ErrorCode::InvalidMesh);
  // degenerate (all collinear)
  CHECK_ERROR(invalid({{0, 0}, {1, 0}, {2, 0}}), ErrorCode::InvalidMesh);
  // bowtie self-intersection
  CHECK_ERROR(invalid({{0, 0}, {2, 2}, {2, 0}, {0, 2}}),
              ErrorCode::InvalidMesh);
  // duplicate consecutive non-cyclic
  CHECK_ERROR(invalid({{0, 0}, {0, 0}, {1, 0}, {0, 1}}),
              ErrorCode::InvalidMesh);
}

TEST_CASE(triangulate_non_finite) {
  const auto invalid = [](std::initializer_list<Vec2> vs) {
    return triangulate_simple_polygon(make_poly(vs));
  };
  CHECK_ERROR(invalid({{0, 0}, {std::nanf(""), 0}, {1, 1}}),
              ErrorCode::InvalidArgument);
  CHECK_ERROR(
      invalid({{0, 0}, {1, 0}, {std::numeric_limits<float>::infinity(), 1}}),
      ErrorCode::InvalidArgument);
}

TEST_CASE(triangulate_large_convex_outline) {
  // 64-gon inscribed in a unit circle: stress the ear loop determinism.
  Polygon p;
  constexpr int kSides = 64;
  for (int i = 0; i < kSides; ++i) {
    const double angle = 2.0 * 3.14159265358979323846 * i / kSides;
    p.vertices.push_back({static_cast<float>(std::cos(angle)),
                          static_cast<float>(std::sin(angle))});
  }
  const auto result = triangulate_simple_polygon(p);
  CHECK(result.has_value());
  if (!result) {
    return;
  }
  CHECK_EQ(result->size(), std::size_t{kSides - 2});
  double sum = 0.0;
  for (const Polygon &t : *result) {
    CHECK(tri_area(t) > 0.0);
    sum += tri_area(t);
  }
  const double input_area = polygon_signed_area_for_test(p);
  CHECK_NEAR(sum, input_area, 1e-4 * std::max(1.0, input_area));
}
