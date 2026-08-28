#include <vwmini/geometry.hpp>

#include "internal/geometry_detail.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace vwmini {

float length(Vec2 value) noexcept {
  const double dx = static_cast<double>(value.x);
  const double dy = static_cast<double>(value.y);
  return static_cast<float>(std::sqrt(dx * dx + dy * dy));
}

Vec2 normalized(Vec2 value) noexcept {
  const float len = length(value);
  if (len == 0.0f) {
    return {0.0f, 0.0f};
  }
  return {value.x / len, value.y / len};
}

namespace {

[[nodiscard]] Error invalid_mesh(const char *reason) {
  return Error{ErrorCode::InvalidMesh, std::string(reason)};
}

// Deterministic ear clipping over the remaining vertices of a validated simple
// CCW polygon. Returns the clipped triangles as index triples in CCW order.
[[nodiscard]] std::vector<std::array<std::size_t, 3>>
clip_ears(const std::vector<Vec2> &v) {
  const std::size_t n = v.size();
  std::vector<std::size_t> next(n);
  std::vector<std::size_t> prev(n);
  std::vector<bool> alive(n, true);
  for (std::size_t i = 0; i < n; ++i) {
    next[i] = (i + 1) % n;
    prev[i] = (i + n - 1) % n;
  }
  std::size_t remaining = n;
  std::vector<std::array<std::size_t, 3>> out;

  // Iterate the cyclic list starting at a fixed seed; after each clip, resume
  // the scan from the predecessor so every vertex is revisited
  // deterministically.
  std::size_t cursor = 0;
  while (remaining > 3) {
    std::size_t i = cursor;
    bool clipped = false;
    for (std::size_t scanned = 0; scanned < remaining; ++scanned) {
      const std::size_t a = prev[i];
      const std::size_t b = i;
      const std::size_t c = next[i];
      const bool convex = detail::signed_area2(v[a], v[b], v[c]) > 0.0;
      if (convex) {
        bool empty = true;
        for (std::size_t k = next[c]; k != a; k = next[k]) {
          if (detail::point_in_triangle_closed(v[k], v[a], v[b], v[c])) {
            empty = false;
            break;
          }
        }
        if (empty) {
          out.push_back({a, b, c});
          alive[b] = false;
          next[a] = c;
          prev[c] = a;
          --remaining;
          cursor = a;
          clipped = true;
          break;
        }
      }
      i = next[i];
    }
    // A validated simple polygon always has at least two ears; an exhausted
    // scan indicates a validation gap and is reported as failure by the caller.
    if (!clipped) {
      out.clear();
      break;
    }
  }
  if (remaining == 3) {
    std::size_t a = 0;
    while (!alive[a]) {
      ++a;
    }
    out.push_back({a, next[a], next[next[a]]});
  }
  return out;
}

} // namespace

Result<std::vector<Polygon>>
triangulate_simple_polygon(const Polygon &polygon) {
  using detail::kEpsilonD;

  for (const Vec2 v : polygon.vertices) {
    if (!detail::is_finite(v)) {
      return std::unexpected(
          Error{ErrorCode::InvalidArgument, "non-finite polygon vertex"});
    }
  }
  const std::size_t n = polygon.vertices.size();
  if (n < 3) {
    return std::unexpected(invalid_mesh("fewer than three outline vertices"));
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (polygon.vertices[i] == polygon.vertices[(i + 1) % n]) {
      return std::unexpected(
          invalid_mesh("cyclic consecutive duplicate vertex"));
    }
  }
  const double area = detail::polygon_signed_area(polygon);
  if (area <= kEpsilonD * kEpsilonD) {
    return std::unexpected(
        invalid_mesh("clockwise or degenerate outline winding"));
  }
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const bool adjacent = (j == i + 1) || (i == 0 && j == n - 1);
      if (adjacent) {
        continue;
      }
      if (detail::segments_intersect(
              polygon.vertices[i], polygon.vertices[(i + 1) % n],
              polygon.vertices[j], polygon.vertices[(j + 1) % n])) {
        return std::unexpected(invalid_mesh("outline self-intersection"));
      }
    }
  }

  const auto clipped = clip_ears(polygon.vertices);
  if (clipped.empty()) {
    return std::unexpected(invalid_mesh("triangulation failed"));
  }

  std::vector<Polygon> result;
  double area_sum = 0.0;
  result.reserve(clipped.size());
  for (const auto &tri : clipped) {
    const Vec2 a = polygon.vertices[tri[0]];
    const Vec2 b = polygon.vertices[tri[1]];
    const Vec2 c = polygon.vertices[tri[2]];
    if (!detail::is_valid_ccw_triangle(a, b, c)) {
      return std::unexpected(invalid_mesh("degenerate output triangle"));
    }
    area_sum += 0.5 * detail::signed_area2(
                          a, b, c); // half-area, matching the shoelace area
    Polygon t;
    t.vertices = {a, b, c};
    result.push_back(std::move(t));
  }
  const double budget = kEpsilonD * std::max(1.0, std::abs(area));
  if (std::abs(area_sum - area) > budget) {
    return std::unexpected(invalid_mesh("triangulation area drift"));
  }
  return result;
}

} // namespace vwmini
