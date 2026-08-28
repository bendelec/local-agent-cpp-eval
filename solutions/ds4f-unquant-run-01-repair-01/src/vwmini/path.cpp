#include <vwmini/nav_mesh.hpp>

#include "internal/geometry_detail.hpp"
#include "nav_mesh_detail.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace vwmini {

namespace {

using detail::kEpsilon;

[[nodiscard]] Error error(ErrorCode code, const char *reason) {
  return Error{code, std::string(reason)};
}

/// Best cell for `point` (input order): a strictly containing cell wins over an
/// epsilon-band match. Near shared edges and vertices the band matches several
/// cells; the strict preference identifies the cell the point actually lies in.
[[nodiscard]] std::optional<std::size_t>
find_cell(const std::vector<Polygon> &triangles, Vec2 point) {
  std::optional<std::size_t> band_match;
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    const auto &v = triangles[i].vertices;
    if (detail::point_in_triangle_strict(point, v[0], v[1], v[2])) {
      return i;
    }
    if (!band_match &&
        detail::point_contained_by_triangle(point, v[0], v[1], v[2])) {
      band_match = i;
    }
  }
  return band_match;
}

struct Event {
  double t{};          // segment parameter in (0, 1)
  std::uint8_t prio{}; // 0: proper edge crossing, 1: through-vertex pass
  std::size_t index{};
};

/// True when the segment [start, goal] is fully contained by the mesh (under
/// the same band policy as `NavMesh::contains`) and ends in a cell that also
/// band-contains the goal. The walk follows the segment through shared edges;
/// vertex pass-throughs resolve via the deterministic epsilon probes. A
/// monotonic `t_walk` keeps the walk strictly advancing along the segment, so
/// an edge crossed once is never reconsidered from its destination cell (this
/// is what lets band-contained segments enter and leave the mesh through a
/// boundary edge without being misclassified as leaving it).
[[nodiscard]] bool
segment_contained(const std::vector<Polygon> &triangles,
                  const std::vector<std::array<std::int32_t, 3>> &neighbours,
                  Vec2 start, Vec2 goal) {
  const Vec2 dir = goal - start;
  const Vec2 unit = normalized(dir);
  // Start cell: prefer the cell just ahead of `start` along the segment
  // (lands strictly inside a cell when `start` sits exactly on a shared edge
  // or vertex), falling back to the band-consistent cell of `start` itself.
  // `contains(start)` iff `find_cell(start)` succeeds, so the fallback keeps
  // the walk coherent with the containment policy.
  const auto current0 = find_cell(triangles, start + unit * kEpsilon);
  const auto start_cell = current0 ? current0 : find_cell(triangles, start);
  if (!start_cell) {
    return false;
  }

  std::size_t current = *start_cell;
  double t_walk = 0.0;
  // Defensive iteration bound: every iteration strictly increases `t_walk`
  // (each event or band crossing is consumed exactly once) and the number of
  // distinct crossings of a straight segment is bounded by the mesh size.
  const std::size_t budget = 12 * triangles.size() + 128;
  for (std::size_t it = 0; it < budget; ++it) {
    const auto &tri = triangles[current].vertices;
    if (detail::point_contained_by_triangle(goal, tri[0], tri[1], tri[2])) {
      return true;
    }

    // Collect exit events of the segment through this cell, choosing the
    // earliest one strictly after the current walk position.
    Event best{};
    bool have_event = false;
    for (std::size_t slot = 0; slot < 3; ++slot) {
      const Vec2 a = tri[slot];
      const Vec2 b = tri[(slot + 1) % 3];
      if (!detail::proper_cross(start, goal, a, b)) {
        continue;
      }
      const double denom = detail::cross2(goal - start, b - a);
      if (denom == 0.0) {
        continue;
      }
      const double t = detail::cross2(a - start, b - a) / denom;
      if (t > t_walk && t < 1.0 &&
          (!have_event || t < best.t || (t == best.t && best.prio != 0))) {
        best = Event{t, 0, slot}; // edge events win exact ties
        have_event = true;
      }
    }
    for (std::size_t i = 0; i < 3; ++i) {
      const Vec2 v = tri[i];
      if (v == start || v == goal ||
          !detail::point_on_segment(v, start, goal)) {
        continue;
      }
      const double denom = detail::dot2(dir, dir);
      if (denom == 0.0) {
        continue;
      }
      const double t = detail::dot2(v - start, dir) / denom;
      if (t > t_walk && t < 1.0 &&
          (!have_event || t < best.t || (t == best.t && best.prio != 0))) {
        best = Event{t, 1, i};
        have_event = true;
      }
    }
    if (!have_event) {
      return false;
    }
    t_walk = best.t;

    if (best.prio == 0) {
      const std::int32_t neighbour = neighbours[current][best.index];
      if (neighbour < 0) {
        // The segment crosses a boundary edge. This can also be a legal
        // entry/exit through the tolerance band (a band-contained endpoint
        // sits just outside the mathematical boundary). Continue only when
        // the segment stays within the band just beyond the crossing; in
        // that case the walk stays in the same cell and the next event is
        // the later re-entry or exit.
        const Vec2 crossing = start + dir * static_cast<float>(best.t);
        if (!find_cell(triangles, crossing + unit * (0.5f * kEpsilon))) {
          return false; // segment leaves the mesh past the tolerance band
        }
        continue;
      }
      current = static_cast<std::size_t>(neighbour);
    } else {
      // Through-vertex: both sides of the vertex must stay in the mesh.
      // Inside the vertex's epsilon ball every adjacent cell band-contains
      // the probe (corner distance), so pick the first cell other than the
      // current one; a half-epsilon probe is robustly inside that ball.
      const Vec2 v = tri[best.index];
      std::optional<std::size_t> ahead;
      for (const float delta : {0.5f * kEpsilon, 1.5f * kEpsilon}) {
        const auto cell = find_cell(triangles, v + unit * delta);
        if (cell && *cell != current) {
          ahead = cell;
          break;
        }
      }
      const auto behind = find_cell(triangles, v - unit * kEpsilon);
      if (!ahead || !behind) {
        return false; // segment leaves the mesh through the vertex
      }
      current = *ahead;
    }
  }
  return false;
}

/// Cell corridor from `start_cell` to `goal_cell` via shared edges, using
/// uniform Dijkstra with deterministic tie-breaking (lowest cell index first).
[[nodiscard]] std::optional<std::vector<std::size_t>>
route_cells(const std::vector<std::array<std::int32_t, 3>> &neighbours,
            std::size_t start_cell, std::size_t goal_cell) {
  if (start_cell == goal_cell) {
    return std::vector<std::size_t>{start_cell};
  }
  const std::size_t n = neighbours.size();
  constexpr std::size_t kInf = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> best(n, kInf);
  std::vector<std::size_t> parent(n, kInf);
  best[start_cell] = 0;

  struct Entry {
    std::size_t cost;
    std::size_t cell;
  };
  struct Order {
    bool operator()(const Entry &a, const Entry &b) const noexcept {
      if (a.cost != b.cost) {
        return a.cost > b.cost; // min-heap on cost
      }
      return a.cell > b.cell; // then lowest cell index first
    }
  };
  std::priority_queue<Entry, std::vector<Entry>, Order> queue;
  queue.push(Entry{0, start_cell});

  while (!queue.empty()) {
    const Entry top = queue.top();
    queue.pop();
    if (top.cost > best[top.cell]) {
      continue;
    }
    if (top.cell == goal_cell) {
      break;
    }
    for (const std::int32_t next : neighbours[top.cell]) {
      if (next < 0) {
        continue;
      }
      const std::size_t v = static_cast<std::size_t>(next);
      if (top.cost + 1 < best[v]) {
        best[v] = top.cost + 1;
        parent[v] = top.cell;
        queue.push(Entry{best[v], v});
      }
    }
  }
  if (best[goal_cell] == kInf) {
    return std::nullopt;
  }
  std::vector<std::size_t> corridor;
  for (std::size_t cell = goal_cell;; cell = parent[cell]) {
    corridor.push_back(cell);
    if (cell == start_cell) {
      break;
    }
  }
  std::reverse(corridor.begin(), corridor.end());
  return corridor;
}

struct Portal {
  Vec2 left;
  Vec2 right;
  Vec2 mid;
};

/// Simple-Stupid-Funnel string pulling through the portal sequence of a
/// corridor. The funnel interior is the wedge counter-clockwise from the right
/// ray to the left ray; `right` is the portal endpoint clockwise of the travel
/// direction.
[[nodiscard]] std::vector<Vec2>
funnel(const std::vector<Vec2> &points,
       const std::vector<std::array<std::size_t, 3>> &verts,
       const std::vector<std::array<std::int32_t, 3>> &neighbours,
       const std::vector<std::size_t> &corridor, Vec2 start, Vec2 goal) {
  std::vector<Portal> portals;
  portals.reserve(corridor.size() + 1);
  portals.push_back(Portal{start, start, start});
  for (std::size_t i = 1; i < corridor.size(); ++i) {
    const std::size_t from = corridor[i - 1];
    const std::size_t to = corridor[i];
    std::size_t a = 0;
    std::size_t b = 0;
    for (std::size_t slot = 0; slot < 3; ++slot) {
      if (neighbours[from][slot] == static_cast<std::int32_t>(to)) {
        a = verts[from][slot];
        b = verts[from][(slot + 1) % 3];
        break;
      }
    }
    const Vec2 pa = points[a];
    const Vec2 pb = points[b];
    const Vec2 mid{(pa.x + pb.x) * 0.5f, (pa.y + pb.y) * 0.5f};
    const Vec2 travel = normalized(mid - portals.back().mid);
    // Label the endpoint on the left of the travel direction as `left`.
    if (detail::cross2(travel, pa - mid) > 0.0) {
      portals.push_back(Portal{pa, pb, mid});
    } else {
      portals.push_back(Portal{pb, pa, mid});
    }
  }
  portals.push_back(Portal{goal, goal, goal});

  std::vector<Vec2> out;
  out.reserve(portals.size());
  Vec2 apex = start;
  Vec2 left = start;
  Vec2 right = start;
  out.push_back(start);

  std::size_t i = 1;
  // Defensive bound: each emission moves the apex onto a portal endpoint and
  // terminates, so this is unreachable in practice; it only guards the loop.
  const std::size_t max_iters = 4 * portals.size() + 16;
  std::size_t iters = 0;
  while (i < portals.size()) {
    if (++iters > max_iters) {
      break;
    }
    const Vec2 l = portals[i].left;
    const Vec2 r = portals[i].right;
    // A right endpoint at/CCW of the right ray narrows the right side,
    // unless it passed the left ray, which forces a waypoint on the left.
    if (detail::signed_area2(apex, right, r) >= 0.0) {
      if (r == apex || detail::signed_area2(apex, left, r) <= 0.0) {
        right = r;
      } else {
        out.push_back(left);
        apex = left;
        left = apex;
        right = apex;
        continue; // re-evaluate the same portal with the narrowed funnel
      }
    }
    // A left endpoint at/CW of the left ray narrows the left side, unless
    // it passed the right ray, which forces a waypoint on the right.
    if (detail::signed_area2(apex, left, l) <= 0.0) {
      if (l == apex || detail::signed_area2(apex, right, l) >= 0.0) {
        left = l;
      } else {
        out.push_back(right);
        apex = right;
        left = apex;
        right = apex;
        continue;
      }
    }
    ++i;
  }
  out.push_back(goal);
  return out;
}

/// Drops interior points within `epsilon` of the previous kept point, then
/// removes exact consecutive duplicates. The final point (the goal) is always
/// preserved.
[[nodiscard]] std::vector<Vec2> clean_polyline(std::vector<Vec2> points) {
  std::vector<Vec2> out;
  out.reserve(points.size());
  if (!points.empty()) {
    out.push_back(points.front());
  }
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    if (length(points[i] - out.back()) > kEpsilon) {
      out.push_back(points[i]);
    }
  }
  if (points.size() >= 2) {
    out.push_back(points.back());
  }
  // exact duplicate removal (e.g. start == goal)
  std::vector<Vec2> dedup;
  for (const Vec2 p : out) {
    if (dedup.empty() || !(dedup.back() == p)) {
      dedup.push_back(p);
    }
  }
  return dedup;
}

} // namespace

Result<Path> find_path(const NavMesh &mesh, Vec2 start, Vec2 goal) {
  if (!detail::is_finite(start) || !detail::is_finite(goal)) {
    return std::unexpected(
        error(ErrorCode::InvalidArgument, "non-finite endpoint"));
  }
  const auto &triangles = mesh.m_impl->triangles;
  const auto &points = mesh.m_impl->points;
  const auto &verts = mesh.m_impl->verts;
  const auto &neighbours = mesh.m_impl->neighbours;

  const auto start_cell = find_cell(triangles, start);
  if (!start_cell) {
    return std::unexpected(
        error(ErrorCode::OutsideMesh, "start outside the mesh"));
  }
  const auto goal_cell = find_cell(triangles, goal);
  if (!goal_cell) {
    return std::unexpected(
        error(ErrorCode::OutsideMesh, "goal outside the mesh"));
  }
  if (start == goal) {
    return Path{{start}};
  }
  if (segment_contained(triangles, neighbours, start, goal)) {
    return Path{{start, goal}};
  }
  const auto corridor =
      route_cells(neighbours, start_cell.value(), goal_cell.value());
  if (!corridor) {
    return std::unexpected(
        error(ErrorCode::NoPath, "disconnected mesh components"));
  }
  std::vector<Vec2> route =
      funnel(points, verts, neighbours, *corridor, start, goal);
  for (std::size_t i = 0; i + 1 < route.size(); ++i) {
    if (!segment_contained(triangles, neighbours, route[i], route[i + 1])) {
      return std::unexpected(
          error(ErrorCode::NoPath, "funnel segment verification failed"));
    }
  }
  return Path{clean_polyline(std::move(route))};
}

} // namespace vwmini
