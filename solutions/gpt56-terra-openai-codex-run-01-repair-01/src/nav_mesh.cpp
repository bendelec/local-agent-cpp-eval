#include <vwmini/nav_mesh.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace vwmini {
namespace {

constexpr double kEpsilon = 1.0e-4;
constexpr double kAreaEpsilon = kEpsilon * kEpsilon;

[[nodiscard]] bool finite(Vec2 value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

[[nodiscard]] double orient(Vec2 a, Vec2 b, Vec2 c) noexcept {
    return (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
           (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
}

[[nodiscard]] double distance_squared(Vec2 a, Vec2 b) noexcept {
    const double x = static_cast<double>(a.x) - b.x;
    const double y = static_cast<double>(a.y) - b.y;
    return x * x + y * y;
}

[[nodiscard]] bool near(Vec2 a, Vec2 b) noexcept {
    return distance_squared(a, b) <= kEpsilon * kEpsilon;
}

[[nodiscard]] double point_segment_distance_squared(Vec2 point, Vec2 a, Vec2 b) noexcept {
    const double x = static_cast<double>(point.x) - a.x;
    const double y = static_cast<double>(point.y) - a.y;
    const double edge_x = static_cast<double>(b.x) - a.x;
    const double edge_y = static_cast<double>(b.y) - a.y;
    const double edge_length_squared = edge_x * edge_x + edge_y * edge_y;
    const double projection = std::clamp((x * edge_x + y * edge_y) / edge_length_squared, 0.0, 1.0);
    const double dx = x - projection * edge_x;
    const double dy = y - projection * edge_y;
    return dx * dx + dy * dy;
}

[[nodiscard]] bool strictly_in_triangle(Vec2 point, const std::array<Vec2, 3> &triangle) noexcept {
    return orient(triangle[0], triangle[1], point) > 0.0 &&
           orient(triangle[1], triangle[2], point) > 0.0 &&
           orient(triangle[2], triangle[0], point) > 0.0;
}

[[nodiscard]] bool proper_crossing(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept {
    const double ab_c = orient(a, b, c);
    const double ab_d = orient(a, b, d);
    const double cd_a = orient(c, d, a);
    const double cd_b = orient(c, d, b);
    return ((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0)) &&
           ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0));
}

[[nodiscard]] Error invalid_argument(const char *message) {
    return {ErrorCode::InvalidArgument, message};
}

[[nodiscard]] Error invalid_mesh(const char *message) {
    return {ErrorCode::InvalidMesh, message};
}

struct EdgeRef {
    std::size_t cell{};
    std::size_t edge{};
    Vec2 a{};
    Vec2 b{};
};

[[nodiscard]] bool same_edge(const EdgeRef &first, const EdgeRef &second) noexcept {
    return (near(first.a, second.a) && near(first.b, second.b)) ||
           (near(first.a, second.b) && near(first.b, second.a));
}

[[nodiscard]] bool same_direction(const EdgeRef &first, const EdgeRef &second) noexcept {
    return near(first.a, second.a) && near(first.b, second.b);
}

} // namespace

struct NavMesh::Impl {
    struct Cell {
        std::array<Vec2, 3> vertices;
        std::array<int, 3> neighbours{{-1, -1, -1}};
        int component{-1};
    };

    std::vector<Cell> cells;
};

struct MeshAccess {

    [[nodiscard]] static bool cell_contains(const NavMesh::Impl::Cell &cell, Vec2 point) noexcept {
        if (orient(cell.vertices[0], cell.vertices[1], point) >= 0.0 &&
            orient(cell.vertices[1], cell.vertices[2], point) >= 0.0 &&
            orient(cell.vertices[2], cell.vertices[0], point) >= 0.0) {
            return true;
        }
        for (std::size_t edge = 0; edge < 3; ++edge) {
            if (point_segment_distance_squared(point, cell.vertices[edge],
                                               cell.vertices[(edge + 1) % 3]) <=
                kEpsilon * kEpsilon) {
                return true;
            }
        }
        return false;
    }

    static void clip_lower(double coefficient, double constant, double &low,
                           double &high) noexcept {
        // coefficient * t + constant >= 0
        if (coefficient > 0.0) {
            low = std::max(low, -constant / coefficient);
        } else if (coefficient < 0.0) {
            high = std::min(high, -constant / coefficient);
        } else if (constant < 0.0) {
            low = 1.0;
            high = 0.0;
        }
    }

    static void append_circle_interval(std::vector<std::pair<double, double>> &intervals,
                                       Vec2 start, Vec2 delta, Vec2 centre) noexcept {
        const double dx = delta.x;
        const double dy = delta.y;
        const double px = static_cast<double>(start.x) - centre.x;
        const double py = static_cast<double>(start.y) - centre.y;
        const double a = dx * dx + dy * dy;
        const double b = 2.0 * (px * dx + py * dy);
        const double c = px * px + py * py - kEpsilon * kEpsilon;
        if (a == 0.0) {
            if (c <= 0.0) {
                intervals.emplace_back(0.0, 1.0);
            }
            return;
        }
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
            return;
        }
        const double root = std::sqrt(discriminant);
        const double low = std::max(0.0, (-b - root) / (2.0 * a));
        const double high = std::min(1.0, (-b + root) / (2.0 * a));
        if (low <= high) {
            intervals.emplace_back(low, high);
        }
    }

    static void append_cell_intervals(std::vector<std::pair<double, double>> &intervals,
                                      const NavMesh::Impl::Cell &cell, Vec2 start,
                                      Vec2 end) noexcept {
        const Vec2 delta = end - start;
        double low = 0.0;
        double high = 1.0;
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const Vec2 a = cell.vertices[edge];
            const Vec2 b = cell.vertices[(edge + 1) % 3];
            const Vec2 direction = b - a;
            clip_lower(cross(direction, delta), cross(direction, start - a), low, high);
        }
        if (low <= high) {
            intervals.emplace_back(low, high);
        }

        for (std::size_t edge = 0; edge < 3; ++edge) {
            const Vec2 a = cell.vertices[edge];
            const Vec2 b = cell.vertices[(edge + 1) % 3];
            const Vec2 direction = b - a;
            const double edge_length_squared = dot(direction, direction);
            double rectangle_low = 0.0;
            double rectangle_high = 1.0;
            clip_lower(dot(delta, direction), dot(start - a, direction), rectangle_low,
                       rectangle_high);
            clip_lower(-dot(delta, direction), edge_length_squared - dot(start - a, direction),
                       rectangle_low, rectangle_high);
            const double width = kEpsilon * std::sqrt(edge_length_squared);
            clip_lower(-cross(direction, delta), width - cross(direction, start - a), rectangle_low,
                       rectangle_high);
            clip_lower(cross(direction, delta), width + cross(direction, start - a), rectangle_low,
                       rectangle_high);
            if (rectangle_low <= rectangle_high) {
                intervals.emplace_back(rectangle_low, rectangle_high);
            }
            append_circle_interval(intervals, start, delta, a);
            append_circle_interval(intervals, start, delta, b);
        }
    }

    [[nodiscard]] static bool segment_contained(const NavMesh::Impl &impl, Vec2 start, Vec2 end,
                                                int component = -1) {
        if (start == end) {
            if (component < 0) {
                for (const auto &cell : impl.cells) {
                    if (cell_contains(cell, start)) {
                        return true;
                    }
                }
            } else {
                for (const auto &cell : impl.cells) {
                    if (cell.component == component && cell_contains(cell, start)) {
                        return true;
                    }
                }
            }
            return false;
        }

        std::vector<std::pair<double, double>> intervals;
        for (const auto &cell : impl.cells) {
            if (component < 0 || cell.component == component) {
                append_cell_intervals(intervals, cell, start, end);
            }
        }
        if (intervals.empty()) {
            return false;
        }
        std::sort(intervals.begin(), intervals.end());
        double covered = 0.0;
        constexpr double rounding_slack = 1.0e-12;
        for (const auto &[low, high] : intervals) {
            if (low > covered + rounding_slack) {
                return false;
            }
            covered = std::max(covered, high);
            if (covered >= 1.0 - rounding_slack) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] static std::vector<int> containing_components(const NavMesh::Impl &impl,
                                                                Vec2 point) {
        std::vector<int> result;
        for (const auto &cell : impl.cells) {
            if (cell_contains(cell, point) &&
                std::find(result.begin(), result.end(), cell.component) == result.end()) {
                result.push_back(cell.component);
            }
        }
        return result;
    }

    [[nodiscard]] static std::vector<Vec2> component_nodes(const NavMesh::Impl &impl, int component,
                                                           Vec2 start, Vec2 goal) {
        std::vector<Vec2> nodes{start, goal};
        for (const auto &cell : impl.cells) {
            if (cell.component != component) {
                continue;
            }
            for (const Vec2 vertex : cell.vertices) {
                if (std::find(nodes.begin(), nodes.end(), vertex) == nodes.end()) {
                    nodes.push_back(vertex);
                }
            }
        }
        return nodes;
    }

    [[nodiscard]] static Result<Path> path_in_component(const NavMesh::Impl &impl, int component,
                                                        Vec2 start, Vec2 goal) {
        const std::vector<Vec2> nodes = component_nodes(impl, component, start, goal);
        const std::size_t count = nodes.size();
        const double infinity = std::numeric_limits<double>::infinity();
        std::vector<double> distances(count, infinity);
        std::vector<std::size_t> previous(count, count);
        std::vector<bool> visited(count, false);
        distances[0] = 0.0;

        for (std::size_t iteration = 0; iteration < count; ++iteration) {
            std::size_t current = count;
            for (std::size_t node = 0; node < count; ++node) {
                if (!visited[node] && (current == count || distances[node] < distances[current])) {
                    current = node;
                }
            }
            if (current == count || !std::isfinite(distances[current])) {
                break;
            }
            if (current == 1) {
                break;
            }
            visited[current] = true;
            for (std::size_t next = 0; next < count; ++next) {
                if (visited[next] || next == current ||
                    !segment_contained(impl, nodes[current], nodes[next], component)) {
                    continue;
                }
                const double candidate =
                    distances[current] + std::sqrt(distance_squared(nodes[current], nodes[next]));
                if (candidate < distances[next]) {
                    distances[next] = candidate;
                    previous[next] = current;
                }
            }
        }
        if (!std::isfinite(distances[1])) {
            return std::unexpected(Error{ErrorCode::NoPath, "mesh cells are disconnected"});
        }

        Path path;
        for (std::size_t node = 1; node != count; node = previous[node]) {
            path.points.push_back(nodes[node]);
            if (node == 0) {
                break;
            }
        }
        std::reverse(path.points.begin(), path.points.end());
        return path;
    }
};

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl)) {}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles) {
    for (const Polygon &triangle : triangles) {
        for (const Vec2 vertex : triangle.vertices) {
            if (!finite(vertex)) {
                return std::unexpected(invalid_argument("mesh vertex is not finite"));
            }
        }
    }
    if (triangles.empty()) {
        return std::unexpected(invalid_mesh("mesh has no triangles"));
    }

    auto impl = std::make_shared<Impl>();
    impl->cells.reserve(triangles.size());
    for (const Polygon &triangle : triangles) {
        if (triangle.vertices.size() != 3) {
            return std::unexpected(invalid_mesh("mesh cell is not a triangle"));
        }
        const std::array<Vec2, 3> vertices{triangle.vertices[0], triangle.vertices[1],
                                           triangle.vertices[2]};
        if (orient(vertices[0], vertices[1], vertices[2]) <= kAreaEpsilon) {
            return std::unexpected(invalid_mesh("mesh triangle is clockwise or degenerate"));
        }
        impl->cells.push_back({vertices});
    }

    std::vector<EdgeRef> edges;
    edges.reserve(impl->cells.size() * 3);
    for (std::size_t cell = 0; cell < impl->cells.size(); ++cell) {
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const auto &vertices = impl->cells[cell].vertices;
            edges.push_back({cell, edge, vertices[edge], vertices[(edge + 1) % 3]});
        }
    }

    for (std::size_t first = 0; first < impl->cells.size(); ++first) {
        for (std::size_t second = first + 1; second < impl->cells.size(); ++second) {
            const auto &a = impl->cells[first].vertices;
            const auto &b = impl->cells[second].vertices;
            for (const Vec2 vertex : a) {
                if (strictly_in_triangle(vertex, b) || strictly_in_triangle(vertex, a)) {
                    return std::unexpected(invalid_mesh("mesh triangle interiors overlap"));
                }
            }
            for (const Vec2 vertex : b) {
                if (strictly_in_triangle(vertex, a)) {
                    return std::unexpected(invalid_mesh("mesh triangle interiors overlap"));
                }
            }
            for (std::size_t edge_a = 0; edge_a < 3; ++edge_a) {
                for (std::size_t edge_b = 0; edge_b < 3; ++edge_b) {
                    if (proper_crossing(a[edge_a], a[(edge_a + 1) % 3], b[edge_b],
                                        b[(edge_b + 1) % 3])) {
                        return std::unexpected(invalid_mesh("mesh triangle interiors overlap"));
                    }
                }
            }
        }
    }

    for (std::size_t vertex_cell = 0; vertex_cell < impl->cells.size(); ++vertex_cell) {
        for (const Vec2 vertex : impl->cells[vertex_cell].vertices) {
            for (std::size_t edge_cell = 0; edge_cell < impl->cells.size(); ++edge_cell) {
                if (edge_cell == vertex_cell) {
                    continue;
                }
                const auto &other = impl->cells[edge_cell].vertices;
                for (std::size_t edge = 0; edge < 3; ++edge) {
                    const Vec2 a = other[edge];
                    const Vec2 b = other[(edge + 1) % 3];
                    if (!near(vertex, a) && !near(vertex, b) &&
                        point_segment_distance_squared(vertex, a, b) <= kEpsilon * kEpsilon) {
                        return std::unexpected(invalid_mesh("mesh has a T-junction"));
                    }
                }
            }
        }
    }

    std::vector<int> mates(edges.size(), -1);
    for (std::size_t first = 0; first < edges.size(); ++first) {
        for (std::size_t second = first + 1; second < edges.size(); ++second) {
            if (edges[first].cell == edges[second].cell ||
                !same_edge(edges[first], edges[second])) {
                continue;
            }
            if (mates[first] != -1 || mates[second] != -1) {
                return std::unexpected(invalid_mesh("mesh has a non-manifold edge"));
            }
            if (same_direction(edges[first], edges[second])) {
                return std::unexpected(invalid_mesh("mesh triangle interiors overlap"));
            }
            mates[first] = static_cast<int>(second);
            mates[second] = static_cast<int>(first);
            impl->cells[edges[first].cell].neighbours[edges[first].edge] =
                static_cast<int>(edges[second].cell);
            impl->cells[edges[second].cell].neighbours[edges[second].edge] =
                static_cast<int>(edges[first].cell);
        }
    }

    int next_component = 0;
    std::vector<std::size_t> pending;
    for (std::size_t start = 0; start < impl->cells.size(); ++start) {
        if (impl->cells[start].component != -1) {
            continue;
        }
        impl->cells[start].component = next_component++;
        pending.push_back(start);
        while (!pending.empty()) {
            const std::size_t cell = pending.back();
            pending.pop_back();
            for (const int neighbour : impl->cells[cell].neighbours) {
                if (neighbour >= 0 &&
                    impl->cells[static_cast<std::size_t>(neighbour)].component == -1) {
                    impl->cells[static_cast<std::size_t>(neighbour)].component =
                        impl->cells[cell].component;
                    pending.push_back(static_cast<std::size_t>(neighbour));
                }
            }
        }
    }
    return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept {
    if (!finite(point)) {
        return false;
    }
    for (const auto &cell : m_impl->cells) {
        if (MeshAccess::cell_contains(cell, point)) {
            return true;
        }
    }
    return false;
}

std::size_t NavMesh::cell_count() const noexcept {
    return m_impl->cells.size();
}

Result<Path> find_path(const NavMesh &mesh, Vec2 start, Vec2 goal) {
    if (!finite(start) || !finite(goal)) {
        return std::unexpected(invalid_argument("path endpoint is not finite"));
    }
    if (!mesh.contains(start) || !mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "path endpoint is outside the mesh"});
    }
    if (start == goal) {
        return Path{{start}};
    }

    const std::vector<int> start_components =
        MeshAccess::containing_components(*mesh.m_impl, start);
    const std::vector<int> goal_components = MeshAccess::containing_components(*mesh.m_impl, goal);
    Result<Path> best = std::unexpected(Error{ErrorCode::NoPath, "mesh cells are disconnected"});
    double best_length = std::numeric_limits<double>::infinity();
    for (const int component : start_components) {
        if (std::find(goal_components.begin(), goal_components.end(), component) ==
            goal_components.end()) {
            continue;
        }
        const auto candidate = MeshAccess::path_in_component(*mesh.m_impl, component, start, goal);
        if (!candidate) {
            continue;
        }
        double candidate_length = 0.0;
        for (std::size_t index = 1; index < candidate->points.size(); ++index) {
            candidate_length +=
                std::sqrt(distance_squared(candidate->points[index - 1], candidate->points[index]));
        }
        if (candidate_length < best_length) {
            best_length = candidate_length;
            best = *candidate;
        }
    }
    return best;
}

} // namespace vwmini
