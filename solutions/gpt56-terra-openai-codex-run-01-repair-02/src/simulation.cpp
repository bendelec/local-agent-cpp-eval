#include <vwmini/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace vwmini {
namespace {

constexpr double kWaypointEpsilon = 1.0e-4;
constexpr float kMaximumSubstep = 0.02F;
constexpr std::size_t kMaximumFineSubsteps = 1024;
constexpr double kSeparationSlack = 1.0e-4;

struct DVec {
    double x{};
    double y{};
};

[[nodiscard]] DVec operator+(DVec left, DVec right) noexcept {
    return {left.x + right.x, left.y + right.y};
}

[[nodiscard]] DVec operator-(DVec left, DVec right) noexcept {
    return {left.x - right.x, left.y - right.y};
}

[[nodiscard]] DVec operator*(DVec value, double scalar) noexcept {
    return {value.x * scalar, value.y * scalar};
}

[[nodiscard]] DVec as_double(Vec2 value) noexcept {
    return {value.x, value.y};
}

[[nodiscard]] bool finite(DVec value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

[[nodiscard]] bool finite(Vec2 value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

[[nodiscard]] double dot(DVec left, DVec right) noexcept {
    return left.x * right.x + left.y * right.y;
}

[[nodiscard]] double magnitude(DVec value) noexcept {
    return std::hypot(value.x, value.y);
}

[[nodiscard]] DVec normalized(DVec value) noexcept {
    const double value_magnitude = magnitude(value);
    return value_magnitude == 0.0 || !std::isfinite(value_magnitude)
               ? DVec{}
               : value * (1.0 / value_magnitude);
}

[[nodiscard]] DVec perpendicular(DVec value) noexcept {
    return {-value.y, value.x};
}

[[nodiscard]] DVec bounded(DVec velocity, double maximum_speed) noexcept {
    const double velocity_magnitude = magnitude(velocity);
    if (!std::isfinite(velocity_magnitude) || velocity_magnitude == 0.0) {
        return {};
    }
    return velocity_magnitude > maximum_speed ? velocity * (maximum_speed / velocity_magnitude)
                                              : velocity;
}

[[nodiscard]] double distance(Vec2 left, Vec2 right) noexcept {
    return magnitude(as_double(left) - as_double(right));
}

[[nodiscard]] bool valid_arrival_radius(float value) noexcept {
    return std::isfinite(value) && (value >= 0.0F || value == -1.0F);
}

[[nodiscard]] Error invalid_argument(const char *message) {
    return {ErrorCode::InvalidArgument, message};
}

[[nodiscard]] Error not_found() {
    return {ErrorCode::NotFound, "agent id was not found"};
}

} // namespace

struct Simulation::Impl {
    struct Agent {
        AgentId id;
        Vec2 position;
        Vec2 velocity{};
        float radius;
        float max_speed;
        std::optional<Vec2> goal;
        float arrival_radius{};
        AgentStatus status{AgentStatus::Idle};
        std::vector<Vec2> route;
        std::size_t waypoint{};
    };

    NavMesh mesh;
    std::vector<Agent> agents;
    std::uint32_t next_id{1};

    explicit Impl(NavMesh input_mesh) : mesh(std::move(input_mesh)) {}

    [[nodiscard]] Agent *find(AgentId id) noexcept {
        const auto found = std::find_if(agents.begin(), agents.end(),
                                        [id](const Agent &agent) { return agent.id == id; });
        return found == agents.end() ? nullptr : &*found;
    }

    [[nodiscard]] const Agent *find(AgentId id) const noexcept {
        const auto found = std::find_if(agents.begin(), agents.end(),
                                        [id](const Agent &agent) { return agent.id == id; });
        return found == agents.end() ? nullptr : &*found;
    }
};

struct SimulationAccess {
    struct PlannedMotion {
        Vec2 endpoint;
        DVec displacement;
        Vec2 velocity;
        double route_progress{};
    };

    static void stop(Simulation::Impl::Agent &agent) noexcept {
        agent.velocity = {};
    }

    static void set_reached(Simulation::Impl::Agent &agent) noexcept {
        agent.status = AgentStatus::Reached;
        stop(agent);
        agent.route.clear();
        agent.waypoint = 0;
    }

    static void set_no_path(Simulation::Impl::Agent &agent) noexcept {
        agent.status = AgentStatus::NoPath;
        stop(agent);
        agent.route.clear();
        agent.waypoint = 0;
    }

    static void apply_route(Simulation::Impl::Agent &agent, const Path &path) noexcept {
        agent.route = path.points;
        agent.waypoint = agent.route.size() > 1 ? 1 : 0;
        agent.status = AgentStatus::Moving;
    }

    [[nodiscard]] static Result<void> assign_goal(Simulation::Impl &impl,
                                                  Simulation::Impl::Agent &agent, Vec2 goal,
                                                  float arrival_radius) {
        const float effective_radius = arrival_radius == -1.0F ? agent.radius : arrival_radius;
        const auto route = find_path(impl.mesh, agent.position, goal);
        agent.goal = goal;
        agent.arrival_radius = effective_radius;
        if (!route) {
            if (route.error().code == ErrorCode::NoPath) {
                set_no_path(agent);
                return {};
            }
            return std::unexpected(route.error());
        }
        if (distance(agent.position, goal) <= effective_radius) {
            set_reached(agent);
            return {};
        }
        apply_route(agent, *route);
        return {};
    }

    [[nodiscard]] static bool direct_motion_is_contained(const NavMesh &mesh, Vec2 start,
                                                         Vec2 end) {
        if (start == end) {
            return mesh.contains(start);
        }
        const auto path = find_path(mesh, start, end);
        return path && path->points.size() == 2 && path->points[0] == start &&
               path->points[1] == end;
    }

    // Float storage can round a legal double displacement to a one-ULP overspeed. Retreating
    // toward the original coordinate chooses the nearest representable endpoint within budget.
    [[nodiscard]] static Vec2 stored_endpoint(Vec2 start, DVec intended_displacement,
                                              double movement_budget) noexcept {
        const DVec target = as_double(start) + intended_displacement;
        if (!finite(target)) {
            return start;
        }
        Vec2 endpoint{static_cast<float>(target.x), static_cast<float>(target.y)};
        if (!finite(endpoint)) {
            return start;
        }

        for (int attempt = 0; attempt < 8; ++attempt) {
            const DVec observed = as_double(endpoint) - as_double(start);
            if (magnitude(observed) <= movement_budget) {
                return endpoint;
            }
            if (std::abs(observed.x) >= std::abs(observed.y) && endpoint.x != start.x) {
                endpoint.x = std::nextafter(endpoint.x, start.x);
            } else if (endpoint.y != start.y) {
                endpoint.y = std::nextafter(endpoint.y, start.y);
            } else {
                return start;
            }
        }
        return start;
    }

    [[nodiscard]] static PlannedMotion committed_motion(Vec2 start, Vec2 endpoint, double elapsed,
                                                        float maximum_speed) {
        const DVec displacement = as_double(endpoint) - as_double(start);
        const double observed_speed = magnitude(displacement) / elapsed;
        if (!finite(displacement) || !std::isfinite(observed_speed) ||
            observed_speed > static_cast<double>(maximum_speed)) {
            return {start, {}, {}, 0.0};
        }
        const Vec2 velocity{static_cast<float>(displacement.x / elapsed),
                            static_cast<float>(displacement.y / elapsed)};
        if (!finite(velocity)) {
            return {start, {}, {}, 0.0};
        }
        return {endpoint, displacement, velocity, 0.0};
    }

    [[nodiscard]] static bool route_continues_from(const Simulation::Impl &impl,
                                                   const Simulation::Impl::Agent &agent,
                                                   Vec2 endpoint) {
        if (endpoint == agent.position || !agent.goal) {
            return true;
        }
        // This reuses routing's complete-segment/containment seam. A tolerance-only endpoint
        // that cannot form a route is not a valid stored motion endpoint.
        return find_path(impl.mesh, endpoint, *agent.goal).has_value();
    }

    // This is the mesh-feasible displacement. It first applies float endpoint rounding and
    // speed budgeting, then limits the complete segment against the immutable mesh while
    // retaining a route-continuing stored endpoint.
    [[nodiscard]] static PlannedMotion mesh_feasible_motion(const Simulation::Impl &impl,
                                                            const Simulation::Impl::Agent &agent,
                                                            DVec intended_velocity,
                                                            double elapsed) {
        intended_velocity = bounded(intended_velocity, agent.max_speed);
        const DVec intended_displacement = intended_velocity * elapsed;
        const double movement_budget = static_cast<double>(agent.max_speed) * elapsed;
        Vec2 endpoint = stored_endpoint(agent.position, intended_displacement, movement_budget);
        if (direct_motion_is_contained(impl.mesh, agent.position, endpoint) &&
            route_continues_from(impl, agent, endpoint)) {
            return committed_motion(agent.position, endpoint, elapsed, agent.max_speed);
        }

        double low = 0.0;
        double high = 1.0;
        for (int iteration = 0; iteration < 24; ++iteration) {
            const double middle = (low + high) * 0.5;
            endpoint =
                stored_endpoint(agent.position, intended_displacement * middle, movement_budget);
            if (direct_motion_is_contained(impl.mesh, agent.position, endpoint) &&
                route_continues_from(impl, agent, endpoint)) {
                low = middle;
            } else {
                high = middle;
            }
        }
        endpoint = stored_endpoint(agent.position, intended_displacement * low, movement_budget);
        return committed_motion(agent.position, endpoint, elapsed, agent.max_speed);
    }

    [[nodiscard]] static DVec intended_velocity(const Simulation::Impl::Agent &agent,
                                                double elapsed) noexcept {
        if (agent.status != AgentStatus::Moving || agent.waypoint >= agent.route.size()) {
            return {};
        }
        const DVec to_waypoint = as_double(agent.route[agent.waypoint]) - as_double(agent.position);
        const double remaining_distance = magnitude(to_waypoint);
        if (remaining_distance == 0.0 || !std::isfinite(remaining_distance)) {
            return {};
        }
        const double speed =
            std::min(static_cast<double>(agent.max_speed), remaining_distance / elapsed);
        return normalized(to_waypoint) * speed;
    }

    [[nodiscard]] static DVec separation_direction(const Simulation::Impl::Agent &agent,
                                                   const Simulation::Impl::Agent &other,
                                                   Vec2 position, Vec2 other_position) noexcept {
        const DVec difference = as_double(position) - as_double(other_position);
        if (magnitude(difference) > kWaypointEpsilon) {
            return normalized(difference);
        }
        return agent.id.value < other.id.value ? DVec{-1.0, 0.0} : DVec{1.0, 0.0};
    }

    static void append_candidate_velocity(std::vector<DVec> &velocities, DVec velocity) {
        constexpr double duplicate_tolerance = 1.0e-12;
        for (const DVec existing : velocities) {
            if (magnitude(existing - velocity) <= duplicate_tolerance) {
                return;
            }
        }
        velocities.push_back(velocity);
    }

    [[nodiscard]] static DVec avoidance_velocity(const Simulation::Impl &impl, std::size_t index,
                                                 const std::vector<Vec2> &positions,
                                                 const std::vector<Vec2> &velocities,
                                                 DVec desired) noexcept {
        const auto &agent = impl.agents[index];
        DVec steered = desired;
        constexpr double horizon = 0.75;
        for (std::size_t other_index = 0; other_index < impl.agents.size(); ++other_index) {
            if (other_index == index) {
                continue;
            }
            const auto &other = impl.agents[other_index];
            const DVec relative_position =
                as_double(positions[index]) - as_double(positions[other_index]);
            const DVec relative_velocity = desired - as_double(velocities[other_index]);
            const double relative_speed_squared = dot(relative_velocity, relative_velocity);
            const double closest_time =
                relative_speed_squared == 0.0
                    ? 0.0
                    : std::clamp(-dot(relative_position, relative_velocity) /
                                     relative_speed_squared,
                                 0.0, horizon);
            const double closest_distance =
                magnitude(relative_position + relative_velocity * closest_time);
            const double caution_distance = agent.radius + other.radius + 0.1;
            if (closest_distance >= caution_distance) {
                continue;
            }
            const DVec away =
                separation_direction(agent, other, positions[index], positions[other_index]);
            DVec tangent = perpendicular(away);
            if (dot(tangent, desired) < 0.0) {
                tangent = tangent * -1.0;
            }
            const double urgency =
                std::clamp((caution_distance - closest_distance) / caution_distance, 0.0, 1.0);
            steered = steered + tangent * (agent.max_speed * (0.45 + 0.35 * urgency)) +
                      away * (agent.max_speed * 0.15 * urgency);
        }
        return bounded(steered, agent.max_speed);
    }

    [[nodiscard]] static std::vector<PlannedMotion>
    motion_options(const Simulation::Impl &impl, std::size_t index,
                   const std::vector<Vec2> &positions, const std::vector<Vec2> &velocities,
                   double elapsed) {
        const auto &agent = impl.agents[index];
        if (agent.status != AgentStatus::Moving) {
            return {{{agent.position, {}, {}, 0.0}}};
        }
        const DVec desired = intended_velocity(agent, elapsed);
        const DVec preferred = avoidance_velocity(impl, index, positions, velocities, desired);
        std::vector<DVec> candidate_velocities;
        candidate_velocities.reserve(8);
        append_candidate_velocity(candidate_velocities, preferred);
        append_candidate_velocity(candidate_velocities, desired);
        append_candidate_velocity(candidate_velocities, preferred * 0.5);
        append_candidate_velocity(candidate_velocities, desired * 0.5);

        std::size_t nearest = impl.agents.size();
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (std::size_t other_index = 0; other_index < impl.agents.size(); ++other_index) {
            if (other_index == index) {
                continue;
            }
            const double candidate_distance = distance(positions[index], positions[other_index]);
            if (candidate_distance < nearest_distance) {
                nearest = other_index;
                nearest_distance = candidate_distance;
            }
        }
        if (nearest != impl.agents.size()) {
            const auto &other = impl.agents[nearest];
            const DVec away =
                separation_direction(agent, other, positions[index], positions[nearest]);
            const DVec tangent = perpendicular(away);
            const double speed = agent.max_speed;
            append_candidate_velocity(candidate_velocities,
                                      bounded(preferred + tangent * (speed * 0.75), speed));
            append_candidate_velocity(candidate_velocities,
                                      bounded(preferred - tangent * (speed * 0.75), speed));
            append_candidate_velocity(candidate_velocities, away * speed);
        }
        append_candidate_velocity(candidate_velocities, {});

        const DVec route_direction =
            normalized(agent.waypoint < agent.route.size()
                           ? as_double(agent.route[agent.waypoint]) - as_double(agent.position)
                           : DVec{});
        std::vector<PlannedMotion> result;
        result.reserve(candidate_velocities.size());
        for (const DVec velocity : candidate_velocities) {
            PlannedMotion motion = mesh_feasible_motion(impl, agent, velocity, elapsed);
            motion.route_progress = dot(motion.displacement, route_direction);
            result.push_back(motion);
        }
        return result;
    }

    [[nodiscard]] static bool pair_is_separated(Vec2 first_position, const PlannedMotion &first,
                                                Vec2 second_position, const PlannedMotion &second,
                                                double required) noexcept {
        const DVec initial = as_double(first_position) - as_double(second_position);
        const DVec relative_displacement = first.displacement - second.displacement;
        const double displacement_squared = dot(relative_displacement, relative_displacement);
        double fraction = 0.0;
        if (displacement_squared > 0.0) {
            fraction =
                std::clamp(-dot(initial, relative_displacement) / displacement_squared, 0.0, 1.0);
        }
        const double closest = magnitude(initial + relative_displacement * fraction);
        return closest >= std::max(0.0, required - kSeparationSlack);
    }

    [[nodiscard]] static bool
    compatible_with_others(const Simulation::Impl &impl, const std::vector<Vec2> &positions,
                           const std::vector<std::vector<PlannedMotion>> &options,
                           const std::vector<std::size_t> &selected, std::size_t first,
                           std::size_t first_option, std::size_t second,
                           std::size_t second_option) noexcept {
        const PlannedMotion &first_motion = options[first][first_option];
        const PlannedMotion &second_motion = options[second][second_option];
        const double pair_radius = impl.agents[first].radius + impl.agents[second].radius;
        if (!pair_is_separated(positions[first], first_motion, positions[second], second_motion,
                               pair_radius)) {
            return false;
        }
        for (std::size_t other = 0; other < impl.agents.size(); ++other) {
            if (other == first || other == second) {
                continue;
            }
            const PlannedMotion &other_motion = options[other][selected[other]];
            if (!pair_is_separated(positions[first], first_motion, positions[other], other_motion,
                                   impl.agents[first].radius + impl.agents[other].radius) ||
                !pair_is_separated(positions[second], second_motion, positions[other], other_motion,
                                   impl.agents[second].radius + impl.agents[other].radius)) {
                return false;
            }
        }
        return true;
    }

    static void select_separated_motions(const Simulation::Impl &impl,
                                         const std::vector<Vec2> &positions,
                                         const std::vector<std::vector<PlannedMotion>> &options,
                                         std::vector<std::size_t> &selected) {
        // Candidate endpoints are all produced from one immutable snapshot. This fixed pass
        // ordering chooses local compatible options without changing any agent position.
        for (int pass = 0; pass < 6; ++pass) {
            bool changed = false;
            for (std::size_t first = 0; first < impl.agents.size(); ++first) {
                for (std::size_t second = first + 1; second < impl.agents.size(); ++second) {
                    const auto &current_first = options[first][selected[first]];
                    const auto &current_second = options[second][selected[second]];
                    const double required = impl.agents[first].radius + impl.agents[second].radius;
                    if (pair_is_separated(positions[first], current_first, positions[second],
                                          current_second, required)) {
                        continue;
                    }

                    std::size_t best_first = selected[first];
                    std::size_t best_second = selected[second];
                    double best_progress = -std::numeric_limits<double>::infinity();
                    for (std::size_t first_option = 0; first_option < options[first].size();
                         ++first_option) {
                        for (std::size_t second_option = 0; second_option < options[second].size();
                             ++second_option) {
                            if (!compatible_with_others(impl, positions, options, selected, first,
                                                        first_option, second, second_option)) {
                                continue;
                            }
                            const bool first_has_priority =
                                impl.agents[first].id.value < impl.agents[second].id.value;
                            const double priority_progress =
                                first_has_priority ? options[first][first_option].route_progress
                                                   : options[second][second_option].route_progress;
                            const double progress = priority_progress * 1000.0 +
                                                    options[first][first_option].route_progress +
                                                    options[second][second_option].route_progress;
                            if (progress > best_progress) {
                                best_progress = progress;
                                best_first = first_option;
                                best_second = second_option;
                            }
                        }
                    }
                    if (best_progress != -std::numeric_limits<double>::infinity() &&
                        (best_first != selected[first] || best_second != selected[second])) {
                        selected[first] = best_first;
                        selected[second] = best_second;
                        changed = true;
                    }
                }
            }
            if (!changed) {
                return;
            }
        }
    }

    static void update_route_status(const NavMesh &mesh, Simulation::Impl::Agent &agent) {
        while (agent.waypoint < agent.route.size()) {
            if (agent.position == agent.route[agent.waypoint]) {
                ++agent.waypoint;
            } else if (agent.waypoint + 1 < agent.route.size() &&
                       direct_motion_is_contained(mesh, agent.position,
                                                  agent.route[agent.waypoint + 1])) {
                // A locally tangential motion can legally pass a portal vertex without landing
                // on its exact float value. Continue to the next directly visible route point.
                ++agent.waypoint;
            } else {
                break;
            }
        }
        if (agent.goal && distance(agent.position, *agent.goal) <= agent.arrival_radius) {
            set_reached(agent);
            return;
        }
        if (agent.goal &&
            (agent.waypoint >= agent.route.size() ||
             !direct_motion_is_contained(mesh, agent.position, agent.route[agent.waypoint]))) {
            // Tangential motion may leave the retained portal sequence behind while still
            // remaining route-connected. Refresh before the next snapshot, never converting a
            // reachable goal into NoPath because a local motion was clipped.
            const auto refreshed = find_path(mesh, agent.position, *agent.goal);
            if (refreshed) {
                apply_route(agent, *refreshed);
            }
        }
    }

    static void advance_one_substep(Simulation::Impl &impl, float seconds) {
        const double elapsed = seconds;
        const std::size_t count = impl.agents.size();
        std::vector<Vec2> positions;
        std::vector<Vec2> velocities;
        positions.reserve(count);
        velocities.reserve(count);
        for (const auto &agent : impl.agents) {
            positions.push_back(agent.position);
            velocities.push_back(agent.velocity);
        }

        std::vector<std::vector<PlannedMotion>> options;
        options.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            options.push_back(motion_options(impl, index, positions, velocities, elapsed));
        }
        std::vector<std::size_t> selected(count, 0);
        select_separated_motions(impl, positions, options, selected);

        for (std::size_t index = 0; index < count; ++index) {
            auto &agent = impl.agents[index];
            if (agent.status != AgentStatus::Moving) {
                stop(agent);
                continue;
            }
            const PlannedMotion &motion = options[index][selected[index]];
            agent.position = motion.endpoint;
            agent.velocity = motion.velocity;
            update_route_status(impl.mesh, agent);
        }
    }

    [[nodiscard]] static bool has_moving_agents(const Simulation::Impl &impl) noexcept {
        return std::any_of(impl.agents.begin(), impl.agents.end(),
                           [](const Simulation::Impl::Agent &agent) {
                               return agent.status == AgentStatus::Moving;
                           });
    }
};

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh))) {}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation &&) noexcept = default;
Simulation &Simulation::operator=(Simulation &&) noexcept = default;

Result<AgentId> Simulation::add_agent(const AgentConfig &config) {
    if (!finite(config.position) || !std::isfinite(config.radius) ||
        !std::isfinite(config.max_speed) || !std::isfinite(config.arrival_radius) ||
        (config.goal && !finite(*config.goal)) || config.radius <= 0.0F ||
        config.max_speed <= 0.0F || !valid_arrival_radius(config.arrival_radius)) {
        return std::unexpected(invalid_argument("agent configuration is invalid"));
    }
    if (!m_impl->mesh.contains(config.position) ||
        (config.goal && !m_impl->mesh.contains(*config.goal))) {
        return std::unexpected(
            Error{ErrorCode::OutsideMesh, "agent position or goal is outside the mesh"});
    }
    if (m_impl->next_id == 0) {
        return std::unexpected(invalid_argument("agent identifier space is exhausted"));
    }

    Impl::Agent candidate{AgentId{m_impl->next_id++},
                          config.position,
                          {},
                          config.radius,
                          config.max_speed,
                          std::nullopt,
                          0.0F,
                          AgentStatus::Idle,
                          {},
                          0};
    if (config.goal) {
        const auto assigned =
            SimulationAccess::assign_goal(*m_impl, candidate, *config.goal, config.arrival_radius);
        if (!assigned) {
            return std::unexpected(assigned.error());
        }
    }
    m_impl->agents.push_back(std::move(candidate));
    return m_impl->agents.back().id;
}

Result<void> Simulation::remove_agent(AgentId id) {
    const auto found = std::find_if(m_impl->agents.begin(), m_impl->agents.end(),
                                    [id](const Impl::Agent &agent) { return agent.id == id; });
    if (found == m_impl->agents.end()) {
        return std::unexpected(not_found());
    }
    m_impl->agents.erase(found);
    return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius) {
    Impl::Agent *target = m_impl->find(id);
    if (!target) {
        return std::unexpected(not_found());
    }
    if (!finite(goal) || !valid_arrival_radius(arrival_radius)) {
        return std::unexpected(invalid_argument("goal or arrival radius is invalid"));
    }
    if (!m_impl->mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal is outside the mesh"});
    }

    const float effective_radius = arrival_radius == -1.0F ? target->radius : arrival_radius;
    const auto route = find_path(m_impl->mesh, target->position, goal);
    if (!route && route.error().code != ErrorCode::NoPath) {
        return std::unexpected(route.error());
    }
    target->goal = goal;
    target->arrival_radius = effective_radius;
    if (!route) {
        SimulationAccess::set_no_path(*target);
    } else if (distance(target->position, goal) <= effective_radius) {
        SimulationAccess::set_reached(*target);
    } else {
        SimulationAccess::apply_route(*target, *route);
    }
    return {};
}

Result<void> Simulation::clear_goal(AgentId id) {
    Impl::Agent *target = m_impl->find(id);
    if (!target) {
        return std::unexpected(not_found());
    }
    target->goal.reset();
    target->arrival_radius = 0.0F;
    target->route.clear();
    target->waypoint = 0;
    target->status = AgentStatus::Idle;
    SimulationAccess::stop(*target);
    return {};
}

Result<void> Simulation::step(float seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0F) {
        return std::unexpected(invalid_argument("step duration is invalid"));
    }
    if (seconds == 0.0F) {
        return {};
    }

    float remaining = seconds;
    std::size_t fine_substeps = 0;
    while (remaining > 0.0F && SimulationAccess::has_moving_agents(*m_impl)) {
        float substep = std::min(remaining, kMaximumSubstep);
        const float after_substep = remaining - substep;
        if (fine_substeps == kMaximumFineSubsteps || after_substep >= remaining) {
            // A float can no longer represent removal of a small step. One final, bounded
            // integration consumes the remaining elapsed time without an unbounded loop.
            substep = remaining;
            remaining = 0.0F;
        } else {
            remaining = after_substep;
            ++fine_substeps;
        }
        SimulationAccess::advance_one_substep(*m_impl, substep);
    }
    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept {
    const Impl::Agent *found = m_impl->find(id);
    if (!found) {
        return std::nullopt;
    }
    return AgentState{found->position,  found->velocity, found->radius,
                      found->max_speed, found->goal,     found->status};
}

std::size_t Simulation::agent_count() const noexcept {
    return m_impl->agents.size();
}

} // namespace vwmini
