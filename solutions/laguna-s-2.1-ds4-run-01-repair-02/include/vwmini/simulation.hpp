#pragma once

#include <vwmini/nav_mesh.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace vwmini {

/** Non-zero opaque identifier of an agent owned by one `Simulation`. */
struct AgentId {
    std::uint32_t value{};

    constexpr bool operator==(const AgentId&) const noexcept = default;
};

/** Current high-level movement state of an agent. */
enum class AgentStatus {
    Idle,    ///< No goal is assigned.
    Moving,  ///< A connected goal route is active.
    Reached, ///< The goal lies within the effective arrival radius.
    NoPath,  ///< The assigned in-mesh goal is disconnected.
};

/** Configuration validated when an agent is added. `arrival_radius == -1` uses `radius`. */
struct AgentConfig {
    Vec2 position{};
    float radius{0.25f};
    float max_speed{1.4f};
    std::optional<Vec2> goal{};
    float arrival_radius{-1.0f};
};

/** Value snapshot returned by `Simulation::agent`; mutating it cannot affect the simulation. */
struct AgentState {
    Vec2 position{};
    Vec2 velocity{};
    float radius{};
    float max_speed{};
    std::optional<Vec2> goal{};
    AgentStatus status{AgentStatus::Idle};
};

/** Single-threaded agent simulation over one immutable navigation mesh. */
class Simulation {
  public:
    /** Takes ownership of an immutable mesh value; all calls are single-threaded. */
    explicit Simulation(NavMesh mesh);
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    /** Adds a validated live agent and returns its non-zero id. */
    [[nodiscard]] Result<AgentId> add_agent(const AgentConfig& config);
    /** Removes a live agent; later use of its id returns `NotFound`. */
    [[nodiscard]] Result<void> remove_agent(AgentId id);
    /** Replaces a live agent's goal and route; `-1` arrival radius uses agent radius. */
    [[nodiscard]] Result<void> set_goal(AgentId id, Vec2 goal, float arrival_radius = -1.0f);
    /** Clears a live agent's goal and makes it idle with zero velocity. */
    [[nodiscard]] Result<void> clear_goal(AgentId id);
    /** Advances the single-threaded simulation by a finite non-negative duration. */
    [[nodiscard]] Result<void> step(float seconds);
    /** Returns a copy of a live agent state, or empty for an unknown/removed id. */
    [[nodiscard]] std::optional<AgentState> agent(AgentId id) const noexcept;
    /** Returns the exact number of live agents. */
    [[nodiscard]] std::size_t agent_count() const noexcept;

  private:
    struct Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace vwmini
