# VWmini Public Interface Specification

The canonical headers in `include/vwmini/` are the fixed **public** C++23 contract.
This document explains that contract. Headers use namespace `vwmini`; all distances and
time values are `float` metres and seconds. `Result<T>` is `std::expected<T, Error>`.
The headers, not these repeated snippets, are authoritative. Implementations may add
private helpers but may not change public signatures, defaults, enum values, include
paths, or semantics.

```cpp
// include/vwmini/geometry.hpp
#pragma once
#include <expected>
#include <string>
#include <vector>

namespace vwmini {
struct Vec2 {
    float x{};
    float y{};
    constexpr bool operator==(const Vec2&) const noexcept = default;
};
[[nodiscard]] constexpr Vec2 operator+(Vec2 a, Vec2 b) noexcept
{ return {a.x + b.x, a.y + b.y}; }
[[nodiscard]] constexpr Vec2 operator-(Vec2 a, Vec2 b) noexcept
{ return {a.x - b.x, a.y - b.y}; }
[[nodiscard]] constexpr Vec2 operator*(Vec2 value, float scalar) noexcept
{ return {value.x * scalar, value.y * scalar}; }
[[nodiscard]] constexpr Vec2 operator*(float scalar, Vec2 value) noexcept
{ return value * scalar; }
[[nodiscard]] constexpr float dot(Vec2 a, Vec2 b) noexcept
{ return a.x * b.x + a.y * b.y; }
[[nodiscard]] constexpr float cross(Vec2 a, Vec2 b) noexcept
{ return a.x * b.y - a.y * b.x; }
[[nodiscard]] float length(Vec2 value) noexcept;
[[nodiscard]] Vec2 normalized(Vec2 value) noexcept; // zero input -> {0, 0}

struct Polygon { std::vector<Vec2> vertices; };
enum class ErrorCode { InvalidArgument, InvalidMesh, OutsideMesh, NoPath, NotFound };
struct Error { ErrorCode code{}; std::string message; };
template<class T> using Result = std::expected<T, Error>;

[[nodiscard]] Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon);
} // namespace vwmini
```

```cpp
// include/vwmini/nav_mesh.hpp
#pragma once
#include <vwmini/geometry.hpp>
#include <cstddef>
#include <memory>
#include <vector>

namespace vwmini {
struct Path;
class NavMesh {
public:
    NavMesh() = delete;
    [[nodiscard]] static Result<NavMesh> create(std::vector<Polygon> triangles);
    [[nodiscard]] bool contains(Vec2 point) const noexcept;
    [[nodiscard]] std::size_t cell_count() const noexcept;
    // Private representation is implementation-defined.
};

struct Path { std::vector<Vec2> points; };
[[nodiscard]] Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal);
} // namespace vwmini
```

```cpp
// include/vwmini/simulation.hpp
#pragma once
#include <vwmini/nav_mesh.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace vwmini {
struct AgentId {
    std::uint32_t value{}; // zero is invalid; generated ids are non-zero.
    constexpr bool operator==(const AgentId&) const noexcept = default;
};

enum class AgentStatus { Idle, Moving, Reached, NoPath };
struct AgentConfig {
    Vec2 position{};
    float radius{0.25f};
    float max_speed{1.4f};
    std::optional<Vec2> goal{};
    float arrival_radius{-1.0f}; // -1 only: use radius
};
struct AgentState {
    Vec2 position{};
    Vec2 velocity{};
    float radius{};
    float max_speed{};
    std::optional<Vec2> goal{};
    AgentStatus status{AgentStatus::Idle};
};

class Simulation {
public:
    explicit Simulation(NavMesh mesh);
    ~Simulation();
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;
    [[nodiscard]] Result<AgentId> add_agent(const AgentConfig& config);
    [[nodiscard]] Result<void> remove_agent(AgentId id);
    [[nodiscard]] Result<void> set_goal(AgentId id, Vec2 goal,
                                        float arrival_radius = -1.0f);
    [[nodiscard]] Result<void> clear_goal(AgentId id);
    [[nodiscard]] Result<void> step(float seconds);
    [[nodiscard]] std::optional<AgentState> agent(AgentId id) const noexcept;
    [[nodiscard]] std::size_t agent_count() const noexcept;
    // Private representation is implementation-defined.
};
} // namespace vwmini
```

## Value and lifetime contract

`NavMesh` is factory-created only and is a regular value type: copying an accepted mesh
preserves its immutable triangles. `Simulation` takes its own `NavMesh` value and is
move-only. `AgentId` is valid only in the `Simulation` that created it, until
`remove_agent` succeeds. `AgentState` and `Path` own their returned
values; no returned reference, pointer, iterator, or span observes internal storage.
All public calls are single-threaded: concurrent calls, including const calls while a
mutation runs, are outside the contract.

## Exact error rules

Non-finite scalar input returns `InvalidArgument`, including a non-finite coordinate in
a mesh triangle. Invalid finite polygon or mesh geometry, including an empty triangle
vector, returns `InvalidMesh`.
`find_path` returns `OutsideMesh` for a finite endpoint outside the mesh and `NoPath`
for disconnected endpoints. `add_agent` returns `OutsideMesh` for a valid finite
position outside the mesh. Agent methods return `NotFound` for an unknown or removed
id. Negative/non-finite `step` durations and invalid configuration scalars return
`InvalidArgument` without mutation; so does `add_agent` once the 32-bit id space is
exhausted — ids are never reused or wrapped (SIM-005 edge case).

`set_goal` and `add_agent` with an in-mesh but disconnected goal are not errors: they
succeed and place the agent in `AgentStatus::NoPath`. `-1.0f` is the only negative
arrival-radius sentinel; any other negative arrival radius is `InvalidArgument`.

## Compatibility rules

Do not change the supplied public declarations, signatures, enum values, defaults,
include paths, target name, or specified semantics. Only private implementation details
may be added to the supplied headers.
