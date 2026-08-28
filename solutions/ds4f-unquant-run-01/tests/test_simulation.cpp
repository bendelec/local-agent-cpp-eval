// WP6: Simulation behaviour — validation, lifecycle, goals, stepping,
// avoidance.

#include "test_framework.hpp"

#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include <limits>

using namespace vwmini;

namespace {

Polygon tri(Vec2 a, Vec2 b, Vec2 c) {
  Polygon p;
  p.vertices = {a, b, c};
  return p;
}

// Two-cell square (0,0)-(2,2) split by the (0,2)-(2,0) diagonal.
std::vector<Polygon> square_mesh() {
  return {tri({0, 0}, {2, 0}, {2, 2}), tri({0, 0}, {2, 2}, {0, 2})};
}

// Large open square (0,0)-(4,4) split by the (0,4)-(4,0) diagonal.
std::vector<Polygon> big_square_mesh() {
  return {tri({0, 0}, {4, 0}, {4, 4}), tri({0, 0}, {4, 4}, {0, 4})};
}

// Two disconnected squares: (0,0)-(2,2) and (5,0)-(7,2).
std::vector<Polygon> island_mesh() {
  return {tri({0, 0}, {2, 0}, {2, 2}), tri({0, 0}, {2, 2}, {0, 2}),
          tri({5, 0}, {7, 0}, {7, 2}), tri({5, 0}, {7, 2}, {5, 2})};
}

[[nodiscard]] NavMesh make_mesh(const std::vector<Polygon> &polys) {
  const auto mesh = NavMesh::create(polys);
  CHECK(mesh.has_value()); // test meshes must be valid
  return *mesh;
}

[[nodiscard]] float dist(Vec2 a, Vec2 b) { return length(a - b); }

} // namespace

void test_sim_add_agent_validation() {
  const NavMesh mesh = make_mesh(square_mesh());
  Simulation sim(mesh);
  const Vec2 inside{0.5f, 0.5f};
  const Vec2 outside{9.0f, 9.0f};

  AgentConfig config;
  config.position = inside;

  config.position = {std::numeric_limits<float>::quiet_NaN(), 0.5f};
  CHECK_ERROR(sim.add_agent(config), ErrorCode::InvalidArgument);
  config.position = inside;

  config.radius = 0.0f;
  CHECK_ERROR(sim.add_agent(config), ErrorCode::InvalidArgument);
  config.radius = -1.0f;
  CHECK_ERROR(sim.add_agent(config), ErrorCode::InvalidArgument);
  config.radius = 0.25f;

  config.max_speed = 0.0f;
  CHECK_ERROR(sim.add_agent(config), ErrorCode::InvalidArgument);
  config.max_speed = 1.4f;

  config.position = outside;
  CHECK_ERROR(sim.add_agent(config), ErrorCode::OutsideMesh);
  config.position = inside;

  config.goal = outside;
  CHECK_ERROR(sim.add_agent(config), ErrorCode::OutsideMesh);
  config.goal.reset();

  config.arrival_radius = -2.0f;
  CHECK_ERROR(sim.add_agent(config), ErrorCode::InvalidArgument);
  config.arrival_radius = -1.0f;

  // -1 uses the radius; -0.0f is a valid explicit zero.
  const auto id1 = sim.add_agent(config);
  CHECK(id1.has_value());
  CHECK(id1->value != 0);
  config.arrival_radius = -0.0f;
  const auto ok = sim.add_agent(config);
  CHECK(ok.has_value());
  CHECK(ok->value != id1->value);
  CHECK_EQ(sim.agent_count(), std::size_t(2));

  // The default arrival radius is the agent radius.
  const auto state = sim.agent(*id1);
  CHECK(state.has_value());
  CHECK_EQ(state->radius, 0.25f);
  CHECK(state->status == AgentStatus::Idle);
}

void test_sim_goal_lifecycle() {
  const NavMesh mesh = make_mesh(square_mesh());
  Simulation sim(mesh);
  const auto id = sim.add_agent(AgentConfig{.position = {0.5f, 0.5f}});
  CHECK(id.has_value());
  const AgentId agent_id = *id;

  CHECK_ERROR(sim.set_goal(AgentId{99}, {1.5f, 0.5f}), ErrorCode::NotFound);
  CHECK_ERROR(
      sim.set_goal(agent_id, {std::numeric_limits<float>::quiet_NaN(), 0.5f}),
      ErrorCode::InvalidArgument);
  CHECK_ERROR(sim.set_goal(agent_id, {9.0f, 9.0f}), ErrorCode::OutsideMesh);

  // A goal within the effective arrival radius is immediately reached.
  CHECK(sim.set_goal(agent_id, {0.6f, 0.6f}).has_value());
  auto state = sim.agent(agent_id);
  CHECK(state.has_value());
  CHECK(state->status == AgentStatus::Reached);
  CHECK_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
  CHECK(state->goal.has_value());
  CHECK_EQ(*state->goal, (Vec2{0.6f, 0.6f}));

  // A farther connected goal produces a Moving route.
  CHECK(sim.set_goal(agent_id, {1.5f, 1.5f}, 0.0f).has_value());
  state = sim.agent(agent_id);
  CHECK(state->status == AgentStatus::Moving);

  // clear_goal -> Idle, no goal, zero velocity, and step does not move it.
  CHECK(sim.clear_goal(agent_id).has_value());
  state = sim.agent(agent_id);
  CHECK(state->status == AgentStatus::Idle);
  CHECK(!state->goal.has_value());
  CHECK_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
  CHECK(sim.step(1.0f).has_value());
  state = sim.agent(agent_id);
  CHECK_EQ(state->position, (Vec2{0.5f, 0.5f}));
}

void test_sim_disconnected_goal() {
  const NavMesh mesh = make_mesh(island_mesh());
  Simulation sim(mesh);
  const auto id = sim.add_agent(AgentConfig{.position = {0.5f, 0.5f}});
  CHECK(id.has_value());

  // In-mesh but disconnected: goal is stored, status NoPath, zero velocity.
  CHECK(sim.set_goal(*id, {6.0f, 1.0f}).has_value());
  const auto state = sim.agent(*id);
  CHECK(state.has_value());
  CHECK(state->status == AgentStatus::NoPath);
  CHECK_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
  CHECK(state->goal.has_value());
  CHECK_EQ(*state->goal, (Vec2{6.0f, 1.0f}));

  // Steps never move a NoPath agent.
  CHECK(sim.step(0.5f).has_value());
  const auto after = sim.agent(*id);
  CHECK_EQ(after->position, (Vec2{0.5f, 0.5f}));

  // A later connected goal restores movement.
  CHECK(sim.set_goal(*id, {1.5f, 1.5f}).has_value());
  CHECK(sim.agent(*id)->status == AgentStatus::Moving);
}

void test_sim_remove_agent() {
  const NavMesh mesh = make_mesh(square_mesh());
  Simulation sim(mesh);
  const auto id = sim.add_agent(AgentConfig{.position = {0.5f, 0.5f}});
  CHECK(id.has_value());
  CHECK_EQ(sim.agent_count(), std::size_t(1));

  CHECK(sim.remove_agent(*id).has_value());
  CHECK_EQ(sim.agent_count(), std::size_t(0));
  CHECK(!sim.agent(*id).has_value());
  CHECK_ERROR(sim.remove_agent(*id), ErrorCode::NotFound);
  CHECK_ERROR(sim.set_goal(*id, {1.5f, 1.5f}), ErrorCode::NotFound);
  CHECK_ERROR(sim.clear_goal(*id), ErrorCode::NotFound);

  // A new agent gets a fresh, never-reused id.
  const auto next = sim.add_agent(AgentConfig{.position = {1.5f, 1.5f}});
  CHECK(next.has_value());
  CHECK(next->value != id->value);
  CHECK_EQ(sim.agent_count(), std::size_t(1));
}

void test_sim_step_validation_transactional() {
  const NavMesh mesh = make_mesh(square_mesh());
  Simulation sim(mesh);
  const auto id = sim.add_agent(AgentConfig{.position = {0.5f, 0.5f}});
  CHECK(id.has_value());
  CHECK(sim.set_goal(*id, {1.7f, 0.3f}).has_value());

  const auto before = sim.agent(*id);
  CHECK(before.has_value());

  CHECK_ERROR(sim.step(-0.5f), ErrorCode::InvalidArgument);
  CHECK_EQ(sim.agent(*id)->position, before->position);
  CHECK_EQ(sim.agent(*id)->velocity, before->velocity);

  CHECK_ERROR(sim.step(std::numeric_limits<float>::quiet_NaN()),
              ErrorCode::InvalidArgument);
  CHECK_EQ(sim.agent(*id)->position, before->position);

  // Zero is a success that changes nothing.
  CHECK(sim.step(0.0f).has_value());
  CHECK_EQ(sim.agent(*id)->position, before->position);

  // A long duration must not overshoot the goal (single straight route).
  CHECK(
      sim.set_goal(*id, {0.5f, 0.5f}).has_value()); // back to start, then jump
  for (int i = 0; i < 100 && sim.agent(*id)->status == AgentStatus::Moving;
       ++i) {
    CHECK(sim.step(1.0f).has_value()); // 1 s in 20 substeps of 0.05
  }
  const auto state = sim.agent(*id);
  CHECK(state->status == AgentStatus::Reached);
  CHECK(dist(state->position, (Vec2{0.5f, 0.5f})) <= 0.25f + 1e-4f);
}

void test_sim_movement_bounds() {
  const NavMesh mesh = make_mesh(square_mesh());
  Simulation sim(mesh);
  const auto id =
      sim.add_agent(AgentConfig{.position = {0.5f, 0.5f}, .max_speed = 1.0f});
  CHECK(id.has_value());
  CHECK(sim.set_goal(*id, {1.7f, 0.2f}, 0.0f).has_value());

  constexpr float kDt = 0.05f;
  for (int i = 0; i < 200 && sim.agent(*id)->status != AgentStatus::Reached;
       ++i) {
    CHECK(sim.step(kDt).has_value());
    const auto state = sim.agent(*id);
    // SIM-012: never exceed max speed, never leave the mesh, no teleports.
    CHECK(length(state->velocity) <= 1.0f + 1e-4f);
    CHECK(mesh.contains(state->position));
  }
  const auto state = sim.agent(*id);
  CHECK(state->status == AgentStatus::Reached);
  CHECK_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
  // Zero arrival radius: the agent must be exactly at the goal.
  CHECK(dist(state->position, (Vec2{1.7f, 0.2f})) <= 1e-4f);
}

// SIM-011 acceptance: two crossing agents pick collision-free motion and make
// progress; no pair ever overlaps by more than 1e-3 after any returned step.
void test_sim_crossing_avoidance() {
  const NavMesh mesh = make_mesh(big_square_mesh());
  Simulation sim(mesh);

  AgentConfig a;
  a.position = {0.5f, 2.0f};
  a.radius = 0.25f;
  a.max_speed = 1.0f;
  const auto ida = sim.add_agent(a);

  AgentConfig b;
  b.position = {2.0f, 0.5f};
  b.radius = 0.25f;
  b.max_speed = 1.0f;
  const auto idb = sim.add_agent(b);
  CHECK(ida.has_value() && idb.has_value());

  CHECK(sim.set_goal(*ida, {3.5f, 2.0f}, /*arrival_radius=*/1.0f).has_value());
  CHECK(sim.set_goal(*idb, {2.0f, 3.5f}, /*arrival_radius=*/1.0f).has_value());

  const float r_sum = 0.5f;
  bool made_progress = false;
  for (int i = 0; i < 240; ++i) {
    CHECK(sim.step(0.05f).has_value());
    const auto sa = sim.agent(*ida);
    const auto sb = sim.agent(*idb);
    CHECK(sa.has_value() && sb.has_value());
    // No overlap beyond the 1e-3 tolerance after every returned step.
    CHECK(dist(sa->position, sb->position) + 1e-3f >= r_sum);
    CHECK(mesh.contains(sa->position));
    CHECK(mesh.contains(sb->position));
    CHECK(length(sa->velocity) <= 1.0f + 1e-4f);
    CHECK(length(sb->velocity) <= 1.0f + 1e-4f);
    if (i == 40) { // well past the crossing point
      made_progress = dist(sa->position, (Vec2{3.5f, 2.0f})) <
                          dist(Vec2{0.5f, 2.0f}, (Vec2{3.5f, 2.0f})) &&
                      dist(sb->position, (Vec2{2.0f, 3.5f})) <
                          dist(Vec2{2.0f, 0.5f}, (Vec2{2.0f, 3.5f}));
    }
  }
  CHECK(made_progress);
  const auto ea = sim.agent(*ida);
  const auto eb = sim.agent(*idb);
  CHECK(ea->status == AgentStatus::Reached);
  CHECK(eb->status == AgentStatus::Reached);
}

// A moving agent must deflect around an idle agent that stays exactly put.
void test_sim_avoids_static_agent() {
  const NavMesh mesh = make_mesh(square_mesh());
  Simulation sim(mesh);

  const auto idle = sim.add_agent(AgentConfig{.position = {1.0f, 1.0f}});
  const auto mover =
      sim.add_agent(AgentConfig{.position = {0.3f, 1.0f}, .max_speed = 1.0f});
  CHECK(idle.has_value() && mover.has_value());

  CHECK(
      sim.set_goal(*mover, {1.7f, 1.0f}, /*arrival_radius=*/0.6f).has_value());

  bool overlapped = false;
  for (int i = 0; i < 160; ++i) {
    CHECK(sim.step(0.05f).has_value());
    const auto sm = sim.agent(*mover);
    const auto si = sim.agent(*idle);
    if (dist(sm->position, si->position) + 1e-3f < 0.5f) {
      overlapped = true;
    }
    CHECK_EQ(si->position, (Vec2{1.0f, 1.0f})); // idle agent never moves
  }
  CHECK(!overlapped);
  CHECK(sim.agent(*mover)->status == AgentStatus::Reached);
  // The mover still ends within reasonable reach of the goal.
  CHECK(dist(sim.agent(*mover)->position, (Vec2{1.7f, 1.0f})) <= 0.6f + 1e-4f);
}

void test_sim_determinism_and_snapshot() {
  const NavMesh mesh = make_mesh(big_square_mesh());

  const auto run = [&mesh]() {
    Simulation sim(mesh);
    const auto ida =
        sim.add_agent(AgentConfig{.position = {0.5f, 2.0f}, .max_speed = 1.0f});
    const auto idb =
        sim.add_agent(AgentConfig{.position = {2.0f, 0.5f}, .max_speed = 1.0f});
    CHECK(sim.set_goal(*ida, {3.5f, 2.0f}, 1.0f).has_value());
    CHECK(sim.set_goal(*idb, {2.0f, 3.5f}, 1.0f).has_value());
    std::vector<Vec2> pa, pb, va, vb;
    for (int i = 0; i < 60; ++i) {
      CHECK(sim.step(0.05f).has_value());
      if (i == 10 || i == 30 || i == 59) {
        pa.push_back(sim.agent(*ida)->position);
        pb.push_back(sim.agent(*idb)->position);
        va.push_back(sim.agent(*ida)->velocity);
        vb.push_back(sim.agent(*idb)->velocity);
      }
    }
    return std::make_tuple(pa, pb, va, vb);
  };

  const auto r1 = run();
  const auto r2 = run();
  CHECK_EQ(std::get<0>(r1), std::get<0>(r2));
  CHECK_EQ(std::get<1>(r1), std::get<1>(r2));
  CHECK_EQ(std::get<2>(r1), std::get<2>(r2));
  CHECK_EQ(std::get<3>(r1), std::get<3>(r2));

  // Snapshot copies do not affect the simulation.
  Simulation sim(mesh);
  const auto id = sim.add_agent(AgentConfig{.position = {0.5f, 0.5f}});
  CHECK(id.has_value());
  auto state = sim.agent(*id);
  CHECK(state.has_value());
  state->position = {42.0f, 42.0f};
  state->velocity = {7.0f, 7.0f};
  state->status = AgentStatus::Reached;
  CHECK_EQ(sim.agent(*id)->position, (Vec2{0.5f, 0.5f}));
  CHECK(sim.agent(*id)->status == AgentStatus::Idle);
}

int register_simulation_tests() {
  using vwmini::test::Registry;
  Registry::instance().add("sim_add_agent_validation",
                           &test_sim_add_agent_validation);
  Registry::instance().add("sim_goal_lifecycle", &test_sim_goal_lifecycle);
  Registry::instance().add("sim_disconnected_goal",
                           &test_sim_disconnected_goal);
  Registry::instance().add("sim_remove_agent", &test_sim_remove_agent);
  Registry::instance().add("sim_step_validation_transactional",
                           &test_sim_step_validation_transactional);
  Registry::instance().add("sim_movement_bounds", &test_sim_movement_bounds);
  Registry::instance().add("sim_crossing_avoidance",
                           &test_sim_crossing_avoidance);
  Registry::instance().add("sim_avoids_static_agent",
                           &test_sim_avoids_static_agent);
  Registry::instance().add("sim_determinism_and_snapshot",
                           &test_sim_determinism_and_snapshot);
  return 0;
}

namespace {
const int simulation_tests_registered = register_simulation_tests();
} // namespace
