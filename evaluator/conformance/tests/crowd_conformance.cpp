// Normative crowd behavior track (C): the two SIM-011 scenarios plus the
// SIM-010 overlap-recovery robustness scenario documented in
// evaluator/docs/benchmark/reference-scenarios.md.
//
// Black-box: no prescribed steering algorithm or passing side is assumed. Uses the
// fixed square mesh from the scenario document; never calls geometry helpers.

#include "conformance_fixture.hpp"

#include <vwmini/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace vwmini;
using namespace vwmini_conformance;

namespace {

// Common procedure from reference-scenarios.md: step 1/30s for 12 seconds, asserting
// finiteness, containment, speed bound, and pair separation after every step, then
// that every agent reaches its goal.
void run_crowd_scenario(const std::vector<AgentConfig>& configs)
{
    const NavMesh mesh = make_square_mesh();
    Simulation sim(mesh);

    std::vector<AgentId> ids;
    ids.reserve(configs.size());
    for (const AgentConfig& config : configs) {
        const auto added = sim.add_agent(config);
        ASSERT_TRUE(added.has_value());
        ids.push_back(*added);
    }
    ASSERT_EQ(ids.size(), configs.size());

    for (int step_index = 0; step_index < 360; ++step_index) {
        ASSERT_TRUE(sim.step(1.0f / 30.0f).has_value());

        std::vector<AgentState> states;
        states.reserve(ids.size());
        for (const AgentId id : ids) {
            const auto state = sim.agent(id);
            ASSERT_TRUE(state.has_value());
            states.push_back(*state);
        }

        for (const AgentState& state : states) {
            EXPECT_TRUE(is_finite(state.position));
            EXPECT_TRUE(is_finite(state.velocity));
            EXPECT_TRUE(mesh.contains(state.position));
            EXPECT_LE(length(state.velocity), state.max_speed + 1e-4f);
        }

        for (std::size_t a = 0; a < states.size(); ++a) {
            for (std::size_t b = a + 1u; b < states.size(); ++b) {
                const float separation = length(states[a].position - states[b].position);
                EXPECT_GE(separation, states[a].radius + states[b].radius - 1e-3f);
            }
        }
    }

    for (const AgentId id : ids) {
        const auto state = sim.agent(id);
        ASSERT_TRUE(state.has_value());
        EXPECT_EQ(state->status, AgentStatus::Reached);
    }
}

} // namespace

TEST(Crowd_Crossing, TwoAgentsCrossAndReachGoals)
{
    AgentConfig a;
    a.position = Vec2{2.0f, 4.0f};
    a.goal = Vec2{8.0f, 6.0f};
    a.radius = 0.25f;
    a.max_speed = 1.0f;
    a.arrival_radius = 0.10f;

    AgentConfig b;
    b.position = Vec2{2.0f, 6.0f};
    b.goal = Vec2{8.0f, 4.0f};
    b.radius = 0.25f;
    b.max_speed = 1.0f;
    b.arrival_radius = 0.10f;

    run_crowd_scenario({a, b});
}

TEST(Crowd_Overtaking, FasterAgentOvertakesAndBothReachGoals)
{
    AgentConfig a;
    a.position = Vec2{3.0f, 5.0f};
    a.goal = Vec2{8.0f, 5.0f};
    a.radius = 0.25f;
    a.max_speed = 0.50f;
    a.arrival_radius = 0.10f;

    AgentConfig b;
    b.position = Vec2{2.0f, 5.0f};
    b.goal = Vec2{8.0f, 6.0f};
    b.radius = 0.25f;
    b.max_speed = 1.25f;
    b.arrival_radius = 0.10f;

    run_crowd_scenario({a, b});
}

TEST(Crowd_OverlapRecovery, InitiallyOverlappingDiscsSeparate)
{
    // SIM-010's explicit robustness edge case: centres may initially overlap.
    // This does not demand a general deadlock solver, but the local response
    // must actually separate this simple open-space pair rather than attract
    // it, stall it, or only preserve finite state.
    const NavMesh mesh = make_square_mesh();
    Simulation sim(mesh);

    AgentConfig a;
    a.position = Vec2{5.0f, 5.0f};
    a.goal = Vec2{5.0f, 1.0f};
    a.radius = 0.25f;
    a.max_speed = 1.0f;
    a.arrival_radius = 0.10f;

    AgentConfig b;
    b.position = Vec2{5.1f, 5.0f};
    b.goal = Vec2{5.0f, 9.0f};
    b.radius = 0.25f;
    b.max_speed = 1.0f;
    b.arrival_radius = 0.10f;

    const auto a_id = sim.add_agent(a).value();
    const auto b_id = sim.add_agent(b).value();
    float greatest_separation = length(a.position - b.position);

    for (int step_index = 0; step_index < 120; ++step_index) {
        ASSERT_TRUE(sim.step(1.0f / 30.0f).has_value());
        const auto a_state = sim.agent(a_id);
        const auto b_state = sim.agent(b_id);
        ASSERT_TRUE(a_state.has_value());
        ASSERT_TRUE(b_state.has_value());
        EXPECT_TRUE(is_finite(a_state->position));
        EXPECT_TRUE(is_finite(b_state->position));
        EXPECT_TRUE(mesh.contains(a_state->position));
        EXPECT_TRUE(mesh.contains(b_state->position));
        EXPECT_LE(length(a_state->velocity), a_state->max_speed + 1e-4f);
        EXPECT_LE(length(b_state->velocity), b_state->max_speed + 1e-4f);
        greatest_separation = std::max(
            greatest_separation, length(a_state->position - b_state->position));
    }

    // SIM-010 only says to *attempt* separation for an initially overlapping
    // start, not that arbitrary crowds must reach a collision-free solution.
    // In this unobstructed two-agent fixture, preserving the 0.1 m initial
    // distance is not an attempt: require a material, observable recovery
    // while deliberately not requiring the full 0.5 m touching distance.
    EXPECT_GE(greatest_separation, 0.25f);
}
