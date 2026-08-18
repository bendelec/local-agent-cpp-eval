// Crowd behavior track (C): the two private scenarios from
// evaluator/docs/benchmark/reference-scenarios.md (SIM-010..SIM-012). Black-box:
// no prescribed steering algorithm or passing side is assumed.

#include "test_fixtures.hpp"

#include <vwmini/simulation.hpp>

#include <cmath>
#include <vector>

using namespace vwmini;
using namespace vwmini_test;

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
