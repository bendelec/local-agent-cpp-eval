// Normative close-following reflex-corner progress and avoidance regression.
#include "conformance_fixture.hpp"

#include <vwmini/simulation.hpp>

#include <cmath>

using namespace vwmini;
using namespace vwmini_conformance;

TEST(Crowd_ReflexCornerFollowing, CloseAgentsBothRoundCornerAndReach)
{
    // A close-following pair should retain its valid bent route while local
    // avoidance is active. Distinct terminal positions avoid an infeasible
    // identical-goal occupancy problem.
    const auto created = NavMesh::create({
        // Bottom arm [0, 5] x [0, 1], then left arm [0, 1] x [1, 5].
        Polygon{{{0.0f, 0.0f}, {5.0f, 0.0f}, {1.0f, 1.0f}}},
        Polygon{{{5.0f, 0.0f}, {5.0f, 1.0f}, {1.0f, 1.0f}}},
        Polygon{{{0.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 5.0f}}},
        Polygon{{{1.0f, 1.0f}, {1.0f, 5.0f}, {0.0f, 5.0f}}},
    });
    ASSERT_TRUE(created.has_value());
    const NavMesh mesh = *created;
    Simulation sim(mesh);

    AgentConfig leader;
    leader.position = Vec2{2.5f, 0.5f};
    leader.goal = Vec2{0.5f, 3.0f};
    leader.radius = 0.25f;
    leader.max_speed = 1.4f;
    leader.arrival_radius = 0.10f;

    AgentConfig follower = leader;
    follower.position = Vec2{3.1f, 0.5f};
    follower.goal = Vec2{0.5f, 4.0f};

    const auto added_leader = sim.add_agent(leader);
    const auto added_follower = sim.add_agent(follower);
    ASSERT_TRUE(added_leader.has_value());
    ASSERT_TRUE(added_follower.has_value());
    const AgentId leader_id = *added_leader;
    const AgentId follower_id = *added_follower;

    const auto initial_leader = sim.agent(leader_id);
    const auto initial_follower = sim.agent(follower_id);
    ASSERT_TRUE(initial_leader.has_value());
    ASSERT_TRUE(initial_follower.has_value());
    ASSERT_EQ(initial_leader->status, AgentStatus::Moving);
    ASSERT_EQ(initial_follower->status, AgentStatus::Moving);

    for (int step_index = 0; step_index < 600; ++step_index) {
        ASSERT_TRUE(sim.step(1.0f / 60.0f).has_value());
        const auto leader_state = sim.agent(leader_id);
        const auto follower_state = sim.agent(follower_id);
        ASSERT_TRUE(leader_state.has_value());
        ASSERT_TRUE(follower_state.has_value());
        for (const AgentState* state : {&*leader_state, &*follower_state}) {
            EXPECT_TRUE(is_finite(state->position));
            EXPECT_TRUE(is_finite(state->velocity));
            EXPECT_TRUE(mesh.contains(state->position));
            EXPECT_LE(length(state->velocity), state->max_speed + 1e-4f);
        }
        EXPECT_GE(length(leader_state->position - follower_state->position),
                  leader_state->radius + follower_state->radius - 1e-3f);
    }

    const auto final_leader = sim.agent(leader_id);
    const auto final_follower = sim.agent(follower_id);
    ASSERT_TRUE(final_leader.has_value());
    ASSERT_TRUE(final_follower.has_value());
    EXPECT_EQ(final_leader->status, AgentStatus::Reached);
    EXPECT_EQ(final_follower->status, AgentStatus::Reached);
}
