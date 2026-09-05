// Simulation tests (WP5/WP6): lifecycle, NotFound/OutsideMesh validation, steering
// toward goals, Reached status, clear_goal/idle, removal, and determinism.
#include "testing.hpp"

#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

using namespace vwmini;

namespace {
inline constexpr float kEps = 1e-4f;

Polygon Tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
}

Result<NavMesh> make_square_quad()
{
    // Convex unit square split into two CCW triangles along the (1,0)->(0,1) diagonal.
    std::vector<Polygon> polys{
        Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),
        Tri(Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}),
    };
    return NavMesh::create(polys);
}
} // namespace

int run_simulation_tests()
{
    auto mesh_res = make_square_quad();
    VWM_REQUIRE(mesh_res.has_value());

    // --- add_agent returns non-zero id and increments count ----------------------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.25f, 0.25f};
        cfg.radius = 0.1f;
        cfg.max_speed = 1.0f;
        auto id = sim.add_agent(cfg);
        VWM_REQUIRE(id.has_value());
        VWM_CHECK_NE(id->value, 0u);
        VWM_CHECK_EQ(sim.agent_count(), 1u);
        auto st = sim.agent(*id);
        VWM_REQUIRE(st.has_value());
        const Vec2 pos{0.25f, 0.25f};
        VWM_CHECK_EQ(st->position, pos);
        VWM_CHECK_EQ(st->status, AgentStatus::Idle);
    }

    // --- unknown agent queries return nullopt / NotFound ------------------------
    {
        Simulation sim(*mesh_res);
        VWM_CHECK(!sim.agent(AgentId{42}).has_value());
        VWM_CHECK_EQ(sim.remove_agent(AgentId{7}).error().code, ErrorCode::NotFound);
        VWM_CHECK_EQ(sim.clear_goal(AgentId{7}).error().code, ErrorCode::NotFound);
        VWM_CHECK_EQ(sim.set_goal(AgentId{7}, Vec2{0.5f, 0.5f}).error().code, ErrorCode::NotFound);
    }

    // --- invalid config rejected ------------------------------------------------
    {
        Simulation sim(*mesh_res);
        AgentConfig bad_pos{};
        bad_pos.position = Vec2{NAN, 0};
        VWM_CHECK_EQ(sim.add_agent(bad_pos).error().code, ErrorCode::InvalidArgument);

        AgentConfig bad_r{};
        bad_r.position = Vec2{0.25f, 0.25f};
        bad_r.radius = 0.0f;
        VWM_CHECK_EQ(sim.add_agent(bad_r).error().code, ErrorCode::InvalidArgument);

        AgentConfig outside{};
        outside.position = Vec2{5, 5};
        VWM_CHECK_EQ(sim.add_agent(outside).error().code, ErrorCode::OutsideMesh);
    }

    // --- set_goal with reachable goal moves the agent closer over steps ---------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.1f, 0.1f};
        cfg.radius = 0.05f;
        cfg.max_speed = 1.0f;
        cfg.goal = Vec2{0.9f, 0.9f};
        auto id = *sim.add_agent(cfg);
        VWM_REQUIRE(sim.step(0.1f).has_value());
        auto st = *sim.agent(id);
        VWM_CHECK_EQ(st.status, AgentStatus::Moving);
        const float before = length(Vec2{0.1f, 0.1f} - Vec2{0.9f, 0.9f});
        const float after = length(st.position - Vec2{0.9f, 0.9f});
        VWM_CHECK(after < before + kEps);
    }

    // --- agent reaches goal within arrival radius -> Reached --------------------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.4f, 0.4f};
        cfg.radius = 0.05f;
        cfg.max_speed = 2.0f;
        cfg.arrival_radius = 0.3f;
        cfg.goal = Vec2{0.6f, 0.6f};
        auto id = *sim.add_agent(cfg);
        for (int i = 0; i < 20; ++i) {
            VWM_CHECK(sim.step(0.05f).has_value());
        }
        auto st = *sim.agent(id);
        VWM_CHECK_EQ(st.status, AgentStatus::Reached);
        const Vec2 g{0.6f, 0.6f};
        VWM_CHECK_LE(length(st.position - g), kEps);
    }

    // --- clear_goal makes an idle agent with zero velocity -----------------------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.25f, 0.25f};
        cfg.max_speed = 1.0f;
        cfg.goal = Vec2{0.8f, 0.2f};
        auto id = *sim.add_agent(cfg);
        VWM_REQUIRE(sim.clear_goal(id).has_value());
        auto st = *sim.agent(id);
        VWM_CHECK_EQ(st.status, AgentStatus::Idle);
        VWM_CHECK_EQ(st.velocity.x, 0.0f);
        VWM_CHECK_EQ(st.velocity.y, 0.0f);
    }

    // --- remove_agent drops count and invalidates the id ------------------------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.25f, 0.25f};
        auto a = *sim.add_agent(cfg);
        auto b = *sim.add_agent(cfg);
        VWM_CHECK_EQ(sim.agent_count(), 2u);
        VWM_REQUIRE(sim.remove_agent(a).has_value());
        VWM_CHECK_EQ(sim.agent_count(), 1u);
        VWM_CHECK(!sim.agent(a).has_value());
        VWM_CHECK(sim.agent(b).has_value());
    }

    // --- step rejects non-finite/negative durations -----------------------------
    {
        Simulation sim(*mesh_res);
        VWM_CHECK_EQ(sim.step(-1.0f).error().code, ErrorCode::InvalidArgument);
        VWM_CHECK_EQ(sim.step(NAN).error().code, ErrorCode::InvalidArgument);
    }

    // --- determinism: two simulations advanced identically stay equal -----------
    {
        Simulation s1(*mesh_res);
        Simulation s2(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.1f, 0.1f};
        cfg.radius = 0.05f;
        cfg.max_speed = 1.0f;
        cfg.goal = Vec2{0.9f, 0.9f};
        auto i1 = *s1.add_agent(cfg);
        auto i2 = *s2.add_agent(cfg);
        for (int i = 0; i < 10; ++i) {
            VWM_CHECK(s1.step(0.05f).has_value());
            VWM_CHECK(s2.step(0.05f).has_value());
        }
        auto a = *s1.agent(i1);
        auto b = *s2.agent(i2);
        VWM_CHECK_EQ(a.position, b.position);
        VWM_CHECK_EQ(a.status, b.status);
    }

    return ::vwmtest::counters().failed;
}
