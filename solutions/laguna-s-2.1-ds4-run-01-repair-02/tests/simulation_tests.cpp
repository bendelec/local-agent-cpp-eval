// Simulation tests (WP5/WP6): lifecycle, NotFound/OutsideMesh validation, steering
// toward goals, Reached status, clear_goal/idle, removal, determinism, and continuous
// per-substep containment/speed/disc-separation regressions plus reflex-corner coverage.
#include <array>
#include <limits>

#include "testing.hpp"

#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

using namespace vwmini;

namespace {
inline constexpr float kEps = 1e-4f;

Polygon Tri(Vec2 a, Vec2 b, Vec2 c) {
    return Polygon{{a, b, c}};
}

Result<NavMesh> make_square_quad() {
    // Convex unit square split into two CCW triangles along the (1,0)->(0,1) diagonal.
    std::vector<Polygon> polys{
        Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),
        Tri(Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}),
    };
    return NavMesh::create(polys);
}

// L-shaped concave mesh (unit square minus its lower-left quadrant). Decomposed into six
// CCW triangles with exact shared edges so adjacency builds cleanly; exposes a reflex
// corner at (0.5,0.5) for steering-around-corner coverage.
Result<NavMesh> make_l_shape() {
    const Vec2 A{0.5f, 0.0f}; // bottom of reflex spine
    const Vec2 B{1.0f, 0.0f};
    const Vec2 H{1.0f, 0.5f};
    const Vec2 F{0.5f, 0.5f}; // reflex vertex
    const Vec2 C{1.0f, 1.0f};
    const Vec2 D{0.5f, 1.0f};
    const Vec2 E{0.0f, 0.5f};
    const Vec2 G{0.0f, 1.0f};
    std::vector<Polygon> polys{
        Tri(A, B, H), // lower-right rectangle half
        Tri(A, H, F), // lower-right rectangle half (shares AH)
        Tri(F, H, C), // upper-right rectangle half (shares FH)
        Tri(F, C, D), // upper-right rectangle half (shares FC)
        Tri(E, F, D), // top strip half (shares FD with above)
        Tri(E, D, G), // top strip half (shares ED)
    };
    return NavMesh::create(polys);
}
} // namespace

int run_simulation_tests() {
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
        VWM_CHECK_LE(length(st.position - g), cfg.arrival_radius);
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

    // --- continuous per-substep invariants: containment, speed budget ------------
    // Advance many tiny steps while steering toward several edge-near targets and assert
    // that the agent never leaves the mesh and never exceeds its speed/arrival budget on
    // any single frame. Exercises reflex corners via an L-shaped concave mesh.
    {
        auto lres = make_l_shape();
        VWM_REQUIRE(lres.has_value());
        const NavMesh& mesh = *lres;

        Simulation sim(mesh);
        AgentConfig cfg{};
        cfg.position = Vec2{0.1f, 0.9f};
        cfg.radius = 0.08f;
        cfg.max_speed = 1.0f;
        cfg.arrival_radius = 0.08f;
        auto id = *sim.add_agent(cfg);

        const std::array<Vec2, 4> legs = {
            Vec2{0.9f, 0.1f}, // wraps the reflex corner at (0.5,0.5)
            Vec2{0.1f, 0.9f},
            Vec2{0.9f, 0.9f},
            Vec2{0.6f, 0.6f},
        };
        int leg = 0;
        VWM_CHECK(sim.set_goal(id, legs[leg]).has_value());

        Vec2 prev = cfg.position;
        bool reached_any = false;
        for (int i = 0; i < 400; ++i) {
            VWM_CHECK(sim.step(0.01f).has_value());
            auto st = *sim.agent(id);
            // Containment invariant across every substep.
            VWM_CHECK(mesh.contains(st.position));
            // Speed-budget invariant: instantaneous speed never exceeds max_speed.
            VWM_CHECK_LE(length(st.velocity), cfg.max_speed + kEps);
            // Displacement-per-frame bound honoured except when snapping to the goal.
            if (st.status != AgentStatus::Reached) {
                VWM_CHECK_LE(length(st.position - prev), cfg.max_speed * 0.01f + kEps);
            }
            prev = st.position;

            if (st.status == AgentStatus::Reached && !reached_any) {
                reached_any = true;
                leg = (leg + 1) % static_cast<int>(legs.size());
                VWM_CHECK(sim.set_goal(id, legs[leg]).has_value());
            }
        }
        // At least one leg completed around the reflex corner.
        VWM_CHECK(reached_any);
    }

    // --- disc-disc separation horizon -------------------------------------------
    // Two agents approach head-on through a wide corridor; RVO must prevent overlap so the
    // centre-to-centre gap never drops below rsum minus tolerance over the whole run.
    {
        auto lres = make_l_shape();
        VWM_REQUIRE(lres.has_value());
        const NavMesh& mesh = *lres;
        const float r = 0.07f;
        const float sum_r = 2.0f * r;

        Simulation sim(mesh);
        AgentConfig left{};
        left.position = Vec2{0.2f, 0.5f};
        left.radius = r;
        left.max_speed = 1.0f;
        left.goal = Vec2{0.8f, 0.5f};
        AgentConfig right_cfg{};
        right_cfg.position = Vec2{0.8f, 0.5f};
        right_cfg.radius = r;
        right_cfg.max_speed = 1.0f;
        right_cfg.goal = Vec2{0.2f, 0.5f};
        auto a = *sim.add_agent(left);
        auto b = *sim.add_agent(right_cfg);

        float min_gap = length((*sim.agent(a)).position - (*sim.agent(b)).position);
        for (int i = 0; i < 300; ++i) {
            VWM_CHECK(sim.step(0.02f).has_value());
            const Vec2 pa = (*sim.agent(a)).position;
            const Vec2 pb = (*sim.agent(b)).position;
            const float g = length(pa - pb);
            if (g < min_gap) {
                min_gap = g;
            }
            // Per-agent speed budgets hold even under mutual avoidance.
            VWM_CHECK_LE(length((*sim.agent(a)).velocity), left.max_speed + kEps);
            VWM_CHECK_LE(length((*sim.agent(b)).velocity), right_cfg.max_speed + kEps);
        }
        // Discs may graze but never interpenetrate beyond floating-point slack.
        VWM_CHECK_GE(min_gap, sum_r - 2.0e-3f);
    }

    // --- contract: scalar validation rejects non-finite / illegal config ----------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.5f, 0.5f};
        cfg.radius = 0.1f;

        cfg.position = Vec2{NAN, 0.5f};
        VWM_CHECK_EQ(sim.add_agent(cfg).error().code, ErrorCode::InvalidArgument);
        cfg.position = Vec2{0.5f, 0.5f};

        cfg.radius = 0.0f;
        VWM_CHECK_EQ(sim.add_agent(cfg).error().code, ErrorCode::InvalidArgument);
        cfg.radius = -0.1f;
        VWM_CHECK_EQ(sim.add_agent(cfg).error().code, ErrorCode::InvalidArgument);
        cfg.radius = 0.1f;

        cfg.max_speed = 0.0f;
        VWM_CHECK_EQ(sim.add_agent(cfg).error().code, ErrorCode::InvalidArgument);
        cfg.max_speed = -1.0f;
        VWM_CHECK_EQ(sim.add_agent(cfg).error().code, ErrorCode::InvalidArgument);
        cfg.max_speed = 1.4f;

        cfg.arrival_radius = -2.0f; // only exactly -1 is a sentinel
        VWM_CHECK_EQ(sim.add_agent(cfg).error().code, ErrorCode::InvalidArgument);
        cfg.arrival_radius = std::numeric_limits<float>::quiet_NaN();
        VWM_CHECK_EQ(sim.add_agent(cfg).error().code, ErrorCode::InvalidArgument);
        cfg.arrival_radius = -1.0f; // valid sentinel -> uses radius

        VWM_CHECK_EQ(sim.agent_count(), static_cast<std::size_t>(0));
    }

    // --- contract: out-of-mesh position rejected without consuming an id -----------
    {
        Simulation sim(*mesh_res);
        AgentConfig bad{};
        bad.position = Vec2{5.0f, 5.0f};
        bad.radius = 0.1f;
        VWM_CHECK_EQ(sim.add_agent(bad).error().code, ErrorCode::OutsideMesh);
        // A subsequent valid add must receive the first assignable id (id continuity).
        AgentConfig ok{};
        ok.position = Vec2{0.5f, 0.5f};
        ok.radius = 0.1f;
        auto id = *sim.add_agent(ok);
        VWM_CHECK_EQ(id.value, static_cast<std::uint32_t>(1));
        VWM_CHECK_EQ(sim.agent_count(), static_cast<std::size_t>(1));
    }

    // --- contract: set_goal with outside-mesh goal preserves prior state -----------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.2f, 0.2f};
        cfg.radius = 0.1f;
        cfg.goal = Vec2{0.8f, 0.8f};
        auto id = *sim.add_agent(cfg);
        VWM_REQUIRE((*sim.agent(id)).status == AgentStatus::Moving);
        const auto before = *sim.agent(id);
        VWM_CHECK_EQ(sim.set_goal(id, Vec2{9.0f, 9.0f}).error().code, ErrorCode::OutsideMesh);
        const auto after = *sim.agent(id);
        VWM_CHECK_EQ(after.status, before.status);                        // unchanged
        VWM_CHECK(after.goal.has_value() && *after.goal == *before.goal); // retained
    }

    // --- contract: disconnected in-mesh goal -> NoPath with zero velocity ----------
    {
        // Two disjoint unit squares sharing no edge form two mesh components.
        std::vector<Polygon> polys{
            Tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),
            Tri(Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}),
            Tri(Vec2{5, 0}, Vec2{6, 0}, Vec2{5, 1}),
            Tri(Vec2{6, 0}, Vec2{6, 1}, Vec2{5, 1}),
        };
        auto dres = NavMesh::create(polys);
        VWM_REQUIRE(dres.has_value());
        Simulation sim(*dres);
        AgentConfig cfg{};
        cfg.position = Vec2{0.4f, 0.4f};
        cfg.radius = 0.05f;
        cfg.max_speed = 1.0f;
        cfg.goal = Vec2{5.4f, 0.4f}; // same component? no -> other square
        auto id = *sim.add_agent(cfg);
        auto st = *sim.agent(id);
        VWM_CHECK_EQ(st.status, AgentStatus::NoPath);
        VWM_CHECK_EQ(st.velocity.x, 0.0f);
        VWM_CHECK_EQ(st.velocity.y, 0.0f);
        VWM_CHECK(sim.step(1.0f).has_value());
        st = *sim.agent(id);
        VWM_CHECK_EQ(st.status, AgentStatus::NoPath); // terminal
    }

    // --- contract: immediate reach retains goal until clear_goal (no teleport) -----
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.4f, 0.4f};
        cfg.radius = 0.05f;
        cfg.max_speed = 2.0f;
        cfg.arrival_radius = 0.3f;
        cfg.goal = Vec2{0.6f, 0.6f}; // dist ~0.28 < arrival -> Reached at start
        auto id = *sim.add_agent(cfg);
        auto st = *sim.agent(id);
        VWM_CHECK_EQ(st.status, AgentStatus::Reached);
        VWM_CHECK_EQ(st.velocity.x, 0.0f);
        VWM_CHECK_EQ(st.velocity.y, 0.0f);
        VWM_CHECK(st.goal.has_value() && length(*st.goal - Vec2{0.6f, 0.6f}) <= kEps);
        VWM_CHECK(length(st.position - Vec2{0.4f, 0.4f}) <= kEps); // not teleported
        VWM_REQUIRE(sim.clear_goal(id).has_value());
        st = *sim.agent(id);
        VWM_CHECK_EQ(st.status, AgentStatus::Idle);
        VWM_CHECK(!st.goal.has_value());
    }

    // --- contract: step rejects non-finite / negative durations --------------------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.5f, 0.5f};
        cfg.radius = 0.1f;
        (void)sim.add_agent(cfg);
        VWM_CHECK_EQ(sim.step(-0.1f).error().code, ErrorCode::InvalidArgument);
        VWM_CHECK_EQ(sim.step(std::numeric_limits<float>::quiet_NaN()).error().code,
                     ErrorCode::InvalidArgument);
        VWM_CHECK(sim.step(0.0f).has_value()); // zero is a valid no-op
    }

    // --- contract: large/FLT_MAX duration terminates for idle agents ---------------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.5f, 0.5f};
        cfg.radius = 0.1f;
        cfg.arrival_radius = 1.0f;
        cfg.goal = Vec2{0.5f, 0.5f}; // coincident -> immediately Reached
        auto id = *sim.add_agent(cfg);
        VWM_CHECK_EQ((*sim.agent(id)).status, AgentStatus::Reached);
        const float huge = std::numeric_limits<float>::max();
        VWM_CHECK(sim.step(huge).has_value()); // must return, not hang
        VWM_CHECK_EQ((*sim.agent(id)).status, AgentStatus::Reached);
    }

    // --- contract: per-step speed budget + containment never violated --------------
    {
        auto lres = make_l_shape();
        VWM_REQUIRE(lres.has_value());
        const NavMesh& mesh = *lres;
        Simulation sim(mesh);
        AgentConfig cfg{};
        cfg.position = Vec2{0.05f, 0.95f};
        cfg.radius = 0.03f;
        cfg.max_speed = 1.0f;
        cfg.goal = Vec2{0.95f, 0.05f}; // routes around the reflex corner at F=(0.5,0.5)
        auto id = *sim.add_agent(cfg);
        constexpr float dt = 0.05f;
        bool reached = false;
        for (int i = 0; i < 5000 && !reached; ++i) {
            const Vec2 prev = (*sim.agent(id)).position;
            VWM_CHECK(sim.step(dt).has_value());
            const auto st = *sim.agent(id);
            VWM_CHECK_LE(length(st.position - prev), cfg.max_speed * dt + kEps); // budget
            VWM_CHECK(mesh.contains(st.position));                               // contained
            if (st.status == AgentStatus::Reached) {
                reached = true;
            }
        }
        VWM_CHECK(reached);
    }

    // --- contract: deterministic replay produces identical states ------------------
    {
        auto lres = make_l_shape();
        VWM_REQUIRE(lres.has_value());
        const NavMesh& mesh = *lres;
        AgentConfig left{}, right_cfg{};
        left.position = Vec2{0.1f, 0.5f};
        left.radius = 0.07f;
        left.max_speed = 1.0f;
        left.goal = Vec2{0.9f, 0.5f};
        right_cfg.position = Vec2{0.9f, 0.5f};
        right_cfg.radius = 0.07f;
        right_cfg.max_speed = 1.0f;
        right_cfg.goal = Vec2{0.1f, 0.5f};
        Simulation a(mesh), b(mesh);
        auto la = *a.add_agent(left);                       // id 1 (left)
        [[maybe_unused]] auto ra = *a.add_agent(right_cfg); // id 2 (right)
        auto lb = *b.add_agent(left);                       // id 1 (identical replay)
        [[maybe_unused]] auto rb = *b.add_agent(right_cfg); // id 2
        VWM_CHECK_EQ(la.value, lb.value);
        bool same = true;
        for (int i = 0; i < 200 && same; ++i) {
            (void)a.step(0.02f);
            (void)b.step(0.02f);
            for (const std::uint32_t sid : {la.value, static_cast<std::uint32_t>(2)}) {
                const auto sa = *a.agent(AgentId{sid});
                const auto sb = *b.agent(AgentId{sid});
                if (!(length(sa.position - sb.position) <= kEps &&
                      length(sa.velocity - sb.velocity) <= kEps)) {
                    same = false;
                    break;
                }
            }
        }
        VWM_CHECK(same);
    }

    // --- contract: moved-from simulation mutators return ordinary errors -----------
    {
        Simulation sim(*mesh_res);
        AgentConfig cfg{};
        cfg.position = Vec2{0.5f, 0.5f};
        cfg.radius = 0.1f;
        auto id = *sim.add_agent(cfg);
        Simulation moved(std::move(sim));
        // The destination owns state and must keep working after a move.
        auto mid = *moved.add_agent(cfg);
        VWM_CHECK_NE(mid.value, id.value);
        // Source is now moved-from; every mutation must fail without crashing.
        VWM_CHECK(!sim.add_agent(cfg).has_value());
        VWM_CHECK(!sim.remove_agent(id).has_value());
        VWM_CHECK(!sim.set_goal(id, Vec2{0.6f, 0.6f}).has_value());
        VWM_CHECK(!sim.clear_goal(id).has_value());
        VWM_CHECK(!sim.step(0.1f).has_value());
        VWM_CHECK_EQ(sim.agent_count(), static_cast<std::size_t>(0));
    }

    return ::vwmtest::counters().failed;
}
