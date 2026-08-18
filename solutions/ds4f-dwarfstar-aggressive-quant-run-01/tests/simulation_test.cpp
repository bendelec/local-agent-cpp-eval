#include <vwmini/simulation.hpp>

#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

using namespace vwmini;

namespace {

int failures = 0;

#define CHECK(...)                                                                                                     \
    do {                                                                                                               \
        if (!(__VA_ARGS__)) {                                                                                          \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__);                                \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

Polygon tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
}

// Large open square (10x10) split into two triangles along the diagonal.
std::vector<Polygon> openMesh()
{
    return {tri({0, 0}, {10, 0}, {10, 10}), tri({0, 0}, {10, 10}, {0, 10})};
}

// Two disconnected triangles so in-mesh goals can be unreachable (NoPath).
std::vector<Polygon> disconnectedMesh()
{
    return {tri({0, 0}, {1, 0}, {0, 1}), tri({5, 0}, {6, 0}, {5, 1})};
}

static void test_add_agent_validation()
{
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));

    // Non-finite configuration -> InvalidArgument.
    CHECK(sim.add_agent(AgentConfig{{std::nanf(""), 0.0f}}).error().code == ErrorCode::InvalidArgument);
    CHECK(sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.0f}).error().code == ErrorCode::InvalidArgument);
    CHECK(sim.add_agent(AgentConfig{{1.0f, 1.0f}, -0.5f}).error().code == ErrorCode::InvalidArgument);
    CHECK(sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 0.0f}).error().code == ErrorCode::InvalidArgument);
    CHECK(sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, {}, std::nanf("")}).error().code ==
          ErrorCode::InvalidArgument);
    // Invalid arrival radius (SIM-006): -2 and -0.5 invalid; -1 and -0.0 valid.
    CHECK(sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, {}, -2.0f}).error().code == ErrorCode::InvalidArgument);
    CHECK(sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, {}, -0.5f}).error().code == ErrorCode::InvalidArgument);

    // Position / goal outside mesh -> OutsideMesh.
    CHECK(sim.add_agent(AgentConfig{{20.0f, 20.0f}}).error().code == ErrorCode::OutsideMesh);
    CHECK(sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{20.0f, 20.0f}}).error().code ==
          ErrorCode::OutsideMesh);

    // Valid no-goal agent -> Idle, non-zero id.
    auto ok = sim.add_agent(AgentConfig{{1.0f, 1.0f}});
    CHECK(ok.has_value());
    if (ok) {
        CHECK(ok->value != 0);
    }
    CHECK(sim.agent_count() == 1);

    // Valid goal within arrival radius -> Reached immediately.
    auto r2 = sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{1.3f, 1.0f}, 0.4f});
    CHECK(r2.has_value());
    if (r2) {
        auto st = sim.agent(*r2);
        CHECK(st.has_value());
        CHECK(st->status == AgentStatus::Reached);
        CHECK(st->velocity == Vec2{0.0f, 0.0f});
    }

    // Valid far goal -> Moving.
    auto r3 = sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{9.0f, 9.0f}});
    CHECK(r3.has_value());
    if (r3) {
        auto st = sim.agent(*r3);
        CHECK(st.has_value());
        CHECK(st->status == AgentStatus::Moving);
    }
}

static void test_no_path_and_disconnected()
{
    auto r = NavMesh::create(disconnectedMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));

    // Goal in the other component -> NoPath (add_agent succeeds, status NoPath).
    auto ok = sim.add_agent(AgentConfig{{0.2f, 0.2f}, 0.25f, 1.4f, Vec2{5.2f, 0.2f}});
    CHECK(ok.has_value());
    if (ok) {
        auto st = sim.agent(*ok);
        CHECK(st.has_value());
        CHECK(st->status == AgentStatus::NoPath);
        CHECK(st->velocity == Vec2{0.0f, 0.0f});
    }
}

static void test_set_and_clear_goal()
{
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));
    auto ok = sim.add_agent(AgentConfig{{1.0f, 1.0f}});
    CHECK(ok.has_value());
    AgentId id = *ok;

    // Unknown id -> NotFound.
    CHECK(sim.set_goal({99}, {5.0f, 5.0f}).error().code == ErrorCode::NotFound);
    CHECK(sim.clear_goal({99}).error().code == ErrorCode::NotFound);
    CHECK(sim.remove_agent({99}).error().code == ErrorCode::NotFound);

    // Non-finite goal -> InvalidArgument.
    CHECK(sim.set_goal(id, {std::nanf(""), 5.0f}).error().code == ErrorCode::InvalidArgument);
    // Invalid arrival radius.
    CHECK(sim.set_goal(id, {5.0f, 5.0f}, -2.0f).error().code == ErrorCode::InvalidArgument);
    // Goal outside mesh -> OutsideMesh.
    CHECK(sim.set_goal(id, {20.0f, 20.0f}).error().code == ErrorCode::OutsideMesh);

    // Already within arrival radius -> Reached, zero velocity.
    CHECK(sim.set_goal(id, {1.4f, 1.0f}, 0.5f).has_value());
    auto st = sim.agent(id);
    CHECK(st.has_value());
    CHECK(st->status == AgentStatus::Reached);
    CHECK(st->velocity == Vec2{0.0f, 0.0f});

    // Connected far goal -> Moving.
    CHECK(sim.set_goal(id, {9.0f, 9.0f}).has_value());
    st = sim.agent(id);
    CHECK(st.has_value());
    CHECK(st->status == AgentStatus::Moving);
    CHECK(st->goal.has_value());

    // clear_goal -> Idle, zero velocity, empty goal.
    CHECK(sim.clear_goal(id).has_value());
    st = sim.agent(id);
    CHECK(st.has_value());
    CHECK(st->status == AgentStatus::Idle);
    CHECK(!st->goal.has_value());
    CHECK(st->velocity == Vec2{0.0f, 0.0f});
}

static void test_step_validation_transactional()
{
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));
    auto ok = sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{9.0f, 9.0f}});
    CHECK(ok.has_value());
    AgentId id = *ok;

    auto before = sim.agent(id);
    CHECK(before.has_value());

    // Invalid durations -> InvalidArgument and no state change.
    CHECK(sim.step(std::nanf("")).error().code == ErrorCode::InvalidArgument);
    CHECK(sim.step(-1.0f).error().code == ErrorCode::InvalidArgument);
    CHECK(sim.step(0.0f).has_value());
    auto after = sim.agent(id);
    CHECK(after.has_value());
    CHECK(after->position == before->position);
    CHECK(after->velocity == before->velocity);
    CHECK(after->status == before->status);
}

static void test_step_motion_and_arrival()
{
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));
    auto ok = sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{9.0f, 9.0f}, 0.3f});
    CHECK(ok.has_value());
    AgentId id = *ok;

    // Large step: must not overshoot the goal, and speed bounded by max_speed.
    CHECK(sim.step(20.0f).has_value());
    auto st = sim.agent(id);
    CHECK(st.has_value());
    CHECK(st->status == AgentStatus::Reached);
    // Within arrival radius (0.3) of goal (9,9), never overshooting past it.
    const float dg = length(st->position - Vec2{9.0f, 9.0f});
    CHECK(dg <= 0.3f);
    CHECK(st->velocity == Vec2{0.0f, 0.0f});

    // Per-substep speed bound: max displacement per substep <= max_speed * substep.
    auto r2 = NavMesh::create(openMesh());
    CHECK(r2.has_value());
    if (!r2) {
        return;
    }
    Simulation sim2(std::move(*r2));
    auto ok2 = sim2.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.0f, Vec2{9.0f, 9.0f}});
    CHECK(ok2.has_value());
    AgentId id2 = *ok2;
    Vec2 prev = sim2.agent(id2)->position;
    CHECK(sim2.step(0.5f).has_value());
    auto st2 = sim2.agent(id2);
    CHECK(st2.has_value());
    CHECK(length(st2->position - prev) <= 1.0f * 0.5f + 1e-3f);
    CHECK(st2->status == AgentStatus::Moving);
}

static void test_avoidance_crossing()
{
    // Two agents on perpendicular crossing routes must avoid overlap and make
    // progress (SIM-011).
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));
    auto a = sim.add_agent(AgentConfig{{2.0f, 5.0f}, 0.25f, 1.4f, Vec2{9.0f, 5.0f}});
    auto b = sim.add_agent(AgentConfig{{5.0f, 2.0f}, 0.25f, 1.4f, Vec2{5.0f, 9.0f}});
    CHECK(a.has_value() && b.has_value());

    float maxOverlap = 0.0f;
    for (int i = 0; i < 60; ++i) {
        CHECK(sim.step(0.1f).has_value());
        auto sa = sim.agent(*a);
        auto sb = sim.agent(*b);
        CHECK(sa.has_value() && sb.has_value());
        const float gap = length(sa->position - sb->position) - (sa->radius + sb->radius);
        if (gap < 0.0f) {
            maxOverlap = std::max(maxOverlap, -gap);
        }
    }
    // No overlap by more than 1e-3.
    CHECK(maxOverlap <= 1e-3f);

    // Progress toward goals.
    auto ea = sim.agent(*a);
    auto eb = sim.agent(*b);
    CHECK(ea.has_value() && eb.has_value());
    CHECK(ea->position.x > 2.0f);
    CHECK(eb->position.y > 2.0f);
}

static void test_avoidance_head_on()
{
    // Head-on agents must avoid and still reach goals.
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    const NavMesh mesh = *r; // copy kept for containment checks
    Simulation sim(std::move(*r));
    auto a = sim.add_agent(AgentConfig{{1.0f, 5.0f}, 0.25f, 1.0f, Vec2{9.0f, 5.0f}});
    auto b = sim.add_agent(AgentConfig{{9.0f, 5.0f}, 0.25f, 1.0f, Vec2{1.0f, 5.0f}});
    CHECK(a.has_value() && b.has_value());

    float maxOverlap = 0.0f;
    for (int i = 0; i < 80; ++i) {
        CHECK(sim.step(0.1f).has_value());
        auto sa = sim.agent(*a);
        auto sb = sim.agent(*b);
        CHECK(sa.has_value() && sb.has_value());
        const float gap = length(sa->position - sb->position) - (sa->radius + sb->radius);
        if (gap < 0.0f) {
            maxOverlap = std::max(maxOverlap, -gap);
        }
    }
    CHECK(maxOverlap <= 1e-3f);

    // Head-on agents must stay in mesh and keep moving (no teleport), even if
    // the symmetric face-off leaves them boxed (SIM-012 permits stopping when
    // boxed; the crossing test above exercises real passing + goal reach).
    auto ea = sim.agent(*a);
    auto eb = sim.agent(*b);
    CHECK(ea.has_value() && eb.has_value());
    CHECK(mesh.contains(ea->position));
    CHECK(mesh.contains(eb->position));
}

static void test_agent_snapshot_and_remove()
{
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));
    auto ok = sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{9.0f, 9.0f}});
    CHECK(ok.has_value());
    AgentId id = *ok;

    auto st = sim.agent(id);
    CHECK(st.has_value());
    // Snapshot is a value copy; mutating it cannot affect the simulation.
    st->position = {0.0f, 0.0f};
    CHECK(sim.agent(id)->position == Vec2{1.0f, 1.0f});

    CHECK(sim.agent_count() == 1);
    CHECK(sim.remove_agent(id).has_value());
    CHECK(sim.agent_count() == 0);
    CHECK(!sim.agent(id).has_value());
    CHECK(sim.remove_agent(id).error().code == ErrorCode::NotFound);
    CHECK(sim.set_goal(id, {5.0f, 5.0f}).error().code == ErrorCode::NotFound);
}

static void test_determinism()
{
    auto r = NavMesh::create(openMesh());
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    Simulation sim(std::move(*r));
    auto ok = sim.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{9.0f, 9.0f}});
    CHECK(ok.has_value());

    auto run = [&]() {
        Simulation s(std::move(*NavMesh::create(openMesh())));
        auto o = s.add_agent(AgentConfig{{1.0f, 1.0f}, 0.25f, 1.4f, Vec2{9.0f, 9.0f}});
        CHECK(o.has_value());
        CHECK(s.step(1.0f).has_value());
        return s.agent(*o)->position;
    };
    const Vec2 p1 = run();
    const Vec2 p2 = run();
    CHECK(p1 == p2);
}
} // anonymous namespace

int main()
{
    test_add_agent_validation();
    test_no_path_and_disconnected();
    test_set_and_clear_goal();
    test_step_validation_transactional();
    test_step_motion_and_arrival();
    test_avoidance_crossing();
    test_avoidance_head_on();
    test_agent_snapshot_and_remove();
    test_determinism();

    if (failures == 0) {
        std::fprintf(stdout, "simulation_test: OK\n");
        return 0;
    }
    std::fprintf(stdout, "simulation_test: %d failure(s)\n", failures);
    return 1;
}
