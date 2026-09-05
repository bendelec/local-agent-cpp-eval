#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>
#include <cassert>
#include <iostream>

using namespace vwmini;

int main() {
    // Vec2 basics
    Vec2 a{1,2}, b{3,4};
    assert((a + b) == Vec2{4,6});
    assert(length(a) > 0);
    assert(normalized({0,0}) == Vec2{0,0});

    // triangulate simple square
    Polygon poly;
    poly.vertices = {{0,0},{1,0},{1,1},{0,1}};
    auto triRes = triangulate_simple_polygon(poly);
    assert(triRes);
    assert(triRes->size() == 2);

    // NavMesh create
    std::vector<Polygon> tris;
    for (auto &p : *triRes) tris.push_back(p);
    auto meshRes = NavMesh::create(tris);
    assert(meshRes);
    NavMesh mesh = *meshRes;
    assert(mesh.cell_count() == 2);
    assert(mesh.contains({0.5f,0.5f}));
    assert(!mesh.contains({2,2}));

    // find_path
    auto pathRes = find_path(mesh, {0.1f,0.1f}, {0.9f,0.9f});
    assert(pathRes);
    assert(pathRes->points.size() >= 2);

    // Simulation
    Simulation sim(mesh);
    AgentConfig cfg;
    cfg.position = {0.2f,0.2f};
    cfg.radius = 0.1f;
    cfg.max_speed = 1.0f;
    cfg.goal = Vec2{0.8f,0.8f};
    auto idRes = sim.add_agent(cfg);
    assert(idRes);
    auto state = sim.agent(*idRes);
    assert(state);
    assert(state->status == AgentStatus::Moving);
    auto stepRes = sim.step(1.0f);
    assert(stepRes);
    auto state2 = sim.agent(*idRes);
    assert(state2);
    // Should be closer or reached
    std::cout << "All basic tests passed\n";
    return 0;
}
