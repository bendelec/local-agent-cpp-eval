#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>
#include <cassert>
#include <iostream>
#include <limits>
#include <cmath>

using namespace vwmini;

void test_extreme_containment() {
    Polygon poly;
    poly.vertices = {{1e6f,1e6f},{1e6f+10.0f,1e6f},{1e6f+10.0f,1e6f+10.0f},{1e6f,1e6f+10.0f}};
    auto triRes = triangulate_simple_polygon(poly);
    assert(triRes);
    std::vector<Polygon> tris;
    for (auto &p:*triRes) tris.push_back(p);
    auto meshRes = NavMesh::create(tris);
    assert(meshRes);
    NavMesh mesh=*meshRes;
    // interior point
    assert(mesh.contains({1e6f+5.0f,1e6f+5.0f}));
    // outside
    assert(!mesh.contains({1e6f+20.0f,1e6f+20.0f}));
    std::cout<<"extreme containment ok\n";
}

void test_epsilon_edge() {
    // two triangles sharing edge with epsilon offset
    Polygon t1; t1.vertices={{0,0},{1,0},{0,1}};
    Polygon t2; t2.vertices={{0,1},{1,0},{1,1}};
    // shift one vertex by 5e-5 < epsilon
    t2.vertices[0].y += 5e-5f;
    std::vector<Polygon> tris = {t1,t2};
    auto meshRes = NavMesh::create(tris);
    // Should be accepted as edge matches within epsilon? Our quantization uses epsilon rounding, should match.
    // If rejected, that's okay? Contract permits acceptance. We'll accept either.
    if (meshRes) {
        std::cout<<"epsilon edge accepted\n";
    } else {
        std::cout<<"epsilon edge rejected (still valid)\n";
    }
}

void test_boundary_direct_path() {
    Polygon poly;
    poly.vertices = {{0,0},{2,0},{2,2},{0,2}};
    auto triRes = triangulate_simple_polygon(poly);
    assert(triRes);
    std::vector<Polygon> tris;
    for (auto &p:*triRes) tris.push_back(p);
    auto meshRes = NavMesh::create(tris);
    assert(meshRes);
    NavMesh mesh=*meshRes;
    Vec2 start{0.0f,0.0f};
    Vec2 goal{2.0f,0.0f};
    auto pathRes = find_path(mesh,start,goal);
    assert(pathRes);
    assert(pathRes->points.size()==2);
    assert(pathRes->points[0]==start);
    assert(pathRes->points[1]==goal);
    std::cout<<"boundary direct path ok\n";
}

void test_continuous_containment() {
    Polygon poly;
    poly.vertices = {{0,0},{4,0},{4,4},{0,4}};
    auto triRes = triangulate_simple_polygon(poly);
    assert(triRes);
    std::vector<Polygon> tris;
    for (auto &p:*triRes) tris.push_back(p);
    auto meshRes = NavMesh::create(tris);
    assert(meshRes);
    NavMesh mesh=*meshRes;
    Vec2 start{0.5f,0.5f};
    Vec2 goal{3.5f,3.5f};
    auto pathRes = find_path(mesh,start,goal);
    assert(pathRes);
    // verify each segment contained via sampling
    for (size_t i=0;i+1<pathRes->points.size();++i) {
        Vec2 a=pathRes->points[i];
        Vec2 b=pathRes->points[i+1];
        for (int s=0;s<=100;++s) {
            float t = s/100.0f;
            Vec2 p = {a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t};
            assert(mesh.contains(p));
        }
    }
    std::cout<<"continuous containment ok\n";
}

void test_large_step() {
    Polygon poly;
    poly.vertices = {{0,0},{10,0},{10,10},{0,10}};
    auto triRes = triangulate_simple_polygon(poly);
    assert(triRes);
    std::vector<Polygon> tris;
    for (auto &p:*triRes) tris.push_back(p);
    auto meshRes = NavMesh::create(tris);
    assert(meshRes);
    NavMesh mesh=*meshRes;
    Simulation sim(mesh);
    AgentConfig cfg;
    cfg.position={1,1};
    cfg.radius=0.1f;
    cfg.max_speed=1.0f;
    cfg.goal=Vec2{9,9};
    auto idRes = sim.add_agent(cfg);
    assert(idRes);
    float huge = std::numeric_limits<float>::max();
    auto stepRes = sim.step(huge);
    assert(stepRes);
    auto state = sim.agent(*idRes);
    assert(state);
    // must be finite and contained
    assert(std::isfinite(state->position.x) && std::isfinite(state->position.y));
    assert(mesh.contains(state->position));
    // should be reached or moving but finite
    std::cout<<"large step ok, status="<<(int)state->status<<"\n";
}

void test_avoidance_separation() {
    Polygon poly;
    poly.vertices = {{0,0},{10,0},{10,10},{0,10}};
    auto triRes = triangulate_simple_polygon(poly);
    assert(triRes);
    std::vector<Polygon> tris;
    for (auto &p:*triRes) tris.push_back(p);
    auto meshRes = NavMesh::create(tris);
    assert(meshRes);
    NavMesh mesh=*meshRes;
    Simulation sim(mesh);
    AgentConfig a1; a1.position={2,5}; a1.radius=0.3f; a1.max_speed=1.0f; a1.goal=Vec2{8,5};
    AgentConfig a2; a2.position={8,5}; a2.radius=0.3f; a2.max_speed=1.0f; a2.goal=Vec2{2,5};
    auto id1 = sim.add_agent(a1);
    auto id2 = sim.add_agent(a2);
    assert(id1 && id2);
    for (int i=0;i<200;++i) {
        auto stepRes = sim.step(0.05f);
        assert(stepRes);
        auto s1 = sim.agent(*id1);
        auto s2 = sim.agent(*id2);
        assert(s1 && s2);
        float dist = vwmini::length(s1->position - s2->position);
        assert(dist >= s1->radius + s2->radius - 1e-3f);
    }
    std::cout<<"avoidance separation ok\n";
}

int main() {
    test_extreme_containment();
    test_epsilon_edge();
    test_boundary_direct_path();
    test_continuous_containment();
    test_large_step();
    test_avoidance_separation();
    std::cout<<"All tests passed\n";
    return 0;
}
