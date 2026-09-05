#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <queue>

namespace vwmini {
namespace {

// epsilon
constexpr float EPS = 1e-4f;
constexpr float EPS2 = EPS * EPS;

inline bool isFiniteVec(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }
inline bool isFiniteFloat(float f) { return std::isfinite(f); }

inline float distPointSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    float ab2 = vwmini::dot(ab,ab);
    if (ab2 <= 1e-12f) return vwmini::length(p - a);
    float t = vwmini::dot(p - a, ab) / ab2;
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;
    Vec2 proj = a + ab * t;
    return vwmini::length(p - proj);
}

int orientation(Vec2 a, Vec2 b, Vec2 c) {
    float v = vwmini::cross(b - a, c - a);
    const float tol = 1e-9f;
    if (std::fabs(v) <= tol) return 0;
    return v > 0 ? 1 : 2;
}
bool onSegment(Vec2 a, Vec2 b, Vec2 c) {
    return std::min(a.x,c.x)-1e-9f <= b.x && b.x <= std::max(a.x,c.x)+1e-9f &&
           std::min(a.y,c.y)-1e-9f <= b.y && b.y <= std::max(a.y,c.y)+1e-9f;
}
bool segmentsIntersect(Vec2 p1, Vec2 p2, Vec2 q1, Vec2 q2) {
    int o1 = orientation(p1,p2,q1);
    int o2 = orientation(p1,p2,q2);
    int o3 = orientation(q1,q2,p1);
    int o4 = orientation(q1,q2,p2);
    if (o1 != o2 && o3 != o4) return true;
    if (o1==0 && onSegment(p1,q1,p2)) return true;
    if (o2==0 && onSegment(p1,q2,p2)) return true;
    if (o3==0 && onSegment(q1,p1,q2)) return true;
    if (o4==0 && onSegment(q1,p2,q2)) return true;
    return false;
}

bool pointOnSegmentEps(Vec2 p, Vec2 a, Vec2 b) {
    if (distPointSegment(p,a,b) <= EPS) {
        return std::min(a.x,b.x)-EPS <= p.x && p.x <= std::max(a.x,b.x)+EPS &&
               std::min(a.y,b.y)-EPS <= p.y && p.y <= std::max(a.y,b.y)+EPS;
    }
    return false;
}

float signedPolygonArea(const std::vector<Vec2>& v) {
    float a = 0;
    size_t n = v.size();
    for (size_t i=0;i<n;++i) {
        size_t j=(i+1)%n;
        a += v[i].x * v[j].y - v[j].x * v[i].y;
    }
    return a*0.5f;
}

bool pointInTriangleStrict(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
    float c1 = vwmini::cross(b - a, p - a);
    float c2 = vwmini::cross(c - b, p - b);
    float c3 = vwmini::cross(a - c, p - c);
    const float tol = 1e-7f;
    return c1 > tol && c2 > tol && c3 > tol;
}

} // namespace

// geometry
float length(Vec2 value) noexcept {
    return std::sqrt(vwmini::dot(value,value));
}
Vec2 normalized(Vec2 value) noexcept {
    float len = vwmini::length(value);
    if (len <= 0.0f) return {0,0};
    return {value.x/len, value.y/len};
}

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon) {
    const auto& verts = polygon.vertices;
    for (auto &v : verts) {
        if (!isFiniteVec(v)) {
            return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite vertex"});
        }
    }
    size_t n = verts.size();
    if (n < 3) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "too few vertices"});
    }
    for (size_t i=0;i<n;++i) {
        size_t j = (i+1)%n;
        if (verts[i] == verts[j]) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "duplicate consecutive vertices"});
        }
    }
    for (size_t i=0;i<n;++i) {
        size_t i2 = (i+1)%n;
        for (size_t j=i+1;j<n;++j) {
            size_t j2 = (j+1)%n;
            if (i==j || i2==j || j2==i) continue;
            if (segmentsIntersect(verts[i], verts[i2], verts[j], verts[j2])) {
                return std::unexpected(Error{ErrorCode::InvalidMesh, "self-intersection"});
            }
        }
    }
    float area = signedPolygonArea(verts);
    if (area <= 0.0f) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "non-CCW or zero area"});
    }
    std::vector<size_t> idx(n);
    for (size_t i=0;i<n;++i) idx[i]=i;
    std::vector<Polygon> out;
    auto pointInTri = [&](Vec2 p, Vec2 a, Vec2 b, Vec2 c)->bool {
        float c1 = vwmini::cross(b-a, p-a);
        float c2 = vwmini::cross(c-b, p-b);
        float c3 = vwmini::cross(a-c, p-c);
        return c1 >= -1e-9f && c2 >= -1e-9f && c3 >= -1e-9f;
    };
    while (idx.size() > 3) {
        bool earFound = false;
        size_t m = idx.size();
        for (size_t i=0;i<m;++i) {
            size_t prev = idx[(i+m-1)%m];
            size_t cur = idx[i];
            size_t next = idx[(i+1)%m];
            Vec2 a = verts[prev];
            Vec2 b = verts[cur];
            Vec2 c = verts[next];
            float triArea2 = std::fabs(vwmini::cross(b-a, c-a));
            if (triArea2 <= EPS2) continue;
            float orient = vwmini::cross(b-a, c-a);
            if (orient <= 0) continue;
            bool hasInside = false;
            for (size_t j=0;j<m;++j) {
                size_t vi = idx[j];
                if (vi==prev || vi==cur || vi==next) continue;
                Vec2 p = verts[vi];
                if (pointInTri(p,a,b,c)) {
                    if (pointOnSegmentEps(p,a,b) || pointOnSegmentEps(p,b,c) || pointOnSegmentEps(p,c,a)) continue;
                    hasInside = true;
                    break;
                }
            }
            if (!hasInside) {
                Polygon tri;
                tri.vertices = {a,b,c};
                out.push_back(std::move(tri));
                idx.erase(idx.begin()+i);
                earFound = true;
                break;
            }
        }
        if (!earFound) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "ear clipping failed"});
        }
    }
    if (idx.size()==3) {
        Vec2 a = verts[idx[0]];
        Vec2 b = verts[idx[1]];
        Vec2 c = verts[idx[2]];
        float orient = vwmini::cross(b-a, c-a);
        Polygon tri;
        if (orient > 0) tri.vertices = {a,b,c};
        else tri.vertices = {a,c,b};
        out.push_back(std::move(tri));
    }
    return out;
}

// NavMesh
struct NavMesh::Impl {
    struct Triangle { Vec2 v[3]; };
    std::vector<Triangle> tris;
    std::vector<std::pair<Vec2,Vec2>> boundaryEdges;
};

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl)) {}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles) {
    if (triangles.empty()) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "empty mesh"});
    }
    std::vector<NavMesh::Impl::Triangle> tris;
    tris.reserve(triangles.size());
    for (auto &poly : triangles) {
        if (poly.vertices.size() != 3) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "non-triangle"});
        }
        for (auto &v : poly.vertices) {
            if (!isFiniteVec(v)) {
                return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite vertex"});
            }
        }
        Vec2 a = poly.vertices[0];
        Vec2 b = poly.vertices[1];
        Vec2 c = poly.vertices[2];
        float area2 = vwmini::cross(b-a, c-a);
        if (area2 <= 0) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "clockwise triangle"});
        }
        if (std::fabs(area2) <= EPS*EPS) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "degenerate triangle"});
        }
        NavMesh::Impl::Triangle t;
        t.v[0]=a; t.v[1]=b; t.v[2]=c;
        tris.push_back(t);
    }
    size_t n = tris.size();
    struct EdgeKey { Vec2 a,b; bool operator==(const EdgeKey& o) const noexcept { return a==o.a && b==o.b; } };
    struct EdgeHash { size_t operator()(EdgeKey const& k) const noexcept { size_t h1 = std::hash<float>{}(k.a.x) ^ std::hash<float>{}(k.a.y); size_t h2 = std::hash<float>{}(k.b.x) ^ std::hash<float>{}(k.b.y); return h1 ^ (h2<<1); } };
    std::unordered_map<EdgeKey, std::vector<size_t>, EdgeHash> edgeMap;
    auto addEdge = [&](size_t ti, int i, int j){
        Vec2 a = tris[ti].v[i];
        Vec2 b = tris[ti].v[j];
        if (a.x > b.x || (a.x==b.x && a.y > b.y)) std::swap(a,b);
        EdgeKey key{a,b};
        edgeMap[key].push_back(ti);
    };
    for (size_t i=0;i<n;++i) {
        addEdge(i,0,1);
        addEdge(i,1,2);
        addEdge(i,2,0);
    }
    for (auto &kv : edgeMap) {
        if (kv.second.size() > 2) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "non-manifold edge"});
        }
    }
    for (size_t i=0;i<n;++i) {
        for (size_t j=i+1;j<n;++j) {
            auto &ti = tris[i];
            auto &tj = tris[j];
            for (int k=0;k<3;++k) {
                Vec2 p = ti.v[k];
                if (pointInTriangleStrict(p, tj.v[0], tj.v[1], tj.v[2])) {
                    return std::unexpected(Error{ErrorCode::InvalidMesh, "overlap"});
                }
            }
            for (int k=0;k<3;++k) {
                Vec2 p = tj.v[k];
                if (pointInTriangleStrict(p, ti.v[0], ti.v[1], ti.v[2])) {
                    return std::unexpected(Error{ErrorCode::InvalidMesh, "overlap"});
                }
            }
            for (int ei=0;ei<3;++ei) {
                Vec2 a1 = ti.v[ei];
                Vec2 a2 = ti.v[(ei+1)%3];
                for (int ej=0;ej<3;++ej) {
                    Vec2 b1 = tj.v[ej];
                    Vec2 b2 = tj.v[(ej+1)%3];
                    bool shared = (a1==b1)||(a1==b2)||(a2==b1)||(a2==b2);
                    if (shared) continue;
                    if (segmentsIntersect(a1,a2,b1,b2)) {
                        return std::unexpected(Error{ErrorCode::InvalidMesh, "edge intersect"});
                    }
                    if (pointOnSegmentEps(ti.v[ei], b1,b2) || pointOnSegmentEps(ti.v[(ei+1)%3], b1,b2)) {
                        return std::unexpected(Error{ErrorCode::InvalidMesh, "T-junction"});
                    }
                }
            }
        }
    }
    auto impl = std::make_shared<Impl>();
    impl->tris = std::move(tris);
    for (auto &kv : edgeMap) {
        if (kv.second.size()==1) {
            impl->boundaryEdges.push_back({kv.first.a, kv.first.b});
        }
    }
    return NavMesh(impl);
}

bool NavMesh::contains(Vec2 point) const noexcept {
    if (!isFiniteVec(point)) return false;
    for (auto &t : m_impl->tris) {
        if (pointInTriangleStrict(point, t.v[0], t.v[1], t.v[2])) return true;
        for (int i=0;i<3;++i) {
            if (distPointSegment(point, t.v[i], t.v[(i+1)%3]) <= EPS) return true;
        }
    }
    return false;
}

std::size_t NavMesh::cell_count() const noexcept { return m_impl->tris.size(); }

// find_path
Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal) {
    if (!isFiniteVec(start) || !isFiniteVec(goal)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite"});
    }
    if (!mesh.contains(start) || !mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "outside"});
    }
    if (start == goal) {
        return Path{{start}};
    }
    auto &tris = mesh.m_impl->tris;
    auto &boundary = mesh.m_impl->boundaryEdges;
    auto segmentContained = [&](Vec2 a, Vec2 b)->bool {
        if (!mesh.contains(a) || !mesh.contains(b)) return false;
        // boundary intersection test
        for (auto &e : boundary) {
            Vec2 p1=a, p2=b, q1=e.first, q2=e.second;
            Vec2 r = p2 - p1;
            Vec2 s = q2 - q1;
            float rxs = vwmini::cross(r,s);
            if (std::fabs(rxs) < 1e-12f) continue;
            Vec2 qp = q1 - p1;
            float t = vwmini::cross(qp, s) / rxs;
            float u = vwmini::cross(qp, r) / rxs;
            if (t > 1e-6f && t < 1-1e-6f && u >= -1e-6f && u <= 1+1e-6f) {
                return false;
            }
        }
        const int samples = 64;
        for (int i=1;i<samples;++i) {
            float s_ = float(i)/samples;
            Vec2 p = a + (b - a) * s_;
            if (!mesh.contains(p)) return false;
        }
        return true;
    };
    if (segmentContained(start, goal)) {
        return Path{{start, goal}};
    }
    auto findContaining = [&](Vec2 p)->int {
        for (size_t i=0;i<tris.size();++i) {
            auto &t = tris[i];
            if (pointInTriangleStrict(p, t.v[0], t.v[1], t.v[2])) return (int)i;
            for (int e=0;e<3;++e) {
                if (distPointSegment(p, t.v[e], t.v[(e+1)%3]) <= EPS) return (int)i;
            }
        }
        return -1;
    };
    int sIdx = findContaining(start);
    int gIdx = findContaining(goal);
    if (sIdx < 0 || gIdx < 0) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "not found"});
    }
    if (sIdx == gIdx) {
        return Path{{start, goal}};
    }
    size_t n = tris.size();
    std::vector<std::vector<int>> adj(n);
    struct EdgeKey { Vec2 a,b; bool operator==(const EdgeKey& o) const noexcept { return a==o.a && b==o.b; } };
    struct EdgeHash { size_t operator()(EdgeKey const& k) const noexcept { size_t h1 = std::hash<float>{}(k.a.x) ^ std::hash<float>{}(k.a.y); size_t h2 = std::hash<float>{}(k.b.x) ^ std::hash<float>{}(k.b.y); return h1 ^ (h2<<1); } };
    std::unordered_map<EdgeKey, std::vector<int>, EdgeHash> emap;
    for (size_t i=0;i<n;++i) {
        auto &t = tris[i];
        for (int e=0;e<3;++e) {
            Vec2 a = t.v[e];
            Vec2 b = t.v[(e+1)%3];
            if (a.x > b.x || (a.x==b.x && a.y > b.y)) std::swap(a,b);
            EdgeKey k{a,b};
            emap[k].push_back((int)i);
        }
    }
    for (auto &kv : emap) {
        auto &v = kv.second;
        if (v.size()==2) {
            int i=v[0], j=v[1];
            adj[i].push_back(j);
            adj[j].push_back(i);
        }
    }
    auto centroid = [&](size_t i)->Vec2 {
        auto &t = tris[i];
        return {(t.v[0].x + t.v[1].x + t.v[2].x)/3.0f,
                (t.v[0].y + t.v[1].y + t.v[2].y)/3.0f};
    };
    const float INF = 1e30f;
    std::vector<float> dist(n, INF);
    std::vector<int> prev(n, -1);
    using PQ = std::pair<float,int>;
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
    dist[sIdx]=0;
    pq.emplace(0, sIdx);
    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (d > dist[u] + 1e-6f) continue;
        if (u == gIdx) break;
        for (int v : adj[u]) {
            float w = vwmini::length(centroid(u) - centroid(v));
            float nd = d + w;
            if (nd < dist[v] - 1e-9f) {
                dist[v] = nd;
                prev[v] = u;
                pq.emplace(nd, v);
            }
        }
    }
    if (dist[gIdx] == INF) {
        return std::unexpected(Error{ErrorCode::NoPath, "disconnected"});
    }
    std::vector<int> triPath;
    for (int cur=gIdx; cur!=-1; cur=prev[cur]) triPath.push_back(cur);
    std::reverse(triPath.begin(), triPath.end());
    std::vector<Vec2> points;
    points.push_back(start);
    for (size_t i=0;i+1<triPath.size();++i) {
        int aIdx = triPath[i];
        int bIdx = triPath[i+1];
        auto &ta = tris[aIdx];
        auto &tb = tris[bIdx];
        Vec2 shared[2];
        bool found=false;
        for (int ea=0; ea<3 && !found; ++ea) {
            Vec2 a1 = ta.v[ea];
            Vec2 a2 = ta.v[(ea+1)%3];
            for (int eb=0; eb<3; ++eb) {
                Vec2 b1 = tb.v[eb];
                Vec2 b2 = tb.v[(eb+1)%3];
                if ((a1==b1 && a2==b2) || (a1==b2 && a2==b1)) {
                    shared[0]=a1; shared[1]=a2;
                    found=true;
                    break;
                }
            }
        }
        if (found) {
            points.push_back(shared[0]);
            points.push_back(shared[1]);
        }
    }
    points.push_back(goal);
    bool changed=true;
    while (changed) {
        changed=false;
        for (size_t i=0;i+1<points.size();++i) {
            for (size_t j=points.size()-1;j>i+1;--j) {
                if (segmentContained(points[i], points[j])) {
                    points.erase(points.begin()+i+1, points.begin()+j);
                    changed=true;
                    break;
                }
            }
            if (changed) break;
        }
    }
    std::vector<Vec2> filtered;
    for (auto &p : points) {
        if (filtered.empty() || vwmini::length(p - filtered.back()) > EPS) {
            filtered.push_back(p);
        }
    }
    return Path{filtered};
}

// Simulation
struct Simulation::Impl {
    NavMesh mesh;
    struct AgentData {
        AgentId id;
        Vec2 pos;
        Vec2 vel{0,0};
        float radius;
        float max_speed;
        std::optional<Vec2> goal;
        float arrival_radius_eff;
        AgentStatus status = AgentStatus::Idle;
        std::vector<Vec2> route;
        size_t route_idx = 0;
    };
    std::vector<AgentData> agents;
    uint32_t next_id = 1;
    Impl(NavMesh m) : mesh(std::move(m)) {}
};

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh))) {}
Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

namespace {
bool validateArrivalRadius(float r) {
    if (!std::isfinite(r)) return false;
    if (r != -1.0f && r < 0.0f) return false;
    return true;
}
}

Result<AgentId> Simulation::add_agent(const AgentConfig& cfg) {
    if (!isFiniteVec(cfg.position) || !isFiniteFloat(cfg.radius) || !isFiniteFloat(cfg.max_speed)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite"});
    }
    if (cfg.radius <= 0.0f || cfg.max_speed <= 0.0f) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-positive"});
    }
    if (!validateArrivalRadius(cfg.arrival_radius)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "arrival radius"});
    }
    if (!m_impl->mesh.contains(cfg.position)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "position outside"});
    }
    if (cfg.goal.has_value()) {
        Vec2 g = *cfg.goal;
        if (!isFiniteVec(g)) {
            return std::unexpected(Error{ErrorCode::InvalidArgument, "goal non-finite"});
        }
        if (!m_impl->mesh.contains(g)) {
            return std::unexpected(Error{ErrorCode::OutsideMesh, "goal outside"});
        }
    }
    AgentId id{ m_impl->next_id++ };
    auto &impl = *m_impl;
    Impl::AgentData ag;
    ag.id = id;
    ag.pos = cfg.position;
    ag.radius = cfg.radius;
    ag.max_speed = cfg.max_speed;
    ag.goal = cfg.goal;
    float effArr = (cfg.arrival_radius == -1.0f) ? cfg.radius : cfg.arrival_radius;
    ag.arrival_radius_eff = effArr;
    if (ag.goal.has_value()) {
        auto pathRes = find_path(impl.mesh, ag.pos, *ag.goal);
        if (pathRes) {
            float d = vwmini::length(*ag.goal - ag.pos);
            if (d <= ag.arrival_radius_eff + EPS) {
                ag.status = AgentStatus::Reached;
                ag.vel = {0,0};
            } else {
                ag.status = AgentStatus::Moving;
                ag.route = pathRes->points;
                ag.route_idx = 0;
            }
        } else {
            if (pathRes.error().code == ErrorCode::NoPath) {
                ag.status = AgentStatus::NoPath;
                ag.vel = {0,0};
            } else {
                ag.status = AgentStatus::Idle;
            }
        }
    } else {
        ag.status = AgentStatus::Idle;
    }
    impl.agents.push_back(std::move(ag));
    return id;
}

Result<void> Simulation::remove_agent(AgentId id) {
    auto &agents = m_impl->agents;
    auto it = std::find_if(agents.begin(), agents.end(), [&](auto &a){ return a.id == id; });
    if (it == agents.end()) return std::unexpected(Error{ErrorCode::NotFound, "agent not found"});
    agents.erase(it);
    return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius) {
    if (!isFiniteVec(goal)) return std::unexpected(Error{ErrorCode::InvalidArgument, "goal non-finite"});
    if (!validateArrivalRadius(arrival_radius)) return std::unexpected(Error{ErrorCode::InvalidArgument, "arrival radius"});
    if (!m_impl->mesh.contains(goal)) return std::unexpected(Error{ErrorCode::OutsideMesh, "goal outside"});
    auto &agents = m_impl->agents;
    auto it = std::find_if(agents.begin(), agents.end(), [&](auto &a){ return a.id == id; });
    if (it == agents.end()) return std::unexpected(Error{ErrorCode::NotFound, "agent not found"});
    auto &ag = *it;
    ag.goal = goal;
    ag.arrival_radius_eff = (arrival_radius == -1.0f) ? ag.radius : arrival_radius;
    auto pathRes = find_path(m_impl->mesh, ag.pos, goal);
    if (!pathRes) {
        if (pathRes.error().code == ErrorCode::NoPath) {
            ag.status = AgentStatus::NoPath;
            ag.vel = {0,0};
            ag.route.clear();
            ag.route_idx = 0;
            return {};
        }
        return std::unexpected(pathRes.error());
    }
    float d = vwmini::length(goal - ag.pos);
    if (d <= ag.arrival_radius_eff + EPS) {
        ag.status = AgentStatus::Reached;
        ag.vel = {0,0};
        ag.route.clear();
        ag.route_idx = 0;
    } else {
        ag.status = AgentStatus::Moving;
        ag.route = pathRes->points;
        ag.route_idx = 0;
    }
    return {};
}

Result<void> Simulation::clear_goal(AgentId id) {
    auto &agents = m_impl->agents;
    auto it = std::find_if(agents.begin(), agents.end(), [&](auto &a){ return a.id == id; });
    if (it == agents.end()) return std::unexpected(Error{ErrorCode::NotFound, "agent not found"});
    auto &ag = *it;
    ag.goal.reset();
    ag.status = AgentStatus::Idle;
    ag.vel = {0,0};
    ag.route.clear();
    ag.route_idx = 0;
    return {};
}

Result<void> Simulation::step(float seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0f) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "seconds"});
    }
    if (seconds <= 0.0f) return {};
    auto &impl = *m_impl;
    float remaining = seconds;
    const float maxSubstep = 0.05f;
    while (remaining > 1e-7f) {
        float dt = std::min(remaining, maxSubstep);
        remaining -= dt;
        // compute desired velocities
        std::vector<Vec2> newVels(impl.agents.size());
        for (size_t i=0;i<impl.agents.size();++i) {
            auto &ag = impl.agents[i];
            Vec2 desired{0,0};
            if (ag.status == AgentStatus::Moving && !ag.route.empty()) {
                size_t idx = ag.route_idx;
                if (idx < ag.route.size()) {
                    Vec2 target = ag.route[idx];
                    Vec2 to = target - ag.pos;
                    float d = vwmini::length(to);
                    if (d > EPS) {
                        desired = vwmini::normalized(to) * ag.max_speed;
                    }
                }
            }
            // simple avoidance
            Vec2 avoid{0,0};
            for (size_t j=0;j<impl.agents.size();++j) if (j!=i) {
                Vec2 delta = ag.pos - impl.agents[j].pos;
                float dist = vwmini::length(delta);
                float minDist = ag.radius + impl.agents[j].radius;
                if (dist < minDist + 0.5f && dist > 1e-6f) {
                    float strength = 1.0f - dist/(minDist + 0.5f);
                    avoid = avoid + vwmini::normalized(delta) * strength * ag.max_speed;
                }
            }
            Vec2 v = desired + avoid * 0.5f;
            float len = vwmini::length(v);
            if (len > ag.max_speed) v = vwmini::normalized(v) * ag.max_speed;
            newVels[i] = v;
        }
        for (size_t i=0;i<impl.agents.size();++i) {
            auto &ag = impl.agents[i];
            if (ag.status != AgentStatus::Moving) {
                ag.vel = {0,0};
                continue;
            }
            Vec2 v = newVels[i];
            ag.vel = v;
            if (!ag.route.empty()) {
                size_t idx = ag.route_idx;
                if (idx < ag.route.size()) {
                    Vec2 target = ag.route[idx];
                    Vec2 to = target - ag.pos;
                    float distToTarget = vwmini::length(to);
                    float moveDist = vwmini::length(v) * dt;
                    if (moveDist >= distToTarget - EPS) {
                        ag.pos = target;
                        ag.route_idx++;
                        if (ag.goal.has_value()) {
                            float dGoal = vwmini::length(*ag.goal - ag.pos);
                            if (dGoal <= ag.arrival_radius_eff + EPS) {
                                ag.status = AgentStatus::Reached;
                                ag.vel = {0,0};
                                ag.route.clear();
                                continue;
                            }
                        }
                        if (ag.route_idx >= ag.route.size()) {
                            ag.status = AgentStatus::Idle;
                            ag.vel = {0,0};
                            ag.route.clear();
                        }
                    } else {
                        ag.pos = ag.pos + v * dt;
                    }
                }
            } else {
                ag.pos = ag.pos + v * dt;
            }
            if (!impl.mesh.contains(ag.pos)) {
                ag.pos = ag.pos; // keep previous? skip
            }
        }
    }
    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept {
    for (auto &ag : m_impl->agents) {
        if (ag.id == id) {
            AgentState s;
            s.position = ag.pos;
            s.velocity = ag.vel;
            s.radius = ag.radius;
            s.max_speed = ag.max_speed;
            s.goal = ag.goal;
            s.status = ag.status;
            return s;
        }
    }
    return std::nullopt;
}

std::size_t Simulation::agent_count() const noexcept {
    return m_impl->agents.size();
}

} // namespace vwmini
