#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <vector>

namespace vwmini {
namespace {

constexpr float EPS = 1e-4f;
constexpr float EPS2 = EPS * EPS;

inline bool isFiniteVec(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }
inline bool isFiniteFloat(float f) { return std::isfinite(f); }

static inline double cross_d(Vec2 a, Vec2 b) { return double(a.x)*b.y - double(a.y)*b.x; }
static inline double dot_d(Vec2 a, Vec2 b) { return double(a.x)*b.x + double(a.y)*b.y; }

inline float distPointSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    double ab2 = dot_d(ab,ab);
    if (ab2 <= 1e-12) return length(p - a);
    double t = dot_d(p - a, ab) / ab2;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
    Vec2 proj = { float(a.x + ab.x * t), float(a.y + ab.y * t) };
    return length(p - proj);
}

int orientation(Vec2 a, Vec2 b, Vec2 c) {
    double v = cross_d(b - a, c - a);
    const double tol = 1e-9;
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
    double a = 0;
    size_t n = v.size();
    for (size_t i=0;i<n;++i) {
        size_t j=(i+1)%n;
        a += double(v[i].x)*v[j].y - double(v[j].x)*v[i].y;
    }
    return float(a*0.5);
}

bool pointInTriangleStrict(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
    double c1 = cross_d(b - a, p - a);
    double c2 = cross_d(c - b, p - b);
    double c3 = cross_d(a - c, p - c);
    const double tol = 1e-7;
    return c1 > tol && c2 > tol && c3 > tol;
}

// quantize for epsilon edge matching
struct QKey {
    long long x,y;
    bool operator==(const QKey& o) const noexcept { return x==o.x && y==o.y; }
};
struct QHash {
    size_t operator()(QKey const& k) const noexcept {
        return std::hash<long long>{}(k.x* 31LL + k.y);
    }
};
inline QKey quantize(Vec2 v) {
    long long qx = llround(v.x / EPS);
    long long qy = llround(v.y / EPS);
    return {qx,qy};
}
struct EdgeQKey {
    QKey a,b;
    bool operator==(const EdgeQKey& o) const noexcept { return a==o.a && b==o.b; }
};
struct EdgeQHash {
    size_t operator()(EdgeQKey const& k) const noexcept {
        return QHash{}(k.a) ^ (QHash{}(k.b)<<1);
    }
};
inline EdgeQKey makeEdgeQ(Vec2 a, Vec2 b) {
    QKey qa = quantize(a);
    QKey qb = quantize(b);
    if (qa.x > qb.x || (qa.x==qb.x && qa.y > qb.y)) std::swap(qa,qb);
    return {qa,qb};
}

} // namespace

float length(Vec2 value) noexcept {
    double d = double(value.x)*value.x + double(value.y)*value.y;
    return float(std::sqrt(d));
}
Vec2 normalized(Vec2 value) noexcept {
    float len = length(value);
    if (len <= 0.0f) return {0,0};
    return {value.x/len, value.y/len};
}

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon) {
    const auto& verts = polygon.vertices;
    for (auto &v : verts) if (!isFiniteVec(v)) return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite"});
    size_t n = verts.size();
    if (n < 3) return std::unexpected(Error{ErrorCode::InvalidMesh, "too few"});
    for (size_t i=0;i<n;++i) if (verts[i]==verts[(i+1)%n]) return std::unexpected(Error{ErrorCode::InvalidMesh, "duplicate"});
    for (size_t i=0;i<n;++i) {
        size_t i2=(i+1)%n;
        for (size_t j=i+1;j<n;++j) {
            size_t j2=(j+1)%n;
            if (i==j || i2==j || j2==i) continue;
            if (segmentsIntersect(verts[i],verts[i2],verts[j],verts[j2]))
                return std::unexpected(Error{ErrorCode::InvalidMesh, "self-intersection"});
        }
    }
    float area = signedPolygonArea(verts);
    if (area <= 0.0f) return std::unexpected(Error{ErrorCode::InvalidMesh, "non-CCW"});
    std::vector<size_t> idx(n);
    for (size_t i=0;i<n;++i) idx[i]=i;
    std::vector<Polygon> out;
    auto pointInTri = [&](Vec2 p, Vec2 a, Vec2 b, Vec2 c)->bool {
        double c1 = cross_d(b-a, p-a);
        double c2 = cross_d(c-b, p-b);
        double c3 = cross_d(a-c, p-c);
        return c1 >= -1e-9 && c2 >= -1e-9 && c3 >= -1e-9;
    };
    while (idx.size() > 3) {
        bool earFound = false;
        size_t m = idx.size();
        for (size_t i=0;i<m;++i) {
            size_t prev = idx[(i+m-1)%m];
            size_t cur = idx[i];
            size_t next = idx[(i+1)%m];
            Vec2 a = verts[prev], b = verts[cur], c = verts[next];
            double triArea2 = std::fabs(cross_d(b-a, c-a));
            if (triArea2 <= EPS2) continue;
            if (cross_d(b-a, c-a) <= 0) continue;
            bool hasInside = false;
            for (size_t j=0;j<m;++j) {
                size_t vi = idx[j];
                if (vi==prev || vi==cur || vi==next) continue;
                Vec2 p = verts[vi];
                if (pointInTri(p,a,b,c)) {
                    if (pointOnSegmentEps(p,a,b) || pointOnSegmentEps(p,b,c) || pointOnSegmentEps(p,c,a)) continue;
                    hasInside = true; break;
                }
            }
            if (!hasInside) {
                Polygon tri; tri.vertices = {a,b,c};
                out.push_back(std::move(tri));
                idx.erase(idx.begin()+i);
                earFound = true;
                break;
            }
        }
        if (!earFound) return std::unexpected(Error{ErrorCode::InvalidMesh, "ear clipping failed"});
    }
    if (idx.size()==3) {
        Vec2 a = verts[idx[0]], b = verts[idx[1]], c = verts[idx[2]];
        Polygon tri;
        if (cross_d(b-a, c-a) > 0) tri.vertices = {a,b,c};
        else tri.vertices = {a,c,b};
        out.push_back(std::move(tri));
    }
    return out;
}

struct NavMesh::Impl {
    struct Triangle { Vec2 v[3]; };
    std::vector<Triangle> tris;
    std::vector<std::pair<Vec2,Vec2>> boundaryEdges;
};

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl)) {}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles) {
    if (triangles.empty()) return std::unexpected(Error{ErrorCode::InvalidMesh, "empty"});
    std::vector<NavMesh::Impl::Triangle> tris;
    tris.reserve(triangles.size());
    for (auto &poly : triangles) {
        if (poly.vertices.size()!=3) return std::unexpected(Error{ErrorCode::InvalidMesh, "non-triangle"});
        for (auto &v : poly.vertices) if (!isFiniteVec(v)) return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite"});
        Vec2 a = poly.vertices[0], b = poly.vertices[1], c = poly.vertices[2];
        double area2 = cross_d(b-a, c-a);
        if (area2 <= 0) return std::unexpected(Error{ErrorCode::InvalidMesh, "clockwise"});
        if (std::fabs(area2) <= EPS*EPS) return std::unexpected(Error{ErrorCode::InvalidMesh, "degenerate"});
        NavMesh::Impl::Triangle t; t.v[0]=a; t.v[1]=b; t.v[2]=c;
        tris.push_back(t);
    }
    size_t n = tris.size();
    std::unordered_map<EdgeQKey, std::vector<size_t>, EdgeQHash> edgeMap;
    auto addEdge = [&](size_t ti, int i, int j){
        Vec2 a = tris[ti].v[i], b = tris[ti].v[j];
        EdgeQKey key = makeEdgeQ(a,b);
        edgeMap[key].push_back(ti);
    };
    for (size_t i=0;i<n;++i) { addEdge(i,0,1); addEdge(i,1,2); addEdge(i,2,0); }
    for (auto &kv : edgeMap) if (kv.second.size() > 2) return std::unexpected(Error{ErrorCode::InvalidMesh, "non-manifold"});
    for (size_t i=0;i<n;++i) {
        for (size_t j=i+1;j<n;++j) {
            auto &ti = tris[i];
            auto &tj = tris[j];
            for (int k=0;k<3;++k) {
                Vec2 p = ti.v[k];
                if (pointInTriangleStrict(p, tj.v[0], tj.v[1], tj.v[2])) return std::unexpected(Error{ErrorCode::InvalidMesh, "overlap"});
            }
            for (int k=0;k<3;++k) {
                Vec2 p = tj.v[k];
                if (pointInTriangleStrict(p, ti.v[0], ti.v[1], ti.v[2])) return std::unexpected(Error{ErrorCode::InvalidMesh, "overlap"});
            }
            for (int ei=0;ei<3;++ei) {
                Vec2 a1 = ti.v[ei], a2 = ti.v[(ei+1)%3];
                for (int ej=0;ej<3;++ej) {
                    Vec2 b1 = tj.v[ej], b2 = tj.v[(ej+1)%3];
                    bool shared = (a1==b1)||(a1==b2)||(a2==b1)||(a2==b2);
                    if (shared) continue;
                    if (segmentsIntersect(a1,a2,b1,b2)) return std::unexpected(Error{ErrorCode::InvalidMesh, "edge intersect"});
                    if (pointOnSegmentEps(ti.v[ei], b1,b2) || pointOnSegmentEps(ti.v[(ei+1)%3], b1,b2)) {
                        // already handled shared, so this is T-junction
                        return std::unexpected(Error{ErrorCode::InvalidMesh, "T-junction"});
                    }
                }
            }
        }
    }
    auto impl = std::make_shared<Impl>();
    impl->tris = std::move(tris);
    for (auto &kv : edgeMap) if (kv.second.size()==1) {
        // retrieve original vertices for boundary edge: need actual Vec2
        // we lost original, so recompute by scanning tris
        // Simpler: rebuild boundary by scanning tris again
    }
    // rebuild boundary edges with actual vertices
    std::unordered_map<EdgeQKey, std::pair<Vec2,Vec2>, EdgeQHash> edgeRep;
    for (size_t i=0;i<impl->tris.size();++i) {
        auto &t = impl->tris[i];
        for (int e=0;e<3;++e) {
            Vec2 a = t.v[e], b = t.v[(e+1)%3];
            EdgeQKey key = makeEdgeQ(a,b);
            auto it = edgeRep.find(key);
            if (it==edgeRep.end()) edgeRep[key] = {a,b};
            else {
                // edge already seen, remove from boundary later
            }
        }
    }
    // Count occurrences using edgeMap
    for (auto &kv : edgeMap) {
        if (kv.second.size()==1) {
            size_t ti = kv.second[0];
            auto &t = impl->tris[ti];
            // find which edge matches key
            for (int e=0;e<3;++e) {
                Vec2 a = t.v[e], b = t.v[(e+1)%3];
                if (makeEdgeQ(a,b)==kv.first) {
                    impl->boundaryEdges.push_back({a,b});
                    break;
                }
            }
        }
    }
    return NavMesh(impl);
}

bool NavMesh::contains(Vec2 point) const noexcept {
    if (!isFiniteVec(point)) return false;
    for (auto &t : m_impl->tris) {
        if (pointInTriangleStrict(point, t.v[0], t.v[1], t.v[2])) return true;
        for (int i=0;i<3;++i) if (distPointSegment(point, t.v[i], t.v[(i+1)%3]) <= EPS) return true;
    }
    return false;
}
std::size_t NavMesh::cell_count() const noexcept { return m_impl->tris.size(); }

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal) {
    if (!isFiniteVec(start) || !isFiniteVec(goal)) return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite"});
    if (!mesh.contains(start) || !mesh.contains(goal)) return std::unexpected(Error{ErrorCode::OutsideMesh, "outside"});
    if (start == goal) return Path{{start}};
    auto &tris = mesh.m_impl->tris;
    auto segmentContained = [&](Vec2 a, Vec2 b)->bool {
        if (!mesh.contains(a) || !mesh.contains(b)) return false;
        // collect t values where segment intersects triangle edges
        std::vector<double> ts;
        ts.push_back(0.0);
        ts.push_back(1.0);
        Vec2 r = b - a;
        double rlen2 = dot_d(r,r);
        if (rlen2 < 1e-12) return true;
        for (auto &t : tris) {
            for (int e=0;e<3;++e) {
                Vec2 e0 = t.v[e], e1 = t.v[(e+1)%3];
                Vec2 s = e1 - e0;
                double rxs = cross_d(r,s);
                if (std::fabs(rxs) < 1e-12) continue;
                Vec2 qp = e0 - a;
                double tpar = cross_d(qp, s) / rxs;
                double upar = cross_d(qp, r) / rxs;
                if (tpar >= -1e-9 && tpar <= 1+1e-9 && upar >= -1e-9 && upar <= 1+1e-9) {
                    ts.push_back(tpar);
                }
            }
        }
        std::sort(ts.begin(), ts.end());
        const double tol = 1e-9;
        for (size_t i=0;i+1<ts.size();++i) {
            double t0 = ts[i], t1 = ts[i+1];
            if (t1 - t0 < tol) continue;
            double tm = (t0 + t1) * 0.5;
            Vec2 p = { float(a.x + r.x * tm), float(a.y + r.y * tm) };
            if (!mesh.contains(p)) return false;
        }
        return true;
    };
    if (segmentContained(start, goal)) return Path{{start, goal}};
    auto findContaining = [&](Vec2 p)->int {
        for (size_t i=0;i<tris.size();++i) {
            auto &t = tris[i];
            if (pointInTriangleStrict(p, t.v[0], t.v[1], t.v[2])) return (int)i;
            for (int e=0;e<3;++e) if (distPointSegment(p, t.v[e], t.v[(e+1)%3]) <= EPS) return (int)i;
        }
        return -1;
    };
    int sIdx = findContaining(start);
    int gIdx = findContaining(goal);
    if (sIdx < 0 || gIdx < 0) return std::unexpected(Error{ErrorCode::OutsideMesh, "not found"});
    if (sIdx == gIdx) return Path{{start, goal}};
    size_t n = tris.size();
    std::vector<std::vector<int>> adj(n);
    std::unordered_map<EdgeQKey, std::vector<int>, EdgeQHash> emap;
    for (size_t i=0;i<n;++i) {
        auto &t = tris[i];
        for (int e=0;e<3;++e) {
            Vec2 a = t.v[e], b = t.v[(e+1)%3];
            EdgeQKey k = makeEdgeQ(a,b);
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
        return {(t.v[0].x + t.v[1].x + t.v[2].x)/3.0f, (t.v[0].y + t.v[1].y + t.v[2].y)/3.0f};
    };
    const float INF = 1e30f;
    std::vector<float> dist(n, INF);
    std::vector<int> prev(n, -1);
    using PQ = std::pair<float,int>;
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
    dist[sIdx]=0; pq.emplace(0, sIdx);
    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (d > dist[u] + 1e-6f) continue;
        if (u == gIdx) break;
        for (int v : adj[u]) {
            float w = length(centroid(u) - centroid(v));
            float nd = d + w;
            if (nd < dist[v] - 1e-9f) {
                dist[v] = nd;
                prev[v] = u;
                pq.emplace(nd, v);
            }
        }
    }
    if (dist[gIdx]==INF) return std::unexpected(Error{ErrorCode::NoPath, "disconnected"});
    std::vector<int> triPath;
    for (int cur=gIdx; cur!=-1; cur=prev[cur]) triPath.push_back(cur);
    std::reverse(triPath.begin(), triPath.end());
    std::vector<Vec2> points;
    points.push_back(start);
    for (size_t i=0;i+1<triPath.size();++i) {
        int aIdx = triPath[i], bIdx = triPath[i+1];
        auto &ta = tris[aIdx]; auto &tb = tris[bIdx];
        Vec2 shared[2]; bool found=false;
        for (int ea=0; ea<3 && !found; ++ea) {
            Vec2 a1 = ta.v[ea], a2 = ta.v[(ea+1)%3];
            for (int eb=0; eb<3; ++eb) {
                Vec2 b1 = tb.v[eb], b2 = tb.v[(eb+1)%3];
                if ((distPointSegment(a1,b1,b2) <= EPS && distPointSegment(a2,b1,b2) <= EPS) ||
                    (distPointSegment(b1,a1,a2) <= EPS && distPointSegment(b2,a1,a2) <= EPS)) {
                    // find common edge endpoints within epsilon
                    // approximate by taking a1,a2
                    shared[0]=a1; shared[1]=a2; found=true; break;
                }
            }
        }
        if (found) { points.push_back(shared[0]); points.push_back(shared[1]); }
    }
    points.push_back(goal);
    bool changed=true;
    while (changed) {
        changed=false;
        for (size_t i=0;i+1<points.size();++i) {
            for (size_t j=points.size()-1;j>i+1;--j) {
                if (segmentContained(points[i], points[j])) {
                    points.erase(points.begin()+i+1, points.begin()+j);
                    changed=true; break;
                }
            }
            if (changed) break;
        }
    }
    std::vector<Vec2> filtered;
    for (auto &p: points) if (filtered.empty() || length(p - filtered.back()) > EPS) filtered.push_back(p);
    return Path{filtered};
}

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
    Impl(NavMesh m): mesh(std::move(m)) {}
};

Simulation::Simulation(NavMesh mesh): m_impl(std::make_unique<Impl>(std::move(mesh))) {}
Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

namespace {
bool validateArrivalRadius(float r){ if (!std::isfinite(r)) return false; if (r!=-1.0f && r<0.0f) return false; return true; }
}

Result<AgentId> Simulation::add_agent(const AgentConfig& cfg) {
    if (!isFiniteVec(cfg.position) || !isFiniteFloat(cfg.radius) || !isFiniteFloat(cfg.max_speed)) return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite"});
    if (cfg.radius <= 0.0f || cfg.max_speed <= 0.0f) return std::unexpected(Error{ErrorCode::InvalidArgument, "non-positive"});
    if (!validateArrivalRadius(cfg.arrival_radius)) return std::unexpected(Error{ErrorCode::InvalidArgument, "arrival radius"});
    if (!m_impl->mesh.contains(cfg.position)) return std::unexpected(Error{ErrorCode::OutsideMesh, "position outside"});
    if (cfg.goal) {
        Vec2 g=*cfg.goal;
        if (!isFiniteVec(g)) return std::unexpected(Error{ErrorCode::InvalidArgument, "goal non-finite"});
        if (!m_impl->mesh.contains(g)) return std::unexpected(Error{ErrorCode::OutsideMesh, "goal outside"});
    }
    AgentId id{ m_impl->next_id++ };
    auto &impl=*m_impl;
    Impl::AgentData ag;
    ag.id=id; ag.pos=cfg.position; ag.radius=cfg.radius; ag.max_speed=cfg.max_speed; ag.goal=cfg.goal;
    ag.arrival_radius_eff = (cfg.arrival_radius==-1.0f)? cfg.radius : cfg.arrival_radius;
    if (ag.goal) {
        auto pr = find_path(impl.mesh, ag.pos, *ag.goal);
        if (pr) {
            float d = length(*ag.goal - ag.pos);
            if (d <= ag.arrival_radius_eff + EPS) { ag.status=AgentStatus::Reached; ag.vel={0,0}; }
            else { ag.status=AgentStatus::Moving; ag.route=pr->points; ag.route_idx=0; }
        } else {
            if (pr.error().code==ErrorCode::NoPath) { ag.status=AgentStatus::NoPath; ag.vel={0,0}; }
            else ag.status=AgentStatus::Idle;
        }
    } else ag.status=AgentStatus::Idle;
    impl.agents.push_back(std::move(ag));
    return id;
}

Result<void> Simulation::remove_agent(AgentId id) {
    auto &agents=m_impl->agents;
    auto it=std::find_if(agents.begin(),agents.end(),[&](auto &a){return a.id==id;});
    if (it==agents.end()) return std::unexpected(Error{ErrorCode::NotFound,"not found"});
    agents.erase(it); return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius) {
    if (!isFiniteVec(goal)) return std::unexpected(Error{ErrorCode::InvalidArgument,"goal non-finite"});
    if (!validateArrivalRadius(arrival_radius)) return std::unexpected(Error{ErrorCode::InvalidArgument,"arrival radius"});
    if (!m_impl->mesh.contains(goal)) return std::unexpected(Error{ErrorCode::OutsideMesh,"goal outside"});
    auto &agents=m_impl->agents;
    auto it=std::find_if(agents.begin(),agents.end(),[&](auto &a){return a.id==id;});
    if (it==agents.end()) return std::unexpected(Error{ErrorCode::NotFound,"not found"});
    auto &ag=*it;
    ag.goal=goal;
    ag.arrival_radius_eff = (arrival_radius==-1.0f)? ag.radius : arrival_radius;
    auto pr = find_path(m_impl->mesh, ag.pos, goal);
    if (!pr) {
        if (pr.error().code==ErrorCode::NoPath) { ag.status=AgentStatus::NoPath; ag.vel={0,0}; ag.route.clear(); ag.route_idx=0; return {}; }
        return std::unexpected(pr.error());
    }
    float d = length(goal - ag.pos);
    if (d <= ag.arrival_radius_eff + EPS) { ag.status=AgentStatus::Reached; ag.vel={0,0}; ag.route.clear(); ag.route_idx=0; }
    else { ag.status=AgentStatus::Moving; ag.route=pr->points; ag.route_idx=0; }
    return {};
}

Result<void> Simulation::clear_goal(AgentId id) {
    auto &agents=m_impl->agents;
    auto it=std::find_if(agents.begin(),agents.end(),[&](auto &a){return a.id==id;});
    if (it==agents.end()) return std::unexpected(Error{ErrorCode::NotFound,"not found"});
    auto &ag=*it;
    ag.goal.reset(); ag.status=AgentStatus::Idle; ag.vel={0,0}; ag.route.clear(); ag.route_idx=0;
    return {};
}

Result<void> Simulation::step(float seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0f) return std::unexpected(Error{ErrorCode::InvalidArgument,"seconds"});
    if (seconds <= 0.0f) return {};
    auto &impl=*m_impl;
    float remaining = seconds;
    const float maxAvoidDt = 0.05f;
    // fast-forward check
    bool anyMoving = false;
    for (auto &ag: impl.agents) if (ag.status==AgentStatus::Moving) { anyMoving=true; break; }
    if (!anyMoving) return {};
    while (remaining > 1e-7f) {
        // compute time to next waypoint
        float minTime = remaining;
        bool hasWaypoint = false;
        for (auto &ag: impl.agents) {
            if (ag.status!=AgentStatus::Moving || ag.route.empty()) continue;
            hasWaypoint=true;
            size_t idx = ag.route_idx;
            if (idx >= ag.route.size()) continue;
            Vec2 target = ag.route[idx];
            float d = length(target - ag.pos);
            if (d > EPS) {
                float t = d / ag.max_speed;
                if (t < minTime) minTime = t;
            }
        }
        if (!hasWaypoint) break;
        float dt = std::min(remaining, minTime);
        if (dt > maxAvoidDt) dt = maxAvoidDt;
        // snapshot positions
        std::vector<Vec2> snapPos;
        snapPos.reserve(impl.agents.size());
        for (auto &ag: impl.agents) snapPos.push_back(ag.pos);
        // desired velocities
        std::vector<Vec2> desired(impl.agents.size());
        for (size_t i=0;i<impl.agents.size();++i) {
            auto &ag = impl.agents[i];
            if (ag.status!=AgentStatus::Moving || ag.route.empty()) { desired[i]={0,0}; continue; }
            size_t idx = ag.route_idx;
            if (idx >= ag.route.size()) { desired[i]={0,0}; continue; }
            Vec2 target = ag.route[idx];
            Vec2 to = target - ag.pos;
            float d = length(to);
            if (d > EPS) desired[i] = normalized(to) * ag.max_speed;
            else desired[i] = {0,0};
        }
        // choose velocities with simple collision avoidance
        std::vector<Vec2> chosen(impl.agents.size());
        for (size_t i=0;i<impl.agents.size();++i) {
            auto &ag = impl.agents[i];
            Vec2 best = desired[i];
            Vec2 predPos = snapPos[i] + best * dt;
            bool collision = false;
            for (size_t j=0;j<impl.agents.size();++j) if (j!=i) {
                Vec2 otherPred = snapPos[j] + chosen[j] * dt;
                float dist = length(predPos - otherPred);
                if (dist < ag.radius + impl.agents[j].radius - 1e-4f) { collision=true; break; }
            }
            if (collision) {
                // try stop
                best = {0,0};
                predPos = snapPos[i];
                bool stillColl = false;
                for (size_t j=0;j<impl.agents.size();++j) if (j!=i) {
                    Vec2 otherPred = snapPos[j] + chosen[j] * dt;
                    if (length(predPos - otherPred) < ag.radius + impl.agents[j].radius - 1e-4f) { stillColl=true; break; }
                }
                if (stillColl) {
                    // keep desired but accept overlap risk
                }
            }
            // containment check
            if (!impl.mesh.contains(predPos)) {
                // try zero
                best = {0,0};
            }
            chosen[i]=best;
        }
        // apply movement
        for (size_t i=0;i<impl.agents.size();++i) {
            auto &ag = impl.agents[i];
            if (ag.status!=AgentStatus::Moving) { ag.vel={0,0}; continue; }
            Vec2 v = chosen[i];
            ag.vel = v;
            Vec2 newPos = ag.pos + v * dt;
            // waypoint handling
            if (!ag.route.empty()) {
                size_t idx = ag.route_idx;
                if (idx < ag.route.size()) {
                    Vec2 target = ag.route[idx];
                    float d = length(target - ag.pos);
                    float moveDist = length(v) * dt;
                    if (moveDist >= d - EPS) {
                        ag.pos = target;
                        ag.route_idx++;
                        if (ag.goal) {
                            float dg = length(*ag.goal - ag.pos);
                            if (dg <= ag.arrival_radius_eff + EPS) {
                                ag.status=AgentStatus::Reached; ag.vel={0,0}; ag.route.clear(); continue;
                            }
                        }
                        if (ag.route_idx >= ag.route.size()) {
                            ag.status=AgentStatus::Idle; ag.vel={0,0}; ag.route.clear();
                        }
                    } else {
                        ag.pos = newPos;
                    }
                }
            } else {
                ag.pos = newPos;
            }
            if (!impl.mesh.contains(ag.pos)) {
                // revert
                ag.pos = snapPos[i];
                ag.vel = {0,0};
            }
        }
        remaining -= dt;
        if (remaining <= 0) break;
        // check termination
        bool movingNow = false;
        for (auto &ag: impl.agents) if (ag.status==AgentStatus::Moving) { movingNow=true; break; }
        if (!movingNow) break;
    }
    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept {
    for (auto &ag: m_impl->agents) if (ag.id==id) {
        AgentState s; s.position=ag.pos; s.velocity=ag.vel; s.radius=ag.radius; s.max_speed=ag.max_speed; s.goal=ag.goal; s.status=ag.status; return s;
    }
    return std::nullopt;
}
std::size_t Simulation::agent_count() const noexcept { return m_impl->agents.size(); }

} // namespace vwmini
