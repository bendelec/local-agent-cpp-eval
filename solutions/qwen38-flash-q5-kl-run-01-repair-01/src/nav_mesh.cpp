#include <vwmini/nav_mesh.hpp>

#include "mesh_topology.hpp"
#include "nav_mesh_impl.hpp"

#include <memory>
#include <utility>

namespace vwmini
{

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl)) {}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles)
{
    auto topology = detail::MeshTopology::build(triangles);
    if (!topology.has_value())
    {
        return std::unexpected(topology.error());
    }
    auto impl = std::make_shared<const Impl>(Impl{std::move(*topology)});
    return NavMesh{std::move(impl)};
}

bool NavMesh::contains(Vec2 point) const noexcept
{
    return m_impl->topology.contains(point);
}

std::size_t NavMesh::cell_count() const noexcept
{
    return m_impl->topology.cell_count();
}

} // namespace vwmini
