#include <vwmini/nav_mesh.hpp>

#include "vwmini/mesh_priv.hpp"

#include <memory>
#include <utility>

namespace vwmini {

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl)) {}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles)
{
    auto mesh = detail::build_mesh_impl(triangles);
    if (!mesh)
        return std::unexpected(mesh.error());
    return NavMesh(std::make_shared<const Impl>(Impl{std::move(*mesh)}));
}

bool NavMesh::contains(Vec2 point) const noexcept
{
    return detail::mesh_contains(m_impl->data, point);
}

std::size_t NavMesh::cell_count() const noexcept
{
    return m_impl->data.cells.size();
}

} // namespace vwmini
