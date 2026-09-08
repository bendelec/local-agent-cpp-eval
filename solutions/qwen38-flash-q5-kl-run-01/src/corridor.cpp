#include "corridor.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace vwmini::detail
{
namespace
{

constexpr std::size_t kNoCell = std::numeric_limits<std::size_t>::max();

[[nodiscard]] bool contains_cell(std::span<const std::size_t> cells, std::size_t cell)
{
    for (const std::size_t candidate : cells)
    {
        if (candidate == cell)
        {
            return true;
        }
    }
    return false;
}

[[nodiscard]] std::vector<std::size_t> reconstruct(const std::vector<std::size_t> &parent,
                                                   std::size_t last)
{
    std::vector<std::size_t> route;
    for (std::size_t cell = last; cell != kNoCell; cell = parent[cell])
    {
        route.push_back(cell);
    }
    std::reverse(route.begin(), route.end());
    return route;
}

} // namespace

std::vector<std::size_t> find_corridor(const MeshTopology &mesh,
                                       std::span<const std::size_t> starts,
                                       std::span<const std::size_t> goals)
{
    const std::size_t count = mesh.cell_count();
    std::vector<float> cost(count, std::numeric_limits<float>::infinity());
    std::vector<std::size_t> parent(count, kNoCell);
    std::vector<bool> settled(count, false);
    for (const std::size_t start : starts)
    {
        cost[start] = 0.0f;
    }

    for (;;)
    {
        // Cheapest unsettled cell; equal costs resolve to the lowest index.
        std::size_t current = kNoCell;
        float cheapest = std::numeric_limits<float>::infinity();
        for (std::size_t cell = 0; cell < count; ++cell)
        {
            if (!settled[cell] && cost[cell] < cheapest)
            {
                cheapest = cost[cell];
                current = cell;
            }
        }
        if (current == kNoCell)
        {
            return {}; // No reachable goal cell.
        }
        if (contains_cell(goals, current))
        {
            return reconstruct(parent, current);
        }
        settled[current] = true;

        for (const std::size_t next : mesh.neighbours(current))
        {
            if (settled[next])
            {
                continue;
            }
            const float step = length(mesh.cell(current).centroid() - mesh.cell(next).centroid());
            const float candidate = cost[current] + step;
            if (candidate < cost[next])
            {
                cost[next] = candidate;
                parent[next] = current;
            }
        }
    }
}

} // namespace vwmini::detail
