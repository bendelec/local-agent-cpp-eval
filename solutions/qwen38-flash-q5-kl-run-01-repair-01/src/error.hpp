#pragma once

// Single place that builds public `Error` values, so code/message pairing lives in one
// place. Modules that can reject caller input (`triangulate_outline`, `MeshTopology::build`,
// `find_path`, `Simulation`) return `Result<T>` themselves; helpers with only a local failure
// to signal return `std::optional`/`bool`.

#include <vwmini/geometry.hpp>

#include <string>

namespace vwmini::detail
{

[[nodiscard]] inline Error make_error(ErrorCode code, std::string message)
{
    return Error{code, std::move(message)};
}

} // namespace vwmini::detail
