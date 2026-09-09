#pragma once

// Shared construction of `std::unexpected(Error)` values so every public entry
// point reports failures the same way (NFR-006). Message text is diagnostic only;
// callers and tests depend on `ErrorCode`, never on the string.

#include <vwmini/geometry.hpp>

#include <expected>
#include <string>
#include <utility>

namespace vwmini::internal {

/** Wraps an error code and message into the unexpected half of a `Result`. */
[[nodiscard]] inline std::unexpected<Error> make_error(ErrorCode code, std::string message)
{
    return std::unexpected<Error>(Error{code, std::move(message)});
}

} // namespace vwmini::internal
