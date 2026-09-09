#pragma once

// Monotonic agent-id allocation (SIM-005), internal to the library. Split out
// as its own component so the exhaustion edge is unit-testable directly;
// reaching it through the public Simulation API would take billions of
// add_agent calls.

#include <cstdint>
#include <optional>

namespace vwmini::internal {

/// Hands out unique, monotonically increasing, non-zero 32-bit ids. Ids are
/// never reused. Once all 2^32 - 1 non-zero ids have been handed out,
/// `acquire()` reports exhaustion (`std::nullopt`) instead of wrapping around
/// to ids that may still be live; a caller that ignored a wrap would silently
/// break id uniqueness and alias two agents onto one identity.
class AgentIdAllocator {
  public:
    /// `first` is the id returned by the initial `acquire()`. It defaults to
    /// 1; other values exist so tests can start near exhaustion. A `first` of
    /// 0 constructs an already-exhausted allocator.
    explicit AgentIdAllocator(std::uint32_t first = 1) noexcept : m_next(first)
    {
    }

    /// Returns the next id, or `std::nullopt` when the space is exhausted.
    /// Never wraps; exhaustion is permanent (sticky).
    [[nodiscard]] std::optional<std::uint32_t> acquire() noexcept
    {
        if (m_next == 0) { // Zero is the exhaustion sentinel, never a valid id.
            return std::nullopt;
        }
        const std::uint32_t id = m_next;
        ++m_next; // UINT32_MAX + 1 wraps to 0, marking exhaustion.
        return id;
    }

  private:
    /// Next id to hand out; 0 means the id space is exhausted.
    std::uint32_t m_next;
};

} // namespace vwmini::internal
