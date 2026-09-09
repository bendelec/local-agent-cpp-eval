#include "vwmini/internal/agent_id.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>

// White-box unit tests for the internal id allocator. Exhaustion is an edge
// the public Simulation API cannot reach within a test lifetime (it would take
// ~4 billion add_agent calls), so it is verified against the component that
// owns the counter.

namespace {

using vwmini::internal::AgentIdAllocator;

constexpr std::uint32_t kMaxId = std::numeric_limits<std::uint32_t>::max();

TEST(AgentIdAllocator, HandsOutSequentialNonZeroIds)
{
    AgentIdAllocator allocator;
    EXPECT_EQ(allocator.acquire(), 1u);
    EXPECT_EQ(allocator.acquire(), 2u);
    EXPECT_EQ(allocator.acquire(), 3u);
}

TEST(AgentIdAllocator, ReportsExhaustionInsteadOfReusingIds)
{
    AgentIdAllocator allocator(kMaxId);
    // The last valid id is handed out normally (zero is never a valid id).
    EXPECT_EQ(allocator.acquire(), kMaxId);
    // Afterwards acquire reports exhaustion forever; wrapping back to 1 would
    // silently alias a possibly-live identity and break uniqueness (SIM-013).
    EXPECT_EQ(allocator.acquire(), std::nullopt);
    EXPECT_EQ(allocator.acquire(), std::nullopt);
}

TEST(AgentIdAllocator, ZeroStartIsAlreadyExhausted)
{
    // Zero is the exhaustion sentinel: an allocator started at zero has no ids
    // left and never hands out the invalid id 0.
    AgentIdAllocator allocator(0);
    EXPECT_EQ(allocator.acquire(), std::nullopt);
}

} // namespace
