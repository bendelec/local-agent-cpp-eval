All three defects are fixed by deletion plus one reused primitive, and every gate passes.

 What changed

 1. Exact segment coverage replaces endpoint+midpoint sampling
 - Deleted sweep_is_inside. Motion now asks MeshTopology::segment_is_contained — the same interval-coverage query that certifies a planned
   route (simulation.cpp).
 - Seam is the narrowest possible: NavMesh gained friend class Simulation; in its private section (include/vwmini/nav_mesh.hpp) and
   simulation.cpp includes the existing nav_mesh_impl.hpp. No public declaration, signature, default, enum, include path or target changed.
 - Unobstructed motion costs one exact query; a stride that doesn't fit is bisected kMotionBisectionSteps (16) times, and every candidate is
   verified before it is taken (coverage along a ray isn't monotone, so bisection returns a verified fraction, not necessarily the largest —
   confirming beats maximising for SIM-012).

 2. Residual accumulation removed — AgentRuntime::residual and all its plumbing are gone. A displacement the stored float position can't
 represent simply doesn't happen, so nothing can inject stale lateral motion after set_goal/clear_goal (the state field no longer exists) and
 per-call/cumulative movement is bounded by max_speed * elapsed.

 3. Range-checked narrowing — Vec2d::to_vec2() (unconditional cast) is replaced by Vec2d::narrow() returning std::optional<Vec2>; finiteness
 and [-FLT_MAX, FLT_MAX] are checked before conversion (NaN fails the same comparison). No unchecked narrowing call site remains.

 Tests (81 total, +3): SimulationTest.MotionNeverTeleportsThroughAGap (the reviewer's exact scenario; probe shows the agent held at the wall
 x=0.999921, never in either island, then arriving at (0,150)), NumericsTest.SmallStepsFarFromTheOriginRespectTheSpeedBudget (per-call and
 cumulative double bounds), NumericsTest.ExtremeDeflectionPastTheFloatLimitStaysFiniteContainedAndCapped (mesh edge at FLT_MAX, 1.5e38 m/s
 deflected away from the goal). AgentSteersAcrossAnOffsetWiderThanTheFloatRange was retuned so one substep's progress is representable
 instead of relying on banked sub-ULP travel — no test deleted.

 Docs now describe only retained code: architecture "Motion"/numeric-policy bullets, README, plan (new slice 8.10 + revisions log), and a
 third-pass section in the implementation report that records candidly that the second pass's sweep and residual were wrong and were removed.

 Actual verification outcomes

 ┌────────────────────────────────────────────────────────────────────────────────────────────┬─────────────────────────────────────────────┐
 │ Gate                                                                                       │ Result                                      │
 ├────────────────────────────────────────────────────────────────────────────────────────────┼─────────────────────────────────────────────┤
 │ scripts/check.sh (GCC 16.2.1 Debug configure + build + ctest + clang-format --dry-run      │ ALL CHECKS PASSED, 81/81, format clean      │
 │ -Werror)                                                                                   │                                             │
 ├────────────────────────────────────────────────────────────────────────────────────────────┼─────────────────────────────────────────────┤
 │ Release (NDEBUG), GCC 16.2.1                                                               │ 0 warnings, 81/81 pass                      │
 ├────────────────────────────────────────────────────────────────────────────────────────────┼─────────────────────────────────────────────┤
 │ Clang 22.1.8, -Wall -Wextra -Wpedantic + strict set                                        │ 0 warnings, 81/81 pass                      │
 ├────────────────────────────────────────────────────────────────────────────────────────────┼─────────────────────────────────────────────┤
 │ ASan + UBSan                                                                               │ 0 warnings, 81/81 pass, no sanitizer        │
 │                                                                                            │ reports                                     │
 ├────────────────────────────────────────────────────────────────────────────────────────────┼─────────────────────────────────────────────┤
 │ -DBUILD_TESTING=OFF                                                                        │ libvwmini.a only, no test targets, 0        │
 │                                                                                            │ warnings                                    │
 ├────────────────────────────────────────────────────────────────────────────────────────────┼─────────────────────────────────────────────┤
 │ -DVWMINI_BUILD_TESTS=OFF alias                                                             │ configures, forwards to BUILD_TESTING       │
 ├────────────────────────────────────────────────────────────────────────────────────────────┼─────────────────────────────────────────────┤
 │ Downstream consumer (tests/consumer, BUILD_TESTING=OFF)                                    │ 0 warnings, exit code 0                     │
 └────────────────────────────────────────────────────────────────────────────────────────────┴─────────────────────────────────────────────┘

 The workspace is still not a git repository, so nothing was committed; all changes are on disk under
 /home/cbendele/Projects/CPlusPlus/vwmini_q38_flash.
