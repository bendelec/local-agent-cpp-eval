# Evaluation — Qwen 3.8 27B

## Run identity

| Field | Value |
|---|---|
| Model | Qwen 3.8 27B |
| Runtime | Local Lemonade / llama.cpp; `Qwen3.8-27B-UD-Q8_K_XL` |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables |
| Final source | [`../solutions/qwen38-27b-q8-xl-run-01-repair-02/`](../solutions/qwen38-27b-q8-xl-run-01-repair-02/) |
| Final tree fingerprint | `622f8a4dcf0dda5006486ebdc57f2964bfe987b624140fbab482662360f371fd` |
| Repair limit | Two repair prompts |
| Final result | **83 / 100** |

## Initial state

The one-shot submission was unusually strong. It built cleanly, preserved the public API,
passed its native suite (72/72), and had a complete C++23 architecture with real design and
implementation-plan documentation. Geometry, mesh construction, and pathfinding were the
strongest parts.

Against the current public suite it scored **69/72**: geometry 15/15, navmesh/path 24/24,
simulation 28/29, and crowd 2/4. The missed behaviors were a single-agent reflex-corner
stall, no material recovery from an initially overlapping pair, and close-following progress
through a reflex corner.

## Final state

The final source builds warning-clean with GCC and Clang. Its native suite passes **76/76**
under GCC and under ASan/UBSan. Public conformance is **71/72**: all geometry, navmesh/path,
and lifecycle tests pass; crossing, overtaking, and overlap recovery pass; the
close-following reflex-corner test fails because the follower remains `Moving`.

Visual testing confirms that a nearby same-direction follower can still cause a leading
agent to become stuck at a valid L-corner. This is a practical local-avoidance progress
failure, not an infeasible shared-goal case.

The final repair also has two material review findings not exposed by its native sanitizer
suite:

- Valid `max_speed == FLT_MAX` together with a huge finite step can overflow the float
  avoidance push and propagate NaN state.
- The landing exemption used to improve degenerate-zone progress can permit a fast moving
  disc's swept segment to pass through a stationary disc when its endpoints are clear.

These are substantial functional deductions, but they are not an observed memory-safety or
undefined-behavior sanitizer failure. The `max_speed == FLT_MAX` case is a targeted
source-review probe, not a failing normative-conformance fixture; it therefore does not
invoke the hard safety cap.

Reproduction used:

```sh
./evaluator/conformance/run.sh solutions/qwen38-27b-q8-xl-run-01-repair-02 /tmp/qwen-final
cmake -S solutions/qwen38-27b-q8-xl-run-01-repair-02 -B /tmp/qwen-native -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/qwen-native --parallel && ctest --test-dir /tmp/qwen-native --output-on-failure
```

## Score

| Area | Score | Assessment |
|---|---:|---|
| Geometry and mesh validation | 20 / 20 | All public checks pass; validation and containment are well structured. |
| Pathfinding | 20 / 20 | Direct, disconnected, and bent routes pass with deterministic, contained output. |
| Agent lifecycle and stepping | 16 / 20 | Public behavior is strong, but extreme valid speed/duration input can yield NaN state. |
| Local crowd behavior | 9 / 15 | Crossing, overtaking, and overlap recovery pass; close following at a reflex corner still deadlocks, and the landing exemption has a tunnelling risk. |
| Tests and functional discipline | 7 / 10 | 76 focused native tests and sanitizer runs are valuable, but the close-following test used an easier goal than the public fixture and missed the extreme-speed and swept-contact cases. |
| Architecture | 7 / 10 | Clear module boundaries, value ownership, PIMPL isolation, and substantive documentation. The final avoidance logic became increasingly special-case-driven and difficult to reason about. |
| C++ quality | 4 / 5 | Modern value-oriented C++ and warning-clean builds; dense float avoidance code leaves overflow and interaction hazards. |
| **Total** | **83 / 100** | |

## Repair prompts and intermediate result

Two repair prompts were used.

1. **First repair:** fixed the single-agent reflex waypoint-consumption stall and the
   overlap-response sign/progress defect. It was a strong, targeted improvement, but did
   not cover huge finite durations or close-following corner behavior.
2. **Second repair:** targeted huge finite-duration safety, close-following reflex-corner
   progress, restoration of avoidance regressions, and documentation accuracy. It added
   detailed regressions and fixed several issues, but its close-following scenario used a
   less demanding terminal arrangement than the public fixture. The remaining public
   corner failure and review findings above prevent a higher score.

## Final assessment

Qwen produced a substantially stronger implementation than the other evaluated model: the
core geometry/path system is reliable, the architecture is coherent, and repair prompts led
to real improvements. Its main weakness is avoidance complexity: successive local patches
addressed individual failures but did not yield a robust, general corner-following policy.
The result is a credible small navigation library with meaningful crowd limitations, rather
than release-quality local avoidance.
