# Non-Functional Requirements

## Overview
VWmini is a small, direct C++23 implementation task of the specified geometry, immutable-mesh, pathfinding, and local-crowd contract. These constraints keep submissions portable, inspectable, and compatible with the library's fixed public surface.

## Requirements

| ID | Type | Description | Acceptance Criteria |
|---|---|---|---|
| NFR-001 | Constraint | Build requirements are fixed. | The project builds with CMake and C++23 using only the standard library; public headers are under `include/vwmini/` and the target is `vwmini::vwmini`. |
| NFR-002 | Constraint | The public surface and documented semantics are fixed. | Supplied public declarations, signatures, enum values, defaults, include paths, and target name remain unchanged. Candidates may add only private implementation details to the supplied headers. |
| NFR-003 | Constraint | Library execution has no mandatory external runtime machinery. | It has no mandatory third-party dependency, global mutable state, raw owning allocation, process exit, or console output from library code. |
| NFR-004 | Non-functional | The implementation is maintainable. | Its ownership, control flow, and dependencies are understandable; it avoids needless complexity and supports focused tests. Internal architecture is the implementer's design choice. |
| NFR-005 | Non-functional | Ownership and observation are safe. | The implementation uses RAII and value semantics, exposes no mutable internal containers, and keeps validation/error paths separate from normal stepping. |
| NFR-006 | Constraint | Public failure reporting is predictable. | Setup and mutation operations return `Result<T>` and `Error` rather than crashing or invoking undefined behavior. Non-finite scalar input returns `InvalidArgument`; invalid polygon or mesh geometry returns `InvalidMesh`; empty mesh creation returns `InvalidMesh`; out-of-mesh path endpoints, agent positions, and goals return `OutsideMesh`; disconnected `find_path` returns `NoPath`; and unknown agent ids return `NotFound`. |
| NFR-007 | Non-functional | Source and tests are reproducible. | The implementation is warning-clean under `-Wall -Wextra -Wpedantic`, consistently formatted, and has deterministic tests. |
| NFR-008 | Constraint | Scope remains proportionate to the library task. | No performance target exists beyond responsiveness for the supplied tests. Avoid parallelism, caches, frameworks, or speculative extension points that are unnecessary to satisfy these requirements. |
| NFR-009 | Deliverable | Architecture is documented throughout implementation. | Before implementation, create `docs/architecture/architecture.md` describing the intended components, responsibility/dependency boundaries, important seams and interfaces, ownership/data flow, and major decisions/trade-offs. Include at least one simple inline diagram (ASCII, Mermaid, or PlantUML), then update it as the design evolves so it reflects the submitted code. It must describe the implementation's own design, not prescribe or imitate any particular architecture. |

## Edge Cases
- NFR-006: `set_goal` with an in-mesh disconnected goal is not an error: it succeeds and produces `NoPath` as required by SIM-007.
- NFR-006: `step` with a non-finite or negative duration returns `InvalidArgument`; `clear_goal`, `set_goal`, and `remove_agent` on an unknown or removed id return `NotFound`.
