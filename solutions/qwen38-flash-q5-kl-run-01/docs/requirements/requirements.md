# Requirements

## Overview
VWmini is a compact C++23 library for immutable 2D navigation meshes, deterministic point-to-point paths, and local steering of disc agents. These documents define its observable behavior, validation rules, and non-functional requirements; an implementation may choose any internal design that satisfies them.

## Topics

| Topic | File | Summary | Req IDs |
|---|---|---|---|
| Geometry and Mesh Input | [mesh.md](mesh.md) | Vector semantics, outline triangulation, triangle-mesh construction, containment, and topology. | MSH-001 – MSH-009 |
| Paths and Crowd Simulation | [simulation.md](simulation.md) | Path queries, agent lifecycle, goals, stepping, and local avoidance. | SIM-001 – SIM-013 |
| Non-Functional Requirements | [non-functional.md](non-functional.md) | Build, API compatibility, quality, scope constraints, architecture documentation, and implementation planning. | NFR-001 – NFR-010 |

## Non-Functional Requirements
- NFR-001 through NFR-010 are normative non-functional requirements and constraints.
- Determinism is required where stated for identical finite inputs on the same platform.
- All public invalid-input cases must return the specified diagnostic result rather than crash, invoke undefined behavior, or mutate an already accepted mesh.

## Out of Scope

VWmini is not a general game-navigation framework. It does not cover mesh edits after
construction; multiple maps or world transitions; holes or
polygon boolean/offset operations; weighted routing; runtime environmental obstacles;
callbacks, rendering, persistence, or concurrency; or global multi-agent planning.
Those capabilities are deliberately absent so the specified geometry, routing, and
local disc avoidance can be implemented and reviewed as one bounded task.

## Open Questions
- None.

## Normative conventions
Coordinates and distances are `float` metres in a Cartesian plane (`+x` right, `+y` up). `1e-4f` metres is the geometric tolerance (`epsilon`) unless a requirement explicitly gives another value. `Vec2::operator==` is exact component-wise `float` equality; it does not use `epsilon`. Geometric predicates use `epsilon` only where these requirements say they do.
