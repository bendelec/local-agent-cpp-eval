# Geometry and Navmesh Input

## Overview

Walkable space is a non-empty collection of counter-clockwise triangles. A point is
walkable when it is contained by at least one triangle under the boundary policy below.
`triangulate_simple_polygon` is the sole geometry authoring helper: it turns one
hole-free simple outline into triangles suitable for `NavMesh::create`.

## Numerical policy

`epsilon` is `1e-4f` metres. It applies only to point/edge distance and shared-edge
endpoint matching. A triangle is non-degenerate when its absolute signed double area
is greater than `epsilon * epsilon`. Input whose non-adjacent edges, or distinct vertices, are separated by no more than
`epsilon` is numerically ambiguous: an implementation may accept or reject it, but must
remain safe and deterministic. Such geometry is outside the interoperability guarantee.

## Requirements

| ID | Type | Description | Acceptance Criteria |
|---|---|---|
| MSH-001 | Functional | `Vec2` provides the documented value-semantic vector operations. | `Vec2::operator==` performs exact component-wise equality. `+`, `-`, scalar `*`, `dot`, `cross`, `length`, and `normalized` have their usual vector meanings; normalizing `{0,0}` returns `{0,0}`. |
| MSH-002 | Functional | `triangulate_simple_polygon` validates and triangulates one simple, CCW, hole-free outline. | Non-finite coordinates return `InvalidArgument`. Fewer than three vertices, cyclic consecutive duplicate vertices (including equal first and last), self-intersections, clockwise winding, and degenerate area return `InvalidMesh`. Non-duplicate collinear outline vertices are permitted. |
| MSH-003 | Functional | A valid outline produces deterministic non-degenerate CCW triangles. | Triangle interiors do not overlap except at edges/vertices; every triangle vertex is an input vertex; and the sum of triangle signed areas differs from the input signed area by at most `epsilon * max(1.0f, input_area)`. Identical input produces identical ordered output on one platform. Triangle ordering is otherwise not contractual. |
| MSH-004 | Functional | `NavMesh::create` accepts only a non-empty list of valid CCW triangles. | A non-finite vertex returns `InvalidArgument`. An empty list, a non-triangle, a clockwise or degenerate triangle, overlapping triangle interiors, a non-manifold edge, or a T-junction returns `InvalidMesh`. |
| MSH-005 | Functional | Mesh adjacency is derived from shared complete edges. | Two triangles are adjacent only if a complete non-zero-length edge matches with corresponding endpoints no farther than `epsilon`. A vertex-only touch creates no adjacency. |
| MSH-006 | Functional | `NavMesh::contains` implements one boundary policy. | It returns false for non-finite points; true for points strictly inside a triangle; and true for a finite point whose Euclidean distance to the **closed finite line segment** of any triangle edge is at most `epsilon`. It returns false otherwise. The query is const and allocation-free. |
| MSH-007 | Constraint | Creation is transactional and accepted meshes are immutable. | Failure exposes no partial `NavMesh`; a successful mesh has the accepted triangle count and is usable through const operations without mutation. |
| MSH-008 | Constraint | Internal mesh representation is unobservable. | An implementation may reorder or otherwise transform accepted triangles internally, but must preserve the public containment and deterministic-path contracts. Caller triangle order may affect deterministic tie-breaking; no particular equal-cost route winner is public behavior. |
| MSH-009 | Constraint | The submitted mesh represents space available to agent centres. | Callers provide any needed wall clearance in the triangle geometry. The library does not derive clearance or alter submitted polygons. |

## Edge Cases

- A collinear outline vertex is allowed; a degenerate triangle is not.
- Disjoint valid triangle components are allowed. A vertex in the middle of another
  triangle edge is a T-junction and invalid.
- Boundary tolerance can admit a point just outside a triangle; it is intentionally
  part of containment and applies consistently to path endpoints and agent positions.
