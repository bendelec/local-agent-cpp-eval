# Repair Feedback

An external verification pass found the following issues in the current implementation.
Please investigate them independently, correct the implementation as appropriate, update
its tests and architecture documentation where needed, and rerun the project’s test
suite. Preserve the supplied public API.

## Observed issues

1. A direct route whose endpoints are accepted through boundary tolerance did not return
   exactly the two supplied endpoints. It returned five points instead.

   **Diagnostic hint:** examine the direct-path decision, the corridor fallback, and path
   reconstruction. In particular, consider whether the goal can enter the result more
   than once and whether a proper edge-crossing test fully represents the documented
   boundary policy.

2. A route through an L-shaped mesh contained a segment that was outside the mesh, even
   though the route’s individual points appeared acceptable.

   **Diagnostic hint:** absence of a proper crossing of a boundary edge is not, by itself,
   necessarily proof that every point of a segment is contained. Inspect visibility and
   containment reasoning around vertices, collinear contact, and tolerance boundaries.

3. In an ordinary two-agent crossing scenario, both agents remained `Moving` after the
   scenario duration instead of reaching their goals. A separate overtaking scenario
   completed successfully.

   **Diagnostic hint:** compare the velocity data used while selecting avoidance choices
   with the velocities that are ultimately committed simultaneously. Also inspect whether
   the no-candidate fallback can produce a persistent stall.

4. Review of the movement commit path found a possible mismatch between the candidate
   motion that is checked and the displacement that is actually applied when approaching
   a waypoint. Mesh membership is also checked at the resulting endpoint.

   **Diagnostic hint:** examine the waypoint-clamping condition, especially when a chosen
   direction is not aligned with the waypoint direction, and distinguish endpoint
   membership from swept-path membership.

5. Review of mesh validation found that two geometrically identical CCW triangles may be
   accepted even though their interiors completely overlap.

   **Diagnostic hint:** inspect overlap handling for coincident vertices and collinear or
   coincident edges, not only strict interior tests and proper crossings.

6. Review of polygon input validation found similar edge-case risk for self-touching
   outlines or overlapping non-adjacent collinear edges.

   **Diagnostic hint:** inspect which segment-contact categories the self-intersection
   test currently recognizes.

Do not assume that fixing only the listed examples is sufficient. Keep behavior
deterministic and maintain the documented error and boundary semantics.
