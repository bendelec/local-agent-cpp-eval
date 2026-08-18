# Reference Crowd Scenarios

These are the only normative crowd-performance scenarios for SIM-011. They are
black-box tests, not prescribed steering algorithms. All coordinates are metres.

## Common procedure

Create the stated single-cell mesh and simulation. Add agents in the listed order.
Call `step(1.0f / 30.0f)` exactly 360 times (12 seconds). After every successful
returned step, read every listed `AgentState` and assert:

- every position and velocity component is finite;
- each centre is contained by the mesh;
- `length(velocity) <= max_speed + 1e-4f`; and
- for every live pair, centre distance is at least `radius_a + radius_b - 1e-3f`.

At the end, every listed agent must have `AgentStatus::Reached`. The mesh is the CCW
square split into these two CCW triangles: `(0,0), (10,0), (10,10)` and
`(0,0), (10,10), (0,10)`.

## Crossing

Add these two agents, in this order. Both use radius `0.25f`, maximum speed `1.0f`,
and arrival radius `0.10f`.

| Agent | Position | Goal |
|---|---:|---:|
| A | `(2, 4)` | `(8, 6)` |
| B | `(2, 6)` | `(8, 4)` |

Their desired lines cross near the middle of open space. A correct implementation may
choose either passing side and need not reproduce an exact trajectory.

## Overtaking

Add these two agents, in this order. Both use radius `0.25f` and arrival radius
`0.10f`.

| Agent | Position | Max speed | Goal |
|---|---:|---:|---:|
| A (slower, ahead) | `(3, 5)` | `0.50f` | `(8, 5)` |
| B (faster, behind) | `(2, 5)` | `1.25f` | `(8, 6)` |

Open lateral space exists, so B must not remain permanently blocked behind A. The
assertion is only that both reach their goals by the common deadline; no test asserts
which side B uses or that B reaches first.
