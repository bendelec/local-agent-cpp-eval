# Reference Crowd Scenarios

These are normative black-box crowd scenarios for SIM-009 through SIM-012.
They do not prescribe a steering algorithm. All coordinates are metres.

## Common procedure for normative scenarios

For Crossing, Overtaking, and Initially Overlapping Discs, create the stated fixed two-triangle square mesh and simulation. The reflex-corner scenario specifies its own mesh. Add agents in the listed order.
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

## Initially overlapping discs

This is the normative SIM-010 robustness scenario. Use the same square mesh,
step interval, deadline, and per-step finite/contained/speed assertions as the
other normative scenarios. Add two radius-`0.25f`, maximum-speed-`1.0f`,
arrival-radius-`0.10f` agents:

| Agent | Position | Goal |
|---|---:|---:|
| A | `(5.0, 5.0)` | `(5.0, 1.0)` |
| B | `(5.1, 5.0)` | `(5.0, 9.0)` |

The intentionally overlapping start is an explicit SIM-010 edge case. This
does not require a general collision-free crowd solution. It does require an
observable separation attempt: during the first 120 30 Hz steps, the greatest
centre separation must reach at least `0.25f`, rather than merely preserving
the initial `0.1f` separation or attracting the discs.

## Close following through a reflex corner

This regression, derived from visual-laboratory use, combines a connected bent
route with active local avoidance.
The mesh is the L-shaped union of the bottom arm `[0,5] × [0,1]` and left arm
`[0,1] × [1,5]`, represented by these CCW triangles:

- `(0,0), (5,0), (1,1)`;
- `(5,0), (5,1), (1,1)`;
- `(0,0), (1,1), (0,5)`; and
- `(1,1), (1,5), (0,5)`.

Add these radius-`0.25f`, maximum-speed-`1.4f`, arrival-radius-`0.10f` agents
in order:

| Agent | Position | Goal |
|---|---:|---:|
| Leader | `(2.5, 0.5)` | `(0.5, 3.0)` |
| Follower | `(3.1, 0.5)` | `(0.5, 4.0)` |

Call `step(1.0f / 60.0f)` 600 times. After each step, both agents must remain
finite and contained, respect their speed bounds, and remain separated by at
least the sum of their radii minus `1e-3f`. Both must reach their distinct
terminal goals. The fixture deliberately avoids identical terminal occupancy:
it tests route progress around the reflex corner while agents are inside each
other's local-avoidance influence, not an infeasible shared-goal crowd solve.
