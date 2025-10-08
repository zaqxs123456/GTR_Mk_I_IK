# Path-Planned Dual-Circle IK Toolkit

This project helps you drive a two-link mechanism whose joints are modeled as
two fixed-radius circles anchored at bases **B1** and **B2**, while the end
effector is the center of a moving circle **A**. Although you can talk to the
inverse kinematics solver directly, the recommended workflow is to let the
**`PathPlanner`** generate synchronized joint motions that keep the effector on
its intended path.

<details>
<summary>Key components at a glance</summary>

- `IKSolver`: converts a single effector point into joint angles (0–2π range).
- `PathPlanner`: subdivides straight moves and assigns angular velocities so
    the effector maintains constant linear speed.
- `SegmentPlan`: dataclass describing each sub-segment (duration, velocities,
    angles).
- `geometry`: low-level helpers reused by the solver and planner.
</details>

---

## Installation

```bash
pip install -r requirements.txt
```

Only `numpy` is required for the math. Install `pygame` as well if you want to
run the visualization examples listed below (they are commented in
`requirements.txt`).

---

## Recommended Workflow: Use the Path Planner

The planner keeps the effector on a straight line while the base joints rotate
at different speeds. This produces smoother motion than telling each joint to
rotate at the same rate.

### 1. Instantiate solver and planner

```python
from ik import IKSolver, PathPlanner

SOLVER = IKSolver(r_a=160.0, r_b=90.0, b_offset=50.0)
PLANNER = PathPlanner(
        SOLVER,
        max_segment_length=10.0,  # mm per sub-segment (tune for your hardware)
        linear_speed=120.0,       # mm/s target effector speed
)
```

### 2. Plan a straight move

```python
start = (40.0, 200.0)
end = (-80.0, 70.0)
segments = PLANNER.plan(start, end)
```

Each `SegmentPlan` contains start/end points, segment length, travel duration,
and angular velocities/positions for both bases so they finish exactly on time.

### 3. Execute the plan on your motors

```python
for seg in segments:
        command_joint("b1", position=seg.b1_start, velocity=seg.b1_velocity)
        command_joint("b2", position=seg.b2_start, velocity=seg.b2_velocity)
        wait(seg.duration)
```

Adapt the `command_joint` and `wait` calls to your hardware API. The key idea is
to start each joint at the supplied angle and run it for `seg.duration` using
the provided angular velocity.

> **⚠️ Units reminder:** The planner supplies joint angles in **radians** and
> angular velocities in **radians per second**. If your actuators use encoders
> with gear reduction or alternate units, scale both the starting positions and
> velocities accordingly before issuing commands.

### 4. Visualize before touching hardware (optional)

```
python examples/path_planner_with_vis.py
```

This Pygame demo plots three polylines: the desired straight path (cyan), a
naive equal-speed joint motion (red), and the synchronized planner output
(green). Watching the paths diverge highlights why the planner is preferred.

---

## Direct IK Solver Usage (Advanced)

If you need raw joint angles—perhaps for diagnostics or to seed a different
planner—you can call the solver directly.

```python
from ik import IKSolver

solver = IKSolver(r_a=180.0, r_b=90.0, b_offset=50.0)

target = (25.0, -40.0)
angles = solver.angles(target)
if angles is None:
        print("Target unreachable")
else:
        b1_angle, b2_angle = angles
        print(f"b1={b1_angle:.3f} rad, b2={b2_angle:.3f} rad")
```

- Use `solver.solve(point)` if you want `LegSolution` objects that tell you
    whether each leg is reachable and provide intersection coordinates.
- `solver.batch_solve(points)` runs multiple targets in one call.
- Flip motor directions with `IKSolver(..., flip_b1=True)` or
    `flip_b2=True` if a joint is mounted in reverse.

### Handling Unreachable Targets

`solver.angles(target)` returns `None` when either leg cannot reach the point.
The `solve` method exposes `sol.b1.reachable` / `sol.b2.reachable` for more
granular checks.

---

## Geometry & Coordinate System

- Origin `(0, 0)` lies midway between the base centers.
- Bases are placed at `(-b_offset, 0)` (B1) and `(b_offset, 0)` (B2).
- Axes follow screen-style math: +X right, +Y down.
- Angles are measured **clockwise** from the global up vector and normalized to
    the range `[0, 2π)`.
- `geometry.circle_intersect` and friends live in `ik.geometry` should you need
    lower-level primitives.

---

## Examples & Project Layout

- `examples/path_planner_with_vis.py` – visual comparison of naive vs. planned motion.
- `examples/ik_usage.py` – console demo for the solver.
- `examples/ik_usage_with_vis.py` – interactive Pygame tool to probe reachability.

Module overview:

```text
ik/
├── __init__.py         # Re-exports IKSolver, PathPlanner, geometry helpers
├── geometry.py         # Circle intersection math
├── ik_solver.py        # Dual-circle IK implementation
├── path_planner.py     # Segment planner producing synchronized joint motion
└── examples/           # Visualization and usage demos
```

---

## Safety Checklist

- Provide geometry values explicitly (no defaults are assumed).
- Validate units and motor limits before executing the generated plan.
- Simulate in software (e.g., run the visualization) prior to commanding real
    hardware.

---

## License

MIT
