# Dual-Circle Inverse Kinematics Solver

This repository provides a compact inverse kinematics helper (`IKSolver`) based on the
intersection of two fixed-radius circles anchored at two base points. It is useful for
simple two-link or symmetric mechanical linkages where the effector position (circle A center)
needs to be translated into motor/actuator angles at two bases (B1, B2).

## Features
- Deterministic selection of one intersection per base (opposite signed internal angles)
- Clockwise-positive angle output from the global UP vector to each leg
- Optional per-leg angle flipping (`flip_b1`, `flip_b2`) for inverted motor installations
- Pure NumPy dependency for the core solver (no heavy imaging or GUI libs required)
- Batch solve convenience
- Safe API: no implicit numeric defaults; you must supply geometry explicitly

## Core Geometry
Given:
- Moving circle A center `(x, y)` and radius `r_a`
- Two fixed bases separated along X by `b_offset` (centers computed from total `width`/`height`)
- Base circle radius `r_b`

The intersections of circle A with each base circle define candidate joint locations.
The solver chooses one per side by the sign of the oriented angle ∠BIA (IB -> IA) so that
legs do not "cross".

## Installation
```bash
pip install -r requirements.txt
```
Only `numpy` is strictly required. Optional extras (commented in `requirements.txt`) can be
installed for visualization or testing.

## Quick Start
```python
from inverse_kinematics import IKSolver

Explicit geometry (replace with your hardware values). Coordinate system:
    - Origin (0,0) midway between the two bases.
    - Bases at (-b_offset, 0) and (+b_offset, 0).
    - +X right, +Y down.
solver = IKSolver(
    r_a=180.0,
    r_b=90.0,
    b_offset=50.0,
    flip_b1=False,
    flip_b2=False,
)

Target effector positions (circle A centers) in solver coordinates
targets = [
    (0.0, -20.0),    # above midpoint
    (30.0, -40.0),   # up-right
    (-10.0, 60.0),   # down-left
]

for t in targets:
    angles = solver.angles(t)
    if angles is None:
        print(f"Target {t}: unreachable")
    else:
        a1, a2 = angles
        print(f"Target {t}: b1={a1:.3f} rad, b2={a2:.3f} rad")
```

## Handling Unreachable Targets
`solver.angles(target)` returns `None` if either leg cannot reach. Use `solver.solve(target)`
if you need partial information per leg (`sol.b1.reachable`, `sol.b2.reachable`).

## Motor Direction Inversion
If a motor is mounted reversed, pass `flip_b1=True` or `flip_b2=True` to invert the sign
of that leg's final angle only (geometry unaffected).

## Batch Solving
```python
centers = [(400, 300), (410, 310), (420, 290)]
solutions = solver.batch_solve(centers)
for c, sol in zip(centers, solutions):
    print(c, sol.angles)
```

## Example Script
See `example_ik_usage.py` for a self-contained console demonstration.

## Safety Note
All numeric parameters have no defaults. Always verify units and ranges before
commanding physical hardware.

## License
MIT
