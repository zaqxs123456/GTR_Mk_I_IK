"""Inverse kinematics (dual-circle intersection) core module.

Clean, dependency-light implementation providing an easy to use `IKSolver`.
No backward compatibility shims retained.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Optional, Iterable, Tuple
import numpy as np

import geometry as gm

Vec2 = gm.Vec2

# ---------------------------------------------------------------------------
# Public dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LegSolution:
    name: str
    intersection: Optional[Vec2]
    angle_from_up_cw: Optional[float]
    @property
    def reachable(self) -> bool:
        return self.intersection is not None and self.angle_from_up_cw is not None

@dataclass(frozen=True)
class IKSolution:
    legs: Dict[str, LegSolution] = field(default_factory=dict)
    @property
    def b1(self) -> LegSolution: return self.legs['b1']
    @property
    def b2(self) -> LegSolution: return self.legs['b2']
    @property
    def angles(self) -> Tuple[Optional[float], Optional[float]]:
        return self.b1.angle_from_up_cw, self.b2.angle_from_up_cw
    @property
    def all_reachable(self) -> bool:
        return self.b1.reachable and self.b2.reachable

# ---------------------------------------------------------------------------
# Solver
# ---------------------------------------------------------------------------

class IKSolver:
    """Dual-circle IK solver selecting opposite signed intersections for two bases.

    Coordinate System:
        - Origin (0,0) is the midpoint between base centers B1 and B2.
        - +X points to the right (toward B2), +Y points downward.
        - Base centers are located at (-b_offset, 0) and (+b_offset, 0).

    Parameters must be explicit (no defaults) for safe hardware integration.
    flip_b1 / flip_b2 optionally invert final angles for motor inversion.
    """
    def __init__(self, r_a: float, r_b: float, b_offset: float,
                 *, flip_b1: bool = False, flip_b2: bool = False) -> None:
        self.r_a = float(r_a)
        self.r_b = float(r_b)
        self.b_offset = float(b_offset)
        self.flip_b1 = bool(flip_b1)
        self.flip_b2 = bool(flip_b2)
        # Base centers in local solver coordinates
        self._b1_center = (-self.b_offset, 0.0)
        self._b2_center = ( self.b_offset, 0.0)

    def solve(self, a_center: Vec2) -> IKSolution:
        a = a_center
        b1 = self._b1_center
        b2 = self._b2_center
        inter_b1 = gm.circle_intersect(a[0], a[1], self.r_a, b1[0], b1[1], self.r_b)
        inter_b2 = gm.circle_intersect(a[0], a[1], self.r_a, b2[0], b2[1], self.r_b)
        sel_b1 = gm._select_intersection(inter_b1, a, b1, want_positive=False)
        sel_b2 = gm._select_intersection(inter_b2, a, b2, want_positive=True)
        legs: Dict[str, LegSolution] = {}
        up = (0.0, 1.0)
        if sel_b1 is not None:
            vx = sel_b1[0] - b1[0]
            vy = sel_b1[1] - b1[1]
            v_math = (vx, -vy)
            angle_cw = gm.angle_diff(up, v_math, clockwise=True)
            if self.flip_b1:
                angle_cw = -angle_cw
            # Normalize to [0, 2π)
            if angle_cw < 0:
                angle_cw = (angle_cw % (2 * np.pi))
            legs['b1'] = LegSolution('b1', sel_b1, angle_cw)
        else:
            legs['b1'] = LegSolution('b1', None, None)
        if sel_b2 is not None:
            vx = sel_b2[0] - b2[0]
            vy = sel_b2[1] - b2[1]
            v_math = (vx, -vy)
            angle_cw = gm.angle_diff(up, v_math, clockwise=True)
            if self.flip_b2:
                angle_cw = -angle_cw
            if angle_cw < 0:
                angle_cw = (angle_cw % (2 * np.pi))
            legs['b2'] = LegSolution('b2', sel_b2, angle_cw)
        else:
            legs['b2'] = LegSolution('b2', None, None)
        return IKSolution(legs=legs)

    def solve_or_none(self, a_center: Vec2) -> Optional[IKSolution]:
        sol = self.solve(a_center)
        if not sol.b1.reachable or not sol.b2.reachable:
            print(f"IK Unreachable for {a_center}: b1={sol.b1.reachable} b2={sol.b2.reachable}")
            return None
        return sol

    def batch_solve(self, centers: Iterable[Vec2]) -> list[IKSolution]:
        return [self.solve(c) for c in centers]

    def angles(self, a_center: Vec2) -> Optional[Tuple[float, float]]:
        sol = self.solve_or_none(a_center)
        if sol is None:
            return None
        return (sol.b1.angle_from_up_cw, sol.b2.angle_from_up_cw)  # type: ignore

