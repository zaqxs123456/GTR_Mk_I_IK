"""Inverse kinematics (dual-circle intersection) core module.

Clean, dependency-light implementation providing an easy to use `IKSolver`.
No backward compatibility shims retained.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, Iterable
import numpy as np

Vec2 = Tuple[float, float]

# ---------------------------------------------------------------------------
# Low-level geometry helpers (encapsulated in a class for organization)
# ---------------------------------------------------------------------------

class Geometry:
    """Namespace-style container for low-level geometric helper methods.

    Methods are static to avoid accidental state; grouping clarifies API surface
    and enables downstream overriding/subclassing if specialized behavior is
    ever required (e.g., numeric robustness tweaks, instrumentation).
    """

    @staticmethod
    def circle_intersect(c0x: float, c0y: float, r0: float, c1x: float, c1y: float, r1: float):
        """Return intersection points of two circles or None if none.

        Returns a tuple (p1, p2) even when tangent (p1==p2) for uniform handling.
        """
        c0 = np.array([c0x, c0y], dtype=float)
        c1 = np.array([c1x, c1y], dtype=float)
        v = c1 - c0
        d = float(np.linalg.norm(v))
        if d == 0.0:
            return None
        if d > r0 + r1 or d < abs(r0 - r1):
            return None
        u = v / d
        a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d)
        p_mid = c0 + a * u
        h_sq = r0 * r0 - a * a
        if h_sq < 0:
            if h_sq > -1e-9:
                h_sq = 0.0
            else:
                return None
        h = float(np.sqrt(h_sq))
        u_perp = np.array([-u[1], u[0]])
        p1 = p_mid + h * u_perp
        p2 = p_mid - h * u_perp
        return (p1, p2)

    @staticmethod
    def angle_diff(p0: Vec2, p1: Vec2, clockwise: bool) -> float:
        """Minimal signed angle difference p0->p1 with chosen positive direction.
        Returns radians in (-pi, pi]."""
        a0 = np.arctan2(p0[1], p0[0])
        a1 = np.arctan2(p1[1], p1[0])
        diff = a1 - a0
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        if clockwise:
            diff = -diff
            diff = (diff + np.pi) % (2 * np.pi) - np.pi
            if np.isclose(diff, -np.pi):
                diff = np.pi
        return diff

    @staticmethod
    def _select_intersection(inter_pair, a: Vec2, b: Vec2, want_positive: bool):
        """Select one intersection by sign of CCW angle ∠BIA (IB->IA)."""
        if inter_pair is None:
            return None
        for inter in inter_pair:
            ix, iy = float(inter[0]), float(inter[1])
            if not (np.isfinite(ix) and np.isfinite(iy)):
                continue
            v_ib_screen = (b[0] - ix, b[1] - iy)
            v_ia_screen = (a[0] - ix, a[1] - iy)
            v_ib = (v_ib_screen[0], -v_ib_screen[1])
            v_ia = (v_ia_screen[0], -v_ia_screen[1])
            diff_ccw = Geometry.angle_diff(v_ib, v_ia, clockwise=False)
            if want_positive and diff_ccw > 0:
                return (ix, iy)
            if not want_positive and diff_ccw < 0:
                return (ix, iy)
        return None

# Backwards compatible function wrappers (can be removed later if undesired)
def circle_intersect(c0x: float, c0y: float, r0: float, c1x: float, c1y: float, r1: float):
    return Geometry.circle_intersect(c0x, c0y, r0, c1x, c1y, r1)

def angle_diff(p0: Vec2, p1: Vec2, clockwise: bool) -> float:
    return Geometry.angle_diff(p0, p1, clockwise)

def _select_intersection(inter_pair, a: Vec2, b: Vec2, want_positive: bool):
    return Geometry._select_intersection(inter_pair, a, b, want_positive)

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
    def leg(self, name: str) -> LegSolution:
        return self.legs[name]
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

    Parameters must be explicit (no defaults) for safe hardware integration.
    flip_b1 / flip_b2 optionally invert final angles for motor inversion.
    """
    def __init__(self, r_a: float, r_b: float, b_offset: float, width: float, height: float,
                 *, flip_b1: bool = False, flip_b2: bool = False) -> None:
        self.r_a = float(r_a)
        self.r_b = float(r_b)
        self.b_offset = float(b_offset)
        self.width = float(width)
        self.height = float(height)
        self.flip_b1 = bool(flip_b1)
        self.flip_b2 = bool(flip_b2)
        cx = self.width * 0.5
        cy = self.height * 0.5
        self._b1_center = (cx - self.b_offset, cy)
        self._b2_center = (cx + self.b_offset, cy)

    def solve(self, a_center: Vec2) -> IKSolution:
        a = a_center
        b1 = self._b1_center
        b2 = self._b2_center
        inter_b1 = Geometry.circle_intersect(a[0], a[1], self.r_a, b1[0], b1[1], self.r_b)
        inter_b2 = Geometry.circle_intersect(a[0], a[1], self.r_a, b2[0], b2[1], self.r_b)
        sel_b1 = Geometry._select_intersection(inter_b1, a, b1, want_positive=False)
        sel_b2 = Geometry._select_intersection(inter_b2, a, b2, want_positive=True)
        legs: Dict[str, LegSolution] = {}
        up = (0.0, 1.0)
        if sel_b1 is not None:
            vx = sel_b1[0] - b1[0]
            vy = sel_b1[1] - b1[1]
            v_math = (vx, -vy)
            angle_cw = Geometry.angle_diff(up, v_math, clockwise=True)
            if self.flip_b1:
                angle_cw = -angle_cw
            legs['b1'] = LegSolution('b1', sel_b1, angle_cw)
        else:
            legs['b1'] = LegSolution('b1', None, None)
        if sel_b2 is not None:
            vx = sel_b2[0] - b2[0]
            vy = sel_b2[1] - b2[1]
            v_math = (vx, -vy)
            angle_cw = Geometry.angle_diff(up, v_math, clockwise=True)
            if self.flip_b2:
                angle_cw = -angle_cw
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

__all__ = [
    'Geometry', 'circle_intersect', 'angle_diff', 'LegSolution', 'IKSolution', 'IKSolver'
]
