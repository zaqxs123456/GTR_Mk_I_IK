"""Low-level geometric helpers for the dual-circle IK solver."""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

Vec2 = Tuple[float, float]


class Geometry:
    """Namespace-style container for low-level geometric helper methods."""

    @staticmethod
    def circle_intersect(c0x: float, c0y: float, r0: float,
                         c1x: float, c1y: float, r1: float):
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
        """Minimal signed angle difference p0->p1 with chosen positive direction."""
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


# Backwards compatible module-level helpers

def circle_intersect(c0x: float, c0y: float, r0: float,
                     c1x: float, c1y: float, r1: float):
    return Geometry.circle_intersect(c0x, c0y, r0, c1x, c1y, r1)


def angle_diff(p0: Vec2, p1: Vec2, clockwise: bool) -> float:
    return Geometry.angle_diff(p0, p1, clockwise)


def _select_intersection(inter_pair, a: Vec2, b: Vec2, want_positive: bool):
    return Geometry._select_intersection(inter_pair, a, b, want_positive)


__all__ = [
    "Vec2",
    "Geometry",
    "circle_intersect",
    "angle_diff",
    "_select_intersection",
]
