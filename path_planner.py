"""Path planning utilities for driving the IK solver along straight segments.

The planner keeps the effector (circle A center) moving at uniform linear
speed while providing angular velocities for each base leg so they complete
each sub-segment simultaneously.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from ik_solver import IKSolver

Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class SegmentPlan:
	"""Description of a single straight-line sub-segment for circle A.

	Attributes
	----------
	index:
		Sequential index of the segment within the planned motion (0-based).
	start, end:
		Start and end positions for circle A in solver coordinates.
	length:
		Euclidean distance between start and end (same units as input).
	duration:
		Time required to traverse the segment at the planner's linear speed.
	b1_start, b1_end:
		Clockwise-from-up angles (radians, 0–2π) for base 1 at the start/end.
	b1_delta, b1_velocity:
		Minimal signed angular displacement (radians) and required angular
		velocity (radians per second) for base 1 to complete the segment on time.
	b2_start, b2_end, b2_delta, b2_velocity:
		Same definitions for base 2.
	"""

	index: int
	start: Vec2
	end: Vec2
	length: float
	duration: float
	b1_start: float
	b1_end: float
	b1_delta: float
	b1_velocity: float
	b2_start: float
	b2_end: float
	b2_delta: float
	b2_velocity: float


class PathPlanner:
	"""Plan straight-line motions for the IK solver with uniform effector speed."""

	def __init__(self, solver: IKSolver, *, max_segment_length: float, linear_speed: float) -> None:
		if max_segment_length <= 0:
			raise ValueError("max_segment_length must be > 0")
		if linear_speed <= 0:
			raise ValueError("linear_speed must be > 0")
		self.solver = solver
		self.max_segment_length = float(max_segment_length)
		self.linear_speed = float(linear_speed)

	def plan(self, a_start: Vec2, a_end: Vec2) -> List[SegmentPlan]:
		"""Create a sequence of segments and joint velocities from start to end."""
		segments = self._segment_points(a_start, a_end)
		if not segments:
			return []

		points = [segments[0][0]] + [seg[1] for seg in segments]
		angles = self._solve_angles(points)

		plans: List[SegmentPlan] = []
		for idx, ((p0, p1), (a0, a1)) in enumerate(zip(segments, angles_pairs(angles))):
			seg_length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
			duration = seg_length / self.linear_speed if seg_length > 0 else 0.0

			b1_delta = shortest_angle_delta(a0[0], a1[0])
			b2_delta = shortest_angle_delta(a0[1], a1[1])
			b1_velocity = b1_delta / duration if duration > 0 else 0.0
			b2_velocity = b2_delta / duration if duration > 0 else 0.0

			plans.append(
				SegmentPlan(
					index=idx,
					start=p0,
					end=p1,
					length=seg_length,
					duration=duration,
					b1_start=a0[0],
					b1_end=a1[0],
					b1_delta=b1_delta,
					b1_velocity=b1_velocity,
					b2_start=a0[1],
					b2_end=a1[1],
					b2_delta=b2_delta,
					b2_velocity=b2_velocity,
				)
			)

		return plans

	def _segment_points(self, start: Vec2, end: Vec2) -> List[Tuple[Vec2, Vec2]]:
		dx = end[0] - start[0]
		dy = end[1] - start[1]
		distance = math.hypot(dx, dy)
		if distance == 0:
			return []

		num_segments = max(1, int(math.ceil(distance / self.max_segment_length)))
		xs = np.linspace(start[0], end[0], num_segments + 1)
		ys = np.linspace(start[1], end[1], num_segments + 1)
		points = list(zip(xs, ys))
		return list(zip(points[:-1], points[1:]))

	def _solve_angles(self, points: Sequence[Vec2]) -> List[Tuple[float, float]]:
		angles: List[Tuple[float, float]] = []
		for pt in points:
			solution = self.solver.solve(pt)
			if not solution.all_reachable:
				raise ValueError(f"IK solution unreachable for point {pt}")
			b1_angle, b2_angle = solution.angles
			if b1_angle is None or b2_angle is None:
				raise ValueError(f"Angles missing for point {pt}")
			angles.append((b1_angle, b2_angle))
		return angles


def shortest_angle_delta(start: float, end: float) -> float:
	"""Return the minimal signed angle delta taking wrap-around into account."""
	diff = (end - start + math.pi) % (2 * math.pi) - math.pi
	return diff


def angles_pairs(angles: Sequence[Tuple[float, float]]) -> Iterable[Tuple[Tuple[float, float], Tuple[float, float]]]:
	"""Yield consecutive angle pairs from a sequence of (b1, b2) angles."""
	for i in range(len(angles) - 1):
		yield angles[i], angles[i + 1]

