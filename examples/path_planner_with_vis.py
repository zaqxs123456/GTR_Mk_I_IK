"""Visual comparison of naive joint rotation vs. path-planned motion.

Run from the project root:
    python examples/path_planner_with_vis.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np

try:
    import pygame  # type: ignore
except ImportError as exc:  # pragma: no cover
    print("pygame not installed. Install with: pip install pygame")
    raise SystemExit(1) from exc

# Make package available when running as a script
sys.path.append(str(Path(__file__).resolve().parents[1]))

try:
    from ik import IKSolver, PathPlanner, circle_intersect  # type: ignore
except ModuleNotFoundError:
    from ik_solver import IKSolver  # type: ignore
    from path_planner import PathPlanner  # type: ignore
    from geometry import circle_intersect  # type: ignore

Vec2 = Tuple[float, float]

# ---------------- Configuration ----------------
WIDTH, HEIGHT = 1920, 1080
SCREEN_ORIGIN_X = WIDTH * 0.5
SCREEN_ORIGIN_Y = HEIGHT * 0.3
FPS = 60

R_A = 160.0*2
R_B = 90.0*2
B_OFFSET = 50.0*2
LINEAR_SPEED = 120.0*2
MAX_SEGMENT_LENGTH = 30.0*2
START_POINT: Vec2 = (90.0*2, 200.0*2)
END_POINT: Vec2 = (-110.0*2, 60.0*2)

COL_BG = (18, 20, 30)
COL_LINE = (120, 140, 200)
COL_NAIVE = (230, 110, 110)
COL_PLANNER = (110, 220, 150)
COL_BASIS = (200, 200, 200)
COL_B_CIR = (255, 170, 120)
COL_TEXT = (235, 235, 235)


# ---------------- Geometry helpers ----------------

def wrap_angle(angle: float) -> float:
    return angle % (2 * math.pi)


def shortest_angle_delta(start: float, end: float) -> float:
    return (end - start + math.pi) % (2 * math.pi) - math.pi


def angle_to_vector(angle_cw: float) -> Vec2:
    angle_math = -angle_cw
    sin_phi = math.sin(angle_math)
    cos_phi = math.cos(angle_math)
    v_math = (-sin_phi, cos_phi)
    return (v_math[0], -v_math[1])


def angles_to_position(
    angle_b1: float,
    angle_b2: float,
    *,
    prev: Vec2 | None,
) -> Vec2:
    b1_center = (-B_OFFSET, 0.0)
    b2_center = (B_OFFSET, 0.0)

    v1 = angle_to_vector(angle_b1)
    v2 = angle_to_vector(angle_b2)

    joint1 = (b1_center[0] + R_B * v1[0], b1_center[1] + R_B * v1[1])
    joint2 = (b2_center[0] + R_B * v2[0], b2_center[1] + R_B * v2[1])

    intersections = circle_intersect(
        joint1[0], joint1[1], R_A,
        joint2[0], joint2[1], R_A,
    )
    if intersections is None:
        raise ValueError("No valid effector position for given angles")

    candidates = [(float(p[0]), float(p[1])) for p in intersections]
    if prev is None:
        return candidates[0]
    return min(candidates, key=lambda p: (p[0] - prev[0]) ** 2 + (p[1] - prev[1]) ** 2)


# ---------------- Path generation ----------------

def compute_naive_path(
    solver: IKSolver,
    *,
    path_length: float,
    start_angles: Tuple[float, float],
    end_angles: Tuple[float, float],
    total_time: float,
    steps: int = 600,
) -> List[Vec2]:
    theta1 = start_angles[0]
    theta2 = start_angles[1]
    delta1 = shortest_angle_delta(theta1, end_angles[0])
    delta2 = shortest_angle_delta(theta2, end_angles[1])
    target1 = theta1 + delta1
    target2 = theta2 + delta2

    sign1 = 1.0 if delta1 >= 0 else -1.0
    sign2 = 1.0 if delta2 >= 0 else -1.0

    max_delta = max(abs(delta1), abs(delta2))
    if total_time <= 0:
        total_time = max_delta / max(LINEAR_SPEED, 1e-6)
    omega_common = max_delta / total_time if total_time > 0 else 0.0
    dt = total_time / steps if steps > 0 else total_time

    path = [START_POINT]
    prev = START_POINT
    for _ in range(steps):
        if omega_common == 0:
            break
        theta1 += sign1 * omega_common * dt
        theta2 += sign2 * omega_common * dt

        if sign1 > 0 and theta1 > target1:
            theta1 = target1
        elif sign1 < 0 and theta1 < target1:
            theta1 = target1

        if sign2 > 0 and theta2 > target2:
            theta2 = target2
        elif sign2 < 0 and theta2 < target2:
            theta2 = target2

        pos = angles_to_position(wrap_angle(theta1), wrap_angle(theta2), prev=prev)
        if pos != prev:
            path.append(pos)
            prev = pos

        if theta1 == target1 and theta2 == target2:
            break

    if path[-1] != END_POINT:
        path.append(END_POINT)
    return path


def compute_planner_path(
    planner: PathPlanner,
    segments: Sequence,
    *,
    start_point: Vec2,
) -> List[Vec2]:
    path = [start_point]
    prev = start_point
    for seg in segments:
        samples = max(2, int(math.ceil(seg.length / 5.0)))
        for i in range(1, samples + 1):
            t = min(seg.duration, seg.duration * i / samples)
            theta1 = seg.b1_start + seg.b1_velocity * t
            theta2 = seg.b2_start + seg.b2_velocity * t
            pos = angles_to_position(wrap_angle(theta1), wrap_angle(theta2), prev=prev)
            if pos != prev:
                path.append(pos)
                prev = pos
    if path[-1] != END_POINT:
        path.append(END_POINT)
    return path


def generate_desired_path(start: Vec2, end: Vec2, samples: int) -> List[Vec2]:
    if samples <= 1:
        return [start, end]
    return [
        (
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
        )
        for t in np.linspace(0.0, 1.0, samples)
    ]


def solver_to_screen(p: Vec2) -> Tuple[int, int]:
    return (int(SCREEN_ORIGIN_X + p[0]), int(SCREEN_ORIGIN_Y + p[1]))


def draw_circle(surface: pygame.Surface, color: Tuple[int, int, int], center: Vec2, radius: float, width: int = 1) -> None:
    pygame.draw.circle(surface, color, solver_to_screen(center), int(radius), width)


def draw_polyline(surface: pygame.Surface, color: Tuple[int, int, int], points: Iterable[Vec2]) -> None:
    pts = [solver_to_screen(p) for p in points]
    if len(pts) >= 2:
        pygame.draw.lines(surface, color, False, pts, 2)


# ---------------- Main visual demo ----------------

def main() -> None:
    solver = IKSolver(r_a=R_A, r_b=R_B, b_offset=B_OFFSET)
    planner = PathPlanner(solver, max_segment_length=MAX_SEGMENT_LENGTH, linear_speed=LINEAR_SPEED)

    start_angles = solver.angles(START_POINT)
    end_angles = solver.angles(END_POINT)
    if start_angles is None or end_angles is None:
        raise SystemExit("Start or end point unreachable")

    path_length = math.hypot(END_POINT[0] - START_POINT[0], END_POINT[1] - START_POINT[1])
    total_time = path_length / LINEAR_SPEED if LINEAR_SPEED > 0 else 1.0

    naive_path = compute_naive_path(
        solver,
        path_length=path_length,
        start_angles=start_angles,
        end_angles=end_angles,
        total_time=total_time,
        steps=100,
    )

    segments = planner.plan(START_POINT, END_POINT)
    planner_path = compute_planner_path(planner, segments, start_point=START_POINT)

    desired_points = generate_desired_path(START_POINT, END_POINT, max(len(planner_path), len(naive_path)))

    pygame.init()
    pygame.display.set_caption("Path Planner Comparison")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 18)

    idx = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False

        idx = min(idx + 1, max(len(naive_path), len(planner_path)) - 1)

        screen.fill(COL_BG)

        # draw base line
        pygame.draw.line(screen, COL_BASIS, solver_to_screen((-B_OFFSET, 0.0)), solver_to_screen((B_OFFSET, 0.0)), 1)
        draw_circle(screen, COL_B_CIR, (-B_OFFSET, 0.0), R_B, width=2)
        draw_circle(screen, COL_B_CIR, (B_OFFSET, 0.0), R_B, width=2)

        draw_polyline(screen, COL_LINE, desired_points)
        draw_polyline(screen, COL_NAIVE, naive_path[: idx + 1])
        draw_polyline(screen, COL_PLANNER, planner_path[: idx + 1])

        # draw points
        if idx < len(naive_path):
            pygame.draw.circle(screen, COL_NAIVE, solver_to_screen(naive_path[idx]), 4)
        if idx < len(planner_path):
            pygame.draw.circle(screen, COL_PLANNER, solver_to_screen(planner_path[idx]), 4)

        # legend
        legend_lines = [
            "Compare end-effector path:",
            "  cyan = desired straight line",
            "  red = naive equal-speed joints",
            "  green = planned uniform-speed motion",
        ]
        for i, line in enumerate(legend_lines):
            txt = font.render(line, True, COL_TEXT)
            screen.blit(txt, (20, 20 + i * 18))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
