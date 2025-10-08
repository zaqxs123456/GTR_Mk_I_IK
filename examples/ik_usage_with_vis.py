"""Interactive visualization demo for the dual-circle IK solver.

This script uses pygame to:
  - Track the mouse position as the moving circle A center.
  - Draw two fixed base circles (B1, B2) and circle A.
  - Display the chosen intersection points and leg angles (clockwise from UP).

Prerequisites:
    pip install pygame numpy

Run from project root:
    python examples/ik_usage_with_vis.py

Close the window or press ESC to exit.

NOTE: This file is for debugging / demonstration only and is optional.
"""
from __future__ import annotations
import math
import sys
from pathlib import Path
from typing import Tuple

sys.path.append(str(Path(__file__).resolve().parents[1]))

try:
    import pygame  # type: ignore
except ImportError as e:  # pragma: no cover
    print("pygame not installed. Install with: pip install pygame")
    sys.exit(1)

from ik_solver import IKSolver

Color = Tuple[int, int, int]

# ---------------- Configuration (edit as needed) ----------------
WIDTH, HEIGHT = 800, 600          # Only for visualization window
R_A = 180.0                      # Radius of moving circle A (solver units)
R_B = 90.0                       # Radius of each base circle (solver units)
B_OFFSET = 50.0                  # Base half-spacing: bases at (-B_OFFSET,0) and (+B_OFFSET,0)
FLIP_B1 = False     # Invert angle sign for leg B1
FLIP_B2 = False     # Invert angle sign for leg B2
BG_COLOR: Color = (20, 20, 25)
CIRCLE_A_COLOR: Color = (120, 180, 255)
CIRCLE_B_COLOR: Color = (255, 170, 120)
LEG_COLOR_1: Color = (150, 210, 160)
LEG_COLOR_2: Color = (210, 160, 150)
INTERSECTION_COLOR: Color = (255, 80, 80)
TEXT_COLOR: Color = (235, 235, 235)
GUIDE_COLOR: Color = (90, 100, 120)
FPS = 60

SCREEN_ORIGIN_X = WIDTH * 0.5   # Screen pixel where solver (0,0) is drawn
SCREEN_ORIGIN_Y = HEIGHT * 0.5  # Placed at visual center

# Instantiate solver (origin at 0,0 midpoint between bases)
SOLVER = IKSolver(r_a=R_A, r_b=R_B, b_offset=B_OFFSET,
                  flip_b1=FLIP_B1, flip_b2=FLIP_B2)

def solver_to_screen(p):
    # (x,y) with +y down already matches screen y direction
    return (int(SCREEN_ORIGIN_X + p[0]), int(SCREEN_ORIGIN_Y + p[1]))

def screen_to_solver(px, py):
    return (float(px - SCREEN_ORIGIN_X), float(py - SCREEN_ORIGIN_Y))

# Precompute base centers in screen coordinates
B1_CENTER_SCREEN = solver_to_screen((-B_OFFSET, 0.0))
B2_CENTER_SCREEN = solver_to_screen(( B_OFFSET, 0.0))

# ---------------- Helper drawing functions ----------------

def draw_circle(surface: pygame.Surface, center: Tuple[float, float], r: float, color: Color, width: int = 1):
    pygame.draw.circle(surface, color, (int(center[0]), int(center[1])), int(r), width)

def draw_point(surface: pygame.Surface, p: Tuple[float, float], color: Color, radius: int = 5):
    pygame.draw.circle(surface, color, (int(p[0]), int(p[1])), radius)

# ---------------- Main loop ----------------

def main() -> None:
    pygame.init()
    pygame.display.set_caption("IK Visual Demo")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 18)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        mouse_x, mouse_y = pygame.mouse.get_pos()
        # Convert mouse position (screen) into solver coordinates
        a_center = screen_to_solver(mouse_x, mouse_y)

        solution = SOLVER.solve(a_center)

        screen.fill(BG_COLOR)

        # Circles
        draw_circle(screen, solver_to_screen(a_center), R_A, CIRCLE_A_COLOR, width=2)
        draw_circle(screen, B1_CENTER_SCREEN, R_B, CIRCLE_B_COLOR, width=2)
        draw_circle(screen, B2_CENTER_SCREEN, R_B, CIRCLE_B_COLOR, width=2)

        # Guides from A to bases
        pygame.draw.line(screen, GUIDE_COLOR, solver_to_screen(a_center), B1_CENTER_SCREEN, 1)
        pygame.draw.line(screen, GUIDE_COLOR, solver_to_screen(a_center), B2_CENTER_SCREEN, 1)

        # B1 leg
        b1 = solution.b1
        if b1.intersection is not None:
            ix, iy = b1.intersection
            p_inter = solver_to_screen((ix, iy))
            pygame.draw.line(screen, LEG_COLOR_1, B1_CENTER_SCREEN, p_inter, 2)
            pygame.draw.line(screen, LEG_COLOR_2, p_inter, solver_to_screen(a_center), 2)
            draw_point(screen, p_inter, INTERSECTION_COLOR)
            if b1.angle_from_up_cw is not None:
                label = f"b1 {math.degrees(b1.angle_from_up_cw):.1f}°"  # 0–360°
            else:
                label = "b1 --"
            ts = font.render(label, True, TEXT_COLOR)
            screen.blit(ts, (p_inter[0] + 6, p_inter[1] - 6))
        else:
            ts = font.render("b1 unreachable", True, TEXT_COLOR)
            screen.blit(ts, (B1_CENTER_SCREEN[0] - 40, B1_CENTER_SCREEN[1] - 60))

        # B2 leg
        b2 = solution.b2
        if b2.intersection is not None:
            ix, iy = b2.intersection
            p_inter = solver_to_screen((ix, iy))
            pygame.draw.line(screen, LEG_COLOR_1, B2_CENTER_SCREEN, p_inter, 2)
            pygame.draw.line(screen, LEG_COLOR_2, p_inter, solver_to_screen(a_center), 2)
            draw_point(screen, p_inter, INTERSECTION_COLOR)
            if b2.angle_from_up_cw is not None:
                label = f"b2 {math.degrees(b2.angle_from_up_cw):.1f}°"  # 0–360°
            else:
                label = "b2 --"
            ts = font.render(label, True, TEXT_COLOR)
            screen.blit(ts, (p_inter[0] + 6, p_inter[1] - 6))
        else:
            ts = font.render("b2 unreachable", True, TEXT_COLOR)
            screen.blit(ts, (B2_CENTER_SCREEN[0] - 40, B2_CENTER_SCREEN[1] - 60))

        # Combined status
        if not solution.all_reachable:
            status = "UNREACHABLE"
            color = (255, 90, 90)
        else:
            status = "OK"
            color = (120, 220, 140)
        status_surf = font.render(status, True, color)
        screen.blit(status_surf, (10, 10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()
