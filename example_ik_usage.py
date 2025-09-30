"""Example usage of the inverse kinematics solver without any GUI.
Run: python example_ik_usage.py
"""
from inverse_kinematics import IKSolver

# Explicit geometry (example numbers – replace with your hardware values)
R_A = 180.0
R_B = 90.0
B_OFFSET = 50.0
WIDTH = 800.0
HEIGHT = 600.0

solver = IKSolver(r_a=R_A, r_b=R_B, b_offset=B_OFFSET, width=WIDTH, height=HEIGHT,
                  flip_b1=False, flip_b2=False)

# Sample target positions for circle A
targets = [
    (WIDTH * 0.5, HEIGHT * 0.5 - 20),  # nominal
    (WIDTH * 0.5 + 30, HEIGHT * 0.5 - 40),
    (WIDTH * 0.5 - 10, HEIGHT * 0.5 + 60),
]

for t in targets:
    angles = solver.angles(t)
    if angles is None:
        print(f"Target {t}: unreachable")
    else:
        a1, a2 = angles
        print(f"Target {t}: angle_b1={a1:.3f} rad, angle_b2={a2:.3f} rad")
