"""Example usage of the inverse kinematics solver without any GUI.
Run: python example_ik_usage.py
"""
from inverse_kinematics import IKSolver

# Explicit geometry (replace with your hardware values)
R_A = 180.0      # Radius of moving circle A
R_B = 90.0       # Radius of each base circle
B_OFFSET = 50.0  # Half distance between base centers (bases at ±B_OFFSET on X)

solver = IKSolver(r_a=R_A, r_b=R_B, b_offset=B_OFFSET, flip_b1=False, flip_b2=False)

# Sample target positions for circle A (in solver coordinates, origin midway between bases)
targets = [
    (0.0, -20.0),   # directly above midpoint
    (30.0, -40.0),  # up and right
    (-10.0, 60.0),  # down and left
]

for t in targets:
    angles = solver.angles(t)
    if angles is None:
        print(f"Target {t}: unreachable")
    else:
        a1, a2 = angles
        print(f"Target {t}: angle_b1={a1:.3f} rad, angle_b2={a2:.3f} rad")
