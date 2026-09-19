"""Smoke test for MF96Tire.

Runs both the canonical and matlab_compat forms over a slip sweep and prints the
forces they produce, so you can see how far apart they are before choosing one.

Uses placeholder coefficients calibrated for realistic peak friction, NOT real
TTC fits. Swap in the real vectors to test properly.

Usage:
    python tire_check.py
"""
import numpy as np
from tire import MF96Tire

rng = np.random.default_rng(0)
# Calibrated so the canonical form gives mu_y ~ 1.5 peaking near 8 deg and
# mu_x ~ 1.7 peaking near 0.12 slip ratio. Fz is in kN, as the fits expect.
a_Fy = np.array([1.35, -0.10, 1.60, 38.5, 2.0, 0.0, -0.05, 0.60,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
a_Fx = np.array([1.65, -0.10, 1.77, 36.7, 2.0, 0.0, -0.05, 0.50,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
a_GFy = np.array([1.20, 0.0, 0.0, 12.0, 6.0, 0.0, -0.10, 0.40,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
a_GFx = np.array([1.0, 12.0, 0.0, 10.0, 1.10, -0.20, 0.0, 0.0])

Fz = np.full(4, 700.0)
alpha = np.radians(np.array([-8.0, -3.0, 3.0, 8.0]))
kappa = np.array([0.0, 0.05, -0.05, 0.1])
gamma = np.radians(np.array([-1.5, 1.5, -1.0, 1.0]))

for compat in (False, True):
    t = MF96Tire(a_Fy, a_Fx, a_GFy, a_GFx, radius=0.2032, matlab_compat=compat)
    Fx, Fy, _ = t.forces(alpha, kappa, gamma, Fz)
    tag = "matlab_compat" if compat else "canonical    "
    print(f"{tag}  Fy = {np.round(Fy,1)}")
    print(f"{tag}  Fx = {np.round(Fx,1)}")

# Peak lateral force across a slip sweep, both forms
sweep = np.radians(np.linspace(-15, 15, 301))
z = np.full_like(sweep, 700.0)
for compat in (False, True):
    t = MF96Tire(a_Fy, a_Fx, a_GFy, a_GFx, radius=0.2032, matlab_compat=compat)
    _, Fy, _ = t.forces(sweep, np.zeros_like(sweep), np.zeros_like(sweep), z)
    i = np.argmax(np.abs(Fy))
    print(f"{'matlab_compat' if compat else 'canonical    '}  peak |Fy| = {abs(Fy[i]):7.1f} N "
          f"at {np.degrees(sweep[i]):+.2f} deg   (mu = {abs(Fy[i])/700:.2f})")
