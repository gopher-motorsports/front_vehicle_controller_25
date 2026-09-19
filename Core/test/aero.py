"""
Aerodynamic package: downforce and drag as a function of speed.

Sign convention: the frame is z-up and `cla` is NEGATIVE for downforce, so
`forces()` returns a negative lift and a negative (rearward) drag. Use
`downforce()` and `drag_magnitude()` if you want positive numbers.

Usage:

    aero = AeroPackage()                      # default coefficients
    aero.shift_fap([lf, 0.0, -cg_height])     # CG-relative application point
    Fz_extra = aero.downforce(speed)          # N, positive
    Fx_loss  = aero.drag_magnitude(speed)     # N, positive

`shift_fap` must be called once before `x_app`/`z_app` are meaningful;
`FourCornerVehicle` does this for you in its constructor.
"""
import numpy as np

RHO_DEFAULT = 1.191   # air density (kg/m^3)
RA_DEFAULT = 1.0      # static reference area (m^2)

# Default aero package.
DUMMY_APP_POINT = np.array([-0.7047550, 0.0, 0.3763982])
DUMMY_CLA = -4.426
DUMMY_CDA = 1.901


class AeroPackage:
    """Fixed-coefficient lift and drag."""

    def __init__(self, app_point_ref=None, cla=DUMMY_CLA, cda=DUMMY_CDA,
                 ra=RA_DEFAULT, rho=RHO_DEFAULT):
        self.app_point_ref = np.asarray(
            DUMMY_APP_POINT if app_point_ref is None else app_point_ref,
            dtype=float)
        self.cla = float(cla)
        self.cda = float(cda)
        self.ra = float(ra)
        self.rho = float(rho)
        self.app_point = self.app_point_ref.copy()

    def shift_fap(self, shift):
        """Move the application point into CG-relative coordinates.

        `shift` is the front axle position relative to the CG, [lf, 0, -h].
        """
        self.app_point = self.app_point_ref + np.asarray(shift, dtype=float)
        return self.app_point

    @staticmethod
    def newton_load_from_coeffs(ra, cl, rho, speed):
        return 0.5 * ra * cl * rho * speed ** 2

    def forces(self, speed):
        """(F_lift, F_drag) in newtons. Both negative: down and rearward."""
        f_lift = self.newton_load_from_coeffs(self.ra, self.cla, self.rho, speed)
        f_drag = -self.newton_load_from_coeffs(self.ra, self.cda, self.rho, speed)
        return f_lift, f_drag

    def downforce(self, speed):
        """Downforce as a positive number."""
        return -self.forces(speed)[0]

    def drag_magnitude(self, speed):
        """Drag as a positive number opposing motion."""
        return -self.forces(speed)[1]

    @property
    def x_app(self):
        """Application point ahead of the CG (m). Needs `shift_fap` first."""
        return float(self.app_point[0])

    @property
    def z_app(self):
        """Application point above the CG (m). Needs `shift_fap` first."""
        return float(self.app_point[2])
