"""
Tire models. Both expose the same interface:

    Fx, Fy, Mz = tire.forces(alpha_rad, kappa, gamma_rad, Fz_newtons)
    radius     = tire.loaded_radius(Fz_newtons)

`SimpleTire` is a simplified Magic Formula with a friction-circle combined-slip
model and linear camber thrust. Its shape coefficients are placeholders; use it
when you want something fast that behaves sensibly.

`MF96Tire` is the full load- and camber-sensitive Pacejka fit. It needs TTC
coefficient vectors (`a_Fy` 18, `a_Fx` 19, `a_GFy` 19, `a_GFx` 8); load them
with `MF96Tire.from_npz`.

Both apply grip derates (`INPUT_SCALE` on slip inputs, `OUTPUT_SCALE` on force
outputs), which scale flat-belt test data to a real track surface, and a
measured tire vertical rate for the load-dependent loaded radius.

`MF96Tire(matlab_compat=...)` selects which form of the Magic Formula to
evaluate. `False` (default) uses the canonical Pacejka expressions; `True`
reproduces the reference MATLAB lap simulation exactly, including four places
where it departs from the canonical form. The two agree at peak force but differ
substantially in the linear range, so set this to `True` if you need to match
existing MATLAB results.

Usage:

    tire = SimpleTire()
    Fx, Fy, Mz = tire.forces(alpha, kappa, gamma, Fz)

    tire = MF96Tire.from_npz("HSR_R20_16x75W7_12psi.npz", radius=0.2006)
    vehicle = FourCornerVehicle(tire=tire)

    # Either tire with a calibrated grip derate (see calibrate.py):
    tire = make_tire(npz_path="HSR_R20_16x75W7_12psi.npz", grip_scale=0.81)
"""
import numpy as np

# Grip derates applied to flat-belt test data.
INPUT_SCALE = 0.9    # applied to slip angle, slip ratio and camber
OUTPUT_SCALE = 0.74  # applied to Fx and Fy

# Tire vertical rate and unloaded radius, measured from the TTC run
# "Springrate 16x7.5W7 R20 12psi". Valid over roughly the 50-250 lbf load
# range that fit was taken from.
TIRE_VERTICAL_STIFFNESS = 126785.0   # N/m at 12 psi, 0 deg camber
UNLOADED_RADIUS = 0.2006             # m (7.899 in)


def _loaded_radius(unloaded_radius, k_vertical, Fz):
    """Rolling radius under load: R0 - Fz/k. Linear in load."""
    return unloaded_radius - np.clip(Fz, 0.0, None) / k_vertical


def _mf_core(B, C, D, E, X):
    """Canonical Pacejka core: D*sin(C*atan(B*X - E*(B*X - atan(B*X))))."""
    BX = B * X
    return D * np.sin(C * np.arctan(BX - E * (BX - np.arctan(BX))))


def _mf_core_matlab(B, C, D, E, X):
    """The reference MATLAB form of the core."""
    BX = B * X
    return D * np.sin(C * np.arctan(BX - E * BX - np.arctan(BX)))


class SimpleTire:
    """Simplified Magic Formula with friction-circle combined slip.

    Shape coefficients are placeholders, not fits.
    """

    def __init__(self, B_lat=10.0, C_lat=1.9, D_lat=1.55, E_lat=0.97,
                 B_lon=11.0, C_lon=1.65, D_lon=1.7, E_lon=0.98,
                 camber_stiffness=0.9,
                 input_scale=INPUT_SCALE, output_scale=OUTPUT_SCALE,
                 vertical_stiffness=TIRE_VERTICAL_STIFFNESS,
                 unloaded_radius=UNLOADED_RADIUS):
        self.B_lat, self.C_lat, self.D_lat, self.E_lat = B_lat, C_lat, D_lat, E_lat
        self.B_lon, self.C_lon, self.D_lon, self.E_lon = B_lon, C_lon, D_lon, E_lon
        # Camber thrust per radian of inclination, as a fraction of Fz.
        self.camber_stiffness = camber_stiffness
        self.input_scale = input_scale
        self.output_scale = output_scale
        self.vertical_stiffness = vertical_stiffness
        self.unloaded_radius = unloaded_radius

    def loaded_radius(self, Fz):
        return _loaded_radius(self.unloaded_radius, self.vertical_stiffness, Fz)

    @property
    def mu_avg(self):
        return 0.5 * (self.D_lon + self.D_lat)

    def forces(self, alpha, kappa, gamma, Fz):
        """alpha, gamma in radians; kappa dimensionless; Fz in newtons."""
        Fz = np.clip(Fz, 0.0, None)
        a = alpha * self.input_scale
        k = kappa * self.input_scale

        Fx0 = _mf_core(self.B_lon, self.C_lon, self.D_lon * Fz, self.E_lon, k)
        # sign: alpha>0 (drifting left) must produce a restoring force to the right
        Fy0 = _mf_core(self.B_lat, self.C_lat, self.D_lat * Fz, self.E_lat, -a)
        # Camber thrust acts toward the lean; linear in gamma is adequate below
        # the saturation limit and is what the friction circle then clips.
        Fy0 = Fy0 + Fz * self.camber_stiffness * gamma

        # Friction-circle scaling
        Fmax = self.mu_avg * Fz
        Ftot = np.sqrt(Fx0 ** 2 + Fy0 ** 2) + 1e-9
        scale = np.minimum(1.0, Fmax / Ftot)

        Fx = Fx0 * scale * self.output_scale
        Fy = Fy0 * scale * self.output_scale
        Mz = np.zeros_like(np.asarray(Fx, dtype=float))  # not modeled
        return Fx, Fy, Mz


class MF96Tire:
    """Full load- and camber-sensitive Magic Formula fit.

    Coefficient vectors keep the 1-indexed MATLAB layout but are stored
    0-indexed here, so `a_Fy[0]` is MATLAB's `a_Fy(1)`. Lengths: a_Fy 18,
    a_Fx 19, a_GFy 19, a_GFx 8.

    Takes slip angle and camber in RADIANS (the MATLAB takes degrees) and Fz in
    newtons, converted to kN internally as the fits expect.
    """

    def __init__(self, a_Fy, a_Fx, a_GFy, a_GFx, radius,
                 input_scale=INPUT_SCALE, output_scale=OUTPUT_SCALE,
                 matlab_compat=False,
                 vertical_stiffness=TIRE_VERTICAL_STIFFNESS):
        self.a_Fy = np.asarray(a_Fy, dtype=float)
        self.a_Fx = np.asarray(a_Fx, dtype=float)
        self.a_GFy = np.asarray(a_GFy, dtype=float)
        self.a_GFx = np.asarray(a_GFx, dtype=float)
        for name, arr, n in (("a_Fy", self.a_Fy, 18), ("a_Fx", self.a_Fx, 19),
                             ("a_GFy", self.a_GFy, 19), ("a_GFx", self.a_GFx, 8)):
            if arr.shape != (n,):
                raise ValueError(f"{name} must have shape ({n},), got {arr.shape}")
        self.radius = float(radius)
        self.unloaded_radius = float(radius)
        self.vertical_stiffness = vertical_stiffness
        self.input_scale = input_scale
        self.output_scale = output_scale
        self.matlab_compat = matlab_compat

    def loaded_radius(self, Fz):
        return _loaded_radius(self.unloaded_radius, self.vertical_stiffness, Fz)

    @classmethod
    def from_npz(cls, path, radius, **kwargs):
        """Build a tire from a .npz holding the four coefficient vectors.

        Expected keys: "a_Fy", "a_Fx", "a_GFy", "a_GFx", corresponding to the
        TTC coefficient .mat files:

            a_Fy  <- Cornering  *.mat, variable `a`
            a_Fx  <- DriveBrake *.mat, variable `a`
            a_GFx <- CombinedFX *.mat, variable `a_fx`
            a_GFy <- CombinedFY *.mat, variable `a_fy`
        """
        d = np.load(path)
        return cls(d["a_Fy"], d["a_Fx"], d["a_GFy"], d["a_GFx"], radius, **kwargs)

    def _core(self, B, C, D, E, X):
        if self.matlab_compat:
            return _mf_core_matlab(B, C, D, E, X)
        return _mf_core(B, C, D, E, X)

    def _E_term(self, e_load, camber_factor, X):
        """Curvature factor E."""
        if self.matlab_compat:
            return e_load * (1.0 - camber_factor) * np.sin(X)
        return e_load * (1.0 - camber_factor * np.sign(X))

    def forces(self, alpha, kappa, gamma, Fz):
        Fz = np.asarray(Fz, dtype=float)
        zero = np.zeros_like(Fz)
        active = Fz > 0.0

        Fz_kn = np.where(active, Fz / 1000.0, 1.0)  # kN, dummy 1.0 where inactive
        a = -alpha * self.input_scale
        g = -gamma * self.input_scale
        k = kappa * self.input_scale

        # ---------------- pure lateral ----------------
        A = self.a_Fy
        Sh = A[8] * Fz_kn + A[9] + A[10] * g
        Sv = A[11] * Fz_kn + A[12] + (A[13] * Fz_kn + A[14]) * g * Fz_kn
        C_y = A[0]
        D_y = (A[1] * Fz_kn + A[2]) * Fz_kn * (1.0 - A[15] * g ** 2)
        B_y = (A[3] * np.sin(2.0 * np.arctan(Fz_kn / A[4]))
               * (1.0 - A[5] * np.abs(g))) / (C_y * D_y)
        X_y = a + Sh
        E_y = self._E_term(A[6] * Fz_kn + A[7], A[16] * g + A[17], X_y)
        Fy = Sv + self._core(B_y, C_y, D_y, E_y, X_y)
        Fy = Fy * 1000.0 * self.output_scale

        # ---------------- pure longitudinal ----------------
        A = self.a_Fx
        Sh = A[8] * Fz_kn + A[9] + A[10] * g
        Sv = A[11] * Fz_kn + A[12] + (A[13] * Fz_kn + A[14]) * g * Fz_kn
        C_x = A[0]
        D_x = (A[1] * Fz_kn + A[2]) * Fz_kn * (1.0 - A[15] * g ** 2)
        B_x = (A[3] * np.sin(2.0 * np.arctan(Fz_kn / A[4]))
               * (1.0 - A[5] * np.abs(g))) / (C_x * D_x)
        X_x = A[18] * k + Sh
        E_x = self._E_term(A[6] * Fz_kn + A[7], A[16] * g + A[17], X_x)
        Fx = Sv + self._core(B_x, C_x, D_x, E_x, X_x)
        Fx = Fx * 1000.0 * self.output_scale

        # ---------------- combined slip: Gxa (slip angle derates Fx) ----------
        G = self.a_GFx
        B_gx = (G[1] + G[2] * g ** 2) * np.cos(np.arctan(G[3] * k))
        C_gx, Sh_gx = G[4], G[7]
        E_gx = G[5] + G[6] * Fz_kn
        arg = a * G[0] if self.matlab_compat else a * G[0] + Sh_gx
        num = np.cos(C_gx * np.arctan(B_gx * arg - E_gx * (B_gx * arg - np.arctan(B_gx * arg))))
        den = np.cos(C_gx * np.arctan(B_gx * Sh_gx - E_gx * (B_gx * Sh_gx - np.arctan(B_gx * Sh_gx))))
        Gxa = num / den

        # ---------------- combined slip: Gyk (slip ratio derates Fy) ----------
        G = self.a_GFy
        B_gy = (G[3] * np.cos(2.0 * np.arctan(Fz_kn / G[4]))
                * (1.0 - G[5] * np.abs(g)))
        C_gy = G[0]
        E_gy = G[6] * Fz_kn + G[7]
        Sh_gy = G[8] * Fz_kn + G[9] + G[10] * g
        arg = G[18] * k + Sh_gy
        num = np.cos(C_gy * np.arctan(B_gy * arg - E_gy * (B_gy * arg - np.arctan(B_gy * arg))))
        den = np.cos(C_gy * np.arctan(B_gy * Sh_gy - E_gy * (B_gy * Sh_gy - np.arctan(B_gy * Sh_gy))))
        Gyk = num / den

        Fx = np.where(active, Fx * Gxa, zero)
        Fy = np.where(active, Fy * Gyk, zero)
        Mz = zero.copy()  # not modeled
        return Fx, Fy, Mz


def make_tire(npz_path=None, grip_scale=None, params=None, radius=UNLOADED_RADIUS,
              matlab_compat=False):
    """Build the tire the rest of the code should use.

    With `npz_path`, an MF96Tire from TTC coefficients; otherwise a SimpleTire
    using the coefficients in `params` (a VehicleParams, defaults if None).
    `grip_scale` overrides the output derate (OUTPUT_SCALE), which is the knob
    calibrate.py fits to a measured skidpad time.

    Returns None when neither argument is given, so FourCornerVehicle builds
    its default tire exactly as before.
    """
    if npz_path is None and grip_scale is None:
        return None
    if npz_path is not None:
        tire = MF96Tire.from_npz(npz_path, radius=radius, matlab_compat=matlab_compat)
    else:
        if params is None:
            from vehicle_model import VehicleParams
            params = VehicleParams()
        tire = SimpleTire(
            B_lat=params.B_lat, C_lat=params.C_lat, D_lat=params.D_lat, E_lat=params.E_lat,
            B_lon=params.B_lon, C_lon=params.C_lon, D_lon=params.D_lon, E_lon=params.E_lon,
        )
    if grip_scale is not None:
        tire.output_scale = float(grip_scale)
    return tire
