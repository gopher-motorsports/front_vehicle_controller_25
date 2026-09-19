"""
Nonlinear 4-corner vehicle dynamics model.

Planar rigid body (vx, vy, yaw rate) with four independently computed tire
forces, per-wheel spin dynamics, quasi-static load transfer, and an aero
package. Tire forces are delegated to a tire object from `tire.py`.

Default values in `VehicleParams` are real vehicle numbers except where marked
PLACEHOLDER.

Not modeled: roll, pitch and heave as degrees of freedom, suspension travel,
chassis torsional deflection. These are left out because the model has to run
millions of steps for reinforcement learning. Lateral load transfer is split
front/rear by `roll_stiffness_dist_f` instead, a quasi-static stand-in for the
spring and anti-roll-bar rates.

State (10,): [x, y, yaw, vx, vy, r, w_fl, w_fr, w_rl, w_rr]
  x, y   -- global position (m)
  yaw    -- heading (rad)
  vx, vy -- body-frame velocities (m/s)
  r      -- yaw rate (rad/s)
  w_*    -- wheel angular velocities (rad/s), order FL, FR, RL, RR

Action (2,): [steer_norm, accel_norm], both in [-1, 1]
  steer_norm -> front steer angle = steer_norm * max_steer_angle
  accel_norm >= 0 -> throttle, accel_norm < 0 -> brake

Usage:

    vehicle = FourCornerVehicle()                    # default params
    vehicle = FourCornerVehicle(VehicleParams(mass=300.0), tire=my_tire)
    vehicle.reset(x=0.0, y=0.0, yaw=0.0, vx=4.0)
    state, info = vehicle.step([steer, accel], dt=0.02)

`info` carries per-corner diagnostics: Fz, alpha, kappa, Fx, Fy, r_eff, and the
CG accelerations ax, ay.
"""
from dataclasses import dataclass, field
import numpy as np

from aero import AeroPackage
from tire import SimpleTire

# Wheel order used everywhere in this file: FL, FR, RL, RR
FL, FR, RL, RR = 0, 1, 2, 3
EPS = 1e-3


@dataclass
class VehicleParams:
    # --- Mass / inertia ---
    mass: float = 285.0            # kg
    yaw_inertia: float = 150.0     # kg*m^2 (Izz)
    roll_inertia: float = 90.0     # kg*m^2 (Ixx), unused by this planar model
    pitch_inertia: float = 150.0   # kg*m^2 (Iyy), unused by this planar model
    wheel_inertia: float = 0.6     # kg*m^2 per wheel (PLACEHOLDER)

    # --- Geometry ---
    lf: float = 0.75               # m, CG to front axle
    lr: float = 0.75               # m, CG to rear axle
    track_f: float = 1.30          # m, front track width
    track_r: float = 1.30          # m, rear track width
    cg_height: float = 0.33        # m
    # Unloaded radius measured from the TTC "Springrate 16x7.5W7 R20 12psi" run
    # (see tire.py). Was 0.2286 m = 9 in, which is wrong for a 16 in tire -- the
    # nominal is 8 in and the measured unloaded value is 7.899 in. Wheel radius
    # scales drive torque directly into longitudinal force and sets the slip
    # ratio denominator, so a 14% error here is not cosmetic.
    wheel_radius: float = 0.2006   # m (7.899 in, measured unloaded)
    # Section width of the 16x7.5 tire. Sets how close the car can run to the
    # cones: the legal line keeps the outer tire edge clear of the cone bases.
    tire_width: float = 0.1905     # m (7.5 in)
    # Front axle to the foremost point of the car (nose). Staging and timing
    # lines use the foremost point (DD.4.2.2, DD.4.3.2). PLACEHOLDER.
    front_overhang: float = 0.60   # m
    g: float = 9.81

    # --- Pacejka simplified Magic Formula coefficients (PLACEHOLDER) ---
    # Lateral: Fy = Fz * D_lat * sin(C_lat * atan(B_lat*a - E_lat*(B_lat*a - atan(B_lat*a))))
    B_lat: float = 10.0
    C_lat: float = 1.9
    D_lat: float = 1.55     # peak lateral friction coefficient (mu_y)
    E_lat: float = 0.97
    # Longitudinal: Fx = Fz * D_lon * sin(C_lon * atan(B_lon*k - E_lon*(B_lon*k - atan(B_lon*k))))
    B_lon: float = 11.0
    C_lon: float = 1.65
    D_lon: float = 1.7      # peak longitudinal friction coefficient (mu_x)
    E_lon: float = 0.98

    # --- Static camber per corner (PLACEHOLDER), radians ---
    # Positive means the top of the tire leans outboard.
    camber_fl: float = np.radians(-1.5)
    camber_fr: float = np.radians(1.5)
    camber_rl: float = np.radians(-1.0)
    camber_rr: float = np.radians(1.0)

    # --- Powertrain / brakes ---
    drivetrain: str = "RWD"          # "RWD", "FWD", or "AWD"
    max_motor_torque: float = 175.0  # N*m per driven wheel
    max_brake_torque: float = 2000.0  # N*m total, all four wheels at full pedal
                                      # (PLACEHOLDER)
    front_brake_bias: float = 0.50   # 1 = 100% front
    max_steer_angle: float = np.radians(25.0)  # front wheels
    ackermann: float = 1.0           # 1 = 100% Ackermann

    # --- Roll stiffness distribution ---
    # Fraction of total roll stiffness at the front axle, governing the
    # front/rear split of lateral load transfer. Raising it moves load to the
    # front in a corner and adds understeer, the same balance knob a stiffer
    # front anti-roll bar gives you. None falls back to lr/wheelbase, i.e. a
    # split by static weight distribution.
    roll_stiffness_dist_f: float = None

    # --- Aero ---
    # cla is NEGATIVE for downforce; see aero.py for the sign convention.
    cla: float = -4.426
    cda: float = 1.901
    aero_ra: float = 1.0
    aero_rho: float = 1.191
    aero_app_point: tuple = (-0.7047550, 0.0, 0.3763982)

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr

    @property
    def half_width(self) -> float:
        """CG to the outer edge of the widest tire (m)."""
        return max(self.track_f, self.track_r) / 2.0 + self.tire_width / 2.0

    @property
    def roll_dist_f(self) -> float:
        if self.roll_stiffness_dist_f is None:
            return self.lr / self.wheelbase
        return float(self.roll_stiffness_dist_f)

    def camber(self):
        return np.array([self.camber_fl, self.camber_fr,
                         self.camber_rl, self.camber_rr])

    def driven_wheels(self):
        if self.drivetrain == "RWD":
            return [RL, RR]
        if self.drivetrain == "FWD":
            return [FL, FR]
        return [FL, FR, RL, RR]  # AWD


class FourCornerVehicle:
    """Stateful nonlinear 4-corner vehicle. Call reset() then step(action, dt)."""

    def __init__(self, params: VehicleParams = None, tire=None, aero=None):
        self.p = params or VehicleParams()

        # Tire model. Any object with .forces() and .loaded_radius() works.
        self.tire = tire or SimpleTire(
            B_lat=self.p.B_lat, C_lat=self.p.C_lat, D_lat=self.p.D_lat, E_lat=self.p.E_lat,
            B_lon=self.p.B_lon, C_lon=self.p.C_lon, D_lon=self.p.D_lon, E_lon=self.p.E_lon,
        )

        # Aero package. The application point is shifted into CG-relative
        # coordinates once, here.
        self.aero = aero or AeroPackage(
            app_point_ref=self.p.aero_app_point,
            cla=self.p.cla, cda=self.p.cda,
            ra=self.p.aero_ra, rho=self.p.aero_rho,
        )
        self.aero.shift_fap([self.p.lf, 0.0, -self.p.cg_height])

        self.state = np.zeros(10)
        self._prev_ax = 0.0
        self._prev_ay = 0.0

    def reset(self, x=0.0, y=0.0, yaw=0.0, vx=1.0):
        """vx defaults to a small nonzero value to avoid tire-slip singularities at a dead stop."""
        p = self.p
        w0 = vx / p.wheel_radius
        self.state = np.array([x, y, yaw, vx, 0.0, 0.0, w0, w0, w0, w0])
        self._prev_ax = 0.0
        self._prev_ay = 0.0
        return self.state.copy()

    # ---------- corner geometry ----------
    def _corner_offsets(self):
        p = self.p
        x_off = np.array([p.lf, p.lf, -p.lr, -p.lr])
        y_off = np.array([p.track_f / 2, -p.track_f / 2, p.track_r / 2, -p.track_r / 2])
        return x_off, y_off

    def _steer_angles(self, delta):
        """Per-wheel steer angles [FL, FR, RL, RR] including Ackermann.

        The inside front wheel steers more than the outside; `ackermann`
        scales how much.
        """
        p = self.p
        if delta == 0.0:
            return np.zeros(4)
        # x is the longitudinal distance from the front axle to the turn center
        x = p.wheelbase / np.tan(delta)
        d_left = (np.arctan(p.wheelbase / (x - p.track_f / 2.0)) - delta) * p.ackermann
        d_right = (np.arctan(p.wheelbase / (x + p.track_f / 2.0)) - delta) * p.ackermann
        return np.array([delta + d_left, delta + d_right, 0.0, 0.0])

    def _normal_loads(self, ax, ay, speed=0.0):
        p = self.p
        wb = p.wheelbase
        static_front = p.mass * p.g * (p.lr / wb) / 2.0
        static_rear = p.mass * p.g * (p.lf / wb) / 2.0

        # --- aero ---
        # Downforce acts at x_app, not at the CG, so it splits front/rear by
        # moment balance about the rear contact patch rather than by weight.
        f_down = self.aero.downforce(speed)
        x_a, z_a = self.aero.x_app, self.aero.z_app
        aero_front = f_down * (x_a + p.lr) / wb
        aero_rear = f_down - aero_front

        # Drag acts at z_app. `d_long` below already treats every longitudinal
        # force, drag included, as acting at CG height, so only the height
        # difference is corrected here to avoid double-counting.
        f_drag = self.aero.drag_magnitude(speed)
        d_long_aero = f_drag * (z_a - p.cg_height) / wb

        d_long = p.mass * ax * p.cg_height / wb + d_long_aero
        # Lateral transfer splits by roll stiffness, not weight distribution.
        roll_f = p.roll_dist_f
        d_lat_f = p.mass * ay * p.cg_height / p.track_f * roll_f
        d_lat_r = p.mass * ay * p.cg_height / p.track_r * (1.0 - roll_f)

        static_front = static_front + aero_front / 2.0
        static_rear = static_rear + aero_rear / 2.0

        Fz = np.array([
            static_front - d_long / 2 - d_lat_f,   # FL
            static_front - d_long / 2 + d_lat_f,   # FR
            static_rear + d_long / 2 - d_lat_r,    # RL
            static_rear + d_long / 2 + d_lat_r,    # RR
        ])
        return np.clip(Fz, 0.0, None)  # no negative load (wheel lift -> zero force, no rollover model)

    # ---------- main integration step ----------
    def step(self, action, dt, substeps=5):
        """action = [steer_norm, accel_norm] in [-1, 1]. Integrates dt using `substeps` RK-ish sub-steps."""
        steer_norm, accel_norm = float(np.clip(action[0], -1, 1)), float(np.clip(action[1], -1, 1))
        h = dt / substeps
        info = {}
        for _ in range(substeps):
            info = self._substep(steer_norm, accel_norm, h)
        return self.state.copy(), info

    def _substep(self, steer_norm, accel_norm, h):
        p = self.p
        x, y, yaw, vx, vy, r, w_fl, w_fr, w_rl, w_rr = self.state
        omega = np.array([w_fl, w_fr, w_rl, w_rr])

        delta = steer_norm * p.max_steer_angle
        steer = self._steer_angles(delta)

        x_off, y_off = self._corner_offsets()
        speed = float(np.hypot(vx, vy))
        Fz = self._normal_loads(self._prev_ax, self._prev_ay, speed)

        # wheel-contact-point body-frame velocity
        vwx = vx - r * y_off
        vwy = vy + r * x_off

        # rotate into tire-aligned frame
        c, s = np.cos(steer), np.sin(steer)
        vx_t = vwx * c + vwy * s
        vy_t = -vwx * s + vwy * c

        alpha = np.arctan2(vy_t, np.where(np.abs(vx_t) < EPS, EPS, vx_t))
        # Loaded radius per corner: the tire deflects under load, so this
        # varies with both weight transfer and speed (through downforce).
        r_eff = self.tire.loaded_radius(Fz)

        vx_t_safe = np.where(np.abs(vx_t) < EPS, np.sign(vx_t) * EPS + EPS, vx_t)
        kappa = (omega * r_eff - vx_t_safe) / np.maximum(np.abs(vx_t_safe), EPS)
        kappa = np.clip(kappa, -1.5, 1.5)

        # Tire forces, including static camber per corner.
        Fx_t, Fy_t, _Mz = self.tire.forces(alpha, kappa, p.camber(), Fz)

        # rotate tire forces back to body frame
        Fx_b = Fx_t * c - Fy_t * s
        Fy_b = Fx_t * s + Fy_t * c

        # --- drive / brake torque commands ---
        drive_t = np.zeros(4)
        brake_t = np.zeros(4)
        if accel_norm >= 0:
            driven = p.driven_wheels()
            per_wheel_torque = accel_norm * p.max_motor_torque
            for wi in driven:
                drive_t[wi] = per_wheel_torque
        else:
            # Brake bias: total pedal demand is split front/rear by
            # `front_brake_bias`, then halved across each axle's two wheels.
            pedal = -accel_norm * p.max_brake_torque
            axle_torque = np.array([
                pedal * p.front_brake_bias / 2.0, pedal * p.front_brake_bias / 2.0,
                pedal * (1.0 - p.front_brake_bias) / 2.0, pedal * (1.0 - p.front_brake_bias) / 2.0,
            ])
            brake_t[:] = axle_torque * -np.sign(omega)

        wheel_torque = drive_t + brake_t - Fx_t * r_eff
        domega = wheel_torque / p.wheel_inertia

        # Clamp so brake torque cannot drive a wheel backwards through zero
        # within one sub-step.
        over_brake = (accel_norm < 0) & (np.sign(omega + domega * h) != np.sign(omega)) & (omega != 0)
        domega = np.where(over_brake, -omega / h, domega)

        # --- chassis forces / moments ---
        # Downforce enters through Fz in _normal_loads(); drag acts here.
        drag = self.aero.drag_magnitude(speed) * np.sign(vx if vx != 0 else 1.0)
        Fx_net = np.sum(Fx_b) - drag
        Fy_net = np.sum(Fy_b)

        M = np.sum(x_off * Fy_b - y_off * Fx_b)

        # Specific force: what an accelerometer at the CG reads. This, not the
        # body-frame velocity derivative below, is what drives weight transfer.
        ax_sf = Fx_net / p.mass
        ay_sf = Fy_net / p.mass

        # Body-frame velocity derivatives, for integration only. The extra r*v
        # terms are kinematic, not real forces, which is why they are excluded
        # from the specific force above.
        ax = ax_sf + r * vy
        ay = ay_sf - r * vx
        r_dot = M / p.yaw_inertia

        # integrate (semi-implicit Euler for the sub-step)
        vx_new = vx + ax * h
        vy_new = vy + ay * h
        r_new = r + r_dot * h
        yaw_new = yaw + r_new * h
        x_new = x + (vx_new * np.cos(yaw) - vy_new * np.sin(yaw)) * h
        y_new = y + (vx_new * np.sin(yaw) + vy_new * np.cos(yaw)) * h
        omega_new = omega + domega * h

        self.state = np.array([x_new, y_new, yaw_new, vx_new, vy_new, r_new, *omega_new])
        self._prev_ax, self._prev_ay = ax_sf, ay_sf

        return {
            "Fz": Fz, "alpha": alpha, "kappa": kappa,
            "Fx": Fx_t, "Fy": Fy_t, "ax": ax, "ay": ay, "r_eff": r_eff,
        }
