#!/usr/bin/env python3
# hydra_teleop/physics.py
# FlyCart 30–tuned motion/actuator + environment shim for teleop velocity shaping.
# References (see also bottom of file):
# - DJI FlyCart 30 specs/manual for mass, tilt, speed, wind resistance:
#   https://www.dji.com/flycart-30/specs
#   https://dl.djicdn.com/downloads/DJI_FlyCart_30/202406UM/DJI_FlyCart_30_User_Manual_v1.1.pdf
# - Quadrotor aerodynamic drag modeling & estimation:
#   Hattenberger et al., "Evaluation of drag coefficient for a quadrotor model", 2023.
#   https://journals.sagepub.com/doi/10.1177/17568293221148378
# - Stochastic wind via Ornstein–Uhlenbeck (OU):
#   Shimoni et al., AIAA 2024; Obukhov et al., Energies 2021 (fractional OU).

from __future__ import annotations
import math, random
from typing import Tuple, Deque, Optional
from collections import deque


class DronePhysics:
    """
    FlyCart 30–leaning physics / environment model for teleop velocity shaping.

    Responsibilities:
      - Command latency (FIFO buffer)
      - 2nd-order actuator dynamics (per-axis)
      - Jerk limiting (accel slew-rate limit)
      - Tilt/thrust caps (lateral accel, asym z accel)
      - Aerodynamic drag (linear + quadratic; tune from logs)
      - Wind gusts via Ornstein–Uhlenbeck on acceleration (scaled by wind level)
      - Yaw dynamics (2nd-order + jerk + rate/accel caps)

    Public API:
      - update_cmd(vx, vy, vz, wz): push a new desired velocity
      - step(dt) -> (vx, vy, vz, wz): advance physics by dt seconds
      - reset(): zero states and buffers

    All tunables come from cfg via getattr with sensible FlyCart 30 defaults.
    """

    # ---------- small math utils ----------
    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        return lo if x < lo else hi if x > hi else x

    @staticmethod
    def _vec_mag(x: float, y: float) -> float:
        return math.hypot(x, y)

    # ---------- ctor ----------
    def __init__(self, cfg) -> None:
        # Rate (used only for default dt + latency queue sizing)
        self._default_dt = 1.0 / max(1.0, float(getattr(cfg, "rate_hz", 50.0)))

        # --- FlyCart 30 physical scale ---
        # DJI spec (with two DB2000 batteries): ~65 kg total mass (no payload).
        # Max tilt (pitch angle) 30°, wind resistance up to ~12 m/s, cruise ~15 m/s.
        # Sources in comments above.
        self._mass_kg       = float(getattr(cfg, "mass_kg", 65.0))

        # 2nd-order actuator dynamics (slightly conservative for a heavy lifter)
        self._zeta_lin      = float(getattr(cfg, "zeta_lin", 0.9))
        self._wn_lin        = float(getattr(cfg, "wn_lin_rad", 6.0))   # rad/s
        self._zeta_yaw      = float(getattr(cfg, "zeta_yaw", 0.9))
        self._wn_yaw        = float(getattr(cfg, "wn_yaw_rad", 5.0))

        # Jerk limits (accel slew)
        self._jmax_lin      = float(getattr(cfg, "jerk_max_lin_mps3", 20.0))
        self._jmax_yaw      = float(getattr(cfg, "jerk_max_yaw_rps3", 40.0))

        # Aerodynamic drag (per-axis), with 1/m folded in.
        # Start conservative; fit from logs later (see refs).
        # a_drag ≈ -(k1 * v + k2 * |v| v)
        self._drag_k1       = float(getattr(cfg, "drag_lin_per_s", 0.20))
        self._drag_k2       = float(getattr(cfg, "drag_quad_per_m", 0.04))

        # Gravity & lateral tilt/thrust geometry
        self._g             = 9.80665
        self._max_tilt_deg  = float(getattr(cfg, "max_tilt_deg", 30.0))  # DJI spec
        self._a_xy_max      = self._g * math.tan(math.radians(self._max_tilt_deg))

        # Vertical accel/velocity caps (manual/specs imply rate limits more than accel)
        # We implement *velocity* caps per DJI (5 m/s up, 3 m/s down) plus accel caps.
        self._a_z_up_max    = float(getattr(cfg, "a_z_up_max_mps2", 4.0))
        self._a_z_down_max  = float(getattr(cfg, "a_z_down_max_mps2", 5.0))
        self._vz_up_max     = float(getattr(cfg, "vz_up_max_mps", 5.0))   # ascent cap
        self._vz_down_max   = float(getattr(cfg, "vz_down_max_mps", 3.0)) # descent cap (positive magnitude)

        # Horizontal speed caps — DJI quotes 15 m/s typical limit, 20 m/s max.
        # Default to 15 m/s unless you explicitly allow "sport" 20 m/s.
        self._v_horiz_max   = float(getattr(cfg, "v_horiz_max_mps", 15.0))
        self._v_horiz_abs_max = float(getattr(cfg, "v_horiz_abs_max_mps", 20.0))  # safeguard ceiling

        # Yaw rate/accel caps (conservative for a big coaxial lifter)
        self._wz_max        = float(getattr(cfg, "max_ang_speed_rps", 1.2))  # ~70 deg/s
        self._awz_max       = float(getattr(cfg, "yaw_acc_max_rps2", 6.0))

        # Command latency (radio/stack + operator)
        self._cmd_latency_s = float(getattr(cfg, "cmd_latency_s", 0.10))
        self._cmd_buf: Deque[Tuple[float, float, float, float]] = deque()

        # --- Wind (OU) ---
        # Scale gust strength by a "wind level" (0..1) relative to ~12 m/s spec.
        # This drives acceleration noise (not mean wind) to emulate buffeting.
        self._wind_tau_s    = float(getattr(cfg, "wind_tau_s", 1.5))  # AIAA/OU-style correlation
        self._wind_level    = float(getattr(cfg, "wind_level_0to1", 0.5))
        # Base accel std tuned so that wind_level=1.0 ~ near the 12 m/s resistance case.
        self._wind_std_base = float(getattr(cfg, "wind_accel_std_base_mps2", 0.8))
        self._wind_ax = self._wind_ay = self._wind_az = 0.0

        # Desired commands (latest)
        self._vx_cmd = self._vy_cmd = self._vz_cmd = self._wz_cmd = 0.0

        # Dynamic states
        self._vx = self._vy = self._vz = 0.0
        self._ax = self._ay = self._az = 0.0
        self._wz = 0.0       # yaw rate
        self._awz = 0.0      # yaw angular acceleration

    # ---------- helpers ----------
    def _accel_2nd_order(self, v: float, a: float, v_cmd: float, wn: float, zeta: float) -> float:
        # v'' = wn^2 * (v_cmd - v) - 2*zeta*wn * v'
        return (wn * wn) * (v_cmd - v) - (2.0 * zeta * wn) * a

    def _apply_jerk_limit(self, a_cur: float, a_des: float, jmax: float, dt: float) -> float:
        if dt <= 0.0 or jmax <= 0.0:
            return a_des
        a_step = jmax * dt
        if a_des > a_cur + a_step:  return a_cur + a_step
        if a_des < a_cur - a_step:  return a_cur - a_step
        return a_des

    def _limit_accel_with_physics(self, ax: float, ay: float, az: float) -> Tuple[float, float, float]:
        # Lateral tilt cap (a_xy <= g * tan(max_tilt))
        mag_xy = self._vec_mag(ax, ay)
        if mag_xy > self._a_xy_max:
            s = self._a_xy_max / max(mag_xy, 1e-6)
            ax *= s; ay *= s
        # Asymmetric vertical caps
        if az > self._a_z_up_max:       az = self._a_z_up_max
        elif az < -self._a_z_down_max:  az = -self._a_z_down_max
        return ax, ay, az

    def _update_wind(self, a_prev: float, dt: float, std: float) -> float:
        # Ornstein–Uhlenbeck: a_t = e^{-dt/τ} a_{t-1} + σ * sqrt(1 - e^{-2 dt/τ}) * N(0,1)
        if std <= 0.0 or dt <= 0.0:
            return a_prev
        tau   = max(self._wind_tau_s, 1e-3)
        decay = math.exp(-dt / tau)
        var   = std * math.sqrt(max(0.0, 1.0 - math.exp(-2.0 * dt / tau)))
        return decay * a_prev + var * random.gauss(0.0, 1.0)

    def _push_cmd(self, vx: float, vy: float, vz: float, wz: float, dt: float) -> None:
        self._cmd_buf.append((vx, vy, vz, wz))
        latency = max(self._cmd_latency_s, 0.0)
        max_len = max(1, int(latency / max(dt, 1e-3)))
        while len(self._cmd_buf) > max_len:
            self._cmd_buf.popleft()

    def _peek_delayed_cmd(self) -> Tuple[float, float, float, float]:
        if self._cmd_buf:
            return self._cmd_buf[0]
        return self._vx_cmd, self._vy_cmd, self._vz_cmd, self._wz_cmd

    # ---------- public API ----------
    def update_cmd(self, vx: float, vy: float, vz: float, wz: float) -> None:
        self._vx_cmd = float(vx); self._vy_cmd = float(vy)
        self._vz_cmd = float(vz); self._wz_cmd = float(wz)

    def step(self, dt: Optional[float]) -> Tuple[float, float, float, float]:
        """
        Advance physics by dt seconds (if None, uses default dt).
        Returns the filtered/publishable (vx, vy, vz, wz).
        """
        dt = float(dt if dt is not None else self._default_dt)
        dt = max(1e-4, dt)

        # Maintain latency buffer using the *latest* desired command
        self._push_cmd(self._vx_cmd, self._vy_cmd, self._vz_cmd, self._wz_cmd, dt)
        vx_cmd, vy_cmd, vz_cmd, wz_cmd = self._peek_delayed_cmd()

        # --- Wind update (OU on accel), scaled by wind_level (0..1) ---
        wind_sigma = max(0.0, min(1.0, self._wind_level)) * self._wind_std_base
        self._wind_ax = self._update_wind(self._wind_ax, dt, wind_sigma)
        self._wind_ay = self._update_wind(self._wind_ay, dt, wind_sigma)
        # Vertical gusts a bit weaker:
        self._wind_az = self._update_wind(self._wind_az, dt, wind_sigma * 0.6)

        # --- Linear desired accelerations via 2nd-order model ---
        ax_des = self._accel_2nd_order(self._vx, self._ax, vx_cmd, self._wn_lin, self._zeta_lin)
        ay_des = self._accel_2nd_order(self._vy, self._ay, vy_cmd, self._wn_lin, self._zeta_lin)
        az_des = self._accel_2nd_order(self._vz, self._az, vz_cmd, self._wn_lin, self._zeta_lin)

        # Aerodynamic drag (opposes velocity); 1/m folded into k1/k2
        ax_des += -(self._drag_k1 * self._vx + self._drag_k2 * abs(self._vx) * self._vx)
        ay_des += -(self._drag_k1 * self._vy + self._drag_k2 * abs(self._vy) * self._vy)
        az_des += -(self._drag_k1 * self._vz + self._drag_k2 * abs(self._vz) * self._vz)

        # Add OU wind accelerations
        ax_des += self._wind_ax; ay_des += self._wind_ay; az_des += self._wind_az

        # Jerk (slew) limits
        ax_des = self._apply_jerk_limit(self._ax, ax_des, self._jmax_lin, dt)
        ay_des = self._apply_jerk_limit(self._ay, ay_des, self._jmax_lin, dt)
        az_des = self._apply_jerk_limit(self._az, az_des, self._jmax_lin, dt)

        # Physical caps (tilt / thrust)
        ax_des, ay_des, az_des = self._limit_accel_with_physics(ax_des, ay_des, az_des)

        # Integrate accel->vel
        self._ax = ax_des; self._ay = ay_des; self._az = az_des
        self._vx += self._ax * dt; self._vy += self._ay * dt; self._vz += self._az * dt

        # --- Velocity clamps ---
        # Horizontal: clamp both to "policy" cap and to an absolute ceiling.
        vxy = self._vec_mag(self._vx, self._vy)
        vxy_cap = min(self._v_horiz_max, self._v_horiz_abs_max)
        if vxy > vxy_cap:
            s = vxy_cap / vxy
            self._vx *= s; self._vy *= s

        # Vertical: DJI ascent/descent rate limits
        if self._vz > self._vz_up_max:
            self._vz = self._vz_up_max
        elif self._vz < -self._vz_down_max:
            self._vz = -self._vz_down_max

        # ---- yaw dynamics ----
        # wz'' = wn^2 (wz_cmd - wz) - 2*zeta*wn * wz'
        awz_des = (self._wn_yaw * self._wn_yaw) * (wz_cmd - self._wz) - (2.0 * self._zeta_yaw * self._wn_yaw) * self._awz
        awz_des = self._apply_jerk_limit(self._awz, awz_des, self._jmax_yaw, dt)
        awz_des = self._clamp(awz_des, -self._awz_max, +self._awz_max)

        self._awz = awz_des
        self._wz += self._awz * dt
        self._wz = self._clamp(self._wz, -self._wz_max, +self._wz_max)

        return self._vx, self._vy, self._vz, self._wz

    def reset(self) -> None:
        self._cmd_buf.clear()
        self._vx = self._vy = self._vz = 0.0
        self._ax = self._ay = self._az = 0.0
        self._wz = self._awz = 0.0
        self._vx_cmd = self._vy_cmd = self._vz_cmd = self._wz_cmd = 0.0
        self._wind_ax = self._wind_ay = self._wind_az = 0.0


# ----------------------------- Notes & Tuning -----------------------------
# DJI FlyCart 30 key limits used here:
# - Weight: ~65 kg with two DB2000 batteries; MTOW up to ~95 kg (with cargo).
# - Max pitch/tilt: ~30°  -> lateral accel cap a_xy_max = g * tan(30°) ≈ 5.66 m/s².
# - Max horizontal speed: 15 m/s typical; 20 m/s max (not enabled by default).
# - Max ascent speed 5 m/s; max descent speed 3 m/s.
# - Max wind resistance ~12 m/s; "wind_level"=1.0 is roughly that scenario.
#
# Validate these against your sim world timestep and logging; if your loop dt is
# coarser than ~20 ms, consider raising damping (zeta) slightly to avoid overshoot.
#
# Drag:
#   Start with k1 ~ 0.15–0.3 1/s, k2 ~ 0.03–0.06 1/m. Fit from logs by regressing:
#     a_residual = dv/dt - model_accel ≈ -(k1 v + k2 |v| v)
#   Hattenberger et al. 2023 provides methodology for estimating aerodynamic drag.
#
# Wind:
#   OU parameters (tau, sigma) approximate gust autocorrelation and strength.
#   Increase wind_level toward 1.0 to emulate stronger buffeting (near 12 m/s).
#   See AIAA 2024 / Energies 2021 for OU-based stochastic wind modeling.
