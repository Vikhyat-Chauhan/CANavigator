#!/usr/bin/env python3
# hydra_teleop/energy_monitor.py
# Deterministic, pose-only multicopter energy monitor with realistic power model.
#
# Terms:
#   P_total = (P_induced + P_profile + P_parasite + P_climb) / eta
# where:
#   - P_induced  ~ k_induced * T^(3/2) / sqrt(2 rho A)
#   - P_profile  ~ k_profile * P_induced_hover   (constant fraction of hover induced aero)
#   - P_parasite = 0.5 * rho * CdA * V^3         (fuselage drag power)
#   - P_climb    = max(0, m g v_z)               (no regen in descent)
#
# Notes:
#   * Deterministic: pose-only, no acceleration or stochastic terms.
#   * Mild exponential smoothing for velocity/climb to avoid log spikes.
#   * Log summary only when log_and_reset() is called (quiet during flight).

import math, time, logging
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from threading import Lock

DEFAULT_POSE_TOPIC = "/model/drone1/pose/info"

def _now_from_msg_or_clock(node: Node, msg: Optional[PoseStamped]) -> float:
    try:
        if msg is not None and msg.header.stamp:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except Exception:
        pass
    try:
        return node.get_clock().now().nanoseconds * 1e-9
    except Exception:
        return time.time()

class EnergyMonitor(Node):
    """
    Realistic, deterministic power & energy estimate for a multicopter (pose-only).
    Quiet by default; emits a single ENERGYSUMMARY when log_and_reset() is called.
    """

    def __init__(
        self,
        pose_topic: str = DEFAULT_POSE_TOPIC,
        *,
        # --- Physical configuration (tune to your airframe) ---
        mass_kg: float = 65.0,           # vehicle + payload
        n_rotors: int = 4,
        rotor_diameter_m: float = 1.375, # per-rotor diameter [m]
        rho_air: float = 1.225,          # air density [kg/m^3]
        eta_elec: float = 0.7,           # electrical -> aero efficiency (0<eta<=1)
        cdA_m2: float = 0.60,            # effective fuselage Cd*A [m^2]
        k_induced: float = 1.15,         # induced power loss factor (>=1)
        k_profile: float = 0.12,         # profile power fraction of hover induced aero (0.08–0.2 typical)

        # --- Numerics / smoothing ---
        min_step_m: float = 0.01,        # motion debounce on pose updates
        v_tau_s: float = 0.25,           # exp smoothing time constant for horizontal speed
        vz_tau_s: float = 0.25,          # exp smoothing time constant for climb rate
        log_every_s: float = 0.25,       # kept for API compatibility; not used in quiet mode
    ):
        super().__init__("energy_monitor")
        self._log = logging.getLogger(__name__)
        self._lock = Lock()

        # Store params
        self.m = float(mass_kg)
        self.n = int(n_rotors)
        self.D = float(rotor_diameter_m)
        self.rho = float(rho_air)
        self.eta = max(1e-3, float(eta_elec))
        self.cdA = max(0.0, float(cdA_m2))
        self.k_i = max(1.0, float(k_induced))
        self.k_prof = max(0.0, float(k_profile))

        self._min_step = max(0.0, float(min_step_m))
        self._v_tau = max(1e-3, float(v_tau_s))
        self._vz_tau = max(1e-3, float(vz_tau_s))

        # Derived geometry
        self.g = 9.80665
        R = 0.5 * self.D
        self.A_disk_total = self.n * math.pi * R * R  # total rotor disk area [m^2]
        self.T_hover = self.m * self.g                # hover thrust [N]

        # Aero powers at hover (reference)
        self.P_induced_hover_aero = (self.T_hover ** 1.5) / math.sqrt(2.0 * self.rho * self.A_disk_total)
        self.P_profile_aero = self.k_prof * self.P_induced_hover_aero

        # Runtime state
        self._prev_t: Optional[float] = None
        self._prev_xyz: Optional[Tuple[float, float, float]] = None

        # Smoothed velocities
        self._v_smooth = 0.0
        self._vz_smooth = 0.0

        # Accumulators
        self._energy_j: float = 0.0
        self._elapsed_s: float = 0.0
        self._segment_start_t_wall: float = time.time()
        self._segment_label: str = "run"

        # Last snapshot for summary
        self._last_pos = (0.0, 0.0, 0.0)
        self._last_speed = 0.0
        self._last_climb = 0.0
        self._last_power = 0.0

        # Subscribe
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 50)

        # One-time param dump
        self._log.info(
            {
                "mass_kg": round(self.m, 3),
                "n_rotors": self.n,
                "rotor_diameter_m": round(self.D, 3),
                "rho_air": round(self.rho, 3),
                "eta_elec": round(self.eta, 3),
                "cdA_m2": round(self.cdA, 3),
                "k_induced": round(self.k_i, 3),
                "k_profile": round(self.k_prof, 3),
                "A_disk_total_m2": round(self.A_disk_total, 4),
                "P_induced_hover_aero_W": round(self.P_induced_hover_aero, 1),
                "P_profile_aero_W": round(self.P_profile_aero, 1),
                "mode": "quiet(log on demand)"
            },
            extra={"type": "LOADEDPARAMS"},
        )

    # ---------- Core power model (deterministic) ----------
    def _p_induced_aero(self) -> float:
        # Momentum theory at hover (constant T≈mg); use k_induced for losses.
        return self.k_i * self.P_induced_hover_aero

    def _p_profile_aero(self) -> float:
        # Constant fraction of hover induced aero (tunable).
        return self.P_profile_aero

    def _p_parasite_aero(self, v: float) -> float:
        # Fuselage drag power: 0.5 * rho * CdA * V^3
        return 0.5 * self.rho * self.cdA * (v ** 3)

    def _p_climb_aero(self, vz: float) -> float:
        # Work against gravity; no regen on descent.
        return max(0.0, self.m * self.g * vz)

    # ---------- Pose callback ----------
    def _on_pose(self, msg: PoseStamped):
        t = _now_from_msg_or_clock(self, msg)
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        with self._lock:
            if self._prev_xyz is None:
                self._prev_xyz = (x, y, z)
                self._prev_t = t
                self._last_pos = (x, y, z)
                return

            dt = max(1e-3, t - (self._prev_t or t))
            dx = x - self._prev_xyz[0]
            dy = y - self._prev_xyz[1]
            dz = z - self._prev_xyz[2]
            step = math.sqrt(dx*dx + dy*dy + dz*dz)

            # Debounce tiny pose jitter
            if step < self._min_step:
                vx, vy, vz = 0.0, 0.0, 0.0
            else:
                vx, vy, vz = dx / dt, dy / dt, dz / dt

            v_xy = math.hypot(vx, vy)

            # Exponential smoothing (deterministic given dt)
            alpha_v = 1.0 - math.exp(-dt / self._v_tau)
            alpha_vz = 1.0 - math.exp(-dt / self._vz_tau)
            self._v_smooth = (1 - alpha_v) * self._v_smooth + alpha_v * v_xy
            self._vz_smooth = (1 - alpha_vz) * self._vz_smooth + alpha_vz * vz

            # Aero powers
            P_induced = self._p_induced_aero()
            P_profile = self._p_profile_aero()
            P_parasite = self._p_parasite_aero(self._v_smooth)
            P_climb = self._p_climb_aero(self._vz_smooth)

            P_aero_total = P_induced + P_profile + P_parasite + P_climb
            P_elec = P_aero_total / self.eta

            # Integrate energy
            self._energy_j += max(0.0, P_elec) * dt
            self._elapsed_s += dt

            # Snapshot
            self._last_pos = (x, y, z)
            self._last_speed = self._v_smooth
            self._last_climb = self._vz_smooth
            self._last_power = P_elec

            # Update state
            self._prev_xyz = (x, y, z)
            self._prev_t = t

    # ---------- Public controls ----------
    def mark_run_start(self, label: str = "run"):
        with self._lock:
            self._segment_label = label
            self._energy_j = 0.0
            self._elapsed_s = 0.0
            self._segment_start_t_wall = time.time()

    def log_and_reset(self, label: Optional[str] = None, include_params: bool = False):
        with self._lock:
            lbl = label if label is not None else self._segment_label
            elapsed = max(0.0, self._elapsed_s)
            energy = max(0.0, self._energy_j)
            mean_power = (energy / elapsed) if elapsed > 1e-9 else 0.0

            summary = {
                "label": lbl,
                "wall_started_at": round(self._segment_start_t_wall, 3),
                "elapsed_s": round(elapsed, 3),
                "energy_j": round(energy, 3),
                "mean_power_w": round(mean_power, 1),
                "last": {
                    "pos": {
                        "x": round(self._last_pos[0], 3),
                        "y": round(self._last_pos[1], 3),
                        "z": round(self._last_pos[2], 3),
                    },
                    "speed_mps": round(self._last_speed, 3),
                    "climb_mps": round(self._last_climb, 3),
                    "inst_power_w": round(self._last_power, 1),
                },
            }
            if include_params:
                summary["params"] = {
                    "mass_kg": self.m,
                    "n_rotors": self.n,
                    "rotor_diameter_m": self.D,
                    "rho_air": self.rho,
                    "eta_elec": self.eta,
                    "cdA_m2": self.cdA,
                    "k_induced": self.k_i,
                    "k_profile": self.k_prof,
                    "A_disk_total_m2": round(self.A_disk_total, 4),
                    "P_induced_hover_aero_W": round(self.P_induced_hover_aero, 1),
                    "P_profile_aero_W": round(self.P_profile_aero, 1),
                }

            self._log.info(summary, extra={"type": "ENERGYSUMMARY"})

            # Reset segment
            self._energy_j = 0.0
            self._elapsed_s = 0.0
            self._segment_start_t_wall = time.time()
            self._segment_label = "run"

            return summary

# ---------- Helpers to construct/add to executor ----------
def start_energy_monitor(
    pose_topic: str = DEFAULT_POSE_TOPIC,
    mass_kg: float = 65.0,
    n_rotors: int = 4,
    rotor_diameter_m: float = 1.375,
    rho_air: float = 1.225,
    eta_elec: float = 0.7,
    cdA_m2: float = 0.60,
    k_induced: float = 1.15,
    k_profile: float = 0.12,
    min_step_m: float = 0.01,
    v_tau_s: float = 0.25,
    vz_tau_s: float = 0.25,
    log_every_s: float = 0.25,  # API compat; unused
    callback_group=None,
):
    node = EnergyMonitor(
        pose_topic=pose_topic,
        mass_kg=mass_kg,
        n_rotors=n_rotors,
        rotor_diameter_m=rotor_diameter_m,
        rho_air=rho_air,
        eta_elec=eta_elec,
        cdA_m2=cdA_m2,
        k_induced=k_induced,
        k_profile=k_profile,
        min_step_m=min_step_m,
        v_tau_s=v_tau_s,
        vz_tau_s=vz_tau_s,
        log_every_s=log_every_s,
    )
    if callback_group is not None:
        try:
            node._sub.callback_group = callback_group  # type: ignore[attr-defined]
        except Exception:
            pass
    return node, None

def add_energy_monitor_to_executor(
    executor,
    **kwargs,
):
    node, _ = start_energy_monitor(**kwargs)
    executor.add_node(node)
    return node
