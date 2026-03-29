#!/usr/bin/env python3
# hydra_teleop/energy_monitor.py
# Paper-only energy model: E = EPM[J/m] * horizontal_distance[m]
# Quiet during flight; emits a single ENERGYSUMMARY on log_and_reset()

import math, time, logging
from typing import Optional, Tuple
from threading import Lock
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

DEFAULT_POSE_TOPIC = "/model/drone1/pose/info"

# Corrected EPM presets (Transportation Research Part D corrigendum)
EPM_PRESETS = {
    "kirschstein_baseline": 208.9,        # large octocopter, baseline
    "kirschstein_avionics": 213.9,        # + avionics overhead
    "kirschstein_wind45": 489.2,          # strong 45 km/h wind
    "kirschstein_profile10wind": 255.7,   # full profile + 10 km/h wind + avionics
    # Alias for DGI FlyKart 30: treat as large octocopter baseline unless overridden
    "flykart30": 208.9,
}

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
    Paper-only energy estimator (pose-only):
      E += EPM[J/m] * d(horizontal distance)
    No aerodynamic/hover/climb terms. Deterministic with smoothing & debounce.
    """

    def __init__(
        self,
        pose_topic: str = DEFAULT_POSE_TOPIC,
        *,
        # Numerics / smoothing
        min_step_m: float = 0.01,
        v_tau_s: float = 0.25,
        vz_tau_s: float = 0.25,

        # Paper model
        epm_preset: Optional[str] = "flykart30",
        epm_j_per_m: Optional[float] = None,
    ):
        super().__init__("energy_monitor")
        self._log = logging.getLogger(__name__)
        self._lock = Lock()

        self._min_step = max(0.0, float(min_step_m))
        self._v_tau = max(1e-3, float(v_tau_s))
        self._vz_tau = max(1e-3, float(vz_tau_s))

        # Choose EPM
        preset = (epm_preset or "").lower().strip()
        if preset and preset not in EPM_PRESETS:
            preset = "kirschstein_baseline"
        self.Epm = float(epm_j_per_m) if epm_j_per_m is not None else EPM_PRESETS.get(preset or "", 208.9)
        self._epm_preset = preset or None

        # Runtime state
        self._prev_t: Optional[float] = None
        self._prev_xyz: Optional[Tuple[float, float, float]] = None

        self._v_smooth = 0.0   # horizontal speed [m/s] (for diagnostics only)
        self._vz_smooth = 0.0  # vertical speed [m/s] (for diagnostics only)

        self._energy_j: float = 0.0
        self._elapsed_s: float = 0.0
        self._segment_start_t_wall: float = 0.0
        self._segment_label: str = "run"

        self._last_pos = (0.0, 0.0, 0.0)
        self._last_speed = 0.0
        self._last_climb = 0.0
        self._last_power = 0.0   # equivalent instantaneous power = dE/dt

        # Sub
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 50)

        # One-time dump
        self._log.info(
            {
                "epm_j_per_m": round(self.Epm, 1),
                "epm_preset": self._epm_preset,
                "mode": "paper_only_epm(log on demand)",
            },
            extra={"type": "LOADEDPARAMS"},
        )

    def _on_pose(self, msg: PoseStamped):
        t = _now_from_msg_or_clock(self, msg)
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

        with self._lock:
            if self._prev_xyz is None:
                self._prev_xyz = (x, y, z)
                self._prev_t = t
                self._last_pos = (x, y, z)
                return

            dt = max(1e-3, t - (self._prev_t or t))
            dx, dy, dz = x - self._prev_xyz[0], y - self._prev_xyz[1], z - self._prev_xyz[2]
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            step_xy = math.hypot(dx, dy)

            # Debounce tiny pose jitter
            if step < self._min_step:
                vx, vy, vz = 0.0, 0.0, 0.0
                dE = 0.0
            else:
                vx, vy, vz = dx / dt, dy / dt, dz / dt
                # EPM-only increment (paper model)
                dE = self.Epm * max(0.0, step_xy)

            # Equivalent instantaneous power for diagnostics
            P_equiv = dE / dt if dt > 0 else 0.0

            # Exponential smoothing (diagnostics only)
            alpha_v = 1.0 - math.exp(-dt / self._v_tau)
            alpha_vz = 1.0 - math.exp(-dt / self._vz_tau)
            v_xy = math.hypot(vx, vy)
            self._v_smooth = (1 - alpha_v) * self._v_smooth + alpha_v * v_xy
            self._vz_smooth = (1 - alpha_vz) * self._vz_smooth + alpha_vz * vz

            # Integrate
            self._energy_j += dE
            self._elapsed_s += dt

            # Snapshot
            self._last_pos = (x, y, z)
            self._last_speed = self._v_smooth
            self._last_climb = self._vz_smooth
            self._last_power = P_equiv

            self._prev_xyz = (x, y, z)
            self._prev_t = t

    def mark_run_start(self, label: str = "run"):
        with self._lock:
            self._segment_label = label
            self._energy_j = 0.0
            self._elapsed_s = 0.0
            self._segment_start_t_wall = self.get_clock().now().nanoseconds * 1e-9
            # Reset pose tracking so a drone teleport between strategies doesn't
            # produce a false energy spike on the first pose message of the new run.
            self._prev_xyz = None
            self._prev_t = None

    def log_and_reset(self, label: Optional[str] = None):
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
                    "pos": {"x": round(self._last_pos[0], 3), "y": round(self._last_pos[1], 3), "z": round(self._last_pos[2], 3)},
                    "speed_mps": round(self._last_speed, 3),
                    "climb_mps": round(self._last_climb, 3),
                    "inst_power_w": round(self._last_power, 1),
                },
                "model": {
                    "name": "epm_only",
                    "epm_j_per_m": round(self.Epm, 1),
                    "epm_preset": self._epm_preset,
                },
            }
            self._log.info(summary, extra={"type": "ENERGYSUMMARY"})

            self._energy_j = 0.0
            self._elapsed_s = 0.0
            self._segment_start_t_wall = self.get_clock().now().nanoseconds * 1e-9
            self._segment_label = "run"
            return summary

def start_energy_monitor(
    pose_topic: str = DEFAULT_POSE_TOPIC,
    min_step_m: float = 0.01,
    v_tau_s: float = 0.25,
    vz_tau_s: float = 0.25,
    epm_preset: Optional[str] = "flykart30",
    epm_j_per_m: Optional[float] = None,
    callback_group=None,
):
    node = EnergyMonitor(
        pose_topic=pose_topic,
        min_step_m=min_step_m, v_tau_s=v_tau_s, vz_tau_s=vz_tau_s,
        epm_preset=epm_preset, epm_j_per_m=epm_j_per_m,
    )
    if callback_group is not None:
        try:
            node._sub.callback_group = callback_group  # type: ignore[attr-defined]
        except Exception:
            pass
    return node, None

def add_energy_monitor_to_executor(executor, **kwargs):
    node, _ = start_energy_monitor(**kwargs)
    executor.add_node(node)
    return node
