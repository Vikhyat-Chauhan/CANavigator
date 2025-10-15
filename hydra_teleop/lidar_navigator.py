#!/usr/bin/env python3
import math, threading, time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from .teleop import GzTeleop
from .logger import Logger
from .config import TeleopConfig


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t0, t1)

def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# -------- FAST & RISKY DEFAULTS (favor time over safety) --------
@dataclass
class GoToConfig:
    goal_radius_m: float = 4.0      # reach earlier
    kp_lin: float = 1.2             # snappier forward response
    kp_z: float = 1.0
    kp_yaw: float = 2.0
    max_v: float = 15.0             # much higher cruise speed
    max_vz: float = 3.5
    max_wz: float = 1.4
    slow_yaw_threshold: float = 1.0 # rad; only slow if badly misaligned
    rate_hz: float = 30.0           # decent loop rate
    # extras for edge guarding (still aggressive)
    edge_guard_m: float = 3.5       # if side is closer than this, cap v a bit
    edge_guard_scale: float = 0.6   # scale for v when edge guarding activates

@dataclass
class AvoidCfg:
    scan_topic: str = "/model/drone1/front_lidar/scan"
    safe_m: float = 5.0              # start avoiding later (riskier)
    hysteresis_m: float = 1.0        # exit avoidance sooner (riskier)
    front_deg: float = 5.0           # narrow frontal window (may miss edges)
    side_deg: float = 30.0           # smaller side windows (riskier)
    side_center_deg: float = 30.0
    turn_rate: float = 0.9           # brisk turn when avoiding (rad/s cap)
    watchdog_sec: float = 0.6        # tolerate scan gaps longer (riskier)
    min_turn_sec: float = 0.7        # shorter commit time (can weave faster)
    hard_stale_sec: float = 1.8      # if scans stale beyond this, hard brake


class _PoseSub(Node):
    def __init__(self, topic: str, node_name: str):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._latest: Optional[PoseStamped] = None
        self.create_subscription(PoseStamped, topic, self._cb, 10)
    def _cb(self, msg: PoseStamped):
        with self._lock:
            self._latest = msg
    def latest(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._latest

class _ScanSub(Node):
    def __init__(self, topic: str):
        super().__init__("hydra_nav_lidar")
        self._lock = threading.Lock()
        self._scan: Optional[LaserScan] = None
        self._t_last = 0.0
        self.create_subscription(LaserScan, topic, self._cb, 10)
        self._printed = False
    def _cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg
            self._t_last = time.time()
        if not self._printed:
            # self.get_logger().info(f"scan: n={len(msg.ranges)} amin={msg.angle_min:.3f} amax={msg.angle_max:.3f} inc={msg.angle_increment:.6f}")
            self._printed = True
    def latest(self) -> Tuple[Optional[LaserScan], float]:
        with self._lock:
            return self._scan, self._t_last

    @staticmethod
    def _sector_min(msg: LaserScan, center_deg: float, half_width_deg: float) -> float:
        if msg is None or not msg.ranges:
            return float('inf')
        n = len(msg.ranges)
        inc = msg.angle_increment
        if not math.isfinite(inc) or abs(inc) < 1e-9:
            # fallback: assume forward at center
            center_idx = n // 2
            half = max(1, int(half_width_deg / 90.0 * n))
            lo = max(0, center_idx - half)
            hi = min(n - 1, center_idx + half)
        else:
            center = math.radians(center_deg)
            half = math.radians(half_width_deg)
            lo = int((center - half - msg.angle_min) / inc)
            hi = int((center + half - msg.angle_min) / inc)
            lo = max(0, min(n - 1, lo))
            hi = max(0, min(n - 1, hi))
            if lo > hi:
                lo, hi = hi, lo
        window = [r for r in msg.ranges[lo:hi+1] if math.isfinite(r) and r > 0.0]
        return min(window) if window else float('inf')


class LidarTargetNavigator:
    """
    Fast go-to with lightweight avoidance tuned for SPEED:
      - High forward speed; minimal slow-down for yaw error (unless huge).
      - Avoidance triggers later and clears earlier (commit window).
      - Slight "edge guard" to reduce corner clipping without losing much time.

    Returns from go_to(...): (reached: bool, elapsed_time_s: float)
    """
    def __init__(self,
                 teleop: GzTeleop,
                 cfg: TeleopConfig,
                 drone_pose_topic: Optional[str] = None,
                 target_pose_topic: Optional[str] = "/model/target_sphere/pose/info",
                 goto_cfg: Optional[GoToConfig] = None,
                 avoid_cfg: Optional[AvoidCfg] = None,
                 logger: Optional[Logger] = None):
        self._teleop = teleop
        self._cfg = cfg
        self._gc = goto_cfg or GoToConfig()
        self._ac = avoid_cfg or AvoidCfg()
        self._logger = logger or Logger(cfg)

        entity = getattr(cfg, "entity_name", "drone1")
        drone_topic = drone_pose_topic or getattr(cfg, "ros_pose_topic", f"/model/{entity}/pose/info")

        self._node_drone  = _PoseSub(drone_topic, "lidar_hydra_nav_drone_pose")
        self._node_target = _PoseSub(target_pose_topic, "lidar_hydra_nav_target_pose")
        self._node_scan   = _ScanSub(self._ac.scan_topic)

        self._exec = SingleThreadedExecutor()
        for n in (self._node_drone, self._node_target, self._node_scan):
            self._exec.add_node(n)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # anti-oscillation state
        self._avoiding = False
        self._avoid_sign = 0          # +1=left, -1=right
        self._avoid_until = 0.0       # commit window end time

    def _spin(self):
        try:
            while rclpy.ok():
                self._exec.spin_once(timeout_sec=0.05)
        except Exception:
            pass

    def shutdown(self):
        try:
            for n in (self._node_drone, self._node_target, self._node_scan):
                self._exec.remove_node(n)
                n.destroy_node()
            self._exec.shutdown()
        except Exception:
            pass

    def _latest_drone(self) -> Optional[Tuple[float, float, float, float]]:
        msg = self._node_drone.latest()
        if msg is None:
            return None
        p, o = msg.pose.position, msg.pose.orientation
        return (p.x, p.y, p.z, _yaw_from_quat(o.x, o.y, o.z, o.w))

    def _latest_target(self) -> Optional[Tuple[float, float, float]]:
        msg = self._node_target.latest()
        if msg is None:
            return None
        p = msg.pose.position
        return (p.x, p.y, p.z)

    def _scan_metrics(self) -> Tuple[float, float, float, bool, float]:
        scan, t_last = self._node_scan.latest()
        now = time.time()
        stale = (now - t_last) > self._ac.watchdog_sec
        if scan is None:
            return float('inf'), float('inf'), float('inf'), True, now
        front = _ScanSub._sector_min(scan, 0.0, self._ac.front_deg)
        left  = _ScanSub._sector_min(scan, +self._ac.side_center_deg, self._ac.side_deg)
        right = _ScanSub._sector_min(scan, -self._ac.side_center_deg, self._ac.side_deg)
        return front, left, right, stale, now

    def go_to(self,
              target_xyz: Optional[Tuple[float, float, float]] = None,
              timeout_s: Optional[float] = None) -> Tuple[bool, float]:
        """
        Returns:
            reached (bool): True if goal_radius_m reached before timeout.
            elapsed_time_s (float): Total wall-clock time.
        """
        rate = max(1.0, float(self._gc.rate_hz))
        dt = 1.0 / rate
        t_start = time.time()
        reached = False

        while True:
            dpose = self._latest_drone()
            if dpose is None:
                # no pose yet; don't spam stop() (keeps last command)
                time.sleep(0.02)
                if timeout_s is not None and (time.time() - t_start) > timeout_s:
                    break
                continue

            if target_xyz is None:
                tpose = self._latest_target()
                if tpose is None:
                    self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                    if timeout_s is not None and (time.time() - t_start) > timeout_s:
                        break
                    time.sleep(0.02)
                    continue
                tx, ty, tz = tpose
            else:
                tx, ty, tz = target_xyz

            x, y, z, yaw = dpose
            ex, ey, ez = (tx - x), (ty - y), (tz - z)
            dist_xy = math.hypot(ex, ey)
            dist_3d = math.sqrt(ex*ex + ey*ey + ez*ez)

            if dist_3d <= self._gc.goal_radius_m:
                reached = True
                break

            # ---- base go-to (aggressive) ----
            hdg_des = math.atan2(ey, ex)
            yaw_err = _wrap_pi(hdg_des - yaw)

            v_feed = self._gc.kp_lin * dist_xy
            v_cmd = min(self._gc.max_v, v_feed)
            vz_cmd = max(-self._gc.max_vz, min(self._gc.max_vz, self._gc.kp_z * ez))
            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_err))

            # If badly misaligned, cap forward speed a bit (still fast)
            if abs(yaw_err) > self._gc.slow_yaw_threshold:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

            # ---- avoidance overlay ----
            front, left, right, stale, now = self._scan_metrics()

            # Hard-stale safety stop (sensor died)
            if (now - (now if stale else now)) > self._ac.hard_stale_sec:  # always False; keep simple:
                pass
            # Instead, compute true hard-stale vs last scan time:
            scan, t_last = self._node_scan.latest()
            if (time.time() - t_last) > self._ac.hard_stale_sec:
                self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                time.sleep(dt)
                if timeout_s is not None and (time.time() - t_start) > timeout_s:
                    break
                continue

            # Edge guard: if walls close on either side, trim forward speed slightly
            if min(left, right) < self._gc.edge_guard_m:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

            if not stale:
                if self._avoiding:
                    if now < self._avoid_until or front < (self._ac.safe_m + self._ac.hysteresis_m):
                        # keep turning until commit window passes and clearance improves
                        v_cmd = 0.0
                        wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
                    else:
                        self._avoiding = False
                else:
                    if front < self._ac.safe_m:
                        self._avoiding = True
                        self._avoid_sign = +1 if left > right else -1
                        self._avoid_until = now + self._ac.min_turn_sec
                        v_cmd = 0.0
                        wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
            # if stale but not hard-stale, we keep current plan (risk-tolerant)

            # ---- send ----
            self._teleop.set_cmd(v_cmd, 0.0, vz_cmd, wz_cmd)

            if timeout_s is not None and (time.time() - t_start) > timeout_s:
                break

            time.sleep(dt)

        # stop/hold (short settle so momentum shim can decay)
        self._teleop.stop()
        time.sleep(max(0.02, 2.0 / max(1.0, float(getattr(self._cfg, "rate_hz", 30.0)))))
        elapsed = time.time() - t_start
        return reached, elapsed
