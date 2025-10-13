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


# Default
@dataclass
class GoToConfig:
    goal_radius_m: float = 4.0
    kp_lin: float = 0.8
    kp_z: float = 0.8
    kp_yaw: float = 1.5
    max_v: float = 20.0
    max_vz: float = 2.0
    max_wz: float = 0.8
    slow_yaw_threshold: float = 0.8
    rate_hz: float = 30.0

@dataclass
class AvoidCfg:
    scan_topic: str = "/model/drone1/front_lidar/scan"
    safe_m: float = 5.0            # start avoiding if front < safe_m
    hysteresis_m: float = 1.0      # extra clearance to exit avoidance
    front_deg: float = 5.0         # front window ±deg
    side_deg: float = 30.0         # side window half-width
    side_center_deg: float = 30.0  # side windows centered at ±this deg
    turn_rate: float = 0.9         # rad/s while avoiding (capped by max_wz)
    watchdog_sec: float = 0.6      # stop if scans stale
    min_turn_sec: float = 0.7      # commit to chosen side at least this long

'''
# Fastest
class GoToConfig:
    goal_radius_m: float = 4.0
    kp_lin: float = 1.2     # was 0.8  → snappier forward response
    kp_z: float = 1.0       # was 0.8
    kp_yaw: float = 2.0     # was 1.5
    max_v: float = 30.0     # was 20.0 → higher top speed
    max_vz: float = 3.5     # was 2.0
    max_wz: float = 1.2     # was 0.8  → faster yawing
    slow_yaw_threshold: float = 1.2  # was 0.8; keep speed even with larger yaw error
    rate_hz: float = 60.0   # was 30.0 → more responsive control loop

@dataclass
class AvoidCfg:
    scan_topic: str = "/model/drone1/front_lidar/scan"
    safe_m: float = 3.0           # was 5.0 → start avoiding later
    hysteresis_m: float = 0.8     # was 1.0
    front_deg: float = 5.0
    side_deg: float = 30.0
    side_center_deg: float = 30.0
    turn_rate: float = 1.2        # was 0.9 → quicker avoidance turns
    watchdog_sec: float = 0.6
    min_turn_sec: float = 0.5     # was 0.7 → commit less time per turn
'''

'''
# Most Accurate
@dataclass
class GoToConfig:
    goal_radius_m: float = 6.0   # stop earlier near goal
    kp_lin: float = 0.6          # gentler forward response
    kp_z: float = 0.7
    kp_yaw: float = 1.2          # smoother heading control
    max_v: float = 12.0          # slower top speed (was 20–30)
    max_vz: float = 1.5
    max_wz: float = 1.0          # adequate turn rate to steer away
    slow_yaw_threshold: float = 0.6  # slow down for smaller yaw errors
    rate_hz: float = 60.0        # faster loop = safer reactions

@dataclass
class AvoidCfg:
    scan_topic: str = "/model/drone1/front_lidar/scan"
    safe_m: float = 7.0          # start avoiding earlier (was ~5)
    hysteresis_m: float = 2.0    # require more clearance before resuming
    front_deg: float = 10.0      # widen frontal check
    side_deg: float = 45.0       # larger side windows
    side_center_deg: float = 35.0
    turn_rate: float = 1.0       # steady turn away from obstacles
    watchdog_sec: float = 0.4    # stop sooner if scans go stale
    min_turn_sec: float = 1.0    # commit longer to the chosen turn
'''

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
    Simple target go-to with dead-simple avoidance:
    - Go toward target
    - If blocked: stop vx and turn toward more open side
    - Commit to that turn for min_turn_sec; exit avoidance only after clearance > safe + hysteresis

    go_to(...) now returns (reached: bool, elapsed_time_s: float)
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

        # minimal state for anti-oscillation
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

    def _scan_metrics(self) -> Tuple[float, float, float, bool]:
        scan, t_last = self._node_scan.latest()
        stale = (time.time() - t_last) > self._ac.watchdog_sec
        if scan is None:
            return float('inf'), float('inf'), float('inf'), True
        front = _ScanSub._sector_min(scan, 0.0, self._ac.front_deg)
        left  = _ScanSub._sector_min(scan, +self._ac.side_center_deg, self._ac.side_deg)
        right = _ScanSub._sector_min(scan, -self._ac.side_center_deg, self._ac.side_deg)
        return front, left, right, stale

    def go_to(self,
              target_xyz: Optional[Tuple[float, float, float]] = None,
              timeout_s: Optional[float] = None) -> Tuple[bool, float]:
        """
        Returns:
            reached (bool): True if goal_radius_m reached before timeout.
            elapsed_time_s (float): Total wall-clock time spent in the loop.
        """
        rate = max(1.0, self._gc.rate_hz)
        dt = 1.0 / rate
        t_start = time.time()
        reached = False

        while True:
            dpose = self._latest_drone()
            if dpose is None:
                time.sleep(0.05); 
                if timeout_s is not None and (time.time() - t_start) > timeout_s: break
                continue

            if target_xyz is None:
                tpose = self._latest_target()
                if tpose is None:
                    self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                    if timeout_s is not None and (time.time() - t_start) > timeout_s: break
                    time.sleep(0.05); 
                    continue
                tx, ty, tz = tpose
            else:
                tx, ty, tz = target_xyz

            x, y, z, yaw = dpose
            ex, ey, ez = (tx - x), (ty - y), (tz - z)
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)
            if dist <= self._gc.goal_radius_m:
                reached = True
                break

            # ---- base go-to (no avoidance) ----
            hdg_des = math.atan2(ey, ex)
            yaw_err = _wrap_pi(hdg_des - yaw)

            v_cmd  = min(self._gc.max_v, self._gc.kp_lin * math.hypot(ex, ey))
            if abs(yaw_err) > self._gc.slow_yaw_threshold:
                v_cmd *= 0.25
            vz_cmd = max(-self._gc.max_vz, min(self._gc.max_vz, self._gc.kp_z * ez))
            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_err))

            # ---- avoidance overlay (tiny + robust) ----
            front, left, right, stale = self._scan_metrics()
            now = time.time()

            if stale:
                # blind → stop safely
                v_cmd = 0.0
                wz_cmd = 0.0
                self._avoiding = False
            else:
                if self._avoiding:
                    # stay in avoidance until both time & clearance satisfied
                    if now < self._avoid_until or front < (self._ac.safe_m + self._ac.hysteresis_m):
                        v_cmd = 0.0
                        wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
                    else:
                        self._avoiding = False  # done avoiding
                else:
                    # not avoiding: check for block
                    if front < self._ac.safe_m:
                        self._avoiding = True
                        # pick side once, commit for min_turn_sec
                        self._avoid_sign = +1 if left > right else -1
                        self._avoid_until = now + self._ac.min_turn_sec
                        v_cmd = 0.0
                        wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)

            # ---- send ----
            self._teleop.set_cmd(v_cmd, 0.0, vz_cmd, wz_cmd)

            if timeout_s is not None and (time.time() - t_start) > timeout_s:
                break

            time.sleep(dt)

        # stop/hold
        self._teleop.stop()
        time.sleep(max(0.05, 2.0 / self._cfg.rate_hz))
        elapsed = time.time() - t_start
        return reached, elapsed
