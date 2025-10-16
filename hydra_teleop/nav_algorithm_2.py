#!/usr/bin/env python3
# APE2 (event-aware, APE2-only):
# - Normal loop: sweep-based heading, TTC braking, breadcrumbs anti-oscillation,
#   progress watchdog, jerk/accel caps, edge/corner guards.
# - Event handling (NO QUEUE):
#     * Only APE2 planner runs with navigation algorithm.
#     * If a new event arrives while one is active → mark violation on the old,
#       drop it, and accept the new one immediately.
#     * If deadline expires with no applied APE2 plan → mark violation (deadline_miss).
#
# Public API unchanged:
#   LidarTargetNavigatorAPE2.go_to(target_xyz=None, timeout_s=None) -> (reached: bool, elapsed_s: float)

import math, threading, time, json
from dataclasses import dataclass
from typing import Optional, Tuple, Set, List, Deque, Dict
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .teleop import GzTeleop
from .logger import Logger
from .config import TeleopConfig

# ---------- small math ----------
def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t0, t1)

def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

# ===== Navigzation =====
@dataclass
class GoToConfig:
    goal_radius_m: float = 4.0
    kp_lin: float = 1.2
    kp_z: float = 1.0
    kp_yaw: float = 2.0
    max_v: float = 12.0
    max_vz: float = 3.0
    max_wz: float = 1.2
    slow_yaw_threshold: float = 1.0
    rate_hz: float = 30.0
    # Edge guard (slow down when hugging walls)
    edge_guard_m: float = 3.0
    edge_guard_scale: float = 0.6

@dataclass
class AvoidCfg:
    scan_topic: str = "/model/drone1/front_lidar/scan"
    safe_m: float = 4.5
    hysteresis_m: float = 1.0
    front_deg: float = 6.0
    side_deg: float = 30.0
    side_center_deg: float = 30.0
    turn_rate: float = 0.9
    watchdog_sec: float = 0.6
    hard_stale_sec: float = 1.8
    min_turn_sec: float = 0.7

@dataclass
class BreadcrumbCfg:
    cell_xy_m: float = 2.0
    cell_z_m: float = 2.0
    capacity: int = 2500

@dataclass
class SafetyCfg:
    ambiguity_eps_m: float = 0.5        # tie threshold for left-vs-right
    ttc_soft_s: float = 2.0
    ttc_hard_s: float = 1.0
    v_min_frac: float = 0.18
    near_obs_m: float = 3.0
    cap_wz_near_obs: float = 0.8
    corner_deg: float = 30.0
    corner_inflate_m: float = 1.0
    progress_window_s: float = 3.0
    min_progress_m: float = 1.0
    escape_yaw_rad: float = 0.8
    escape_time_s: float = 0.8
    crumb_oscillations_to_flip: int = 12
    dv_max_mps_per_s: float = 7.0
    jw_max_radps2: float = 2.3
    clear_ahead_thresh_m: float = 10.0
    dv_clear_scale: float = 0.55
    yaw_align_rad: float = 0.25

@dataclass
class EventDecisionCfg:
    event_topic: str = "/hydra/event"
    # micro budget for the (only) event planner
    ape2_budget_ms: int = 25
    # temporary safety bump while handling event
    safe_inflate_m: float = 0.8
    v_cap_frac: float = 0.65

# ---- internal subscribers ----
class _PoseSub(Node):
    def __init__(self, topic: str, node_name: str):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._latest: Optional[PoseStamped] = None
        self.create_subscription(PoseStamped, topic, self._cb, 10)
    def _cb(self, msg: PoseStamped):
        with self._lock: self._latest = msg
    def latest(self) -> Optional[PoseStamped]:
        with self._lock: return self._latest

class _ScanSub(Node):
    def __init__(self, topic: str):
        super().__init__("hydra_nav_lidar")
        self._lock = threading.Lock()
        self._scan: Optional[LaserScan] = None
        self._t_last = 0.0
        self.create_subscription(LaserScan, topic, self._cb, 10)
    def _cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg
            self._t_last = time.time()
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
            lo = max(0, center_idx - half); hi = min(n - 1, center_idx + half)
        else:
            center = math.radians(center_deg); half = math.radians(half_width_deg)
            lo = int((center - half - msg.angle_min) / inc)
            hi = int((center + half - msg.angle_min) / inc)
            lo = max(0, min(n - 1, lo)); hi = max(0, min(n - 1, hi))
            if lo > hi: lo, hi = hi, lo
        window = [r for r in msg.ranges[lo:hi+1] if math.isfinite(r) and r > 0.0]
        return min(window) if window else float('inf')

    @staticmethod
    def _window_vals(msg: LaserScan, center_deg: float, half_width_deg: float) -> List[float]:
        if msg is None or not msg.ranges:
            return []
        n = len(msg.ranges)
        inc = msg.angle_increment
        if not math.isfinite(inc) or abs(inc) < 1e-9:
            center_idx = n // 2
            half = max(1, int(half_width_deg / 90.0 * n))
            lo = max(0, center_idx - half); hi = min(n - 1, center_idx + half)
        else:
            center = math.radians(center_deg); half = math.radians(half_width_deg)
            lo = int((center - half - msg.angle_min) / inc)
            hi = int((center + half - msg.angle_min) / inc)
            lo = max(0, min(n - 1, lo)); hi = max(0, min(n - 1, hi))
            if lo > hi: lo, hi = hi, lo
        return [r for r in msg.ranges[lo:hi+1] if math.isfinite(r) and r > 0.0]

class _EventSub(Node):
    """Subscribe to /hydra/event (std_msgs/String with JSON payload)."""
    def __init__(self, topic: str):
        super().__init__("hydra_event_sub")
        self._lock = threading.Lock()
        self._pending: Optional[Dict] = None
        self.create_subscription(String, topic, self._cb, 10)
    def _cb(self, msg: String):
        try:
            obj = json.loads(msg.data)
            obj["t_recv"] = time.time()
        except Exception:
            return
        with self._lock:
            self._pending = obj
    def pop(self) -> Optional[Dict]:
        with self._lock:
            v = self._pending
            self._pending = None
            return v

class LidarTargetNavigatorAPE2:
    """
    baseline navigation with event-aware behavior (APE2-only).
    - Normal loop: sweep-based heading, TTC braking, breadcrumbs anti-oscillation,
      progress watchdog, jerk/accel caps.
    - Event loop: run one tiny planner (APE2 sweep). No queue; if a new event
      arrives while one is active -> violation on old + replace. Deadline miss -> violation.
    """
    def __init__(self,
                 teleop: GzTeleop,
                 cfg: TeleopConfig,
                 drone_pose_topic: Optional[str] = None,
                 target_pose_topic: Optional[str] = "/model/target_sphere/pose/info",
                 goto_cfg: Optional[GoToConfig] = None,
                 avoid_cfg: Optional[AvoidCfg] = None,
                 crumb_cfg: Optional[BreadcrumbCfg] = None,
                 safety_cfg: Optional[SafetyCfg] = None,
                 logger: Optional[Logger] = None):
        self._teleop = teleop
        self._cfg = cfg
        self._gc = goto_cfg or GoToConfig()
        self._ac = avoid_cfg or AvoidCfg()
        self._bc = crumb_cfg or BreadcrumbCfg()
        self._sc = safety_cfg or SafetyCfg()
        self._logger = logger or Logger(cfg)

        entity = getattr(cfg, "entity_name", "drone1")
        drone_topic = drone_pose_topic or getattr(cfg, "ros_pose_topic", f"/model/{entity}/pose/info")

        self._node_drone  = _PoseSub(drone_topic, "lidar_hydra_nav_drone_pose")
        self._node_target = _PoseSub(target_pose_topic, "lidar_hydra_nav_target_pose")
        self._node_scan   = _ScanSub(self._ac.scan_topic)

        self._edc = EventDecisionCfg()
        self._node_evt = _EventSub(getattr(cfg, "event_topic", self._edc.event_topic))

        self._exec = SingleThreadedExecutor()
        for n in (self._node_drone, self._node_target, self._node_scan, self._node_evt):
            self._exec.add_node(n)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # Avoidance state
        self._avoiding = False
        self._avoid_sign = 0
        self._avoid_until = 0.0

        # Breadcrumbs
        self._crumb_set: Set[Tuple[int,int,int]] = set()
        self._crumb_fifo: Deque[Tuple[int,int,int]] = deque()
        self._side_bias: int = +1
        self._crumb_hits_recent: int = 0

        # Progress watchdog
        self._progress_t0: Optional[float] = None
        self._progress_d0: Optional[float] = None
        self._escape_until: float = 0.0

        # Command shapers
        self._v_cmd_prev = 0.0
        self._wz_cmd_prev = 0.0

        # Event state
        self._pending_evt: Optional[Dict] = None
        self._evt_deadline_at: float = 0.0
        self._evt_lock = threading.Lock()
        self._evt_proposals: Dict[str, Dict] = {}
        self._evt_threads: List[threading.Thread] = []
        self._evt_active: bool = False
        self._evt_resolved: bool = False

    # ---------- threading ----------
    def _spin(self):
        try:
            while rclpy.ok():
                self._exec.spin_once(timeout_sec=0.05)
        except Exception:
            pass

    def shutdown(self):
        try:
            for n in (self._node_drone, self._node_target, self._node_scan, self._node_evt):
                self._exec.remove_node(n)
                n.destroy_node()
            self._exec.shutdown()
        except Exception:
            pass

    # ---------- helpers ----------
    def _latest_drone(self):
        msg = self._node_drone.latest()
        if msg is None: return None
        p, o = msg.pose.position, msg.pose.orientation
        return (p.x, p.y, p.z, _yaw_from_quat(o.x, o.y, o.z, o.w))

    def _latest_target(self):
        msg = self._node_target.latest()
        if msg is None: return None
        p = msg.pose.position
        return (p.x, p.y, p.z)

    def _scan_metrics(self):
        scan, t_last = self._node_scan.latest()
        now = time.time()
        stale = (now - t_last) > self._ac.watchdog_sec
        if scan is None:
            return float('inf'), float('inf'), float('inf'), True, None, now
        front = _ScanSub._sector_min(scan, 0.0, self._ac.front_deg)
        left  = _ScanSub._sector_min(scan, +self._ac.side_center_deg, self._ac.side_deg)
        right = _ScanSub._sector_min(scan, -self._ac.side_center_deg, self._ac.side_deg)
        return front, left, right, stale, scan, now

    def _cell(self, x: float, y: float, z: float):
        return (int(round(x/self._bc.cell_xy_m)),
                int(round(y/self._bc.cell_xy_m)),
                int(round(z/self._bc.cell_z_m)))

    def _crumb_add(self, cell):
        if cell in self._crumb_set: return
        self._crumb_set.add(cell)
        self._crumb_fifo.append(cell)
        if len(self._crumb_fifo) > max(1, self._bc.capacity):
            old = self._crumb_fifo.popleft()
            self._crumb_set.discard(old)

    # ---------- event helpers ----------
    def _evt_violate(self, reason: str = "miss"):
        if self._pending_evt is None: return
        try:
            kind = self._pending_evt.get("kind", "?")
            dl   = float(self._pending_evt.get("deadline_s", 0.0))
            self._logger.warn(f"[EVENT VIOLATION] kind={kind} deadline_s={dl:.3f} reason={reason}")
        except Exception:
            pass

    def _evt_clear(self):
        with self._evt_lock:
            self._evt_proposals.clear()
        for t in self._evt_threads:
            if t.is_alive(): t.join(timeout=0.002)
        self._evt_threads.clear()
        self._pending_evt = None
        self._evt_deadline_at = 0.0
        self._evt_active = False
        self._evt_resolved = False

    # ---------- event planner (single, APE2) ----------
    def _evt_put(self, name, v, wz, vz, score):
        with self._evt_lock:
            self._evt_proposals[name] = {"v": v, "wz": wz, "vz": vz, "score": score, "ready_t": time.time()}

    def _evt_plan_ape2(self, snap, budget_ms):
        # Opening sweep (−60..+60), pick largest clearance, moderate speed
        t0 = time.time()
        scan = snap["scan"]
        if scan is None:
            return self._evt_put("APE2", snap["v_cmd"], snap["wz_cmd"], snap["vz_cmd"], score=1e3)
        best_deg, best_r = 0.0, -1.0
        for deg in range(-60, 61, 2):
            r = _ScanSub._sector_min(scan, deg, 2.0)
            if math.isfinite(r) and r > best_r:
                best_r, best_deg = r, deg
            if (time.time() - t0)*1000.0 > budget_ms:
                break
        yaw_goal = _wrap_pi(math.radians(best_deg))
        wz = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_goal))
        v = min(snap["v_cmd"], 0.45 * self._gc.max_v)
        # prefer more clearance and smaller |wz|
        score = - (best_r - 0.5*abs(wz))
        self._evt_put("APE2", v, wz, snap["vz_cmd"], score)

    # ---------- core ----------
    def go_to(self,
              target_xyz: Optional[Tuple[float, float, float]] = None,
              timeout_s: Optional[float] = None) -> Tuple[bool, float]:
        rate = max(1.0, float(self._gc.rate_hz))
        dt = 1.0 / rate
        t_start = time.time()
        reached = False

        while True:
            dpose = self._latest_drone()
            if dpose is None:
                time.sleep(0.05)
                if timeout_s is not None and (time.time() - t_start) > timeout_s: break
                continue

            if target_xyz is None:
                tpose = self._latest_target()
                if tpose is None:
                    self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                    if timeout_s is not None and (time.time() - t_start) > timeout_s: break
                    time.sleep(0.05)
                    continue
                tx, ty, tz = tpose
            else:
                tx, ty, tz = target_xyz

            x, y, z, yaw = dpose
            ex, ey, ez = (tx - x), (ty - y), (tz - z)
            dist_xy = math.hypot(ex, ey)
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)
            if dist <= self._gc.goal_radius_m:
                reached = True
                break

            # Base go-to
            hdg_des = math.atan2(ey, ex)
            yaw_err = _wrap_pi(hdg_des - yaw)

            v_cmd  = min(self._gc.max_v, self._gc.kp_lin * dist_xy)
            if abs(yaw_err) > self._gc.slow_yaw_threshold:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

            vz_cmd = max(-self._gc.max_vz, min(self._gc.max_vz, self._gc.kp_z * ez))
            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_err))

            # Scan & metrics
            front, left, right, stale, scan, now = self._scan_metrics()

            # Event intake: if active, violate & replace; else accept
            evt = self._node_evt.pop()
            if evt is not None:
                if self._evt_active:
                    self._evt_violate("preempted_by_new_event")
                    self._evt_clear()
                self._pending_evt = evt
                self._evt_deadline_at = evt["t_recv"] + max(0.0, float(evt.get("deadline_s", 0.0)))
                self._evt_active = True
                self._evt_resolved = False
                with self._evt_lock:
                    self._evt_proposals = {}
                for t in self._evt_threads:
                    if t.is_alive(): t.join(timeout=0.001)
                self._evt_threads = []
                snap = {
                    "x": x, "y": y, "z": z, "yaw": yaw,
                    "tx": tx, "ty": ty, "tz": tz,
                    "v_cmd": v_cmd, "wz_cmd": wz_cmd, "vz_cmd": vz_cmd,
                    "scan": scan,
                    "front": front, "left": left, "right": right,
                    "yaw_err": _wrap_pi(math.atan2(ey, ex) - yaw),
                }
                self._evt_threads = [
                    threading.Thread(target=self._evt_plan_ape2, args=(snap, self._edc.ape2_budget_ms), daemon=True),
                ]
                for t in self._evt_threads: t.start()

            def _evt_time_left():
                return max(0.0, self._evt_deadline_at - time.time())

            event_active = self._evt_active and (self._pending_evt is not None)

            # Hard-stale LiDAR → brake/hold
            _, t_last = self._node_scan.latest()
            if (time.time() - t_last) > self._ac.hard_stale_sec:
                self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                time.sleep(dt)
                if timeout_s is not None and (time.time() - t_start) > timeout_s: break
                continue

            # Breadcrumbs
            cell = self._cell(x, y, z)
            revisit = (cell in self._crumb_set)
            if revisit:
                self._crumb_hits_recent += 1
            else:
                self._crumb_hits_recent = max(0, self._crumb_hits_recent - 1)
            self._crumb_add(cell)

            # Effective safe distance (corner inflate)
            effective_safe_m = self._ac.safe_m
            if abs(math.degrees(yaw_err)) > self._sc.corner_deg:
                effective_safe_m += self._sc.corner_inflate_m

            # Edge guard slowdown near walls
            if min(left, right) < self._gc.edge_guard_m:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

            # Event window: temporary safety + use APE2 if ready by deadline
            if event_active:
                effective_safe_m += self._edc.safe_inflate_m
                v_cmd = min(v_cmd, self._edc.v_cap_frac * self._gc.max_v)

                tl = _evt_time_left()
                if tl <= 0.0:
                    if not self._evt_resolved:
                        self._evt_violate("deadline_miss")
                    self._evt_clear()
                else:
                    with self._evt_lock:
                        ready = dict(self._evt_proposals)
                    prop = ready.get("APE2", None)
                    if prop is not None:
                        v_cmd, wz_cmd, vz_cmd = prop["v"], prop["wz"], prop["vz"]
                        if not self._evt_resolved:
                            self._evt_resolved = True
                            try:
                                kind = self._pending_evt.get("kind", "?")
                                self._logger.info(f"[EVENT RESOLVED] kind={kind} by APE2 with {tl:.3f}s left")
                            except Exception:
                                pass

            # Avoidance state machine
            if stale:
                v_cmd = 0.0
                wz_cmd = 0.0
                self._avoiding = False
            else:
                if self._avoiding:
                    if now < self._avoid_until or front < (effective_safe_m + self._ac.hysteresis_m):
                        v_cmd = 0.0
                        wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
                    else:
                        self._avoiding = False
                else:
                    if front < effective_safe_m:
                        self._avoiding = True
                        diff = abs(left - right)
                        if diff < self._sc.ambiguity_eps_m:
                            self._avoid_sign = (+1 if self._side_bias > 0 else -1)
                        else:
                            self._avoid_sign = (+1 if left > right else -1)
                        self._avoid_until = now + self._ac.min_turn_sec
                        v_cmd = 0.0
                        wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
                    else:
                        # Normal sweep heading (−50..+50)
                        if scan is not None:
                            best_deg, best_r = 0.0, -1.0
                            for deg in range(-50, 51, 2):
                                r = _ScanSub._sector_min(scan, deg, 2.0)
                                if math.isfinite(r) and r > best_r:
                                    best_r, best_deg = r, deg
                            yaw_goal = _wrap_pi(math.radians(best_deg))
                            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_goal))
                        # Sticky-wall nudge if one side very close
                        side_min = min(left, right)
                        if side_min < 0.7 + 0.6:
                            if left < right:  wz_cmd -= 0.25
                            else:             wz_cmd += 0.25
                        # Flip side-bias if oscillating
                        if self._crumb_hits_recent >= self._sc.crumb_oscillations_to_flip:
                            self._side_bias *= -1
                            self._crumb_hits_recent = 0
                            try: self._logger.info("[APE2] Breadcrumb: flipped side-bias")
                            except Exception: pass

            # TTC + stopping distance
            dmin = float('inf')
            if scan is not None:
                window = _ScanSub._window_vals(scan, 0.0, max(5.0, self._ac.front_deg))
                dmin = min(window) if window else float('inf')
                if v_cmd > 0.05 and math.isfinite(dmin) and dmin > 0.0:
                    ttc = dmin / max(v_cmd, 1e-3)
                    if ttc < self._sc.ttc_soft_s:
                        num = (ttc - self._sc.ttc_hard_s)
                        den = max(self._sc.ttc_soft_s - self._sc.ttc_hard_s, 1e-3)
                        frac = max(self._sc.v_min_frac, min(1.0, num / den))
                        v_cmd = self._gc.max_v * frac
                    # simple stopping-distance cap: v^2/(2a) + margin <= dmin
                    a = 5.0  # decel m/s^2 for APE2
                    margin = 0.8
                    if math.isfinite(dmin) and dmin > margin:
                        vmax = math.sqrt(max(0.0, 2.0*a*(dmin - margin)))
                        v_cmd = min(v_cmd, vmax)

            # Yaw-rate cap near obstacles
            nearest = min(front, left, right)
            if nearest < self._sc.near_obs_m:
                wz_cmd = max(-self._sc.cap_wz_near_obs, min(self._sc.cap_wz_near_obs, wz_cmd))

            # Progress watchdog (escape rotate)
            if self._progress_t0 is None:
                self._progress_t0 = now
                self._progress_d0 = dist

            if now < self._escape_until:
                v_cmd = 0.0
                wz_cmd = (self._sc.escape_yaw_rad / self._sc.escape_time_s) * (1 if self._avoid_sign >= 0 else -1)
            elif (now - self._progress_t0) > self._sc.progress_window_s:
                gained = (self._progress_d0 - dist)
                if gained < self._sc.min_progress_m:
                    self._escape_until = now + self._sc.escape_time_s
                    v_cmd = 0.0
                    wz_cmd = (self._sc.escape_yaw_rad / self._sc.escape_time_s) * (1 if self._avoid_sign >= 0 else -1)
                self._progress_t0 = now
                self._progress_d0 = dist

            # Command ramp/jerk caps
            base_dv_max = self._sc.dv_max_mps_per_s * dt
            dwz_max = self._sc.jw_max_radps2 * dt
            empty_heading = (
                (not self._avoiding) and
                math.isfinite(dmin) and dmin >= self._sc.clear_ahead_thresh_m and
                abs(wz_cmd) <= self._sc.yaw_align_rad
            )
            dv_up_max = base_dv_max * (self._sc.dv_clear_scale if empty_heading else 1.0)
            dv_down_max = base_dv_max * 1.5

            if v_cmd > self._v_cmd_prev:
                v_cmd = min(self._v_cmd_prev + dv_up_max, v_cmd)
            else:
                v_cmd = max(self._v_cmd_prev - dv_down_max, v_cmd)

            wz_cmd = max(self._wz_cmd_prev - dwz_max, min(self._wz_cmd_prev + dwz_max, wz_cmd))
            self._v_cmd_prev = v_cmd
            self._wz_cmd_prev = wz_cmd

            # send
            self._teleop.set_cmd(v_cmd, 0.0, vz_cmd, wz_cmd)

            if timeout_s is not None and (time.time() - t_start) > timeout_s:
                break
            time.sleep(dt)

        self._teleop.stop()
        time.sleep(max(0.05, 2.0 / max(1.0, float(getattr(self._cfg, "rate_hz", 30.0)))))
        elapsed = time.time() - t_start
        return reached, elapsed