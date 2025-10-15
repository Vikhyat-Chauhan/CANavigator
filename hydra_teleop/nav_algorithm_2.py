#!/usr/bin/env python3
# APE2+: APE1 + Breadcrumbs + Safety shims + LiDAR-aware heading + (optional) tiny ML risk scaler
# Drop-in replacement for your APE2/APE3 hybrid; preserves threading and public API.
#
# Adds:
#   - Corridor centering & doorway alignment from LiDAR scan
#   - Clearance-aware heading chooser (local sweep; no grid/map)
#   - TTC + stopping-distance braking (physics-aware)
#   - Soft no-fly-edge aversion (if rectangles provided via cfg)
#   - Tiny logistic risk scaler (optional JSON weights) to trade time for fewer violations
#
# Public API (unchanged):
#   LidarTargetNavigatorAPE3.go_to(target_xyz=None, timeout_s=None) -> (reached: bool, elapsed_s: float)

import math, threading, time
from dataclasses import dataclass
from typing import Optional, Tuple, Set, List, Deque
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

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


# ===== APE1 configs (defaults preserved) =====
@dataclass
class GoToConfig:
    goal_radius_m: float = 4.0
    kp_lin: float = 1.2
    kp_z: float = 1.0
    kp_yaw: float = 2.0
    max_v: float = 15.0
    max_vz: float = 3.5
    max_wz: float = 1.4
    slow_yaw_threshold: float = 1.0
    rate_hz: float = 30.0
    # edge guard to reduce corner clips without losing too much speed
    edge_guard_m: float = 3.5
    edge_guard_scale: float = 0.6

@dataclass
class AvoidCfg:
    scan_topic: str = "/model/drone1/front_lidar/scan"
    safe_m: float = 5.0            # start avoiding if front < safe_m
    hysteresis_m: float = 1.0      # extra clearance to exit avoidance
    front_deg: float = 5.0         # front window ±deg
    side_deg: float = 30.0         # side window half-width
    side_center_deg: float = 30.0  # side windows centered at ±this deg
    turn_rate: float = 0.9         # rad/s while avoiding (capped by max_wz)
    watchdog_sec: float = 0.6      # soft stale threshold
    hard_stale_sec: float = 1.8    # hard stale -> brake
    min_turn_sec: float = 0.7      # commit to chosen side at least this long

# ===== Breadcrumbs (minimal) =====
@dataclass
class BreadcrumbCfg:
    cell_xy_m: float = 2.0
    cell_z_m: float = 2.0
    capacity: int = 3000

# ===== Safety / reliability =====
@dataclass
class SafetyCfg:
    # Use crumbs ONLY when tie/near-tie
    ambiguity_eps_m: float = 0.5     # |left-right| < eps => ambiguous

    # TTC braking (distance / forward_speed)
    ttc_soft_s: float = 2.0          # start slowing under this TTC
    ttc_hard_s: float = 1.0          # hard clamp under this TTC
    v_min_frac: float = 0.15         # never go below this * max_v (unless escaping)

    # Cap turn-rate when any obstacle is close
    near_obs_m: float = 3.0
    cap_wz_near_obs: float = 0.8

    # Corner guard: when heading change is sharp, inflate safe distance
    corner_deg: float = 30.0
    corner_inflate_m: float = 1.5    # extra meters added to safe_m when sharp

    # Progress watchdog
    progress_window_s: float = 3.0
    min_progress_m: float = 1.0
    escape_yaw_rad: float = 0.8
    escape_time_s: float = 0.8

    # Oscillation detection → flip bias
    crumb_oscillations_to_flip: int = 12

# ===== LiDAR-aware & ML risk tuning (optional) =====
@dataclass
class RiskCfg:
    # Vehicle / physics for stopping-distance rule
    vehicle_radius_m: float = 0.7
    max_decel_mps2: float = 6.0
    stop_margin_m: float = 1.0

    # Corridor / doorway logic
    gate_half_deg: float = 12.0     # +/- sector to estimate “doorway” width
    center_weight: float = 0.6      # bias to keep centered in corridor
    align_weight: float = 0.8       # prefer headings that reduce yaw_err
    sweep_max_deg: float = 60.0     # scan heading candidates in [-this,+this]
    sweep_step_deg: float = 2.5

    # No-fly boundary soft-penalty (optional; provide rectangles on cfg)
    nofly_min_dist_m: float = 3.0   # if available, keep at least this much
    nofly_weight: float = 1.0       # (reserved for future cost integration)

    # ML risk scaler (optional)
    risk_json_path: Optional[str] = None  # JSON: {"w":[...]} logistic weights
    risk_gain_safe: float = 1.1     # safe_m *= (1 + gain*sigmoid)
    risk_gain_speed: float = 0.35   # max_v *= (1 - gain*sigmoid)


# ---- internal subscribers ----
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


class LidarTargetNavigatorAPE2:
    """
    APE2+ = APE1 + Breadcrumbs (tiny memory) + safety shims + LiDAR-aware heading.
    Public API and threading model remain the same as APE1.
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
                 risk_cfg: Optional[RiskCfg] = None,
                 logger: Optional[Logger] = None):
        self._teleop = teleop
        self._cfg = cfg
        self._gc = goto_cfg or GoToConfig()
        self._ac = avoid_cfg or AvoidCfg()
        self._bc = crumb_cfg or BreadcrumbCfg()
        self._sc = safety_cfg or SafetyCfg()
        self._rc = risk_cfg or RiskCfg()
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

        # APE1 avoidance state
        self._avoiding = False
        self._avoid_sign = 0          # +1=left, -1=right
        self._avoid_until = 0.0       # commit window end time

        # Breadcrumbs: O(1) membership + FIFO pruning
        self._crumb_set: Set[Tuple[int,int,int]] = set()
        self._crumb_fifo: Deque[Tuple[int,int,int]] = deque()
        self._side_bias: int = +1
        self._crumb_hits_recent: int = 0

        # Progress watchdog
        self._progress_t0: Optional[float] = None
        self._progress_d0: Optional[float] = None
        self._escape_until: float = 0.0

        # Optional ML risk weights
        self._risk_w = self._load_risk(self._rc.risk_json_path)

    # ---------- threading ----------
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

    # ---------- accessors ----------
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

    def _scan_metrics(self) -> Tuple[float, float, float, bool, Optional[LaserScan], float]:
        scan, t_last = self._node_scan.latest()
        now = time.time()
        stale = (now - t_last) > self._ac.watchdog_sec
        if scan is None:
            return float('inf'), float('inf'), float('inf'), True, None, now
        front = _ScanSub._sector_min(scan, 0.0, self._ac.front_deg)
        left  = _ScanSub._sector_min(scan, +self._ac.side_center_deg, self._ac.side_deg)
        right = _ScanSub._sector_min(scan, -self._ac.side_center_deg, self._ac.side_deg)
        return front, left, right, stale, scan, now

    def _cell(self, x: float, y: float, z: float) -> Tuple[int,int,int]:
        return (int(round(x/self._bc.cell_xy_m)),
                int(round(y/self._bc.cell_xy_m)),
                int(round(z/self._bc.cell_z_m)))

    def _crumb_add(self, cell: Tuple[int,int,int]) -> None:
        if cell in self._crumb_set:
            return
        self._crumb_set.add(cell)
        self._crumb_fifo.append(cell)
        if len(self._crumb_fifo) > max(1, self._bc.capacity):
            old = self._crumb_fifo.popleft()
            self._crumb_set.discard(old)

    # ---------- optional risk model ----------
    def _load_risk(self, path: Optional[str]):
        if not path: return None
        try:
            import json, os
            if os.path.isfile(path):
                with open(path) as f:
                    obj = json.load(f)
                w = list(map(float, obj.get("w", [])))
                return w if len(w) >= 1 else None
        except Exception:
            pass
        return None

    def _risk_sigmoid(self, feats: List[float]) -> float:
        w = self._risk_w
        if not w: return 0.0
        s = 0.0
        for i in range(min(len(w), len(feats))):
            s += w[i]*feats[i]
        s = max(-30.0, min(30.0, s))  # clamp
        return 1.0/(1.0 + math.exp(-s))

    # ---------- LiDAR helpers ----------
    def _frange(self, a: float, b: float, step: float):
        x = a
        while x <= b + 1e-9:
            yield x
            x += step

    def _sweep_candidates(self, scan: LaserScan) -> List[float]:
        step = max(0.5, float(self._rc.sweep_step_deg))
        M = max(5.0, float(self._rc.sweep_max_deg))
        return [d for d in self._frange(-M, +M, step)]

    def _range_at(self, scan: LaserScan, center_deg: float, half_w_deg: float=2.0) -> float:
        return _ScanSub._sector_min(scan, center_deg, half_w_deg)

    def _gap_metrics(self, scan: LaserScan) -> Tuple[float, float]:
        """Approximate corridor width & skew near forward using ±gate_half_deg."""
        L = self._range_at(scan, +self._rc.gate_half_deg, half_w_deg=2.0)
        R = self._range_at(scan, -self._rc.gate_half_deg, half_w_deg=2.0)
        width = (L + R)  # crude proxy for opening width
        skew  = (L - R)  # >0 => more space on left
        if not math.isfinite(width): width = float('inf')
        if not math.isfinite(skew):  skew = 0.0
        return width, skew

    def _choose_heading(self, scan: LaserScan, yaw_err: float) -> float:
        """
        Heuristic heading chooser (deg around forward):
          score = range(θ) 
                  + center_weight * (-|skew at θ|) 
                  + align_weight * (-|θ - yaw_err_deg|)
        Returns desired heading offset (rad) relative to forward.
        """
        best = 0.0
        best_score = -1e18
        yaw_err_deg = math.degrees(yaw_err)
        for deg in self._sweep_candidates(scan):
            r = self._range_at(scan, deg, half_w_deg=2.0)
            if not math.isfinite(r): r = 0.0
            l = self._range_at(scan, deg + self._rc.gate_half_deg, 2.0)
            rr = self._range_at(scan, deg - self._rc.gate_half_deg, 2.0)
            skew = abs(l - rr) if (math.isfinite(l) and math.isfinite(rr)) else 0.0
            score = r - self._rc.center_weight*skew - self._rc.align_weight*abs(deg - yaw_err_deg)
            if score > best_score:
                best_score, best = score, deg
        return math.radians(best)

    def _stopping_limited_speed(self, v_des: float, dmin: float) -> float:
        """Enforce v^2/(2a) + margin <= dmin."""
        if not math.isfinite(dmin) or dmin <= self._rc.stop_margin_m:
            return 0.0
        vmax = math.sqrt(max(0.0, 2.0*self._rc.max_decel_mps2*(dmin - self._rc.stop_margin_m)))
        return min(v_des, vmax)

    def _min_dist_nofly(self, x: float, y: float) -> float:
        """
        Optional: supply rectangles via cfg.nofly_rects_xywh = [(cx,cy,w,h), ...].
        If not provided, returns +inf and is ignored.
        """
        rects = getattr(self._cfg, "nofly_rects_xywh", None)
        if not rects: return float('inf')
        best = float('inf')
        for (cx, cy, w, h) in rects:
            dx = max(0.0, abs(x - cx) - 0.5*w)
            dy = max(0.0, abs(y - cy) - 0.5*h)
            best = min(best, math.hypot(dx, dy))
        return best

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

            # ---------- APE1 go-to ----------
            hdg_des = math.atan2(ey, ex)
            yaw_err = _wrap_pi(hdg_des - yaw)

            v_cmd  = min(self._gc.max_v, self._gc.kp_lin * dist_xy)
            if abs(yaw_err) > self._gc.slow_yaw_threshold:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

            vz_cmd = max(-self._gc.max_vz, min(self._gc.max_vz, self._gc.kp_z * ez))
            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_err))

            # ---------- Scan & metrics ----------
            front, left, right, stale, scan, now = self._scan_metrics()

            # Hard-stale: brake until scans recover
            _, t_last = self._node_scan.latest()
            if (time.time() - t_last) > self._ac.hard_stale_sec:
                self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                time.sleep(dt)
                if timeout_s is not None and (time.time() - t_start) > timeout_s: break
                continue

            # ---------- Breadcrumb bookkeeping ----------
            cell = self._cell(x, y, z)
            revisit = (cell in self._crumb_set)
            if revisit:
                self._crumb_hits_recent += 1
            else:
                self._crumb_hits_recent = max(0, self._crumb_hits_recent - 1)
            self._crumb_add(cell)

            # ---------- Doorway/corridor metrics ----------
            corr_width, corr_skew = (float('inf'), 0.0)
            if scan is not None:
                corr_width, corr_skew = self._gap_metrics(scan)

            # ---------- Optional no-fly proximity ----------
            nf_dist = self._min_dist_nofly(x, y)

            # ---------- Tiny ML risk scaler ----------
            risk = 0.0
            if self._risk_w is not None:
                feats = [
                    1.0,
                    min(front, 50.0), min(left, 50.0), min(right, 50.0),
                    float(abs(corr_skew)), float(min(corr_width, 100.0)),
                    float(abs(yaw_err)),
                    float(nf_dist if math.isfinite(nf_dist) else 1e6),
                ]
                risk = self._risk_sigmoid(feats)

            # ---------- Corner/edge guard ----------
            effective_safe_m = self._ac.safe_m
            if abs(math.degrees(yaw_err)) > self._sc.corner_deg:
                effective_safe_m += self._sc.corner_inflate_m
            if min(left, right) < self._gc.edge_guard_m:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

            # Inflate safe by risk; enforce min distance near no-fly
            if risk > 0.0:
                effective_safe_m *= (1.0 + self._rc.risk_gain_safe * risk)
            if math.isfinite(nf_dist) and nf_dist < self._rc.nofly_min_dist_m:
                effective_safe_m = max(effective_safe_m, self._rc.nofly_min_dist_m)

            max_v_risk_cap = self._gc.max_v * (1.0 - self._rc.risk_gain_speed * risk)

            # ---------- Avoidance ----------
            if stale:
                v_cmd = 0.0
                wz_cmd = 0.0
                self._avoiding = False
            else:
                if self._avoiding:
                    # stay avoiding until both time & clearance satisfied
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
                            # ambiguous ⇒ crumbs decide: bias left(+1)/right(-1)
                            self._avoid_sign = (+1 if self._side_bias > 0 else -1)
                        else:
                            self._avoid_sign = (+1 if left > right else -1)
                        self._avoid_until = now + self._ac.min_turn_sec
                        v_cmd = 0.0
                        wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
                    else:
                        # Free path: pick a heading that is clear and aligns with goal
                        if scan is not None:
                            hdg_off = self._choose_heading(scan, yaw_err)
                            yaw_goal = _wrap_pi(hdg_off)
                            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_goal))
                        # Flip side-bias if oscillating in crumbs
                        if self._crumb_hits_recent >= self._sc.crumb_oscillations_to_flip:
                            self._side_bias *= -1
                            self._crumb_hits_recent = 0
                            try:
                                self._logger.info("[APE2+] Breadcrumb: flip side-bias due to oscillation")
                            except Exception:
                                pass

            # ---------- TTC + stopping distance + doorway handling ----------
            if scan is not None:
                window = _ScanSub._window_vals(scan, 0.0, max(5.0, self._ac.front_deg))
                dmin = min(window) if window else float('inf')

                # Doorway: if narrow, align first and go slow
                min_clear = 2.0*self._rc.vehicle_radius_m + 0.6  # +margin
                if math.isfinite(corr_width) and corr_width < (min_clear + 1.0):
                    v_cmd = min(v_cmd, 0.3 * self._gc.max_v)
                    # push yaw toward center (reduce skew)
                    sgn = 1.0 if corr_skew > 0.0 else -1.0   # more room on left -> steer left
                    wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, wz_cmd + 0.5*sgn))

                # TTC map
                if v_cmd > 0.05 and math.isfinite(dmin) and dmin > 0.0:
                    ttc = dmin / max(v_cmd, 1e-3)
                    if ttc < self._sc.ttc_soft_s:
                        num = (ttc - self._sc.ttc_hard_s)
                        den = max(self._sc.ttc_soft_s - self._sc.ttc_hard_s, 1e-3)
                        frac = max(self._sc.v_min_frac, min(1.0, num / den))
                        v_cmd = self._gc.max_v * frac

                    # Stopping-distance hard gate
                    v_cmd = self._stopping_limited_speed(v_cmd, dmin)

            # Heading-rate cap near obstacles and final risk cap on speed
            nearest = min(front, left, right)
            if nearest < self._sc.near_obs_m:
                wz_cmd = max(-self._sc.cap_wz_near_obs, min(self._sc.cap_wz_near_obs, wz_cmd))
            v_cmd = min(v_cmd, max_v_risk_cap)

            # ---------- Progress watchdog (escape rotate if stuck) ----------
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
                # reset window
                self._progress_t0 = now
                self._progress_d0 = dist

            # ---------- send ----------
            self._teleop.set_cmd(v_cmd, 0.0, vz_cmd, wz_cmd)

            if timeout_s is not None and (time.time() - t_start) > timeout_s:
                break
            time.sleep(dt)

        # stop/hold
        self._teleop.stop()
        time.sleep(max(0.05, 2.0 / max(1.0, float(getattr(self._cfg, "rate_hz", 30.0)))))
        elapsed = time.time() - t_start
        return reached, elapsed