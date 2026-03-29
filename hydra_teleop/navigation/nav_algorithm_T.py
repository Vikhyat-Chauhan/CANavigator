#!/usr/bin/env python3
# TROOP (event-aware): default TROOP + Breadcrumbs + Safety shims + LiDAR-aware heading
# + NFZ repulsion + jerk/accel caps + deadline-aware parallel APEs
#
# Public API:
#   LidarTargetNavigatorTROOP.go_to(
#       target_xyz=None, timeout_s=None
#   ) -> (reached: bool, elapsed_s: float, total_latency_us: float,
#          compute_energy_j: float, events_handled: int, events_violated: int,
#          events_violated_deadline: int, events_violated_preemptive: int)

import math, threading, time, json
from dataclasses import dataclass
from typing import Optional, Tuple, Set, List, Deque, Dict
from collections import deque
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from hydra_teleop.navigation.teleop import GzTeleop
from hydra_teleop.config import TeleopConfig
from hydra_teleop.tools.orin_nx_cycle_model import OrinNxCycleMeter, latency_to_energy_j
import logging

# ---------- small math ----------
def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t0, t1)

def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ===== Navigation configs (defaults preserved) =====
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
    edge_guard_m: float = 4.5
    edge_guard_scale: float = 0.6

@dataclass
class AvoidCfg:
    scan_topic: str = "/model/drone1/front_lidar/scan"
    safe_m: float = 5.0
    hysteresis_m: float = 1.0
    front_deg: float = 5.0
    side_deg: float = 30.0
    side_center_deg: float = 30.0
    turn_rate: float = 0.9
    watchdog_sec: float = 0.6
    hard_stale_sec: float = 1.2
    min_turn_sec: float = 0.7

@dataclass
class BreadcrumbCfg:
    cell_xy_m: float = 2.0
    cell_z_m: float = 2.0
    capacity: int = 3000

@dataclass
class SafetyCfg:
    ambiguity_eps_m: float = 0.5
    ttc_soft_s: float = 2.2
    ttc_hard_s: float = 1.4
    v_min_frac: float = 0.20
    near_obs_m: float = 3.0
    cap_wz_near_obs: float = 1.2
    corner_deg: float = 30.0
    corner_inflate_m: float = 2.0
    progress_window_s: float = 3.0
    min_progress_m: float = 1.0
    escape_yaw_rad: float = 0.8
    escape_time_s: float = 0.8
    crumb_oscillations_to_flip: int = 12
    dv_max_mps_per_s: float = 6.0
    jw_max_radps2: float = 3.0
    clear_ahead_thresh_m: float = 16.0
    dv_clear_scale: float = 0.35
    yaw_align_rad: float = 0.25

@dataclass
class RiskCfg:
    vehicle_radius_m: float = 0.7
    max_decel_mps2: float = 4.5
    stop_margin_m: float = 2.0
    gate_half_deg: float = 12.0
    center_weight: float = 0.8
    align_weight: float = 0.8
    sweep_max_deg: float = 60.0
    sweep_step_deg: float = 2.5
    arc_check_m: float = 4.0
    nofly_min_dist_m: float = 3.0
    nofly_soft_w: float = 9.0
    curvature_k: float = 0.9

@dataclass
class EventDecisionCfg:
    event_topic: str = "/hydra/event"
    # APE planning budgets (ms) — derived from orin_nx_cycle_model.py.
    # budget_ms = APE_LATENCY_US[name] × DEADLINE_SCALE=1000
    # (1µs native Cortex-A78AE compute → 1ms effective budget under
    #  Python interpreter ~100× + system contention ~10× on Orin NX)
    # Each APE thread sleeps for budget_ms at startup to emulate this latency.
    ape1_budget_ms: int = 523
    ape2_budget_ms: int = 1343
    ape3_budget_ms: int = 2035

    # Selector thresholds (ms) — decoupled from budgets so the selection
    # distribution can be tuned without changing actual compute times.
    # Invariant: ape2_select_threshold_ms >= ape2_budget_ms (APE2 must finish
    # before its threshold). ape3_select_threshold_ms >= ape3_budget_ms likewise.
    # Default: same as budgets (preserves original behavior).
    ape2_select_threshold_ms: int = 1393   # APE2 budget (1343ms) + 50ms safety margin
    ape3_select_threshold_ms: int = 2035   # use APE3 when deadline >= this

    v_cap_frac: float = 0.75
    selector_mode: str = "TROOP"
    commit_hold_s: float = 0.9

    sudden_obj_radius_m: float = 1.2
    sudden_obj_clearance_m: float = 0.3
    sidestep_deg: float = 110.0
    sidestep_speed_frac: float = 0.35


# ---- internal subscribers ----
class _PoseSub(Node):
    def __init__(self, topic: str, node_name: str, *, callback_group=None):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._latest: Optional[PoseStamped] = None
        cbg = callback_group or ReentrantCallbackGroup()
        self.create_subscription(PoseStamped, topic, self._cb, 10, callback_group=cbg)
    def _cb(self, msg: PoseStamped):
        with self._lock:
            self._latest = msg
    def latest(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._latest

class _ScanSub(Node):
    def __init__(self, topic: str, *, callback_group=None):
        super().__init__("hydra_nav_lidar")
        self._lock = threading.Lock()
        self._scan: Optional[LaserScan] = None
        self._t_last = 0.0
        cbg = callback_group or ReentrantCallbackGroup()
        self.create_subscription(LaserScan, topic, self._cb, 10, callback_group=cbg)
    def _cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg
            self._t_last = self.get_clock().now().nanoseconds * 1e-9
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
    def __init__(self, topic: str, *, callback_group=None):
        super().__init__("hydra_event_sub")
        self._lock = threading.Lock()
        self._pending: Optional[Dict] = None
        self._t_last = 0.0
        cbg = callback_group or ReentrantCallbackGroup()
        self.create_subscription(String, topic, self._cb, 10, callback_group=cbg)

    def _cb(self, msg: String):
        try:
            obj = json.loads(msg.data)
            # Drop reset sentinels — used to flush stale buffered events at run boundaries
            if obj.get("kind") == "__RESET__":
                with self._lock:
                    self._pending = None   # explicitly clear any stale event
                return
            obj["t_recv"] = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            return
        with self._lock:
            self._pending = obj
            self._t_last = obj["t_recv"]

    def pop(self) -> Optional[Dict]:
        with self._lock:
            v = self._pending
            self._pending = None
            return v


class LidarTargetNavigatorTROOP:
    """
    Default navigator. When an event arrives, APE1/APE2/APE3 workers run in
    parallel; the selector picks the highest-quality plan that can finish before
    the deadline. The winner is determined at event intake from the deadline alone
    — APE budgets are fixed constants, so no per-tick re-evaluation is needed.
    """

    # ------------------------------------------------------------------
    # TROOP selection table (all values in seconds)
    #
    #   deadline >= ape3_budget_s  →  wait for APE3  (best quality)
    #   deadline >= ape2_budget_s  →  wait for APE2
    #   deadline >= ape1_budget_s  →  wait for APE1  (fastest)
    #   deadline <  ape1_budget_s  →  DEADLINE_MISS  (no APE can finish)
    # ------------------------------------------------------------------

    def __init__(self,
                 teleop: GzTeleop,
                 cfg: TeleopConfig,
                 selector_mode: str,
                 drone_pose_topic: Optional[str] = None,
                 target_pose_topic: Optional[str] = "/model/target_sphere/pose/info",
                 goto_cfg: Optional[GoToConfig] = None,
                 avoid_cfg: Optional[AvoidCfg] = None,
                 crumb_cfg: Optional[BreadcrumbCfg] = None,
                 safety_cfg: Optional[SafetyCfg] = None,
                 risk_cfg: Optional[RiskCfg] = None,
                 ape2_select_threshold_ms: Optional[int] = None,
                 ape3_select_threshold_ms: Optional[int] = None):
        self._teleop = teleop
        self._cfg = cfg
        self._gc = goto_cfg or GoToConfig()
        self._ac = avoid_cfg or AvoidCfg()
        self._bc = crumb_cfg or BreadcrumbCfg()
        self._sc = safety_cfg or SafetyCfg()
        self._rc = risk_cfg or RiskCfg()
        self._logger = logging.getLogger(__name__)
        self._logger.propagate = True
        self._cycle_meter = OrinNxCycleMeter()

        self._events_handled: int = 0
        self._events_violated: int = 0
        self._events_violated_deadline: int = 0
        self._events_violated_preemptive: int = 0

        entity = getattr(cfg, "entity_name", "drone1")
        drone_topic = drone_pose_topic or getattr(cfg, "ros_pose_topic", f"/model/{entity}/pose/info")

        self._cbg = ReentrantCallbackGroup()
        self._node_drone  = _PoseSub(drone_topic, "lidar_hydra_nav_drone_pose", callback_group=self._cbg)
        self._node_target = _PoseSub(target_pose_topic, "lidar_hydra_nav_target_pose", callback_group=self._cbg)
        self._node_scan   = _ScanSub(self._ac.scan_topic, callback_group=self._cbg)

        self._edc = EventDecisionCfg()
        try:
            sm = selector_mode
            if sm:
                self._edc.selector_mode = str(sm).upper().strip()
        except Exception:
            pass
        if ape2_select_threshold_ms is not None:
            self._edc.ape2_select_threshold_ms = ape2_select_threshold_ms
        if ape3_select_threshold_ms is not None:
            self._edc.ape3_select_threshold_ms = ape3_select_threshold_ms

        self._log("CFG",
                  type="CFG",
                  ape_budgets_ms=[self._edc.ape1_budget_ms, self._edc.ape2_budget_ms, self._edc.ape3_budget_ms],
                  ape_select_thresholds_ms=[self._edc.ape1_budget_ms, self._edc.ape2_select_threshold_ms, self._edc.ape3_select_threshold_ms],
                  v_cap_frac=self._edc.v_cap_frac,
                  selector_mode=self._edc.selector_mode,
                  commit_hold_s=self._edc.commit_hold_s,
                  sudden_obj_radius_m=self._edc.sudden_obj_radius_m,
                  sudden_obj_clearance_m=self._edc.sudden_obj_clearance_m,
                  sidestep_deg=self._edc.sidestep_deg,
                  sidestep_speed_frac=self._edc.sidestep_speed_frac)

        self._node_evt = _EventSub(getattr(cfg, "event_topic", self._edc.event_topic), callback_group=self._cbg)

        self._executor = None
        self._nodes = (self._node_drone, self._node_target, self._node_scan, self._node_evt)

        self._avoiding = False
        self._avoid_sign = 0
        self._avoid_until = 0.0

        self._crumb_set: Set[Tuple[int,int,int]] = set()
        self._crumb_fifo: Deque[Tuple[int,int,int]] = deque()
        self._side_bias: int = +1
        self._crumb_hits_recent: int = 0

        self._progress_t0: Optional[float] = None
        self._progress_d0: Optional[float] = None
        self._escape_until: float = 0.0

        self._v_cmd_prev = 0.0
        self._wz_cmd_prev = 0.0

        self._pending_evt: Optional[Dict] = None
        self._evt_deadline_at: float = 0.0
        self._evt_lock = threading.Lock()
        self._evt_proposals: Dict[str, Dict] = {}
        self._evt_threads: List[threading.Thread] = []

        self._evt_active: bool = False
        self._evt_resolved: bool = False

        # Winner APE name decided at intake, not per-tick.
        self._evt_winner: str = "APE1"

        self._resolved_cmd: tuple = (0.0, 0.0, 0.0)
        self._evt_resolved_at: float = 0.0
        self._commit_hold_active: bool = False
        self._nav_start_logged: bool = False

    # ---------- logging ----------
    def _log(self, msg: str, **fields):
        log_type = fields.pop("type", "GEN")
        try:
            self._logger.info(msg, extra={"type": log_type, "payload": fields})
        except Exception as e:
            print("LOGGING_ERROR:", e)

    # ---------- sim-time ----------
    def _sim_time(self) -> float:
        """Return current sim time (seconds). Falls back to wall time before /clock arrives."""
        ns = self._node_scan.get_clock().now().nanoseconds
        if ns > 0:
            return ns * 1e-9
        return time.time()

    # ---------- executor ----------
    def attach_to_executor(self, executor) -> None:
        if self._executor is not None:
            return
        for n in self._nodes:
            executor.add_node(n)
        self._executor = executor

    def shutdown(self):
        try:
            if self._executor is not None:
                for n in self._nodes:
                    try:
                        self._executor.remove_node(n)
                    except Exception:
                        pass
                self._executor = None
            for n in self._nodes:
                try:
                    n.destroy_node()
                except Exception:
                    pass
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
        now = self._sim_time()
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

    def _range_at(self, scan: LaserScan, center_deg: float, half_w_deg: float = 2.0) -> float:
        return _ScanSub._sector_min(scan, center_deg, half_w_deg)

    def _gap_metrics(self, scan: LaserScan) -> Tuple[float, float]:
        L = self._range_at(scan, +self._rc.gate_half_deg, half_w_deg=2.0)
        R = self._range_at(scan, -self._rc.gate_half_deg, half_w_deg=2.0)
        width = (L + R)
        skew  = (L - R)
        if not math.isfinite(width): width = float('inf')
        if not math.isfinite(skew):  skew = 0.0
        return width, skew

    def _confidence_from_scan(self, scan: Optional[LaserScan], yaw_err: float) -> float:
        if scan is None:
            return 0.0
        fwd_vals = _ScanSub._window_vals(scan, 0.0, max(5.0, self._ac.front_deg))
        dmin = min(fwd_vals) if fwd_vals else float('inf')
        dmin_n = max(0.0, min(1.0, (dmin - 4.0) / 16.0))
        w, _ = self._gap_metrics(scan)
        w_n = 0.0 if not math.isfinite(w) else max(0.0, min(1.0, (w - 3.0) / 12.0))
        yaw_n = 1.0 - max(0.0, min(1.0, abs(yaw_err) / math.radians(40.0)))
        cL = self._arc_is_clear(scan, -15.0, 4.0)
        cC = self._arc_is_clear(scan,   0.0, 4.0)
        cR = self._arc_is_clear(scan, +15.0, 4.0)
        cons_n = ((1.0 if cL else 0.0) + (1.0 if cC else 0.0) + (1.0 if cR else 0.0)) / 3.0
        conf = 0.35*dmin_n + 0.30*w_n + 0.25*yaw_n + 0.10*cons_n
        return max(0.0, min(1.0, conf))

    # ---------- No-fly helpers ----------
    def _nofly_rects(self):
        return getattr(self._cfg, "nofly_rects_xywh", None) or []

    def _min_dist_nofly(self, x: float, y: float) -> float:
        rects = self._nofly_rects()
        if not rects: return float('inf')
        best = float('inf')
        for (cx, cy, w, h) in rects:
            dx = max(0.0, abs(x - cx) - 0.5*w)
            dy = max(0.0, abs(y - cy) - 0.5*h)
            best = min(best, math.hypot(dx, dy))
        return best

    def _nfz_repulsion_vec(self, x: float, y: float) -> Tuple[float, float, float]:
        rects = self._nofly_rects()
        if not rects: return 0.0, 0.0, 0.0
        eps = 0.5
        fx = fy = 0.0
        cost = 0.0
        for (cx, cy, w, h) in rects:
            dx = x - cx
            dy = y - cy
            dx_out = max(0.0, abs(dx) - 0.5*w)
            dy_out = max(0.0, abs(dy) - 0.5*h)
            d = math.hypot(dx_out, dy_out)
            cost += 1.0 / (d + eps)
            if d > 1e-3:
                fx += (dx_out / d) / (d + eps)
                fy += (dy_out / d) / (d + eps)
        return fx, fy, cost

    # ---------- arc safety check ----------
    def _arc_is_clear(self, scan: LaserScan, deg: float, arc_m: float) -> bool:
        r = self._range_at(scan, deg, half_w_deg=2.0)
        return (not math.isfinite(r)) or (r >= arc_m)

    def _choose_heading(self, scan: LaserScan, yaw_err: float, x: float, y: float) -> float:
        yaw_err_deg = math.degrees(yaw_err)
        best = 0.0
        best_score = -1e18
        _, _, nfz_soft0 = self._nfz_repulsion_vec(x, y)
        for deg in self._sweep_candidates(scan):
            if not self._arc_is_clear(scan, deg, self._rc.arc_check_m):
                continue
            r = self._range_at(scan, deg, half_w_deg=2.0)
            if not math.isfinite(r): r = 0.0
            l = self._range_at(scan, deg + self._rc.gate_half_deg, 2.0)
            rr = self._range_at(scan, deg - self._rc.gate_half_deg, 2.0)
            skew = abs(l - rr) if (math.isfinite(l) and math.isfinite(rr)) else 0.0
            score = r - self._rc.center_weight*skew - self._rc.align_weight*abs(deg - yaw_err_deg) - self._rc.nofly_soft_w*nfz_soft0
            if score > best_score:
                best_score, best = score, deg
        return math.radians(best)

    def _stopping_limited_speed(self, v_des: float, dmin: float) -> float:
        if not math.isfinite(dmin) or dmin <= self._rc.stop_margin_m:
            return 0.0
        vmax = math.sqrt(max(0.0, 2.0*self._rc.max_decel_mps2*(dmin - self._rc.stop_margin_m)))
        return min(v_des, vmax)

    # ---------- event planners ----------
    def _evt_put(self, name, v, wz, vz, score):
        with self._evt_lock:
            self._evt_proposals[name] = {"v": v, "wz": wz, "vz": vz, "score": score, "ready_t": self._sim_time()}

    def _shared_motion_caps(self, base_v: float, yaw_goal_rad: float, base_vz: float,
                            scan: Optional[LaserScan],
                            v_cap_frac_override: Optional[float] = None,
                            curvature_k_scale: float = 1.0):
        wz = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * _wrap_pi(yaw_goal_rad)))
        dmin = float('inf')
        if scan is not None:
            window = _ScanSub._window_vals(scan, 0.0, max(5.0, self._ac.front_deg))
            dmin = min(window) if window else float('inf')
        curv_k = max(0.05, float(self._rc.curvature_k) * float(curvature_k_scale))
        curv_cap = self._gc.max_v / (1.0 + curv_k * abs(wz))
        v_event_cap = (v_cap_frac_override if v_cap_frac_override is not None else self._edc.v_cap_frac) * self._gc.max_v
        v = min(base_v, v_event_cap, curv_cap)
        v = self._stopping_limited_speed(v, dmin)
        return v, wz, base_vz, dmin

    def _evt_plan_ape1(self, snap, budget_ms):
        time.sleep(budget_ms / 1000.0)
        v_cmd, scan = snap["v_cmd"], snap["scan"]
        target_off = math.radians(110.0)
        slow_frac = 0.05
        base_v = min(v_cmd, slow_frac * self._gc.max_v)
        v_cap_local = 0.3 * self._edc.v_cap_frac
        v, wz, vz, _ = self._shared_motion_caps(base_v, target_off, 0.0, scan,
                                                  v_cap_frac_override=v_cap_local)
        return self._evt_put("APE1", v, wz, vz, score=-1e6)

    def _evt_plan_ape2(self, snap, budget_ms):
        time.sleep(budget_ms / 1000.0)
        v_cmd, scan = snap["v_cmd"], snap["scan"]
        pick_left = (self._side_bias > 0)
        target_off = math.radians(self._edc.sidestep_deg) * (+1 if pick_left else -1)
        base_v = min(v_cmd, self._edc.sidestep_speed_frac * self._gc.max_v)
        v, wz, vz, _ = self._shared_motion_caps(base_v, target_off, 0.0, scan)
        score = -0.2 * abs(wz)
        return self._evt_put("APE2", v, wz, vz, score)

    def _evt_plan_ape3(self, snap, budget_ms):
        time.sleep(budget_ms / 1000.0)
        v_cmd, scan = snap["v_cmd"], snap["scan"]
        yaw_err = snap["yaw_err"]
        conf = self._confidence_from_scan(scan, yaw_err)
        ds = (self._edc.sudden_obj_radius_m + self._rc.vehicle_radius_m + self._edc.sudden_obj_clearance_m)
        pick_left = (self._side_bias > 0)
        target_off = math.radians(self._edc.sidestep_deg) * (+1 if pick_left else -1)
        sidestep_speed_frac_eff = min(0.95, self._edc.sidestep_speed_frac + 0.2 * conf)
        base_v = min(v_cmd, sidestep_speed_frac_eff * self._gc.max_v)
        vcap = min(0.95, self._edc.v_cap_frac + 0.2 * conf)
        curv_scale = (1.0 - 0.4 * conf)
        v, wz, vz, _ = self._shared_motion_caps(base_v, target_off, 0.0, scan,
                                                  v_cap_frac_override=vcap,
                                                  curvature_k_scale=curv_scale)
        score = +0.12 * ds - 0.04 * abs(wz) + 0.02 * v
        return self._evt_put("APE3", v, wz, vz, score)

    # ---------- event helpers ----------
    def _evt_winner_for_deadline(self, deadline_s: float) -> str:
        """
        Determine which APE to wait for, purely from the deadline.

        APE budgets are fixed constants — all three threads launch at t=0
        and post their proposals at t=ape_budget_s. The highest-quality APE
        that can post before the deadline is the winner. No per-tick logic needed.

        Returns "APE3", "APE2", "APE1", or "MISS" (no APE can finish in time).
        """
        ape3_s = self._edc.ape3_select_threshold_ms / 1000.0
        ape2_s = self._edc.ape2_select_threshold_ms / 1000.0
        ape1_s = self._edc.ape1_budget_ms / 1000.0   # hard lower bound — APE1 cannot go lower

        if deadline_s >= ape3_s:
            return "APE3"
        elif deadline_s >= ape2_s:
            return "APE2"
        elif deadline_s >= ape1_s:
            return "APE1"
        else:
            return "MISS"

    def _evt_violate(self, reason: str = "miss"):
        if self._pending_evt is None:
            return
        self._events_violated += 1
        if reason == "DEADLINE":
            self._events_violated_deadline += 1
        elif reason == "PREEMPTIVE":
            self._events_violated_preemptive += 1

    def _evt_clear(self):
        with self._evt_lock:
            self._evt_proposals.clear()
        for t in self._evt_threads:
            if t.is_alive():
                t.join(timeout=0.002)
        self._evt_threads.clear()
        self._pending_evt = None
        self._evt_deadline_at = 0.0
        self._evt_active = False
        self._evt_resolved = False
        self._evt_winner = "APE1"

    # ---------- APE calibration ----------
    def _calibrate_budgets(self, n_reps: int = 30) -> None:
        import statistics
        from sensor_msgs.msg import LaserScan as _LS
        _scan = _LS()
        _scan.angle_min       = -math.radians(30.0)
        _scan.angle_max       = +math.radians(30.0)
        _scan.angle_increment =  math.radians(1.0)
        _scan.range_min       = 0.1
        _scan.range_max       = 30.0
        _scan.ranges          = [10.0] * 61
        _snap = {
            "v_cmd": 10.0,
            "scan": _scan,
            "yaw_err": 0.1,
        }
        results = {}
        for name, fn in [("APE1", self._evt_plan_ape1),
                          ("APE2", self._evt_plan_ape2),
                          ("APE3", self._evt_plan_ape3)]:
            times_ms = []
            for _ in range(n_reps):
                with self._evt_lock:
                    self._evt_proposals.pop(name, None)
                t0 = time.perf_counter()
                fn(_snap, 0)
                times_ms.append((time.perf_counter() - t0) * 1000.0)
            results[name] = {
                "mean": statistics.mean(times_ms),
                "p95":  sorted(times_ms)[int(0.95 * len(times_ms))],
                "wcet": max(times_ms),
            }
        with self._evt_lock:
            self._evt_proposals.clear()
        self._log("CALIBRATION", type="CALIBRATION",
                  ape1_mean_ms=round(results["APE1"]["mean"], 3),
                  ape1_p95_ms =round(results["APE1"]["p95"],  3),
                  ape1_wcet_ms=round(results["APE1"]["wcet"], 3),
                  ape2_mean_ms=round(results["APE2"]["mean"], 3),
                  ape2_p95_ms =round(results["APE2"]["p95"],  3),
                  ape2_wcet_ms=round(results["APE2"]["wcet"], 3),
                  ape3_mean_ms=round(results["APE3"]["mean"], 3),
                  ape3_p95_ms =round(results["APE3"]["p95"],  3),
                  ape3_wcet_ms=round(results["APE3"]["wcet"], 3),
                  configured_budget_ms=[self._edc.ape1_budget_ms,
                                        self._edc.ape2_budget_ms,
                                        self._edc.ape3_budget_ms])

    # ---------- core ----------
    def go_to(self,
              target_xyz: Optional[Tuple[float, float, float]] = None,
              timeout_s: Optional[float] = None) -> Tuple[bool, float, float, float, int, int, int, int]:
        if self._executor is None:
            raise RuntimeError("TROOP navigator is not attached to an executor. Call attach_to_executor(executor) first.")
        rate = max(1.0, float(self._gc.rate_hz))
        dt = 1.0 / rate
        t_start = self._sim_time()
        self._cycle_meter.begin()
        reached = False
        self._teleop.start()

        self._events_handled = 0
        self._events_violated = 0
        self._events_violated_deadline = 0
        self._events_violated_preemptive = 0
        self._nav_start_logged = False

        def _evt_time_left() -> float:
            return max(0.0, self._evt_deadline_at - self._sim_time())

        while True:
            dpose = self._latest_drone()
            if dpose is None:
                time.sleep(0.05)
                if timeout_s is not None and (self._sim_time() - t_start) > timeout_s:
                    break
                continue

            if target_xyz is None:
                tpose = self._latest_target()
                if tpose is None:
                    self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                    if timeout_s is not None and (self._sim_time() - t_start) > timeout_s:
                        break
                    time.sleep(0.05)
                    continue
                tx, ty, tz = tpose
            else:
                tx, ty, tz = target_xyz

            x, y, z, yaw = dpose
            ex, ey, ez = (tx - x), (ty - y), (tz - z)
            dist_xy = math.hypot(ex, ey)
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)

            if not self._nav_start_logged:
                self._log("POSES", type="POSES",
                          nav_start_drone_pose=(x, y, z, yaw),
                          nav_start_target=(tx, ty, tz),
                          nav_start_dist_m=round(dist, 3),
                          nav_start_dist_xy_m=round(dist_xy, 3))
                self._nav_start_logged = True

            if dist <= self._gc.goal_radius_m:
                reached = True
                break

            # ---------- Base go-to ----------
            hdg_des = math.atan2(ey, ex)
            yaw_err = _wrap_pi(hdg_des - yaw)

            v_cmd  = min(self._gc.max_v, self._gc.kp_lin * dist_xy)
            if abs(yaw_err) > self._gc.slow_yaw_threshold:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

            vz_cmd = max(-self._gc.max_vz, min(self._gc.max_vz, self._gc.kp_z * ez))
            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_err))

            front, left, right, stale, scan, now = self._scan_metrics()

            # ---- Event intake ----
            evt = self._node_evt.pop()
            if evt is not None:
                deadline_s = max(0.0, float(evt.get("deadline_s", 0.0)))
                self._log("EVENT", type="ARRIVAL",
                          t_rec=evt["t_recv"],
                          deadline_s=deadline_s,
                          deadline_computed=evt["t_recv"] + deadline_s)
                self._events_handled += 1

                # Salvage old active event before replacing it
                if self._evt_active:
                    with self._evt_lock:
                        ready_curr = dict(self._evt_proposals)
                    # Try winner first, then cascade down
                    salvage_order = {"APE3": ["APE3", "APE2", "APE1"],
                                     "APE2": ["APE2", "APE1"],
                                     "APE1": ["APE1"]}.get(self._evt_winner, ["APE1"])
                    if self._edc.selector_mode != "TROOP":
                        salvage_order = [self._edc.selector_mode]
                    chosen_curr = next(
                        ((n, ready_curr[n]) for n in salvage_order if n in ready_curr),
                        None
                    )
                    if chosen_curr is not None:
                        _, prop = chosen_curr
                        v_cmd, wz_cmd, vz_cmd = prop["v"], prop["wz"], prop["vz"]
                        if not self._evt_resolved:
                            self._evt_resolved = True
                        self._evt_clear()
                    else:
                        self._evt_violate("PREEMPTIVE")
                        self._log("EVENT", type="PREEMPTIVE", c_time=self._sim_time())
                        self._evt_clear()

                # Determine winner APE for this event at intake time
                if self._edc.selector_mode == "TROOP":
                    winner = self._evt_winner_for_deadline(deadline_s)
                else:
                    winner = self._edc.selector_mode  # forced single-APE mode

                if winner == "MISS":
                    # No APE can finish in time — count as deadline violation immediately
                    self._events_violated += 1
                    self._events_violated_deadline += 1
                    self._log("EVENT", type="DEADLINE_PREEMPT",
                              c_time=self._sim_time(), deadline_s=deadline_s,
                              reason="deadline shorter than fastest APE budget")
                else:
                    # Start tracking the new event
                    self._commit_hold_active = False
                    self._pending_evt = evt
                    self._evt_deadline_at = evt["t_recv"] + deadline_s
                    self._evt_active = True
                    self._evt_resolved = False
                    self._evt_winner = winner

                    with self._evt_lock:
                        self._evt_proposals = {}
                    for t in self._evt_threads:
                        if t.is_alive():
                            t.join(timeout=0.001)
                    self._evt_threads = []

                    snap = {
                        "v_cmd": v_cmd,
                        "scan": scan,
                        "yaw_err": _wrap_pi(math.atan2(ey, ex) - yaw),
                    }

                    mode = self._edc.selector_mode
                    threads = []
                    if mode in ("TROOP", "APE1"):
                        threads.append(threading.Thread(
                            target=self._evt_plan_ape1,
                            args=(snap, self._edc.ape1_budget_ms), daemon=True))
                    if mode in ("TROOP", "APE2"):
                        threads.append(threading.Thread(
                            target=self._evt_plan_ape2,
                            args=(snap, self._edc.ape2_budget_ms), daemon=True))
                    if mode in ("TROOP", "APE3"):
                        threads.append(threading.Thread(
                            target=self._evt_plan_ape3,
                            args=(snap, self._edc.ape3_budget_ms), daemon=True))
                    self._evt_threads = threads
                    for t in self._evt_threads:
                        t.start()

            event_active = self._evt_active and (self._pending_evt is not None)

            # Hard-stale: brake until scans recover
            _, t_last = self._node_scan.latest()
            if (self._sim_time() - t_last) > self._ac.hard_stale_sec:
                self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                time.sleep(dt)
                if timeout_s is not None and (self._sim_time() - t_start) > timeout_s:
                    break
                continue

            # ---------- Breadcrumb bookkeeping ----------
            cell = self._cell(x, y, z)
            if cell in self._crumb_set:
                self._crumb_hits_recent += 1
            else:
                self._crumb_hits_recent = max(0, self._crumb_hits_recent - 1)
            self._crumb_add(cell)

            # ---------- Doorway/corridor metrics ----------
            corr_width, corr_skew = (float('inf'), 0.0)
            if scan is not None:
                corr_width, corr_skew = self._gap_metrics(scan)

            # ---------- No-fly proximity & repulsion ----------
            nf_dist = self._min_dist_nofly(x, y)
            fx, fy, nfz_soft = self._nfz_repulsion_vec(x, y)
            rep_angle = math.atan2(fy, fx) if (fx*fx + fy*fy) > 1e-6 else None
            if rep_angle is not None and math.isfinite(rep_angle):
                yaw_rep_err = _wrap_pi(rep_angle - yaw)
                wz_cmd += 0.4 * max(-self._gc.max_wz, min(self._gc.max_wz, yaw_rep_err))

            # ---------- Corner/edge guard ----------
            effective_safe_m = self._ac.safe_m
            if abs(math.degrees(yaw_err)) > self._sc.corner_deg:
                effective_safe_m += self._sc.corner_inflate_m
            if min(left, right) < self._gc.edge_guard_m:
                v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)
            if math.isfinite(nf_dist) and nf_dist < self._rc.nofly_min_dist_m:
                effective_safe_m = max(effective_safe_m, self._rc.nofly_min_dist_m)
                v_cmd = min(v_cmd, 0.4 * self._gc.max_v)

            # ---------- Event window — simplified selector ----------
            if event_active:
                tl = _evt_time_left()
                if tl <= 0.0:
                    self._evt_violate("DEADLINE")
                    self._log("EVENT", type="DEADLINE", c_time=self._sim_time())
                    self._evt_clear()
                else:
                    with self._evt_lock:
                        ready = dict(self._evt_proposals)

                    # self._evt_winner was fixed at intake. Just wait for it.
                    # No fallback logic: if it hasn't posted yet, keep waiting.
                    # If it has posted, take it immediately.
                    chosen = None
                    if self._evt_winner in ready:
                        chosen = (self._evt_winner, ready[self._evt_winner])

                    if chosen is not None:
                        winner_name, prop = chosen
                        v_cmd, wz_cmd, vz_cmd = prop["v"], prop["wz"], prop["vz"]
                        if not self._evt_resolved:
                            self._log("EVENT", type="RESOLVED",
                                      planner=winner_name,
                                      ready_t=prop["ready_t"])
                            _running = (["APE1", "APE2", "APE3"]
                                        if self._edc.selector_mode == "TROOP"
                                        else [winner_name])
                            self._cycle_meter.record_event(winner_name, _running)
                            self._evt_resolved = True
                            self._resolved_cmd = (prop["v"], prop["wz"], prop["vz"])
                            self._evt_resolved_at = self._sim_time()
                            self._commit_hold_active = True
                            self._evt_clear()

            # ---------- Commitment hold ----------
            if self._commit_hold_active:
                hold_elapsed = self._sim_time() - self._evt_resolved_at
                if hold_elapsed < self._edc.commit_hold_s:
                    v_cmd, wz_cmd, vz_cmd = self._resolved_cmd
                else:
                    self._commit_hold_active = False

            # ---------- Avoidance / heading select ----------
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
                        if scan is not None:
                            hdg_off = self._choose_heading(scan, yaw_err, x, y)
                            yaw_goal = _wrap_pi(hdg_off)
                            wz_cmd = max(-self._gc.max_wz,
                                         min(self._gc.max_wz, self._gc.kp_yaw * yaw_goal))
                        side_min = min(left, right)
                        if side_min < 1.2 * self._rc.vehicle_radius_m + 0.6:
                            if left < right:
                                wz_cmd -= 0.25
                            else:
                                wz_cmd += 0.25
                        if self._crumb_hits_recent >= self._sc.crumb_oscillations_to_flip:
                            self._side_bias *= -1
                            self._crumb_hits_recent = 0

            # ---------- Doorway + TTC + stopping distance ----------
            dmin = float('inf')
            if scan is not None:
                window = _ScanSub._window_vals(scan, 0.0, max(5.0, self._ac.front_deg))
                dmin = min(window) if window else float('inf')
                min_clear = 2.0*self._rc.vehicle_radius_m + 0.6
                if math.isfinite(corr_width) and corr_width < (min_clear + 1.0):
                    v_cmd = min(v_cmd, 0.25 * self._gc.max_v)
                    sgn = 1.0 if corr_skew > 0.0 else -1.0
                    wz_cmd = max(-self._gc.max_wz,
                                 min(self._gc.max_wz, wz_cmd + 0.5*sgn))
                if v_cmd > 0.05 and math.isfinite(dmin) and dmin > 0.0:
                    ttc = dmin / max(v_cmd, 1e-3)
                    if ttc < self._sc.ttc_soft_s:
                        num = (ttc - self._sc.ttc_hard_s)
                        den = max(self._sc.ttc_soft_s - self._sc.ttc_hard_s, 1e-3)
                        frac = max(self._sc.v_min_frac, min(1.0, num / den))
                        v_cmd = self._gc.max_v * frac
                    v_cmd = self._stopping_limited_speed(v_cmd, dmin)

            v_cmd = min(v_cmd, self._gc.max_v / (1.0 + self._rc.curvature_k * abs(wz_cmd)))

            nearest = min(front, left, right)
            if nearest < self._sc.near_obs_m:
                wz_cmd = max(self._sc.cap_wz_near_obs * -1.0,
                             min(self._sc.cap_wz_near_obs, wz_cmd))

            # ---------- Progress watchdog ----------
            if self._progress_t0 is None:
                self._progress_t0 = now
                self._progress_d0 = dist

            if now < self._escape_until:
                v_cmd = 0.0
                wz_cmd = (self._sc.escape_yaw_rad / self._sc.escape_time_s) * (
                    1 if self._avoid_sign >= 0 else -1)
            elif (now - self._progress_t0) > self._sc.progress_window_s:
                gained = (self._progress_d0 - dist)
                if gained < self._sc.min_progress_m:
                    self._escape_until = now + self._sc.escape_time_s
                    v_cmd = 0.0
                    wz_cmd = (self._sc.escape_yaw_rad / self._sc.escape_time_s) * (
                        1 if self._avoid_sign >= 0 else -1)
                self._progress_t0 = now
                self._progress_d0 = dist

            # ---------- Command ramp/jerk caps ----------
            base_dv_max = self._sc.dv_max_mps_per_s * dt
            dwz_max = self._sc.jw_max_radps2 * dt

            empty_heading = (
                (not self._avoiding)
                and math.isfinite(dmin)
                and dmin >= self._sc.clear_ahead_thresh_m
                and abs(wz_cmd) <= self._sc.yaw_align_rad
            )

            dv_up_max = base_dv_max * (self._sc.dv_clear_scale if empty_heading else 1.0)
            dv_down_max = base_dv_max * 1.5

            if v_cmd > self._v_cmd_prev:
                v_cmd = min(self._v_cmd_prev + dv_up_max, v_cmd)
            else:
                v_cmd = max(self._v_cmd_prev - dv_down_max, v_cmd)

            wz_cmd = max(self._wz_cmd_prev - dwz_max,
                         min(self._wz_cmd_prev + dwz_max, wz_cmd))

            self._v_cmd_prev = v_cmd
            self._wz_cmd_prev = wz_cmd

            self._teleop.set_cmd(v_cmd, 0.0, vz_cmd, wz_cmd)

            if timeout_s is not None and (self._sim_time() - t_start) > timeout_s:
                break
            time.sleep(dt)

        self._teleop.stop()
        time.sleep(max(0.05, 2.0 / max(1.0, self._cfg.rate_hz)))
        elapsed = self._sim_time() - t_start

        total_latency_us, _ = self._cycle_meter.end()
        compute_energy_j = latency_to_energy_j(total_latency_us, elapsed)
        return (reached, elapsed, total_latency_us, compute_energy_j,
                self._events_handled, self._events_violated,
                self._events_violated_deadline, self._events_violated_preemptive)


'''
Research references used for this model

Kansal et al., "Joulemeter: Virtualized Power Estimation in Datacenters." SOSP Workshop, 2010.
Fan, Weber, Barroso. "Power provisioning for a warehouse-sized computer." ISCA 2007.
Bircher & John. "Complete system power estimation using processor performance events." IEEE TC, 2012.
Coroama & Hilty. "Energy consumption of servers—Modeling and validation." IT Professional, 2014.
Ryffel et al., "Accurate and Lightweight Power Modeling for Modern Processors." 2015–2018.
'''