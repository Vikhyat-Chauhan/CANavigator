#!/usr/bin/env python3
# hydra_teleop/events.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple, List
import json, time, threading, random, math, csv, os

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import logging

from hydra_teleop.config import TeleopConfig

# ----------------------------- Data classes -----------------------------

@dataclass
class PhaseSpec:
    """One phase/block in the experiment."""
    duration_s: float                 # phase length
    rate_hz: float                    # baseline Poisson rate (events/sec)
    mix_enemy: float                  # probability mass for ENEMY events
    mix_obstacle: float               # probability mass for SUDDEN_OBSTACLE
    mix_lane: float                   # probability mass for LANE_BLOCK
    burstiness: float = 0.0           # 0=Poisson; >0 => occasional bursts (Hawkes-like)

@dataclass
class EventCfg:
    topic: str = "/hydra/event"
    # Randomness / reproducibility
    seed: Optional[int] = None
    jitter_s: float = 0.05

    # Deadlines (constant, fairness-oriented)
    default_deadline_s: float = 0.35
    min_deadline_s: float = 0.12
    max_deadline_s: float = 0.80
    deadline_scale: float = 1.0
    # Optional per-kind constants (if None, fall back to default_deadline_s)
    enemy_deadline_s: Optional[float] = None
    obstacle_deadline_s: Optional[float] = None
    lane_deadline_s: Optional[float] = None

    # Bounds for 2D arena (for spawned events)
    x_min: float = -100.0; x_max: float = 100.0
    y_min: float =  -50.0; y_max: float =   50.0

    # Enemy parameters
    enemy_speed_min: float = 1.0
    enemy_speed_max: float = 4.0
    enemy_radius_min: float = 1.0
    enemy_radius_max: float = 3.0
    enemy_cross_dist_ahead: float = 15.0   # spawn crossing this far ahead of drone
    enemy_cross_lateral: float = 8.0       # lateral offset for crossing trajectory

    # Sudden obstacle parameters
    obstacle_radius_min: float = 1.0
    obstacle_radius_max: float = 2.5
    obstacle_ahead_min: float = 8.0
    obstacle_ahead_max: float = 20.0
    obstacle_side_span: float  = 10.0

    # Lane block parameters (rectangle in XY)
    lane_w_min: float = 6.0
    lane_w_max: float = 12.0
    lane_h_min: float = 4.0
    lane_h_max: float = 10.0
    lane_ahead_min: float = 15.0
    lane_ahead_max: float = 35.0

    # Drone pose topic (for relative spawning)
    drone_pose_topic: str = "/model/drone1/pose/info"

    # Trial/phase control
    auto_start: bool = True               # start phases automatically
    warmup_s: float = 2.0
    phases: List[PhaseSpec] = field(default_factory=lambda: [
        PhaseSpec(duration_s=40.0, rate_hz=0.12, mix_enemy=0.6, mix_obstacle=0.3, mix_lane=0.1, burstiness=0.0),
        PhaseSpec(duration_s=40.0, rate_hz=0.20, mix_enemy=0.5, mix_obstacle=0.3, mix_lane=0.2, burstiness=0.3),
        PhaseSpec(duration_s=40.0, rate_hz=0.28, mix_enemy=0.5, mix_obstacle=0.2, mix_lane=0.3, burstiness=0.5),
    ])

    # Logging
    log_csv_path: Optional[str] = "events_log.csv"

    # Convenience builder from TeleopConfig
    @staticmethod
    def from_teleop_cfg(cfg: TeleopConfig) -> "EventCfg":
        # Optional tuples from cfg (provide sane fallbacks)
        bounds = getattr(cfg, "event_bounds_xy", (-100.0, 100.0, -50.0, 50.0))
        phases_cfg = getattr(cfg, "event_phases", None)  # list of dicts optional

        def _build_phases():
            if not phases_cfg:
                return EventCfg().phases
            phases: List[PhaseSpec] = []
            for p in phases_cfg:
                phases.append(PhaseSpec(
                    duration_s=float(p.get("duration_s", 40.0)),
                    rate_hz=float(p.get("rate_hz", 0.2)),
                    mix_enemy=float(p.get("mix_enemy", 0.5)),
                    mix_obstacle=float(p.get("mix_obstacle", 0.3)),
                    mix_lane=float(p.get("mix_lane", 0.2)),
                    burstiness=float(p.get("burstiness", 0.0)),
                ))
            return phases

        ec = EventCfg(
            topic=getattr(cfg, "event_topic", "/hydra/event"),
            seed=getattr(cfg, "event_seed", None),
            jitter_s=float(cfg.event_deadline_jitter_s),
            default_deadline_s=float(cfg.event_default_deadline_s),
            min_deadline_s=cfg.event_min_deadline_s,
            max_deadline_s=cfg.event_max_deadline_s,
            deadline_scale=float(cfg.event_deadline_scale), 
            enemy_deadline_s=getattr(cfg, "event_enemy_deadline_s", None),
            obstacle_deadline_s=getattr(cfg, "event_obstacle_deadline_s", None),
            lane_deadline_s=getattr(cfg, "event_lane_deadline_s", None),
            x_min=float(bounds[0]), x_max=float(bounds[1]),
            y_min=float(bounds[2]), y_max=float(bounds[3]),
            enemy_speed_min=float(getattr(cfg, "event_enemy_speed_min", 1.0)),
            enemy_speed_max=float(getattr(cfg, "event_enemy_speed_max", 4.0)),
            enemy_radius_min=float(getattr(cfg, "event_enemy_radius_min", 1.0)),
            enemy_radius_max=float(getattr(cfg, "event_enemy_radius_max", 3.0)),
            enemy_cross_dist_ahead=float(getattr(cfg, "event_enemy_cross_dist_ahead", 15.0)),
            enemy_cross_lateral=float(getattr(cfg, "event_enemy_cross_lateral", 8.0)),
            obstacle_radius_min=float(getattr(cfg, "event_obstacle_radius_min", 1.0)),
            obstacle_radius_max=float(getattr(cfg, "event_obstacle_radius_max", 2.5)),
            obstacle_ahead_min=float(getattr(cfg, "event_obstacle_ahead_min", 8.0)),
            obstacle_ahead_max=float(getattr(cfg, "event_obstacle_ahead_max", 20.0)),
            obstacle_side_span=float(getattr(cfg, "event_obstacle_side_span", 10.0)),
            lane_w_min=float(getattr(cfg, "event_lane_w_min", 6.0)),
            lane_w_max=float(getattr(cfg, "event_lane_w_max", 12.0)),
            lane_h_min=float(getattr(cfg, "event_lane_h_min", 4.0)),
            lane_h_max=float(getattr(cfg, "event_lane_h_max", 10.0)),
            lane_ahead_min=float(getattr(cfg, "event_lane_ahead_min", 15.0)),
            lane_ahead_max=float(getattr(cfg, "event_lane_ahead_max", 35.0)),
            drone_pose_topic=getattr(cfg, "ros_pose_topic", "/model/drone1/pose/info"),
            auto_start=bool(getattr(cfg, "event_auto_start", True)),
            warmup_s=float(getattr(cfg, "event_warmup_s", 2.0)),
            phases=_build_phases(),
            log_csv_path=getattr(cfg, "event_log_csv_path", "events_log.csv"),
        )
        ec.topic = getattr(cfg, "event_topic", ec.topic)
        return ec

# ----------------------------- Emitter Node -----------------------------

class EventEmitter(Node):
    """
    B-like experiment generator:
      • Phased Poisson arrivals with optional burstiness
      • Drone-relative enemy crossings & sudden obstacles
      • **Fairness**: constant, configurable deadlines (no TTC dependency)
      • Reproducible via seed
      • CSV logging
    """
    def __init__(self, teleop_cfg: TeleopConfig, gen_cfg: Optional[EventCfg] = None):
        super().__init__("hydra_event_emitter")
        self._cfg = gen_cfg or EventCfg.from_teleop_cfg(teleop_cfg)

        # RNG
        self._rnd = random.Random(self._cfg.seed)

        # Publisher
        self._pub = self.create_publisher(String, self._cfg.topic, 10)

        # Drone pose sub (for relative events)
        self._pose_lock = threading.Lock()
        self._pose_latest: Optional[PoseStamped] = None
        self.create_subscription(PoseStamped, self._cfg.drone_pose_topic, self._on_pose, 10)

        # Executor + spin thread
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._stop_flag = threading.Event()
        self._spin_thread: Optional[threading.Thread] = None

        # Experiment thread
        self._exp_thread: Optional[threading.Thread] = None

        # Logging
        self._csv_fp = None
        if self._cfg.log_csv_path:
            os.makedirs(os.path.dirname(self._cfg.log_csv_path) or ".", exist_ok=True)
            self._csv_fp = open(self._cfg.log_csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_fp)
            self._csv.writerow(["t_emit","kind","deadline_s","meta_json","phase_idx"])

    # ---- lifecycle ----
    def start(self):
        if self._spin_thread is None:
            self._stop_flag.clear()
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
        if self._cfg.auto_start and self._exp_thread is None:
            self._exp_thread = threading.Thread(target=self._experiment_loop, daemon=True)
            self._exp_thread.start()

    def stop(self):
        try:
            self._stop_flag.set()
            if self._exp_thread:
                self._exp_thread.join(timeout=1.0)
            if self._spin_thread:
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._csv_fp:
                self._csv_fp.flush(); self._csv_fp.close()
        except Exception:
            pass
        try:
            self._exec.remove_node(self)
            self.destroy_node()
        except Exception:
            pass

    def _spin_loop(self):
        try:
            while rclpy.ok() and not self._stop_flag.is_set():
                self._exec.spin_once(timeout_sec=0.05)
        except Exception:
            pass

    # ---- ROS callbacks ----
    def _on_pose(self, msg: PoseStamped):
        with self._pose_lock:
            self._pose_latest = msg

    def _get_pose_xyyaw(self) -> Optional[Tuple[float,float,float]]:
        with self._pose_lock:
            msg = self._pose_latest
        if not msg:
            return None
        p = msg.pose.position; o = msg.pose.orientation
        # yaw from quaternion
        t0 = +2.0 * (o.w * o.z + o.x * o.y)
        t1 = +1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(t0, t1)
        return p.x, p.y, yaw

    # --------------------- Core experiment ---------------------
    def _experiment_loop(self):
        # Warmup once
        t0 = time.time()
        while not self._stop_flag.is_set() and (time.time() - t0) < self._cfg.warmup_s:
            time.sleep(0.05)

        if not self._cfg.phases:
            self.get_logger().warn("[events] No phases configured; idling until stop().")
            while not self._stop_flag.is_set():
                time.sleep(0.25)
            self.get_logger().info("[events] Stopped (no phases).")
            return

        self.get_logger().info("[events] Event loop starting; cycling phases until stop() is called.")
        # Loop forever over phases until stop flag
        while not self._stop_flag.is_set():
            for phase_idx, ph in enumerate(self._cfg.phases):
                if self._stop_flag.is_set():
                    break

                self.get_logger().info(
                    f"[events] Phase {phase_idx} start: dur={ph.duration_s}s "
                    f"rate={ph.rate_hz}/s burst={ph.burstiness}"
                )

                t_phase_start = time.time()
                t = t_phase_start
                while not self._stop_flag.is_set() and (t - t_phase_start) < ph.duration_s:
                    # Draw inter-arrival from exponential with optional burstiness
                    lam = max(1e-6, ph.rate_hz)
                    dt = self._expovariate_bursty(lam, ph.burstiness)
                    time.sleep(dt)
                    t = time.time()
                    if self._stop_flag.is_set():
                        break

                    # Choose kind by phase mix
                    r = self._rnd.random()
                    if r < ph.mix_enemy:
                        self._emit_enemy_trial(phase_idx)
                    elif r < ph.mix_enemy + ph.mix_obstacle:
                        self._emit_obstacle_trial(phase_idx)
                    else:
                        self._emit_lane_trial(phase_idx)

        self.get_logger().info("[events] Stopped by stop().")

    def _expovariate_bursty(self, lam: float, burstiness: float) -> float:
        """Exponential inter-arrival with simple burstiness: shorten dt with prob ~burstiness."""
        base = self._rnd.expovariate(lam)
        if burstiness <= 1e-6:
            return base
        if self._rnd.random() < burstiness:
            return base * self._rnd.uniform(0.2, 0.6)  # burst cluster
        return base

    # --------------------- Deadline helper ---------------------
    def _pick_deadline(self, kind: str) -> float:
        """Fairness: constant deadlines (per-kind override -> default) * scale, then jitter+clamp."""
        if kind == "ENEMY" and self._cfg.enemy_deadline_s is not None:
            base = float(self._cfg.enemy_deadline_s)
        elif kind == "SUDDEN_OBSTACLE" and self._cfg.obstacle_deadline_s is not None:
            base = float(self._cfg.obstacle_deadline_s)
        elif kind == "LANE_BLOCK" and self._cfg.lane_deadline_s is not None:
            base = float(self._cfg.lane_deadline_s)
        else:
            base = self._cfg.default_deadline_s

        d = max(0.0, base * self._cfg.deadline_scale)

        # jitter
        if self._cfg.jitter_s > 0.0:
            d = d + self._rnd.uniform(-self._cfg.jitter_s, +self._cfg.jitter_s)
            d = max(0.0, d)

        # clamp
        d = min(max(d, self._cfg.min_deadline_s), self._cfg.max_deadline_s)
        return d

    # --------------------- Emitters (geometry realistic; deadline constant) ---------------------

    def _emit(self, kind: str, meta: Dict[str, Any], deadline_s: float, phase_idx: int):
        obj = {"kind": kind, "t_emit": time.time(), "deadline_s": deadline_s, "meta": meta}
        msg = String(); msg.data = json.dumps(obj)
        self._pub.publish(msg)
        if self._csv_fp:
            self._csv.writerow([obj["t_emit"], kind, deadline_s, json.dumps(meta), phase_idx])

    def _emit_enemy_trial(self, phase_idx: int):
        pose = self._get_pose_xyyaw()
        if pose is None:
            # fallback: arena-based random
            x = self._rnd.uniform(self._cfg.x_min, self._cfg.x_max)
            y = self._rnd.uniform(self._cfg.y_min, self._cfg.y_max)
            speed = self._rnd.uniform(self._cfg.enemy_speed_min, self._cfg.enemy_speed_max)
            ang = self._rnd.uniform(-math.pi, math.pi)
            vx, vy = speed*math.cos(ang), speed*math.sin(ang)
            rad = self._rnd.uniform(self._cfg.enemy_radius_min, self._cfg.enemy_radius_max)
            deadline_s = self._pick_deadline("ENEMY")
            self._emit("ENEMY", {"pos":[x,y],"vel":[vx,vy],"radius":rad}, deadline_s, phase_idx)
            return

        x, y, yaw = pose
        ahead = self._cfg.enemy_cross_dist_ahead
        lateral = self._cfg.enemy_cross_lateral * (1 if self._rnd.random()<0.5 else -1)
        cx = x + ahead*math.cos(yaw) - lateral*math.sin(yaw)
        cy = y + ahead*math.sin(yaw) + lateral*math.cos(yaw)

        speed = self._rnd.uniform(self._cfg.enemy_speed_min, self._cfg.enemy_speed_max)
        vx = -speed*math.sin(yaw)
        vy =  speed*math.cos(yaw)
        rad = self._rnd.uniform(self._cfg.enemy_radius_min, self._cfg.enemy_radius_max)

        deadline_s = self._pick_deadline("ENEMY")
        self._emit("ENEMY", {"pos":[cx,cy],"vel":[vx,vy],"radius":rad}, deadline_s, phase_idx)

    def _emit_obstacle_trial(self, phase_idx: int):
        pose = self._get_pose_xyyaw()
        if pose is None:
            x = self._rnd.uniform(self._cfg.x_min, self._cfg.x_max)
            y = self._rnd.uniform(self._cfg.y_min, self._cfg.y_max)
            rad = self._rnd.uniform(self._cfg.obstacle_radius_min, self._cfg.obstacle_radius_max)
            deadline_s = self._pick_deadline("SUDDEN_OBSTACLE")
            self._emit("SUDDEN_OBSTACLE", {"pos":[x,y], "radius":rad}, deadline_s, phase_idx)
            return

        x, y, yaw = pose
        ahead = self._rnd.uniform(self._cfg.obstacle_ahead_min, self._cfg.obstacle_ahead_max)
        side  = self._rnd.uniform(-self._cfg.obstacle_side_span, self._cfg.obstacle_side_span)
        ox = x + ahead*math.cos(yaw) - side*math.sin(yaw)
        oy = y + ahead*math.sin(yaw) + side*math.cos(yaw)
        rad = self._rnd.uniform(self._cfg.obstacle_radius_min, self._cfg.obstacle_radius_max)

        deadline_s = self._pick_deadline("SUDDEN_OBSTACLE")
        self._emit("SUDDEN_OBSTACLE", {"pos":[ox,oy], "radius":rad}, deadline_s, phase_idx)

    def _emit_lane_trial(self, phase_idx: int):
        pose = self._get_pose_xyyaw()
        if pose is None:
            cx = self._rnd.uniform(self._cfg.x_min, self._cfg.x_max)
            cy = self._rnd.uniform(self._cfg.y_min, self._cfg.y_max)
            w  = self._rnd.uniform(self._cfg.lane_w_min, self._cfg.lane_w_max)
            h  = self._rnd.uniform(self._cfg.lane_h_min, self._cfg.lane_h_max)
            deadline_s = self._pick_deadline("LANE_BLOCK")
            self._emit("LANE_BLOCK", {"rect":[cx,cy,w,h]}, deadline_s, phase_idx)
            return

        x, y, yaw = pose
        ahead = self._rnd.uniform(self._cfg.lane_ahead_min, self._cfg.lane_ahead_max)
        lateral = self._rnd.uniform(-0.5*(self._cfg.lane_w_max+4.0), 0.5*(self._cfg.lane_w_max+4.0))
        cx = x + ahead*math.cos(yaw) - lateral*math.sin(yaw)
        cy = y + ahead*math.sin(yaw) + lateral*math.cos(yaw)
        w  = self._rnd.uniform(self._cfg.lane_w_min, self._cfg.lane_w_max)
        h  = self._rnd.uniform(self._cfg.lane_h_min, self._cfg.lane_h_max)

        deadline_s = self._pick_deadline("LANE_BLOCK")
        self._emit("LANE_BLOCK", {"rect":[cx,cy,w,h]}, deadline_s, phase_idx)

    # --------------------- Manual API (still available) ---------------------

    def emit_enemy(self, pos_xy, vel_xy=(0.0,0.0), radius=2.0, deadline_s: Optional[float]=None, phase_idx:int=-1):
        dl = self._pick_deadline("ENEMY") if deadline_s is None else float(deadline_s)
        self._emit("ENEMY", {"pos":list(pos_xy),"vel":list(vel_xy),"radius":float(radius)}, dl, phase_idx)

    def emit_lane_block(self, rect_xywh, deadline_s: Optional[float]=None, phase_idx:int=-1):
        dl = self._pick_deadline("LANE_BLOCK") if deadline_s is None else float(deadline_s)
        self._emit("LANE_BLOCK", {"rect":list(rect_xywh)}, dl, phase_idx)

    def emit_sudden_obstacle(self, pos_xy, radius=1.5, deadline_s: Optional[float]=None, phase_idx:int=-1):
        dl = self._pick_deadline("SUDDEN_OBSTACLE") if deadline_s is None else float(deadline_s)
        self._emit("SUDDEN_OBSTACLE", {"pos":list(pos_xy), "radius":float(radius)}, dl, phase_idx)
