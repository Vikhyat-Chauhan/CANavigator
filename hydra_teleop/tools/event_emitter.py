#!/usr/bin/env python3
# hydra_teleop/event_emitter.py (B-style)
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple
import json, time, threading, random, math, csv, os

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from hydra_teleop.config import TeleopConfig

# ----------------------------- Config -----------------------------
@dataclass
class EventCfg:
    topic: str = "/hydra/event"
    seed: Optional[int] = None

    # Timing parameters (log-uniform inter-arrival)
    dt_min_s: float = 0.02
    dt_max_s: float = 4.0

    # Optional fixed global deadline. If None, we derive per-event deadlines from Δt.
    global_deadline_s: Optional[float] = None
    # B-style: deadline = clamp( α * Δt , [deadline_min_s, deadline_max_s] )
    deadline_alpha: float = 0.85
    deadline_min_s: float = 0.12
    deadline_max_s: float = 1.2

    # Event mix probabilities
    mix_enemy: float = 0.33
    mix_obstacle: float = 0.33
    mix_lane: float = 0.34

    # Arena bounds
    x_min: float = -100.0; x_max: float = 100.0
    y_min: float =  -50.0; y_max: float =   50.0

    # Drone pose topic
    drone_pose_topic: str = "/model/drone1/pose/info"

    # Geometry parameters (kept identical)
    enemy_speed_min: float = 1.0
    enemy_speed_max: float = 4.0
    enemy_radius_min: float = 1.0
    enemy_radius_max: float = 3.0
    enemy_cross_dist_ahead: float = 15.0
    enemy_cross_lateral: float = 8.0

    obstacle_radius_min: float = 1.0
    obstacle_radius_max: float = 2.5
    obstacle_ahead_min: float = 8.0
    obstacle_ahead_max: float = 20.0
    obstacle_side_span: float = 10.0

    lane_w_min: float = 6.0
    lane_w_max: float = 12.0
    lane_h_min: float = 4.0
    lane_h_max: float = 10.0
    lane_ahead_min: float = 15.0
    lane_ahead_max: float = 35.0

    # Logging
    log_csv_path: Optional[str] = "events_log.csv"

    @staticmethod
    def from_teleop_cfg(cfg: TeleopConfig) -> "EventCfg":
        bounds = getattr(cfg, "event_bounds_xy", (-100.0, 100.0, -50.0, 50.0))
        return EventCfg(
            topic=getattr(cfg, "event_topic", "/hydra/event"),
            seed=getattr(cfg, "event_seed", None),
            dt_min_s=float(getattr(cfg, "event_dt_min_s", 0.02)),
            dt_max_s=float(getattr(cfg, "event_dt_max_s", 4.0)),
            global_deadline_s=getattr(cfg, "event_global_deadline_s", None),
            deadline_alpha=float(getattr(cfg, "deadline_alpha", 0.85)),
            deadline_min_s=float(getattr(cfg, "deadline_min_s", 0.12)),
            deadline_max_s=float(getattr(cfg, "deadline_max_s", 1.20)),
            mix_enemy=float(getattr(cfg, "event_mix_enemy", 0.33)),
            mix_obstacle=float(getattr(cfg, "event_mix_obstacle", 0.33)),
            mix_lane=float(getattr(cfg, "event_mix_lane", 0.34)),
            x_min=float(bounds[0]), x_max=float(bounds[1]),
            y_min=float(bounds[2]), y_max=float(bounds[3]),
            drone_pose_topic=getattr(cfg, "ros_pose_topic", "/model/drone1/pose/info"),
            log_csv_path=getattr(cfg, "event_log_csv_path", "events_log.csv"),
        )

# ----------------------------- Emitter -----------------------------
class EventEmitter(Node):
    """B-style event emitter: log-uniform inter-arrival Δt defines constraint pressure."""

    def __init__(self, teleop_cfg: TeleopConfig, gen_cfg: Optional[EventCfg] = None):
        super().__init__("hydra_event_emitter")
        self._cfg = gen_cfg or EventCfg.from_teleop_cfg(teleop_cfg)
        self._rnd = random.Random(self._cfg.seed)

        self._pub = self.create_publisher(String, self._cfg.topic, 10)
        self._pose_lock = threading.Lock()
        self._pose_latest: Optional[PoseStamped] = None
        self.create_subscription(PoseStamped, self._cfg.drone_pose_topic, self._on_pose, 10)

        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._stop_flag = threading.Event()
        self._spin_thread: Optional[threading.Thread] = None
        self._exp_thread: Optional[threading.Thread] = None
        self._stopped = False

        # CSV logging setup
        self._csv_fp = None
        if self._cfg.log_csv_path:
            os.makedirs(os.path.dirname(self._cfg.log_csv_path) or ".", exist_ok=True)
            self._csv_fp = open(self._cfg.log_csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_fp)
            self._csv.writerow(["t_emit", "kind", "delta_t_s", "deadline_s", "meta_json"])

    # ---------------- Lifecycle ----------------
    def start(self):
        if self._spin_thread is None:
            self._stop_flag.clear()
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
        if self._exp_thread is None:
            self._exp_thread = threading.Thread(target=self._loop, daemon=True)
            self._exp_thread.start()

    def stop(self):
        self._stop_flag.set()
        if self._exp_thread: self._exp_thread.join(timeout=1.0)
        if self._spin_thread: self._spin_thread.join(timeout=1.0)
        if self._csv_fp:
            self._csv_fp.flush(); self._csv_fp.close()
            self._csv_fp.flush()
        self._exec.remove_node(self)
        self.destroy_node()

    def _spin_loop(self):
        while rclpy.ok() and not self._stop_flag.is_set():
            self._exec.spin_once(timeout_sec=0.01)

    def _on_pose(self, msg: PoseStamped):
        with self._pose_lock:
            self._pose_latest = msg

    def _get_pose_xyyaw(self) -> Optional[Tuple[float, float, float]]:
        with self._pose_lock:
            msg = self._pose_latest
        if not msg: return None
        p = msg.pose.position; o = msg.pose.orientation
        t0 = +2.0 * (o.w * o.z + o.x * o.y)
        t1 = +1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(t0, t1)
        return p.x, p.y, yaw

    # ---------------- Main Loop ----------------
    def _loop(self):
        last_emit_t = time.time()
        while not self._stop_flag.is_set():
            # Wait Δt ~ log-uniform(dt_min, dt_max)
            dt = math.exp(self._rnd.uniform(math.log(self._cfg.dt_min_s), math.log(self._cfg.dt_max_s)))
            time.sleep(dt)

            now = time.time()
            delta_t = now - last_emit_t
            last_emit_t = now

            # Choose event kind
            r = self._rnd.random()
            if r < self._cfg.mix_enemy:
                self._emit_enemy(delta_t)
            elif r < self._cfg.mix_enemy + self._cfg.mix_obstacle:
                self._emit_obstacle(delta_t)
            else:
                self._emit_lane(delta_t)

    # ---------------- Emitters ----------------
    def _emit(self, kind: str, meta: Dict[str, Any], delta_t: float):
        # If a fixed global deadline is not provided, derive it from Δt (pressure model)
        derived_deadline = (
            self._cfg.global_deadline_s
            if self._cfg.global_deadline_s is not None
            else max(self._cfg.deadline_min_s,
                     min(self._cfg.deadline_max_s, self._cfg.deadline_alpha * float(delta_t)))
        )
        obj = {
            "kind": kind,
            "t_emit": time.time(),
            "delta_t_s": delta_t,
            "deadline_s": derived_deadline,
            "meta": meta,
        }
        msg = String(); msg.data = json.dumps(obj)
        self._pub.publish(msg)
        if self._csv_fp:
            self._csv.writerow([obj["t_emit"], kind, delta_t, obj["deadline_s"], json.dumps(meta)])

    def _emit_enemy(self, delta_t: float):
        pose = self._get_pose_xyyaw()
        if pose:
            x, y, yaw = pose
            ahead = self._cfg.enemy_cross_dist_ahead
            lateral = self._cfg.enemy_cross_lateral * (1 if self._rnd.random() < 0.5 else -1)
            cx = x + ahead * math.cos(yaw) - lateral * math.sin(yaw)
            cy = y + ahead * math.sin(yaw) + lateral * math.cos(yaw)
        else:
            cx = self._rnd.uniform(self._cfg.x_min, self._cfg.x_max)
            cy = self._rnd.uniform(self._cfg.y_min, self._cfg.y_max)
        speed = self._rnd.uniform(self._cfg.enemy_speed_min, self._cfg.enemy_speed_max)
        vx, vy = -speed * math.sin(yaw if pose else 0), speed * math.cos(yaw if pose else 0)
        rad = self._rnd.uniform(self._cfg.enemy_radius_min, self._cfg.enemy_radius_max)
        self._emit("ENEMY", {"pos": [cx, cy], "vel": [vx, vy], "radius": rad}, delta_t)

    def _emit_obstacle(self, delta_t: float):
        pose = self._get_pose_xyyaw()
        if pose:
            x, y, yaw = pose
            ahead = self._rnd.uniform(self._cfg.obstacle_ahead_min, self._cfg.obstacle_ahead_max)
            side = self._rnd.uniform(-self._cfg.obstacle_side_span, self._cfg.obstacle_side_span)
            ox = x + ahead * math.cos(yaw) - side * math.sin(yaw)
            oy = y + ahead * math.sin(yaw) + side * math.cos(yaw)
        else:
            ox = self._rnd.uniform(self._cfg.x_min, self._cfg.x_max)
            oy = self._rnd.uniform(self._cfg.y_min, self._cfg.y_max)
        rad = self._rnd.uniform(self._cfg.obstacle_radius_min, self._cfg.obstacle_radius_max)
        self._emit("SUDDEN_OBSTACLE", {"pos": [ox, oy], "radius": rad}, delta_t)

    def _emit_lane(self, delta_t: float):
        pose = self._get_pose_xyyaw()
        if pose:
            x, y, yaw = pose
            ahead = self._rnd.uniform(self._cfg.lane_ahead_min, self._cfg.lane_ahead_max)
            lateral = self._rnd.uniform(-0.5 * (self._cfg.lane_w_max + 4.0), 0.5 * (self._cfg.lane_w_max + 4.0))
            cx = x + ahead * math.cos(yaw) - lateral * math.sin(yaw)
            cy = y + ahead * math.sin(yaw) + lateral * math.cos(yaw)
        else:
            cx = self._rnd.uniform(self._cfg.x_min, self._cfg.x_max)
            cy = self._rnd.uniform(self._cfg.y_min, self._cfg.y_max)
        w = self._rnd.uniform(self._cfg.lane_w_min, self._cfg.lane_w_max)
        h = self._rnd.uniform(self._cfg.lane_h_min, self._cfg.lane_h_max)
        self._emit("LANE_BLOCK", {"rect": [cx, cy, w, h]}, delta_t)
