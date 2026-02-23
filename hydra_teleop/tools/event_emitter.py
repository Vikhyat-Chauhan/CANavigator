#!/usr/bin/env python3
# hydra_teleop/event_emitter.py — deterministic emission on flag

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
import json, time, threading, random, math, csv, os
from rclpy.node import Node
from std_msgs.msg import String
# Pose is only used in non-deterministic mode
from geometry_msgs.msg import PoseStamped

from hydra_teleop.config import TeleopConfig

# ----------------------------- Config -----------------------------
@dataclass
class EventCfg:
    # ---- The only two user-facing knobs ----
    seed: Optional[int] = None
    event_deterministic: bool = False

    # ---- Fixed defaults (not meant to be tuned) ----
    topic: str = field(default="/hydra/event", init=False)
    dt_min_s: float = field(default=0.02, init=False)   # log-uniform Δt min
    dt_max_s: float = field(default=4.0, init=False)    # log-uniform Δt max

    # Deadline model: deadline = clamp(α * Δt, [min, max])
    deadline_alpha: float = field(default=0.80, init=False)
    deadline_min_s: float = field(default=0.12, init=False)
    deadline_max_s: float = field(default=1.20, init=False)
    global_deadline_s: Optional[float] = field(default=None, init=False)

    # Event mix probabilities
    mix_enemy: float = field(default=0.33, init=False)
    mix_obstacle: float = field(default=0.33, init=False)
    mix_lane: float = field(default=0.34, init=False)

    # CSV log
    log_csv_path: Optional[str] = field(default="logs/events_log.csv", init=False)

    @staticmethod
    def from_teleop_cfg(cfg: TeleopConfig) -> "EventCfg":
        return EventCfg(
            seed=getattr(cfg, "event_seed", None),
            event_deterministic=bool(getattr(cfg, "event_deterministic", False)),
        )

# ----------------------------- Emitter -----------------------------
class EventEmitter(Node):
    """
    Event emitter.
    - If cfg.event_deterministic == True:
        * Emission timing, types, and deadlines are purely RNG-based from `seed`.
        * Ignores drone pose and uses a logical clock (t=0, t+=Δt).
        * Meta is minimal and stable: {"i": idx, "kind": "..."}.
    - If False (default behavior preserved):
        * Uses wall-clock deltas and optional pose-derived geometry.
    """

    def __init__(self, teleop_cfg: TeleopConfig, gen_cfg: Optional[EventCfg] = None):
        super().__init__("hydra_event_emitter")
        self._cfg = gen_cfg or EventCfg.from_teleop_cfg(teleop_cfg)
        self._rnd = random.Random(self._cfg.seed)

        self._pub = self.create_publisher(String, self._cfg.topic, 10)

        # Only subscribe to pose in non-deterministic (geometry) mode
        self._pose_lock = threading.Lock()
        self._pose_latest: Optional[PoseStamped] = None
        if not self._cfg.event_deterministic:
            pose_topic = getattr(teleop_cfg, "ros_pose_topic", "/model/drone1/pose/info")
            self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)

        self._stop_flag = threading.Event()
        self._exp_thread: Optional[threading.Thread] = None

        # Deterministic logical clock + index
        self._t_logical = 0.0
        self._i = 0

        # CSV logging
        self._csv_fp = None
        if self._cfg.log_csv_path:
            os.makedirs(os.path.dirname(self._cfg.log_csv_path) or ".", exist_ok=True)
            self._csv_fp = open(self._cfg.log_csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_fp)
            self._csv.writerow(["t_emit", "kind", "deadline_s", "meta_json"])

    # ---------------- Lifecycle ----------------
    def start(self):
        # no executor spin here; the app adds this node to its shared executor
        if self._exp_thread is None:
            self._stop_flag.clear()
            self._exp_thread = threading.Thread(target=self._loop, daemon=True)
            self._exp_thread.start()

    def stop(self, destroy: bool = True):
        self._stop_flag.set()
        if self._exp_thread:
            self._exp_thread.join()
        self._exp_thread = None

        if destroy:
            if self._csv_fp:
                try:
                    self._csv_fp.flush()
                finally:
                    self._csv_fp.close()
                self._csv_fp = None
            try:
                self.destroy_node()
            except Exception:
                pass

    def reset(self):
        """Deterministic reset: rewind RNG/clock/index; keep node & CSV open."""
        self.stop(destroy=False)
        self._rnd = random.Random(self._cfg.seed)
        self._t_logical = 0.0
        self._i = 0
        self._stop_flag.clear()
        self.start()

    # ---------------- Internals ----------------
    def _on_pose(self, msg: PoseStamped):
        with self._pose_lock:
            self._pose_latest = msg

    def _draw_dt(self) -> float:
        # Δt ~ log-uniform(dt_min, dt_max)
        return math.exp(self._rnd.uniform(math.log(self._cfg.dt_min_s), math.log(self._cfg.dt_max_s)))

    def _deadline_from_dt(self, delta_t: float) -> float:
        if self._cfg.global_deadline_s is not None:
            return float(self._cfg.global_deadline_s)
        d = self._cfg.deadline_alpha * float(delta_t)
        return max(self._cfg.deadline_min_s, min(self._cfg.deadline_max_s, d))

    def _choose_kind(self) -> str:
        r = self._rnd.random()
        if r < self._cfg.mix_enemy: return "ENEMY"
        if r < self._cfg.mix_enemy + self._cfg.mix_obstacle: return "SUDDEN_OBSTACLE"
        return "LANE_BLOCK"

    # ---------------- Main Loop ----------------
    def _loop(self):
        last_wall_t = time.time()
        while not self._stop_flag.is_set():
            dt = self._draw_dt()
            # Interruptible sleep: exits immediately when stop() sets the flag
            if self._stop_flag.wait(dt):
                break

            if self._cfg.event_deterministic:
                # Purely logical timing & deterministic delta
                self._t_logical += dt
                delta_t = dt
                t_emit = self._t_logical
                kind = self._choose_kind()
                meta = {"i": self._i, "kind": kind}
                self._i += 1
            else:
                # Wall-clock delta; kind/geometry still seeded but may use pose
                now = time.time()
                delta_t = now - last_wall_t
                last_wall_t = now
                t_emit = now
                kind = self._choose_kind()
                meta = self._make_meta_nondet(kind)

            # Emit
            deadline = self._deadline_from_dt(delta_t)
            obj = {"kind": kind, "t_emit": t_emit, "deadline_s": deadline, "meta": meta}
            msg = String(); msg.data = json.dumps(obj)
            # Guard publish in case shutdown raced ahead
            if self._stop_flag.is_set():
                break
            try:
                self._pub.publish(msg)
            except Exception as e:
                # Likely shutdown in progress; exit cleanly
                try:
                    self.get_logger().warn(f"publish aborted during shutdown: {e}")
                except Exception:
                    pass
                break
            if self._csv_fp:
                self._csv.writerow([obj["t_emit"], kind, obj["deadline_s"], json.dumps(meta)])

    # ---------------- Non-deterministic meta (kept minimal) ----------------
    def _make_meta_nondet(self, kind: str) -> Dict[str, Any]:
        if kind == "ENEMY":
            # Minimal stable-ish placeholder without strict geometry needs
            return {"speed": round(self._rnd.uniform(1.0, 4.0), 3)}
        if kind == "SUDDEN_OBSTACLE":
            return {"radius": round(self._rnd.uniform(1.0, 2.5), 3)}
        # LANE_BLOCK
        w = self._rnd.uniform(6.0, 12.0)
        h = self._rnd.uniform(4.0, 10.0)
        return {"rect_wh": [round(w, 3), round(h, 3)]}

def make_event_emitter_node(teleop_cfg: TeleopConfig, gen_cfg: Optional[EventCfg] = None,
                            callback_group=None) -> EventEmitter:
    """
    Construct the EventEmitter node (no spinning). Caller must:
      1) add this node to their shared executor, and
      2) call emitter.start() to begin emission thread.
    """
    node = EventEmitter(teleop_cfg, gen_cfg)
    if callback_group is not None:
        # assign the callback group to the subscription if present
        try:
            for sub in node.subscriptions:
                sub.callback_group = callback_group
        except Exception:
            pass
    return node


def add_event_emitter_to_executor(executor, teleop_cfg: TeleopConfig,
                                  gen_cfg: Optional[EventCfg] = None,
                                  callback_group=None) -> EventEmitter:
    """
    Convenience: make the emitter node and add it to the provided executor.
    Returns the node (call .start() to begin emitting).
    """
    node = make_event_emitter_node(teleop_cfg, gen_cfg, callback_group=callback_group)
    executor.add_node(node)
    return node