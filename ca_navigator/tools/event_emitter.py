#!/usr/bin/env python3
# ca_navigator/event_emitter.py — deterministic emission on flag

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
import json, time, threading, random, math, csv, os
import rclpy.parameter
from rclpy.node import Node
from std_msgs.msg import String
# Pose is only used in non-deterministic mode
from geometry_msgs.msg import PoseStamped

from ca_navigator.config import TeleopConfig

# ----------------------------- Config -----------------------------
@dataclass
class EventCfg:
    # ---- The only two user-facing knobs ----
    seed: Optional[int] = None
    event_deterministic: bool = False

    # ---- Fixed defaults (not meant to be tuned) ----
    topic: str = field(default="/ca_navigator/event", init=False)
    dt_min_s: float = field(default=0.900, init=False)
    # Ensures minimum inter-arrival (900ms) > APE1 sleep (523ms) by 183ms,
    # guaranteeing APE1 resolves before any subsequent event arrives.
    # Closes the preemptive violation channel for APE1 by construction.
    dt_max_s: float = field(default=4.0,  init=False)  # log-uniform upper bound

    # Deadline model: deadline = clamp(α * Δt, [min, max])
    # Values sourced from TeleopConfig at construction time via from_teleop_cfg().
    # Defaults here match TeleopConfig so a bare EventCfg() is still self-consistent.
    #
    # deadline_min = 0.60s: APE1 budget (523ms) < 600ms → APE1 always resolves.
    # deadline_max = 3.50s: APE3 budget (2035ms) < 3500ms → APE3 resolves on
    #   long-deadline events (~30% of the distribution).
    # deadline_alpha = 0.85: deadlines span [600ms, 3500ms] with log-uniform dt
    #   in [0.900s, 4.0s]. Expected tier split (1M sample simulation):
    #     APE1 wins (0.52s < d ≤ 1.73s): ~61% of CA resolutions
    #     APE2 wins (1.73s < d ≤ 2.03s): ~9%  of CA resolutions
    #     APE3 wins (d > 2.03s):          ~30% of CA resolutions
    deadline_alpha: float = field(default=0.85, init=False)
    deadline_min_s: float = field(default=0.60, init=False)
    deadline_max_s: float = field(default=3.50, init=False)
    global_deadline_s: Optional[float] = field(default=None, init=False)

    # Event mix probabilities
    mix_enemy: float = field(default=0.33, init=False)
    mix_obstacle: float = field(default=0.33, init=False)
    mix_lane: float = field(default=0.34, init=False)

    # CSV log
    log_csv_path: Optional[str] = field(default="logs/events_log.csv", init=False)

    @staticmethod
    def from_teleop_cfg(cfg: TeleopConfig) -> "EventCfg":
        ecfg = EventCfg(
            seed=getattr(cfg, "event_seed", None),
            event_deterministic=bool(getattr(cfg, "event_deterministic", False)),
        )
        # Propagate deadline knobs from TeleopConfig so the emitter and selector
        # thresholds are always derived from the same source of truth.
        ecfg.deadline_alpha = float(getattr(cfg, "deadline_alpha", ecfg.deadline_alpha))
        ecfg.deadline_min_s = float(getattr(cfg, "deadline_min_s", ecfg.deadline_min_s))
        ecfg.deadline_max_s = float(getattr(cfg, "deadline_max_s", ecfg.deadline_max_s))
        return ecfg

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
        super().__init__(
            "can_event_emitter",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
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
        # Flush a sentinel null event so the navigator's _EventSub._pending
        # is cleared of any stale message from the previous run before new
        # events begin arriving. Without this, the first real event of the
        # new run collides with the buffered tail of the old run, causing a
        # ~1-4ms inter-arrival gap that triggers a preemptive violation.
        try:
            sentinel = json.dumps({"kind": "__RESET__", "deadline_s": 0.0, "t_emit": -1.0, "meta": {}})
            msg = String()
            msg.data = sentinel
            self._pub.publish(msg)
        except Exception:
            pass
        # -------------------- Restart emission loop with reset RNG/clock/index ------------------
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

    # ---------------- Sim-time helper ----------------
    def _sim_time(self) -> float:
        """Return sim time from the ROS clock (requires use_sim_time=True).
        Falls back to wall time before /clock messages arrive."""
        try:
            ns = self.get_clock().now().nanoseconds
            if ns > 0:
                return ns * 1e-9
        except Exception:
            pass
        return time.time()

    # ---------------- Main Loop ----------------
    def _loop(self):
        # Poll interval: short enough to not miss tight deadlines
        _POLL_S = 0.010

        if self._cfg.event_deterministic:
            # Sim-time threshold for next emission; initialise once clock is live
            _sim_t_next: Optional[float] = None
            _next_dt: float = self._draw_dt()

            while not self._stop_flag.is_set():
                if self._stop_flag.wait(_POLL_S):
                    break

                now_sim = self._sim_time()

                # Initialise threshold on the first valid sim-time tick
                if _sim_t_next is None:
                    _sim_t_next = now_sim + _next_dt
                    continue

                if now_sim < _sim_t_next:
                    continue

                # Fire event
                dt = _next_dt
                self._t_logical += dt
                t_emit = self._t_logical
                kind = self._choose_kind()
                meta = {"i": self._i, "kind": kind}
                self._i += 1
                deadline = self._deadline_from_dt(dt)

                # Schedule next
                _next_dt = self._draw_dt()
                _sim_t_next = now_sim + _next_dt

                obj = {"kind": kind, "t_emit": t_emit, "deadline_s": deadline, "meta": meta}
                msg = String(); msg.data = json.dumps(obj)
                if self._stop_flag.is_set():
                    break
                try:
                    self._pub.publish(msg)
                except Exception as e:
                    try:
                        self.get_logger().warn(f"publish aborted during shutdown: {e}")
                    except Exception:
                        pass
                    break
                if self._csv_fp:
                    try:
                        self._csv.writerow([t_emit, kind, deadline, json.dumps(meta)])
                    except Exception:
                        pass
            return  # deterministic path done

        else:
            # Non-deterministic: sim-time delta and t_emit
            last_sim_t = self._sim_time()
            dt = self._draw_dt()
            while not self._stop_flag.is_set():
                if self._stop_flag.wait(_POLL_S):
                    break

                now_sim = self._sim_time()
                if now_sim - last_sim_t < dt:
                    continue

                # Fire event
                delta_t = now_sim - last_sim_t
                last_sim_t = now_sim
                t_emit = now_sim
                kind = self._choose_kind()
                meta = self._make_meta_nondet(kind)
                deadline = self._deadline_from_dt(delta_t)

                # Draw next interval before publishing
                dt = self._draw_dt()

                obj = {"kind": kind, "t_emit": t_emit, "deadline_s": deadline, "meta": meta}
                msg = String(); msg.data = json.dumps(obj)
                if self._stop_flag.is_set():
                    break
                try:
                    self._pub.publish(msg)
                except Exception as e:
                    try:
                        self.get_logger().warn(f"publish aborted during shutdown: {e}")
                    except Exception:
                        pass
                    break
                if self._csv_fp:
                    try:
                        self._csv.writerow([t_emit, kind, deadline, json.dumps(meta)])
                    except Exception:
                        pass

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

def add_event_emitter_to_executor(executor, teleop_cfg: TeleopConfig,
                                  gen_cfg: Optional[EventCfg] = None,
                                  callback_group=None) -> EventEmitter:
    """Construct the EventEmitter, attach to executor, and return it.
    Call .start() to begin emission."""
    node = EventEmitter(teleop_cfg, gen_cfg)
    if callback_group is not None:
        try:
            for sub in node.subscriptions:
                sub.callback_group = callback_group
        except Exception:
            pass
    executor.add_node(node)
    return node