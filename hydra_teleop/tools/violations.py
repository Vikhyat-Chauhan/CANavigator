#!/usr/bin/env python3
# hydra_teleop/violations.py
# Deep-penetration NFZ violations (QUIET mode: logs only on demand):
# - Loads axis-aligned rectangles (cx, cy, w, h) from generated_nofly_meta.json
# - Counts a ZONEVIOLATION once per visit when depth >= deep_margin_m for at least dwell_s
# - Does NOT log per-event during flight
# - Emits a single VIOLATIONSUMMARY only when log_and_reset() is called
# - One-time LOADEDBOXES record at startup
#
# Tunables:
#   deep_margin_m : required depth inside the rectangle (meters) to count as "too deep"
#   dwell_s       : required continuous time spent deeper than deep_margin_m before violation
#   min_step_m    : ignore pose updates with near-zero motion (noise debounce)

import json, os, threading, math, time
from typing import List, Tuple, Optional, Dict, Any
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import logging
from threading import Lock

META_PATH = os.path.join("models", "generated", "generated_nofly_meta.json")


def load_rects(meta_path: str) -> List[Tuple[float, float, float, float]]:
    """Return list of rectangles as (cx, cy, w, h) from meta JSON."""
    with open(meta_path, "r") as f:
        meta = json.load(f)
    rects = meta.get("rectangles_xywh", [])
    return [tuple(map(float, r)) for r in rects]


def _now_from_msg_or_clock(node: Node, msg: Optional[PoseStamped]) -> float:
    """Seconds (float). Prefer msg.header.stamp; fallback to ROS clock; then time.time()."""
    try:
        if msg is not None and msg.header.stamp:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except Exception:
        pass
    try:
        return node.get_clock().now().nanoseconds * 1e-9
    except Exception:
        return time.time()


def point_in_rect(x: float, y: float, cx: float, cy: float, w: float, h: float) -> bool:
    """Axis-aligned membership (no padding)."""
    half_w = max(0.0, w * 0.5)
    half_h = max(0.0, h * 0.5)
    return (cx - half_w) <= x <= (cx + half_w) and (cy - half_h) <= y <= (cy + half_h)


def rect_depth_inside(x: float, y: float, cx: float, cy: float, w: float, h: float) -> float:
    """
    Signed "depth" (meters) relative to rectangle boundary (no padding).
      > 0  : inside; value = min distance to the nearest side
      = 0  : exactly on boundary
      < 0  : outside; magnitude = how far to cross the boundary
    """
    half_w = max(0.0, w * 0.5)
    half_h = max(0.0, h * 0.5)
    dx = half_w - abs(x - cx)
    dy = half_h - abs(y - cy)
    return min(dx, dy)


class ViolationMonitor(Node):
    """
    Subscribes to drone pose, tracks:
      - a once-per-visit deep violation when depth_inside >= deep_margin_m for ≥ dwell_s

    QUIET mode:
      - No per-event logs during flight.
      - Call log_and_reset() to emit a VIOLATIONSUMMARY and clear counters for the next run.
    """

    def __init__(
        self,
        pose_topic: str = "/model/drone1/pose/info",
        meta_path: str = META_PATH,
        deep_margin_m: float = 1.0,     # how deep into the rect to count as violation
        dwell_s: float = 0.15,          # must remain deeper than deep_margin_m for this duration
        min_step_m: float = 1e-3,       # ignore tiny jitter steps
    ):
        super().__init__("violation_monitor")

        # Python logger (uses your existing logging format/handlers)
        self._logger = logging.getLogger(__name__)
        self._lock = Lock()

        # ROS params (constructor defaults are picked up if params not provided)
        self.declare_parameter("deep_margin_m", deep_margin_m)
        self.declare_parameter("dwell_s", dwell_s)
        self.declare_parameter("min_step_m", min_step_m)

        self._deep = max(0.0, float(self.get_parameter("deep_margin_m").value))
        self._dwell = max(0.0, float(self.get_parameter("dwell_s").value))
        self._min_step = max(0.0, float(self.get_parameter("min_step_m").value))

        self._rects = load_rects(meta_path)
        n = len(self._rects)

        # Per-rect state for detection (visit logic)
        self._armed = [True] * n                  # fire once per visit (re-armed on exit)
        self._inside_prev = [False] * n           # previous inside status
        self._deep_since: List[Optional[float]] = [None] * n  # time when we first got deep this visit

        # Per-rect counters (quiet accumulation)
        self._violations_per_rect: List[int] = [0] * n

        # Global state
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._total_violations: int = 0
        self._segment_label: str = "run"
        self._segment_start_wall: float = time.time()
        self._last_pose: Tuple[float, float] = (0.0, 0.0)

        # Subscribe
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 20)

        # Startup log (one-time)
        self._logger.info(
            {
                "boxes": n,
                "deep_margin": round(self._deep, 3),
                "dwell_s": round(self._dwell, 3),
                "mode": "quiet(log on demand)",
            },
            extra={"type": "LOADEDBOXES"},
        )

    # --------------------- Callbacks ----------------------
    def _on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        t_now = _now_from_msg_or_clock(self, msg)

        with self._lock:
            # Motion debounce (optional; still updates state)
            if self._prev_xy is not None:
                if math.hypot(x - self._prev_xy[0], y - self._prev_xy[1]) < self._min_step:
                    pass

            # Per-rect checks (QUIET: no ENTER/EXIT or per-event logs)
            for i, (cx, cy, w, h) in enumerate(self._rects):
                now_in = point_in_rect(x, y, cx, cy, w, h)

                # On exit, re-arm and clear deep timer (quiet)
                if self._inside_prev[i] and not now_in:
                    self._armed[i] = True
                    self._deep_since[i] = None
                    self._inside_prev[i] = False
                    continue

                self._inside_prev[i] = now_in

                # Only consider when inside
                if not now_in:
                    self._deep_since[i] = None
                    continue

                # Depth relative to rectangle boundary
                depth = rect_depth_inside(x, y, cx, cy, w, h)

                # Not deep enough: reset deep timer
                if depth < self._deep:
                    self._deep_since[i] = None
                    continue

                # Deep enough: start/continue dwell timer
                if self._deep_since[i] is None:
                    self._deep_since[i] = t_now

                # If dwelled long enough and armed → record once per visit (QUIET: no log)
                if self._armed[i] and (t_now - self._deep_since[i] >= self._dwell):
                    self._violations_per_rect[i] += 1
                    self._total_violations += 1
                    self._armed[i] = False  # disarm until EXIT

            # Update last pose
            self._prev_xy = (x, y)
            self._last_pose = (x, y)

    # --------------------- Public controls ----------------------
    def mark_run_start(self, label: str = "run"):
        """
        Reset accumulators and visit-state without logging. Use at the start of a new run.
        """
        with self._lock:
            self._segment_label = label
            self._segment_start_wall = time.time()
            self._total_violations = 0
            for i in range(len(self._violations_per_rect)):
                self._violations_per_rect[i] = 0

            # Clear visit state so a run boundary doesn't inherit a partial visit
            n = len(self._rects)
            self._armed = [True] * n
            self._inside_prev = [False] * n
            self._deep_since = [None] * n

    def _build_summary(self, label: str, include_boxes: bool) -> Dict[str, Any]:
        summary: Dict[str, Any] = {
            "label": label,
            "wall_started_at": round(self._segment_start_wall, 3),
            "total_violations": int(self._total_violations),
            "last_pose": {"x": round(self._last_pose[0], 3), "y": round(self._last_pose[1], 3)},
            "deep_margin_m": round(self._deep, 3),
            "dwell_s": round(self._dwell, 3),
        }
        if include_boxes:
            per_box = []
            for i, (cx, cy, w, h) in enumerate(self._rects):
                per_box.append({
                    "rect_idx": i,
                    "center": {"x": round(cx, 3), "y": round(cy, 3)},
                    "w": round(w, 3),
                    "h": round(h, 3),
                    "violations": int(self._violations_per_rect[i]),
                })
            summary["per_box"] = per_box
        return summary

    def log_and_reset(self, label: Optional[str] = None, include_boxes: bool = False) -> Dict[str, Any]:
        """
        Emit a single VIOLATIONSUMMARY log with totals since last reset/mark_run_start, then reset.
        Returns the summary dict for programmatic use.
        """
        with self._lock:
            lbl = label if label is not None else self._segment_label
            summary = self._build_summary(lbl, include_boxes)

            # Single log line with summary
            self._logger.info(summary, extra={"type": "VIOLATIONSUMMARY"})

            # Reset for next run (same behavior as mark_run_start but keep label default)
            self._segment_label = "run"
            self._segment_start_wall = time.time()
            self._total_violations = 0
            for i in range(len(self._violations_per_rect)):
                self._violations_per_rect[i] = 0
            n = len(self._rects)
            self._armed = [True] * n
            self._inside_prev = [False] * n
            self._deep_since = [None] * n

            return summary


def start_violation_monitor(
    pose_topic: str = "/model/drone1/pose/info",
    meta_path: str = META_PATH,
    deep_margin_m: float = 1.0,
    dwell_s: float = 0.15,
    min_step_m: float = 1e-3,
    callback_group=None,
):
    """
    Create (but DO NOT SPIN) the ViolationMonitor node.

    Returns:
        (node, None)  # thread is None to maintain backward compatibility

    Notes:
      - Add this node to your shared executor (MultiThreadedExecutor).
      - Do not call rclpy.spin on this node anywhere else.
    """
    node = ViolationMonitor(
        pose_topic=pose_topic,
        meta_path=meta_path,
        deep_margin_m=deep_margin_m,
        dwell_s=dwell_s,
        min_step_m=min_step_m,
    )
    if callback_group is not None:
        # If the caller passes a callback group, assign it to the subscription
        try:
            node._sub.callback_group = callback_group  # type: ignore[attr-defined]
        except Exception:
            pass
    return node, None


def add_violation_monitor_to_executor(
    executor,
    pose_topic: str = "/model/drone1/pose/info",
    meta_path: str = META_PATH,
    deep_margin_m: float = 1.0,
    dwell_s: float = 0.15,
    min_step_m: float = 1e-3,
    callback_group=None,
):
    """
    Convenience helper: construct the node and add it to the provided executor.
    Returns the created node.
    """
    node, _ = start_violation_monitor(
        pose_topic=pose_topic,
        meta_path=meta_path,
        deep_margin_m=deep_margin_m,
        dwell_s=dwell_s,
        min_step_m=min_step_m,
        callback_group=callback_group,
    )
    executor.add_node(node)
    return node
