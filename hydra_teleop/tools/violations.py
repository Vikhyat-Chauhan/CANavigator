#!/usr/bin/env python3
# hydra_teleop/violations.py
# Deep-penetration NFZ violations (no ENTER/EXIT events, no padding):
# - Loads axis-aligned rectangles (cx, cy, w, h) from generated_nofly_meta.json
# - Logs ZONEVIOLATION once per visit when depth >= deep_margin_m for at least dwell_s
# - One-time LOADEDBOXES record at startup
#
# Tunables:
#   deep_margin_m : required depth inside the rectangle (meters) to count as "too deep"
#   dwell_s       : required continuous time spent deeper than deep_margin_m before violation
#   min_step_m    : ignore pose updates with near-zero motion (noise debounce)

import json, os, threading, math, time
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import logging

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
    Subscribes to drone pose, detects:
      - ZONEVIOLATION once per visit when depth_inside >= deep_margin_m for ≥ dwell_s
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

        # ROS params (constructor defaults are picked up if params not provided)
        self.declare_parameter("deep_margin_m", deep_margin_m)
        self.declare_parameter("dwell_s", dwell_s)
        self.declare_parameter("min_step_m", min_step_m)

        self._deep = max(0.0, float(self.get_parameter("deep_margin_m").value))
        self._dwell = max(0.0, float(self.get_parameter("dwell_s").value))
        self._min_step = max(0.0, float(self.get_parameter("min_step_m").value))

        self._rects = load_rects(meta_path)
        n = len(self._rects)

        # Per-rect state
        self._armed = [True] * n                 # fire once per visit (re-armed on exit)
        self._inside_prev = [False] * n          # previous inside status (no logs on transitions)
        self._deep_since: List[Optional[float]] = [None] * n  # time when we first got deep this visit

        # Global state
        self._prev_xy: Optional[Tuple[float, float]] = None

        # Subscribe
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 20)

        # Startup log
        self._logger.info(
            {
                "boxes": n,
                "deep_margin": round(self._deep, 3),
                "dwell_s": round(self._dwell, 3),
            },
            extra={"type": "LOADEDBOXES"},
        )

    # --------------------- Callbacks ----------------------
    def _on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        t_now = _now_from_msg_or_clock(self, msg)

        # Motion debounce (optional)
        if self._prev_xy is not None:
            if math.hypot(x - self._prev_xy[0], y - self._prev_xy[1]) < self._min_step:
                # Still update per-rect state below; this just avoids overreacting to jitter
                pass

        # Per-rect checks (no ENTER/EXIT logs)
        for i, (cx, cy, w, h) in enumerate(self._rects):
            now_in = point_in_rect(x, y, cx, cy, w, h)

            # On exit, re-arm and clear deep timer (but do not log)
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

            # If dwelled long enough and armed → fire once per visit
            if self._armed[i] and (t_now - self._deep_since[i] >= self._dwell):
                self._logger.info(
                    {
                        "rect_idx": i,
                        "rect_center": {"x": round(cx, 3), "y": round(cy, 3)},
                        "pose": {"x": round(x, 3), "y": round(y, 3)},
                        "depth_m": round(depth, 3),
                        "deep_margin_m": round(self._deep, 3),
                        "dwell_s": round(self._dwell, 3),
                    },
                    extra={"type": "ZONEVIOLATION"},
                )
                # Disarm until we EXIT this rectangle
                self._armed[i] = False

        # Update last pose
        self._prev_xy = (x, y)


def start_violation_monitor(
    pose_topic: str = "/model/drone1/pose/info",
    meta_path: str = META_PATH,
    deep_margin_m: float = 1.0,
    dwell_s: float = 0.15,
    min_step_m: float = 1e-3,
):
    """
    Start the ViolationMonitor in a background thread.
    Assumes rclpy.init() is called elsewhere (e.g., your main).
    Returns the node and the thread.
    """
    node = ViolationMonitor(
        pose_topic=pose_topic,
        meta_path=meta_path,
        deep_margin_m=deep_margin_m,
        dwell_s=dwell_s,
        min_step_m=min_step_m,
    )
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    return node, t
