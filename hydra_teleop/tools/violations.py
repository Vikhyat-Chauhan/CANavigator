#!/usr/bin/env python3
# hydra_teleop/violations.py
# Simple NFZ visit-based violations (QUIET mode):
# - Loads axis-aligned rectangles (cx, cy, w, h) from generated_nofly_meta.json
# - Counts exactly one ZONEVIOLATION per visit (i.e., on first ENTER after being outside)
# - No depth/dwell thresholds; no per-event logs during flight
# - Emits a single VIOLATIONSUMMARY only when log_and_reset() is called
# - One-time LOADEDBOXES record at startup

import json, os
from typing import List, Tuple, Optional, Dict, Any
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


def point_in_rect(x: float, y: float, cx: float, cy: float, w: float, h: float) -> bool:
    """Axis-aligned membership (no padding)."""
    half_w = max(0.0, w * 0.5)
    half_h = max(0.0, h * 0.5)
    return (cx - half_w) <= x <= (cx + half_w) and (cy - half_h) <= y <= (cy + half_h)


class ViolationMonitor(Node):
    """
    Subscribes to drone pose, tracks:
      - exactly one violation per visit (first ENTER after being outside) per rectangle

    QUIET mode:
      - No per-event logs during flight.
      - Call log_and_reset() to emit a VIOLATIONSUMMARY and clear counters for the next run.
    """

    def __init__(
        self,
        pose_topic: str = "/model/drone1/pose/info",
        meta_path: str = META_PATH,
    ):
        super().__init__("violation_monitor")

        self._logger = logging.getLogger(__name__)
        self._lock = Lock()

        self._rects = load_rects(meta_path)
        n = len(self._rects)

        # Per-rect visit state
        # armed[i] == True  → next ENTER will count a violation and disarm
        # becomes re-armed only after EXIT (i.e., when we go from inside→outside)
        self._armed: List[bool] = [True] * n
        self._inside_prev: List[bool] = [False] * n

        # Per-rect counters (quiet accumulation)
        self._violations_per_rect: List[int] = [0] * n

        # Global state
        self._total_violations: int = 0
        self._segment_label: str = "run"
        self._segment_start_wall: float = 0.0
        self._last_pose: Tuple[float, float] = (0.0, 0.0)

        # Subscribe
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 20)

        # Startup log (one-time)
        self._logger.info(
            {
                "boxes": n,
                "mode": "quiet(log on demand)",
                "rule": "one_violation_per_visit",
            },
            extra={"type": "LOADEDBOXES"},
        )

    # --------------------- Callbacks ----------------------
    def _on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y

        with self._lock:
            # Visit logic: count exactly once on ENTER per rectangle
            for i, (cx, cy, w, h) in enumerate(self._rects):
                now_in = point_in_rect(x, y, cx, cy, w, h)

                was_in = self._inside_prev[i]
                self._inside_prev[i] = now_in

                # EXIT → re-arm
                if was_in and not now_in:
                    self._armed[i] = True
                    continue

                # ENTER while armed → count violation once, disarm until next EXIT
                if not was_in and now_in and self._armed[i]:
                    self._violations_per_rect[i] += 1
                    self._total_violations += 1
                    self._armed[i] = False

            self._last_pose = (x, y)

    # --------------------- Public controls ----------------------
    def mark_run_start(self, label: str = "run"):
        """
        Reset accumulators and visit-state without logging. Use at the start of a new run.
        """
        with self._lock:
            self._segment_label = label
            self._segment_start_wall = self.get_clock().now().nanoseconds * 1e-9
            self._total_violations = 0
            for i in range(len(self._violations_per_rect)):
                self._violations_per_rect[i] = 0

            # Clear visit state so a run boundary doesn't inherit a partial visit.
            n = len(self._rects)
            self._armed = [True] * n
            self._inside_prev = [False] * n

    def _build_summary(self, label: str, include_boxes: bool) -> Dict[str, Any]:
        summary: Dict[str, Any] = {
            "label": label,
            "wall_started_at": round(self._segment_start_wall, 3),
            "total_violations": int(self._total_violations),
            "last_pose": {"x": round(self._last_pose[0], 3), "y": round(self._last_pose[1], 3)},
            "rule": "one_violation_per_visit",
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
            self._segment_start_wall = self.get_clock().now().nanoseconds * 1e-9
            self._total_violations = 0
            for i in range(len(self._violations_per_rect)):
                self._violations_per_rect[i] = 0
            n = len(self._rects)
            self._armed = [True] * n
            self._inside_prev = [False] * n

            return summary


def start_violation_monitor(
    pose_topic: str = "/model/drone1/pose/info",
    meta_path: str = META_PATH,
    callback_group=None,
):
    """
    Create (but DO NOT SPIN) the ViolationMonitor node.
    Add the returned node to a shared MultiThreadedExecutor; do not call rclpy.spin on it.
    Returns (node, None) for backwards compatibility.
    """
    node = ViolationMonitor(pose_topic=pose_topic, meta_path=meta_path)
    if callback_group is not None:
        try:
            node._sub.callback_group = callback_group  # type: ignore[attr-defined]
        except Exception:
            pass
    return node, None


def add_violation_monitor_to_executor(
    executor,
    pose_topic: str = "/model/drone1/pose/info",
    meta_path: str = META_PATH,
    callback_group=None,
):
    """Convenience helper: construct the node and add it to the provided executor."""
    node, _ = start_violation_monitor(
        pose_topic=pose_topic,
        meta_path=meta_path,
        callback_group=callback_group,
    )
    executor.add_node(node)
    return node
