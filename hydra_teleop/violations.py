#!/usr/bin/env python3
# hydra_teleop/violations.py
# Violation checker: subscribe to PoseStamped, count entries into red boxes.
# Minimal change: allow stricter zones via 'zone_padding_m' (meters).

import json, os, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

META_PATH = os.path.join("models", "generated", "generated_nofly_meta.json")


def load_rects(meta_path):
    """Load list of (cx, cy, w, h) rectangles from JSON metadata."""
    with open(meta_path) as f:
        meta = json.load(f)
    return [tuple(map(float, r)) for r in meta.get("rectangles_xywh", [])]


def inside_rect(x, y, cx, cy, w, h, pad=0.0):
    """Return True if (x,y) is inside rectangle centered (cx,cy) with size (w,h),
       inflated by 'pad' meters on each side (total +2*pad)."""
    w_eff = max(0.0, w + 2.0 * pad)
    h_eff = max(0.0, h + 2.0 * pad)
    return (abs(x - cx) <= w_eff / 2.0) and (abs(y - cy) <= h_eff / 2.0)


class ViolationCounter(Node):
    def __init__(self, pose_topic="/model/drone1/pose/info", meta_path=META_PATH, zone_padding_m=0.0):
        super().__init__("violation_counter")

        # Allow tuning via ROS param or constructor arg (arg wins if set).
        self.declare_parameter("zone_padding_m", zone_padding_m)
        # Clamp to non-negative; stricter only.
        self._pad = max(0.0, float(self.get_parameter("zone_padding_m").value))

        self._rects = load_rects(meta_path)
        self._violations = 0
        self._inside_any = False

        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self.get_logger().info(
            f"[violations] Loaded {len(self._rects)} boxes. zone_padding_m={self._pad:.3f} m"
        )

    def _on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        in_zone = any(inside_rect(x, y, cx, cy, w, h, self._pad) for (cx, cy, w, h) in self._rects)

        # detect new entry
        if in_zone and not self._inside_any:
            self._violations += 1
            self.get_logger().warn(
                f"Violation {self._violations}: entered restricted zone at ({x:.2f}, {y:.2f})"
            )

        self._inside_any = in_zone

    def get_total(self) -> int:
        """Return total violations so far (does not reset)."""
        return self._violations

    def reset(self):
        """Reset violation counter to zero."""
        self._violations = 0


def start_violation_monitor(pose_topic="/model/drone1/pose/info", meta_path=META_PATH, zone_padding_m=0.0):
    """
    Start the ViolationCounter in a background thread.
    Assumes rclpy.init() has already been called elsewhere.

    Examples:
      # programmatic
      node = start_violation_monitor(zone_padding_m=0.5)

      # via ROS params (if you expose as an executable):
      # ros2 run hydra_teleop violations --ros-args -p zone_padding_m:=0.5
    """
    node = ViolationCounter(pose_topic=pose_topic, meta_path=meta_path, zone_padding_m=zone_padding_m)
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    return node
