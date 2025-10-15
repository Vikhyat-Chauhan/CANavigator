#!/usr/bin/env python3
# hydra_teleop/violations.py
# Violation checker: adjustable padding and corner exclusion (rounded corners).
# Counts a violation when inside a padded rectangle BUT NOT within corner_margin_m
# of any rectangle corner (reduces corner-graze false positives).

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


def inside_rect_rounded(x, y, cx, cy, w, h, pad=0.0, corner_r=0.0):
    """
    Return True if (x,y) is inside a rectangle centered at (cx,cy) with size (w,h),
    inflated by 'pad' on each side, *excluding* rounded corners of radius corner_r.

    corner_r == 0 -> plain padded rectangle (legacy behavior).
    """
    # Effective (possibly inflated) rectangle dimensions
    W = max(0.0, w + 2.0 * pad)
    H = max(0.0, h + 2.0 * pad)

    # Translate to rect-centered coordinates
    dx = abs(x - cx)
    dy = abs(y - cy)

    # First, must be inside the padded axis-aligned rectangle
    if dx > W / 2.0 or dy > H / 2.0:
        return False

    # If no corner rounding requested, done.
    if corner_r <= 0.0:
        return True

    # Exclude near-corner region: treat each corner as a quarter circle of radius corner_r
    # with centers at (±W/2, ±H/2). If the point lies within any quarter circle, reject.
    # Compute distance to the nearest corner in the first quadrant using symmetry.
    # (dx, dy) are non-negative due to abs above.
    ddx = max(0.0, (W / 2.0) - dx)  # distance from inside point to vertical edge (inward)
    ddy = max(0.0, (H / 2.0) - dy)  # distance from inside point to horizontal edge (inward)

    # When close to a corner, both ddx and ddy are small. Distance to the corner inside
    # the rectangle is sqrt((W/2 - dx)^2 + (H/2 - dy)^2).
    dist2_corner = ((W / 2.0) - dx) ** 2 + ((H / 2.0) - dy) ** 2

    # If the point is within the corner radius of the true corner, exclude it.
    # (This effectively "rounds" the rectangle by cutting out quarter circles.)
    return dist2_corner > (corner_r ** 2)


class ViolationCounter(Node):
    def __init__(
        self,
        pose_topic="/model/drone1/pose/info",
        meta_path=META_PATH,
        zone_padding_m=0.0,
        corner_margin_m=0.25,  # NEW: radius of rounded corner cutouts
    ):
        super().__init__("violation_counter")

        # Allow tuning via ROS params or constructor args (constructor defaults shown above).
        self.declare_parameter("zone_padding_m", zone_padding_m)
        self.declare_parameter("corner_margin_m", corner_margin_m)

        # Clamp to non-negative; stricter only.
        self._pad = max(0.0, float(self.get_parameter("zone_padding_m").value))
        self._corner_r = max(0.0, float(self.get_parameter("corner_margin_m").value))

        self._rects = load_rects(meta_path)
        self._violations = 0
        self._inside_any = False

        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self.get_logger().info(
            f"[violations] Loaded {len(self._rects)} boxes. "
            f"zone_padding_m={self._pad:.3f} m, corner_margin_m={self._corner_r:.3f} m"
        )

    def _on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        in_zone = any(
            inside_rect_rounded(x, y, cx, cy, w, h, pad=self._pad, corner_r=self._corner_r)
            for (cx, cy, w, h) in self._rects
        )

        # detect new entry (but ignore rounded-corner cutouts)
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
        # (Optional) If you ever want to fully clear state, you could also:
        # self._inside_any = False


def start_violation_monitor(
    pose_topic="/model/drone1/pose/info",
    meta_path=META_PATH,
    zone_padding_m=0.0,
    corner_margin_m=0.25,
):
    """
    Start the ViolationCounter in a background thread.
    Assumes rclpy.init() has already been called elsewhere.

    Examples:
      # programmatic
      node = start_violation_monitor(zone_padding_m=0.5, corner_margin_m=0.3)

      # via ROS params (if you expose as an executable):
      # ros2 run hydra_teleop violations \
      #   --ros-args -p zone_padding_m:=0.5 -p corner_margin_m:=0.3
    """
    node = ViolationCounter(
        pose_topic=pose_topic,
        meta_path=meta_path,
        zone_padding_m=zone_padding_m,
        corner_margin_m=corner_margin_m,
    )
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    return node
