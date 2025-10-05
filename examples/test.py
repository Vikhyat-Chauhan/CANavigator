#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SAFE_M = 5.0
FWD_SPEED = 2.0
TURN_SPEED = 0.8
CENTER_DEG = 5.0

class SimpleAvoid(Node):
    def __init__(self):
        super().__init__('simple_avoid')
        self.sub = self.create_subscription(
            LaserScan, '/model/drone1/front_lidar/scan', self.on_scan, 10)
        self.pub = self.create_publisher(Twist, '/model/drone1/cmd_vel', 10)
        self._printed_meta = False

    def on_scan(self, msg: LaserScan):
        # Print scan meta once for sanity
        if not self._printed_meta:
            self.get_logger().info(
                f"scan meta: angle_min={msg.angle_min:.3f}, angle_max={msg.angle_max:.3f}, "
                f"angle_inc={msg.angle_increment:.6f}, n={len(msg.ranges)}")
            self._printed_meta = True

        if not msg.ranges:
            return  # nothing to do yet
        
        # Robust index window for "front"
        n = len(msg.ranges)
        inc = msg.angle_increment
        width_rad = math.radians(CENTER_DEG)
        
        # Compute indices from angles normally
        lo = int((0.0 - width_rad - msg.angle_min) / inc)
        hi = int((0.0 + width_rad - msg.angle_min) / inc)
        lo = max(0, min(n - 1, lo))
        hi = max(0, min(n - 1, hi))
        if lo > hi:
            lo, hi = hi, lo

        # Min valid range in window
        window = [r for r in msg.ranges[lo:hi+1] if math.isfinite(r) and r > 0.0]
        min_front = min(window) if window else float('inf')

        """cmd = Twist()
        if min_front < SAFE_M:
            cmd.angular.z = TURN_SPEED
            cmd.linear.x = 0.0
            self.get_logger().debug(f"Obstacle {min_front:.2f}m → turning")
        else:
            cmd.linear.x = FWD_SPEED
            cmd.angular.z = 0.0
        self.pub.publish(cmd)"""

def main():
    rclpy.init()
    node = SimpleAvoid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
