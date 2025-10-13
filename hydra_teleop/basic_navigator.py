#!/usr/bin/env python3
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped

from .teleop import GzTeleop
from .logger import Logger
from .config import TeleopConfig


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (rad) from quaternion (x,y,z,w), Z-up convention."""
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t0, t1)

def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class GoToConfig:
    goal_radius_m: float = 4.0        # stop when within this 3D distance
    kp_lin: float = 0.8               # proportional gain on planar distance
    kp_z: float = 0.8                 # proportional gain on vertical error
    kp_yaw: float = 1.5               # proportional gain on yaw error
    max_v: float = 20.0                # linear speed cap (m/s, +x body)
    max_vz: float = 2.0               # vertical speed cap (m/s)
    max_wz: float = 0.8               # yaw rate cap (rad/s)
    slow_yaw_threshold: float = 0.8   # if |yaw_err| > this, reduce forward speed
    rate_hz: float = 30.0             # control loop rate


class _PoseSub(Node):
    """ROS2 subscriber node: keeps the latest PoseStamped for a single topic."""
    def __init__(self, topic: str, node_name: str):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._latest: Optional[PoseStamped] = None
        self._sub = self.create_subscription(PoseStamped, topic, self._cb, 10)
        self._topic = topic

    def _cb(self, msg: PoseStamped):
        with self._lock:
            self._latest = msg

    def latest(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._latest


class TargetNavigator:
    """
    Minimal position controller:
      - Subscribes to drone PoseStamped (default: /model/<entity>/pose/info)
      - Subscribes to target PoseStamped (default: /model/target_sphere/pose/info)
      - Commands body-frame (vx,0,vz,wz) via GzTeleop until inside goal sphere

    go_to(...) returns (reached: bool, elapsed_time_s: float)
    """

    def __init__(self,
                 teleop: GzTeleop,
                 cfg: TeleopConfig,
                 drone_pose_topic: Optional[str] = None,
                 target_pose_topic: Optional[str] = "/model/target_sphere/pose/info",
                 logger: Optional[Logger] = None,
                 goto_cfg: Optional[GoToConfig] = None):
        self._teleop = teleop
        self._cfg = cfg
        self._logger = logger or Logger(cfg)
        self._gc = goto_cfg or GoToConfig()

        entity = getattr(cfg, "entity_name", "drone1")
        default_drone_topic = f"/model/{entity}/pose/info"
        drone_topic = drone_pose_topic or getattr(cfg, "ros_pose_topic", default_drone_topic)
        target_topic = target_pose_topic or "/model/target_sphere/pose/info"

        # Two small nodes (one for each topic) on a shared executor thread
        self._node_drone = _PoseSub(drone_topic, "basic_hydra_nav_drone_pose")
        self._node_target = _PoseSub(target_topic, "basic_hydra_nav_target_pose")
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node_drone)
        self._exec.add_node(self._node_target)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self._logger.log(source="Navigator", event="init",
                         message=f"drone_topic='{drone_topic}', target_topic='{target_topic}'")

    def _spin(self):
        try:
            while rclpy.ok():
                self._exec.spin_once(timeout_sec=0.1)
        except Exception:
            pass  # quiet on shutdown

    def shutdown(self):
        try:
            self._exec.remove_node(self._node_drone)
            self._exec.remove_node(self._node_target)
            self._node_drone.destroy_node()
            self._node_target.destroy_node()
            self._exec.shutdown()
        except Exception:
            pass

    def _latest_drone(self) -> Optional[Tuple[float, float, float, float]]:
        msg = self._node_drone.latest()
        if msg is None:
            return None
        p, o = msg.pose.position, msg.pose.orientation
        return (p.x, p.y, p.z, _yaw_from_quat(o.x, o.y, o.z, o.w))

    def _latest_target(self) -> Optional[Tuple[float, float, float]]:
        msg = self._node_target.latest()
        if msg is None:
            return None
        p = msg.pose.position
        return (p.x, p.y, p.z)

    def go_to(self,
              target_xyz: Optional[Tuple[float, float, float]] = None,
              timeout_s: Optional[float] = None) -> Tuple[bool, float]:
        """
        If target_xyz is None, follows the TARGET TOPIC (dynamic target).
        Otherwise, uses the provided static (x,y,z).

        Returns:
            reached (bool): True if goal_radius_m reached before timeout.
            elapsed_time_s (float): Total wall-clock time spent in the loop.
        """
        rate = max(1.0, self._gc.rate_hz)
        dt = 1.0 / rate
        t0 = time.time()
        reached = False

        mode = "topic" if target_xyz is None else "static"
        self._logger.log(source="Navigator", event="start",
                         message=f"mode={mode}, R={self._gc.goal_radius_m:.2f}",
                         vx="", vy="", vz="", wx="", wy="", wz="")

        while True:
            dpose = self._latest_drone()
            if dpose is None:
                time.sleep(0.05)
                if timeout_s is not None and (time.time() - t0) > timeout_s:
                    self._logger.log(source="Navigator", event="timeout",
                                     message="no drone pose")
                    break
                continue

            if target_xyz is None:
                tpose = self._latest_target()
                if tpose is None:
                    # no target yet; hover
                    self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                    if timeout_s is not None and (time.time() - t0) > timeout_s:
                        self._logger.log(source="Navigator", event="timeout",
                                         message="no target pose received")
                        break
                    time.sleep(0.05)
                    continue
                tx, ty, tz = tpose
            else:
                tx, ty, tz = target_xyz

            x, y, z, yaw = dpose
            ex, ey, ez = (tx - x), (ty - y), (tz - z)
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)

            if dist <= self._gc.goal_radius_m:
                reached = True
                self._logger.log(source="Navigator", event="reached",
                                 message=f"dist={dist:.2f} at ({x:.2f},{y:.2f},{z:.2f})")
                break

            # Desired heading (world-frame) and yaw error
            hdg_des = math.atan2(ey, ex)
            yaw_err = _wrap_pi(hdg_des - yaw)

            # Forward speed (reduce if misaligned)
            v_cmd = min(self._gc.max_v, self._gc.kp_lin * math.hypot(ex, ey))
            if abs(yaw_err) > self._gc.slow_yaw_threshold:
                v_cmd *= 0.25

            # Vertical and yaw rates
            vz_cmd = max(-self._gc.max_vz, min(self._gc.max_vz, self._gc.kp_z * ez))
            wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_err))

            # Command body-frame velocities: forward v_cmd, no lateral vy
            self._teleop.set_cmd(v_cmd, 0.0, vz_cmd, wz_cmd)

            # timeout check
            if timeout_s is not None and (time.time() - t0) > timeout_s:
                self._logger.log(source="Navigator", event="timeout",
                                 message=f"after {timeout_s:.1f}s")
                break

            time.sleep(dt)

        # Stop & hold zero
        self._teleop.stop()
        time.sleep(max(0.05, 2.0 / self._cfg.rate_hz))
        elapsed = time.time() - t0
        self._logger.log(source="Navigator", event="finish",
                         message=f"reached={reached}, elapsed_s={elapsed:.3f}")
        return reached, elapsed
