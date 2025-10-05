#!/usr/bin/env python3
import time
import rclpy

from .config import TeleopConfig
from .sim import start_sim, stop_sim
from .teleop import GzTeleop
from .pose_republisher import start_pose_bridge
from .basic_navigator import TargetNavigator, GoToConfig as GoToCfgBasic
from .lidar_navigator import LidarTargetNavigator, GoToConfig as GoToCfgLidar, AvoidCfg
from .bridge import start_parameter_bridge
from .violations import start_violation_monitor

def main():
    # --- Boot sim + bridge ---
    cfg = TeleopConfig()
    sim = start_sim(cfg)

    # world → ROS pose bridge for entities we care about
    pose_bridge = start_pose_bridge(["drone1", "target_sphere"])
    # --- Bring up topic bridges ---
    rosgz_bridge = start_parameter_bridge([
        ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
        ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),]
    )

    # body-frame velocity publisher
    ctrl = GzTeleop(cfg.topic, cfg)
    ctrl.start()

    violation_monitor = start_violation_monitor()
    #simple_navigator = TargetNavigator(ctrl, cfg)
    lidar_navigator = LidarTargetNavigator(
        ctrl, cfg,
        goto_cfg=GoToCfgLidar(
            goal_radius_m=2.0,
            kp_lin=0.8, kp_z=0.8, kp_yaw=1.5,
            max_v=min(5.0, cfg.speed_x),
            max_vz=min(5.0, cfg.speed_z),
            max_wz=min(0.8, cfg.yaw_rate),
            slow_yaw_threshold=0.8,
            rate_hz=20.0,
        ),
        avoid_cfg=AvoidCfg(
            safe_m=5.0,            # start avoiding if something is < 6 m ahead
            front_deg=5.0,         # central cone half-width
            side_deg=30.0,         # side cone half-width
            side_center_deg=30.0,  # side cone centers at ±30°
            turn_rate=0.2,         # rad/s while actively avoiding (capped by max_wz)
            hysteresis_m=0.0,      # extra clearance before resuming forward
            watchdog_sec=0.6       # stop if scans go stale
        )
    )

    try:
        #reached = simple_navigator.go_to()
        reached = lidar_navigator.go_to()
        violations = violation_monitor.get_total()  # returns int
        print(f"[hydra] Simple Navigator result: {'reached with violations ' + str(violations) if reached else 'timeout'}")
    except KeyboardInterrupt:
        #simple_navigator.shutdown()
        lidar_navigator.shutdown()
        violation_monitor.reset()
        stop_sim(sim)
        pose_bridge.stop()
        rosgz_bridge.stop()
    finally:
        #simple_navigator.shutdown()
        lidar_navigator.shutdown()
        violation_monitor.reset()
        stop_sim(sim)
        pose_bridge.stop()
        rosgz_bridge.stop()

if __name__ == "__main__":
    main()
