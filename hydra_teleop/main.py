#!/usr/bin/env python3
from logging import Logger
import time
import os
import math
import csv

from .config import TeleopConfig
from .sim import start_sim, stop_sim
from .teleop import GzTeleop
from .pose_republisher import start_pose_bridge
from .basic_navigator import TargetNavigator, GoToConfig as GoToCfgBasic
from .lidar_navigator import LidarTargetNavigator, GoToConfig as GoToCfgLidar, AvoidCfg
from .bridge import start_parameter_bridge
from .violations import start_violation_monitor
from .tools.nofly_generator import NoFlyGenerator, NoFlyGenCfg
from .tools.target_generator import TargetGenerator, TargetGenCfg
from hydra_teleop.nav_algorithm_1 import LidarTargetNavigatorAPE1
from hydra_teleop.nav_algorithm_2 import LidarTargetNavigatorAPE2
from hydra_teleop.nav_algorithm_3 import LidarTargetNavigatorAPE3
from .statistics_analyzer import StatsAnalyzer


def _record(stats, run_idx, strategy, reached, elapsed, violations):
    stats.append({
        "run": run_idx,
        "strategy": strategy,
        "reached": bool(reached),
        "elapsed_s": float(elapsed) if elapsed is not None else float("nan"),
        "violations": int(violations),
    })


def _write_csv(stats, path):
    # ensure parent directory exists
    parent = os.path.dirname(os.path.expanduser(path))
    if parent:
        os.makedirs(parent, exist_ok=True)

    with open(os.path.expanduser(path), "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["run", "strategy", "reached", "elapsed_s", "violations"]
        )
        writer.writeheader()
        writer.writerows(stats)
    print(f"[hydra] Wrote {os.path.abspath(os.path.expanduser(path))}")


def main():
    cfg = TeleopConfig()

    # Bridges (pose + ROS<->GZ)
    pose_bridge = start_pose_bridge(["drone1", "target_sphere"])
    rosgz_bridge = start_parameter_bridge([
        ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
        ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
    ])

    # Velocity publisher
    ctrl = GzTeleop(cfg.topic, cfg)
    ctrl.start()

    # Generate a single no-fly map (shared), target will be reset per-run
    nofly = NoFlyGenerator(cfg, NoFlyGenCfg())
    nofly.run()
    target = TargetGenerator(cfg, TargetGenCfg())
    #target.run()

    # Violation monitor
    violation_monitor = start_violation_monitor(zone_padding_m=0, corner_margin_m=0.5)

    stats = []
    N_REPEATS = 1
    TIMEOUT_S = 200

    for i in range(N_REPEATS):
        run_idx = i + 1
        print(f"\n=== Hydra Experiment Run {run_idx} (new target) ===")
        nofly.run()
        target.run()

        # ---- APE1 (baseline lidar local planner) ----
        sim = start_sim(cfg)
        
        try:
            logger = Logger("APE1")
            nav = LidarTargetNavigatorAPE1(ctrl, cfg, logger=logger)
            reached, elapsed = nav.go_to(timeout_s=TIMEOUT_S)
            violations = violation_monitor.get_total()
            _record(stats, run_idx, "NAVALGO1", reached, elapsed, violations)
            print(f"[hydra] NAVALGO1: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {math.floor(elapsed)} s")
        finally:
            nav.shutdown()
            violation_monitor.reset()
            stop_sim(sim)
        
        # ---- APE2 (breadcrumbs variant) ----
        sim = start_sim(cfg)
        try:
            logger = Logger("APE2")
            nav = LidarTargetNavigatorAPE2(ctrl, cfg, logger=logger)
            reached, elapsed = nav.go_to(timeout_s=TIMEOUT_S)
            violations = violation_monitor.get_total()
            _record(stats, run_idx, "NAVALGO2", reached, elapsed, violations)
            print(f"[hydra] NAVALGO2: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {math.floor(elapsed)} s")
        finally:
            nav.shutdown()
            violation_monitor.reset()
            stop_sim(sim)
        
        # ---- APE3 (breadcrumbs variant) ----
        sim = start_sim(cfg)
        try:
            logger = Logger("APE3")
            nav = LidarTargetNavigatorAPE3(ctrl, cfg, logger=logger)
            reached, elapsed = nav.go_to(timeout_s=TIMEOUT_S)
            violations = violation_monitor.get_total()
            _record(stats, run_idx, "NAVALGO3", reached, elapsed, violations)
            print(f"[hydra] NAVALGO3: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {math.floor(elapsed)} s")
        finally:
            nav.shutdown()
            violation_monitor.reset()
            stop_sim(sim)

    # Cleanup bridges
    pose_bridge.stop()
    rosgz_bridge.stop()

    # Save raw results only (no plots, no stats here)
    _write_csv(stats, path=cfg.results_csv_path)
     # --- optionally run analyzer after finishing the experiment ---
    # Auto-run analyzer based on config
    if cfg.run_analyzer_after:
        try:
            StatsAnalyzer.from_config(cfg).analyze()
        except Exception as e:
            print(f"[hydra] Analyzer skipped due to error: {e}")


if __name__ == "__main__":
    main()
