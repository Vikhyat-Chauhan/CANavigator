#!/usr/bin/env python3
"""
Hydra experiment runner:
- Spawns bridges (pose + ROS↔GZ)
- Emits events
- Generates no-fly zones & targets
- Runs each navigator (APE1, APE2, APE3, TROOP) in a fresh sim
- Records results to CSV and (optionally) runs the analyzer

All public APIs are preserved.
"""

from __future__ import annotations

import os
import csv
import math
import logging
from typing import List, Dict, Any, Optional, Tuple

from .config import TeleopConfig
from .sim import start_sim, stop_sim
from .teleop import GzTeleop
from .pose_republisher import start_pose_bridge
from .bridge import start_parameter_bridge
from .violations import start_violation_monitor
from .tools.nofly_generator import NoFlyGenerator, NoFlyGenCfg
from .tools.target_generator import TargetGenerator, TargetGenCfg
from .tools.event_emitter import EventEmitter, EventCfg
from .navigation.nav_algorithm_1 import LidarTargetNavigatorAPE1
from .navigation.nav_algorithm_2 import LidarTargetNavigatorAPE2
from .navigation.nav_algorithm_3 import LidarTargetNavigatorAPE3
from .navigation.nav_algorithm_T import LidarTargetNavigatorTROOP
from .analysis.log_transformer import log_transformer
from .analysis.statistics_analyzer import StatsAnalyzer
from .logging.async_logger import setup_async_logger, AsyncLoggerCfg

def _run_one_strategy(
    strategy_name: str,
    nav_ctor,
    ctrl: GzTeleop,
    cfg: TeleopConfig,
    timeout_s: float
) -> Tuple[bool, Optional[float], int]:
    """
    Start a fresh sim, run a navigator to target (with timeout), collect results.
    Ensures sim/nav clean shutdown even on exceptions.
    """
    sim = None
    nav = None
    try:
        sim = start_sim(cfg)
        nav = nav_ctor(ctrl, cfg)
        reached, elapsed = nav.go_to(timeout_s=timeout_s)
        return reached, elapsed
    finally:
        # Try to shut down navigator even if construction failed mid-way
        try:
            if nav is not None:
                nav.shutdown()
        except Exception as e:
            print(f"[hydra][WARN] {strategy_name} nav.shutdown() error: {e}")

        # Stop simulation even if start failed later
        try:
            if sim is not None:
                stop_sim(sim)
        except Exception as e:
            print(f"[hydra][WARN] stop_sim error for {strategy_name}: {e}")


def main() -> None:
    cfg = TeleopConfig()
    logcfg = AsyncLoggerCfg(
        logfile=cfg.log_path,
        queue_maxsize=8000,
        drop_on_full=True,
        console=False,
        level=logging.INFO,
        monitor_interval_s=10.0,
        json_format=True
    )
    log_handle = setup_async_logger(logcfg)
    logger = logging.getLogger("hydra_teleop.main")
    logger.info("[LOGGING INITILIZED]")

    # --- Bridges (pose + ROS↔GZ) ---
    pose_bridge = None
    rosgz_bridge = None  # kept for symmetry if you add other bridges later

    # --- Event emitter ---
    emitter = None

    # --- Velocity publisher ---
    ctrl = None

    # --- Generators (single instances; re-run each loop) ---
    nofly_gen = None
    target_gen = None

    # --- Violation monitor (single instance; reset per run) ---
    violation_monitor = None

    # Results
    stats: List[Dict[str, Any]] = []

    try:
        # Pose + ROS↔GZ bridges
        pose_bridge = start_pose_bridge(["drone1", "target_sphere"])
        # Parameter bridge lines (cmd_vel, lidar) for world
        rosgz_bridge = start_parameter_bridge([
            ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
            ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
        ])

        # Start event emitter (separate executor thread)
        emitter = EventEmitter(cfg)
        emitter.start()

        # Velocity publisher
        ctrl = GzTeleop(cfg.topic, cfg)
        ctrl.start()

        # Generators (No-fly + Target)
        nofly_gen = NoFlyGenerator(cfg, NoFlyGenCfg())
        target_gen = TargetGenerator(cfg, TargetGenCfg())

        # Violation monitor
        violation_monitor = start_violation_monitor(zone_padding_m=0, corner_margin_m=0.5)

        # --- Experiment control ---
        N_REPEATS = 20
        TIMEOUT_S = 200

        for i in range(N_REPEATS):
            run_idx = i + 1
            print(f"\n=== Hydra Experiment Run {run_idx} (fresh target & NFZ) ===")

            # Regenerate NFZ and target every run
            nofly_gen.run()
            target_gen.run()

            # ---------------- APE1 ----------------
            strategy = "APE1"
            reached, elapsed = _run_one_strategy(strategy, LidarTargetNavigatorAPE1, ctrl, cfg, TIMEOUT_S)
            logger.info({
                            "reached" : reached,
                            "elapsed" : elapsed
                        },
                        extra = {
                            "strategy" : strategy
                        })  
            # ---------------- APE2 ----------------
            strategy = "APE2"
            reached, elapsed = _run_one_strategy(strategy, LidarTargetNavigatorAPE2, ctrl, cfg, TIMEOUT_S)
            logger.info({
                            "reached" : reached,
                            "elapsed" : elapsed
                        },
                        extra = {
                            "strategy" : strategy
                        })  
            # ---------------- APE3 ----------------
            strategy = "APE3"
            reached, elapsed = _run_one_strategy(strategy, LidarTargetNavigatorAPE3, ctrl, cfg, TIMEOUT_S)
            logger.info({
                            "reached" : reached,
                            "elapsed" : elapsed
                        },
                        extra = {
                            "strategy" : strategy
                        })  
            # ---------------- TROOP ----------------
            strategy = "TROOP"
            reached, elapsed = _run_one_strategy(strategy, LidarTargetNavigatorTROOP, ctrl, cfg, TIMEOUT_S)
            logger.info({
                            "reached" : reached,
                            "elapsed" : elapsed
                        },
                        extra = {
                            "strategy" : strategy
                        })  

    finally:
        # Teardown in reverse order of startup
        try:
            if emitter is not None:
                emitter.stop()
        except Exception as e:
            print(f"[hydra][WARN] emitter.stop() error: {e}")

        try:
            if ctrl is not None:
                ctrl.stop()
        except Exception as e:
            print(f"[hydra][WARN] ctrl.stop() error: {e}")

        try:
            if pose_bridge is not None:
                pose_bridge.stop()
        except Exception as e:
            print(f"[hydra][WARN] pose_bridge.stop() error: {e}")

        try:
            if rosgz_bridge is not None:
                rosgz_bridge.stop()
        except Exception as e:
            print(f"[hydra][WARN] rosgz_bridge.stop() error: {e}")

        try:
            if log_handle is not None:
                log_handle.stop()
        except Exception as e:
            print(f"[hydra][WARN] rosgz_bridge.stop() error: {e}")

        log_transformer(
            in_path=cfg.log_path,
            out_path=cfg.results_csv_path
        )

        StatsAnalyzer.from_config(cfg).analyze()

if __name__ == "__main__":
    main()
