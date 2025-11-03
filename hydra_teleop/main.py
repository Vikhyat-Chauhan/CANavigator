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
from .simulation.sim import start_sim, stop_sim
from .navigation.teleop import GzTeleop
from .simulation.pose_republisher import start_pose_bridge
from .tools.bridge import start_parameter_bridge
from .tools.violations import start_violation_monitor
from .tools.event_emitter import EventEmitter, EventCfg
from .navigation.nav_algorithm_T import LidarTargetNavigatorTROOP
from .analysis.log_transformer import run_from_cfg
from .analysis.statistics_analyzer import run_analysis
from .logging.async_logger import setup_async_logger, AsyncLoggerCfg
from .tools.arena_generator import ArenaGenerator, ArenaGenCfg

def _run_one_strategy(
    strategy_name: str,
    ctrl: GzTeleop,
    cfg: TeleopConfig
) -> Tuple[bool, Optional[float], int]:
    """
    Start a fresh sim, run a navigator to target (with timeout), collect results.
    Ensures sim/nav clean shutdown even on exceptions.
    """
    sim = None
    nav = None
    try:
        sim = start_sim(cfg)
        nav = LidarTargetNavigatorTROOP(ctrl, cfg, strategy_name)
        reached, elapsed = nav.go_to(timeout_s=cfg.simulation_timeout)
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
        max_bytes=0, # disables rotation
        queue_maxsize=8000,
        drop_on_full=False,
        console=False,
        level=logging.INFO,
        monitor_interval_s=10.0,
        json_format=True
    )
    log_handle = setup_async_logger(logcfg)
    logger = logging.getLogger("hydra_teleop.main")
    print(f"\n=== [LOGGING INITILIZED] ===")

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
    arena_gen = None

    # --- Violation monitor (single instance; reset per run) ---
    violation_monitor = None

    # Results
    stats: List[Dict[str, Any]] = []
    if(cfg.simulation_runs != 0):
        try:
            # Pose + ROS↔GZ bridges
            pose_bridge = start_pose_bridge(["drone1", "target_sphere"])
            # Parameter bridge lines (cmd_vel, lidar) for world
            rosgz_bridge = start_parameter_bridge([
                ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
                ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
            ])

            # Start event emitter (separate executor thread)
            emitter = EventEmitter(cfg, gen_cfg=EventCfg(seed=42, event_deterministic=True))
            emitter.start()

            # Velocity publisher
            ctrl = GzTeleop(cfg.topic, cfg)
            
            # Violation monitor
            #start_violation_monitor(zone_padding_m=-0.30, corner_margin_m=0.30)
            viol_node, viol_thread = start_violation_monitor(
                pose_topic="/model/drone1/pose/info",
                meta_path="models/generated/generated_nofly_meta.json",
                deep_margin_m= 2.0,
                dwell_s=1
            )
            for i in range(cfg.simulation_runs):
                run_idx = i + 1
                # Generators (No-fly + Target)
                arena_gen = ArenaGenerator(cfg,ArenaGenCfg(
                    seed=run_idx,
                    #density=0.25, corr_len_m=12.0,
                    target_min_dist=20.0,
                    pass_through=True, visual_alpha=0.0,
                    outdir="models/generated",
                ))
                # Fix Required : because violation monitor depends on the target and Zone generation we have to run it before
                arena_gen.run()
                print(f"\n=== Hydra Experiment Run {run_idx} (fresh target & NFZ) ===")
                # ---------------- TROOP ----------------
                for strategy in (cfg.analyzer_strategies):
                    # Event emitter will be resetted every run for now, to keep the sequence same for each isolated run. 
                    # This will allow us to observe all APEs under same event load since the seed is fixed.
                    print(f"\n=== Hydra Experiment Strategy {strategy} ===")
                    emitter.reset()
                    reached, elapsed = _run_one_strategy(strategy, ctrl, cfg)
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
    else:
        print(f"\n=== Simulations are 0, running in analysis mode only ===")
    
    run_from_cfg()
    result = run_analysis(zone_metric="mean")
    print(" Best strategy:", result["best_strategy"])
    print(" Metrics:", result["metrics"])
    print(" Summary CSV:", result["summary_csv"])
    print(" 2D plot:", result["plot_2d"])
    print(" 3D plot:", result["plot_3d"])

if __name__ == "__main__":
    main()
