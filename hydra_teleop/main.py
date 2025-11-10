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
import threading  # <-- NEW
from typing import List, Dict, Any, Optional, Tuple

import rclpy

from .config import TeleopConfig
from .simulation.sim import start_sim, stop_sim
from .navigation.teleop import GzTeleop
from .simulation.pose_republisher import add_pose_republisher_to_executor
from .tools.bridge import start_parameter_bridge
from .tools.violations import add_violation_monitor_to_executor, start_violation_monitor
from .tools.energy_monitor import add_energy_monitor_to_executor, start_energy_monitor
from .tools.event_emitter import add_event_emitter_to_executor, EventCfg
from .navigation.nav_algorithm_T import LidarTargetNavigatorTROOP
from .analysis.log_transformer import run_from_cfg
from .analysis.statistics_analyzer import run_analysis
from .logging.async_logger import setup_async_logger, AsyncLoggerCfg
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


def _run_one_strategy(
    strategy_name: str,
    ctrl: GzTeleop,
    cfg: TeleopConfig,
    exec: MultiThreadedExecutor,
) -> Tuple[bool, float]:  # <-- CHANGED: correct return type (2-tuple)
    """
    Start a fresh sim, run a navigator to target (with timeout), collect results.
    Ensures sim/nav clean shutdown even on exceptions.
    """
    sim = None
    nav = None
    try:
        sim = start_sim(cfg)
        nav = LidarTargetNavigatorTROOP(ctrl, cfg, strategy_name)
        nav.attach_to_executor(exec)
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
        max_bytes=0,  # disables rotation
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
    pose_node = None
    rosgz_bridge = None  # kept for symmetry if you add other bridges later

    # --- Event emitter ---
    emitter = None

    # --- Velocity publisher ---
    ctrl = None

    # --- Generators (single instances; re-run each loop) ---
    arena_gen = None

    # --- Violation monitor (single instance; reset per run) ---
    viol_node= None
    ener_node= None

    # Results
    stats: List[Dict[str, Any]] = []

    if cfg.simulation_runs != 0:
        exec = None
        spin_stop = threading.Event()  # <-- NEW
        spin_thread = None             # <-- NEW
        try:
            rclpy.init()
            exec = MultiThreadedExecutor(num_threads=3)  # <-- bump to 3 if events + nav + pose
            cbg = ReentrantCallbackGroup()

            # Pose republisher node (no spin here; we attach to shared executor)
            pose_node = add_pose_republisher_to_executor(
                exec, ["drone1", "target_sphere"], callback_group=cbg
            )

            # Start the ONE spin loop in a dedicated thread (non-blocking)
            def _spin():
                while rclpy.ok() and not spin_stop.is_set():
                    exec.spin_once(timeout_sec=0.005)  # low latency

            spin_thread = threading.Thread(target=_spin, daemon=True)
            spin_thread.start()

            # Parameter bridge lines (cmd_vel, lidar) for world
            rosgz_bridge = start_parameter_bridge([
                ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
                ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
            ])

            # Start event emitter (own internal thread; not an rclpy spin)
            emitter = add_event_emitter_to_executor(exec, 
                cfg, 
                gen_cfg=EventCfg(seed=42, event_deterministic=True),
                callback_group=cbg)
            emitter.start()

            # Velocity publisher
            ctrl = GzTeleop(cfg.topic, cfg)

            # Violation & energy monitors (optional; keep disabled if they spin internally)
            viol_node = add_violation_monitor_to_executor(
                exec,
                pose_topic="/model/drone1/pose/info",
                meta_path="models/generated/generated_nofly_meta.json",
                #deep_margin_m=4.0,
                #dwell_s=1.0,
                callback_group=cbg,  # optional, matches your other nodes
            )
            ener_node = add_energy_monitor_to_executor(exec,  
                pose_topic="/model/drone1/pose/info", 
                callback_group=cbg)

            for i in range(cfg.simulation_runs):
                run_idx = i + 1

                # Generators (No-fly + Target)
                if(cfg.simulation_world_style == "city"):
                    from .tools.arena_generator_city import ArenaGenerator, ArenaGenCfg
                else:
                    from .tools.arena_generator_perlin import ArenaGenerator, ArenaGenCfg
                arena_gen = ArenaGenerator(cfg, ArenaGenCfg(
                    seed=run_idx+cfg.world_gen_seed_offset,
                    target_min_dist=20.0,
                    pass_through=True, visual_alpha=0.0,
                    outdir="models/generated",
                ))
                logger.info(
                        {"simulationruns": cfg.simulation_runs,"seedoffset": cfg.world_gen_seed_offset,"arena": cfg.simulation_world_style,}
                )
                # Violation monitor depends on targets/NFZ → generate first
                arena_gen.run()

                print(f"\n=== Hydra Experiment Run {run_idx} (fresh target & NFZ) ===")

                # Strategies (APE1/APE2/APE3/TROOP) — each run uses same event seed
                for strategy in (cfg.analyzer_strategies):
                    print(f"\n=== Hydra Experiment Strategy {strategy} ===")
                    emitter.reset()
                    viol_node.mark_run_start(label=strategy)
                    ener_node.mark_run_start(label=strategy)
                    reached, elapsed = _run_one_strategy(strategy, ctrl, cfg, exec)
                    violation_summary = viol_node.log_and_reset(label=strategy, include_boxes=True)
                    energy_summary = ener_node.log_and_reset(label=strategy, include_params=True)
                    logger.info(
                        {"reached": reached, "elapsed": elapsed, "violations": violation_summary.get("total_violations"), "energy_j": energy_summary.get("energy_j"), "mean_power_w": energy_summary.get("mean_power_w")},
                        extra={"strategy": strategy}
                    )
        finally:
            # Stop spin loop first (so no callbacks run during teardown)
            try:
                spin_stop.set()
                if spin_thread is not None:
                    spin_thread.join(timeout=1.0)
            except Exception:
                pass

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
                if pose_node is not None and exec is not None:
                    exec.remove_node(pose_node)
                    exec.remove_node(viol_node)
                    exec.remove_node(ener_node)
                    pose_node.close()
            except Exception as e:
                print(f"[hydra][WARN] pose_node close/remove error: {e}")

            try:
                if exec is not None:
                    exec.shutdown()
            except Exception:
                pass

            try:
                rclpy.shutdown()
            except Exception:
                pass

            try:
                if rosgz_bridge is not None:
                    rosgz_bridge.stop()
            except Exception as e:
                print(f"[hydra][WARN] rosgz_bridge.stop() error: {e}")

            try:
                if log_handle is not None:
                    log_handle.stop()
            except Exception as e:
                print(f"[hydra][WARN] log_handle.stop() error: {e}")
    else:
        print(f"\n=== Simulations are 0, running in analysis mode only ===")

    run_from_cfg()
    result = run_analysis(zone_metric="mean")
    print(" Zone Metric:", result["zone_metric"])
    print(" Summary Csv:", result["summary_csv"])
    print(" Summary:", result["summary"])


if __name__ == "__main__":
    main()
