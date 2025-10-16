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
from .basic_navigator import TargetNavigator, GoToConfig as GoToCfgBasic  # noqa: F401 (kept for compatibility)
from .lidar_navigator import LidarTargetNavigator, GoToConfig as GoToCfgLidar, AvoidCfg  # noqa: F401
from .violations import start_violation_monitor
from .tools.nofly_generator import NoFlyGenerator, NoFlyGenCfg
from .tools.target_generator import TargetGenerator, TargetGenCfg
from .tools.event_emitter import EventEmitter, EventCfg
from hydra_teleop.nav_algorithm_1 import LidarTargetNavigatorAPE1
from hydra_teleop.nav_algorithm_2 import LidarTargetNavigatorAPE2
from hydra_teleop.nav_algorithm_3 import LidarTargetNavigatorAPE3
from hydra_teleop.nav_algorithm_T import LidarTargetNavigatorTROOP
from .statistics_analyzer import StatsAnalyzer


def _record(stats: List[Dict[str, Any]],
            run_idx: int,
            strategy: str,
            reached: bool,
            elapsed: Optional[float],
            violations: int) -> None:
    stats.append({
        "run": run_idx,
        "strategy": strategy,
        "reached": bool(reached),
        "elapsed_s": float(elapsed) if elapsed is not None else float("nan"),
        "violations": int(violations),
    })


def _write_csv(stats: List[Dict[str, Any]], path: str) -> None:
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


def _safe_floor_elapsed(elapsed: Optional[float]) -> str:
    if elapsed is None:
        return "nan"
    try:
        return str(math.floor(elapsed))
    except Exception:
        return "nan"


def _run_one_strategy(
    strategy_name: str,
    nav_ctor,
    ctrl: GzTeleop,
    cfg: TeleopConfig,
    violation_monitor,
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
        logger = logging.getLogger(strategy_name)
        nav = nav_ctor(ctrl, cfg, logger=logger)
        reached, elapsed = nav.go_to(timeout_s=timeout_s)
        violations = violation_monitor.get_total()
        return reached, elapsed, violations
    finally:
        # Try to shut down navigator even if construction failed mid-way
        try:
            if nav is not None:
                nav.shutdown()
        except Exception as e:
            print(f"[hydra][WARN] {strategy_name} nav.shutdown() error: {e}")

        # Reset violation monitor per run
        try:
            violation_monitor.reset()
        except Exception as e:
            print(f"[hydra][WARN] violation_monitor.reset() error: {e}")

        # Stop simulation even if start failed later
        try:
            if sim is not None:
                stop_sim(sim)
        except Exception as e:
            print(f"[hydra][WARN] stop_sim error for {strategy_name}: {e}")


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    cfg = TeleopConfig()

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
        N_REPEATS = 1
        TIMEOUT_S = 160

        for i in range(N_REPEATS):
            run_idx = i + 1
            print(f"\n=== Hydra Experiment Run {run_idx} (fresh target & NFZ) ===")

            # Regenerate NFZ and target every run
            nofly_gen.run()
            target_gen.run()

            # ---------------- APE1 ----------------
            strategy = "NAVALGO1"
            reached, elapsed, violations = _run_one_strategy(
                strategy, LidarTargetNavigatorAPE1, ctrl, cfg, violation_monitor, TIMEOUT_S
            )
            _record(stats, run_idx, strategy, reached, elapsed, violations)
            print(f"[hydra] {strategy}: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {_safe_floor_elapsed(elapsed)} s")

            # ---------------- APE2 ----------------
            strategy = "NAVALGO2"
            reached, elapsed, violations = _run_one_strategy(
                strategy, LidarTargetNavigatorAPE2, ctrl, cfg, violation_monitor, TIMEOUT_S
            )
            _record(stats, run_idx, strategy, reached, elapsed, violations)
            print(f"[hydra] {strategy}: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {_safe_floor_elapsed(elapsed)} s")

            # ---------------- APE3 ----------------
            strategy = "NAVALGO3"
            reached, elapsed, violations = _run_one_strategy(
                strategy, LidarTargetNavigatorAPE3, ctrl, cfg, violation_monitor, TIMEOUT_S
            )
            _record(stats, run_idx, strategy, reached, elapsed, violations)
            print(f"[hydra] {strategy}: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {_safe_floor_elapsed(elapsed)} s")

            # ---------------- TROOP ----------------
            strategy = "TROOP"
            reached, elapsed, violations = _run_one_strategy(
                strategy, LidarTargetNavigatorTROOP, ctrl, cfg, violation_monitor, TIMEOUT_S
            )
            _record(stats, run_idx, strategy, reached, elapsed, violations)
            print(f"[hydra] {strategy}: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {_safe_floor_elapsed(elapsed)} s")

        # Save raw results
        _write_csv(stats, path=cfg.results_csv_path)

        # Optionally run analyzer after finishing
        if getattr(cfg, "run_analyzer_after", False):
            try:
                StatsAnalyzer.from_config(cfg).analyze()
            except Exception as e:
                print(f"[hydra] Analyzer skipped due to error: {e}")

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

if __name__ == "__main__":
    main()
