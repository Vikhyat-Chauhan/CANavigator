#!/usr/bin/env python3
"""
Hydra experiment runner:
- Spawns bridges (pose + ROS↔GZ)
- Emits events
- Generates no-fly zones & targets
- Runs each navigator (APE1, APE2, APE3, TROOP) in a fresh sim
- Records results to CSV and (optionally) runs the analyzer

Runs until cfg.simulation_runs "good runs" are collected (all strategies
reached == True). Any attempt where a strategy fails is discarded immediately.
For each good run, one row per strategy is written to cfg.results_csv_path
with a run_id (1..cfg.simulation_runs) plus the buffered fields.
"""

import os
import csv
import logging
import threading
from typing import List, Dict, Any, Tuple

import rclpy

from .config import TeleopConfig
from .simulation.sim import start_sim, stop_sim, reset_sim
from .navigation.teleop import GzTeleop
from .simulation.pose_republisher import add_pose_republisher_to_executor
from .tools.bridge import start_parameter_bridge
from .tools.violations import add_violation_monitor_to_executor
from .tools.energy_monitor import add_energy_monitor_to_executor
from .tools.event_emitter import add_event_emitter_to_executor, EventCfg
from .navigation.nav_algorithm_T import LidarTargetNavigatorTROOP
from .tools.rtf_monitor import RtfMonitor
from .analysis.statistics_analyzer import run_analysis
from .logging.async_logger import setup_async_logger, AsyncLoggerCfg
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


def _run_one_strategy(
    strategy_name: str,
    ctrl: GzTeleop,
    cfg: TeleopConfig,
    exec: MultiThreadedExecutor,
) -> Tuple[bool, float, float, float, int, int, int, int]:
    """
    Run a navigator to target (with timeout) inside an already-running sim.
    Sim lifetime is managed by the caller — this function only manages the navigator.

    Returns:
      (reached: bool, elapsed_s: float, latency_us: float,
       compute_energy_j: float, events_handled: int, events_violated: int,
       events_violated_deadline: int, events_violated_preemptive: int)
    """
    nav = None
    try:
        nav = LidarTargetNavigatorTROOP(ctrl, cfg, strategy_name,
                                        # For TROOP, APE3 is selected only when deadline >= 2589ms.
                                        # This is intentionally above the raw APE3 budget (2035ms)
                                        # to add a 554ms safety margin, shifting ~14% of events from
                                        # APE3 to APE2 (empirical tier split: APE1≈61%, APE2≈23%,
                                        # APE3≈16%). Solo APE1/2/3 strategies use default thresholds
                                        # (None → EventDecisionCfg defaults).
                                        ape3_select_threshold_ms=2589 if strategy_name == "TROOP" else None)
        nav.attach_to_executor(exec)
        reached, elapsed, latency_us, compute_energy_j, events_handled, \
            events_violated, events_violated_deadline, events_violated_preemptive = nav.go_to(
            timeout_s=cfg.simulation_timeout
        )
        return reached, elapsed, latency_us, compute_energy_j, events_handled, \
            events_violated, events_violated_deadline, events_violated_preemptive
    finally:
        try:
            if nav is not None:
                nav.shutdown()
        except Exception as e:
            print(f"[hydra][WARN] {strategy_name} nav.shutdown() error: {e}")

        # Stop the publish loop before any potential Gazebo reset/teardown.
        # Without this, the gz-transport publisher and the plugin destructor race to
        # free the same Twist message → double free / SIGABRT.
        try:
            ctrl.pause()
        except Exception as e:
            print(f"[hydra][WARN] ctrl.pause() error: {e}")


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
        json_format=True,
    )
    log_handle = setup_async_logger(logcfg)
    logger = logging.getLogger("hydra_teleop.main")
    print(f"\n=== [LOGGING INITIALIZED] ===")

    pose_node = None
    rosgz_bridge = None
    emitter = None
    ctrl = None
    arena_gen = None
    viol_node = None
    ener_node = None

    if cfg.simulation_runs != 0:
        exec = None
        spin_stop = threading.Event()
        spin_thread = None

        # -----------------------------
        # CSV setup (one file per cfg)
        # -----------------------------
        results_csv_path = cfg.results_csv_path
        csv_fieldnames = [
            "run",
            "strategy",
            "elapse_time",
            "zone_violations",
            "compute_latency_us",
            "compute_energy_j",
            "propulsion_energy_j",   # EPM×distance (Kirschstein model)
            "propulsion_mean_power_w",
            "events_handled",
            "event_violated",
            "event_violated_deadline",
            "event_violated_preemptive",
            "event_violation_rate",
        ]

        # Create/initialize CSV with header if not present
        if not os.path.exists(results_csv_path):
            os.makedirs(os.path.dirname(results_csv_path), exist_ok=True)
            with open(results_csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=csv_fieldnames)
                writer.writeheader()

        try:
            rclpy.init()
            exec = MultiThreadedExecutor(num_threads=3)  # events + nav + pose
            cbg = ReentrantCallbackGroup()

            # Pose republisher node (we attach to shared executor)
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
            rosgz_bridge = start_parameter_bridge(
                [
                    (
                        "/model/drone1/front_lidar/scan",
                        "sensor_msgs/msg/LaserScan",
                        "gz.msgs.LaserScan",
                    ),
                    (
                        "/model/drone1/cmd_vel",
                        "geometry_msgs/msg/Twist",
                        "gz.msgs.Twist",
                    ),
                ]
            )

            # Start event emitter (own internal thread; not an rclpy spin)
            emitter = add_event_emitter_to_executor(
                exec,
                cfg,
                gen_cfg=EventCfg(seed=42, event_deterministic=True),
                callback_group=cbg,
            )
            emitter.start()

            # Velocity publisher
            ctrl = GzTeleop(cfg.topic, cfg)

            # Violation & energy monitors
            viol_node = add_violation_monitor_to_executor(
                exec,
                pose_topic="/model/drone1/pose/info",
                meta_path="models/generated/generated_nofly_meta.json",
                callback_group=cbg,
            )
            ener_node = add_energy_monitor_to_executor(
                exec,
                pose_topic="/model/drone1/pose/info",
                callback_group=cbg,
            )

            # ------------------------------------------------------------------
            # GOOD RUN LOOP:
            #   We keep going until we collect cfg.simulation_runs runs where
            #   *all* strategies have reached == True.
            #
            #   For each attempt/world:
            #     - generate NFZ + target
            #     - run strategies in order
            #     - if any strategy fails (reached == False), abort remaining
            #       strategies *immediately* and discard this attempt.
            #
            #   For each good run, we append rows to results_csv_path.
            # ------------------------------------------------------------------
            good_runs = 0       # number of successful "kept" runs
            attempt_idx = 0     # counts all attempts, including discarded

            while good_runs < cfg.simulation_runs:
                attempt_idx += 1
                run_idx = good_runs + 1  # logical index for "kept" runs

                # Generators (No-fly + Target)
                if cfg.simulation_world_style == "city":
                    from .tools.arena_generator_city import ArenaGenerator, ArenaGenCfg
                else:
                    from .tools.arena_generator_perlin import ArenaGenerator, ArenaGenCfg

                # Use attempt_idx for seed so each attempt is unique
                arena_gen = ArenaGenerator(
                    cfg,
                    ArenaGenCfg(
                        seed= cfg.world_gen_seed_offset + (0 if cfg.fixed_seed else ((attempt_idx - 1) % 100) + 1),
                        target_min_dist=cfg.target_distance,
                        pass_through=True,
                        visual_alpha=0.0,
                        outdir="models/generated",
                    ),
                )

                logger.info(
                    {
                        "simulationruns": cfg.simulation_runs,
                        "seedoffset": cfg.world_gen_seed_offset,
                        "arena": cfg.simulation_world_style,
                        "attempt_idx": attempt_idx,
                        "run_idx": run_idx,
                    }
                )

                # Violation monitor depends on targets/NFZ → generate first
                arena_gen.run()

                print(
                    f"\n=== Hydra Experiment Run {run_idx} "
                    f"(attempt {attempt_idx}, fresh target & NFZ) ==="
                )

                all_reached = True
                buffered_records: List[Dict[str, Any]] = []

                # Boot Gazebo once per attempt — all strategies reuse the same instance.
                # Between strategies we teleport the drone back to start via gz service
                # instead of restarting, saving ~(N_strategies - 1) × 10s per attempt.
                sim = None
                rtf_mon = None
                try:
                    sim = start_sim(cfg)
                    rtf_mon = RtfMonitor(world_name="airport", interval_s=5.0)
                    rtf_mon.start()

                    # Strategies (APE1/APE2/APE3/TROOP) — each run uses same event seed
                    for strategy_idx, strategy in enumerate(cfg.analyzer_strategies):
                        print(f"\n=== Hydra Experiment Strategy {strategy} ===")

                        # Teleport drone back to start before every strategy except the first
                        # (first strategy starts from the freshly booted Gazebo initial pose)
                        if strategy_idx > 0:
                            reset_sim(sim, cfg)

                        emitter.reset()
                        viol_node.mark_run_start(label=strategy)
                        ener_node.mark_run_start(label=strategy)

                        reached, elapsed, latency_us, compute_energy_j, events_handled, \
                            events_violated, events_violated_deadline, events_violated_preemptive = _run_one_strategy(
                            strategy, ctrl, cfg, exec
                        )

                        violation_summary = viol_node.log_and_reset(
                            label=strategy, include_boxes=True
                        )
                        energy_summary = ener_node.log_and_reset(label=strategy)

                        logger.info(
                            {
                                "reached": reached,
                                "elapsed": elapsed,
                                "violations": violation_summary.get("total_violations"),
                                "compute_latency_us": latency_us,
                                "compute_energy_j": compute_energy_j,
                                "energy_j": energy_summary.get("energy_j"),
                                "mean_power_w": energy_summary.get("mean_power_w"),
                                "events_handled": events_handled,
                                "events_violated": events_violated,
                                "events_violated_deadline": events_violated_deadline,
                                "events_violated_preemptive": events_violated_preemptive,
                            },
                            extra={"strategy": strategy},
                        )

                        buffered_records.append(
                            {
                                "strategy": strategy,
                                "reached": reached,
                                "elapsed": elapsed,
                                "violations": violation_summary.get(
                                    "total_violations"
                                ),
                                "compute_latency_us": latency_us,
                                "compute_energy_j": compute_energy_j,
                                "energy_j": energy_summary.get("energy_j"),
                                "mean_power_w": energy_summary.get("mean_power_w"),
                                "events_handled": events_handled,
                                "events_violated": events_violated,
                                "events_violated_deadline": events_violated_deadline,
                                "events_violated_preemptive": events_violated_preemptive,
                            }
                        )

                        if not reached:
                            all_reached = False
                            print(
                                f"❌ Attempt {attempt_idx}, strategy {strategy} failed "
                                f"(reached == False). Aborting remaining strategies "
                                f"for this attempt."
                            )
                            # EARLY EXIT: don't waste time running remaining strategies
                            break

                finally:
                    try:
                        if rtf_mon is not None:
                            rtf_mon.stop()
                    except Exception as e:
                        print(f"[hydra][WARN] rtf_mon.stop() error: {e}")
                    try:
                        stop_sim(sim)
                    except Exception as e:
                        print(f"[hydra][WARN] stop_sim error for attempt {attempt_idx}: {e}")

                # --------- Post-strategy logic: success check + APE ordering ----------
                if all_reached:
                    good_runs += 1
                    print(
                        f"✅ Run {run_idx}: all strategies reached target "
                        f"({good_runs}/{cfg.simulation_runs} good runs)"
                    )

                    # ----------------------------------------------------------
                    # Commit buffered results to CSV instead of logging them.
                    # ----------------------------------------------------------
                    with open(results_csv_path, "a", newline="") as f:
                        writer = csv.DictWriter(f, fieldnames=csv_fieldnames)
                        for rec in buffered_records:
                            writer.writerow(
                                {
                                    "run": run_idx,
                                    "strategy": rec["strategy"],
                                    "elapse_time": round(rec["elapsed"], 2),
                                    "zone_violations": rec["violations"],
                                    "compute_latency_us": round(rec["compute_latency_us"], 2),
                                    "compute_energy_j": round(rec["compute_energy_j"], 2),
                                    "propulsion_energy_j": round(rec["energy_j"], 2),
                                    "propulsion_mean_power_w": round(rec["mean_power_w"], 2),
                                    "events_handled": rec["events_handled"],
                                    "event_violated": rec["events_violated"],
                                    "event_violated_deadline": rec["events_violated_deadline"],
                                    "event_violated_preemptive": rec["events_violated_preemptive"],
                                    "event_violation_rate": round(
                                        rec["events_violated"] / rec["events_handled"]
                                        if rec["events_handled"] else 0.0,
                                        2,
                                    ),
                                }
                            )
                else:
                    print(
                        f"⏭️  Attempt {attempt_idx} discarded: at least one "
                        f"strategy failed (not counting towards "
                        f"{cfg.simulation_runs} good runs)."
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
                    if viol_node is not None:
                        exec.remove_node(viol_node)
                    if ener_node is not None:
                        exec.remove_node(ener_node)
                    # depending on implementation, pose_node.close() may be a no-op
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

    # Post-processing: transform logs and run analyzer
    run_analysis(zone_metric="mean")


if __name__ == "__main__":
    main()