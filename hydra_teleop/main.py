#!/usr/bin/env python3
import time
import math
import csv

# Headless plotting
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy

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


# ------------ helpers: record, aggregate, plot ------------

def _record(stats, run_idx, strategy, reached, elapsed, violations):
    stats.append({
        "run": run_idx,
        "strategy": strategy,
        "reached": bool(reached),
        "elapsed_s": float(elapsed) if elapsed is not None else float("nan"),
        "violations": int(violations),
    })

def _aggregate_by_strategy(stats):
    agg = {}
    for row in stats:
        s = row["strategy"]
        d = agg.setdefault(s, {"n": 0, "sum_elapsed": 0.0, "sum_viol": 0.0, "sum_reached": 0})
        if not math.isnan(row["elapsed_s"]):
            d["sum_elapsed"] += row["elapsed_s"]
        d["sum_viol"] += row["violations"]
        d["sum_reached"] += int(row["reached"])
        d["n"] += 1

    rows = []
    for s, d in agg.items():
        n = max(d["n"], 1)
        rows.append({
            "strategy": s,
            "n_runs": d["n"],
            "avg_elapsed_s": d["sum_elapsed"] / n,
            "avg_violations": d["sum_viol"] / n,
            "reach_rate": d["sum_reached"] / n,
        })
    rows.sort(key=lambda r: r["strategy"])
    return rows

def _plot_violations_vs_time(agg_rows, out_prefix="hydra"):
    x = [r["avg_elapsed_s"] for r in agg_rows]
    y = [r["avg_violations"] for r in agg_rows]
    labels = [r["strategy"] for r in agg_rows]

    plt.figure(figsize=(6, 4))
    plt.scatter(x, y)
    for xi, yi, lbl in zip(x, y, labels):
        plt.annotate(lbl, (xi, yi), textcoords="offset points", xytext=(6, 6))
    plt.xlabel("Average elapsed time (s)")
    plt.ylabel("Average violations (count)")
    plt.title("Average Violations vs Average Time by Strategy")
    plt.tight_layout()
    out_path = f"{out_prefix}_avg_violations_vs_time.png"
    plt.savefig(out_path, dpi=150)
    plt.close()
    return out_path

def _save_artifacts(stats, out_prefix="hydra"):
    # Raw per-run CSV
    csv_path = f"{out_prefix}_results.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["run", "strategy", "reached", "elapsed_s", "violations"])
        writer.writeheader()
        writer.writerows(stats)
    print(f"[hydra] Wrote {csv_path}")

    # Aggregated averages per strategy
    agg_rows = _aggregate_by_strategy(stats)
    agg_csv = f"{out_prefix}_results_agg.csv"
    with open(agg_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["strategy", "n_runs", "avg_elapsed_s", "avg_violations", "reach_rate"])
        writer.writeheader()
        writer.writerows(agg_rows)
    print(f"[hydra] Wrote {agg_csv}")

    # Scatter: avg violations vs avg time
    scatter_path = _plot_violations_vs_time(agg_rows, out_prefix=out_prefix)
    print(f"[hydra] Wrote {scatter_path}")


# ---------------- main experiment ----------------

def main():
    # --- Boot sim infra once ---
    cfg = TeleopConfig()
    pose_bridge = start_pose_bridge(["drone1", "target_sphere"])
    rosgz_bridge = start_parameter_bridge([
        ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
        ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
    ])

    ctrl = GzTeleop(cfg.topic, cfg)
    ctrl.start()

    # ---------- Generate NO-FLY + TARGET ONCE so all runs share the same map ----------
    nofly = NoFlyGenerator(cfg, NoFlyGenCfg())
    nofly.run()
    target = TargetGenerator(cfg, TargetGenCfg())
    target.run()
    # ----------------------------------------------------------------------------------
    # Count violations across runs (reset between each run)
    violation_monitor = start_violation_monitor(zone_padding_m=0.3)
    
    sim = start_sim(cfg)
    #time.sleep(100)
    stop_sim(sim)
    stats = []
    N_REPEATS = 1

    for i in range(N_REPEATS):
        target = TargetGenerator(cfg, TargetGenCfg())
        target.run()
        run_idx = i + 1
        print(f"\n=== Hydra Experiment Run {run_idx} (same map) ===")
        time.sleep(0.5)

        # ---------- 1) SIMPLE NAV ----------
        sim = start_sim(cfg)
        simple_navigator = TargetNavigator(ctrl, cfg)
        try:
            reached, elapsed = simple_navigator.go_to()
            violations = violation_monitor.get_total()
            _record(stats, run_idx, "Basic", reached, elapsed, violations)
            print(f"[hydra] Basic: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {math.floor(elapsed)} s")
        finally:
            simple_navigator.shutdown()
            violation_monitor.reset()
            stop_sim(sim)

        # ---------- 2) LIDAR: SAFE (slower/safer) ----------
        sim = start_sim(cfg)
        lidar_navigator = LidarTargetNavigator(
            ctrl, cfg,
            goto_cfg=GoToCfgLidar(
                goal_radius_m=4.0,
                kp_lin=1.2, kp_z=1.0, kp_yaw=2.0,
                max_v=15.0,   # m/s (DJI route speed)
                max_vz=3.5,   # m/s ascent cap
                max_wz=1.4,   # rad/s (~80°/s), conservative
                slow_yaw_threshold=1.0,
                rate_hz=60.0,
            ),
            avoid_cfg=AvoidCfg(
                safe_m=5.0,
                hysteresis_m=1.0,
                front_deg=5.0,
                side_deg=30.0,
                side_center_deg=30.0,
                turn_rate=0.9,
                watchdog_sec=0.6,
                min_turn_sec=0.7,
            )
        )
        try:
            reached, elapsed = lidar_navigator.go_to()
            violations = violation_monitor.get_total()
            _record(stats, run_idx, "LidarSafe", reached, elapsed, violations)
            print(f"[hydra] LidarSafe: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {math.floor(elapsed)} s")
        finally:
            lidar_navigator.shutdown()
            violation_monitor.reset()
            stop_sim(sim)

        # ---------- 3) LIDAR: FAST (more aggressive) ----------
        sim = start_sim(cfg)
        lidar_navigator = LidarTargetNavigator(
            ctrl, cfg,
            goto_cfg=GoToCfgLidar(
                goal_radius_m=4.0,
                kp_lin=1.2, kp_z=1.0, kp_yaw=2.0,
                max_v=20.0, max_vz=5, max_wz=1.6,
                slow_yaw_threshold=1.0, rate_hz=60.0,
            ),
            avoid_cfg=AvoidCfg(
                safe_m=3.0,
                hysteresis_m=0.8,
                front_deg=5.0,
                side_deg=30.0,
                side_center_deg=30.0,
                turn_rate=1.2,
                watchdog_sec=0.6,
                min_turn_sec=0.5,
            )
        )
        try:
            reached, elapsed = lidar_navigator.go_to()
            violations = violation_monitor.get_total()
            _record(stats, run_idx, "LidarFast", reached, elapsed, violations)
            print(f"[hydra] LidarFast: {'reached' if reached else 'timeout'} | "
                  f"{violations} violations | {math.floor(elapsed)} s")
        finally:
            lidar_navigator.shutdown()
            violation_monitor.reset()
            stop_sim(sim)

    # Cleanup bridges
    pose_bridge.stop()
    rosgz_bridge.stop()

    # Save results & averaged scatter
    _save_artifacts(stats, out_prefix="hydra")


if __name__ == "__main__":
    main()
