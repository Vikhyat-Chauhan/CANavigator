# config.py
from dataclasses import dataclass
import os
from typing import Optional, Tuple, List

@dataclass
class TeleopConfig:
    # --- Sim / world ---
    world_path: str = "/home/vikhyat-chauhan/Documents/Hydra/worlds/airport_world.sdf"
    sim_cmd: tuple[str, ...] = ("gz", "sim", "-r")  # GUI
    #sim_cmd: tuple[str, ...] = ("gz", "sim", "4", "-s", "-r")  # headless
    sim_env: dict | None = None
    sim_boot_secs: float = 8.0
    # --- Gazebo transport ---
    topic: str = "/model/drone1/cmd_vel"  # gz.msgs.Twist
    world_pose_topic: str = "/world/airport/pose/info"

    # --- Teleop dynamics ---
    rate_hz: float = 30.0
    speed_x: float = 5.0
    speed_y: float = 5.0
    speed_z: float = 5.0
    yaw_rate: float = 0.8

    # --- Logging ---
    log_path: str = "logs/run_logs.json"

    def __post_init__(self):
        if self.sim_env is None:
            self.sim_env = {
                **os.environ,
                "QT_QPA_PLATFORM": "xcb",
                "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
                "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            }

    # --- Teleop momentum controls ---
    tau_rise_lin_s: float = 0.20
    tau_fall_lin_s: float = 0.35
    max_lin_accel_mps2: float = 5.0
    lin_drag_per_s: float = 0.30

    tau_rise_ang_s: float = 0.20
    tau_fall_ang_s: float = 0.30
    max_ang_accel_rps2: float = 4.0
    ang_drag_per_s: float = 0.40

    max_lin_speed_mps: float = 8.0

    # =======================
    # RESULTS & ANALYZER CFG
    # =======================

    # Where main experiment writes the consolidated results CSV:
    results_csv_path: str = "logs/results/experiment_summary.csv"

    # Analyzer output directory (plots + summaries will go here):
    analyzer_out_dir: str = "logs/results"

    # Optional explicit strategy names (exactly two); leave None to auto-detect
    analyzer_pair_order: Optional[Tuple[str, str]] = None
    # e.g. analyzer_pair_order = ("NAVALGO1", "NAVALGO2")

    # Plot toggles
    analyzer_annotate_points: bool = False
    # If True, keep runs where either strategy timed out; if False, drop them
    analyzer_include_timeouts: bool = True
    analyzer_strategies = ["APE1", "APE2", "APE3", "TROOP"]

    # =======================
    # Generated Simulation & Algo Selector
    # =======================
    targets_json_path = "models/generated/targets.json"  # your file
    target_index = 0                                     # or None if single target
    # OR:
    target_key = "target"                                # if your json has {"target": {...}}

    time_budget_s = 60                                   # optional
    n_repeats = 5
    timeout_s = 500

    # Fixed start pose for selector (no ROS needed for this)
    start_x: float = -95.0
    start_y: float =   0.0
    start_z: float =   1.0

    # (optional, for completeness if any code uses orientation)
    start_roll:  float = 0.0
    start_pitch: float = 0.0
    start_yaw:   float = 0.0  # radians
    # Target file path (from your earlier JSON)
    target_json_path: str = "models/generated/generated_target_meta.json"

    # Optional: experiment knobs
    n_repeats: int = 1
    timeout_s: int = 500
    time_budget_s: float | None = None  # set a number if you want a budget

    # =======================
    # Event Generator
    # =======================
    # Topics
    event_topic = "/hydra/event"
    ros_pose_topic = "/model/drone1/pose/info"

    # Reproducibility (optional)
    event_seed = 42

    # Deadlines — tight & clamped (CLAMP IS ENABLED in your emitter)
    event_deadline_scale = 0.62   # global squeeze; makes everyone work harder
    event_deadline_jitter_s = 0.03
    event_default_deadline_s = 0.16
    event_min_deadline_s = 0.09
    event_max_deadline_s = 0.60

    # Per-kind deadlines (hardest first)
    # Lane & Obstacle are intentionally tighter → APE3 struggles most here.
    event_lane_deadline_s     = 0.12
    event_obstacle_deadline_s = 0.13
    event_enemy_deadline_s    = 0.18   # a bit more forgiving
    # Arena bounds
    event_bounds_xy = (-100.0, 100.0, -50.0, 50.0)

    # Enemy
    event_enemy_speed_min = 1.0
    event_enemy_speed_max = 4.0
    event_enemy_radius_min = 1.0
    event_enemy_radius_max = 3.0
    event_enemy_cross_dist_ahead = 15.0
    event_enemy_cross_lateral = 8.0

    # Sudden obstacle
    event_obstacle_radius_min = 1.0
    event_obstacle_radius_max = 2.5
    event_obstacle_ahead_min = 8.0
    event_obstacle_ahead_max = 20.0
    event_obstacle_side_span = 10.0

    # Lane block
    event_lane_w_min = 6.0
    event_lane_w_max = 12.0
    event_lane_h_min = 4.0
    event_lane_h_max = 10.0
    event_lane_ahead_min = 15.0
    event_lane_ahead_max = 35.0

    # Phases — increasing arrival rate + burstiness so late windows are toughest.
    # Mix leans a bit toward LANE_BLOCK (hardest) and keeps some OBSTACLE.
    event_phases = [
        # Warm-ish: mostly solvable for APE1/APE2; APE3 starts to miss.
        {"duration_s": 40.0, "rate_hz": 0.14,
        "mix_enemy": 0.30, "mix_obstacle": 0.30, "mix_lane": 0.40,
        "burstiness": 0.00},

        # Harder: more arrivals + mild burst clusters → more tight overlaps.
        {"duration_s": 40.0, "rate_hz": 0.22,
        "mix_enemy": 0.28, "mix_obstacle": 0.30, "mix_lane": 0.42,
        "burstiness": 0.30},

        # Spicy: high rate + burstiness → many deadlines tight while planning.
        {"duration_s": 40.0, "rate_hz": 0.30,
        "mix_enemy": 0.25, "mix_obstacle": 0.30, "mix_lane": 0.45,
        "burstiness": 0.55},
    ]

    # Logging
    event_log_csv_path = "events_log.csv"
    