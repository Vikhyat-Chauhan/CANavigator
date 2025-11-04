# config.py
from dataclasses import dataclass
import os
from typing import Optional, Tuple, List

@dataclass
class TeleopConfig:
    # --- Run Options ---
    simulation_runs = 0
    simulation_timeout = 200
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
    rate_hz: float = 30
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
    #analyzer_strategies = ["TROOP"]

    # =======================
    # Generated Simulation & Algo Selector
    # =======================
    targets_json_path = "models/generated/targets.json"  # your file

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
    # Event Generator (B-style)
    # =======================
    # Topics
    event_topic = "/hydra/event"
    ros_pose_topic = "/model/drone1/pose/info"

    # -----------------------
    # Time constraint model
    # -----------------------
    # Event spacing follows a log-uniform distribution between these bounds.
    # The time between two events (Δt) is the *actual constraint*.
    # Short intervals create overlapping / near-deadline pressure on APEs.
    event_dt_min_s = 0.08   # lower bound (tightest events)
    event_dt_max_s = 3.5    # upper bound (slow cycles)

    # Optional single global deadline for legacy consumers.
    # Usually left None (since Δt itself acts as the time constraint).
    event_global_deadline_s = None

    # -----------------------
    # Mixture of event types
    # -----------------------
    # Defines probabilities for each kind.
    event_mix_enemy = 0.33
    event_mix_obstacle = 0.33
    event_mix_lane = 0.34

    # -----------------------
    # Arena bounds
    # -----------------------
    event_bounds_xy = (-100.0, 100.0, -50.0, 50.0)

    # -----------------------
    # Logging
    # -----------------------
    event_log_csv_path = "logs/events_log.csv"
    
    deadline_alpha: float = 0.85
    deadline_min_s: float = 0.12
    deadline_max_s: float = 1.20
    # -----------------------
    # Physics
    # -----------------------
    # Deterministic
    cmd_latency_s: float = 0.0 #0.10
    wind_level_0to1: float = 0.0 #0.5
    wind_accel_std_base_mps2: float = 0.0 #0.8
    '''
    # Non Deterministic
    cmd_latency_s: float = 0.10
    wind_level_0to1: float = 0.5
    wind_accel_std_base_mps2: float = 0.8  '''
