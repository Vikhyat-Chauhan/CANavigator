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
    log_to_csv: bool = False
    log_dir: str = "logs"
    log_name: str = "teleop"
    log_headers: list[str] | None = None

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
    results_csv_path: str = "logs/results/hydra_results.csv"

    # Whether to automatically run the analyzer after main finishes:
    run_analyzer_after: bool = True

    # Analyzer output directory (plots + summaries will go here):
    analyzer_out_dir: str = "logs/results"

    # Optional explicit strategy names (exactly two); leave None to auto-detect
    analyzer_pair_order: Optional[Tuple[str, str]] = None
    # e.g. analyzer_pair_order = ("NAVALGO1", "NAVALGO2")

    # Plot toggles
    analyzer_annotate_points: bool = False
    # If True, keep runs where either strategy timed out; if False, drop them
    analyzer_include_timeouts: bool = False
    analyzer_strategies = ["NAVALGO1", "NAVALGO2", "NAVALGO3"]

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