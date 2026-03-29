# config.py
from dataclasses import dataclass
import os

@dataclass
class TeleopConfig:
    # --- Run Options ---
    simulation_runs = 1000
    simulation_timeout = 200
    simulation_world_style = "city" #"perlin|city"
    # --- Sim / world ---
    world_path: str = "/home/vikhyat/Documents/Hydra/worlds/airport_world.sdf"
    sim_cmd: tuple[str, ...] = ("gz", "sim", "-r", "-s")
    sim_env: dict | None = None
    sim_boot_secs: float = 8.0
    fixed_seed = False
    world_gen_seed_offset = 112
    target_distance = 150
    # --- Gazebo transport ---
    topic: str = "/model/drone1/cmd_vel"  # gz.msgs.Twist
    world_pose_topic: str = "/world/airport/pose/info"

    # --- Teleop dynamics ---
    rate_hz: float = 100

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

    analyzer_strategies = ["APE1", "APE2", "APE3", "TROOP"]

    # =======================
    # Generated Simulation & Algo Selector
    # =======================

    # Fixed start pose for selector (no ROS needed for this)
    start_x: float = -95.0
    start_y: float =   0.0
    start_z: float =   1.0

    start_yaw:   float = 0.0  # radians
    # Target file path (from your earlier JSON)
    target_json_path: str = "models/generated/generated_target_meta.json"

    # =======================
    # Event Generator (B-style)
    # =======================
    # Topics
    event_topic = "/hydra/event"
    ros_pose_topic = "/model/drone1/pose/info"

    # -----------------------
    # Mixture of event types
    # -----------------------
    # Defines probabilities for each kind.
    event_mix_enemy = 0.33
    event_mix_obstacle = 0.33
    event_mix_lane = 0.34

    # -----------------------
    # Logging
    # -----------------------
    event_log_csv_path = "logs/events_log.csv"

    # -----------------------
    # Deadline model — AUTHORITATIVE SOURCE
    # Propagated into EventCfg via EventCfg.from_teleop_cfg().
    # Must stay consistent with EventDecisionCfg thresholds in nav_algorithm_T.py.
    #
    # Values derived from APE_LATENCY_US × DEADLINE_SCALE=1000:
    #   APE1 budget: 523ms  — APE1_sleep (523ms) < deadline_min (600ms) → 0% violations
    #   APE2 budget: 1343ms — APE2_sleep (1343ms) within [600ms, 3500ms] → ~61% violations
    #   APE3 budget: 2035ms — APE3_sleep (2035ms) within [600ms, 3500ms] → ~70% violations
    #   TROOP: always falls back to APE1 → 0% violations, APE2/3 quality on ~39% of events
    #
    # Physical grounding at v_max=15m/s:
    #   deadline_min=600ms → 9m   (obstacle at LiDAR safe_m boundary)
    #   deadline_max=3500ms → 52m (far-field threat, ample replanning horizon)
    # -----------------------
    deadline_alpha: float = 0.85
    deadline_min_s: float = 0.70
    deadline_max_s: float = 3.50
    # -----------------------
    # Physics
    # -----------------------
    cmd_latency_s: float = 0.10
    wind_level_0to1: float = 0.5
    wind_accel_std_base_mps2: float = 0.8
    physics_seed: int = 42  # seed for wind RNG; same seed → identical wind gusts across runs