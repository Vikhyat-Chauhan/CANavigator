# Configuration Reference

All configuration is centralized in [`hydra_teleop/config.py`](../hydra_teleop/config.py) via the `TeleopConfig` dataclass. This document lists every parameter with its default value, type, and description.

---

## Simulation

| Parameter | Type | Default | Description |
|---|---|---|---|
| `simulation_runs` | `int` | `1000` | Number of successful (good) runs to collect before stopping |
| `simulation_timeout` | `int` | `200` | Per-strategy timeout in seconds; navigation aborts if exceeded |
| `simulation_world_style` | `str` | `"city"` | Arena generation style: `"city"` (grid blocks) or `"perlin"` (noise-based) |
| `fixed_seed` | `bool` | `False` | If `True`, all attempts use the same world seed (for debugging) |
| `world_path` | `str` | `"worlds/airport_world.sdf"` | Path to the base Gazebo world SDF file |
| `sim_cmd` | `tuple[str, ...]` | `("gz", "sim", "-r", "-s")` | Command to launch Gazebo (`-s` = headless) |
| `sim_env` | `dict \| None` | Auto-configured | Environment variables for Gazebo (GPU, display settings) |
| `sim_boot_secs` | `float` | `8.0` | Seconds to wait for Gazebo to fully initialize |

---

## World Generation

| Parameter | Type | Default | Description |
|---|---|---|---|
| `world_gen_seed_offset` | `int` | `112` | Base offset added to attempt index for world RNG seed |
| `target_distance` | `int` | `150` | Minimum Euclidean distance (meters) from start to target |
| `target_json_path` | `str` | `"models/generated/generated_target_meta.json"` | Path to the generated target position JSON |

---

## Gazebo Transport

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic` | `str` | `"/model/drone1/cmd_vel"` | Gazebo topic for velocity commands (Twist messages) |
| `world_pose_topic` | `str` | `"/world/airport/pose/info"` | Gazebo topic for all model poses |

---

## Teleop Dynamics

| Parameter | Type | Default | Description |
|---|---|---|---|
| `rate_hz` | `float` | `100` | Control loop frequency in Hz |

---

## Start Pose

| Parameter | Type | Default | Description |
|---|---|---|---|
| `start_x` | `float` | `-95.0` | Drone start position X (meters) |
| `start_y` | `float` | `0.0` | Drone start position Y (meters) |
| `start_z` | `float` | `1.0` | Drone start position Z (meters) |
| `start_yaw` | `float` | `0.0` | Drone start yaw (radians) |

---

## Event Generation

### Deadline Model

| Parameter | Type | Default | Description |
|---|---|---|---|
| `deadline_alpha` | `float` | `0.85` | Deadline fraction: `deadline = alpha * delta_t` |
| `deadline_min_s` | `float` | `0.70` | Minimum deadline clamp (seconds) |
| `deadline_max_s` | `float` | `3.50` | Maximum deadline clamp (seconds) |

At `v_max = 15 m/s`: `deadline_min = 0.70s` → ~10.5 m reaction distance; `deadline_max = 3.50s` → ~52.5 m far-field horizon.

### Event Mix

| Parameter | Type | Default | Description |
|---|---|---|---|
| `event_mix_enemy` | `float` | `0.33` | Probability of enemy-type events |
| `event_mix_obstacle` | `float` | `0.33` | Probability of obstacle-type events |
| `event_mix_lane` | `float` | `0.34` | Probability of lane-type events |

### Topics

| Parameter | Type | Default | Description |
|---|---|---|---|
| `event_topic` | `str` | `"/hydra/event"` | ROS 2 topic for publishing event JSON payloads |
| `ros_pose_topic` | `str` | `"/model/drone1/pose/info"` | ROS 2 topic for drone pose (used by event emitter) |

---

## Physics

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cmd_latency_s` | `float` | `0.10` | Command-to-actuation delay (FIFO buffer depth) |
| `wind_level_0to1` | `float` | `0.5` | Wind intensity scaling factor (0 = no wind, 1 = maximum) |
| `wind_accel_std_base_mps2` | `float` | `0.8` | Base standard deviation of wind acceleration (m/s²) |
| `physics_seed` | `int` | `42` | RNG seed for wind disturbance; same seed → identical gusts across strategies |

---

## Logging

| Parameter | Type | Default | Description |
|---|---|---|---|
| `log_path` | `str` | `"logs/run_logs.json"` | Path for structured JSON event logs |
| `event_log_csv_path` | `str` | `"logs/events_log.csv"` | Path for event-specific CSV log |

---

## Results & Analysis

| Parameter | Type | Default | Description |
|---|---|---|---|
| `results_csv_path` | `str` | `"logs/results/experiment_summary.csv"` | Path for the main experiment results CSV |
| `analyzer_out_dir` | `str` | `"logs/results"` | Directory for analysis outputs (summaries) |
| `analyzer_strategies` | `list[str]` | `["APE1", "APE2", "APE3", "TROOP"]` | Strategy names to evaluate and compare |

---

## Navigation (GoToConfig)

Defined in `nav_algorithm_T.py` as `GoToConfig`:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `goal_radius_m` | `float` | `4.0` | Distance threshold to consider target reached |
| `kp_lin` | `float` | `1.2` | Proportional gain for linear velocity |
| `kp_z` | `float` | `1.0` | Proportional gain for altitude control |
| `kp_yaw` | `float` | `2.0` | Proportional gain for yaw control |
| `max_v` | `float` | `15.0` | Maximum linear velocity (m/s) |
| `max_vz` | `float` | `3.5` | Maximum vertical velocity (m/s) |
| `max_wz` | `float` | `1.4` | Maximum yaw rate (rad/s) |
| `slow_yaw_threshold` | `float` | `1.0` | Yaw error threshold for speed reduction |
| `rate_hz` | `float` | `30.0` | Navigation loop frequency (Hz) |
| `edge_guard_m` | `float` | `4.5` | NFZ edge guard distance (meters) |
| `edge_guard_scale` | `float` | `0.6` | Speed scale factor near NFZ edges |

---

## Obstacle Avoidance (AvoidCfg)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `scan_topic` | `str` | `"/model/drone1/front_lidar/scan"` | LiDAR scan ROS 2 topic |
| `safe_m` | `float` | `5.0` | Start avoiding when front distance < this |
| `hysteresis_m` | `float` | `1.0` | Extra clearance needed to exit avoidance |
| `front_deg` | `float` | `5.0` | Front window half-width (degrees) |
| `side_deg` | `float` | `30.0` | Side window half-width (degrees) |
| `side_center_deg` | `float` | `30.0` | Side windows centered at +/- this angle |
| `turn_rate` | `float` | `0.9` | Turning rate during avoidance (rad/s) |
| `watchdog_sec` | `float` | `0.6` | Soft stale LiDAR threshold |
| `hard_stale_sec` | `float` | `1.2` | Hard stale LiDAR threshold (triggers brake) |
| `min_turn_sec` | `float` | `0.7` | Minimum commit time for turn direction |

---

## Safety (SafetyCfg)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ambiguity_eps_m` | `float` | `0.5` | Clearance difference threshold for tie-breaking |
| `ttc_soft_s` | `float` | `2.2` | Time-to-collision for soft braking |
| `ttc_hard_s` | `float` | `1.4` | Time-to-collision for hard speed clamp |
| `v_min_frac` | `float` | `0.20` | Minimum speed as fraction of max (during escape) |

---

## Breadcrumbs (BreadcrumbCfg)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cell_xy_m` | `float` | `2.0` | Horizontal cell size for visited-space tracking |
| `cell_z_m` | `float` | `2.0` | Vertical cell size for visited-space tracking |
| `capacity` | `int` | `3000` | Maximum number of breadcrumb entries |
