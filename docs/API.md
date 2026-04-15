# API Reference

Module-level documentation for all public classes, functions, and interfaces in the CANavigator framework.

---

## Table of Contents

- [ca_navigator.main](#ca_navigatormain)
- [ca_navigator.config](#ca_navigatorconfig)
- [ca_navigator.navigation](#ca_navigatornavigation)
  - [nav_algorithm_T](#nav_algorithm_tpy)
  - [teleop](#teleoppy)
  - [transport](#transportpy)
- [ca_navigator.simulation](#ca_navigatorsimulation)
  - [sim](#simpy)
  - [physics](#physicspy)
  - [pose_republisher](#pose_republisherpy)
- [ca_navigator.tools](#ca_navigatortools)
  - [event_emitter](#event_emitterpy)
  - [violations](#violationspy)
  - [energy_monitor](#energy_monitorpy)
  - [arena_generator_city](#arena_generator_citypy)
  - [arena_generator_perlin](#arena_generator_perlinpy)
  - [bridge](#bridgepy)
- [ca_navigator.logging](#ca_navigatorlogging)
  - [async_logger](#async_loggerpy)
- [ca_navigator.analysis](#ca_navigatoranalysis)
  - [statistics_analyzer](#statistics_analyzerpy)
  - [log_transformer](#log_transformerpy)

---

## ca_navigator.main

Entry point for the CANavigator experiment framework.

### `main() -> None`

Orchestrates the full experiment lifecycle:

1. Initializes configuration, logging, and ROS 2 runtime
2. Creates all middleware components (bridges, emitter, monitors)
3. Runs the good-run loop collecting `cfg.simulation_runs` successful runs
4. Tears down all components
5. Runs post-experiment analysis

### `_run_one_strategy(strategy_name, ctrl, cfg, exec) -> tuple`

Runs a single navigation strategy within the currently running Gazebo instance.

**Parameters:**
- `strategy_name` (`str`): One of `"APE1"`, `"APE2"`, `"APE3"`, `"CA"`
- `ctrl` (`GzTeleop`): Velocity controller instance
- `cfg` (`TeleopConfig`): Configuration
- `exec` (`MultiThreadedExecutor`): ROS 2 executor

**Returns:** `(reached: bool, elapsed_s: float, latency_us: float, compute_energy_j: float, events_handled: int, events_violated: int, events_violated_deadline: int, events_violated_preemptive: int)`

---

## ca_navigator.config

### `class TeleopConfig`

Centralized configuration dataclass. All experiment parameters are defined as class or instance attributes with sensible defaults.

See [CONFIGURATION.md](CONFIGURATION.md) for the complete parameter reference.

---

## ca_navigator.navigation

### nav_algorithm_T.py

#### `class LidarTargetNavigatorCA`

The primary navigation engine implementing deadline-aware APE selection with LiDAR-based obstacle avoidance.

**Constructor:**
```python
LidarTargetNavigatorCA(
    ctrl: GzTeleop,
    cfg: TeleopConfig,
    strategy: str,
    ape3_select_threshold_ms: int | None = None,
)
```

- `ctrl`: Velocity controller for sending commands
- `cfg`: Global configuration
- `strategy`: Which planning mode to use (`"APE1"`, `"APE2"`, `"APE3"`, or `"CA"`)
- `ape3_select_threshold_ms`: Override for the APE3 selection threshold; `None` uses `EventDecisionCfg` defaults. CA uses `2589` (APE3 budget + 554ms safety margin).

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `attach_to_executor` | `(exec: MultiThreadedExecutor) -> None` | Register ROS 2 subscriptions with the executor |
| `go_to` | `(target_xyz=None, timeout_s=None) -> tuple` | Navigate to target; returns `(reached, elapsed_s, latency_us, compute_energy_j, events_handled, events_violated, events_violated_deadline, events_violated_preemptive)` |
| `shutdown` | `() -> None` | Clean up all subscriptions and internal state |

#### `class GoToConfig`

Navigation tuning parameters (dataclass). See [CONFIGURATION.md](CONFIGURATION.md#navigation-gotoconfig).

#### `class AvoidCfg`

LiDAR obstacle avoidance parameters (dataclass). See [CONFIGURATION.md](CONFIGURATION.md#obstacle-avoidance-avoidcfg).

#### `class BreadcrumbCfg`

Visited-space tracking parameters (dataclass).

#### `class SafetyCfg`

Safety subsystem parameters (dataclass).

#### `class EventDecisionCfg`

APE timing thresholds (dataclass).

| Field | Type | Default | Description |
|---|---|---|---|
| `ape1_budget_ms` | `int` | `523` | APE1 compute budget (ms) |
| `ape2_budget_ms` | `int` | `1343` | APE2 compute budget (ms) |
| `ape2_select_threshold_ms` | `int` | `1393` | APE2 budget + 50ms safety margin |
| `ape3_budget_ms` | `int` | `2035` | APE3 compute budget (ms) |

---

### teleop.py

#### `class GzTeleop`

Velocity teleop with physics-based motion shaping.

**Constructor:**
```python
GzTeleop(topic: str, cfg: TeleopConfig)
```

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `start` | `() -> None` | Begin the velocity publishing loop |
| `set_cmd` | `(vx, vy, vz, wz) -> None` | Set desired velocity setpoint (thread-safe) |
| `publish_once` | `(linear: tuple, angular: tuple) -> None` | Send a single one-shot command (bypasses physics shaping) |
| `pause` | `() -> None` | Stop publish loop and zero velocity; counterpart to `start()` |
| `stop` | `() -> None` | Zero velocity setpoint and reset physics state |
| `shutdown` | `(join_timeout=0.3) -> None` | Full cleanup: zero command, final publish, stop loop |

---

### transport.py

#### `class GzVelPub`

Low-level Gazebo transport publisher for Twist messages.

**Constructor:**
```python
GzVelPub(topic: str = "/model/drone1/cmd_vel")
```

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `send` | `(linear: tuple, angular: tuple) -> None` | Publish a Twist message via Gazebo transport |

---

## ca_navigator.simulation

### sim.py

#### `start_sim(cfg: TeleopConfig) -> subprocess.Popen`

Launch a Gazebo simulation instance with the configured world and environment.

**Returns:** The Gazebo subprocess handle.

#### `stop_sim(proc: subprocess.Popen) -> None`

Gracefully terminate a Gazebo instance, falling back to SIGKILL if needed.

#### `reset_sim(proc: subprocess.Popen, cfg: TeleopConfig) -> None`

Teleport the drone back to its start pose via `gz service` without restarting Gazebo.

#### `kill_process_tree(pid: int) -> None`

Kill a process and all its children recursively.

---

### physics.py

#### `class DronePhysics`

FlyCart 30-tuned physics model for velocity shaping.

**Constructor:**
```python
DronePhysics(cfg: TeleopConfig)
```

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `update_cmd` | `(vx, vy, vz, wz) -> None` | Push a new desired velocity into the latency buffer |
| `step` | `(dt: float) -> tuple[float, float, float, float]` | Advance physics by `dt` seconds; returns shaped `(vx, vy, vz, wz)` |
| `reset` | `() -> None` | Zero all internal states and buffers |

**Physics Pipeline:**
1. Command latency (FIFO buffer, depth = `cmd_latency_s`)
2. 2nd-order actuator dynamics (per-axis)
3. Jerk limiting (acceleration slew-rate)
4. Tilt/thrust caps (lateral acceleration, asymmetric vertical)
5. Aerodynamic drag (linear + quadratic)
6. Wind gusts (Ornstein-Uhlenbeck process, seeded by `physics_seed`)

---

### pose_republisher.py

#### `class MultiWorldPoseRepublisher`

Bridges Gazebo world-level pose data to individual ROS 2 PoseStamped topics.

#### `add_pose_republisher_to_executor(exec, model_names, callback_group) -> Node`

Factory function that creates a republisher node, adds it to the executor, and returns it.

**Parameters:**
- `exec` (`MultiThreadedExecutor`): ROS 2 executor
- `model_names` (`list[str]`): Models to track (e.g., `["drone1", "target_sphere"]`)
- `callback_group`: ROS 2 callback group

---

## ca_navigator.tools

### event_emitter.py

#### `class EventCfg`

Configuration for event generation.

| Field | Type | Default | Description |
|---|---|---|---|
| `seed` | `int` | `42` | RNG seed for deterministic replay |
| `event_deterministic` | `bool` | `True` | If `True`, use seeded RNG; if `False`, use system entropy |

#### `class EventEmitter`

ROS 2 node that generates and publishes events on `/ca_navigator/event`.

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `start` | `() -> None` | Begin event generation thread |
| `stop` | `() -> None` | Stop event generation |
| `reset` | `() -> None` | Reset RNG to initial seed (for same-sequence replay across strategies) |

#### `add_event_emitter_to_executor(exec, cfg, gen_cfg, callback_group) -> EventEmitter`

Factory function.

---

### violations.py

#### `class ViolationMonitor`

ROS 2 node that tracks drone entries into no-fly zone rectangles.

State machine per NFZ rectangle: `armed → inside → outside → armed → ...`

One violation is counted per entry (not per tick inside).

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `mark_run_start` | `(label: str) -> None` | Reset counters for a new strategy run |
| `log_and_reset` | `(label: str, include_boxes: bool) -> dict` | Return violation summary and reset |

#### `add_violation_monitor_to_executor(exec, pose_topic, meta_path, callback_group) -> ViolationMonitor`

Factory function.

---

### energy_monitor.py

#### `class EnergyMonitor`

ROS 2 node that estimates motion energy using the Energy-Per-Meter (EPM) model.

Default EPM: **208.9 J/m** (DJI FlyCart 30 baseline).

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `mark_run_start` | `(label: str) -> None` | Reset accumulator for a new strategy run |
| `log_and_reset` | `(label: str, include_params: bool) -> dict` | Return energy summary (`energy_j`, `mean_power_w`) and reset |

#### `add_energy_monitor_to_executor(exec, pose_topic, callback_group) -> EnergyMonitor`

Factory function.

---

### arena_generator_city.py

#### `class ArenaGenCfg`

Configuration for arena generation.

| Field | Type | Default | Description |
|---|---|---|---|
| `seed` | `int` | required | RNG seed for this arena |
| `target_min_dist` | `float` | required | Minimum start-to-target distance |
| `pass_through` | `bool` | `True` | Allow pass-through (non-solid) NFZ visuals |
| `visual_alpha` | `float` | `0.0` | Visual transparency (0 = invisible, 1 = opaque) |
| `outdir` | `str` | `"models/generated"` | Output directory for SDF + JSON |

#### `class ArenaGenerator`

Generates city-grid style no-fly zones with streets, blocks, lots, and buildings.

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `run` | `() -> None` | Generate NFZ SDF model, target sphere SDF, and metadata JSON files |

**Output files:**
- `models/generated/generated_nofly.sdf` -- NFZ obstacle model
- `models/generated/generated_nofly_meta.json` -- List of NFZ rectangles `[{x, y, w, h}, ...]`
- `models/generated/generated_target.sdf` -- Target sphere model
- `models/generated/generated_target_meta.json` -- Target position `{x, y, z}`

---

### arena_generator_perlin.py

#### `class ArenaGenerator`

Generates Perlin-noise based no-fly zone layouts. Same interface as the city generator.

---

### bridge.py

#### `start_parameter_bridge(mappings: list[tuple]) -> RosGzBridge`

Start the `ros_gz_bridge parameter_bridge` process with the given topic mappings.

**Parameters:**
- `mappings`: List of `(topic, ros_type, gz_type)` tuples

**Returns:** A `RosGzBridge` handle with a `.stop()` method.

**Example:**
```python
bridge = start_parameter_bridge([
    ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
    ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
])
# later:
bridge.stop()
```

---

## ca_navigator.logging

### async_logger.py

#### `class AsyncLoggerCfg`

Configuration for the async logging system.

| Field | Type | Default | Description |
|---|---|---|---|
| `logfile` | `str` | required | Output file path |
| `max_bytes` | `int` | `0` | Max log file size (0 = no rotation) |
| `queue_maxsize` | `int` | `8000` | Max entries in log queue |
| `drop_on_full` | `bool` | `False` | Drop messages when queue is full vs. block |
| `console` | `bool` | `False` | Also print to stdout |
| `level` | `int` | `logging.INFO` | Minimum log level |
| `monitor_interval_s` | `float \| None` | `None` | Unused; accepted for API compatibility |
| `json_format` | `bool` | `True` | Use JSON formatting for log entries |

#### `setup_async_logger(cfg: AsyncLoggerCfg) -> LogHandle`

Initialize the async logging system. Returns a handle with a `.stop()` method that flushes and closes the logger.

---

## ca_navigator.analysis

### statistics_analyzer.py

#### `run_analysis(zone_metric: str = "mean") -> dict`

Run statistical analysis on the experiment CSV results. Reads `cfg.results_csv_path`; writes summary CSV to `cfg.analyzer_out_dir`.

**Parameters:**
- `zone_metric`: Aggregation method for zone violations (`"mean"` or `"median"`)

**Returns:** Dictionary with keys:
- `zone_metric`: The zone metric used
- `summary_csv`: Path to the generated summary CSV
- `summary`: Aggregated statistics dictionary keyed by strategy name

---

### log_transformer.py

Standalone utility for converting JSON log files to CSV format. Not called by the main experiment loop.

#### `run_from_cfg() -> None`

Transform JSON log files into CSV format using paths from `TeleopConfig`.

#### `transform(input_path: str, output_path: str) -> None`

Convert a single JSON log file to CSV format.
