# API Reference

Module-level documentation for all public classes, functions, and interfaces in the Hydra framework.

---

## Table of Contents

- [hydra_teleop.main](#hydra_teleopmain)
- [hydra_teleop.config](#hydra_teleopconfig)
- [hydra_teleop.navigation](#hydra_teleopnavigation)
  - [nav_algorithm_T](#nav_algorithm_tpy)
  - [teleop](#teleoppy)
  - [transport](#transportpy)
- [hydra_teleop.simulation](#hydra_teleopsimulation)
  - [sim](#simpy)
  - [physics](#physicspy)
  - [pose_republisher](#pose_republisherpy)
- [hydra_teleop.tools](#hydra_teleoptools)
  - [event_emitter](#event_emitterpy)
  - [violations](#violationspy)
  - [energy_monitor](#energy_monitorpy)
  - [arena_generator_city](#arena_generator_citypy)
  - [arena_generator_perlin](#arena_generator_perlinpy)
  - [bridge](#bridgepy)
- [hydra_teleop.logging](#hydra_teleoplogging)
  - [async_logger](#async_loggerpy)
- [hydra_teleop.analysis](#hydra_teleopanalysis)
  - [statistics_analyzer](#statistics_analyzerpy)
  - [log_transformer](#log_transformerpy)

---

## hydra_teleop.main

Entry point for the Hydra experiment framework.

### `main() -> None`

Orchestrates the full experiment lifecycle:

1. Initializes configuration, logging, and ROS 2 runtime
2. Creates all middleware components (bridges, emitter, monitors)
3. Runs the good-run loop collecting `cfg.simulation_runs` successful runs
4. Tears down all components
5. Runs post-experiment analysis

### `_run_one_strategy(strategy_name, ctrl, cfg, exec) -> tuple`

Runs a single navigation strategy in a fresh Gazebo instance.

**Parameters:**
- `strategy_name` (`str`): One of `"APE1"`, `"APE2"`, `"APE3"`, `"TROOP"`
- `ctrl` (`GzTeleop`): Velocity controller instance
- `cfg` (`TeleopConfig`): Configuration
- `exec` (`MultiThreadedExecutor`): ROS 2 executor

**Returns:** `(reached: bool, elapsed_s: float, energy_j: float, events_handled: int, events_violated: int)`

---

## hydra_teleop.config

### `class TeleopConfig`

Centralized configuration dataclass. All experiment parameters are defined as class or instance attributes with sensible defaults.

See [CONFIGURATION.md](CONFIGURATION.md) for the complete parameter reference.

---

## hydra_teleop.navigation

### nav_algorithm_T.py

#### `class LidarTargetNavigatorTROOP`

The primary navigation engine implementing deadline-aware APE selection with LiDAR-based obstacle avoidance.

**Constructor:**
```python
LidarTargetNavigatorTROOP(ctrl: GzTeleop, cfg: TeleopConfig, strategy: str)
```

- `ctrl`: Velocity controller for sending commands
- `cfg`: Global configuration
- `strategy`: Which planning mode to use (`"APE1"`, `"APE2"`, `"APE3"`, or `"TROOP"`)

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `attach_to_executor` | `(exec: MultiThreadedExecutor) -> None` | Register ROS 2 subscriptions with the executor |
| `go_to` | `(target_xyz=None, timeout_s=None) -> tuple` | Navigate to target; returns `(reached, elapsed_s, energy_j, events_handled, events_violated)` |
| `shutdown` | `() -> None` | Clean up all subscriptions and internal state |

#### `class GoToConfig`

Navigation tuning parameters (dataclass). See [CONFIGURATION.md](CONFIGURATION.md#navigation-gotoconfig).

#### `class AvoidCfg`

LiDAR obstacle avoidance parameters (dataclass). See [CONFIGURATION.md](CONFIGURATION.md#obstacle-avoidance-avoidcfg).

#### `class BreadcrumbCfg`

Visited-space tracking parameters (dataclass).

#### `class SafetyCfg`

Safety subsystem parameters (dataclass).

---

### teleop.py

#### `class GzTeleop`

Thread-safe velocity controller that wraps `DronePhysics` and publishes shaped commands.

**Constructor:**
```python
GzTeleop(topic: str, cfg: TeleopConfig)
```

**Methods:**

| Method | Signature | Description |
|---|---|---|
| `start` | `() -> None` | Begin the velocity publishing loop |
| `set_cmd` | `(vx, vy, vz, wz) -> None` | Set desired velocity (thread-safe) |
| `publish_once` | `(vx, vy, vz, wz) -> None` | Send a single velocity command |
| `stop` | `() -> None` | Stop the publishing loop |
| `shutdown` | `() -> None` | Full cleanup including transport |

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
| `publish` | `(vx, vy, vz, wz) -> None` | Publish a velocity command via Gazebo transport |
| `close` | `() -> None` | Release transport resources |

---

## hydra_teleop.simulation

### sim.py

#### `start_sim(cfg: TeleopConfig) -> subprocess.Popen`

Launch a Gazebo simulation instance with the configured world and environment.

**Returns:** The Gazebo subprocess handle.

#### `stop_sim(proc: subprocess.Popen) -> None`

Gracefully terminate a Gazebo instance, falling back to SIGKILL if needed.

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
1. Command latency (FIFO buffer)
2. 2nd-order actuator dynamics (per-axis)
3. Jerk limiting (acceleration slew-rate)
4. Tilt/thrust caps (lateral acceleration, asymmetric vertical)
5. Aerodynamic drag (linear + quadratic)
6. Wind gusts (Ornstein-Uhlenbeck process)

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

## hydra_teleop.tools

### event_emitter.py

#### `class EventCfg`

Configuration for event generation.

| Field | Type | Default | Description |
|---|---|---|---|
| `seed` | `int` | `42` | RNG seed for deterministic replay |
| `event_deterministic` | `bool` | `True` | If `True`, use seeded RNG; if `False`, use system entropy |

#### `class EventEmitter`

ROS 2 node that generates and publishes events on `/hydra/event`.

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
| `log_and_reset` | `(label: str, include_params: bool) -> dict` | Return energy summary and reset |

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

#### `start_parameter_bridge(mappings: list[tuple]) -> subprocess.Popen`

Start the `ros_gz_bridge parameter_bridge` process with the given topic mappings.

**Parameters:**
- `mappings`: List of `(topic, ros_type, gz_type)` tuples

**Example:**
```python
bridge = start_parameter_bridge([
    ("/model/drone1/front_lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
    ("/model/drone1/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist"),
])
```

---

## hydra_teleop.logging

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
| `monitor_interval_s` | `float` | `10.0` | Queue health monitoring interval |
| `json_format` | `bool` | `True` | Use JSON formatting for log entries |

#### `setup_async_logger(cfg: AsyncLoggerCfg) -> LogHandle`

Initialize the async logging system. Returns a handle with a `.stop()` method.

---

## hydra_teleop.analysis

### statistics_analyzer.py

#### `run_analysis(zone_metric: str = "mean") -> dict`

Run statistical analysis on the experiment CSV results.

**Parameters:**
- `zone_metric`: Aggregation method for zone violations (`"mean"`, `"median"`, etc.)

**Returns:** Dictionary with keys:
- `zone_metric`: The computed zone metric value
- `summary_csv`: Path to the generated summary CSV
- `summary`: Aggregated statistics dictionary

---

### log_transformer.py

#### `run_from_cfg() -> None`

Transform JSON log files into CSV format using paths from `TeleopConfig`.

#### `transform(input_path: str, output_path: str) -> None`

Convert a single JSON log file to CSV format.
