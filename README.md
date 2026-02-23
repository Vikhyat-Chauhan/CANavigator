<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros&logoColor=white" alt="ROS2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiPjwvc3ZnPg==" alt="Gazebo Harmonic">
  <img src="https://img.shields.io/badge/Python-3.10+-yellow?logo=python&logoColor=white" alt="Python 3.10+">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="MIT License">
  <img src="https://img.shields.io/badge/Platform-Ubuntu_22.04+-purple?logo=ubuntu&logoColor=white" alt="Ubuntu 22.04+">
</p>

# Hydra

**Heterogeneous, Yet Dynamic Resource Allocation for Autonomous Drones**

A high-fidelity drone navigation and benchmarking framework that evaluates **deadline-aware adaptive planning** under dynamic, event-driven constraints. Hydra orchestrates thousands of reproducible simulation runs comparing multiple planning strategies (APE1/APE2/APE3/TROOP) in procedurally generated environments with real-time no-fly zones, LiDAR-based obstacle avoidance, and physics-accurate energy modeling.

---

## Screenshots

<!--
  HOW TO ADD SCREENSHOTS:
  1. Take screenshots of the Gazebo simulation, RViz, terminal output, or results plots
  2. Save them to docs/screenshots/ (e.g. docs/screenshots/gazebo_city.png)
  3. Uncomment and update the relevant lines below

  Recommended screenshots:
  - Gazebo view of the drone navigating a city-grid arena
  - Gazebo view of a Perlin-noise arena
  - Terminal output showing a successful experiment run
  - RViz LiDAR visualization
  - Results plot comparing APE strategies
-->

<p align="center">
  <!-- <img src="docs/screenshots/gazebo_city.png" width="48%" alt="City-grid arena in Gazebo"> -->
  <!-- <img src="docs/screenshots/gazebo_perlin.png" width="48%" alt="Perlin-noise arena in Gazebo"> -->
</p>
<p align="center">
  <!-- <img src="docs/screenshots/lidar_rviz.png" width="48%" alt="LiDAR visualization in RViz"> -->
  <!-- <img src="docs/screenshots/results_plot.png" width="48%" alt="Strategy comparison results"> -->
</p>

> **To add screenshots:** save images to `docs/screenshots/` and uncomment the lines above. See the HTML comments in this file for detailed instructions.

---

## Highlights

- **Deadline-Aware APE Selection** -- Dynamically selects between fast (8 ms), medium (25 ms), and full (90 ms) planning algorithms based on real-time event deadlines
- **TROOP Meta-Algorithm** -- Time-budget-aware selector that picks the best available planner within deadline constraints, maximizing planning quality without missing deadlines
- **Procedural World Generation** -- City-grid and Perlin-noise arena generators create unique no-fly zone layouts for each run, ensuring statistical robustness
- **Physics-Accurate Simulation** -- DJI FlyCart 30-tuned dynamics model with actuator latency, aerodynamic drag, jerk limiting, and Ornstein-Uhlenbeck wind gusts
- **Full ROS 2 + Gazebo Integration** -- LiDAR perception, velocity control, and pose tracking through standard ROS 2 topics bridged to Gazebo
- **Energy-Aware Benchmarking** -- Dual energy model tracking both CPU computational cost and motion energy (208.9 J/m EPM baseline)
- **Deterministic Reproducibility** -- Seeded RNG for events, world generation, and navigation ensures bit-exact replay across runs

---

## Architecture

```
                         ┌─────────────────────────────────┐
                         │     Experiment Controller       │
                         │          (main.py)              │
                         └──────────┬──────────────────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              ▼                     ▼                     ▼
     ┌────────────────┐   ┌────────────────┐   ┌─────────────────┐
     │  Arena          │   │  Event         │   │  Gazebo         │
     │  Generator      │   │  Emitter       │   │  Simulator      │
     │  (city/perlin)  │   │  (deadline-    │   │  (SDF world +   │
     │                 │   │   aware)       │   │   X3 UAV)       │
     └────────┬───────┘   └────────┬───────┘   └────────┬────────┘
              │                    │                     │
              └────────────────────┼─────────────────────┘
                                   ▼
                  ┌────────────────────────────────────┐
                  │        ROS 2 Middleware Layer       │
                  │                                    │
                  │  Pose Republisher  ·  LiDAR Bridge │
                  │  Violation Monitor · Energy Monitor │
                  │  Parameter Bridge (cmd_vel, scan)  │
                  └──────────────────┬─────────────────┘
                                     ▼
              ┌──────────────────────────────────────────┐
              │        TROOP Navigator Engine            │
              │                                          │
              │   ┌──────┐  ┌──────┐  ┌──────┐          │
              │   │ APE1 │  │ APE2 │  │ APE3 │          │
              │   │ 8 ms │  │ 25ms │  │ 90ms │          │
              │   │ Fast │  │ Med  │  │ Full │          │
              │   └──┬───┘  └──┬───┘  └──┬───┘          │
              │      └─────────┼─────────┘              │
              │          Deadline Selector               │
              │                                          │
              │   LiDAR Avoidance · NFZ Repulsion       │
              │   Target Tracking · TTC Braking         │
              └──────────────────┬───────────────────────┘
                                 ▼
              ┌──────────────────────────────────────────┐
              │      Velocity Controller (GzTeleop)      │
              │                                          │
              │   DronePhysics: FlyCart 30 dynamics      │
              │   Actuator model · Drag · Wind · Jerk   │
              └──────────────────┬───────────────────────┘
                                 ▼
                    /model/drone1/cmd_vel → Gazebo
```

### APE Deadline Selection

The TROOP meta-algorithm selects planners based on time remaining before an event deadline:

```
Time Left (t_left)          Selected Planner
─────────────────────────────────────────────
t_left  > t_med             APE3 (full planning, best quality)
t_hard  < t_left ≤ t_med    APE2 (corridor planning, balanced)
t_left  ≤ t_hard            APE1 (fast dodge, emergency)
```

---

## Project Structure

```
hydra/
├── hydra_teleop/                    # Core Python package
│   ├── main.py                      # Experiment orchestrator & CSV recorder
│   ├── config.py                    # TeleopConfig — all tunables in one place
│   ├── navigation/
│   │   ├── nav_algorithm_T.py       # TROOP navigator + APE1/APE2/APE3 engines
│   │   ├── teleop.py               # GzTeleop velocity controller
│   │   └── transport.py            # Gazebo transport publisher
│   ├── simulation/
│   │   ├── sim.py                  # Gazebo process management
│   │   ├── physics.py              # FlyCart 30 drone dynamics model
│   │   └── pose_republisher.py     # Gazebo → ROS 2 pose relay
│   ├── tools/
│   │   ├── event_emitter.py        # Deterministic event generation
│   │   ├── violations.py           # No-fly zone violation tracker
│   │   ├── energy_monitor.py       # Energy-per-meter estimation
│   │   ├── arena_generator_city.py # City-grid NFZ generation
│   │   ├── arena_generator_perlin.py # Perlin-noise NFZ generation
│   │   ├── target_generator.py     # Safe target placement
│   │   ├── nofly_generator.py      # No-fly zone SDF generation
│   │   └── bridge.py              # ROS 2 ↔ Gazebo parameter bridge
│   ├── logging/
│   │   └── async_logger.py         # Queue-based async JSON logger
│   └── analysis/
│       ├── statistics_analyzer.py  # CSV aggregation & statistical analysis
│       └── log_transformer.py      # JSON log → CSV transformation
├── models/
│   └── x3-uav/                     # X3 UAV drone model (SDF)
├── worlds/
│   └── airport_world.sdf           # Gazebo simulation world
├── run_hydra.sh                    # Full setup & execution script
├── quick_run.sh                    # Fast re-run (skips setup)
├── kill_ros.sh                     # ROS 2 / Gazebo cleanup utility
├── requirements.txt                # Python dependencies
├── docs/                           # Extended documentation
│   ├── ARCHITECTURE.md             # System design deep-dive
│   ├── CONFIGURATION.md            # All configuration parameters
│   └── API.md                      # Module & class API reference
└── LICENSE                         # MIT
```

---

## Quick Start

### Prerequisites

| Dependency | Version |
|---|---|
| Ubuntu | 22.04+ |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| Python | 3.10+ |

### Installation

```bash
# Clone the repository
git clone https://github.com/vikhyat-chauhan/Hydra.git
cd Hydra

# Run the automated setup (installs dependencies, creates venv, runs experiment)
./run_hydra.sh
```

The setup script will:
1. Install required system packages (`python3-venv`, `python3-gz-transport13`, etc.)
2. Create a Python virtual environment with system site-packages access
3. Install pip dependencies (`numpy`, `pyyaml`, `packaging`, `argcomplete`)
4. Verify Gazebo transport bindings
5. Launch the experiment

### Quick Re-Run

```bash
# Skip setup, clear logs, and run immediately
./quick_run.sh
```

### Manual Execution

```bash
source venv/bin/activate
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
python3 -m hydra_teleop.main
```

---

## How It Works

### Experiment Loop

Each experiment consists of up to **1,000 successful runs**. For each run:

1. **Arena Generation** -- A unique no-fly zone layout and target position are procedurally generated (city-grid or Perlin-noise style)
2. **Strategy Evaluation** -- All four strategies (APE1, APE2, APE3, TROOP) navigate the same arena with identical event sequences
3. **Validation** -- Only runs where all strategies reach the target are kept, ensuring fair head-to-head comparison
4. **Recording** -- Results (elapsed time, energy, violations, event handling) are written to CSV

### Event-Driven Navigation

Dynamic events are emitted with log-uniform spacing and assigned deadlines based on `deadline = alpha * delta_t`. The navigator must respond before the deadline expires:

| Event Type | Description | Probability |
|---|---|---|
| Enemy | Hostile target requiring evasive action | 33% |
| Obstacle | Dynamic obstacle insertion | 33% |
| Lane | Corridor restriction | 34% |

### Physics Model

The `DronePhysics` model is tuned to the DJI FlyCart 30 with:

- **2nd-order actuator dynamics** with per-axis modeling
- **Jerk-limited acceleration** shaping (Cuniato et al., 2022)
- **Aerodynamic drag**: linear + quadratic (Hattenberger et al., 2023)
- **Wind disturbances**: Ornstein-Uhlenbeck stochastic process
- **Energy model**: 208.9 J/m EPM + CPU power estimation

---

## Output & Analysis

Results are saved to `logs/results/experiment_summary.csv` with the following metrics per strategy per run:

| Metric | Description |
|---|---|
| `elapse_time` | Time to reach target (seconds) |
| `zone_violations` | Number of no-fly zone entries |
| `compute_energy_kj` | CPU computational energy (kJ) |
| `energy_kj` | Total motion energy (kJ) |
| `mean_power_kw` | Average power consumption (kW) |
| `events_handled` | Successfully resolved events |
| `event_violated` | Missed deadline events |
| `event_violation_rate` | Fraction of events that missed deadline |

The built-in `statistics_analyzer` produces aggregated summaries comparing all strategies.

---

## Configuration

All parameters are centralized in [`hydra_teleop/config.py`](hydra_teleop/config.py) via the `TeleopConfig` dataclass. Key groups:

| Category | Examples | Description |
|---|---|---|
| **Simulation** | `simulation_runs`, `simulation_timeout` | How many runs, per-run timeout |
| **World Generation** | `simulation_world_style`, `target_distance` | City vs. Perlin, target placement |
| **Dynamics** | `speed_x/y/z`, `yaw_rate`, `rate_hz` | Velocity limits, control frequency |
| **Events** | `event_dt_min_s`, `deadline_alpha` | Event timing, deadline model |
| **Physics** | `cmd_latency_s`, `wind_level_0to1` | Actuator delay, wind intensity |
| **Analysis** | `analyzer_strategies`, `results_csv_path` | Which strategies to compare |

See [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) for a full parameter reference.

---

## Documentation

| Document | Description |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | System design, data flow, and component interactions |
| [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) | Complete parameter reference with defaults and descriptions |
| [`docs/API.md`](docs/API.md) | Module-level API reference for all classes and functions |

---

## Tech Stack

| Layer | Technology |
|---|---|
| **Simulation** | Gazebo Harmonic (SDF 1.9), X3 UAV model |
| **Middleware** | ROS 2 Jazzy, `ros_gz_bridge` |
| **Navigation** | Custom TROOP + APE1/APE2/APE3 planners |
| **Perception** | 2D LiDAR (front-mounted), pose tracking |
| **Physics** | Custom FlyCart 30 dynamics (Python) |
| **Analysis** | NumPy, pandas, CSV/JSON logging |
| **Language** | Python 3.10+ |

---

## License

This project is licensed under the [MIT License](LICENSE).

Copyright (c) 2025 Vikhyat Chauhan
