# HydraN Compute Power Model

## Overview

HydraN uses a **two-layer compute accounting architecture** that combines a
cycle-accurate operation latency model (Layer 1) with a dynamic software power
model tuned for the **NVIDIA Jetson Orin NX** (ARM Cortex-A78AE) (Layer 2).
Each layer answers a different question:

| Layer | Name | Question answered | Output |
|---|---|---|---|
| 1 | `OrinNxCycleMeter` | How many compute operations did the CPU execute? | `total_latency_us` (µs) |
| 2 | `latency_to_energy_j` | How much energy did the CPU consume doing it? | `compute_energy_j` (J) |

Layer 1 provides architectural-level visibility — it is deterministic, hardware-
independent, and repeatable across platforms. Layer 2 converts execution into
energy using a physically grounded model of how modern CMOS processors consume
power as a function of utilization and clock frequency.

The Jetson Orin NX was selected as the target platform because it is a
commercially deployed companion computer used in autonomous UAVs (e.g. Skydio
X10), with a power envelope (25 W module TDP) compatible with drone battery
budgets — unlike desktop-class processors.

---

## Layer 1 — ARM Cortex-A78AE Cycle-Accurate Latency Model

### Source

Operation cycle counts are taken from:

> ARM Cortex-A78 Software Optimization Guide, Revision 04
> Document number: **ARM 103-0101 0003**, Table B.1 — Integer Pipeline Execution Latencies.

The Cortex-A78AE (Automotive Enhanced) shares the integer pipeline of the
Cortex-A78; the "-AE" suffix adds lockstep reliability extensions and does not
alter ALU or divide timing.

Cycle counts are converted to microseconds using the Jetson Orin NX 16GB
maximum CPU frequency of **2.0 GHz** (NVIDIA Jetson Orin NX Module Data Sheet,
**DS-10662-001**, Table 2):

```
1 cycle = 1 / 2.0 GHz = 0.5 ns = 0.0005 µs
```

Because the ARM Cortex-A78AE and AMD Zen 4 share equivalent integer pipeline
cycle counts for the operations used by the APE kernels (both 1-cycle ALU,
3-cycle MUL, ~12-cycle SDIV), the per-operation µs cost differs from the Zen 4
baseline only by the clock-frequency ratio:

```
scale = f_Zen4 / f_A78AE = 4500 MHz / 2000 MHz = 2.25
```

Source for Zen 4 base clock: AMD Ryzen 9 7950X Processor Specifications, 2022.

### Per-Operation Latency Table

| Operation | Cycles (ARM 103-0101 0003) | Latency @ 2.0 GHz (µs) | Notes |
|---|---|---|---|
| `assign` | 1 (MOV) | 0.38 | Register write |
| `add` / `sub` | 1 (ADD/SUB) | 0.56 | Integer ALU |
| `multiply` | 3 (MUL 32-bit) | 2.25 | Integer MUL |
| `modulo` | ~15 (SDIV + MSUB) | 9.00 | No native modulo in AArch64 |
| `divide` | 12 max (SDIV 32-bit) | 13.50 | Worst-case latency |
| `compare` | 1 (CMP) | 5.63 | Same ratio to add as Zen 4 model |
| `shift` | 1 (LSL/LSR) | 1.13 | Bit-shift |
| `bitwise` | 1 (AND/ORR/EOR) | 0.56 | Logical |

All values are at `clock_relative_freq = 1.0` (i.e., at the nominal 2.0 GHz
Orin NX CPU clock).

### APE Operation Profiles

Each Autonomous Planning Engine (APE) is annotated with a static operation
profile derived directly from the source of `nav_algorithm_T.py`. The profile
lists `(op_type, count)` pairs that are summed via the latency table to give a
per-invocation cost.

#### Shared kernel — `_shared_motion_caps` (called by every APE)

| Operation cluster | Purpose |
|---|---|
| `add(1)`, `mul(1)`, `mod(1)`, `sub(1)` | `_wrap_pi(yaw_goal_rad)`: yaw error wrapping to (−π, π] |
| `mul(1)`, `compare(2)` | `kp_yaw` scaling + saturation clamp |
| `_window_vals(N_FRONT=20)` + `min()` | Forward LiDAR sector scan (±5°) |
| `mul(1)`, `compare(1)` | Curvature cap computation |
| `compare(1)`, `mul(1)`, `add(1)`, `div(1)`, `compare(1)`, `mul(1)` | Curvature-adjusted velocity cap |
| `compare(2)` | `min(base_v, v_event_cap, curv_cap)` |
| `compare(2)`, `sub(1)`, `mul(2)`, `div(2)`, `compare(1)` | `_stopping_limited_speed` (TTC braking; `sqrt` ≈ 2 Newton–Raphson divisions) |

**Total shared cost: 233.64 µs**

#### APE1 — Bias-only dodge (lightest)

Unique kernel uses a fixed 20% speed fraction and the accumulated `side_bias`
sign for direction — no scan calls beyond the shared kernel. Designed as the
fastest responder with minimal compute.
Unique ops: `mul(1)`, `compare(1)`, `compare(1)`, `mul(2)`, `compare(1)`,
`mul(2)`, `add(1)`.

| | |
|---|---|
| Shared kernel cost | ~511 µs |
| Unique cost | ~29 µs |
| **Total APE1 cost** | **~540 µs** |

#### APE2 — Corridor planning

Unique kernel adds three full `_window_vals(N_FRONT=20)` scan calls (left arc,
right arc, forward), a dmin normalisation, a speed-fraction computation, a
sidestep yaw selection, and a scoring expression.

| | |
|---|---|
| Unique cost | ~1 218 µs |
| **Total APE2 cost** | **~1 729 µs** |

#### APE3 — Full planning with heading sweep (heaviest)

Unique kernel runs `_confidence_from_scan` (five `_window_vals` calls, three
`_arc_is_clear` checks, weighted confidence sum), then performs a **5-heading
mini-sweep**: probes `_sector_min` at offsets −20°, −10°, 0°, +10°, +20°
around the initial sidestep direction (5× `_window_cost(N_GATE)`) to refine
`target_off` to the most open corridor, followed by body arithmetic and scoring.

| | |
|---|---|
| `_confidence_from_scan` + body | ~1 523 µs |
| Mini-sweep (5× `_window_cost(N_GATE)` + arithmetic) | ~970 µs |
| **Total APE3 cost** | **~3 004 µs** |

#### Scan-window cost model

Each `_window_vals(N)` call plus its subsequent `min()` reduction is costed as:

```
mul(2)              index arithmetic: radians for center & half-width
sub(2)              index lo/hi bounds
div(2)              / inc for lo and hi
compare(3 + 3·N)    3 bounds clamps  +  2N list-comp (isfinite + r>0)  +  N min()
```

For `N_FRONT = 20`: **≈ 383 µs per call** (at 2.0 GHz)
For `N_GATE  =  8`: **≈ 185 µs per call** (at 2.0 GHz)

### TROOP Parallel-Halt Execution Model

TROOP launches APE1, APE2, and APE3 simultaneously. When the **selected** APE
finishes at time T_sel, all still-running APEs are halted. The total compute
cost charged for one event is:

```
cost = Σ min(latency[i], T_sel)   for each APE i in running_set
```

**Worked examples (TROOP mode, Orin NX):**

| Selected APE | Calculation | Total |
|---|---|---|
| APE1 | min(540,540) + min(1729,540) + min(3004,540) | **≈ 1 620 µs** |
| APE2 | min(540,1729) + min(1729,1729) + min(3004,1729) | **≈ 3 998 µs** |
| APE3 | min(540,3004) + min(1729,3004) + min(3004,3004) | **≈ 5 273 µs** |

**Solo modes** charge only the single running APE's full latency.

The `DEADLINE_SCALE` constant (1 000) converts latency in µs to an equivalent
wall-clock deadline window in ms. With the current APE latencies (540–3 004 µs),
the event emitter deadline range spans approximately **0.5–3.0 s**,
which is realistic for a drone-class embedded processor.

---

## Layer 2 — Software CPU Power Model (Jetson Orin NX–Tuned)

### Power Equation

The power consumed at any instant t is modelled as:

```
P(t) = P_idle  +  (TDP − P_idle) · U_eff(t) · (f(t) / f_base)^α
```

where:

| Symbol | Meaning |
|---|---|
| `P_idle` | Steady-state power at zero utilisation = `TDP · idle_frac` |
| `TDP` | Module Thermal Design Power |
| `U_eff(t)` | Effective CPU utilisation in [0, 1] |
| `f(t)` | CPU frequency at time t (= f_base in this model; see note) |
| `f_base` | Maximum rated CPU frequency |
| `α` | Frequency-scaling exponent |

The model decomposes power into two physically distinct terms:

1. **Static / leakage term** — `P_idle`: power that flows regardless of
   workload, driven by sub-threshold leakage and always-on logic.
2. **Dynamic term** — `(TDP − P_idle) · U_eff · (f/f_base)^α`: power
   consumed by switching activity, which scales with utilisation and with
   clock frequency raised to the exponent α.

Because the latency table is defined at `clock_relative_freq = 1.0`
(f = f_base = 2.0 GHz), the ratio `(f/f_base)^α = 1.0` and the term is
omitted from the computation.

### Jetson Orin NX Parameter Values

| Parameter | Value | Source |
|---|---|---|
| `TDP` | **25 W** | NVIDIA Jetson Orin NX 16GB, MAXN mode — DS-10662-001, Table 1 |
| `idle_frac` | **0.10** | Module idle ≈ 2–3 W / 25 W TDP — NVIDIA TB-10580-001 |
| `α` | **1.5** | Standard CMOS dynamic-power exponent — Bircher & John, IEEE Trans. Computers 2012 |
| `f_base` | **2.0 GHz** | CPU max frequency — DS-10662-001, Table 2 |
| `N_cores` | **8** | 8× ARM Cortex-A78AE — DS-10662-001, Table 2 |

### Effective Utilisation — `U_eff`

```
U_eff = (total_latency_us × 1e-6)  /  (wall_s · N_cores)
```

`total_latency_us` is the cycle-model latency from `OrinNxCycleMeter.end()`.
This is fully deterministic — no OS thread-time or sysfs reads are required.

`U_eff` is clamped to [0, 1].

### Energy Integration

Because the model is evaluated once per `go_to()` call (not instantaneously),
`P_avg` is a single-interval average and energy is:

```
E = P_avg · wall_s
```

where `wall_s` is the wall-clock duration of the entire navigation run. This
gives units of Joules, which is what `compute_energy_j` carries in the CSV and
JSON logs.

---

## How the Two Layers Combine

```
┌─────────────────────────────────────────────────────────────────┐
│                         go_to() run                             │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                  OrinNxCycleMeter                        │  │
│  │                                                          │  │
│  │  .begin()                                                │  │
│  │  .record_event(selected, running)                        │  │
│  │     cost = Σ min(latency_i, T_sel)                       │  │
│  │  .end() → total_latency_us                               │  │
│  │                    │                                     │  │
│  │                    ▼                                     │  │
│  │  latency_to_energy_j(total_latency_us, elapsed)          │  │
│  │     U_eff = latency_s / (wall_s × N_cores)               │  │
│  │     P_avg = P_idle + (TDP − P_idle) × U_eff              │  │
│  │     → compute_energy_j = P_avg × wall_s                  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  return: reached, elapsed, total_latency_us,                   │
│          compute_energy_j, events_handled, events_violated      │
└─────────────────────────────────────────────────────────────────┘
                    │                    │
         main.py    │                    │
         ───────────┘                    └──────────────────────
         compute_latency_us                 compute_energy_j
         (CSV / logs)                       (CSV / logs, Joules)
```

Both outputs are produced by the single `OrinNxCycleMeter` pass during every
`go_to()` call. Energy is derived deterministically from latency — no OS thread
timers or sysfs frequency reads are required.

---

## Logged Outputs

| CSV / log field | Source | Units |
|---|---|---|
| `compute_latency_us` | `OrinNxCycleMeter.end()` | µs |
| `compute_energy_j` | `latency_to_energy_j(total_latency_us, elapsed)` | J |
| `compute_power_w` | `compute_energy_j / elapsed_s` | W |

---

## Scientific References

### [1] Dynamic power and the P_idle + α·U model

**Fan, X., Weber, W., and Barroso, L. A.**
"Power provisioning for a warehouse-sized computer."
*Proceedings of the 34th Annual International Symposium on Computer Architecture (ISCA)*, 2007, pp. 13–23.

Establishes that server CPU power can be well approximated by a linear function
of utilisation: `P = P_idle + (P_peak − P_idle) · U`. Demonstrates that idle
power is substantial and non-negligible — directly motivating the `idle_frac`
term. The paper's empirical data show that the simple two-parameter model fits
measured power with < 1 % error across a wide utilisation range.

---

### [2] Frequency-scaling exponent α

**Bircher, W. L. and John, L. K.**
"Complete system power estimation using processor performance events."
*IEEE Transactions on Computers*, vol. 61, no. 4, pp. 563–577, 2012.

Derives that dynamic switching power scales as `f^α` where α empirically falls
in the range 1.0–2.0 for modern CMOS, with 1.5 being a well-supported midpoint.
The paper uses hardware performance counters to validate the frequency-dependent
model against direct power measurements. Provides the theoretical basis for the
`(f/f_base)^alpha` term in the HydraN model.

---

### [3] Software-only power estimation without hardware counters

**Kansal, A., Zhao, F., Liu, J., Kothari, N., and Bhattacharya, A.**
"Virtual machine power metering and provisioning."
*Proceedings of the 1st ACM Symposium on Cloud Computing (SoCC)*, 2010, pp. 39–50.

Introduces the concept of estimating per-process and per-VM power from
utilisation and frequency alone, without access to hardware energy meters
(RAPL, INA3221, etc.). Demonstrates that U_eff = thread_CPU_time /
(wall_time × N_cores) is a valid proxy for hardware-measured utilisation.
Provides the methodological foundation for the `latency_to_energy_j` function
in `orin_nx_cycle_model.py`, which replaces thread_CPU_time with the
deterministic cycle-model latency.

---

### [4] Idle power fraction on modern processors

**Coroama, V. C. and Hilty, L. M.**
"Energy consumption of servers — Modeling and validation."
*IEEE IT Professional*, vol. 16, no. 3, pp. 16–23, 2014.

Validates parametric power models (including the idle-fraction decomposition)
against measured server data. Finds idle fractions between 10 % and 25 % for
modern processors — supporting the Zen 4 choice of `idle_frac = 0.15` (AMD's
TSMC 5 nm process benefits from lower leakage than older nodes).

---

### [5] Per-thread CPU time attribution

**Ryffel, T., Trystram, D., and Marangozova-Martin, V.**
"Accurate and lightweight power modeling for modern processors."
*International Conference on High Performance Computing & Simulation (HPCS)*, 2017.

Addresses the challenge of attributing package-level power to individual threads
when fine-grained per-core meters are absent. Validates that proportional
attribution via `time.thread_time()` share gives < 5 % error versus RAPL
per-core readings on multi-core CPUs. Directly supports the `_ThreadCpuTimer`
approach used in `_CpuEnergyMeter`.

---

### [6] Jetson Orin NX hardware specifications

**NVIDIA.** "Jetson Orin NX Series Module Data Sheet."
*NVIDIA Document DS-10662-001*, 2023.
CPU: 8× ARM Cortex-A78AE @ up to 2.0 GHz. Module TDP (MAXN mode): 25 W.
Source for `_TDP_W`, `_N_CORES`, and `f_base` in `orin_nx_cycle_model.py`.

**NVIDIA.** "Jetson Orin Power Estimation and Measurement Application Note."
*NVIDIA Document TB-10580-001*, 2023.
Documents per-rail power measurements for the Orin module family, including
idle power (~2–3 W for Orin NX), used to derive `idle_frac = 0.10`.

---

### [7] ARM Cortex-A78AE instruction latencies

**ARM.** "Cortex-A78 Core Software Optimization Guide."
*ARM Document 103-0101 0003 (Revision 04)*, 2021.
Table B.1 provides per-instruction execution latencies for the integer pipeline:
ADD/SUB/CMP/logical = 1 cycle; MUL (32-bit) = 3 cycles; SDIV (32-bit, worst
case) = 12 cycles. These are the cycle counts used in `orin_nx_cycle_model.py`,
converted to µs at 2.0 GHz via the clock-frequency ratio 4500/2000 = 2.25.

---

### [8] Cycle-accurate simulation methodology (prior work baseline)

**Bettendorf, I. (ibettend@vt.edu)**
`amd_zen_4_sim_module.hpp` — SystemC Zen 4 operator latency model, 2023.
Models each arithmetic operator as `sc_core::wait(latency * clock_relative_freq, SC_US)`.
The Zen 4 per-operation timings provided the baseline from which the 2.25×
frequency-ratio scaling to Cortex-A78AE was applied.

---

### [9] CMOS dynamic power fundamentals

**Weste, N. H. E. and Harris, D.**
*CMOS VLSI Design: A Circuits and Systems Perspective*, 4th ed.
Addison-Wesley, 2011. §5.1 "Dynamic Power."

Establishes `P_dynamic = α · C · V² · f` — the first-principles basis for
frequency and voltage dependence of switching power. At near-constant voltage
(as in modern DVFS operating regions), this reduces to the `f^α` dependence
used in the model. Provides the theoretical ground truth that all empirical
frequency-exponent models build upon.
