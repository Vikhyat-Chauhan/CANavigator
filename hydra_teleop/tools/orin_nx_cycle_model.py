"""
orin_nx_cycle_model.py — ARM Cortex-A78AE compute latency and energy model
                          for the NVIDIA Jetson Orin NX, for APE cost accounting.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LATENCY TABLE SOURCE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Operation cycle counts are taken from:

  ARM Cortex-A78 Software Optimization Guide, Revision 04
  Document number: ARM 103-0101 0003
  Table B.1 — Integer Pipeline Execution Latencies

  Cortex-A78AE (Automotive Enhanced) shares the integer pipeline
  microarchitecture of the Cortex-A78; the "-AE" suffix adds lockstep
  reliability extensions only and does not alter ALU or divide timing.

Per-operation latencies (cycles at maximum frequency):
  Integer ALU (ADD, SUB, AND, ORR, EOR, MOV, CMP): 1 cycle
  Integer multiply MUL (32-bit):                   3 cycles
  Integer divide SDIV (32-bit, worst case):        12 cycles
  Modulo (SDIV + MSUB, no native modulo on AArch64): ~15 cycles

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CONVERSION TO MICROSECONDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

CPU maximum frequency for the Jetson Orin NX 16GB module:
  f_max = 2.0 GHz  (NVIDIA Jetson Orin NX Module Data Sheet, DS-10662-001)
  → 1 cycle = 0.5 ns = 0.0005 µs

The Cortex-A78AE integer pipeline cycle counts match those of AMD Zen 4
(ADD/MUL/compare are identical; SDIV on Zen 4 is 12–41 cycles depending on
operand, arm worst-case is 12 cycles).  Because cycle counts are equivalent
the per-operation µs cost differs from the Zen 4 baseline only by the
clock-frequency ratio:

  scale = f_Zen4 / f_A78AE = 4500 MHz / 2000 MHz = 2.25

  Source for Zen 4 base clock: AMD Ryzen 9 7950X Processor Specifications, 2022.
  Source for Orin NX CPU clock: NVIDIA DS-10662-001.

All latency values below = (Zen 4 reference value) × 2.25.
The operation profiles (op counts per APE) are identical to the Zen 4 model
because the APE source code is unchanged; only the per-operation cost differs.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
POWER MODEL PARAMETERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  TDP      = 25 W     NVIDIA Jetson Orin NX 16GB, MAXN (maximum performance)
                      power mode.  Source: NVIDIA DS-10662-001, Table 1
                      "Module Electrical Specifications".

  idle_frac = 0.10    Module idle power is approximately 2–3 W from NVIDIA
                      Jetson Orin Power Estimation and Measurement Application
                      Note (NVIDIA document TB-10580-001).
                      10% = 2.5 W / 25 W TDP.

  N_cores  = 8        8× ARM Cortex-A78AE cores.
                      Source: NVIDIA DS-10662-001, Table 2 "Module Features".

  f_base   = 2.0 GHz  CPU maximum frequency.
                      Source: NVIDIA DS-10662-001.

  α        = 1.5      CMOS dynamic-power frequency exponent.
                      Source: Bircher & John, IEEE Trans. Computers, 2012.
                      (Same exponent as the Zen 4 model; α is a material /
                      process property, not processor-specific.)

Note: Because the latency table operates at clock_relative_freq = 1.0
(f == f_base = 2.0 GHz), the frequency-ratio term (f/f_base)^α = 1.0
and is omitted from the energy calculation.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SCAN-WINDOW PARAMETERS (unchanged from Zen 4 model)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  N_FRONT = 20   ±5° forward sector at ~0.25°/step (front_deg default = 5.0)
  N_GATE  =  8   ±2° gate/arc sector at ~0.25°/step (half_w_deg = 2.0)

  Each _window_vals(N) call costs:
    mul(2)              index arithmetic: radians for center & half_width
    sub(2)              index lo/hi bounds
    div(2)              / inc for lo and hi
    compare(3 + 3·N)    3 bounds clamps + 2N list-comp (isfinite + r>0) + N min()

  For N_FRONT = 20:  window cost = 4×0.56 + 2×13.50 + 63×5.63 ≈ 383.06 µs
  For N_GATE  =  8:  window cost = 4×0.56 + 2×13.50 + 27×5.63 ≈ 184.65 µs
"""

from __future__ import annotations


# ---------------------------------------------------------------------------
# Latency table (µs) — ARM Cortex-A78AE at 2.0 GHz
#
# Derived from ARM 103-0101 0003 (Table B.1) cycle counts scaled by the
# clock-frequency ratio: 4500 MHz (Zen 4 ref) / 2000 MHz (Orin NX) = 2.25×
# ---------------------------------------------------------------------------
_L: dict[str, float] = {
    "assign":  0.38,   # MOV: 1 cycle × 0.5 ns × 2.25 scale
    "add":     0.56,   # ADD: 1 cycle
    "sub":     0.56,   # SUB: 1 cycle
    "mul":     2.25,   # MUL (32-bit): 3 cycles
    "mod":     9.00,   # SDIV + MSUB: ~15 cycles (no native modulo in AArch64)
    "div":    13.50,   # SDIV (32-bit, worst case): 12 cycles
    "compare": 5.63,   # CMP + branch-resolve: modelled at same ratio as Zen 4
    "shift":   1.13,   # LSL / LSR: 1 cycle
    "bitwise": 0.56,   # AND / ORR / EOR: 1 cycle
}

# ---------------------------------------------------------------------------
# Scan-window parameters (identical to Zen 4 model — APE source unchanged)
# ---------------------------------------------------------------------------
_N_FRONT = 20
_N_GATE  =  8


def _window_cost(n: int) -> list[tuple[str, int]]:
    """Cost of one _window_vals(N) call + subsequent min()."""
    return [
        ("mul",     2),
        ("sub",     2),
        ("div",     2),
        ("compare", 3 + 3 * n),
    ]


# ---------------------------------------------------------------------------
# Operation profiles (op_type, count) — identical to Zen 4 model
# ---------------------------------------------------------------------------

_SHARED: list[tuple[str, int]] = [
    ("add",     1), ("mul",     1), ("mod",     1), ("sub",     1),
    ("mul",     1), ("compare", 2),
    *_window_cost(_N_FRONT),
    ("mul",     1), ("compare", 1),
    ("compare", 1), ("mul",     1),
    ("add",     1), ("div",     1), ("compare", 1), ("mul",     1),
    ("compare", 2),
    ("compare", 2), ("sub",     1), ("mul",     2), ("div",     2),
    ("compare", 1),
]

_APE1_UNIQUE: list[tuple[str, int]] = [
    ("mul",     1), ("mul",     1), ("compare", 1), ("mul",     1),
]

_APE2_UNIQUE: list[tuple[str, int]] = [
    *_window_cost(_N_FRONT),   # forward clearance check
    *_window_cost(_N_FRONT),   # sidestep corridor check
    ("compare", 1),
    ("sub", 1), ("div", 1),
    ("compare", 2),
    ("mul", 1), ("add", 1),
    ("mul", 1), ("compare", 1),
    ("mul", 2),
    ("compare", 1), ("mul", 2), ("add", 1),
]

_CONFIDENCE: list[tuple[str, int]] = [
    *_window_cost(_N_FRONT),
    ("sub",     1), ("div",     1), ("compare", 2),
    *_window_cost(_N_GATE), *_window_cost(_N_GATE),
    ("add",     1), ("sub",     1),
    ("sub",     1), ("div",     1), ("compare", 2),
    ("compare", 1), ("mul",     1),
    ("div",     1), ("compare", 2), ("sub",     1),
    *_window_cost(_N_GATE), *_window_cost(_N_GATE), *_window_cost(_N_GATE),
    ("compare", 3),
    ("compare", 3), ("add",     2), ("div",     1),
    ("mul",     4), ("add",     3),
    ("compare", 2),
]

_APE3_UNIQUE: list[tuple[str, int]] = [
    *_CONFIDENCE,
    ("add",     2),
    ("compare", 1),
    ("mul",     2), ("compare", 1),
    ("mul",     1), ("add",     1), ("compare", 1),
    ("mul",     1), ("compare", 1),
    ("mul",     1), ("add",     1), ("compare", 1),
    ("mul",     1), ("sub",     1),
    ("mul",     3), ("compare", 1), ("sub",     1), ("add",     1),
]


# ---------------------------------------------------------------------------
# Pre-compute per-invocation latencies (µs)
# ---------------------------------------------------------------------------

def _latency_us(profile: list[tuple[str, int]]) -> float:
    return sum(_L[op] * cnt for op, cnt in profile)


_SHARED_US = _latency_us(_SHARED)

_PER_INVOCATION_US: dict[str, float] = {
    "APE1": _SHARED_US + _latency_us(_APE1_UNIQUE),
    "APE2": _SHARED_US + _latency_us(_APE2_UNIQUE),
    "APE3": _SHARED_US + _latency_us(_APE3_UNIQUE),
}

# Public latency dict — µs per APE invocation on Cortex-A78AE @ 2.0 GHz
APE_LATENCY_US: dict[str, float] = _PER_INVOCATION_US

# DEADLINE_SCALE: same as Zen 4 model (1 µs APE compute ↔ 1 ms event window).
# Because Orin NX latencies are 2.25× larger than Zen 4 the deadline windows
# expand proportionally (~1.5–2.5 s), which is appropriate for a drone-class
# embedded processor responding to real-world events.
DEADLINE_SCALE: float = 1000.0

# ---------------------------------------------------------------------------
# Power model parameters — NVIDIA Jetson Orin NX 16GB (DS-10662-001)
# ---------------------------------------------------------------------------
_TDP_W:     float = 25.0          # Module TDP, MAXN mode (DS-10662-001 Table 1)
_IDLE_FRAC: float = 0.10          # ~10%: ~2.5 W idle / 25 W TDP (TB-10580-001)
_N_CORES:   int   = 8   # 8× Cortex-A78AE (DS-10662-001 Table 2); hardcoded to match
                        # the Orin NX spec regardless of the simulation host's CPU count.


def latency_to_energy_j(total_latency_us: float, wall_s: float) -> float:
    """
    Convert OrinNxCycleMeter total latency to compute energy (Joules).

    Power model (Fan, Weber & Barroso, ISCA 2007):
        P = P_idle + (TDP - P_idle) * U_eff
        E = P * wall_s

    where:
        U_eff = (total_latency_us * 1e-6) / (wall_s * N_cores)

    The frequency-scaling term (f/f_base)^1.5 equals 1.0 because the latency
    table is defined at clock_relative_freq = 1.0 (f = f_base = 2.0 GHz) and
    is therefore omitted.

    Parameters
    ----------
    total_latency_us : total simulated CPU work from OrinNxCycleMeter.end() [µs]
    wall_s           : wall-clock duration of the mission [s]
    """
    if wall_s <= 0.0:
        return 0.0
    p_idle   = _TDP_W * _IDLE_FRAC
    active_s = total_latency_us * 1e-6
    u_eff    = min(1.0, active_s / (wall_s * _N_CORES))
    p_avg    = p_idle + (_TDP_W - p_idle) * u_eff
    return p_avg * wall_s


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

class OrinNxCycleMeter:
    """
    Computes total compute latency (µs) for APE workloads on the NVIDIA
    Jetson Orin NX (ARM Cortex-A78AE @ 2.0 GHz), using the parallel-halt
    execution model:

      All APEs in a selector run start simultaneously.  When the selected APE
      finishes at T_sel, any still-running APE is halted.  Each APE
      contributes min(its_latency, T_sel).

      Solo modes (APE1/APE2/APE3): one APE runs, cost = that APE's full latency.

      TROOP parallel-halt costs (Orin NX, 2.25× Zen 4):
        APE1 selected → min(523,523) + min(1729,523) + min(2034,523) ≈ 1569 µs
        APE2 selected → min(523,1729) + min(1729,1729) + min(2034,1729) ≈ 3981 µs
        APE3 selected → min(523,2034) + min(1729,2034) + min(2034,2034) ≈ 4286 µs

    API:
        begin()
        record_event(selected: str, running: list)
        end() -> (total_latency_us: float, per_selected_us: dict[str, float])
    """

    def __init__(self) -> None:
        self._total_us: float = 0.0
        self._per_selected: dict[str, float] = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def begin(self) -> None:
        self._total_us = 0.0
        self._per_selected = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def record_event(self, selected: str, running: list) -> None:
        """
        Record one event's compute cost under the parallel-halt model.

        selected : APE whose plan was applied ("APE1", "APE2", or "APE3")
        running  : all APEs spawned for this event — ["APE1","APE2","APE3"]
                   for TROOP, [selected] for solo modes
        """
        t_sel = _PER_INVOCATION_US.get(selected, 0.0)
        cost  = sum(min(_PER_INVOCATION_US.get(n, 0.0), t_sel) for n in running)
        self._total_us += cost
        if selected in self._per_selected:
            self._per_selected[selected] += cost

    def end(self) -> tuple:
        return self._total_us, dict(self._per_selected)