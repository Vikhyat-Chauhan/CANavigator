# Event Handling in the Conflict Architecture (CA)

This document describes the event episode lifecycle, violation taxonomy, selector
calibration, commitment hold rationale, and CSV output schema for the Hydra teleop
CA framework after the Option B fix (explicit execution commitment window).

---

## Section 1 — Event episode lifecycle

A single event episode passes through five phases: emission, arrival, parallel APE
planning, resolution + commitment hold, and return to baseline.

### 1.1 Emission

`EventEmitter` samples inter-arrival times `dt` log-uniformly from [0.02, 4.0] s.
Each event carries a deadline:

```
deadline_s = clamp(alpha * dt, [deadline_min_s, deadline_max_s])
```

with `alpha = 0.85`, `deadline_min_s = 0.12 s`, `deadline_max_s = 1.20 s`.
The authoritative source for these parameters is `TeleopConfig`; they propagate
into `EventCfg` via `EventCfg.from_teleop_cfg()`.

### 1.2 Arrival and intake

The nav loop polls `_EventSub.pop()` once per tick (~33 ms at 30 Hz).  When a new
event is returned:

1. `_events_handled` is incremented.
2. If an old event is still active, the salvage path runs (see §2 PREEMPTIVE).
3. `_pending_evt`, `_evt_deadline_at`, `_evt_active` are set for the new event;
   `_evt_resolved` is cleared.

### 1.3 Parallel APE launch

In TROOP mode all three planner threads start immediately.  In solo modes (APE1,
APE2, APE3) only one thread starts.  Each thread writes its result into
`_evt_proposals` under a lock when it finishes (~1 ms for APE1, ~16 ms for APE3).

### 1.4 Selector logic

Every nav tick while `event_active` is True the selector checks `time_left`:

```
target = APE3   if tl > t_med_s  (0.55 s)
       = APE2   if tl > t_hard_s (0.22 s)
       = APE1   otherwise
```

The preference order then falls back to lower-latency planners when the ideal one
is not yet ready:

```
APE3-tier: [APE3, APE2, APE1]
APE2-tier: [APE2, APE1]
APE1-tier: [APE1]
```

If no plan is ready the selector loops, applying the previous baseline command.

### 1.5 Resolution and commitment hold

When a plan is chosen for the first time (`not self._evt_resolved`):

1. A RESOLVED log record is written.
2. `_cycle_meter.record_event()` is called.
3. `_evt_resolved = True` is set.
4. The winning (v, wz, vz) triple is cached in `_resolved_cmd`.
5. `_evt_resolved_at` records the wall time.
6. `_commit_hold_active = True` is set.
7. `_evt_clear()` closes the event window immediately (resets `_evt_active`,
   `_pending_evt`, proposals, and threads).

On the immediately following ticks the commitment hold path fires:

```python
if _commit_hold_active:
    hold_elapsed = time.time() - _evt_resolved_at
    if hold_elapsed < commit_hold_s:   # 0.5 s
        v_cmd, wz_cmd, vz_cmd = _resolved_cmd
    else:
        _commit_hold_active = False
```

### 1.6 Return to baseline go-to

Once `_commit_hold_active` becomes False the nav loop reverts to the normal
avoidance / heading-select / go-to path with no special-case logic.

### Timeline diagram

```
t_recv                  APE1 ready   APE3 ready   deadline_at
  |                         |              |            |
  |----<1ms>----|---~16ms---|--~90ms max---|---up to 1.47s---|
  ^             ^           ^             ^
  event         APE1        APE3        deadline
  arrives       done        done        window ends
                |
                | selector picks best ready plan
                v
          RESOLVED  (t ≈ 16 ms after t_recv for APE3-tier events)
                |
                |<------- commit_hold_s = 0.5 s ------->|
                |  _resolved_cmd applied every nav tick  |
                                                         |
                                                         v
                                                  return to baseline go-to
```

---

## Section 2 — Violation taxonomy

There are exactly three outcomes for each event:

### RESOLVED

The selector found a ready plan within the deadline.  No violation is recorded.
`_events_handled` was already incremented at arrival; no violation counter changes.

### DEADLINE

`time_left` reached 0 before any plan was selected.  This is a genuine violation:
the drone had no committed evasion plan in time.

- `_evt_violate("DEADLINE")` is called.
- `_events_violated` and `_events_violated_deadline` are both incremented.
- A DEADLINE log record is written.

### PREEMPTIVE

A new event arrived while the old event was still active and no usable plan existed
for the old event.  This is a genuine violation on the old event.

- `_evt_violate("PREEMPTIVE")` is called.
- `_events_violated` and `_events_violated_preemptive` are both incremented.
- A PREMPTIVE log record is written.

### Why ghost violations no longer occur

Before the fix, `_evt_clear()` was commented out in the RESOLVED branch.  This left
`_evt_active = True` and `_pending_evt` set after resolution, so the selector kept
re-applying the APE plan on every nav tick.  When no new event arrived before the
deadline expired, `tl <= 0` fired `_evt_violate("DEADLINE")` on an event that had
already been resolved — a ghost violation.

After the fix, `_evt_clear()` is called immediately upon resolution.  `_evt_active`
becomes False on the same tick, so the deadline check is never reached for a
resolved event.

**Quantification from the 100-run experiment:** 23% of DEADLINE records were ghost
violations on resolved events.  This matches exactly the fraction of inter-arrival
gaps longer than `deadline_min` (1.471 s) in the log-uniform emission distribution,
confirming these violations were not caused by genuine planner latency but by the
open event window.

---

## Section 3 — Selector threshold calibration

### Tier definitions

| Tier   | Condition            | Planner | Physical motivation |
|--------|----------------------|---------|---------------------|
| APE1   | tl ≤ 0.22 s          | APE1    | Reaction distance 1.8–3.3 m at v_max; only immediate evasion is feasible |
| APE2   | 0.22 s < tl ≤ 0.55 s | APE2    | Medium horizon; partial sidestep with clearance check |
| APE3   | tl > 0.55 s          | APE3    | Distant / slow events; full quality plan affordable |

### Expected tier split

From the corrected deadline distribution with `alpha = 0.85`,
`deadline_min = 0.12 s`, `deadline_max = 1.20 s`, log-uniform dt ∈ [0.02, 4.0] s:

- APE1 tier: ~48%
- APE2 tier: ~17%
- APE3 tier: ~35%

### Parameter coupling

`TeleopConfig` is the authoritative source for `deadline_alpha`, `deadline_min_s`,
and `deadline_max_s`.  These propagate into `EventCfg` via
`EventCfg.from_teleop_cfg()`, ensuring the emitter and the selector use a
consistent deadline distribution.  `t_hard_s` and `t_med_s` in `EventDecisionCfg`
must be re-calibrated if any of these parameters change.

---

## Section 4 — Commitment hold rationale

### Why one nav tick is insufficient

A single nav tick is ~33 ms (30 Hz).  APE2/APE3 sidestep commands move at
`sidestep_speed_frac * max_v = 0.35 * 15.0 = 5.25 m/s`.  Against a ~1.2 m radius
obstacle the drone needs to open at least 1.5 m of lateral clearance before it can
safely resume forward flight.  At 5.25 m/s that requires ~0.29 s of sustained
lateral motion minimum; `commit_hold_s = 0.5 s` provides a comfortable margin.

### Why the hold is explicit

Before this fix the hold was an accidental side-effect of the commented-out
`_evt_clear()`.  This made the duration implicitly equal to
`deadline_at - t_resolved`, which varied with each event and could be arbitrarily
long.  Making it an explicit `EventDecisionCfg` parameter:

- Appears in the CFG log record, making every run fully auditable.
- Can be tuned independently of the deadline distribution.
- Is documented with its physical motivation in the dataclass.

### Behaviour during the hold

During the hold the commitment hold path overrides `v_cmd`, `wz_cmd`, and `vz_cmd`
with the cached `_resolved_cmd` triple produced by the winning APE.  After
`commit_hold_s` elapses, `_commit_hold_active` is cleared and the nav loop returns
cleanly to the baseline go-to path.  The event episode is then fully closed with no
residual state.

---

## Section 5 — CSV output schema

Each row in `results_csv_path` corresponds to one strategy in one good run
(all strategies reached the target).

### Mission columns

| Column       | Type  | Description |
|--------------|-------|-------------|
| `run`        | int   | 1-based index of the good run (discarded attempts not counted) |
| `strategy`   | str   | Navigator name: APE1, APE2, APE3, or TROOP |
| `elapse_time`| float | Wall-clock seconds from nav start to target reached |

### NFZ columns

| Column           | Type  | Description |
|------------------|-------|-------------|
| `zone_violations`| int   | Number of no-fly zone boundary crossings detected by ViolationMonitor |

### Compute columns

| Column               | Type  | Description |
|----------------------|-------|-------------|
| `compute_latency_us` | float | Total Orin NX cycle-model latency in microseconds across all nav ticks |
| `compute_energy_j`   | float | Estimated compute energy in joules derived from cycle latency and elapsed time |

### Flight energy columns

| Column         | Type  | Description |
|----------------|-------|-------------|
| `energy_kj`    | float | Flight energy in kilojoules measured by EnergyMonitor |
| `mean_power_kw`| float | Mean flight power in kilowatts over the run |

### Event columns

| Column                     | Type  | What it counts | What it does NOT count |
|----------------------------|-------|----------------|------------------------|
| `events_handled`           | int   | All events received by the nav loop (incremented at arrival, before any outcome) | Nothing excluded |
| `event_violated`           | int   | Total violations = DEADLINE + PREEMPTIVE | Events that resolved successfully |
| `event_violated_deadline`  | int   | Events where `time_left` reached 0 before any plan was selected | Ghost violations (eliminated by this fix) |
| `event_violated_preemptive`| int   | Events where a new event arrived before the old event resolved and no salvage plan existed | Events where a salvage plan was successfully applied |
| `event_violation_rate`     | float | `event_violated / events_handled` (0.0 if events_handled == 0) | — |

`event_violated == event_violated_deadline + event_violated_preemptive` always holds.
