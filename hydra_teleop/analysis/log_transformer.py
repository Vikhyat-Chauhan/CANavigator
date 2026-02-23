#!/usr/bin/env python3
"""
Log transformer for the *new* Hydra schema (STRICT TeleopConfig version).

- Detects run boundaries as: collect EVENTS up to hydra_teleop.teleop STOP,
  then emits a row on the immediately following hydra_teleop.main
  {reached, elapsed, violations, energy_j, mean_power_w, compute_energy_j}.
- Requires at least one EVENT in the run (otherwise no row).
- CSV columns (order preserved):
    timestamp, strategy, run, reached, elapse_time, energy_kj, mean_power_kw,
    compute_energy_j, nav_start_dist_xy_m, zone_violations,
    event_violated, events_handled, event_violations
"""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional
import json
import csv
from hydra_teleop.config import TeleopConfig  # kept for STRICT-ness, not used directly below

# ---------- Shape helpers ----------
def _is_stop(rec: Dict[str, Any]) -> bool:
    return rec.get("name") == "hydra_teleop.navigation.teleop" and isinstance(rec.get("msg"), dict) and rec["msg"].get("event") == "STOP"

def _is_main_terminator(rec: Dict[str, Any]) -> bool:
    if rec.get("name") != "hydra_teleop.main":
        return False
    msg = rec.get("msg")
    # Terminator keyed by reached & elapsed; other fields validated at emit-time.
    return isinstance(msg, dict) and ("reached" in msg) and ("elapsed" in msg)

def _is_event(rec: Dict[str, Any]) -> bool:
    return rec.get("type") == "EVENT" and rec.get("msg") == "EVENT" and isinstance(rec.get("payload"), dict)

def _is_nav_start(rec: Dict[str, Any]) -> bool:
    # POSES line that carries nav-start distances
    return rec.get("type") == "POSES" and rec.get("msg") == "POSES" and isinstance(rec.get("payload"), dict)

def _event_outcome_is_success(payload: Dict[str, Any]) -> bool:
    outcome = payload.get("outcome")
    if isinstance(outcome, bool):
        return outcome
    if isinstance(outcome, str):
        up = outcome.strip().upper()
        if up in {"RESOLVED", "SUCCESS", "TRUE"}:
            return True
        if up in {"VIOLATION", "FAIL", "FALSE"}:
            return False
    reason = payload.get("reason")
    if isinstance(reason, str) and reason.strip().upper() == "SUCCESS":
        return True
    return False

# ---------- Core transformer ----------
@dataclass
class TransformCfg:
    input_log_path: str
    output_csv_path: str
    # Optional distance filter: only emit rows when nav_start_dist_xy_m >= this threshold
    min_nav_start_dist_xy_m: Optional[float] = None

def transform(cfg: TransformCfg) -> List[Dict[str, Any]]:
    in_path = Path(cfg.input_log_path)
    out_path = Path(cfg.output_csv_path)

    if not in_path.exists():
        raise FileNotFoundError(f"[log_transformer] Input log not found: {in_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)

    current: Dict[str, Any] = {
        "events_handled": 0,
        "event_violations": 0,
        "saw_any_event": False,
        "run_closed": False,            # becomes True at STOP; emit on next MAIN reached
        "nav_seen": False,              # must see POSES before emitting a row
        "nav_start_dist_m": None,       # from POSES
        "nav_start_dist_xy_m": None,    # from POSES
    }
    rows: List[Dict[str, Any]] = []

    def reset_counters():
        current["events_handled"] = 0
        current["event_violations"] = 0
        current["saw_any_event"] = False
        current["run_closed"] = False
        current["nav_seen"] = False
        current["nav_start_dist_m"] = None
        current["nav_start_dist_xy_m"] = None

    with in_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception:
                continue

            # Required single POSES before nav per run (your guarantee)
            if _is_nav_start(rec):
                payload = rec["payload"]
                try:
                    current["nav_start_dist_xy_m"] = float(payload["nav_start_dist_xy_m"])
                except Exception:
                    raise ValueError(f"[log_transformer] POSES missing 'nav_start_dist_xy_m' at ts={rec.get('ts')}")
                try:
                    current["nav_start_dist_m"] = float(payload["nav_start_dist_m"])
                except Exception:
                    raise ValueError(f"[log_transformer] POSES missing 'nav_start_dist_m' at ts={rec.get('ts')}")
                current["nav_seen"] = True
                continue

            # Legacy per-event ZONEVIOLATION records are intentionally ignored.

            if _is_event(rec):
                current["saw_any_event"] = True
                payload = rec["payload"]
                if _event_outcome_is_success(payload):
                    current["events_handled"] += 1
                else:
                    current["event_violations"] += 1
                continue

            if _is_stop(rec):
                current["run_closed"] = True
                continue

            if _is_main_terminator(rec) and current["run_closed"]:
                # We require: at least one EVENT and the POSES record
                if current["saw_any_event"]:
                    if not current["nav_seen"]:
                        raise RuntimeError("[log_transformer] Run reached terminator without prior POSES line (nav_start).")

                    # --- Distance filter applied here ---
                    if cfg.min_nav_start_dist_xy_m is not None:
                        nav_dist = current["nav_start_dist_xy_m"]
                        if nav_dist is None:
                            raise RuntimeError("[log_transformer] Missing nav_start_dist_xy_m when applying distance filter.")
                        if nav_dist < float(cfg.min_nav_start_dist_xy_m):
                            reset_counters()
                            continue
                    # -----------------------------------

                    ts = rec.get("ts")
                    msg = rec.get("msg", {})
                    strategy = rec.get("strategy")

                    # STRICT: pull final metrics from hydra_teleop.main
                    try:
                        zone_violations = int(msg["violations"])
                    except Exception:
                        raise ValueError(f"[log_transformer] hydra_teleop.main terminator missing integer 'violations' at ts={ts}")

                    # Energy & power (convert to kJ / kW)
                    try:
                        energy_kj = float(msg["energy_j"]) / 1000.0
                    except Exception:
                        raise ValueError(f"[log_transformer] hydra_teleop.main terminator missing numeric 'energy_j' at ts={ts}")
                    try:
                        mean_power_kw = float(msg["mean_power_w"]) / 1000.0
                    except Exception:
                        raise ValueError(f"[log_transformer] hydra_teleop.main terminator missing numeric 'mean_power_w' at ts={ts}")

                    # NEW: compute_energy_j (leave in joules, tiny magnitude compared to flight energy)
                    compute_energy_j = 0.0
                    try:
                        if "compute_energy_j" in msg and msg["compute_energy_j"] is not None:
                            compute_energy_j = float(msg["compute_energy_j"])
                    except Exception:
                        # If present but malformed, surface a clear error
                        raise ValueError(f"[log_transformer] terminator 'compute_energy_j' not numeric at ts={ts}")

                    row = {
                        "timestamp": ts,
                        "strategy": strategy,
                        # 'run' is assigned in a post-pass once we know grouping across strategies
                        "reached": bool(msg.get("reached")),
                        "elapse_time": float(msg.get("elapsed", 0.0)),
                        "energy_kj": energy_kj,
                        "mean_power_kw": mean_power_kw,
                        "compute_energy_j": compute_energy_j,  # <-- added
                        "nav_start_dist_xy_m": current["nav_start_dist_xy_m"],
                        "zone_violations": zone_violations,
                        "event_violated": current["event_violations"] > 0,
                        "events_handled": int(current["events_handled"]),
                        "event_violations": int(current["event_violations"]),
                    }
                    rows.append(row)
                reset_counters()
                continue

    # ---------- Assign per-cycle run tags ----------
    def assign_run_tags(rows: List[Dict[str, Any]]) -> None:
        CYCLE_STRATS = {"APE1", "APE2", "APE3", "TROOP"}
        seen: set = set()
        run_idx = 1
        for r in rows:
            # Default to current run index
            r["run"] = run_idx
            s = str(r.get("strategy")) if r.get("strategy") is not None else ""
            if s in CYCLE_STRATS:
                seen.add(s)
                if seen == CYCLE_STRATS:
                    # Completed one full cycle; next row starts a new run
                    seen.clear()
                    run_idx += 1

    assign_run_tags(rows)

    # Write CSV (order preserved)
    cols = [
        "timestamp",
        "strategy",
        "run",  # grouping across strategies
        "reached",
        "elapse_time",
        "energy_kj",
        "mean_power_kw",
        "compute_energy_j",          # <-- added to CSV
        "nav_start_dist_xy_m",
        "zone_violations",
        "event_violated",
        "events_handled",
        "event_violations",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    return rows

# ---------- Public entrypoint: STRICT TeleopConfig ----------
def run_from_cfg() -> List[Dict[str, Any]]:
    # Point to the uploaded log by default
    return transform(TransformCfg(
        input_log_path="logs/run_logs.json",
        output_csv_path="logs/results/experiment_summary.csv",
        min_nav_start_dist_xy_m=0,
    ))

if __name__ == "__main__":
    run_from_cfg()