#!/usr/bin/env python3
"""
Log transformer for the *new* Hydra schema (STRICT TeleopConfig version).

- Detects run boundaries as: collect EVENTS up to hydra_teleop.teleop STOP,
  then emits a row on the immediately following hydra_teleop.main {reached, elapsed}.
- Requires at least one EVENT in the run (otherwise no row).
- CSV columns (order preserved):
    timestamp, strategy, reached, elapse_time, zone_violations, event_violated, events_handled, event_violations

Input/Output are taken **only** from TeleopConfig:
    - cfg.log_path: path to the input JSON log (required)
    - cfg.results_csv_path: path to write the output CSV (required)

No fallbacks, no guessing. If a required field is missing or the file does not exist, an exception is raised.
"""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List
import json
import csv
from hydra_teleop.config import TeleopConfig

# ---------- Shape helpers ----------
def _is_stop(rec: Dict[str, Any]) -> bool:
    return rec.get("name") == "hydra_teleop.teleop" and isinstance(rec.get("msg"), dict) and rec["msg"].get("event") == "STOP"

def _is_main_terminator(rec: Dict[str, Any]) -> bool:
    if rec.get("name") != "hydra_teleop.main":
        return False
    msg = rec.get("msg")
    return isinstance(msg, dict) and ("reached" in msg) and ("elapsed" in msg)

def _is_zone_violation(rec: Dict[str, Any]) -> bool:
    return rec.get("type") == "ZONEVIOLATION"

def _is_event(rec: Dict[str, Any]) -> bool:
    return rec.get("type") == "EVENT" and rec.get("msg") == "EVENT" and isinstance(rec.get("payload"), dict)

def _event_outcome_is_success(payload: Dict[str, Any]) -> bool:
    """
    True if the EVENT outcome indicates a success (resolved),
    False if it indicates a violation/failure.
    """
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

def transform(cfg: TransformCfg) -> List[Dict[str, Any]]:
    """
    Transform a structured JSON log into an experiment summary CSV.
    Raises if input file is missing.
    """
    in_path = Path(cfg.input_log_path)
    out_path = Path(cfg.output_csv_path)

    if not in_path.exists():
        raise FileNotFoundError(f"[log_transformer] Input log not found: {in_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)

    current = {
        "events_handled": 0,
        "event_violations": 0,
        "zone_violations": 0,
        "saw_any_event": False,
        "run_closed": False,  # becomes True at STOP; emit on next MAIN reached
    }
    rows: List[Dict[str, Any]] = []

    def reset_counters():
        current["events_handled"] = 0
        current["event_violations"] = 0
        current["zone_violations"] = 0
        current["saw_any_event"] = False
        current["run_closed"] = False

    with in_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception:
                # skip non-JSON noise lines
                continue

            if _is_zone_violation(rec):
                current["zone_violations"] += 1
                continue

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
                # Only emit a row if we actually saw an EVENT this run
                if current["saw_any_event"]:
                    ts = rec.get("ts")
                    msg = rec.get("msg", {})
                    strategy = rec.get("strategy")
                    row = {
                        "timestamp": ts,
                        "strategy": strategy,
                        "reached": bool(msg.get("reached")),
                        "elapse_time": float(msg.get("elapsed", 0.0)),
                        "zone_violations": int(current["zone_violations"]),
                        "event_violated": current["event_violations"] > 0,
                        "events_handled": int(current["events_handled"]),
                        "event_violations": int(current["event_violations"]),
                    }
                    rows.append(row)
                reset_counters()
                continue

    # Write CSV in fixed column order
    cols = [
        "timestamp",
        "strategy",
        "reached",
        "elapse_time",
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
    """
    Read required paths from TeleopConfig only. No fallbacks.
    Required TeleopConfig attributes:
      - log_path: str (path to input JSON log)
      - results_csv_path: str (path to output CSV)
    """
    cfg = TeleopConfig()

    # Validate presence & types
    if not hasattr(cfg, "log_path") or not isinstance(cfg.log_path, str) or not cfg.log_path:
        raise AttributeError("TeleopConfig must define a non-empty string 'log_path' (input JSON log path).")
    if not hasattr(cfg, "results_csv_path") or not isinstance(cfg.results_csv_path, str) or not cfg.results_csv_path:
        raise AttributeError("TeleopConfig must define a non-empty string 'results_csv_path' (output CSV path).")
    
    return transform(TransformCfg(
        input_log_path=cfg.log_path,
        output_csv_path=cfg.results_csv_path,
    ))

if __name__ == "__main__":
    # Strict: will raise if TeleopConfig/paths are not correct
    run_from_cfg()
