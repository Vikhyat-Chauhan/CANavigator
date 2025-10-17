#!/usr/bin/env python3
"""
hydra_teleop.tools.log_to_csv

Convert Hydra JSON-lines logs into a per-run CSV summary.

Public API:
    log_transformer(in_path: Union[str, Path], out_path: Optional[Union[str, Path]] = None, logger=None) -> Path
    summarize_lines(lines_iterable: Iterable[str]) -> List[Dict[str, Any]]

A 'run' ends whenever the main process emits a record containing:
    msg.reached (bool) and msg.elapsed (float)

Between such run-terminator records, we count:
    - events_handled
    - events_success
    - events_deadline_miss
    - events_preemptive
    - zone_violations

CSV columns:
    timestamp, strategy, reached, elapsed_time,
    events_handled, events_success, events_deadline_miss,
    events_preemptive, zone_violations
"""

from __future__ import annotations
import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Union

MAIN_NAME_HINTS = (
    "hydra_teleop.main",
    ".main",        # e.g., "hydra_teleop.runner.main"
    "main",         # fallback
)

# ----------------- Helpers: schema normalization -----------------

def _get(obj: Dict[str, Any], *path, default=None):
    cur = obj
    try:
        for p in path:
            cur = cur[p]
        return cur
    except Exception:
        return default

def _is_from_main(obj: Dict[str, Any]) -> bool:
    name = obj.get("name", "")
    if not isinstance(name, str):
        return False
    return any(h in name for h in MAIN_NAME_HINTS)

def _is_reached_record(obj: Dict[str, Any]) -> bool:
    msg = obj.get("msg")
    if not isinstance(msg, dict):
        return False
    return ("reached" in msg) and ("elapsed" in msg) and _is_from_main(obj)

def _extract_strategy(obj: Dict[str, Any]) -> str:
    candidates = [
        obj.get("strategy"),
        _get(obj, "msg", "strategy"),
        _get(obj, "extra", "strategy"),
        _get(obj, "msg", "extra", "strategy"),
    ]
    for c in candidates:
        if isinstance(c, str) and c.strip():
            return c.strip()
    return ""

def _looks_like_event(obj: Dict[str, Any]) -> bool:
    msg = obj.get("msg")
    typ = obj.get("type") or (msg.get("type") if isinstance(msg, dict) else None)
    if typ == "EVENT":
        return True
    if isinstance(msg, dict) and isinstance(msg.get("event"), str):
        ev = msg["event"].upper()
        return ev.startswith("EVENT_") or ev == "EVENT"
    return False

def _event_outcome_reason(obj: Dict[str, Any]):
    msg = obj.get("msg", {})
    if not isinstance(msg, dict):
        return None, None

    outcome = msg.get("outcome")
    reason  = msg.get("reason")

    ev = msg.get("event")
    if isinstance(ev, str):
        evu = ev.upper()
        if evu == "EVENT_RESOLVED":
            return "RESOLVED", "SUCCESS"
        if evu == "EVENT_VIOLATION" and not reason:
            return "VIOLATION", None

    if isinstance(outcome, str):
        outcome = outcome.upper()
    if isinstance(reason, str):
        reason = reason.upper().replace("-", "_")

    return outcome, reason

ZONE_RE_STRINGS = (
    "restricted zone",
    "entered restricted zone",
    "zone violation",
    "no-fly",
    "nfz",
)

def _looks_like_zone_violation(obj: Dict[str, Any]) -> bool:
    msg = obj.get("msg")
    typ = obj.get("type") or (msg.get("type") if isinstance(msg, dict) else None)
    if isinstance(typ, str) and typ.upper() == "ZONEVIOLATION":
        return True

    if isinstance(msg, dict):
        reason = msg.get("reason") or ""
        if isinstance(reason, str) and reason.upper().startswith("ZONE"):
            return True

    if isinstance(msg, str):
        mlow = msg.lower()
        if any(s in mlow for s in ZONE_RE_STRINGS):
            return True

    name = obj.get("name", "")
    if isinstance(name, str) and "violation_counter" in name:
        if isinstance(msg, str):
            mlow = msg.lower()
            if "violation" in mlow or "restricted zone" in mlow:
                return True

    return False

# ----------------- Core: summarize -----------------

def summarize_lines(lines_iterable: Iterable[str]) -> List[Dict[str, Any]]:
    """
    Summarize JSON-lines records provided as an iterable of raw lines.
    Returns a list of row dicts ready to be written to CSV.
    """
    rows: List[Dict[str, Any]] = []

    counts = {
        "events_total": 0,
        "events_success": 0,
        "events_deadline_miss": 0,
        "events_preemptive": 0,
        "zone_violations": 0,
    }

    def reset_counts():
        for k in counts:
            counts[k] = 0

    for line in lines_iterable:
        line = (line or "").strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except Exception:
            # Skip malformed lines silently (or log if desired)
            continue

        # Count events
        if _looks_like_event(obj):
            counts["events_total"] += 1
            outcome, reason = _event_outcome_reason(obj)
            if (outcome == "RESOLVED") and (reason in {None, "SUCCESS", "RESOLVED"}):
                counts["events_success"] += 1
            elif outcome == "VIOLATION":
                if reason == "DEADLINE_MISS":
                    counts["events_deadline_miss"] += 1
                elif reason in {"PREEMPTIVE", "PREEMPTED", "PREMPTIVE", "PRE_EMPTIVE"}:
                    counts["events_preemptive"] += 1

        # Count zone violations
        elif _looks_like_zone_violation(obj):
            counts["zone_violations"] += 1

        # On run-terminator, emit one row and reset
        if _is_reached_record(obj):
            msg = obj.get("msg", {})
            row = {
                "timestamp": obj.get("ts") or obj.get("@timestamp") or "",
                "strategy": _extract_strategy(obj),
                "reached": bool(msg.get("reached")),
                "elapsed_time": (float(msg.get("elapsed")) if msg.get("elapsed") is not None else ""),
                "events_handled": counts["events_total"],
                "events_success": counts["events_success"],
                "events_deadline_miss": counts["events_deadline_miss"],
                "events_preemptive": counts["events_preemptive"],
                "zone_violations": counts["zone_violations"],
            }
            rows.append(row)
            reset_counts()

    return rows

def _write_csv(rows: List[Dict[str, Any]], out_path: Path) -> Path:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "timestamp",
        "strategy",
        "reached",
        "elapsed_time",
        "events_handled",
        "events_success",
        "events_deadline_miss",
        "events_preemptive",
        "zone_violations",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return out_path

def log_transformer(
    in_path: Union[str, Path],
    out_path: Optional[Union[str, Path]] = None,
    logger=None
) -> Path:
    """
    Read a JSON-lines log from disk and write the per-run CSV.
    Returns the output CSV Path.
    """
    in_path = Path(in_path)
    if out_path is None:
        out_path = in_path.with_name("experiment_reached_summary.csv")
    out_path = Path(out_path)

    if not in_path.exists():
        raise FileNotFoundError(f"Input log not found: {in_path}")

    with in_path.open("r", encoding="utf-8") as f:
        rows = summarize_lines(f)

    _write_csv(rows, out_path)
    if logger:
        try:
            logger.info(
                {"event": "CSV_WRITTEN", "type": "RUN_SUMMARY", "rows": len(rows), "path": str(out_path)}
            )
        except Exception:
            # don't break main just because logger failed
            pass
    return out_path

# Optional CLI (kept for convenience)
if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Hydra teleop: convert JSON-lines logs to per-run CSV.")
    ap.add_argument("--in", dest="in_path", required=True, help="Path to JSON-lines log (e.g., run_logs.json)")
    ap.add_argument("--out", dest="out_path", required=False, help="Output CSV path (default: experiment_reached_summary.csv next to input)")
    args = ap.parse_args()
    path = log_transformer(args.in_path, args.out_path)
    print(f"Wrote summary → {path}")
