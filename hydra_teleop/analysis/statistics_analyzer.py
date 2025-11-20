# hydra_teleop/analysis/statistics_analyzer.py
from __future__ import annotations
import os, re
import numpy as np
import pandas as pd
from typing import Dict, Any

# -------------------------------------------------------------------
# Bettendorf-aware one-shot analysis (summary only)
# -------------------------------------------------------------------
def run_analysis(
    order=("APE1", "APE2", "APE3", "TROOP"),  # kept for signature compatibility (unused)
    deadline_ok_tol: float = 0.0,             # kept for signature compatibility (unused)
    zone_metric: str = "median",              # kept for signature compatibility; controls filename only
) -> Dict[str, Any]:
    """
    Summarize experiment CSV (B-style) per strategy.

    TeleopConfig (STRICT):
      - cfg.results_csv_path  : str
      - cfg.analyzer_out_dir  : str

    EXPECTED CSV (case-insensitive; REQUIRED):
      strategy | elapse_time | zone_violations | energy_kj | mean_power_kw

    OPTIONAL BUT SUPPORTED IF PRESENT:
      run                # run index (1..N)
      compute_energy_kj  # CPU-side compute energy in kJ
      compute_energy_j   # legacy name for CPU energy (if present, treated as kJ-equivalent for stats)
      events_handled
      event_violations
      event_violation_rate

    NOTE:
      - We no longer require or use a 'reached' column; the runner only writes
        rows for fully successful runs where all strategies reached the target.
      - We also drop any timeout-based metrics that depended on 'reached'.
    """
    zm = str(zone_metric).strip().lower()
    if zm not in {"median", "mean"}:
        raise ValueError("zone_metric must be 'median' or 'mean'")

    # --- Strict dependency on TeleopConfig ---
    from hydra_teleop.config import TeleopConfig
    cfg = TeleopConfig()

    if not (isinstance(getattr(cfg, "results_csv_path", None), str) and cfg.results_csv_path):
        raise AttributeError("TeleopConfig must define a non-empty string 'results_csv_path' (input CSV path).")
    if not (isinstance(getattr(cfg, "analyzer_out_dir", None), str) and cfg.analyzer_out_dir):
        raise AttributeError("TeleopConfig must define a non-empty string 'analyzer_out_dir' (output folder).")

    csv_path: str = cfg.results_csv_path
    out_dir: str = cfg.analyzer_out_dir
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV not found at {csv_path}")
    os.makedirs(out_dir, exist_ok=True)

    # ---------- Read ----------
    df_raw = pd.read_csv(csv_path)

    # ---------- Column resolution ----------
    def resolve(col_name_expected: str) -> str:
        expected_lower = col_name_expected.strip().lower()
        for c in df_raw.columns:
            if c.strip().lower() == expected_lower:
                return c
        patt = re.compile(rf"^{re.escape(col_name_expected)}$", re.I)
        for c in df_raw.columns:
            if patt.search(c):
                return c
        raise KeyError(f"Missing required column: '{col_name_expected}'. Found columns: {list(df_raw.columns)}")

    def resolve_opt(col_name_expected: str) -> str | None:
        expected_lower = col_name_expected.strip().lower()
        for c in df_raw.columns:
            if c.strip().lower() == expected_lower:
                return c
        patt = re.compile(rf"^{re.escape(col_name_expected)}$", re.I)
        for c in df_raw.columns:
            if patt.search(c):
                return c
        return None  # optional

    # Required columns (case-insensitive)
    COL = {
        "strategy":        resolve("strategy"),
        "elapsed":         resolve("elapse_time"),
        "zone_violations": resolve("zone_violations"),
        "energy_kj":       resolve("energy_kj"),
        "mean_power_kw":   resolve("mean_power_kw"),
    }

    # Optional / legacy columns
    col_run = resolve_opt("run")
    col_compute_kj = resolve_opt("compute_energy_kj")
    col_compute_j_legacy = resolve_opt("compute_energy_j")  # legacy transformer output
    col_events_handled = resolve_opt("events_handled")
    col_event_violations = resolve_opt("event_violations")
    col_event_violation_rate = resolve_opt("event_violation_rate")

    # ---------- Coercions ----------
    def to_num(s: pd.Series) -> pd.Series:
        return pd.to_numeric(s, errors="coerce")

    # pick whichever compute-energy column exists
    compute_energy_col = col_compute_kj or col_compute_j_legacy

    # Build normalized dataframe; missing optional columns become NaN
    data = {
        "_strategy":      df_raw[COL["strategy"]].astype(str).str.strip(),
        "_elapsed_s":     to_num(df_raw[COL["elapsed"]]),
        "_zone_viol":     to_num(df_raw[COL["zone_violations"]]),
        "_energy_kj":     to_num(df_raw[COL["energy_kj"]]),
        "_mean_power_kw": to_num(df_raw[COL["mean_power_kw"]]),
    }

    if compute_energy_col is not None:
        data["_compute_energy"] = to_num(df_raw[compute_energy_col])
    else:
        data["_compute_energy"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_events_handled is not None:
        data["_events_handled"] = to_num(df_raw[col_events_handled])
    else:
        data["_events_handled"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_event_violations is not None:
        data["_event_violations"] = to_num(df_raw[col_event_violations])
    else:
        data["_event_violations"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_event_violation_rate is not None:
        data["_event_violation_rate"] = to_num(df_raw[col_event_violation_rate])
    else:
        data["_event_violation_rate"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_run is not None:
        data["_run"] = to_num(df_raw[col_run])
    else:
        data["_run"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    df = pd.DataFrame(data)

    # NOTE:
    # We *do not* do any round-filtering based on 'reached' anymore.
    # The runner only writes rows for "good runs" where all strategies reached.
    df_f = df.copy()
    if df_f.empty:
        empty_csv = os.path.join(out_dir, f"strategy_summary_zone-{zm}.csv")
        pd.DataFrame().to_csv(empty_csv, index=False)
        return {
            "zone_metric": zm,
            "summary_csv": empty_csv,
            "summary": {},
        }

    # ---------- Per-strategy summary on all (already-successful) data ----------
    g = df_f.groupby("_strategy", dropna=False)

    def med(s: pd.Series) -> float:
        s = s.dropna()
        return float(s.median()) if len(s) else float("nan")

    def p95(s: pd.Series) -> float:
        s = s.dropna()
        return float(np.percentile(s, 95)) if len(s) else float("nan")

    def any_pos(s: pd.Series) -> float:
        s = s.dropna()
        if not len(s):
            return float("nan")
        return float((s > 0).mean())

    summary = pd.DataFrame({
        "runs_considered":        g.size(),
        "reach_time_median_s":    g["_elapsed_s"].apply(med),
        "reach_time_mean_s":      g["_elapsed_s"].mean(),
        "reach_time_p95_s":       g["_elapsed_s"].apply(p95),
        "zone_violations_mean":   g["_zone_viol"].mean(),
        "zone_violations_median": g["_zone_viol"].median(),
        "p_any_zone_violation":   g["_zone_viol"].apply(any_pos),
        "energy_kj_median":       g["_energy_kj"].median(),
        "mean_power_kw_mean":     g["_mean_power_kw"].mean(),
    })

    # Compute-energy summary (if present)
    if not df_f["_compute_energy"].isna().all():
        summary["compute_energy_median"] = g["_compute_energy"].apply(med)

    # Event-related summaries (if present)
    if not df_f["_event_violation_rate"].isna().all():
        summary["event_violation_rate_mean"] = g["_event_violation_rate"].mean()
        summary["event_violation_rate_p95"] = g["_event_violation_rate"].apply(p95)

    if not df_f["_event_violations"].isna().all():
        summary["event_violations_mean"] = g["_event_violations"].mean()
        summary["p_any_event_violation"] = g["_event_violations"].apply(any_pos)

    # ---------- Write summary CSV and return ----------
    summary_csv = os.path.join(out_dir, f"strategy_summary_zone-{zm}.csv")
    summary.round(6).to_csv(summary_csv)

    return {
        "zone_metric": zm,
        "summary_csv": summary_csv,
        "summary": summary.round(6).to_dict(orient="index"),
    }
