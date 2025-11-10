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
    order=("APE1", "APE2", "APE3", "TROOP"),
    deadline_ok_tol: float = 0.0,   # kept for signature compatibility (unused)
    zone_metric: str = "median",    # kept for signature compatibility; controls filename only
) -> Dict[str, Any]:
    """
    Summarize experiment CSV (B-style) per strategy.

    TeleopConfig (STRICT):
      - cfg.results_csv_path  : str
      - cfg.analyzer_out_dir  : str

    CSV (case-insensitive accepted; all REQUIRED):
      strategy | reached | elapse_time | event_violated | zone_violations | energy_kj | mean_power_kw

    Output:
      * Writes per-strategy summary CSV to analyzer_out_dir/strategy_summary_zone-{zone_metric}.csv
      * Returns a dict with the path and the summary as a nested dict.
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

    # ---------- Column resolution (ALL required) ----------
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

    COL = {
        "strategy": resolve("strategy"),
        "reached": resolve("reached"),
        "elapsed": resolve("elapse_time"),
        "deadline_violation": resolve("event_violated"),
        "zone_violations": resolve("zone_violations"),
        "energy_kj": resolve("energy_kj"),
        "mean_power_kw": resolve("mean_power_kw"),
    }

    # ---------- Coercions ----------
    def to_bool(s: pd.Series) -> pd.Series:
        mapping = {
            "true": True, "1": True, "yes": True, "y": True, "t": True,
            "false": False, "0": False, "no": False, "n": False, "f": False
        }
        def conv(v):
            if pd.isna(v): return np.nan
            t = str(v).strip().lower()
            return mapping.get(t, np.nan)
        return s.map(conv)

    def to_num(s: pd.Series) -> pd.Series:
        return pd.to_numeric(s, errors="coerce")

    df = pd.DataFrame({
        "_strategy": df_raw[COL["strategy"]].astype(str).str.strip(),
        "_reached": to_bool(df_raw[COL["reached"]]),
        "_elapsed_s": to_num(df_raw[COL["elapsed"]]),
        "_deadline_violation": to_bool(df_raw[COL["deadline_violation"]]),
        "_zone_viol": to_num(df_raw[COL["zone_violations"]]),
        "_energy_kj": to_num(df_raw[COL["energy_kj"]]),
        "_mean_power_kw": to_num(df_raw[COL["mean_power_kw"]]),
    })

    # ---------- Round filter (retain ONLY full successful rounds APE1→... in `order`) ----------
    if len(order) == 0:
        raise ValueError("`order` must contain at least one strategy.")

    df2 = df.reset_index().rename(columns={"index": "_orig_idx"})
    is_first = df2["_strategy"] == order[0]
    round_starts = list(df2.index[is_first])

    valid_indices: list[int] = []
    for i, start_pos in enumerate(round_starts):
        end_pos = round_starts[i + 1] if (i + 1) < len(round_starts) else len(df2)
        chunk = df2.iloc[start_pos:end_pos]

        chosen_rows = []
        round_ok = True
        for strat in order:
            sub = chunk[chunk["_strategy"] == strat]
            if sub.empty:
                round_ok = False
                break
            row = sub.iloc[0]
            if not (isinstance(row["_reached"], (bool, np.bool_)) and bool(row["_reached"])):
                round_ok = False
                break
            chosen_rows.append(int(row["_orig_idx"]))

        if round_ok and len(chosen_rows) == len(order):
            valid_indices.extend(chosen_rows)

    df_f = df.loc[valid_indices].copy()
    if df_f.empty:
        # Return an empty summary but keep the same structure
        empty_csv = os.path.join(out_dir, f"strategy_summary_zone-{zm}.csv")
        pd.DataFrame().to_csv(empty_csv, index=False)
        return {
            "zone_metric": zm,
            "summary_csv": empty_csv,
            "summary": {},
        }

    # ---------- Per-strategy summary on filtered (successful-round) data ----------
    g = df_f.groupby("_strategy", dropna=False)

    def med(s: pd.Series) -> float:
        s = s.dropna()
        return float(s.median()) if len(s) else float("nan")

    def p95(s: pd.Series) -> float:
        s = s.dropna()
        return float(np.percentile(s, 95)) if len(s) else float("nan")

    def any_pos(s: pd.Series) -> float:
        s = s.dropna()
        if not len(s): return float("nan")
        return float((s > 0).mean())

    summary = pd.DataFrame({
        "runs_considered": g.size(),
        "deadline_violation_rate": g["_deadline_violation"].mean(),
        "reach_time_median_s": g["_elapsed_s"].apply(med),
        "reach_time_mean_s": g["_elapsed_s"].mean(),
        "reach_time_p95_s": g["_elapsed_s"].apply(p95),
        "zone_violations_mean": g["_zone_viol"].mean(),
        "zone_violations_median": g["_zone_viol"].median(),
        "p_any_zone_violation": g["_zone_viol"].apply(any_pos),
        "energy_kj_median": g["_energy_kj"].median(),
        # keep mean_power_kw parsed but not summarized/ranked; add optional visibility if desired:
        # "mean_power_kw_median": g["_mean_power_kw"].median(),
    })

    # ---------- Timeout rate over ALL runs (unfiltered) ----------
    g_all = df.groupby("_strategy", dropna=False)
    def timeout_rate(s_bool: pd.Series) -> float:
        s = s_bool.dropna().astype(bool)
        if not len(s):
            return float("nan")
        return float((~s).mean())
    summary = summary.join(
        g_all["_reached"].apply(timeout_rate).rename("timeout_rate_all"),
        how="left"
    )

    # ---------- Write summary CSV and return ----------
    summary_csv = os.path.join(out_dir, f"strategy_summary_zone-{zm}.csv")
    summary.round(6).to_csv(summary_csv)

    return {
        "zone_metric": zm,
        "summary_csv": summary_csv,
        "summary": summary.round(6).to_dict(orient="index"),
    }
