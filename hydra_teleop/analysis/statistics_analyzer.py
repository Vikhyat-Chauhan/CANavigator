# hydra_teleop/analysis/statistics_analyzer.py
from __future__ import annotations
import os, re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from typing import Dict, Any
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# -------------------------------------------------------------------
# B-style one-shot analysis, callable from main
# -------------------------------------------------------------------
def run_analysis(
    order=("APE1", "APE2", "APE3", "TROOP"),
) -> Dict[str, Any]:
    """
    Analyze experiment CSV (B-style) and pick the winner.

    TeleopConfig (STRICT):
      - cfg.results_csv_path  : str, path to input CSV (e.g., '.../experiment_reached_summary.csv')
      - cfg.analyzer_out_dir  : str, path to output directory for artifacts

    CSV columns (case-insensitive names accepted):
      strategy | reached | elapse_time | event_violated | (optional) events_handled | (optional) zone_violations

    Returns:
      {
        "best_strategy": str,
        "metrics": {
          "deadline_violation_rate": float,
          "reach_time_median_s": float,
          "zone_violations_mean": float,
          "runs_considered": int,
          "pareto_optimal": bool,
        },
        "summary_csv": str,
        "plot_2d": str,
        "plot_3d": str,
      }
    """
    # --- Strict dependency on TeleopConfig ---
    from hydra_teleop.config import TeleopConfig  # hard dependency by request
    cfg = TeleopConfig()

    if not hasattr(cfg, "results_csv_path") or not isinstance(cfg.results_csv_path, str) or not cfg.results_csv_path:
        raise AttributeError("TeleopConfig must define a non-empty string 'results_csv_path' (input CSV path).")
    if not hasattr(cfg, "analyzer_out_dir") or not isinstance(cfg.analyzer_out_dir, str) or not cfg.analyzer_out_dir:
        raise AttributeError("TeleopConfig must define a non-empty string 'analyzer_out_dir' (output folder).")

    csv_path: str = cfg.results_csv_path
    out_dir: str = cfg.analyzer_out_dir

    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV not found at {csv_path}")
    os.makedirs(out_dir, exist_ok=True)

    # Read CSV once, BEFORE using resolve()
    df_raw = pd.read_csv(csv_path)

    # ---- column resolution (case-insensitive exact; then loose) ----
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
    }
    # optional
    try:
        COL["events_handled"] = resolve("events_handled")
    except KeyError:
        COL["events_handled"] = None
    try:
        COL["zone_violations"] = resolve("zone_violations")
    except KeyError:
        COL["zone_violations"] = None

    # ---- coercions ----
    def to_bool(s: pd.Series) -> pd.Series:
        mapped = s.astype(str).str.strip().str.lower().map({
            "true": True, "1": True, "yes": True, "y": True, "t": True,
            "false": False, "0": False, "no": False, "n": False, "f": False
        })
        return mapped.fillna(False)

    def to_num(s: pd.Series) -> pd.Series:
        return pd.to_numeric(s, errors="coerce")

    df = pd.DataFrame({
        "_strategy": df_raw[COL["strategy"]].astype(str).str.strip(),
        "_reached": to_bool(df_raw[COL["reached"]]),
        "_elapsed_s": to_num(df_raw[COL["elapsed"]]),
        "_deadline_violation": to_bool(df_raw[COL["deadline_violation"]]),
        "_events_handled": to_num(df_raw[COL["events_handled"]]) if COL["events_handled"] else np.nan,
        "_zone_viol": to_num(df_raw[COL["zone_violations"]]) if COL["zone_violations"] else 0.0,
    })

    # ---- round filter ----
    valid = []
    skip_until_next_ape1 = False
    for idx, r in df.iterrows():
        strat = r["_strategy"]
        in_order = strat in order

        if in_order and strat == order[0]:  # APE1 resets round
            skip_until_next_ape1 = False

        if in_order and skip_until_next_ape1:
            continue

        if not bool(r["_reached"]):
            if in_order:
                skip_until_next_ape1 = True
            continue

        valid.append(idx)

    df_f = df.loc[valid].copy()
    if df_f.empty:
        raise SystemExit("No valid rows after round filtering.")

    # ---- Per-strategy metrics ----
    g = df_f.groupby("_strategy", dropna=False)

    def med(s: pd.Series) -> float:
        s = s.dropna()
        return float(s.median()) if len(s) else float("nan")

    summary = pd.DataFrame({
        "runs_considered": g.size(),
        "deadline_violation_rate": g["_deadline_violation"].mean(),
        "reach_time_median_s": g["_elapsed_s"].apply(med),
        "reach_time_mean_s": g["_elapsed_s"].mean(),
        "zone_violations_mean": g["_zone_viol"].mean(),
        "zone_violations_median": g["_zone_viol"].median(),
    })

    # Per-event violation rate (if events_handled exists)
    if df_f["_events_handled"].notna().any():
        total_viols = g["_deadline_violation"].sum()
        total_events = g["_events_handled"].sum().replace(0, np.nan)
        summary["per_event_violation_rate"] = (total_viols / total_events)

    # ---- Lexicographic winner: (violations → reach time → zone) ----
    S = summary.copy()
    S["reach_time_median_s_rank"] = S["reach_time_median_s"].fillna(np.inf)
    S["zone_violations_mean_rank"] = S["zone_violations_mean"].fillna(np.inf)
    S_sort = S.sort_values(
        by=["deadline_violation_rate", "reach_time_median_s_rank", "zone_violations_mean_rank", "runs_considered"],
        ascending=[True, True, True, False]
    )
    best_strategy = S_sort.index[0]
    best_row = S.loc[best_strategy]

    # ---- Pareto (minimize all three) ----
    vals = S[["deadline_violation_rate", "reach_time_median_s_rank", "zone_violations_mean_rank"]].values
    pareto = np.ones(len(S), dtype=bool)
    for i, vi in enumerate(vals):
        if not pareto[i]:
            continue
        dom = np.all(vals <= vi, axis=1) & np.any(vals < vi, axis=1)
        dom &= (np.arange(len(S)) != i)
        if dom.any():
            pareto[i] = False
    S["pareto_optimal"] = pareto

    # ---- Write summary CSV ----
    summary_csv = os.path.join(out_dir, "strategy_summary.csv")
    S.round(6).to_csv(summary_csv)

    # ---- Plots ----
    # 2D: violation rate vs median reach time, bubble = mean zone viol
    plt.figure(figsize=(7, 5))
    x = S["deadline_violation_rate"].values
    y = S["reach_time_median_s"].values
    sizes = (S["zone_violations_mean"].fillna(0.0).values + 0.05) * 1000
    plt.scatter(x, y, s=sizes)
    for name, xv, yv, is_p in zip(S.index, x, y, S["pareto_optimal"].values):
        if np.isfinite(yv):
            lbl = f"{name}{' *' if is_p else ''}"
            plt.annotate(lbl, (xv, yv), textcoords="offset points", xytext=(5, 5))
    plt.xlabel("Deadline violation rate (per run)")
    plt.ylabel("Median reach time (s)")
    plt.title("Trade-offs per strategy (size = mean zone violations)")
    plt.tight_layout()
    plot2d = os.path.join(out_dir, "tradeoff_2d.png")
    plt.savefig(plot2d, dpi=160)
    plt.close()

    # 3D: (violations, reach_time, zone_viol)
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        S["deadline_violation_rate"].values,
        S["reach_time_median_s"].values,
        S["zone_violations_mean"].values,
    )
    for name, xv, yv, zv in zip(S.index, S["deadline_violation_rate"].values, S["reach_time_median_s"].values, S["zone_violations_mean"].values):
        if np.isfinite(yv):
            ax.text(xv, yv, zv, name)
    ax.set_xlabel("Deadline violation rate")
    ax.set_ylabel("Median reach time (s)")
    ax.set_zlabel("Mean zone violations")
    ax.set_title("3D Trade-off Landscape")
    plt.tight_layout()
    plot3d = os.path.join(out_dir, "tradeoff_3d.png")
    plt.savefig(plot3d, dpi=160)
    plt.close()

    # ---- Return a compact result ----
    return {
        "best_strategy": best_strategy,
        "metrics": {
            "deadline_violation_rate": None if pd.isna(best_row["deadline_violation_rate"]) else float(best_row["deadline_violation_rate"]),
            "reach_time_median_s": None if pd.isna(best_row["reach_time_median_s"]) else float(best_row["reach_time_median_s"]),
            "zone_violations_mean": None if pd.isna(best_row["zone_violations_mean"]) else float(best_row["zone_violations_mean"]),
            "runs_considered": int(S.loc[best_strategy, "runs_considered"]),
            "pareto_optimal": bool(S.loc[best_strategy, "pareto_optimal"]),
        },
        "summary_csv": summary_csv,
        "plot_2d": plot2d,
        "plot_3d": plot3d,
    }
