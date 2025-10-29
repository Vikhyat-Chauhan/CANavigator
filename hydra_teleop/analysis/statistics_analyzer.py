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
# Bettendorf-aware one-shot analysis
# -------------------------------------------------------------------
def run_analysis(
    order=("APE1", "APE2", "APE3", "TROOP"),
    deadline_ok_tol: float = 0.0,   # admissible if deadline_violation_rate <= this
) -> Dict[str, Any]:
    """
    Analyze experiment CSV (B-style) and pick the winner.

    TeleopConfig (STRICT):
      - cfg.results_csv_path  : str
      - cfg.analyzer_out_dir  : str

    CSV (case-insensitive accepted):
      strategy | reached | elapse_time | event_violated
      [optional] events_handled | zone_violations
    """
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

    COL = {
        "strategy": resolve("strategy"),
        "reached": resolve("reached"),
        "elapsed": resolve("elapse_time"),
        "deadline_violation": resolve("event_violated"),
    }
    try:
        COL["events_handled"] = resolve("events_handled")
    except KeyError:
        COL["events_handled"] = None
    try:
        COL["zone_violations"] = resolve("zone_violations")
    except KeyError:
        COL["zone_violations"] = None

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
        "_events_handled": to_num(df_raw[COL["events_handled"]]) if COL["events_handled"] else np.nan,
        "_zone_viol": to_num(df_raw[COL["zone_violations"]]) if COL["zone_violations"] else 0.0,
    })

    # ---------- Round filter (retain only successful reaches; enforce sequence blocks) ----------
    valid = []
    skip_until_next_ape1 = False
    for idx, r in df.iterrows():
        strat = r["_strategy"]
        in_order = strat in order
        if in_order and strat == order[0]:  # APE1 resets a "round"
            skip_until_next_ape1 = False
        if in_order and skip_until_next_ape1:
            continue
        if not (isinstance(r["_reached"], (bool, np.bool_)) and bool(r["_reached"])):
            if in_order:
                skip_until_next_ape1 = True
            continue
        valid.append(idx)

    df_f = df.loc[valid].copy()
    if df_f.empty:
        raise SystemExit("No valid rows after round filtering.")

    # ---------- Per-strategy metrics ----------
    g = df_f.groupby("_strategy", dropna=False)

    def med(s: pd.Series) -> float:
        s = s.dropna()
        return float(s.median()) if len(s) else float("nan")

    def p95(s: pd.Series) -> float:
        s = s.dropna()
        return float(np.percentile(s, 95)) if len(s) else float("nan")

    def any_zone(s: pd.Series) -> float:
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
        "p_any_zone_violation": g["_zone_viol"].apply(any_zone),
    })

    # Validation: required fields for ranking
    required_cols = ["deadline_violation_rate", "reach_time_median_s", "zone_violations_mean"]
    nan_mask = summary[required_cols].isna()
    if nan_mask.any().any():
        problems = []
        for strat, row in nan_mask.iterrows():
            missing_fields = [col for col, is_nan in row.items() if is_nan]
            if missing_fields:
                problems.append(f"{strat}: missing {', '.join(missing_fields)}")
        details = "; ".join(problems) if problems else "unknown"
        raise ValueError(
            "Missing values detected in required metrics; cannot rank/Pareto with incomplete data. "
            f"Please fix your input CSV or drop problematic rows.\nDetails: {details}"
        )

    # Per-event violation rate (if possible)
    if COL["events_handled"]:
        total_viols = g["_deadline_violation"].sum(min_count=1)
        total_events = g["_events_handled"].sum(min_count=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            per_event = total_viols / total_events
        summary["per_event_violation_rate"] = per_event

    # ---------- Admissibility (Bettendorf: deadlines are hard) ----------
    summary["admissible"] = summary["deadline_violation_rate"] <= float(deadline_ok_tol)

    # ---------- Lexicographic winner (deadline → time → zone → runs) ----------
    S = summary.copy()
    S["reach_time_median_s_rank"] = S["reach_time_median_s"]
    S["zone_violations_mean_rank"] = S["zone_violations_mean"]
    S_sort = S.sort_values(
        by=["deadline_violation_rate", "reach_time_median_s_rank", "zone_violations_mean_rank", "runs_considered"],
        ascending=[True, True, True, False]
    )
    best_strategy = S_sort.index[0]
    best_row = S.loc[best_strategy]

    # ---------- Pareto set on (deadline, time, zone) ----------
    vals = S[["deadline_violation_rate", "reach_time_median_s_rank", "zone_violations_mean_rank"]].values
    pareto = np.ones(len(S), dtype=bool)
    for i, vi in enumerate(vals):
        if not pareto[i]:
            continue
        # any j dominates i if j <= i in all coords and < in at least one
        dom = (vals <= vi).all(axis=1) & (vals < vi).any(axis=1)
        dom &= (np.arange(len(S)) != i)
        if dom.any():
            pareto[i] = False
    S["pareto_optimal"] = pareto

    # ---------- Write summary CSV ----------
    summary_csv = os.path.join(out_dir, "strategy_summary.csv")
    S.round(6).to_csv(summary_csv)

    # ---------- Plots ----------
    # 2D: violation rate vs median reach; bubble=size zone_mean; star on Pareto; green edge if admissible
    plt.figure(figsize=(7, 5))
    x = S["deadline_violation_rate"].values
    y = S["reach_time_median_s"].values
    sizes = (S["zone_violations_mean"].values + 0.05) * 1000
    colors = ["tab:blue"] * len(S)
    edges = ["tab:green" if adm else "k" for adm in S["admissible"].values]
    plt.scatter(x, y, s=sizes, c=colors, edgecolors=edges, linewidths=1.2)
    for name, xv, yv, is_p in zip(S.index, x, y, S["pareto_optimal"].values):
        if np.isfinite(yv):
            lbl = f"{name}{' *' if is_p else ''}"
            plt.annotate(lbl, (xv, yv), textcoords="offset points", xytext=(5, 5))
    plt.xlabel("Deadline violation rate (per run)")
    plt.ylabel("Median reach time (s)")
    plt.title("Trade-offs per strategy (size = mean zone violations; green edge = admissible)")
    plt.tight_layout()
    plot2d = os.path.join(out_dir, "tradeoff_2d.png")
    plt.savefig(plot2d, dpi=160)
    plt.close()

    # 2D: MEAN zone violations vs deadline violation rate
    plt.figure(figsize=(7, 5))
    x2 = S["deadline_violation_rate"].values
    y2 = S["zone_violations_mean"].values
    plt.scatter(x2, y2, edgecolors=edges, linewidths=1.0)
    for name, xv, yv in zip(S.index, x2, y2):
        if np.isfinite(yv):
            plt.annotate(name, (xv, yv), textcoords="offset points", xytext=(5, 5))
    plt.xlabel("Deadline violation rate (per run)")
    plt.ylabel("Mean zone violations (per run)")
    plt.title("Mean Zone Violations vs Deadline Violation Rate")
    plt.tight_layout()
    plot2d_zoneMean_vs_deadline = os.path.join(out_dir, "tradeoff_2d_zoneMean_vs_deadline.png")
    plt.savefig(plot2d_zoneMean_vs_deadline, dpi=160)
    plt.close()

    # 2D: MEDIAN reach time vs MEAN zone violations
    plt.figure(figsize=(7, 5))
    x3 = S["zone_violations_mean"].values
    y3 = S["reach_time_median_s"].values
    plt.scatter(x3, y3, edgecolors=edges, linewidths=1.0)
    for name, xv, yv in zip(S.index, x3, y3):
        if np.isfinite(yv):
            plt.annotate(name, (xv, yv), textcoords="offset points", xytext=(5, 5))
    plt.xlabel("Mean zone violations (per run)")
    plt.ylabel("Median reach time (s)")
    plt.title("Median Reach Time vs Mean Zone Violations")
    plt.tight_layout()
    plot2d_reachMedian_vs_zoneMean = os.path.join(out_dir, "tradeoff_2d_reachMedian_vs_zoneMean.png")
    plt.savefig(plot2d_reachMedian_vs_zoneMean, dpi=160)
    plt.close()

    # 3D: (violations, reach_time, zone_mean)
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

    # ---------- Return compact result ----------
    return {
        "best_strategy": str(best_strategy),
        "metrics": {
            "deadline_violation_rate": float(S.loc[best_strategy, "deadline_violation_rate"]),
            "reach_time_median_s": float(S.loc[best_strategy, "reach_time_median_s"]),
            "reach_time_p95_s": float(S.loc[best_strategy, "reach_time_p95_s"]),
            "zone_violations_mean": float(S.loc[best_strategy, "zone_violations_mean"]),
            "p_any_zone_violation": float(S.loc[best_strategy, "p_any_zone_violation"]),
            "per_event_violation_rate": float(S.loc[best_strategy, "per_event_violation_rate"]) if "per_event_violation_rate" in S.columns and not pd.isna(S.loc[best_strategy, "per_event_violation_rate"]) else None,
            "runs_considered": int(S.loc[best_strategy, "runs_considered"]),
            "admissible": bool(S.loc[best_strategy, "admissible"]),
            "pareto_optimal": bool(S.loc[best_strategy, "pareto_optimal"]),
        },
        "summary_csv": summary_csv,
        "plot_2d": plot2d,
        "plot_3d": plot3d,
        "plot_2d_zoneMean_vs_deadline": plot2d_zoneMean_vs_deadline,
        "plot_2d_reachMedian_vs_zoneMean": plot2d_reachMedian_vs_zoneMean,
    }
