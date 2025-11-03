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
# Bettendorf-aware one-shot analysis (no per-event metrics)
# -------------------------------------------------------------------
def run_analysis(
    order=("APE1", "APE2", "APE3", "TROOP"),
    deadline_ok_tol: float = 0.0,   # admissible if deadline_violation_rate <= this
    zone_metric: str = "median",    # "median" (default) or "mean"
) -> Dict[str, Any]:
    """
    Analyze experiment CSV (B-style) and pick the winner.

    TeleopConfig (STRICT):
      - cfg.results_csv_path  : str
      - cfg.analyzer_out_dir  : str

    CSV (case-insensitive accepted):
      strategy | reached | elapse_time | event_violated
      [optional] zone_violations

    NEW: A full round = {APE1, APE2, APE3, TROOP} is kept only if *all* those runs
         reached=True; otherwise the whole round is dropped.

    zone_metric: which aggregation to use for zone violations in ranking & plots.
                 One of {"median","mean"} (default: "median").
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

    COL = {
        "strategy": resolve("strategy"),
        "reached": resolve("reached"),
        "elapsed": resolve("elapse_time"),
        "deadline_violation": resolve("event_violated"),
    }
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
        "_zone_viol": to_num(df_raw[COL["zone_violations"]]) if COL["zone_violations"] else 0.0,
    })

    # ---------- Round filter (retain ONLY full successful rounds APE1→APE2→APE3→TROOP) ----------
    if len(order) == 0:
        raise ValueError("`order` must contain at least one strategy.")

    df2 = df.reset_index().rename(columns={"index": "_orig_idx"})
    is_ape1 = df2["_strategy"] == order[0]
    ape1_starts = list(df2.index[is_ape1])

    valid_indices: list[int] = []
    for i, start_pos in enumerate(ape1_starts):
        end_pos = ape1_starts[i + 1] if (i + 1) < len(ape1_starts) else len(df2)
        chunk = df2.iloc[start_pos:end_pos]

        # Pick first occurrence of each strategy from the expected set
        chosen_rows = []
        round_ok = True
        for strat in order:
            sub = chunk[chunk["_strategy"] == strat]
            if sub.empty:
                round_ok = False
                break
            row = sub.iloc[0]
            r_reached = row["_reached"]
            if not (isinstance(r_reached, (bool, np.bool_)) and bool(r_reached)):
                round_ok = False
                break
            chosen_rows.append(int(row["_orig_idx"]))

        if round_ok and len(chosen_rows) == len(order):
            valid_indices.extend(chosen_rows)

    df_f = df.loc[valid_indices].copy()
    if df_f.empty:
        raise SystemExit("No valid rows after round filtering (every round had at least one timeout/failure).")

    # ---------- Per-strategy metrics on filtered (successful-round) data ----------
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

    # ---------- Tie-breaker metric over ALL runs (unfiltered): TIMEOUT RATE ----------
    g_all = df.groupby("_strategy", dropna=False)
    def timeout_rate(s_bool: pd.Series) -> float:
        s = s_bool.dropna().astype(bool)
        if not len(s):
            return float("nan")
        return float((~s).mean())
    timeout_rate_all = g_all["_reached"].apply(timeout_rate)
    timeout_rate_all.name = "timeout_rate_all"
    summary = summary.join(timeout_rate_all, how="left")

    # ---------- Choose the active zone metric ----------
    zone_col = "zone_violations_median" if zm == "median" else "zone_violations_mean"
    zone_label = f"{'Median' if zm == 'median' else 'Mean'} zone violations (per run)"

    # ---------- Validation: required fields for ranking & Pareto ----------
    required_cols = [
        "deadline_violation_rate",
        "reach_time_median_s",
        zone_col,
        "timeout_rate_all",
    ]
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

    # ---------- Admissibility (deadline gate) ----------
    S = summary.copy()
    S["admissible"] = S["deadline_violation_rate"] <= float(deadline_ok_tol)

    # ---------- Winner selection (lexicographic)
    # Sort by: deadline_violation_rate (asc) → median reach time (asc) → CHOSEN zone metric (asc) → timeout_rate_all (asc)
    S["reach_time_median_s_rank"] = S["reach_time_median_s"]
    S["zone_chosen_rank"] = S[zone_col]
    S_sort = S.sort_values(
        by=[
            "deadline_violation_rate",
            "reach_time_median_s_rank",
            "zone_chosen_rank",
            "timeout_rate_all",
        ],
        ascending=[True, True, True, True]
    )
    best_strategy = S_sort.index[0]

    # ---------- Pareto set (3D: time_median, chosen zone, timeout_rate) ----------
    pareto_cols = [
        "reach_time_median_s_rank",
        "zone_chosen_rank",
        "timeout_rate_all",
    ]
    vals = S[pareto_cols].values.astype(float)
    n = len(S)
    pareto = np.ones(n, dtype=bool)
    for i in range(n):
        if not pareto[i]:
            continue
        vi = vals[i]
        le_all = (vals <= vi)
        lt_any = (vals < vi)
        dominates_i = le_all.all(axis=1) & lt_any.any(axis=1)
        dominates_i[i] = False
        if dominates_i.any():
            pareto[i] = False
    S["pareto_optimal"] = pareto

    # ---------- Write summary CSV ----------
    summary_csv = os.path.join(out_dir, f"strategy_summary_zone-{zm}.csv")
    S.round(6).to_csv(summary_csv)

    # ---------- Plots (use CHOSEN zone metric throughout) ----------
    # 2D: deadline violation rate vs median reach time; bubble size = chosen zone metric
    plt.figure(figsize=(7, 5))
    x = S["deadline_violation_rate"].values
    y = S["reach_time_median_s"].values
    sizes = (S[zone_col].values + 0.05) * 1000
    colors = ["tab:blue"] * len(S)
    edges = ["tab:green" if adm else "k" for adm in S["admissible"].values]
    plt.scatter(x, y, s=sizes, c=colors, edgecolors=edges, linewidths=1.2)
    for name, xv, yv, is_p in zip(S.index, x, y, S["pareto_optimal"].values):
        if np.isfinite(yv):
            lbl = f"{name}{' *' if is_p else ''}"
            plt.annotate(lbl, (xv, yv), textcoords="offset points", xytext=(5, 5))
    plt.xlabel("Deadline violation rate (per run)")
    plt.ylabel("Median reach time (s)")
    plt.title(f"Trade-offs per strategy (size = {zone_label}; green edge = admissible)")
    plt.tight_layout()
    plot2d = os.path.join(out_dir, f"tradeoff_2d_zone-{zm}.png")
    plt.savefig(plot2d, dpi=160)
    plt.close()

    # 2D: deadline violation rate vs CHOSEN zone metric
    plt.figure(figsize=(7, 5))
    x2 = S["deadline_violation_rate"].values
    y2 = S[zone_col].values
    plt.scatter(x2, y2, edgecolors=edges, linewidths=1.0)
    for name, xv, yv in zip(S.index, x2, y2):
        if np.isfinite(yv):
            plt.annotate(name, (xv, yv), textcoords="offset points", xytext=(5, 5))
    plt.xlabel("Deadline violation rate (per run)")
    plt.ylabel(zone_label)
    plt.title(f"{zone_label} vs Deadline Violation Rate")
    plt.tight_layout()
    plot2d_zone_vs_deadline = os.path.join(out_dir, f"tradeoff_2d_zone_vs_deadline_zone-{zm}.png")
    plt.savefig(plot2d_zone_vs_deadline, dpi=160)
    plt.close()

    # 2D: CHOSEN zone metric vs MEDIAN reach time
    plt.figure(figsize=(7, 5))
    x3 = S[zone_col].values
    y3 = S["reach_time_median_s"].values
    plt.scatter(x3, y3, edgecolors=edges, linewidths=1.0)
    for name, xv, yv in zip(S.index, x3, y3):
        if np.isfinite(yv):
            plt.annotate(name, (xv, yv), textcoords="offset points", xytext=(5, 5))
    plt.xlabel(zone_label)
    plt.ylabel("Median reach time (s)")
    plt.title(f"Median Reach Time vs {zone_label}")
    plt.tight_layout()
    plot2d_reach_vs_zone = os.path.join(out_dir, f"tradeoff_2d_reach_vs_zone_zone-{zm}.png")
    plt.savefig(plot2d_reach_vs_zone, dpi=160)
    plt.close()

    # 3D: (deadline, MEDIAN reach time, CHOSEN zone metric)
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        S["deadline_violation_rate"].values,
        S["reach_time_median_s"].values,
        S[zone_col].values,
    )
    for name, xv, yv, zv in zip(
        S.index,
        S["deadline_violation_rate"].values,
        S["reach_time_median_s"].values,
        S[zone_col].values
    ):
        if np.isfinite(yv):
            ax.text(xv, yv, zv, name)
    ax.set_xlabel("Deadline violation rate")
    ax.set_ylabel("Median reach time (s)")
    ax.set_zlabel(zone_label)
    ax.set_title(f"3D Trade-off Landscape ({zone_label.lower()})")
    plt.tight_layout()
    plot3d = os.path.join(out_dir, f"tradeoff_3d_zone-{zm}.png")
    plt.savefig(plot3d, dpi=160)
    plt.close()

    # ---------- Return compact result ----------
    best_strategy_metrics = {
        "deadline_violation_rate": float(S.loc[best_strategy, "deadline_violation_rate"]),
        "reach_time_median_s": float(S.loc[best_strategy, "reach_time_median_s"]),
        "reach_time_p95_s": float(S.loc[best_strategy, "reach_time_p95_s"]),
        "zone_violations_median": float(S.loc[best_strategy, "zone_violations_median"]),
        "zone_violations_mean": float(S.loc[best_strategy, "zone_violations_mean"]),
        "zone_chosen": float(S.loc[best_strategy, zone_col]),
        "p_any_zone_violation": float(S.loc[best_strategy, "p_any_zone_violation"]),
        "runs_considered": int(S.loc[best_strategy, "runs_considered"]),
        "timeout_rate_all": float(S.loc[best_strategy, "timeout_rate_all"]),
        "admissible": bool(S.loc[best_strategy, "admissible"]),
        "pareto_optimal": bool(S.loc[best_strategy, "pareto_optimal"]),
    }

    return {
        "best_strategy": str(best_strategy),
        "zone_metric": zm,
        "metrics": best_strategy_metrics,
        "summary_csv": summary_csv,
        "plot_2d": plot2d,
        "plot_3d": plot3d,
        "plot_2d_zone_vs_deadline": plot2d_zone_vs_deadline,
        "plot_2d_reach_vs_zone": plot2d_reach_vs_zone,
    }
