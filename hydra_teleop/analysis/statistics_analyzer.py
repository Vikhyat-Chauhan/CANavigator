# hydra_teleop/plugins/statics_analyser.py
#!/usr/bin/env python3
"""
Stats analyzer for Hydra experiment CSVs (import-only).
Supports 2–4 strategies (APEs), works with the "generated CSV"
schema produced by hydra_teleop.tools.log_to_csv (one row per run end).

Required columns (case-insensitive; aliases supported):
- strategy
- reached (bool-like)
- elapsed_time (seconds)  -> aliased to elapsed_s
- zone_violations         -> aliased to violations

Optional event metrics:
- events_deadline_miss
- events_success
- events_handled
- events_preemptive

New: Pareto analysis minimizing:
- avg elapsed time
- avg zone violations
- avg event deadline misses

Artifacts written to cfg.analyzer_out_dir:
- successful_summaries.csv
- runs_scatter_*.png (zone violations, deadline misses, events success)
- avg_*_vs_time.png
- box_*.png
- pareto_summary.csv
- pareto_scatter.png
"""

from __future__ import annotations
import os
import math
import random
from typing import Dict, List, Optional, Sequence, Set

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import warnings
warnings.filterwarnings("ignore", category=matplotlib.MatplotlibDeprecationWarning)
import matplotlib.pyplot as plt

from hydra_teleop.config import TeleopConfig


# ---------------- Aliases & helpers ----------------

ALIASES = {
    "run": ["run", "trial", "seed", "idx"],  # optional; synthesized if absent
    "strategy": ["strategy", "algo", "algorithm"],
    "reached": ["reached", "success", "done"],
    "elapsed_s": ["elapsed_s", "elapsed_time", "time_s", "time", "duration_s"],
    # primary "violations" axis continues to use zone_violations
    "violations": ["violations", "zone_violations", "nfz_violations"],

    # optional event metrics
    "events_deadline_miss": ["events_deadline_miss", "deadline_miss", "deadline_misses"],
    "events_success": ["events_success", "handled_success", "events_handled_success"],
    "events_handled": ["events_handled", "events_total", "num_events"],
    "events_preemptive": ["events_preemptive", "preemptive", "preempted"],
}

def _resolve_columns(df: pd.DataFrame) -> Dict[str, str]:
    lower = {c.lower(): c for c in df.columns}
    out = {}
    for canonical, options in ALIASES.items():
        for opt in options:
            if opt.lower() in lower:
                out[canonical] = lower[opt.lower()]
                break
        # 'run' and optional event metrics are not required
        if canonical not in out and canonical not in {"run", "events_deadline_miss", "events_success", "events_handled", "events_preemptive"}:
            raise KeyError(
                f"Missing required column like: '{canonical}' (aliases: {options}). "
                f"Found columns: {list(df.columns)}"
            )
    return out

def _to_bool_series(s: pd.Series) -> pd.Series:
    return s.astype(str).str.strip().str.lower().isin(["true", "1", "yes", "y", "t"])

def _ensure_outdir(path: str) -> None:
    os.makedirs(path, exist_ok=True)

def _aggregate_by_strategy(stats: List[dict], metric_keys: List[str]) -> List[dict]:
    agg = {}
    for row in stats:
        s = row["strategy"]
        d = agg.setdefault(s, {"n": 0, "sum_elapsed": 0.0})
        if not math.isnan(row["elapsed_s"]):
            d["sum_elapsed"] += row["elapsed_s"]
        for mk in metric_keys:
            d[f"sum_{mk}"] = d.get(f"sum_{mk}", 0.0) + float(row[mk])
        d["n"] += 1

    rows = []
    for s, d in agg.items():
        n = max(d["n"], 1)
        entry = {
            "strategy": s,
            "n_runs": d["n"],
            "avg_elapsed_s": d["sum_elapsed"] / n,
        }
        for mk in metric_keys:
            entry[f"avg_{mk}"] = d[f"sum_{mk}"] / n
        rows.append(entry)
    rows.sort(key=lambda r: r["strategy"])
    return rows

def _plot_avg_metric_vs_time(agg_rows: List[dict], metric_key: str, label: str, out_path: str) -> None:
    x = [r["avg_elapsed_s"] for r in agg_rows]
    y = [r[f"avg_{metric_key}"] for r in agg_rows]
    labels = [r["strategy"] for r in agg_rows]

    plt.figure(figsize=(6, 4))
    plt.scatter(x, y)
    for xi, yi, lbl in zip(x, y, labels):
        plt.annotate(lbl, (xi, yi), textcoords="offset points", xytext=(6, 6))
    plt.xlabel("Average elapsed time (s)")
    plt.ylabel(f"Average {label}")
    plt.title(f"Average {label} vs Average Time by Strategy")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()

def _plot_per_run_scatter_metric(
    stats: List[dict],
    metric_key: str,
    ylabel: str,
    out_path: str,
    annotate: bool,
    include_incomplete_or_timeouts: bool,
    clean_runs: Set[int],
) -> None:
    random.seed(42)
    by_strategy = {}
    for row in stats:
        if math.isnan(row["elapsed_s"]):
            continue
        by_strategy.setdefault(row["strategy"], []).append(row)

    plt.figure(figsize=(12, 7))
    legend_handles = []

    for strategy, rows in sorted(by_strategy.items(), key=lambda kv: kv[0]):
        xs_clean, ys_clean, pts_clean = [], [], []
        xs_mixed, ys_mixed, pts_mixed = [], [], []

        for r in rows:
            x = r["elapsed_s"]
            y = float(r[metric_key]) + random.uniform(-0.06, 0.06)  # tiny jitter
            if r["run"] in clean_runs:
                xs_clean.append(x); ys_clean.append(y); pts_clean.append(r)
            else:
                xs_mixed.append(x); ys_mixed.append(y); pts_mixed.append(r)

        if xs_clean:
            h_clean = plt.scatter(xs_clean, ys_clean, marker="o",
                                  label=f"{strategy} (clean)", alpha=0.35, s=20)
            legend_handles.append(h_clean)

        if include_incomplete_or_timeouts and xs_mixed:
            h_mixed = plt.scatter(xs_mixed, ys_mixed, marker="x",
                                  label=f"{strategy} (mixed/timeout)", alpha=0.5, s=18)
            legend_handles.append(h_mixed)

        if annotate:
            def _label_point(x, y, r):
                txt = f"r{r['run']} · {r['strategy']} · {r['elapsed_s']:.1f}s · {metric_key}={r[metric_key]}"
                plt.annotate(txt, (x, y), textcoords="offset points", xytext=(6, 5),
                             fontsize=7, alpha=0.9)
            for x, y, r in zip(xs_clean, ys_clean, pts_clean):
                _label_point(x, y, r)
            if include_incomplete_or_timeouts:
                for x, y, r in zip(xs_mixed, ys_mixed, pts_mixed):
                    _label_point(x, y, r)

    plt.xlabel("Elapsed time per run (s)")
    plt.ylabel(ylabel)
    ttl = f"Per-Run Time vs {ylabel}"
    if not include_incomplete_or_timeouts:
        ttl += " (clean runs only)"
    plt.title(ttl)
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
    if legend_handles:
        plt.legend(handles=legend_handles, loc="upper left", fontsize=9, frameon=False)
    plt.tight_layout()
    plt.savefig(out_path, dpi=180)
    plt.close()


# ---------------- Analyzer class ----------------

class StatsAnalyzer:
    """
    Build from TeleopConfig and run:
        StatsAnalyzer.from_config(cfg).analyze()

    Supports 2–4 strategies. If cfg.analyzer_strategies is not set,
    uses up to the first four strategies present in the CSV.

    Compatible with the generated CSV schema:
      - strategy
      - reached
      - elapsed_time (aliased to elapsed_s)
      - zone_violations (aliased to violations)
      - (optional) run
      - (optional) events_deadline_miss, events_success, events_handled, events_preemptive
    """

    def __init__(
        self,
        input_csv: str,
        out_dir: str,
        strategies: Sequence[str],
        annotate_points: bool,
        exclude_incomplete_or_timeouts: bool,
    ):
        self.input_csv = input_csv
        self.out_dir = out_dir
        self.strategies = list(strategies)
        self.annotate_points = annotate_points
        self.exclude_incomplete_or_timeouts = exclude_incomplete_or_timeouts

    @classmethod
    def from_config(cls, cfg: TeleopConfig) -> "StatsAnalyzer":
        chosen: Optional[Sequence[str]] = getattr(cfg, "analyzer_strategies", None)
        if not chosen and getattr(cfg, "analyzer_pair_order", None):
            chosen = list(cfg.analyzer_pair_order)

        return cls(
            input_csv=cfg.results_csv_path,
            out_dir=cfg.analyzer_out_dir,
            strategies=chosen or [],
            annotate_points=cfg.analyzer_annotate_points,
            exclude_incomplete_or_timeouts=not cfg.analyzer_include_timeouts,
        )

    def _resolve_strategies(self, df: pd.DataFrame) -> List[str]:
        present = df["strategy"].unique().tolist()
        if self.strategies:
            missing = [s for s in self.strategies if s not in present]
            if missing:
                raise SystemExit(
                    f"Selected strategies not present in data: {missing}. "
                    f"Found strategies: {present}"
                )
            if not (2 <= len(self.strategies) <= 4):
                raise SystemExit("Please select between 2 and 4 strategies.")
            return list(self.strategies)
        if len(present) < 2:
            raise SystemExit(f"Need at least 2 strategies; found: {present}")
        return present[:4]

    def analyze(self) -> None:
        assert os.path.exists(self.input_csv), f"Input not found: {self.input_csv}"
        _ensure_outdir(self.out_dir)

        # Load & normalize (map to canonical column names)
        df_raw = pd.read_csv(self.input_csv)
        cols = _resolve_columns(df_raw)

        # If 'run' is missing, synthesize a stable run index (1..N by CSV order)
        if "run" in cols:
            run_col = cols["run"]
            run_series = df_raw[run_col].astype(int)
        else:
            run_series = pd.Series(range(1, len(df_raw) + 1), index=df_raw.index, name="run")

        def _num(col_key: str) -> pd.Series:
            if col_key in cols:
                return pd.to_numeric(df_raw[cols[col_key]], errors="coerce").fillna(0)
            return pd.Series(0, index=df_raw.index, dtype=float)

        df = pd.DataFrame({
            "run": run_series,
            "strategy": df_raw[cols["strategy"]].astype(str),
            "reached": _to_bool_series(df_raw[cols["reached"]]),
            "elapsed_s": pd.to_numeric(df_raw[cols["elapsed_s"]], errors="coerce"),
            "violations": pd.to_numeric(df_raw[cols["violations"]], errors="coerce").fillna(0),
            "events_deadline_miss": _num("events_deadline_miss"),
            "events_success": _num("events_success"),
            "events_handled": _num("events_handled"),
            "events_preemptive": _num("events_preemptive"),
        })

        # Decide which strategies to include (2–4)
        selected = self._resolve_strategies(df)
        df = df[df["strategy"].isin(selected)].copy()

        # Build completeness map
        ct_per_run = (df.groupby(["run"])["strategy"].nunique().rename("n_strats_in_run"))
        reached_per_run = (df.groupby(["run"])["reached"].agg(all_reached=lambda x: bool(np.all(x))))
        run_meta = pd.concat([ct_per_run, reached_per_run], axis=1).reset_index()

        complete_runs = set(run_meta[run_meta["n_strats_in_run"] == len(selected)]["run"].astype(int).tolist())
        clean_runs = set(run_meta[
            (run_meta["n_strats_in_run"] == len(selected)) &
            (run_meta["all_reached"])
        ]["run"].astype(int).tolist())

        # Filtering
        if self.exclude_incomplete_or_timeouts and len(complete_runs) > 0:
            df_filt = df[df["run"].isin(clean_runs)].copy()
        else:
            df_filt = df.copy()

        # Rebuild stats list after filtering
        stats = [{
            "run": int(r.run),
            "strategy": str(r.strategy),
            "reached": bool(r.reached),
            "elapsed_s": float(r.elapsed_s) if pd.notna(r.elapsed_s) else float("nan"),
            "violations": float(r.violations),
            "events_deadline_miss": float(r.events_deadline_miss),
            "events_success": float(r.events_success),
            "events_handled": float(r.events_handled),
            "events_preemptive": float(r.events_preemptive),
        } for r in df_filt.itertuples(index=False)]

        # Summaries on successful only
        succ_df = df_filt[df_filt["reached"]].copy()

        def iqr(x):
            q75, q25 = np.percentile(x, [75, 25])
            return float(q75 - q25)

        # ---- Expanded summary (successful runs only) ----
        if len(succ_df) > 0:
            summ = (succ_df.groupby("strategy")
                    .agg(n=("elapsed_s", "size"),
                         mean_time_s=("elapsed_s", "mean"),
                         median_time_s=("elapsed_s", "median"),
                         p90_time_s=("elapsed_s", lambda x: np.percentile(x, 90)),
                         iqr_time_s=("elapsed_s", iqr),

                         mean_zone_viol=("violations", "mean"),
                         median_zone_viol=("violations", "median"),
                         p90_zone_viol=("violations", lambda x: np.percentile(x, 90)),

                         mean_deadline_miss=("events_deadline_miss", "mean"),
                         median_deadline_miss=("events_deadline_miss", "median"),
                         p90_deadline_miss=("events_deadline_miss", lambda x: np.percentile(x, 90)),

                         mean_events_success=("events_success", "mean"),
                         median_events_success=("events_success", "median"),
                         p90_events_success=("events_success", lambda x: np.percentile(x, 90)),
                    )
                    .reset_index()
                    .sort_values("strategy"))
        else:
            summ = pd.DataFrame(columns=[
                "strategy","n","mean_time_s","median_time_s","p90_time_s","iqr_time_s",
                "mean_zone_viol","median_zone_viol","p90_zone_viol",
                "mean_deadline_miss","median_deadline_miss","p90_deadline_miss",
                "mean_events_success","median_events_success","p90_events_success",
            ])

        summ_out = os.path.join(self.out_dir, "successful_summaries.csv")
        summ.round(4).to_csv(summ_out, index=False)

        # ---- Plots (per-run) ----
        _plot_per_run_scatter_metric(
            stats, "violations", "Zone Violations (count)",
            os.path.join(self.out_dir, "runs_scatter_zone_violations.png"),
            annotate=self.annotate_points,
            include_incomplete_or_timeouts=not self.exclude_incomplete_or_timeouts,
            clean_runs=clean_runs,
        )
        _plot_per_run_scatter_metric(
            stats, "events_deadline_miss", "Event Deadline Misses (count)",
            os.path.join(self.out_dir, "runs_scatter_deadline_miss.png"),
            annotate=self.annotate_points,
            include_incomplete_or_timeouts=not self.exclude_incomplete_or_timeouts,
            clean_runs=clean_runs,
        )
        _plot_per_run_scatter_metric(
            stats, "events_success", "Events Handled Successfully (count)",
            os.path.join(self.out_dir, "runs_scatter_events_success.png"),
            annotate=self.annotate_points,
            include_incomplete_or_timeouts=not self.exclude_incomplete_or_timeouts,
            clean_runs=clean_runs,
        )

        # ---- Average metric vs average time by strategy ----
        agg_rows = _aggregate_by_strategy(
            stats,
            metric_keys=["violations", "events_deadline_miss", "events_success"]
        )
        _plot_avg_metric_vs_time(
            agg_rows, "violations", "Zone Violations",
            os.path.join(self.out_dir, "avg_zone_violations_vs_time.png")
        )
        _plot_avg_metric_vs_time(
            agg_rows, "events_deadline_miss", "Event Deadline Misses",
            os.path.join(self.out_dir, "avg_deadline_miss_vs_time.png")
        )
        _plot_avg_metric_vs_time(
            agg_rows, "events_success", "Events Handled Successfully",
            os.path.join(self.out_dir, "avg_events_success_vs_time.png")
        )

        # =========================
        # Pareto trade-off analysis
        # =========================
        # Build per-strategy averages from df_filt (all rows, not only successful)
        by_strat = (df_filt.groupby("strategy")
                    .agg(avg_elapsed_s=("elapsed_s", "mean"),
                         avg_zone_viol=("violations", "mean"),
                         avg_deadline_miss=("events_deadline_miss", "mean"),
                         runs=("elapsed_s", "size"))
                    .reset_index())

        # Compute Pareto frontier for minimization of all three metrics
        vals = by_strat[["avg_elapsed_s", "avg_zone_viol", "avg_deadline_miss"]].values
        is_dominated = np.zeros(len(by_strat), dtype=bool)
        for i in range(len(by_strat)):
            for j in range(len(by_strat)):
                if i == j:
                    continue
                # j dominates i if j is <= in all and < in at least one
                if np.all(vals[j] <= vals[i]) and np.any(vals[j] < vals[i]):
                    is_dominated[i] = True
                    break

        by_strat["pareto_optimal"] = ~is_dominated

        # Choose the "best-for-speed under constraints"
        # = Pareto-optimal with minimal avg_elapsed_s, then min zone, then min deadline
        best = (by_strat[by_strat["pareto_optimal"]]
                .sort_values(["avg_elapsed_s", "avg_zone_viol", "avg_deadline_miss"],
                             ascending=[True, True, True])
                .head(1))
        best_strategy = best["strategy"].iloc[0] if len(best) else None

        # Save Pareto summary
        pareto_out = os.path.join(self.out_dir, "pareto_summary.csv")
        (by_strat
         .sort_values(["pareto_optimal", "avg_elapsed_s", "avg_zone_viol", "avg_deadline_miss"],
                      ascending=[False, True, True, True])
         .round(4)
         .to_csv(pareto_out, index=False))

        # Plot: 2D scatter (time vs zone), color = deadline misses; circle Pareto points
        plt.figure(figsize=(8, 6))
        x = by_strat["avg_elapsed_s"].values
        y = by_strat["avg_zone_viol"].values
        c = by_strat["avg_deadline_miss"].values
        labels = by_strat["strategy"].tolist()

        sc = plt.scatter(x, y, c=c, cmap="viridis", s=80, alpha=0.85)
        plt.colorbar(sc, label="Avg Event Deadline Misses")

        # Emphasize Pareto points (unfilled circle marker)
        for xi, yi, lbl, po in zip(x, y, labels, by_strat["pareto_optimal"].tolist()):
            if po:
                plt.scatter([xi], [yi], facecolors='none', edgecolors='black', s=180, linewidths=1.5)
            # label each point
            plt.annotate(lbl, (xi, yi), textcoords="offset points", xytext=(6, 6), fontsize=9)

        # Mark the chosen best (star)
        if best_strategy is not None:
            row = by_strat[by_strat["strategy"] == best_strategy].iloc[0]
            plt.scatter([row["avg_elapsed_s"]], [row["avg_zone_viol"]],
                        marker="*", s=220, edgecolors="black", facecolors="gold", linewidths=1.2,
                        label=f"Best (time-min Pareto): {best_strategy}")

        plt.xlabel("Avg Elapsed Time (s)")
        plt.ylabel("Avg Zone Violations")
        plt.title("Pareto Frontier: Time vs Zone Violations (color = Deadline Misses)")
        if best_strategy is not None:
            plt.legend(loc="upper left", frameon=False)
        plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
        plt.tight_layout()
        plt.savefig(os.path.join(self.out_dir, "pareto_scatter.png"), dpi=180)
        plt.close()

        # Console summary
        print("[ok] Wrote artifacts to:", self.out_dir)
        for name in [
            "successful_summaries.csv",
            "avg_zone_violations_vs_time.png",
            "avg_deadline_miss_vs_time.png",
            "avg_events_success_vs_time.png",
            "runs_scatter_zone_violations.png",
            "runs_scatter_deadline_miss.png",
            "runs_scatter_events_success.png",
            "box_time.png",
            "box_zone_violations.png",
            "box_deadline_miss.png",
            "box_events_success.png",
            "pareto_summary.csv",
            "pareto_scatter.png",
        ]:
            p = os.path.join(self.out_dir, name)
            if os.path.exists(p):
                print(" -", p)
        if best_strategy is not None:
            r = best.iloc[0]
            print(f"[best] Strategy minimizing time on Pareto frontier: {best_strategy} "
                  f"(avg_time={r['avg_elapsed_s']:.2f}s, "
                  f"avg_zone_viol={r['avg_zone_viol']:.2f}, "
                  f"avg_deadline_miss={r['avg_deadline_miss']:.2f})")
