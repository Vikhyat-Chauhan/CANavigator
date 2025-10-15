# hydra_teleop/plugins/statics_analyser.py
#!/usr/bin/env python3
"""
Stats analyzer for Hydra experiment CSVs (import-only).
Now supports 2–4 strategies (APEs).

Usage from your main:
    from hydra_teleop.plugins.statics_analyser import StatsAnalyzer
    from hydra_teleop.config import TeleopConfig
    cfg = TeleopConfig()
    # Optional: cfg.analyzer_strategies = ["NAVALGO1", "NAVALGO2", "APE3", "APE4"]
    StatsAnalyzer.from_config(cfg).analyze()

Reads settings from TeleopConfig:
- results_csv_path
- analyzer_out_dir
- analyzer_pair_order              (DEPRECATED: ignored if analyzer_strategies provided)
- analyzer_annotate_points
- analyzer_include_timeouts
- analyzer_strategies              (NEW: list of 2–4 strategy names to include)
"""

from __future__ import annotations
import os
import math
import random
from typing import Dict, List, Tuple, Optional, Sequence, Set

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
    "run": ["run", "seed", "trial"],
    "strategy": ["strategy", "algo", "algorithm"],
    "reached": ["reached", "success", "done"],
    "elapsed_s": ["elapsed_s", "elapsed", "time_s", "time", "duration_s"],
    "violations": ["violations", "viols", "violation_count", "errors"],
}

def _resolve_columns(df: pd.DataFrame) -> Dict[str, str]:
    lower = {c.lower(): c for c in df.columns}
    out = {}
    for canonical, options in ALIASES.items():
        for opt in options:
            if opt.lower() in lower:
                out[canonical] = lower[opt.lower()]
                break
        if canonical not in out:
            raise KeyError(
                f"Missing required column like: '{canonical}' (aliases: {options}). "
                f"Found columns: {list(df.columns)}"
            )
    return out

def _to_bool_series(s: pd.Series) -> pd.Series:
    return s.astype(str).str.strip().str.lower().isin(["true", "1", "yes", "y", "t"])

def _ensure_outdir(path: str) -> None:
    os.makedirs(path, exist_ok=True)

def _aggregate_by_strategy(stats: List[dict]) -> List[dict]:
    agg = {}
    for row in stats:
        s = row["strategy"]
        d = agg.setdefault(s, {"n": 0, "sum_elapsed": 0.0, "sum_viol": 0.0, "sum_reached": 0})
        if not math.isnan(row["elapsed_s"]):
            d["sum_elapsed"] += row["elapsed_s"]
        d["sum_viol"] += row["violations"]
        d["sum_reached"] += int(row["reached"])
        d["n"] += 1

    rows = []
    for s, d in agg.items():
        n = max(d["n"], 1)
        rows.append({
            "strategy": s,
            "n_runs": d["n"],
            "avg_elapsed_s": d["sum_elapsed"] / n,
            "avg_violations": d["sum_viol"] / n,
            "reach_rate": d["sum_reached"] / n,
        })
    rows.sort(key=lambda r: r["strategy"])
    return rows

def _plot_avg_violations_vs_time(agg_rows: List[dict], out_path: str) -> None:
    x = [r["avg_elapsed_s"] for r in agg_rows]
    y = [r["avg_violations"] for r in agg_rows]
    labels = [r["strategy"] for r in agg_rows]

    plt.figure(figsize=(6, 4))
    plt.scatter(x, y)
    for xi, yi, lbl in zip(x, y, labels):
        plt.annotate(lbl, (xi, yi), textcoords="offset points", xytext=(6, 6))
    plt.xlabel("Average elapsed time (s)")
    plt.ylabel("Average violations (count)")
    plt.title("Average Violations vs Average Time by Strategy")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()

def _plot_per_run_scatter(
    stats: List[dict],
    out_path: str,
    annotate: bool,
    include_incomplete_or_timeouts: bool,
    clean_runs: Set[int],
) -> None:
    """
    Per-run scatter: elapsed time vs violations.
    - Color encodes strategy.
    - Marker encodes run completeness across selected strategies:
        'o'  -> clean run (all selected strategies reached=True for that run)
        'x'  -> mixed/timeout (at least one missing or not reached)
    - If include_incomplete_or_timeouts=False, only clean runs are plotted.
    """
    random.seed(42)

    # Build per-strategy buckets with jitter on y to reduce overplot
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
            y = r["violations"] + random.uniform(-0.06, 0.06)  # tiny jitter
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
                txt = f"r{r['run']} · {r['strategy']} · {r['elapsed_s']:.1f}s · v{r['violations']}"
                plt.annotate(txt, (x, y), textcoords="offset points", xytext=(6, 5),
                             fontsize=7, alpha=0.9)
            for x, y, r in zip(xs_clean, ys_clean, pts_clean):
                _label_point(x, y, r)
            if include_incomplete_or_timeouts:
                for x, y, r in zip(xs_mixed, ys_mixed, pts_mixed):
                    _label_point(x, y, r)

    plt.xlabel("Elapsed time per run (s)")
    plt.ylabel("Violations per run (count)")
    ttl = "Per-Run Time vs Violations"
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
        # Prefer explicit list if provided
        chosen: Optional[Sequence[str]] = getattr(cfg, "analyzer_strategies", None)
        # Backward-compat: fall back to pair_order if present (but allow 2 only)
        if not chosen and getattr(cfg, "analyzer_pair_order", None):
            chosen = list(cfg.analyzer_pair_order)

        return cls(
            input_csv=cfg.results_csv_path,
            out_dir=cfg.analyzer_out_dir,
            strategies=chosen or [],  # will be resolved after reading CSV
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
        # Auto-pick up to 4
        if len(present) < 2:
            raise SystemExit(f"Need at least 2 strategies; found: {present}")
        return present[:4]

    def analyze(self) -> None:
        assert os.path.exists(self.input_csv), f"Input not found: {self.input_csv}"
        _ensure_outdir(self.out_dir)

        # Load & normalize
        df_raw = pd.read_csv(self.input_csv)
        cols = _resolve_columns(df_raw)

        df = df_raw[[cols["run"], cols["strategy"], cols["reached"], cols["elapsed_s"], cols["violations"]]].copy()
        df.columns = ["run", "strategy", "reached", "elapsed_s", "violations"]
        df["reached"] = _to_bool_series(df["reached"])

        # Decide which strategies to include (2–4)
        selected = self._resolve_strategies(df)
        df = df[df["strategy"].isin(selected)].copy()

        # Build completeness map per run:
        # A run is "complete" if it contains ALL selected strategies (present),
        # and "clean" if complete AND all reached=True.
        ct_per_run = (df.groupby(["run"])["strategy"]
                        .nunique()
                        .rename("n_strats_in_run"))
        reached_per_run = (df.groupby(["run"])["reached"]
                             .agg(all_reached=lambda x: bool(np.all(x))))
        run_meta = pd.concat([ct_per_run, reached_per_run], axis=1).reset_index()

        complete_runs = set(run_meta[run_meta["n_strats_in_run"] == len(selected)]["run"].astype(int).tolist())
        clean_runs = set(run_meta[
            (run_meta["n_strats_in_run"] == len(selected)) &
            (run_meta["all_reached"])
        ]["run"].astype(int).tolist())

        # Optionally filter to clean runs only (strict apples-to-apples)
        if self.exclude_incomplete_or_timeouts:
            df_filt = df[df["run"].isin(clean_runs)].copy()
        else:
            # Keep all rows from runs that have at least one selected strategy;
            # the scatter will mark mixed/timeout runs with 'x'.
            df_filt = df.copy()

        # Rebuild stats list after filtering
        stats = [{
            "run": int(r.run),
            "strategy": str(r.strategy),
            "reached": bool(r.reached),
            "elapsed_s": float(r.elapsed_s) if pd.notna(r.elapsed_s) else float("nan"),
            "violations": int(r.violations),
        } for r in df_filt.itertuples(index=False)]

        # Summaries on successful only
        succ_df = df_filt[df_filt["reached"]].copy()

        def iqr(x):
            q75, q25 = np.percentile(x, [75, 25])
            return float(q75 - q25)

        if len(succ_df) > 0:
            summ = (succ_df.groupby("strategy")
                    .agg(n=("elapsed_s", "size"),
                         mean_time_s=("elapsed_s", "mean"),
                         median_time_s=("elapsed_s", "median"),
                         p90_time_s=("elapsed_s", lambda x: np.percentile(x, 90)),
                         iqr_time_s=("elapsed_s", iqr),
                         mean_viol=("violations", "mean"),
                         median_viol=("violations", "median"),
                         p90_viol=("violations", lambda x: np.percentile(x, 90)))
                    .reset_index()
                    .sort_values("strategy"))
        else:
            # empty summary with expected columns
            summ = pd.DataFrame(columns=[
                "strategy","n","mean_time_s","median_time_s","p90_time_s","iqr_time_s",
                "mean_viol","median_viol","p90_viol"
            ])

        summ_out = os.path.join(self.out_dir, "successful_summaries.csv")
        summ.round(4).to_csv(summ_out, index=False)

        # Plots
        _plot_per_run_scatter(
            stats,
            os.path.join(self.out_dir, "runs_scatter.png"),
            annotate=self.annotate_points,
            include_incomplete_or_timeouts=not self.exclude_incomplete_or_timeouts,
            clean_runs=clean_runs,
        )
        _plot_avg_violations_vs_time(
            _aggregate_by_strategy(stats),
            os.path.join(self.out_dir, "avg_violations_vs_time.png")
        )

        # Box: time (successful)
        if len(succ_df) > 0:
            plt.figure(figsize=(12, 6))
            strategies = list(sorted(succ_df["strategy"].unique()))
            groups = [succ_df[succ_df["strategy"] == s]["elapsed_s"].values for s in strategies]
            plt.boxplot(groups, labels=strategies, showmeans=True)
            plt.title("Elapsed time by strategy (successful runs)")
            plt.ylabel("Seconds")
            plt.tight_layout()
            plt.savefig(os.path.join(self.out_dir, "box_time.png"), dpi=180)
            plt.close()

            # Box: violations (successful)
            plt.figure(figsize=(12, 6))
            groups_v = [succ_df[succ_df["strategy"] == s]["violations"].values for s in strategies]
            plt.boxplot(groups_v, labels=strategies, showmeans=True)
            plt.title("Violations by strategy (successful runs)")
            plt.ylabel("Count")
            plt.tight_layout()
            plt.savefig(os.path.join(self.out_dir, "box_violations.png"), dpi=180)
            plt.close()

        print("[ok] Wrote artifacts to:", self.out_dir)
        for name in [
            "successful_summaries.csv",
            "avg_violations_vs_time.png",
            "runs_scatter.png",
            "box_time.png",
            "box_violations.png",
        ]:
            p = os.path.join(self.out_dir, name)
            if os.path.exists(p):
                print(" -", p)
