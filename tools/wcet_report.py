"""
tools/wcet_report.py  -- ANA-5: aggregate WCET (Worst-Case Execution Time) profiling data.

Discovers wcet_*.csv files under the repo and aggregates timing statistics per controller.

Expected CSV format (produced by timing instrumentation in sim/main.py):
    controller, step_time_us, step_index

Usage:
    python tools/wcet_report.py [options]

Options:
    --glob  PATTERN   glob for WCET CSV files (default: case-study/**/wcet_*.csv)
    --out   FILE      output summary CSV (default: wcet_summary.csv)
    --alpha FLOAT     tail probability for WCET estimate (default: 0.999)
    --plot            emit a bar chart PNG of WCET estimates

If no wcet_*.csv files exist, the script prints guidance on how to add timing
instrumentation to a sim/main.py.
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))

try:
    import numpy as np
    import pandas as pd
except ImportError:
    print("ERROR: numpy and pandas required.")
    sys.exit(1)

_INSTRUMENTATION_HINT = """
--- How to add WCET instrumentation to a sim/main.py ---

Add the following snippet around the per-step controller call:

    import time

    wcet_rows = []
    for k, t in enumerate(t_arr):
        t0 = time.perf_counter()
        u = ctrl.compute(error)
        dt_us = (time.perf_counter() - t0) * 1e6
        wcet_rows.append({"controller": ctrl_name, "step_time_us": dt_us, "step_index": k})

    # After all controllers:
    pd.DataFrame(wcet_rows).to_csv(logs_dir / f"wcet_{scenario}.csv", index=False)
---
"""


def _collect_csvs(glob_pattern: str) -> list[Path]:
    parts = glob_pattern.split("**/", 1)
    if len(parts) == 2:
        base = _ROOT / parts[0]
        tail = parts[1]
        return sorted(base.rglob(tail))
    return sorted(_ROOT.glob(glob_pattern))


def _compute_wcet(series: np.ndarray, alpha: float) -> float:
    """WCET estimate at quantile alpha (e.g. 0.999 = 99.9th percentile)."""
    return float(np.quantile(series, alpha))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--glob",  default="case-study/**/wcet_*.csv")
    ap.add_argument("--out",   default="wcet_summary.csv")
    ap.add_argument("--alpha", type=float, default=0.999)
    ap.add_argument("--plot",  action="store_true")
    args = ap.parse_args(argv)

    csvs = _collect_csvs(args.glob)
    if not csvs:
        print(f"No WCET CSV files found matching: {args.glob}")
        print(_INSTRUMENTATION_HINT)
        sys.exit(0)

    print(f"Found {len(csvs)} WCET CSV file(s)")
    frames = []
    for p in csvs:
        try:
            df = pd.read_csv(p)
            df["_source"] = p.name
            frames.append(df)
        except Exception as exc:
            print(f"WARN: skipping {p.name}: {exc}")

    if not frames:
        print("No valid data loaded.")
        sys.exit(1)

    all_df = pd.concat(frames, ignore_index=True)
    required = {"controller", "step_time_us"}
    missing  = required - set(all_df.columns)
    if missing:
        print(f"ERROR: CSV columns missing: {missing}")
        print(_INSTRUMENTATION_HINT)
        sys.exit(1)

    summary_rows = []
    for ctrl, grp in all_df.groupby("controller"):
        t = grp["step_time_us"].dropna().values
        if len(t) == 0:
            continue
        row = {
            "controller":   ctrl,
            "n_samples":    len(t),
            "mean_us":      float(np.mean(t)),
            "median_us":    float(np.median(t)),
            "p99_us":       float(np.quantile(t, 0.99)),
            "wcet_us":      _compute_wcet(t, args.alpha),
            "max_us":       float(np.max(t)),
        }
        summary_rows.append(row)

    if not summary_rows:
        print("No data after grouping.")
        sys.exit(1)

    summary_df = pd.DataFrame(summary_rows).sort_values("wcet_us", ascending=False)
    summary_df.to_csv(args.out, index=False)
    print(f"\nWCET summary written to: {args.out}")
    print(summary_df.to_string(index=False, float_format="{:.2f}".format))

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(max(6, len(summary_df) * 0.6), 5))
            ctrls = summary_df["controller"].tolist()
            ax.bar(range(len(ctrls)), summary_df["wcet_us"], color="steelblue", label="WCET")
            ax.bar(range(len(ctrls)), summary_df["mean_us"], color="orange", alpha=0.7, label="Mean")
            ax.set_xticks(range(len(ctrls)))
            ax.set_xticklabels(ctrls, rotation=45, ha="right", fontsize=8)
            ax.set_ylabel("Time [us]")
            ax.set_title(f"WCET report  (alpha={args.alpha})")
            ax.legend()
            ax.grid(axis="y", alpha=0.3)
            fig.tight_layout()
            plot_path = Path(args.out).with_suffix(".png")
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f"Plot saved: {plot_path}")
        except ImportError:
            print("WARN: matplotlib not found — skipping plot")


if __name__ == "__main__":
    main()
