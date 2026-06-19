"""
tools/wcet_report.py  -- ANA-5: aggregate WCET (Worst-Case Execution Time) profiling data.

Discovers wcet_*.csv files under each case-study's logs/ directory and writes one
aggregated wcet_summary.csv per study (case-study/<study>/wcet_summary.csv).

Expected CSV format (produced by timing instrumentation in sim/main.py):
    controller, step_time_us, step_index

Usage:
    python tools/wcet_report.py [options]

Options:
    --study NAME      restrict to one study (partial match on folder name; default: all)
    --glob  PATTERN   glob for WCET CSV files (default: case-study/**/wcet_*.csv)
    --out   FILE      override output path - only valid when exactly one study matches
                      (default: case-study/<study>/wcet_summary.csv per matching study)
    --alpha FLOAT     tail probability for WCET estimate (default: 0.999)
    --plot            emit a bar chart PNG of WCET estimates next to each summary CSV

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


def _study_dir_of(csv_path: Path) -> Path:
    """Return the case-study/<Study>/ directory that owns this CSV."""
    try:
        rel = csv_path.relative_to(_ROOT / "case-study")
    except ValueError:
        return csv_path.parent
    return _ROOT / "case-study" / rel.parts[0]


def _compute_wcet(series: np.ndarray, alpha: float) -> float:
    """WCET estimate at quantile alpha (e.g. 0.999 = 99.9th percentile)."""
    return float(np.quantile(series, alpha))


def _summarise(csvs: list[Path], alpha: float) -> "pd.DataFrame | None":
    frames = []
    for p in csvs:
        try:
            df = pd.read_csv(p)
            df["_source"] = p.name
            frames.append(df)
        except Exception as exc:
            print(f"WARN: skipping {p.name}: {exc}")
    if not frames:
        return None

    all_df = pd.concat(frames, ignore_index=True)
    required = {"controller", "step_time_us"}
    missing  = required - set(all_df.columns)
    if missing:
        print(f"ERROR: CSV columns missing: {missing}")
        return None

    summary_rows = []
    for ctrl, grp in all_df.groupby("controller"):
        t = grp["step_time_us"].dropna().values
        if len(t) == 0:
            continue
        summary_rows.append({
            "controller":   ctrl,
            "n_samples":    len(t),
            "mean_us":      float(np.mean(t)),
            "median_us":    float(np.median(t)),
            "p99_us":       float(np.quantile(t, 0.99)),
            "wcet_us":      _compute_wcet(t, alpha),
            "max_us":       float(np.max(t)),
        })
    if not summary_rows:
        return None
    return pd.DataFrame(summary_rows).sort_values("wcet_us", ascending=False)


def _plot(summary_df: pd.DataFrame, out_path: Path, alpha: float) -> None:
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
        ax.set_title(f"WCET report  (alpha={alpha})")
        ax.legend()
        ax.grid(axis="y", alpha=0.3)
        fig.tight_layout()
        plot_path = out_path.with_suffix(".png")
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        print(f"Plot saved: {plot_path}")
    except ImportError:
        print("WARN: matplotlib not found - skipping plot")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study", default="", help="restrict to one study (partial match)")
    ap.add_argument("--glob",  default="case-study/**/wcet_*.csv")
    ap.add_argument("--out",   default="", help="override path; only valid with exactly one matching study")
    ap.add_argument("--alpha", type=float, default=0.999)
    ap.add_argument("--plot",  action="store_true")
    args = ap.parse_args(argv)

    csvs = _collect_csvs(args.glob)
    if args.study:
        csvs = [p for p in csvs if args.study.lower() in _study_dir_of(p).name.lower()]
    if not csvs:
        suffix = f" (study filter: {args.study})" if args.study else ""
        print(f"No WCET CSV files found matching: {args.glob}{suffix}")
        print(_INSTRUMENTATION_HINT)
        sys.exit(0)

    print(f"Found {len(csvs)} WCET CSV file(s)")

    by_study: dict[Path, list[Path]] = {}
    for p in csvs:
        by_study.setdefault(_study_dir_of(p), []).append(p)

    if args.out and len(by_study) > 1:
        print(f"ERROR: --out given but {len(by_study)} studies matched; "
              f"omit --out or narrow with --study to use it.")
        sys.exit(1)

    wrote_any = False
    for study_dir, group_csvs in sorted(by_study.items()):
        summary_df = _summarise(group_csvs, args.alpha)
        if summary_df is None:
            print(f"[{study_dir.name}] No usable WCET data.")
            continue

        out_path = Path(args.out) if args.out else (study_dir / "wcet_summary.csv")
        summary_df.to_csv(out_path, index=False)
        wrote_any = True
        print(f"\n[{study_dir.name}]  WCET summary written to: {out_path}")
        print(summary_df.to_string(index=False, float_format="{:.2f}".format))

        if args.plot:
            _plot(summary_df, out_path, args.alpha)

    if not wrote_any:
        print("No data after grouping.")
        sys.exit(1)


if __name__ == "__main__":
    main()
