"""
tools/mc_plots.py  -- ANA-2: visualisations for Monte Carlo mc_summary CSV.

Usage:
    python tools/mc_plots.py mc_summary_DrillString.csv [options]

Options:
    --metric  NAME     metric column to plot (default: iae)
    --out     DIR      output directory for PNG files (default: same dir as CSV)
    --show             show interactive plots (requires display)
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
    import matplotlib
    matplotlib.use("Agg")        # headless by default; --show overrides below
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: numpy, pandas, matplotlib required.")
    sys.exit(1)


def _violin_plot(df: pd.DataFrame, metric: str, out_dir: Path, study: str):
    controllers = sorted(df["controller"].unique())
    data_per_ctrl = [df[df["controller"] == c][metric].dropna().values for c in controllers]

    fig, ax = plt.subplots(figsize=(max(8, len(controllers) * 0.7), 5))
    parts = ax.violinplot(data_per_ctrl, showmedians=True, showextrema=True)
    ax.set_xticks(range(1, len(controllers) + 1))
    ax.set_xticklabels(controllers, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel(metric)
    ax.set_title(f"{study} - MC distribution of {metric} (n={len(df)//len(controllers)} per ctrl)")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out_path = out_dir / f"mc_violin_{metric}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


def _scatter_plot(df: pd.DataFrame, metric: str, param_col: str, out_dir: Path, study: str):
    fig, ax = plt.subplots(figsize=(7, 5))
    controllers = sorted(df["controller"].unique())
    cmap = plt.get_cmap("tab10")
    for i, ctrl in enumerate(controllers):
        sub = df[df["controller"] == ctrl]
        ax.scatter(sub[param_col], sub[metric], label=ctrl,
                   color=cmap(i % 10), alpha=0.6, s=20)
    ax.set_xlabel(param_col)
    ax.set_ylabel(metric)
    ax.set_title(f"{study} - {metric} vs {param_col}")
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    param_short = param_col.replace("param_", "")[:20]
    out_path = out_dir / f"mc_scatter_{metric}_vs_{param_short}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="mc_summary CSV file")
    ap.add_argument("--metric", default="iae")
    ap.add_argument("--out",    default="")
    ap.add_argument("--show",   action="store_true")
    args = ap.parse_args(argv)

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: {csv_path} not found")
        sys.exit(1)

    if args.show:
        matplotlib.use("TkAgg")

    df = pd.read_csv(csv_path)
    if args.metric not in df.columns:
        print(f"ERROR: column '{args.metric}' not found in CSV.  Available: {list(df.columns)}")
        sys.exit(1)

    out_dir = Path(args.out) if args.out else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    study = df["study"].iloc[0] if "study" in df.columns else csv_path.stem

    print(f"Plotting MC results for {study} ({len(df)} rows)")

    # Violin plot
    _violin_plot(df, args.metric, out_dir, study)

    # Scatter plots for each perturbed parameter column
    param_cols = [c for c in df.columns if c.startswith("param_")]
    for pcol in param_cols[:4]:    # limit to 4 scatter plots
        _scatter_plot(df, args.metric, pcol, out_dir, study)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
