"""
tools/mu_plots.py  -- ANA-7: visualisations for mu_summary.csv.

Usage:
    python tools/mu_plots.py mu_summary.csv [options]

Options:
    --out   DIR   output directory (default: same dir as CSV)
    --show        show interactive plots
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
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: numpy, pandas, matplotlib required.")
    sys.exit(1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv",  help="mu_summary.csv from mu_analysis.py")
    ap.add_argument("--out",  default="")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args(argv)

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: {csv_path} not found")
        sys.exit(1)

    if args.show:
        matplotlib.use("TkAgg")

    df = pd.read_csv(csv_path).dropna(subset=["peak_T"])
    out_dir = Path(args.out) if args.out else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # Bar chart: peak T per study
    by_study = df.groupby("study")["peak_T"].mean().sort_values(ascending=False)
    fig, ax = plt.subplots(figsize=(max(6, len(by_study) * 0.8), 5))
    colors = ["red" if v > 1.0 else "steelblue" for v in by_study.values]
    ax.bar(range(len(by_study)), by_study.values, color=colors)
    ax.axhline(1.0, color="black", linestyle="--", linewidth=1)
    ax.set_xticks(range(len(by_study)))
    ax.set_xticklabels(by_study.index, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("Peak |T(z)| (mu upper bound)")
    ax.set_title("Unstructured mu upper bound per study (averaged over scenarios)")
    fig.tight_layout()
    out_path = out_dir / "mu_bar_by_study.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved: {out_path}")

    # Scatter: peak_S vs peak_T
    fig, ax = plt.subplots(figsize=(7, 5))
    studies = sorted(df["study"].unique())
    cmap = plt.get_cmap("tab10")
    for i, s in enumerate(studies):
        sub = df[df["study"] == s]
        ax.scatter(sub["peak_S"], sub["peak_T"], label=s[:30],
                   color=cmap(i % 10), s=40, alpha=0.8)
    ax.axvline(1.0, color="gray", linestyle=":", linewidth=1)
    ax.axhline(1.0, color="gray", linestyle=":", linewidth=1)
    ax.set_xlabel("Peak |S(z)|")
    ax.set_ylabel("Peak |T(z)|")
    ax.set_title("Sensitivity vs Complementary Sensitivity")
    ax.legend(fontsize=7, ncol=2, bbox_to_anchor=(1.01, 1), loc="upper left")
    fig.tight_layout()
    out_path = out_dir / "mu_scatter_S_vs_T.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
