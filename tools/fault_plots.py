"""
tools/fault_plots.py  -- ANA-3: heatmap and degradation plots for fault sweep results.

Usage:
    python tools/fault_plots.py fault_sweep_DrillString.csv [options]

Options:
    --metric  NAME   metric to plot (default: iae)
    --out     DIR    output directory (default: same dir as CSV)
    --show           show interactive plots
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
    from matplotlib.colors import LogNorm
except ImportError:
    print("ERROR: numpy, pandas, matplotlib required.")
    sys.exit(1)


def _heatmap(df: pd.DataFrame, metric: str, fault_kind: str, out_dir: Path, study: str):
    sub = df[df["fault_kind"] == fault_kind].copy()
    if sub.empty:
        return

    pivot = sub.pivot_table(index="controller", columns="magnitude", values=metric, aggfunc="mean")
    if pivot.empty:
        return

    fig, ax = plt.subplots(figsize=(max(6, len(pivot.columns) * 1.2), max(4, len(pivot.index) * 0.5)))
    vals = pivot.values
    # Use log norm if values span > 2 orders of magnitude
    vmin, vmax = np.nanmin(vals), np.nanmax(vals)
    if vmax > 0 and vmax / max(vmin, 1e-9) > 100:
        norm = LogNorm(vmin=max(vmin, 1e-6), vmax=vmax)
    else:
        norm = None

    im = ax.imshow(vals, aspect="auto", norm=norm, cmap="RdYlGn_r")
    plt.colorbar(im, ax=ax, label=metric)
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels([f"{v:.2f}" for v in pivot.columns], fontsize=8)
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    ax.set_xlabel("Fault magnitude")
    ax.set_ylabel("Controller")
    ax.set_title(f"{study} - {metric} vs {fault_kind} magnitude")
    fig.tight_layout()
    out_path = out_dir / f"fault_heatmap_{fault_kind}_{metric}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


def _degradation_curve(df: pd.DataFrame, metric: str, fault_kind: str, out_dir: Path, study: str):
    sub = df[df["fault_kind"] == fault_kind].copy()
    if sub.empty:
        return

    fig, ax = plt.subplots(figsize=(8, 5))
    controllers = sorted(sub["controller"].unique())
    cmap = plt.get_cmap("tab10")
    for i, ctrl in enumerate(controllers):
        csub = sub[sub["controller"] == ctrl].sort_values("magnitude")
        ax.plot(csub["magnitude"], csub[metric], marker="o", label=ctrl,
                color=cmap(i % 10), linewidth=1.5, markersize=4)
    ax.set_xlabel(f"{fault_kind} magnitude")
    ax.set_ylabel(metric)
    ax.set_title(f"{study} - {metric} degradation under {fault_kind}")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out_path = out_dir / f"fault_degradation_{fault_kind}_{metric}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv",   help="fault_sweep CSV file")
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
        print(f"ERROR: column '{args.metric}' not in CSV")
        sys.exit(1)

    out_dir = Path(args.out) if args.out else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    study = df["study"].iloc[0] if "study" in df.columns else csv_path.stem

    print(f"Plotting fault sweep for {study}")
    for fk in df["fault_kind"].unique():
        _heatmap(df, args.metric, fk, out_dir, study)
        _degradation_curve(df, args.metric, fk, out_dir, study)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
