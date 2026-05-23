#!/usr/bin/env python3
"""
visualize.py  -  Visualize simulation results from simulate_all

Input:  sim_<name>.csv files produced by the simulate_all binary
        (columns: t, y, error, u)
        sim_summary.csv  (per-controller metrics)

Output: One PNG per controller + a combined comparison figure.
        Figures saved to ./figures/ (created if absent).

Usage:
    python visualize.py [--dir <results_dir>] [--out <output_dir>]

The script searches for all sim_*.csv files in <results_dir>
(default: current directory) and generates:
  figures/sim_<name>.png        -- 3-panel plot (y, error, u)
  figures/comparison.png        -- all controllers' step responses overlaid
  figures/metrics.png           -- bar chart of ISE, overshoot, settling time
"""

import os
import sys
import glob
import argparse
import csv
import math
import collections

# Graceful degradation if matplotlib not installed
try:
    import matplotlib
    matplotlib.use("Agg")                # non-interactive backend
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[visualize] WARNING: matplotlib not found. ASCII fallback active.")

# ---------------------------------------------------------------------------
# Friendly display names
# ---------------------------------------------------------------------------
DISPLAY_NAME = {
    "pid":     "PID (IMC)",
    "pid_zn":  "PID (ZN)",
    "lqr":     "LQR",
    "lqg":     "LQG",
    "mpc":     "MPC",
    "leadlag": "Lead-Lag",
    "smc":     "SMC",
    "adrc":    "ADRC",
    "esc":     "ESC",
    "smith":   "Smith Predictor",
    "stack":   "Stack (SMC->PID)",
}

# Color cycle for comparison plot
COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
    "#bcbd22", "#17becf", "#aec7e8",
]

# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_csv(path):
    """Return dict of lists: {col: [values]}."""
    data = collections.defaultdict(list)
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                data[k].append(float(v))
    return dict(data)


def load_summary(path):
    """Return list of dicts from sim_summary.csv."""
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: (float(v) if k != "controller" else v)
                         for k, v in row.items()})
    return rows


# ---------------------------------------------------------------------------
# ASCII fallback
# ---------------------------------------------------------------------------
def ascii_plot(name, data, out_dir):
    """Print a simple ASCII representation of the step response."""
    t = data["t"]
    y = data["y"]
    width = 60
    height = 20
    y_min, y_max = min(y), max(y)
    if abs(y_max - y_min) < 1e-12:
        y_min -= 0.5; y_max += 0.5
    print(f"\n{'='*width}")
    print(f"  {name}  |  y_final={y[-1]:.4f}  t=[0,{t[-1]:.1f}s]")
    print(f"{'='*width}")
    print(f"  {'y_max':>6} = {y_max:.4f}")
    rows = []
    for row in range(height):
        threshold = y_max - (y_max - y_min) * row / (height - 1)
        line = []
        stride = max(1, len(t) // width)
        for ki in range(0, len(t), stride):
            line.append("*" if y[ki] >= threshold else " ")
        rows.append("".join(line[:width]))
    for row in rows:
        print("  |" + row + "|")
    print(f"  {'y_min':>6} = {y_min:.4f}")
    print(f"  t: 0 {'':>{width-8}} {t[-1]:.1f}s")
    # Save text report
    txt_path = os.path.join(out_dir, f"sim_{name}.txt")
    with open(txt_path, "w") as f:
        f.write(f"# {name} simulation\n# t,y,error,u\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.4f},{data['y'][i]:.6f},"
                    f"{data['error'][i]:.6f},{data['u'][i]:.6f}\n")
    print(f"  Text data written: {txt_path}")


# ---------------------------------------------------------------------------
# Matplotlib single-controller plot
# ---------------------------------------------------------------------------
def plot_single(name, data, out_dir, disturbance_t=7.5):
    """3-panel figure: output, error, control input."""
    t   = data["t"]
    y   = data["y"]
    err = data["error"]
    u   = data["u"]
    dn  = DISPLAY_NAME.get(name, name)

    fig = plt.figure(figsize=(10, 8))
    gs  = gridspec.GridSpec(3, 1, hspace=0.45)

    # --- Panel 1: output ----
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, y, color="#1f77b4", lw=1.5, label="y(t)")
    ax1.axhline(1.0, color="gray", lw=0.8, ls="--", label="Reference")
    ax1.axvline(disturbance_t, color="red", lw=0.7, ls=":", alpha=0.6,
                label=f"Disturbance +0.2 @ t={disturbance_t}s")
    ax1.axhspan(0.98, 1.02, alpha=0.08, color="green", label="+/-2% band")
    ax1.set_ylabel("Output  y(t)")
    ax1.set_title(f"Closed-loop response - {dn}")
    ax1.legend(fontsize=7, loc="lower right")
    ax1.grid(True, ls="--", alpha=0.4)

    # --- Panel 2: error ----
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, err, color="#d62728", lw=1.2, label="error = r - y")
    ax2.axhline(0, color="gray", lw=0.5)
    ax2.axvline(disturbance_t, color="red", lw=0.7, ls=":", alpha=0.6)
    ax2.set_ylabel("Error  e(t)")
    ax2.legend(fontsize=7)
    ax2.grid(True, ls="--", alpha=0.4)

    # --- Panel 3: control effort ----
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, u, color="#2ca02c", lw=1.2, label="u(t)")
    ax3.axvline(disturbance_t, color="red", lw=0.7, ls=":", alpha=0.6)
    ax3.set_xlabel("Time  t (s)")
    ax3.set_ylabel("Control  u(t)")
    ax3.legend(fontsize=7)
    ax3.grid(True, ls="--", alpha=0.4)

    # ISE annotation
    ise = sum(e**2 * (t[1] - t[0]) for e in err)
    y_arr = y
    overshoot = max(0.0, (max(y_arr) - 1.0) / 1.0 * 100.0)
    fig.text(0.5, 0.01,
             f"ISE={ise:.4f}   Overshoot={overshoot:.1f}%   y_final={y[-1]:.4f}",
             ha="center", fontsize=9, color="#555555")

    out_path = os.path.join(out_dir, f"sim_{name}.png")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ---------------------------------------------------------------------------
# Comparison plot
# ---------------------------------------------------------------------------
def plot_comparison(datasets, out_dir, disturbance_t=7.5):
    """All step responses on one axis for visual comparison."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax_y = axes[0]
    ax_e = axes[1]

    for i, (name, data) in enumerate(datasets.items()):
        col = COLORS[i % len(COLORS)]
        dn  = DISPLAY_NAME.get(name, name)
        ax_y.plot(data["t"], data["y"],   color=col, lw=1.2, label=dn)
        ax_e.plot(data["t"], data["error"], color=col, lw=1.0, label=dn)

    for ax in axes:
        ax.axvline(disturbance_t, color="black", lw=0.6, ls=":", alpha=0.5)
        ax.grid(True, ls="--", alpha=0.3)
        ax.legend(fontsize=7, loc="lower right")

    ax_y.axhline(1.0, color="gray", lw=0.6, ls="--")
    ax_y.axhspan(0.98, 1.02, alpha=0.06, color="green")
    ax_y.set_xlabel("Time  t (s)"); ax_y.set_ylabel("Output  y(t)")
    ax_y.set_title("Step response comparison - all controllers")

    ax_e.axhline(0, color="gray", lw=0.5)
    ax_e.set_xlabel("Time  t (s)"); ax_e.set_ylabel("Error  e(t)")
    ax_e.set_title("Tracking error comparison")

    out_path = os.path.join(out_dir, "comparison.png")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ---------------------------------------------------------------------------
# Metrics bar chart
# ---------------------------------------------------------------------------
def plot_metrics(summary, out_dir):
    if not summary:
        return

    names    = [r["controller"] for r in summary]
    dn_names = [DISPLAY_NAME.get(n, n) for n in names]
    ise_vals  = [r["ISE"]           for r in summary]
    os_vals   = [r["overshoot_pct"] for r in summary]
    ts_vals   = [r["t_settle_s"]    for r in summary]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Controller performance metrics", fontsize=13)

    def bar_panel(ax, vals, title, ylabel, color):
        bars = ax.bar(range(len(names)), vals, color=color, alpha=0.8)
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels(dn_names, rotation=35, ha="right", fontsize=8)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", ls="--", alpha=0.4)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                    f"{v:.3f}", ha="center", va="bottom", fontsize=7)

    bar_panel(axes[0], ise_vals,  "Integral Squared Error (ISE)", "ISE",      "#1f77b4")
    bar_panel(axes[1], os_vals,   "Overshoot (%)",                "% above ref","#d62728")
    bar_panel(axes[2], ts_vals,   "2% Settling time (s)",         "t_settle (s)","#2ca02c")

    plt.tight_layout()
    out_path = os.path.join(out_dir, "metrics.png")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ---------------------------------------------------------------------------
# Phase-plane / state portrait (ESC convergence)
# ---------------------------------------------------------------------------
def plot_esc_convergence(data, out_dir):
    """Special plot for ESC: theta vs cost J."""
    if not HAS_MPL:
        return
    t     = data["t"]
    theta = data["y"]
    err   = data["error"]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    axes[0].plot(t, theta, lw=1.2, color="#9467bd", label="θ (estimate)")
    axes[0].axhline(1.0, ls="--", color="gray", lw=0.8, label="Optimum (θ*=1)")
    axes[0].set_xlabel("Time (s)"); axes[0].set_ylabel("θ")
    axes[0].set_title("ESC - operating point convergence")
    axes[0].legend(fontsize=8); axes[0].grid(True, ls="--", alpha=0.3)

    cost = [e**2 for e in err]
    axes[1].plot(t, cost, lw=1.0, color="#ff7f0e", label="J = (θ-1)^2")
    axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("Cost J")
    axes[1].set_title("ESC - cost convergence")
    axes[1].legend(fontsize=8); axes[1].grid(True, ls="--", alpha=0.3)
    axes[1].set_yscale("log")

    plt.tight_layout()
    out_path = os.path.join(out_dir, "sim_esc_convergence.png")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Visualize controller simulation results.")
    parser.add_argument("--dir", default=".", help="Directory containing sim_*.csv files")
    parser.add_argument("--out", default="figures", help="Output directory for figures")
    args = parser.parse_args()

    results_dir = os.path.abspath(args.dir)
    out_dir     = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    # Find all sim_*.csv files
    pattern  = os.path.join(results_dir, "sim_*.csv")
    csv_files = sorted(glob.glob(pattern))

    if not csv_files:
        print(f"[visualize] No sim_*.csv files found in {results_dir}")
        print("  Run the simulate_all binary first to generate CSV files.")
        sys.exit(1)

    print(f"[visualize] Found {len(csv_files)} simulation file(s) in {results_dir}")
    print(f"[visualize] Output directory: {out_dir}")
    print(f"[visualize] matplotlib: {'available' if HAS_MPL else 'NOT found (ASCII fallback)'}\n")

    datasets = {}
    for path in csv_files:
        base = os.path.basename(path)          # sim_pid.csv
        name = base[4:-4]                       # pid
        if name == "summary":
            continue
        print(f"Loading: {base}")
        data = load_csv(path)
        datasets[name] = data

        if HAS_MPL:
            plot_single(name, data, out_dir)
        else:
            ascii_plot(name, data, out_dir)

        # Special ESC convergence plot
        if name == "esc" and HAS_MPL:
            plot_esc_convergence(data, out_dir)

    if not datasets:
        print("[visualize] No valid datasets loaded.")
        sys.exit(1)

    if HAS_MPL:
        print("\nGenerating comparison figure...")
        plot_comparison(datasets, out_dir)

        summary = load_summary(os.path.join(results_dir, "sim_summary.csv"))
        if summary:
            print("Generating metrics bar chart...")
            plot_metrics(summary, out_dir)
        else:
            print("[visualize] sim_summary.csv not found; skipping metrics chart.")
    else:
        # ASCII summary table
        print("\n=== Summary ===")
        print(f"{'Controller':<16} {'y_final':>10} {'ISE':>10}")
        for name, data in datasets.items():
            ise = sum(e**2 * (data['t'][1]-data['t'][0]) for e in data['error'])
            print(f"{name:<16} {data['y'][-1]:>10.4f} {ise:>10.4f}")

    print(f"\n[visualize] Done. Outputs in: {out_dir}")


if __name__ == "__main__":
    main()
