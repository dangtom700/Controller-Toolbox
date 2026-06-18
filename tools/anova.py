"""
tools/anova.py  -- ANA-4: one-way ANOVA + Tukey HSD for controller comparison.

Reads a mc_summary CSV (produced by monte_carlo.py or any CSV with a 'controller' column
and one or more numeric metric columns) and performs:
  1. One-way ANOVA test (F-statistic, p-value) for each metric
  2. Tukey HSD post-hoc pairwise comparison (if scipy available)

Usage:
    python tools/anova.py mc_summary.csv [options]

Options:
    --metric  NAME   metric column to analyse (default: iae)
    --alpha   FLOAT  significance level (default 0.05)
    --out     FILE   write Tukey table to CSV (optional)
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
    from scipy import stats
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False
    print("WARN: scipy not found - Tukey HSD will be skipped.  Install with: pip install scipy")


def _one_way_anova(groups: list[np.ndarray], alpha: float) -> tuple[float, float]:
    """Return (F, p) for one-way ANOVA across groups."""
    if not _HAS_SCIPY:
        return float("nan"), float("nan")
    F, p = stats.f_oneway(*groups)
    return float(F), float(p)


def _tukey_hsd(df: pd.DataFrame, group_col: str, value_col: str, alpha: float) -> pd.DataFrame | None:
    """Compute pairwise Tukey HSD comparisons.  Returns None if scipy missing."""
    if not _HAS_SCIPY:
        return None
    try:
        from statsmodels.stats.multicomp import pairwise_tukeyhsd
        result = pairwise_tukeyhsd(df[value_col].dropna(), df[group_col], alpha=alpha)
        summary_df = pd.DataFrame(
            data=result.summary().data[1:],
            columns=result.summary().data[0],
        )
        return summary_df
    except ImportError:
        print("WARN: statsmodels not found - Tukey HSD unavailable.  pip install statsmodels")
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv",   help="mc_summary or compare_controllers CSV file")
    ap.add_argument("--metric", default="iae")
    ap.add_argument("--alpha",  type=float, default=0.05)
    ap.add_argument("--out",    default="")
    args = ap.parse_args(argv)

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: {csv_path} not found")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    if "controller" not in df.columns:
        print("ERROR: CSV must have a 'controller' column")
        sys.exit(1)
    if args.metric not in df.columns:
        print(f"ERROR: metric '{args.metric}' not found.  Available: {list(df.columns)}")
        sys.exit(1)

    df = df.dropna(subset=[args.metric])
    controllers = sorted(df["controller"].unique())
    n_ctrl = len(controllers)
    print(f"\nANOVA analysis  |  metric: {args.metric}  |  alpha: {args.alpha}")
    print(f"Controllers: {n_ctrl}  Total observations: {len(df)}\n")

    # Per-controller summary
    summary = df.groupby("controller")[args.metric].agg(["count", "mean", "std", "min", "max"])
    summary.columns = ["n", "mean", "std", "min", "max"]
    summary = summary.sort_values("mean")
    print("--- Per-controller summary ---")
    print(summary.to_string(float_format="{:.4f}".format))

    # One-way ANOVA
    groups = [df[df["controller"] == c][args.metric].dropna().values for c in controllers]
    F, p = _one_way_anova([g for g in groups if len(g) > 0], args.alpha)
    print(f"\n--- One-way ANOVA ---")
    print(f"F = {F:.4f}   p = {p:.6f}   significant: {'YES' if p < args.alpha else 'NO'} (alpha={args.alpha})")

    if p < args.alpha:
        print(f"  -> Significant differences exist between controllers (reject H0: all means equal)")
    else:
        print(f"  -> No significant difference detected at alpha={args.alpha}")

    # Tukey HSD
    tukey_df = _tukey_hsd(df, "controller", args.metric, args.alpha)
    if tukey_df is not None:
        print("\n--- Tukey HSD pairwise comparisons ---")
        sig = tukey_df[tukey_df["reject"] == True] if "reject" in tukey_df.columns else tukey_df
        print(sig.to_string(index=False) if not sig.empty else "No significant pairs found.")
        if args.out:
            tukey_df.to_csv(args.out, index=False)
            print(f"\nTukey table written to: {args.out}")

    return {"F": F, "p": p, "n_controllers": n_ctrl, "n_obs": len(df)}


if __name__ == "__main__":
    main()
