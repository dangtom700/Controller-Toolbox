"""
tools/compare_controllers.py  -- ANA-1: performance comparison table across all case studies.

Auto-discovers case-study/*/logs/run_*.csv, extracts metrics, and prints a ranked table.

Usage:
    python tools/compare_controllers.py [options]

Options:
    --study    NAME      filter to a specific study (partial match, case-insensitive)
    --scenario NAME      filter to a specific scenario (partial match)
    --metric   NAME      primary sort column: iae (default), rms_error, settle_time_s,
                         overshoot_pct, max_u, energy_var
    --sort     asc|desc  sort direction (default asc for error metrics, desc for --metric none)
    --wide               show all metrics in the table (default: only IAE + chosen metric)
    --csv      FILE      also write results to FILE as CSV
"""
from __future__ import annotations
import argparse
import os
import sys
import csv
import re
import math
from pathlib import Path

# Allow running from repo root without installing
_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))

try:
    import pandas as pd
    import numpy as np
except ImportError:
    print("ERROR: pandas and numpy are required.  Run: conda run -n soft_robotics -- python ...")
    sys.exit(1)

from tools.metrics import extract_final_iae, compute_metrics_from_df

# ---------------------------------------------------------------------------
# File discovery
# ---------------------------------------------------------------------------

def _discover_csvs(root: Path) -> list[Path]:
    return sorted((root / "case-study").rglob("logs/run_*.csv"))


def _parse_filename(path: Path) -> tuple[str, str, str]:
    """Return (study, scenario, controller) from a logs/run_*.csv path.

    Study  = parent of 'logs/' directory.
    Stem   = run_{scenario_parts}_{controller}  where controller has no underscores
             in the standard naming convention.  We split on '_', take the last
             token as controller and rejoin the rest as scenario.
    """
    study = path.parent.parent.name        # parent of logs/
    stem  = path.stem                      # e.g. run_s01_step_ref_PID
    # Strip the leading 'run_'
    rest = stem[4:] if stem.startswith("run_") else stem
    parts = rest.split("_")
    if len(parts) >= 2:
        controller = parts[-1]
        scenario   = "_".join(parts[:-1])
    else:
        controller = rest
        scenario   = "unknown"
    return study, scenario, controller


# ---------------------------------------------------------------------------
# Per-file metric extraction
# ---------------------------------------------------------------------------

METRICS_ORDER = ["iae", "rms_error", "settle_time_s", "overshoot_pct", "max_u", "energy_var"]

def _extract_row(path: Path) -> dict:
    study, scenario, controller = _parse_filename(path)
    row = dict(study=study, scenario=scenario, controller=controller)
    try:
        df = pd.read_csv(path)
        row["iae"] = extract_final_iae(df)
        m = compute_metrics_from_df(df)
        row["rms_error"]    = m["rms_error"]
        row["settle_time_s"] = m["settle_time_s"]
        row["overshoot_pct"] = m["overshoot_pct"]
        row["max_u"]         = m["max_u"]
        row["energy_var"]    = m["energy_var"]
    except Exception as exc:
        row["iae"] = float("nan")
        for k in ["rms_error", "settle_time_s", "overshoot_pct", "max_u", "energy_var"]:
            row[k] = float("nan")
        row["_error"] = str(exc)
    return row


# ---------------------------------------------------------------------------
# Filtering helpers
# ---------------------------------------------------------------------------

def _matches(value: str, pattern: str) -> bool:
    return pattern.lower() in value.lower()


# ---------------------------------------------------------------------------
# Table formatting
# ---------------------------------------------------------------------------

def _fmt(value, metric: str) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "   N/A"
    if metric == "settle_time_s":
        if value < 0:
            return " never"
        return f"{value:8.3f}"
    if metric in ("iae", "rms_error", "energy_var", "max_u"):
        return f"{value:10.4f}"
    if metric == "overshoot_pct":
        return f"{value:7.2f}%"
    return str(value)


def _print_table(rows: list[dict], sort_by: str, sort_asc: bool, wide: bool, show_study: bool):
    if not rows:
        print("No CSV files found matching the filters.")
        return

    sorted_rows = sorted(
        rows,
        key=lambda r: (float("inf") if math.isnan(r.get(sort_by, float("nan"))) else r.get(sort_by, float("nan"))),
        reverse=not sort_asc,
    )

    # Column widths
    study_w = max((len(r["study"]) for r in sorted_rows), default=5)
    scen_w  = max((len(r["scenario"]) for r in sorted_rows), default=8)
    ctrl_w  = max((len(r["controller"]) for r in sorted_rows), default=10)

    cols = ["iae"]
    if wide:
        cols = METRICS_ORDER
    elif sort_by != "iae" and sort_by in METRICS_ORDER:
        cols = ["iae", sort_by]

    # Header
    header_parts = []
    if show_study:
        header_parts.append(f"{'Study':<{study_w}}")
    header_parts.append(f"{'Scenario':<{scen_w}}")
    header_parts.append(f"{'Controller':<{ctrl_w}}")
    for c in cols:
        header_parts.append(f"  {c:>12}")
    header = "  ".join(header_parts)
    sep = "-" * len(header)
    print(sep)
    print(header)
    print(sep)

    for r in sorted_rows:
        parts = []
        if show_study:
            parts.append(f"{r['study']:<{study_w}}")
        parts.append(f"{r['scenario']:<{scen_w}}")
        parts.append(f"{r['controller']:<{ctrl_w}}")
        for c in cols:
            val = r.get(c, float("nan"))
            parts.append(f"  {_fmt(val, c):>12}")
        print("  ".join(parts))

    print(sep)
    print(f"  {len(sorted_rows)} runs  |  sort: {sort_by} {'asc' if sort_asc else 'desc'}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study",    default="", help="filter study name (partial match)")
    ap.add_argument("--scenario", default="", help="filter scenario name (partial match)")
    ap.add_argument("--controller", default="", help="filter controller name (partial match)")
    ap.add_argument("--metric",   default="iae",
                    choices=METRICS_ORDER,
                    help="primary sort metric (default: iae)")
    ap.add_argument("--sort",     default="asc", choices=["asc", "desc"],
                    help="sort direction (default: asc)")
    ap.add_argument("--wide",     action="store_true",
                    help="show all metrics in the table")
    ap.add_argument("--csv",      default="", metavar="FILE",
                    help="also write the full results table to this CSV file")
    args = ap.parse_args(argv)

    csvs = _discover_csvs(_ROOT)
    if not csvs:
        print(f"No run_*.csv files found under {_ROOT / 'case-study'}")
        sys.exit(1)

    # Filter
    if args.study:
        csvs = [p for p in csvs if _matches(p.parent.parent.name, args.study)]
    if args.scenario:
        csvs = [p for p in csvs if _matches(_parse_filename(p)[1], args.scenario)]
    if args.controller:
        csvs = [p for p in csvs if _matches(_parse_filename(p)[2], args.controller)]

    print(f"Processing {len(csvs)} CSV files ...")
    rows = [_extract_row(p) for p in csvs]

    # Warn about parse errors
    errors = [(r["study"], r["controller"], r.get("_error")) for r in rows if "_error" in r]
    if errors:
        print(f"\nWARN: {len(errors)} file(s) could not be parsed:")
        for study, ctrl, err in errors[:5]:
            print(f"  {study}/{ctrl}: {err}")
        if len(errors) > 5:
            print(f"  ... and {len(errors)-5} more")

    # Multi-study display
    unique_studies = {r["study"] for r in rows}
    show_study = len(unique_studies) > 1

    _print_table(rows, args.metric, args.sort == "asc", args.wide, show_study)

    # Optional CSV output
    if args.csv:
        fields = ["study", "scenario", "controller"] + METRICS_ORDER
        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        print(f"\nResults written to: {args.csv}")


if __name__ == "__main__":
    main()
