#!/usr/bin/env python3
"""
tools/compare_controllers.py - IAE / ISE comparison table across all case-study CSVs.

Usage:
    conda run -n soft_robotics -- python tools/compare_controllers.py [--study PATTERN] [--scenario PATTERN]

Options:
    --study     Filter case studies by name substring (case-insensitive).
    --scenario  Filter scenarios by name substring (case-insensitive).
    --sort      Sort column: iae (default), ise, controller, scenario.
    --wide      Print all IAE sub-channels individually (default: sum).

Each case-study logs/ directory is auto-discovered.  CSV filename convention:
    run_<scenario>_<ControllerName>.csv

IAE / ISE detection:
    * If the CSV already has columns matching /IAE_/ or /ISE_/, the last row is used
      (these are the running totals emitted by the simulation).
    * Otherwise, error columns matching /error/ are integrated numerically using the
      time column (first column or column matching /^t/).
"""

import csv
import math
import os
import re
import sys
import argparse
from pathlib import Path
from collections import defaultdict


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_case_study_roots(repo_root: Path):
    """Yield (study_name, logs_dir) for every case study that has CSV logs."""
    cs_dir = repo_root / "case-study"
    if not cs_dir.is_dir():
        return
    for study_dir in sorted(cs_dir.iterdir()):
        if not study_dir.is_dir():
            continue
        logs_dir = study_dir / "logs"
        if logs_dir.is_dir() and any(logs_dir.glob("*.csv")):
            yield study_dir.name, logs_dir


_FILE_RE = re.compile(r"^run_(.+)_([^_]+)\.csv$")


def _parse_filename(name: str):
    """Return (scenario, controller) from 'run_<scenario>_<controller>.csv', or None."""
    m = _FILE_RE.match(name)
    return (m.group(1), m.group(2)) if m else None


def _read_csv_metrics(csv_path: Path):
    """
    Return a dict with keys 'iae', 'ise', 'iae_channels', 'ise_channels'.
    iae / ise are scalar sums across all tracked channels.
    iae_channels / ise_channels are {col_name: value} dicts for per-channel detail.
    Returns None on parse failure.
    """
    try:
        with open(csv_path, newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            rows = list(reader)
    except Exception:
        return None

    if len(rows) < 2:
        return None

    header = rows[0]
    last   = rows[-1]
    if len(last) != len(header):
        return None

    row = {h: v for h, v in zip(header, last)}

    # -- Try pre-computed IAE / ISE columns first --------------------------------
    iae_cols = [h for h in header if re.match(r"IAE_", h, re.I)]
    ise_cols = [h for h in header if re.match(r"ISE_", h, re.I)]

    if iae_cols:
        iae_ch = {}
        for c in iae_cols:
            try:
                iae_ch[c] = float(row[c])
            except (ValueError, KeyError):
                pass
        ise_ch = {}
        for c in ise_cols:
            try:
                ise_ch[c] = float(row[c])
            except (ValueError, KeyError):
                pass
        iae_sum = sum(v for v in iae_ch.values() if math.isfinite(v))
        ise_sum = sum(v for v in ise_ch.values() if math.isfinite(v))
        return {"iae": iae_sum, "ise": ise_sum if ise_ch else None,
                "iae_channels": iae_ch, "ise_channels": ise_ch}

    # -- Fall back: integrate error columns numerically -------------------------
    err_cols = [h for h in header if re.search(r"error", h, re.I)]
    if not err_cols:
        return None

    # Detect time column
    t_col = next((h for h in header if re.match(r"^t\b", h, re.I)), None)

    iae_ch, ise_ch = {}, {}
    for ec in err_cols:
        prev_t, prev_e = None, None
        iae_acc, ise_acc = 0.0, 0.0
        for data_row in rows[1:]:
            if len(data_row) != len(header):
                continue
            r = {h: v for h, v in zip(header, data_row)}
            try:
                e = float(r[ec])
                t = float(r[t_col]) if t_col else 1.0
            except (ValueError, KeyError):
                continue
            if prev_t is not None:
                dt = t - prev_t
                if dt > 0:
                    iae_acc += abs(e) * dt
                    ise_acc += e * e * dt
            prev_t, prev_e = t, e
        iae_ch[ec] = iae_acc
        ise_ch[ec] = ise_acc

    iae_sum = sum(v for v in iae_ch.values() if math.isfinite(v))
    ise_sum = sum(v for v in ise_ch.values() if math.isfinite(v))
    return {"iae": iae_sum, "ise": ise_sum,
            "iae_channels": iae_ch, "ise_channels": ise_ch}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="IAE/ISE comparison table for all case studies.")
    parser.add_argument("--study",    default="", help="Filter by study name substring.")
    parser.add_argument("--scenario", default="", help="Filter by scenario name substring.")
    parser.add_argument("--sort",     default="iae", choices=["iae", "ise", "controller", "scenario"],
                        help="Column to sort by (default: iae).")
    parser.add_argument("--wide",     action="store_true",
                        help="Print individual IAE channels instead of sum.")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent

    # Collect all data rows
    records = []  # list of dicts

    for study_name, logs_dir in _find_case_study_roots(repo_root):
        if args.study and args.study.lower() not in study_name.lower():
            continue

        for csv_file in sorted(logs_dir.glob("*.csv")):
            parsed = _parse_filename(csv_file.name)
            if parsed is None:
                continue
            scenario, controller = parsed

            if args.scenario and args.scenario.lower() not in scenario.lower():
                continue

            metrics = _read_csv_metrics(csv_file)
            if metrics is None:
                continue

            records.append({
                "study":      study_name,
                "scenario":   scenario,
                "controller": controller,
                "iae":        metrics["iae"],
                "ise":        metrics["ise"],
                "iae_ch":     metrics["iae_channels"],
                "ise_ch":     metrics["ise_channels"],
            })

    if not records:
        print("No CSV data found. Run the case-study simulations first (python run.py).")
        sys.exit(1)

    # Sort
    def _sort_key(r):
        if args.sort == "iae":
            return (r["study"], r["scenario"], r["iae"] if r["iae"] is not None else 1e18)
        if args.sort == "ise":
            return (r["study"], r["scenario"], r["ise"] if r["ise"] is not None else 1e18)
        if args.sort == "controller":
            return (r["study"], r["scenario"], r["controller"])
        return (r["study"], r["controller"], r["scenario"])

    records.sort(key=_sort_key)

    # Compute per-(study, scenario) rank by IAE
    rank_map = {}
    groups = defaultdict(list)
    for rec in records:
        groups[(rec["study"], rec["scenario"])].append(rec)
    for key, group in groups.items():
        ranked = sorted(group, key=lambda r: r["iae"])
        for i, r in enumerate(ranked):
            rank_map[(r["study"], r["scenario"], r["controller"])] = i + 1

    # Print
    print()
    print(f"{'Study':<45}  {'Scenario':<30}  {'Controller':<22}  {'IAE':>12}  {'ISE':>12}  {'Rank':>4}")
    print("-" * 135)

    current_study = None
    for rec in records:
        if rec["study"] != current_study:
            if current_study is not None:
                print()
            current_study = rec["study"]

        iae_str = f"{rec['iae']:>12.3f}" if rec["iae"] is not None else f"{'N/A':>12}"
        ise_str = f"{rec['ise']:>12.3f}" if rec["ise"] is not None else f"{'N/A':>12}"
        rank    = rank_map.get((rec["study"], rec["scenario"], rec["controller"]), "-")
        rank_str = f"#{rank:>3}" if isinstance(rank, int) else f"{'':>4}"

        # Study abbreviation to fit column
        study_short = rec["study"][:44]
        print(f"{study_short:<45}  {rec['scenario']:<30}  {rec['controller']:<22}  {iae_str}  {ise_str}  {rank_str}")

        if args.wide and rec["iae_ch"]:
            for ch, val in rec["iae_ch"].items():
                print(f"  {'':45}  {'':30}  {'  IAE '+ch:<22}  {val:>12.3f}")

    print()

    # Summary: best controller per study
    print("=" * 135)
    print("Best controller per (study, scenario) by IAE:")
    print()
    for (study, scenario), group in sorted(groups.items()):
        best = min(group, key=lambda r: r["iae"])
        study_short = study[:44]
        print(f"  {study_short:<44}  {scenario:<30}  {best['controller']:<22}  IAE={best['iae']:.3f}")
    print()


if __name__ == "__main__":
    main()
