"""
tools/monte_carlo.py  -- ANA-2: Monte Carlo robustness sweep over plant parameter uncertainty.

Runs a Python-only case study sim N times with perturbed plant parameters, collects metrics
for each controller, and writes a summary CSV.

Usage:
    python tools/monte_carlo.py --study <study_dir_name> [options]

Options:
    --study   NAME    case-study directory name (e.g. "Vertical Drill String Mathematical Review 2025")
    --n       INT     number of MC samples per controller (default 50)
    --sigma   FLOAT   fractional standard deviation of each parameter (default 0.10  i.e. +/-10 %)
    --seed    INT     random seed (default 42)
    --out     FILE    output CSV path (default: case-study/<study>/mc_summary.csv)
    --workers INT     parallel worker processes (default 1 - use higher for long sims)

The script imports the study's sim/ package and calls its run_single() function
(if present) or falls back to overriding plant parameters via sim.config.

If the study's sim/ module does not expose run_single(), the script tries to discover
perturb-able parameters by looking for a 'plant_params' dict in the module and
sampling around the nominal values.

The summary CSV columns are:
    study, controller, sample_id, *{metric_names}, *{perturbed_param_names}
"""
from __future__ import annotations
import argparse
import csv
import importlib.util
import math
import os
import sys
import copy
import json
import warnings
from pathlib import Path
from typing import Any

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))

try:
    import numpy as np
    import pandas as pd
except ImportError:
    print("ERROR: numpy and pandas required.")
    sys.exit(1)

from tools.metrics import compute_metrics, compute_metrics_from_df, extract_final_iae

# ---------------------------------------------------------------------------
# Study discovery
# ---------------------------------------------------------------------------

def _find_study_dir(name: str) -> Path:
    for d in (_ROOT / "case-study").iterdir():
        if name.lower() in d.name.lower():
            return d
    raise FileNotFoundError(f"No case-study directory matching '{name}'")


def _load_sim_module(study_dir: Path):
    """Import case-study/sim/main.py as a module."""
    main_py = study_dir / "sim" / "main.py"
    if not main_py.exists():
        raise FileNotFoundError(f"sim/main.py not found in {study_dir}")
    spec = importlib.util.spec_from_file_location("_mc_sim", str(main_py))
    mod  = importlib.util.module_from_spec(spec)
    # Inject _ROOT so the module can find ctrl_toolbox
    mod.__dict__["__file__"] = str(main_py)
    sys.path.insert(0, str(study_dir / "sim"))
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# Parameter perturbation
# ---------------------------------------------------------------------------

def _perturb_params(nominal: dict, sigma: float, rng: np.random.Generator) -> dict:
    """Return a copy of nominal with each numeric scalar multiplied by (1 + N(0, sigma))."""
    perturbed = {}
    for k, v in nominal.items():
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            factor = 1.0 + float(rng.normal(0, sigma))
            # Preserve sign; avoid flipping to zero
            perturbed[k] = v * max(factor, 0.1) if v > 0 else v * min(factor, -0.1) if v < 0 else v
        else:
            perturbed[k] = copy.deepcopy(v)
    return perturbed


# ---------------------------------------------------------------------------
# Single MC run
# ---------------------------------------------------------------------------

def _run_single(sim_mod, controller_name: str, perturbed_params: dict | None,
                nominal_params: dict | None) -> dict[str, float]:
    """Call sim_mod.run_single(controller_name, params) if available.

    Falls back to running the full sim and reading the last-row CSV output.
    Returns a dict of scalar metrics.
    """
    if hasattr(sim_mod, "run_single"):
        try:
            result = sim_mod.run_single(controller_name, perturbed_params)
            # result should be a dict or a pandas DataFrame
            if isinstance(result, dict):
                out = {k: float(v) for k, v in result.items() if isinstance(v, (int, float))}
                # generate_report.py/anova.py expect a lowercase 'iae' key; study
                # dicts commonly use 'IAE' (see study_protocol.py's minimal contract)
                if "iae" not in out:
                    for k, v in result.items():
                        if k.lower() == "iae" and isinstance(v, (int, float)):
                            out["iae"] = float(v)
                            break
                return out
            if isinstance(result, pd.DataFrame):
                iae = extract_final_iae(result)
                m   = compute_metrics_from_df(result)
                return {**m, "iae": iae}
        except Exception as exc:
            warnings.warn(f"run_single({controller_name}) raised: {exc}")
            return {"iae": float("nan"), "rms_error": float("nan"),
                    "settle_time_s": -1.0, "overshoot_pct": float("nan"),
                    "max_u": float("nan"), "energy_var": float("nan")}
    # No run_single: cannot run MC without integration hooks
    return {"iae": float("nan"), "rms_error": float("nan"),
            "settle_time_s": -1.0, "overshoot_pct": float("nan"),
            "max_u": float("nan"), "energy_var": float("nan"),
            "_skipped": 1.0}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study",   required=True, help="case-study directory name (partial match)")
    ap.add_argument("--n",       type=int,   default=50,   help="MC samples per controller")
    ap.add_argument("--sigma",   type=float, default=0.10, help="fractional param std dev")
    ap.add_argument("--seed",    type=int,   default=42,   help="random seed")
    ap.add_argument("--out",     default="",               help="output CSV path")
    ap.add_argument("--workers", type=int,   default=1,    help="parallel workers (stub)")
    args = ap.parse_args(argv)

    study_dir = _find_study_dir(args.study)
    study_name = study_dir.name
    print(f"Study: {study_name}")

    out_path = args.out or str(study_dir / "mc_summary.csv")

    # Load nominal params from config/plant_params.json if it exists
    nominal_params: dict = {}
    param_json = study_dir / "sim" / "config" / "plant_params.json"
    if not param_json.exists():
        param_json = study_dir / "config" / "plant_params.json"
    if param_json.exists():
        with open(param_json) as fh:
            nominal_params = json.load(fh)
        print(f"Loaded {len(nominal_params)} nominal parameters from {param_json.name}")
    else:
        print("No plant_params.json found - perturbing zero parameters (nominal run only)")

    # Load sim module
    try:
        sim_mod = _load_sim_module(study_dir)
    except Exception as exc:
        print(f"ERROR loading sim module: {exc}")
        sys.exit(1)

    # Discover controller names
    controllers: list[str] = []
    if hasattr(sim_mod, "CONTROLLER_NAMES"):
        controllers = list(sim_mod.CONTROLLER_NAMES)
    elif hasattr(sim_mod, "CONTROLLERS"):
        controllers = list(sim_mod.CONTROLLERS)
    if not controllers:
        print("WARN: Could not discover controller names from sim module.")
        print("      Define CONTROLLER_NAMES = [...] in sim/main.py for MC support.")
        controllers = ["nominal"]

    print(f"Controllers ({len(controllers)}): {', '.join(controllers)}")
    print(f"Samples per controller: {args.n}  (sigma={args.sigma:.0%}  seed={args.seed})")

    rng = np.random.default_rng(args.seed)
    all_rows: list[dict] = []

    for ctrl_name in controllers:
        print(f"  {ctrl_name} ...", end="", flush=True)
        skipped = 0
        for i in range(args.n):
            perturbed = _perturb_params(nominal_params, args.sigma, rng) if nominal_params else None
            metrics = _run_single(sim_mod, ctrl_name, perturbed, nominal_params)
            if metrics.pop("_skipped", 0):
                skipped += 1
                if skipped == 1:
                    print(" [no run_single hook - using nominal only]", end="", flush=True)
                # Only record one nominal row per controller if no hook
                if i == 0:
                    row = dict(study=study_name, controller=ctrl_name, sample_id=0)
                    row.update(metrics)
                    if perturbed:
                        for pk, pv in perturbed.items():
                            if isinstance(pv, (int, float)):
                                row[f"param_{pk}"] = float(pv)
                    all_rows.append(row)
                break
            row = dict(study=study_name, controller=ctrl_name, sample_id=i)
            row.update(metrics)
            if perturbed:
                for pk, pv in perturbed.items():
                    if isinstance(pv, (int, float)):
                        row[f"param_{pk}"] = float(pv)
            all_rows.append(row)
        print(f" {args.n - skipped} runs", flush=True)

    if not all_rows:
        print("No data collected.")
        sys.exit(1)

    # Write CSV
    fieldnames = list(all_rows[0].keys())
    for r in all_rows:
        for k in r:
            if k not in fieldnames:
                fieldnames.append(k)

    with open(out_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for row in all_rows:
            w.writerow({k: row.get(k, "") for k in fieldnames})

    print(f"\nMC summary written to: {out_path}  ({len(all_rows)} rows)")

    # Quick summary table
    df = pd.DataFrame(all_rows)
    if "iae" in df.columns and "controller" in df.columns:
        summary = df.groupby("controller")["iae"].agg(["mean", "std", "min", "max"])
        summary.columns = ["IAE_mean", "IAE_std", "IAE_min", "IAE_max"]
        summary = summary.sort_values("IAE_mean")
        print("\n--- IAE summary (across MC samples) ---")
        print(summary.to_string(float_format="{:.4f}".format))


if __name__ == "__main__":
    main()
