"""
tools/fault_sweep.py  -- ANA-3: sweep fault magnitudes across controllers and write summary CSV.

For each (controller, fault_kind, magnitude) combination, calls sim.run_with_fault()
(if the study exposes it) and records the resulting metrics.

Usage:
    python tools/fault_sweep.py --study <name> [options]

Options:
    --study   NAME    case-study directory name (partial match)
    --faults  LIST    comma-separated fault kinds to sweep (default: sensor_bias,actuator_loss)
    --magnitudes LIST comma-separated list of magnitudes to try per fault kind
                     (default: 0,0.05,0.10,0.20,0.30,0.50)
    --out     FILE    output CSV path (default: fault_sweep_{study}.csv)

Output CSV columns:
    study, controller, fault_kind, magnitude, iae, rms_error, settle_time_s,
    overshoot_pct, max_u, energy_var
"""
from __future__ import annotations
import argparse
import csv
import importlib.util
import sys
import warnings
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

from tools.metrics import compute_metrics_from_df, extract_final_iae
from tools.fault_injector import FaultSpec, FaultInjector


def _find_study_dir(name: str) -> Path:
    for d in (_ROOT / "case-study").iterdir():
        if name.lower() in d.name.lower():
            return d
    raise FileNotFoundError(f"No case-study directory matching '{name}'")


def _load_sim_module(study_dir: Path):
    main_py = study_dir / "sim" / "main.py"
    if not main_py.exists():
        raise FileNotFoundError(f"sim/main.py not found in {study_dir}")
    spec = importlib.util.spec_from_file_location("_fault_sim", str(main_py))
    mod  = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(study_dir / "sim"))
    spec.loader.exec_module(mod)
    return mod


def _run_with_fault(sim_mod, ctrl_name: str, fault: FaultSpec) -> dict[str, float]:
    if hasattr(sim_mod, "run_with_fault"):
        try:
            result = sim_mod.run_with_fault(ctrl_name, fault)
            if isinstance(result, dict):
                return {k: float(v) for k, v in result.items() if isinstance(v, (int, float))}
            if isinstance(result, pd.DataFrame):
                iae = extract_final_iae(result)
                m   = compute_metrics_from_df(result)
                return {**m, "iae": iae}
        except Exception as exc:
            warnings.warn(f"run_with_fault({ctrl_name}, {fault.kind}) raised: {exc}")
    return {"iae": float("nan"), "rms_error": float("nan"),
            "settle_time_s": -1.0, "overshoot_pct": float("nan"),
            "max_u": float("nan"), "energy_var": float("nan"),
            "_no_hook": True}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study",      required=True)
    ap.add_argument("--faults",     default="sensor_bias,actuator_loss")
    ap.add_argument("--magnitudes", default="0,0.05,0.10,0.20,0.30,0.50")
    ap.add_argument("--out",        default="")
    args = ap.parse_args(argv)

    study_dir   = _find_study_dir(args.study)
    study_name  = study_dir.name
    fault_kinds = [f.strip() for f in args.faults.split(",")]
    magnitudes  = [float(m.strip()) for m in args.magnitudes.split(",")]
    out_path    = args.out or f"fault_sweep_{study_name[:40].replace(' ', '_')}.csv"

    print(f"Study: {study_name}")
    print(f"Faults: {fault_kinds}  Magnitudes: {magnitudes}")

    try:
        sim_mod = _load_sim_module(study_dir)
    except Exception as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)

    controllers: list[str] = []
    if hasattr(sim_mod, "CONTROLLER_NAMES"):
        controllers = list(sim_mod.CONTROLLER_NAMES)
    elif hasattr(sim_mod, "CONTROLLERS"):
        controllers = list(sim_mod.CONTROLLERS)
    if not controllers:
        print("WARN: no CONTROLLER_NAMES in sim module - using ['nominal']")
        controllers = ["nominal"]

    metric_keys = ["iae", "rms_error", "settle_time_s", "overshoot_pct", "max_u", "energy_var"]
    rows: list[dict] = []
    no_hook_warned = False

    for ctrl_name in controllers:
        for fk in fault_kinds:
            for mag in magnitudes:
                fault = FaultSpec(kind=fk, fault_time=0.0, magnitude=mag)
                metrics = _run_with_fault(sim_mod, ctrl_name, fault)
                if metrics.pop("_no_hook", False) and not no_hook_warned:
                    print("WARN: sim module has no run_with_fault() hook - all rows will be NaN.")
                    print("      Add run_with_fault(ctrl_name, FaultSpec) -> pd.DataFrame to sim/main.py")
                    no_hook_warned = True
                row = dict(study=study_name, controller=ctrl_name,
                           fault_kind=fk, magnitude=mag)
                for k in metric_keys:
                    row[k] = metrics.get(k, float("nan"))
                rows.append(row)

    fieldnames = ["study", "controller", "fault_kind", "magnitude"] + metric_keys
    with open(out_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    print(f"\nFault sweep written to: {out_path}  ({len(rows)} rows)")


if __name__ == "__main__":
    main()
