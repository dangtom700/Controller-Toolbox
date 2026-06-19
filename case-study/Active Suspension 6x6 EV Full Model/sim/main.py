"""
main.py
Active Suspension 6x6 EV Full Model - entry point.

Runs 18 controllers x 5 scenarios = 90 simulations.
Writes CSV telemetry to ../logs/ and prints a summary table.

Usage (from project root):
  conda run -n soft_robotics -- python "case-study/Active Suspension 6x6 EV Full Model/sim/main.py"
"""

import csv as _csv
import json
import os
import sys

# Path setup
_SIM_DIR  = os.path.dirname(os.path.abspath(__file__))
_BASE_DIR = os.path.dirname(_SIM_DIR)
_ROOT     = os.path.dirname(os.path.dirname(_BASE_DIR))

sys.path.insert(0, _SIM_DIR)
sys.path.insert(0, _ROOT)
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DiscretePID'):
        raise AttributeError("ctrl_toolbox missing DiscretePID - rebuild bindings")
    _CTRL_AVAILABLE = True
    _CTRL_ERR = None
except (ImportError, AttributeError) as _e:
    _CTRL_AVAILABLE = False
    _CTRL_ERR = str(_e)

from ev6x6_plant      import EV6x6Plant, DEFAULT_PARAMS
from controllers      import make_controllers, controller_names
from simulation_runner import run_simulation


def _load_json(path: str) -> dict:
    with open(path, 'r') as fh:
        return json.load(fh)


def main():
    if not _CTRL_AVAILABLE:
        print(f"SKIP: {_CTRL_ERR}")
        sys.exit(0)
    config_dir = os.path.join(_BASE_DIR, 'config')
    scen_dir   = os.path.join(config_dir, 'scenarios')
    log_dir    = os.path.join(_BASE_DIR,  'logs')
    os.makedirs(log_dir, exist_ok=True)

    # Load plant params (fall back to defaults if JSON absent)
    plant_json = os.path.join(config_dir, 'plant_params.json')
    if os.path.isfile(plant_json):
        plant_params = _load_json(plant_json)
        merged = dict(DEFAULT_PARAMS)
        merged.update(plant_params)
        plant_params = merged
    else:
        plant_params = dict(DEFAULT_PARAMS)

    # Load scenarios
    scen_files = sorted([
        os.path.join(scen_dir, f)
        for f in os.listdir(scen_dir)
        if f.endswith('.json')
    ])
    if not scen_files:
        print("ERROR: no scenario JSON files found in", scen_dir)
        sys.exit(1)

    scenarios = [_load_json(sf) for sf in scen_files]

    # Build plant once (for LQR/LQG/MPC constructors that need discretised matrices)
    plant_ref = EV6x6Plant(plant_params)

    # Build controllers (metaheuristic ones run offline optimisation in __init__)
    print("Building controllers (GA/PSO/DE offline optimisation will run now)...")
    controllers = make_controllers(plant_params, plant_ref)

    total = len(scenarios) * len(controllers)
    done  = 0

    print("=" * 74)
    print("  Active Suspension 6x6 EV Full Model Case Study")
    print(f"  {len(controllers)} controllers x {len(scenarios)} scenarios = {total} runs")
    print("=" * 74)

    results = []
    for sc in scenarios:
        for ctr in controllers:
            done += 1
            label = f"[{done:>2}/{total}]  {sc['id']:28s} | {ctr.name()}"
            print(label, end=' ... ', flush=True)
            try:
                res = run_simulation(plant_params, sc, ctr, log_dir)
                results.append(res)
                print(f"IAE={res['IAE']:.4f} m*s  RMS_body={res['RMS_body']*1e3:.2f} mm")
            except Exception as exc:
                print(f"ERROR: {exc}")

    print()
    print("=" * 74)
    print(f"  Done. {len(results)}/{total} runs completed.  Logs: {log_dir}")
    print("=" * 74)
    print()
    print(f"  {'Scenario':<28} {'Controller':<20} {'IAE [m*s]':>10} {'RMS_body[mm]':>13}")
    print("  " + "-" * 74)
    for r in results:
        print(f"  {r['scenario_id']:<28} {r['name']:<20} "
              f"{r['IAE']:>10.4f} {r['RMS_body']*1e3:>13.3f}")


if __name__ == '__main__':
    main()


# ===========================================================================
# Analysis hooks - for tools/monte_carlo.py and tools/fault_sweep.py
# See tools/study_protocol.py for the full contract.
# ===========================================================================
import copy as _copy
import tempfile as _tempfile

_H_CFG  = os.path.join(_BASE_DIR, 'config')
_H_SCEN = os.path.join(_H_CFG,    'scenarios')


def _h_json(path):
    with open(path) as _fh:
        return json.load(_fh)


def _h_nom():
    _pj = os.path.join(_H_CFG, 'plant_params.json')
    raw = _h_json(_pj) if os.path.isfile(_pj) else {}
    merged = dict(DEFAULT_PARAMS)
    merged.update(raw)
    return merged


def _h_scenario(sid=None):
    fnames = sorted(f for f in os.listdir(_H_SCEN) if f.endswith('.json'))
    if not fnames:
        raise RuntimeError("No scenario JSONs in " + _H_SCEN)
    if sid is None:
        return _h_json(os.path.join(_H_SCEN, fnames[0]))
    for fn in fnames:
        sc = _h_json(os.path.join(_H_SCEN, fn))
        if sc.get('id') == sid:
            return sc
    raise ValueError(f"Scenario {sid!r} not found")


# CONTROLLER_NAMES: static roster, no controller construction at import time
# (controller_names() just reads _CONTROLLER_FACTORIES - instant).
try:
    _H_NOM = _h_nom()
    CONTROLLER_NAMES = controller_names() if _CTRL_AVAILABLE else []
except Exception:
    _H_NOM = dict(DEFAULT_PARAMS)
    CONTROLLER_NAMES = []


def run_single(ctrl_name, params=None, scenario_id=None):
    """Run ctrl_name with optional perturbed params; returns metrics dict."""
    if not _CTRL_AVAILABLE:
        raise ImportError(f"ctrl_toolbox not available: {_CTRL_ERR}")
    _p = {**_copy.deepcopy(_H_NOM), **(params or {})}
    sc = _h_scenario(scenario_id)
    plant_ref = EV6x6Plant(_p)
    ctrl_obj = make_controllers(_p, plant_ref, only=ctrl_name)[0]
    with _tempfile.TemporaryDirectory() as tmp:
        return run_simulation(_p, sc, ctrl_obj, tmp)


def run_with_fault(ctrl_name, fault, scenario_id=None):
    """Run ctrl_name with FaultSpec injected; returns metrics dict."""
    if not _CTRL_AVAILABLE:
        raise ImportError(f"ctrl_toolbox not available: {_CTRL_ERR}")
    from tools.fault_injector import FaultInjector
    _p = _copy.deepcopy(_H_NOM)
    sc = _h_scenario(scenario_id)
    plant_ref = EV6x6Plant(_p)
    ctrl_obj = make_controllers(_p, plant_ref, only=ctrl_name)[0]
    with _tempfile.TemporaryDirectory() as tmp:
        return run_simulation(_p, sc, ctrl_obj, tmp,
                              fault_injector=FaultInjector([fault]))


def run_wcet_profile(scenario_id=None, log_dir=None):
    """Time controller.compute() per step for every controller on one scenario.

    Writes logs/wcet_{scenario_id}.csv (controller, step_time_us, step_index)
    for tools/wcet_report.py. Uses make_controllers(only=name) so each
    controller is built in isolation - construction cost is excluded from
    the per-step compute() timing.
    """
    if not _CTRL_AVAILABLE:
        raise ImportError(f"ctrl_toolbox not available: {_CTRL_ERR}")
    sc = _h_scenario(scenario_id)
    log_dir = log_dir or os.path.join(_BASE_DIR, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    rows = []
    for ctrl_name in CONTROLLER_NAMES:
        plant_ref = EV6x6Plant(_H_NOM)
        ctrl_obj = make_controllers(_H_NOM, plant_ref, only=ctrl_name)[0]
        with _tempfile.TemporaryDirectory() as tmp:
            run_simulation(_H_NOM, sc, ctrl_obj, tmp, wcet_sink=rows)

    out_path = os.path.join(log_dir, f"wcet_{sc['id']}.csv")
    with open(out_path, 'w', newline='') as fh:
        writer = _csv.DictWriter(fh, fieldnames=["controller", "step_time_us", "step_index"])
        writer.writeheader()
        writer.writerows(rows)
    return out_path
