"""
main.py
Drill String Torsional Control Case Study - entry point.

Runs all 17 controllers x 5 scenarios = 85 simulations.
Writes CSV telemetry to ../logs/ and prints a summary table.

Usage (from project root):
  conda run -n soft_robotics -- python "case-study/Vertical Drill String Mathematical Review 2025/sim/main.py"
"""

import json
import os
import sys

# Allow imports from this sim/ directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controllers import make_controllers
from simulation_runner import run_simulation


def load_json(path: str) -> dict:
    with open(path, 'r') as fh:
        return json.load(fh)


def main():
    this_dir   = os.path.dirname(os.path.abspath(__file__))
    base_dir   = os.path.dirname(this_dir)                        # StudyName/
    config_dir = os.path.join(base_dir, 'config')
    scen_dir   = os.path.join(config_dir, 'scenarios')
    log_dir    = os.path.join(base_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    # Load plant
    plant_params = load_json(os.path.join(config_dir, 'plant_params.json'))

    # Discover scenarios
    scen_files = sorted([
        os.path.join(scen_dir, f)
        for f in os.listdir(scen_dir)
        if f.endswith('.json')
    ])
    if not scen_files:
        print("ERROR: no scenario JSON files found in", scen_dir)
        sys.exit(1)

    scenarios = [load_json(sf) for sf in scen_files]

    # Create controllers
    controllers = make_controllers(plant_params)

    total = len(scenarios) * len(controllers)
    done  = 0

    print("=" * 60)
    print("  Drill String Torsional Control Case Study")
    print(f"  {len(controllers)} controllers x {len(scenarios)} scenarios = {total} runs")
    print("=" * 60)

    results = []
    for sc in scenarios:
        for ctrl in controllers:
            done += 1
            label = f"[{done:>2}/{total}]  {sc['id']:30s} | {ctrl.name()}"
            print(label, end=' ... ', flush=True)
            try:
                res = run_simulation(plant_params, sc, ctrl, log_dir)
                results.append(res)
                print(f"IAE={res['IAE']:8.2f}  max_err={res['max_error']:6.2f}")
            except Exception as exc:
                print(f"ERROR: {exc}")

    # Summary table
    print()
    print("=" * 60)
    print(f"  Done. {len(results)}/{total} runs completed. Logs: {log_dir}")
    print("=" * 60)
    print()
    print(f"  {'Scenario':<30} {'Controller':<16} {'IAE':>10} {'MaxErr':>8}")
    print("  " + "-" * 68)
    for r in results:
        print(f"  {r['scenario_id']:<30} {r['name']:<16} "
              f"{r['IAE']:>10.2f} {r['max_error']:>8.2f}")


if __name__ == '__main__':
    main()


# ===========================================================================
# Analysis hooks - for tools/monte_carlo.py and tools/fault_sweep.py
# See tools/study_protocol.py for the full contract.
# ===========================================================================
import copy as _copy
import tempfile as _tempfile

_H_SIM  = os.path.dirname(os.path.abspath(__file__))
_H_BASE = os.path.dirname(_H_SIM)
_H_CFG  = os.path.join(_H_BASE, 'config')
_H_SCEN = os.path.join(_H_CFG,  'scenarios')


def _h_json(path):
    with open(path) as _fh:
        return json.load(_fh)


def _h_nom():
    return _h_json(os.path.join(_H_CFG, 'plant_params.json'))


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


try:
    _H_NOM = _h_nom()
    CONTROLLER_NAMES = [c.name() for c in make_controllers(_H_NOM)]
except Exception:
    _H_NOM = {}
    CONTROLLER_NAMES = []


def run_single(ctrl_name, params=None, scenario_id=None):
    """Run ctrl_name with optional perturbed params; returns metrics dict."""
    _p = {**_copy.deepcopy(_H_NOM), **(params or {})}
    sc = _h_scenario(scenario_id)
    ctrls = make_controllers(_p)
    ctrl_obj = next((c for c in ctrls if c.name() == ctrl_name), None)
    if ctrl_obj is None:
        raise ValueError(f"Unknown controller {ctrl_name!r}")
    with _tempfile.TemporaryDirectory() as tmp:
        return run_simulation(_p, sc, ctrl_obj, tmp)


def run_with_fault(ctrl_name, fault, scenario_id=None):
    """Run ctrl_name with FaultSpec injected; returns metrics dict."""
    from tools.fault_injector import FaultInjector
    _p = _copy.deepcopy(_H_NOM)
    sc = _h_scenario(scenario_id)
    ctrls = make_controllers(_p)
    ctrl_obj = next((c for c in ctrls if c.name() == ctrl_name), None)
    if ctrl_obj is None:
        raise ValueError(f"Unknown controller {ctrl_name!r}")
    with _tempfile.TemporaryDirectory() as tmp:
        return run_simulation(_p, sc, ctrl_obj, tmp,
                              fault_injector=FaultInjector([fault]))
