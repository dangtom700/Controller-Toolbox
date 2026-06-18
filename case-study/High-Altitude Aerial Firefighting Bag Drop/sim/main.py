"""
main.py
High-Altitude Aerial Firefighting Bag Drop Case Study - entry point.

Runs 12 planners x 5 scenarios = 60 simulations.
Primary metric: CEP (circular error probable) [m] instead of IAE.
Writes CSV telemetry to ../logs/ and prints a summary table.

Usage (from project root):
  conda run -n soft_robotics -- python "case-study/High-Altitude Aerial Firefighting Bag Drop/sim/main.py"

Reference:
  Sun et al. (2025). Water-absorbing bags for high-altitude aerial
  firefighting: Mathematical modeling and drop pattern analysis.
  Results in Engineering, 27, 105940.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from planners import make_planners
from simulation_runner import run_simulation


def load_json(path: str) -> dict:
    with open(path, 'r') as fh:
        return json.load(fh)


def main():
    this_dir   = os.path.dirname(os.path.abspath(__file__))
    base_dir   = os.path.dirname(this_dir)            # StudyName/
    config_dir = os.path.join(base_dir, 'config')
    scen_dir   = os.path.join(config_dir, 'scenarios')
    log_dir    = os.path.join(base_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    plant_params = load_json(os.path.join(config_dir, 'plant_params.json'))

    scen_files = sorted([
        os.path.join(scen_dir, f)
        for f in os.listdir(scen_dir)
        if f.endswith('.json')
    ])
    if not scen_files:
        print("ERROR: no scenario JSON files found in", scen_dir)
        sys.exit(1)

    scenarios = [load_json(sf) for sf in scen_files]
    planners  = make_planners(plant_params)

    total = len(scenarios) * len(planners)
    done  = 0

    print("=" * 72)
    print("  High-Altitude Aerial Firefighting Bag Drop Case Study")
    print(f"  {len(planners)} planners x {len(scenarios)} scenarios = {total} runs")
    print("  Primary metric: CEP = 50th-percentile radial impact error [m]")
    print("=" * 72)

    results = []
    for sc in scenarios:
        for pl in planners:
            done += 1
            label = f"[{done:>2}/{total}]  {sc['id']:30s} | {pl.name()}"
            print(label, end=' ... ', flush=True)
            try:
                res = run_simulation(plant_params, sc, pl, log_dir)
                results.append(res)
                print(f"CEP={res['IAE']:6.2f} m  P95={res['max_error']:6.2f} m  "
                      f"W={res['pattern_width']:5.2f} m")
            except Exception as exc:
                import traceback
                print(f"ERROR: {exc}")
                traceback.print_exc()

    print()
    print("=" * 72)
    print(f"  Done. {len(results)}/{total} runs completed.  Logs: {log_dir}")
    print("=" * 72)
    print()
    print(f"  {'Scenario':<30} {'Planner':<20} {'CEP [m]':>9} "
          f"{'P95 [m]':>8} {'Width [m]':>9}")
    print("  " + "-" * 78)
    for r in results:
        print(f"  {r['scenario_id']:<30} {r['name']:<20} "
              f"{r['IAE']:>9.2f} {r['max_error']:>8.2f} "
              f"{r['pattern_width']:>9.2f}")


if __name__ == '__main__':
    main()


# ===========================================================================
# Analysis hooks - for tools/monte_carlo.py and tools/fault_sweep.py
# See tools/study_protocol.py for the full contract.
# Note: "controllers" here are drop planners; CONTROLLER_NAMES lists planners.
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
    CONTROLLER_NAMES = [p.name() for p in make_planners(_H_NOM)]
except Exception:
    _H_NOM = {}
    CONTROLLER_NAMES = []


def run_single(ctrl_name, params=None, scenario_id=None):
    """Run planner ctrl_name with optional perturbed params; returns metrics dict."""
    _p = {**_copy.deepcopy(_H_NOM), **(params or {})}
    sc = _h_scenario(scenario_id)
    pl_list = make_planners(_p)
    pl = next((x for x in pl_list if x.name() == ctrl_name), None)
    if pl is None:
        raise ValueError(f"Unknown planner {ctrl_name!r}")
    with _tempfile.TemporaryDirectory() as tmp:
        return run_simulation(_p, sc, pl, tmp)


def run_with_fault(ctrl_name, fault, scenario_id=None):
    """Run planner ctrl_name with FaultSpec injected; returns metrics dict."""
    from tools.fault_injector import FaultInjector
    _p = _copy.deepcopy(_H_NOM)
    sc = _h_scenario(scenario_id)
    pl_list = make_planners(_p)
    pl = next((x for x in pl_list if x.name() == ctrl_name), None)
    if pl is None:
        raise ValueError(f"Unknown planner {ctrl_name!r}")
    with _tempfile.TemporaryDirectory() as tmp:
        return run_simulation(_p, sc, pl, tmp,
                              fault_injector=FaultInjector([fault]))
