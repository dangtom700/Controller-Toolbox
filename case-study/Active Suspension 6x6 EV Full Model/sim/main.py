"""
main.py
Active Suspension 6x6 EV Full Model - entry point.

Runs 18 controllers x 5 scenarios = 90 simulations.
Writes CSV telemetry to ../logs/ and prints a summary table.

Usage (from project root):
  conda run -n soft_robotics -- python "case-study/Active Suspension 6x6 EV Full Model/sim/main.py"
"""

import json
import os
import sys

# Path setup
_SIM_DIR  = os.path.dirname(os.path.abspath(__file__))
_BASE_DIR = os.path.dirname(_SIM_DIR)
_ROOT     = os.path.dirname(os.path.dirname(_BASE_DIR))

sys.path.insert(0, _SIM_DIR)
sys.path.insert(0, os.path.join(_ROOT, 'build', 'bindings'))
if sys.platform == 'win32' and hasattr(os, 'add_dll_directory'):
    for _p in [r'C:\msys64\mingw64\bin']:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DiscretePID'):
        raise AttributeError("ctrl_toolbox missing DiscretePID - rebuild bindings")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

from ev6x6_plant      import EV6x6Plant, DEFAULT_PARAMS
from controllers      import make_controllers
from simulation_runner import run_simulation


def _load_json(path: str) -> dict:
    with open(path, 'r') as fh:
        return json.load(fh)


def main():
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
