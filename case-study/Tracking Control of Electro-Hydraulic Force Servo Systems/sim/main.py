"""
main.py
Electro-Hydraulic Force Servo Systems Control Case Study - entry point.

Runs 12 controllers x 5 scenarios = 60 simulations.
Writes CSV telemetry to ../logs/ and prints a summary table.

Usage (from project root):
  conda run -n soft_robotics -- python "case-study/Tracking Control of Electro-Hydraulic Force Servo Systems/sim/main.py"
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controllers import make_controllers
from simulation_runner import run_simulation


def load_json(path: str) -> dict:
    with open(path, 'r') as fh:
        return json.load(fh)


def main():
    this_dir   = os.path.dirname(os.path.abspath(__file__))
    base_dir   = os.path.dirname(this_dir)
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

    scenarios   = [load_json(sf) for sf in scen_files]
    controllers = make_controllers(plant_params)

    total = len(scenarios) * len(controllers)
    done  = 0

    print("=" * 70)
    print("  Electro-Hydraulic Force Servo Systems Control Case Study")
    print(f"  {len(controllers)} controllers x {len(scenarios)} scenarios = {total} runs")
    print("=" * 70)

    results = []
    for sc in scenarios:
        for ctrl in controllers:
            done += 1
            label = f"[{done:>2}/{total}]  {sc['id']:28s} | {ctrl.name()}"
            print(label, end=' ... ', flush=True)
            try:
                res = run_simulation(plant_params, sc, ctrl, log_dir)
                results.append(res)
                print(f"IAE={res['IAE']:10.1f} N*s  max_err={res['max_error']:7.0f} N")
            except Exception as exc:
                print(f"ERROR: {exc}")

    print()
    print("=" * 70)
    print(f"  Done. {len(results)}/{total} runs completed.  Logs: {log_dir}")
    print("=" * 70)
    print()
    print(f"  {'Scenario':<28} {'Controller':<16} {'IAE [N*s]':>12} {'MaxErr [N]':>10}")
    print("  " + "-" * 70)
    for r in results:
        print(f"  {r['scenario_id']:<28} {r['name']:<16} "
              f"{r['IAE']:>12.1f} {r['max_error']:>10.0f}")


if __name__ == '__main__':
    main()
