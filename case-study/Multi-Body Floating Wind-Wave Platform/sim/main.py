"""
main.py
Multi-Body Floating Wind-Wave Platform + WEC Case Study - entry point.

Runs all 16 controllers x 5 scenarios = 80 simulations.
Writes CSV telemetry to ../logs/ and prints a summary table.

Usage (from project root):
  conda run -n soft_robotics -- python "case-study/Multi-Body Floating Wind-Wave Platform/sim/main.py"
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

    print("=" * 68)
    print("  Multi-Body Floating Wind-Wave Platform + WEC Case Study")
    print(f"  {len(controllers)} controllers x {len(scenarios)} scenarios = {total} runs")
    print("=" * 68)

    results = []
    for sc in scenarios:
        for ctrl in controllers:
            done += 1
            label = f"[{done:>2}/{total}]  {sc['id']:30s} | {ctrl.name()}"
            print(label, end=' ... ', flush=True)
            try:
                res = run_simulation(plant_params, sc, ctrl, log_dir)
                results.append(res)
                print(f"P_mean={res['mean_power']:8.1f} W  "
                      f"FOWT_rms={res['fowt_rms']:.3f} m")
            except Exception as exc:
                print(f"ERROR: {exc}")

    print()
    print("=" * 68)
    print(f"  Done. {len(results)}/{total} runs completed. Logs: {log_dir}")
    print("=" * 68)
    print()
    print(f"  {'Scenario':<30} {'Controller':<14} {'P_mean[W]':>10} "
          f"{'FOWT_rms[m]':>12} {'IAE':>10}")
    print("  " + "-" * 80)
    for r in results:
        print(f"  {r['scenario_id']:<30} {r['name']:<14} "
              f"{r['mean_power']:>10.1f} {r['fowt_rms']:>12.4f} "
              f"{r['IAE']:>10.2f}")


if __name__ == '__main__':
    main()
