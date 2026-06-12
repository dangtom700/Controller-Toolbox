"""
main.py
Air-Cooled Battery Thermal Management System Case Study - entry point.

Runs 12 controllers x 5 scenarios = 60 simulations.
Primary metric: IAE of DeltaT (integral of temperature non-uniformity [K*s]).
Writes CSV telemetry to ../logs/ and prints a summary table.

Usage (from project root):
  conda run -n soft_robotics -- python "case-study/Air-Cooled Battery Thermal Management System/sim/main.py"

Reference:
  Zhang J., Zhang Z., Wu X., Song M., Chen K. (2026). Operation optimization of
  battery thermal management systems based on transient heat transfer model and
  self-adaptive control strategy. Applied Thermal Engineering, 298, 130921.
  https://doi.org/10.1016/j.applthermaleng.2026.130921
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

    print("=" * 74)
    print("  Air-Cooled Battery Thermal Management System Case Study")
    print(f"  {len(controllers)} controllers x {len(scenarios)} scenarios = {total} runs")
    print("  Primary metric: IAE of DeltaT  [K*s]  (temperature non-uniformity)")
    print("=" * 74)

    results = []
    for sc in scenarios:
        for ctr in controllers:
            done += 1
            label = f"[{done:>2}/{total}]  {sc['id']:32s} | {ctr.name()}"
            print(label, end=' ... ', flush=True)
            try:
                res = run_simulation(plant_params, sc, ctr, log_dir)
                results.append(res)
                print(f"IAE={res['IAE']:8.1f} K*s  "
                      f"maxDT={res['max_delta_T']:5.2f} K  "
                      f"sw={res['n_switches']:3d}")
            except Exception as exc:
                import traceback
                print(f"ERROR: {exc}")
                traceback.print_exc()

    print()
    print("=" * 74)
    print(f"  Done. {len(results)}/{total} runs completed.  Logs: {log_dir}")
    print("=" * 74)
    print()
    print(f"  {'Scenario':<32} {'Controller':<14} {'IAE[K*s]':>9} "
          f"{'maxDT[K]':>9} {'avgDT[K]':>9} {'sw':>4}")
    print("  " + "-" * 80)
    for r in results:
        print(f"  {r['scenario_id']:<32} {r['name']:<14} "
              f"{r['IAE']:>9.1f} "
              f"{r['max_delta_T']:>9.3f} "
              f"{r['avg_delta_T']:>9.3f} "
              f"{r['n_switches']:>4d}")


if __name__ == '__main__':
    main()
