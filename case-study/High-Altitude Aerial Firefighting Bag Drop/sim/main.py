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
