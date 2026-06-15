"""
Nonlinear Surface Ship Manoeuvring Control - case study main script.

Usage:
  conda run -n soft_robotics -- python "case-study/Nonlinear Surface Ship Manoeuvring Control/sim/main.py"

Plant  : 3-DOF MMG model, MARIN free-running ship (Meng et al., 2025)
State  : [u, v, r, psi, x, y]
Input  : [n_rps (propeller rev/s), delta_rad (rudder angle)]
Source : Meng et al., Ocean Engineering 321 (2025) 120432
"""

import os
import sys
import json
import glob

_THIS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))
sys.path.insert(0, _ROOT)
import _setup_bindings  # noqa: F401

sys.path.insert(0, _THIS)

from controllers import make_controllers
from simulation_runner import run_simulation


def _load_json(path):
    with open(path, "r") as fh:
        return json.load(fh)


def main():
    cfg_dir      = os.path.join(_THIS, "..", "config")
    scenarios_dir = os.path.join(cfg_dir, "scenarios")
    log_dir       = os.path.join(_THIS, "..", "logs")

    plant_params = _load_json(os.path.join(cfg_dir, "plant_params.json"))

    scenario_files = sorted(glob.glob(os.path.join(scenarios_dir, "*.json")))
    scenarios = [_load_json(f) for f in scenario_files]
    controllers = make_controllers(plant_params)

    print(f"Ship Manoeuvring Case Study  |  {len(controllers)} controllers x {len(scenarios)} scenarios = {len(controllers)*len(scenarios)} runs")
    print(f"Plant: 3-DOF MMG, MARIN free-running model, Ts={plant_params['integration']['Ts']} s")
    print()

    # Header
    col_w = 14
    hdr = (f"{'Scenario':<18} {'Controller':<18}"
           f" {'MeanErr[m]':>{col_w}} {'FinalErr[m]':>{col_w}}"
           f" {'IAE[m*s]':>{col_w}} {'MeanU[m/s]':>{col_w}}"
           f" {'MaxDelta[deg]':>{col_w}}")
    print(hdr)
    print("-" * len(hdr))

    for scenario in scenarios:
        for controller in controllers:
            try:
                metrics = run_simulation(plant_params, scenario, controller, log_dir)
                print(f"{scenario['id'] + ' ' + scenario['name'][:12]:<18} {controller.name():<18}"
                      f" {metrics['mean_pos_err']:>{col_w}.3f}"
                      f" {metrics['final_pos_err']:>{col_w}.3f}"
                      f" {metrics['IAE']:>{col_w}.1f}"
                      f" {metrics['mean_u']:>{col_w}.3f}"
                      f" {metrics['max_delta_deg']:>{col_w}.2f}")
            except Exception as exc:
                print(f"{scenario['id']:<18} {controller.name():<18}  ERROR: {exc}")

    print()
    print(f"CSV logs written to: {os.path.abspath(log_dir)}")


if __name__ == "__main__":
    main()
