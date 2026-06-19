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
import csv as _csv

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


# ===========================================================================
# Analysis hooks - for tools/monte_carlo.py and tools/fault_sweep.py
# See tools/study_protocol.py for the full contract.
# Note: plant_params has a nested structure; mc_perturb_keys in analysis.json
#       should target identified_params sub-keys (a1-a6, b1-b7, c1-c6).
#       run_single accepts flat {key: value} for identified_params keys.
# ===========================================================================
import copy as _copy
import tempfile as _tempfile

_H_SIM  = _THIS
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


def _h_merge_params(base, override):
    """Merge override into base; identified_params sub-keys are handled flat."""
    if not override:
        return base
    result = _copy.deepcopy(base)
    id_keys = set(result.get('identified_params', {}).keys())
    for k, v in override.items():
        if k in id_keys:
            result['identified_params'][k] = v
        else:
            result[k] = v
    return result


try:
    _H_NOM = _h_nom()
    CONTROLLER_NAMES = [c.name() for c in make_controllers(_H_NOM)]
except Exception:
    _H_NOM = {}
    CONTROLLER_NAMES = []


def run_single(ctrl_name, params=None, scenario_id=None):
    """Run ctrl_name with optional perturbed params; returns metrics dict.

    params may be a flat dict of identified_params keys (a1, b2, etc.)
    or top-level keys; both are handled by _h_merge_params.
    """
    _p = _h_merge_params(_H_NOM, params)
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


def run_wcet_profile(log_dir=None):
    """Time controller.compute() per step for every controller, across every
    scenario (not just the nominal one).

    Writes one logs/wcet_{scenario_id}.csv (controller, step_time_us,
    step_index) per scenario for tools/wcet_report.py.
    """
    log_dir = log_dir or os.path.join(_H_BASE, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    fnames = sorted(f for f in os.listdir(_H_SCEN) if f.endswith('.json'))
    out_paths = []
    for fn in fnames:
        sc = _h_json(os.path.join(_H_SCEN, fn))
        rows = []
        for ctrl_name in CONTROLLER_NAMES:
            ctrls = make_controllers(_H_NOM)
            ctrl_obj = next((c for c in ctrls if c.name() == ctrl_name), None)
            if ctrl_obj is None:
                continue
            with _tempfile.TemporaryDirectory() as tmp:
                run_simulation(_H_NOM, sc, ctrl_obj, tmp, wcet_sink=rows)

        out_path = os.path.join(log_dir, f"wcet_{sc['id']}.csv")
        with open(out_path, 'w', newline='') as fh:
            writer = _csv.DictWriter(fh, fieldnames=["controller", "step_time_us", "step_index"])
            writer.writeheader()
            writer.writerows(rows)
        out_paths.append(out_path)
    return out_paths
