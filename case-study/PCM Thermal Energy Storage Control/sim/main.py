"""
main.py - PCM-HP thermal-energy-storage case study.

Runs all 12 controllers x 5 scenarios, writes CSV telemetry to ../logs/, prints a
summary. Auto-discovered by run.py Phase 7.

Usage (from repo root):
  conda run -n soft_robotics -- python "case-study/PCM Thermal Energy Storage Control/sim/main.py"
"""

import csv as _csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controllers import make_controllers          # noqa: E402
from simulation_runner import run_simulation       # noqa: E402


def load_json(path):
    with open(path, "r") as fh:
        return json.load(fh)


def main():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.dirname(this_dir)
    config_dir = os.path.join(base_dir, "config")
    scen_dir = os.path.join(config_dir, "scenarios")
    log_dir = os.path.join(base_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)

    plant_params = load_json(os.path.join(config_dir, "plant_params.json"))
    scen_files = sorted(f for f in os.listdir(scen_dir) if f.endswith(".json"))
    scenarios = [load_json(os.path.join(scen_dir, f)) for f in scen_files]

    controllers = make_controllers(plant_params)
    total = len(scenarios) * len(controllers)
    done = 0
    print("=" * 64)
    print("  PCM Thermal Energy Storage Control - HP compressor / SoC management")
    print(f"  {len(controllers)} controllers x {len(scenarios)} scenarios = {total} runs")
    print("=" * 64)

    for sc in scenarios:
        for c in controllers:
            done += 1
            try:
                res = run_simulation(plant_params, sc, c, log_dir)
                print(f"  [{done:>3}/{total}] {sc['id']:22s} | {c.name():14s} "
                      f"IAE={res['IAE']:8.4f}  cost={res['cost']:8.3f}")
            except Exception as exc:
                print(f"  [{done:>3}/{total}] {sc['id']:22s} | {c.name():14s} ERROR: {exc}")


if __name__ == "__main__":
    main()


# ===========================================================================
# Analysis hooks - see tools/study_protocol.py for the contract.
# ===========================================================================
import copy as _copy
import tempfile as _tempfile

_H_SIM = os.path.dirname(os.path.abspath(__file__))
_H_BASE = os.path.dirname(_H_SIM)
_H_CFG = os.path.join(_H_BASE, "config")
_H_SCEN = os.path.join(_H_CFG, "scenarios")


def _h_json(path):
    with open(path) as fh:
        return json.load(fh)


def _h_scenario(sid=None):
    fnames = sorted(f for f in os.listdir(_H_SCEN) if f.endswith(".json"))
    if not fnames:
        raise RuntimeError("No scenario JSONs in " + _H_SCEN)
    if sid is None:
        return _h_json(os.path.join(_H_SCEN, fnames[0]))
    for fn in fnames:
        sc = _h_json(os.path.join(_H_SCEN, fn))
        if sc.get("id") == sid:
            return sc
    raise ValueError(f"Scenario {sid!r} not found")


try:
    _H_NOM = _h_json(os.path.join(_H_CFG, "plant_params.json"))
    CONTROLLER_NAMES = [c.name() for c in make_controllers(_H_NOM)]
except Exception:
    _H_NOM = {}
    CONTROLLER_NAMES = []


def run_single(ctrl_name, params=None, scenario_id=None):
    _p = {**_copy.deepcopy(_H_NOM), **(params or {})}
    sc = _h_scenario(scenario_id)
    ctrls = make_controllers(_p)
    ctrl_obj = next((c for c in ctrls if c.name() == ctrl_name), None)
    if ctrl_obj is None:
        raise ValueError(f"Unknown controller {ctrl_name!r}")
    with _tempfile.TemporaryDirectory() as tmp:
        return run_simulation(_p, sc, ctrl_obj, tmp)


def run_with_fault(ctrl_name, fault, scenario_id=None):
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
    log_dir = log_dir or os.path.join(_H_BASE, "logs")
    os.makedirs(log_dir, exist_ok=True)
    fnames = sorted(f for f in os.listdir(_H_SCEN) if f.endswith(".json"))
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
        with open(out_path, "w", newline="") as fh:
            writer = _csv.DictWriter(fh, fieldnames=["controller", "step_time_us", "step_index"])
            writer.writeheader()
            writer.writerows(rows)
        out_paths.append(out_path)
    return out_paths
