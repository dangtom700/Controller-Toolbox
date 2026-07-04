"""main.py - entry point for Satellite Launch Vehicle Systems (TEMPLATE).

Runs every controller x scenario, writes CSV to ../logs/, prints a summary.
Auto-discovered by run.py Phase 7 (no CMake/compile registration needed).

Usage (from repo root):
  conda run -n soft_robotics -- python "case-study/Satellite Launch Vehicle Systems/sim/main.py"
"""
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
    base_dir = os.path.dirname(this_dir)                  # StudyName/
    config_dir = os.path.join(base_dir, "config")
    scen_dir = os.path.join(config_dir, "scenarios")
    log_dir = os.path.join(base_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)

    from satellite_launch_vehicle_systems_plant import Plant                      # noqa: E402
    plant_params = load_json(os.path.join(config_dir, "plant_params.json"))

    scenarios = []
    for fn in sorted(os.listdir(scen_dir)):
        if fn.endswith(".json"):
            scenarios.append(load_json(os.path.join(scen_dir, fn)))

    controllers = make_controllers(plant_params.get("Ts", 0.01))
    print("Satellite Launch Vehicle Systems: %d controllers x %d scenarios" % (len(controllers), len(scenarios)))

    for scen in scenarios:
        for name, c in controllers:
            plant = Plant(plant_params)
            iae = run_simulation(plant, scen, name, c, log_dir)
            print("  %s / %s  IAE=%.4f" % (scen["id"], name, iae))


if __name__ == "__main__":
    main()
