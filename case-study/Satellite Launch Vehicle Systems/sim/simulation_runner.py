"""simulation_runner.py - one (controller, scenario) run for Satellite Launch Vehicle Systems (TEMPLATE)."""
import csv
import os


def run_simulation(plant, scenario: dict, name: str, controller, log_dir: str) -> float:
    """Run one simulation, write CSV telemetry, return final cumulative IAE."""
    Ts = plant.Ts
    T_sim = scenario.get("T_sim", 30.0)
    step_time = scenario.get("step_time", 1.0)
    ref_init = scenario.get("ref_init", 0.0)
    ref_final = scenario.get("ref_final", 1.0)

    plant.reset()
    if hasattr(controller, "reset"):
        controller.reset()

    os.makedirs(log_dir, exist_ok=True)
    csv_path = os.path.join(log_dir, "run_%s_%s.csv" % (scenario["id"], name))
    n = int(T_sim / Ts)
    iae = 0.0
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "ref", "y", "u", "iae_cumulative"])
        for k in range(n):
            t = k * Ts
            ref = ref_final if t >= step_time else ref_init
            y = plant.output()
            e = ref - y
            u = controller.compute(e)        # TODO: sign convention
            plant.step(u)
            iae += abs(e) * Ts
            w.writerow([t, ref, y, u, iae])
    return iae
