"""
simulation_runner.py
Run one (scenario, controller) pair for the PCM-HP thermal-energy-storage case
study.

CSV columns:
  time, soc_ref, soc, u, r, T_o, P_load, price, Q_hp, e_hp,
  cost_cumulative, error, iae_cumulative

Metrics returned:
  IAE   - integral of |soc_ref - soc| dt (primary; drives the tracker semaphore)
  cost  - integral of price * e_hp dt [normalised electricity cost]
  max_error - peak |soc_ref - soc|
"""

import csv
import os
import time

from pcm_thermal_energy_storage_control_plant import (PCMPlant, make_profiles,
                                                      hp_cooling, hp_electricity,
                                                      R_MAX)


def run_simulation(plant_params: dict,
                   scenario: dict,
                   controller,
                   log_dir: str,
                   fault_injector=None,
                   wcet_sink: list = None) -> dict:
    plant = PCMPlant(plant_params)
    T_o_fn, P_load_fn, price_fn, soc_ref_fn = make_profiles(scenario)
    controller.reset()
    fi = fault_injector

    Ts = plant_params["Ts"]
    T_sim = float(scenario["T_sim"])
    N = int(round(T_sim / Ts))

    ctrl_name = controller.name()
    scenario_id = scenario["id"]
    csv_path = os.path.join(log_dir, f"run_{scenario_id}_{ctrl_name}.csv")

    iae = 0.0
    cost = 0.0
    max_error = 0.0

    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["time", "soc_ref", "soc", "u", "r", "T_o", "P_load", "price",
                    "Q_hp", "e_hp", "cost_cumulative", "error", "iae_cumulative"])

        for k in range(N):
            t = k * Ts
            T_o = T_o_fn(t)
            P_load = P_load_fn(t)
            price = price_fn(t)
            ref = soc_ref_fn(t)

            soc = plant.state()
            soc_obs = fi.inject_sensor(t, soc) if fi else soc

            if wcet_sink is not None:
                t0 = time.perf_counter()
                u = controller.compute(ref, soc_obs, T_o, P_load, t)
                wcet_sink.append({"controller": ctrl_name,
                                  "step_time_us": (time.perf_counter() - t0) * 1e6,
                                  "step_index": k})
            else:
                u = controller.compute(ref, soc_obs, T_o, P_load, t)

            if fi:
                u = fi.inject_actuator(t, u)
            u = max(0.0, min(1.0, u))

            r = R_MAX * u
            Q_hp = hp_cooling(r, T_o)
            e_hp = hp_electricity(r, T_o)

            error = ref - soc          # true state for metrics
            iae += abs(error) * Ts
            cost += price * e_hp * Ts
            max_error = max(max_error, abs(error))

            w.writerow([f"{t:.2f}", f"{ref:.4f}", f"{soc:.4f}", f"{u:.4f}",
                        f"{r:.1f}", f"{T_o:.2f}", f"{P_load:.3f}", f"{price:.3f}",
                        f"{Q_hp:.3f}", f"{e_hp:.3f}", f"{cost:.3f}",
                        f"{error:.4f}", f"{iae:.4f}"])

            plant.step(u, T_o, P_load)

    return {
        "name": ctrl_name,
        "scenario_id": scenario_id,
        "IAE": iae,
        "cost": cost,
        "max_error": max_error,
        "csv": csv_path,
    }
