"""
Simulation runner for the Nonlinear Surface Ship Manoeuvring case study.
"""

import os
import csv
import math

from ship_plant import ShipPlant, make_disturbance_fn, make_ref_fn


def run_simulation(plant_params, scenario, controller, log_dir,
                   fault_injector=None):
    """
    Run one closed-loop simulation.

    Returns dict with metrics:
      name          str   controller name
      scenario_id   str   scenario id
      mean_pos_err  [m]    mean Euclidean position error
      final_pos_err [m]    position error at final time step
      IAE           [m*s]  integral of position error
      mean_u        [m/s]  mean surge speed
      max_delta_deg [deg]  peak rudder angle used
      csv           str   path to written CSV file
    Optional fault_injector: tools.fault_injector.FaultInjector instance.
      - sensor fault: biases heading (psi) measurement seen by controller
      - actuator fault: corrupts rudder command (delta_cmd)
    """
    Ts      = plant_params["integration"]["Ts"]
    t_max   = scenario["t_max"]
    N_steps = int(round(t_max / Ts))
    fi      = fault_injector

    plant = ShipPlant(plant_params)
    plant.set_disturbance_fn(make_disturbance_fn(scenario, plant_params))
    controller.reset()

    ref_fn = make_ref_fn(scenario)

    os.makedirs(log_dir, exist_ok=True)
    safe_name   = controller.name().replace(" ", "_")
    scenario_id = scenario['id']
    csv_path    = os.path.join(log_dir, f"run_{scenario_id}_{safe_name}.csv")

    IAE = 0.0
    sum_pos_err = 0.0
    sum_u       = 0.0
    max_delta   = 0.0
    final_err   = 0.0

    with open(csv_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "time", "xd", "yd", "x", "y",
            "u", "v", "r", "psi_deg",
            "psi_d_deg", "n_rps", "delta_deg",
            "pos_error", "iae_cumulative"
        ])

        for k in range(N_steps):
            t = k * Ts
            x_ref = ref_fn(t)
            xd, yd = x_ref[0], x_ref[1]

            state = plant.state()
            u, v, r, psi, x, y = state

            # Sensor fault: corrupt heading measurement seen by controller
            if fi:
                psi_obs   = fi.inject_sensor(t, psi)
                state_obs = (u, v, r, psi_obs, x, y)
            else:
                state_obs = state

            n_cmd, delta_cmd = controller.compute(x_ref, state_obs, t)
            # Actuator fault: corrupt rudder angle command
            delta_cmd = fi.inject_actuator(t, delta_cmd) if fi else delta_cmd

            n_actual, delta_actual = plant.step(n_cmd, delta_cmd, t)

            pos_err = math.sqrt((x - xd)**2 + (y - yd)**2)
            IAE += pos_err * Ts
            sum_pos_err += pos_err
            sum_u       += u
            max_delta    = max(max_delta, abs(math.degrees(delta_actual)))
            final_err    = pos_err

            xd_d, yd_d = x_ref[2], x_ref[3]
            psi_d = math.atan2(xd_d, yd_d)

            writer.writerow([
                f"{t:.4f}",
                f"{xd:.4f}", f"{yd:.4f}",
                f"{x:.4f}",  f"{y:.4f}",
                f"{u:.5f}",  f"{v:.5f}",  f"{r:.5f}",
                f"{math.degrees(psi):.4f}",
                f"{math.degrees(psi_d):.4f}",
                f"{n_actual:.5f}",
                f"{math.degrees(delta_actual):.4f}",
                f"{pos_err:.5f}",
                f"{IAE:.5f}"
            ])

    return {
        "name":          controller.name(),
        "scenario_id":   scenario_id,
        "mean_pos_err":  sum_pos_err / N_steps if N_steps > 0 else 0.0,
        "final_pos_err": final_err,
        "IAE":           IAE,
        "mean_u":        sum_u / N_steps if N_steps > 0 else 0.0,
        "max_delta_deg": max_delta,
        "csv":           csv_path,
    }
