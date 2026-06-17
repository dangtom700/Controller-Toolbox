"""
simulation_runner.py
Runs a single (controller, scenario) pair for the 6x6 EV Active Suspension study.

Returns a dict with IAE, RMS metrics, and peak forces.
Writes one CSV file to log_dir.
"""

import csv
import os
import math
import numpy as np

from ev6x6_plant import EV6x6Plant, N_DOF
from road_model  import RoadProfile
from controllers import ControllerBase


def run_simulation(plant_params: dict,
                   scenario:      dict,
                   controller:    ControllerBase,
                   log_dir:       str,
                   fault_injector=None) -> dict:
    """
    Simulate one (controller, scenario) pair.

    Returns
    -------
    dict with keys: name, scenario_id, IAE, RMS_body, RMS_head, max_body,
                    peak_force, csv.
    Optional fault_injector: tools.fault_injector.FaultInjector instance.
      - sensor fault: biases body vertical displacement (state[0])
      - actuator fault: scales all 6 actuator forces uniformly
    """
    Ts        = plant_params['Ts']
    T_sim     = float(scenario.get('T_sim', 5.0))
    N_steps   = int(round(T_sim / Ts))
    v_vehicle = float(plant_params.get('v_vehicle', 22.2))
    L1        = float(plant_params.get('L1', 2.0))
    L2        = float(plant_params.get('L2', 2.0))
    fi        = fault_injector

    plant = EV6x6Plant(plant_params)
    plant.reset()

    road = RoadProfile(scenario, Ts, v_vehicle=v_vehicle, L1=L1, L2=L2)
    road.reset()

    controller.reset()

    scen_id   = scenario.get('id', 'unknown')
    ctrl_name = controller.name()
    csv_name  = f"run_{scen_id}_{ctrl_name}.csv"
    csv_path  = os.path.join(log_dir, csv_name)

    iae_cum  = 0.0
    body_sq  = 0.0
    head_sq  = 0.0
    max_body = 0.0
    peak_f   = 0.0

    rows = []

    for k in range(N_steps):
        t   = k * Ts
        z_r = road.step()
        x   = plant.state()

        # Sensor fault: bias body vertical displacement seen by controller
        if fi:
            x_obs = np.array(x, dtype=float)
            x_obs[0] = fi.inject_sensor(t, x_obs[0])
        else:
            x_obs = x

        F_act = controller.compute(x_obs, z_r, t)

        # Actuator fault: scale all forces uniformly
        if fi:
            F_act = np.array([fi.inject_actuator(t, f) for f in F_act],
                             dtype=float)

        body_Z      = plant.body_Z()
        head_accel  = plant.head_accel()
        seat_accel  = plant.seat_accel()
        body_accel  = plant.body_accel()

        iae_cum  += abs(body_Z) * Ts
        body_sq  += body_Z ** 2
        head_sq  += head_accel ** 2
        max_body  = max(max_body, abs(body_Z))
        peak_f    = max(peak_f,   np.max(np.abs(F_act)))

        row = [f"{t:.5f}", f"{body_Z:.6f}", f"{x[19]:.6f}",
               f"{x[1]:.6f}", f"{x[2]:.6f}"]
        # Z_wr1-3
        row += [f"{x[3+i]:.6f}" for i in range(3)]
        # Z_wl1-3
        row += [f"{x[6+i]:.6f}" for i in range(3)]
        # F_act_1-6
        row += [f"{F_act[i]:.2f}" for i in range(6)]
        # z_r_1-6
        row += [f"{z_r[i]:.6f}" for i in range(6)]
        row += [f"{seat_accel:.4f}", f"{head_accel:.4f}", f"{body_accel:.4f}",
                f"{iae_cum:.6f}"]
        rows.append(row)

        plant.step(F_act, z_r)

    rms_body = math.sqrt(body_sq / max(N_steps, 1))
    rms_head = math.sqrt(head_sq  / max(N_steps, 1))

    header = (['t', 'Z_body', 'Z_head', 'theta', 'phi']
              + [f'Z_wr{i+1}' for i in range(3)]
              + [f'Z_wl{i+1}' for i in range(3)]
              + [f'F_act_{i+1}' for i in range(6)]
              + [f'z_r_{i+1}'  for i in range(6)]
              + ['seat_accel', 'head_accel', 'body_accel', 'iae_cumulative'])

    with open(csv_path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)

    return {
        'name':        ctrl_name,
        'scenario_id': scen_id,
        'IAE':         iae_cum,
        'RMS_body':    rms_body,
        'RMS_head':    rms_head,
        'max_body':    max_body,
        'peak_force':  peak_f,
        'csv':         csv_path,
    }
