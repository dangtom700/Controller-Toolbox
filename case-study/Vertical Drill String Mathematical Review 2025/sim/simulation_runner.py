"""
simulation_runner.py
Run one (scenario, controller) pair and write CSV telemetry.

CSV columns:
  time, omega_ref, omega_b, phi, omega_t, error, iae_cumulative
"""

import csv
import math
import os

from drill_string_plant import DrillStringPlant


def _get_reference(scenario: dict, t: float) -> float:
    """Return the reference bit speed at time t [rad/s]."""
    ref_type = scenario['ref_type']

    if ref_type == 'step':
        if t < scenario['step_time']:
            return float(scenario['omega_init'])
        return float(scenario['omega_final'])

    elif ref_type == 'ramp':
        t_start = float(scenario['ramp_start'])
        t_end   = t_start + float(scenario['ramp_duration'])
        w0 = float(scenario['omega_init'])
        wf = float(scenario['omega_final'])
        if t < t_start:
            return w0
        if t >= t_end:
            return wf
        alpha = (t - t_start) / (t_end - t_start)
        return w0 + alpha * (wf - w0)

    elif ref_type == 'two_step':
        if t < scenario['step_time1']:
            return 0.0
        if t < scenario['step_time2']:
            return float(scenario['omega_first'])
        return float(scenario['omega_second'])

    # Fallback: constant final value
    return float(scenario.get('omega_final', 0.0))


def run_simulation(plant_params: dict,
                   scenario:     dict,
                   controller,
                   log_dir:      str) -> dict:
    """
    Simulate the drill string for one scenario/controller pair.

    Returns a summary dict: {name, scenario_id, IAE, settling_time, max_error}
    """
    plant = DrillStringPlant(plant_params)
    controller.reset()

    Ts    = plant_params['Ts']
    T_sim = float(scenario['T_sim'])
    N     = int(round(T_sim / Ts))

    ctrl_name   = controller.name()
    scenario_id = scenario['id']
    csv_path    = os.path.join(log_dir,
                               f"run_{scenario_id}_{ctrl_name}.csv")

    iae         = 0.0
    max_error   = 0.0
    settle_time = T_sim  # default: never settled
    settled     = False
    settle_thr  = 0.5   # |error| < 0.5 rad/s for settling

    with open(csv_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['time', 'omega_ref', 'omega_b', 'phi', 'omega_t',
                         'error', 'iae_cumulative'])

        for k in range(N):
            t         = k * Ts
            omega_ref = _get_reference(scenario, t)

            # Clamp to safe range
            omega_t = controller.compute(omega_ref, plant.omega_b, plant.phi, t)
            omega_t = max(-plant_params['omega_max'],
                          min(plant_params['omega_max'], omega_t))

            phi, omega_b = plant.state()
            error = omega_ref - omega_b
            iae  += abs(error) * Ts
            max_error = max(max_error, abs(error))

            # Track settling: sustained |error| < threshold after step
            if not settled and t > 10.0 and abs(error) < settle_thr:
                settled     = True
                settle_time = t

            writer.writerow([f"{t:.3f}", f"{omega_ref:.4f}",
                             f"{omega_b:.4f}", f"{phi:.4f}",
                             f"{omega_t:.4f}", f"{error:.4f}",
                             f"{iae:.4f}"])

            # Advance plant
            plant.step(omega_t)

    return {
        'name':         ctrl_name,
        'scenario_id':  scenario_id,
        'IAE':          iae,
        'settling_time': settle_time,
        'max_error':    max_error,
        'csv':          csv_path,
    }
