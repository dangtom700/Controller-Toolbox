"""
simulation_runner.py
Run one (scenario, controller) pair for the EHFS case study.

CSV columns:
  t, F_ref, F, x_p, v_p, P_A_bar, P_B_bar, u_v, phase_error_deg, iae_cumulative
"""

import csv
import math
import os

from ehfs_plant import EHFSPlant


def _get_reference(scenario: dict, t: float) -> float:
    """Return force reference F_ref [N] at time t."""
    ref_type = scenario['ref_type']

    if ref_type == 'step':
        return float(scenario['F_final']) if t >= scenario['step_time'] else float(scenario['F_init'])

    if ref_type == 'sine':
        amp  = float(scenario['amplitude'])
        freq = float(scenario['frequency'])
        phi  = math.radians(float(scenario.get('phase_deg', 0.0)))
        return amp * math.sin(2.0 * math.pi * freq * t + phi)

    if ref_type == 'chirp':
        amp  = float(scenario['amplitude'])
        f0   = float(scenario['f_start'])
        f1   = float(scenario['f_end'])
        T    = float(scenario['T_sim'])
        phase = 2.0 * math.pi * (f0 * t + 0.5 * (f1 - f0) / T * t * t)
        return amp * math.sin(phase)

    return 0.0


def _get_kL(scenario: dict, t: float, k_L_nominal: float) -> float:
    """Return load stiffness [N/m], switching at k_L_time if specified."""
    if 'k_L_switch' in scenario and t >= scenario['k_L_time']:
        return float(scenario['k_L_switch'])
    return k_L_nominal


def run_simulation(plant_params: dict,
                   scenario:     dict,
                   controller,
                   log_dir:      str,
                   fault_injector=None) -> dict:
    """
    Simulate the EHFS for one scenario/controller pair.
    Returns summary: {name, scenario_id, IAE, max_error, settling_time, csv}
    Optional fault_injector: tools.fault_injector.FaultInjector instance.
    """
    # Clone params so per-scenario k_L changes don't leak
    params = dict(plant_params)
    plant  = EHFSPlant(params)
    controller.reset()
    fi = fault_injector

    Ts    = params['Ts']
    T_sim = float(scenario['T_sim'])
    N     = int(round(T_sim / Ts))

    ctrl_name   = controller.name()
    scenario_id = scenario['id']
    csv_path    = os.path.join(log_dir, f"run_{scenario_id}_{ctrl_name}.csv")

    iae          = 0.0
    max_error    = 0.0
    settle_time  = T_sim
    settled      = False
    settle_thr   = 200.0   # |error| < 200 N counts as settled

    with open(csv_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['t', 'F_ref', 'F', 'x_p', 'v_p',
                         'P_A_bar', 'P_B_bar', 'u_v',
                         'phase_error_deg', 'iae_cumulative'])

        for k in range(N):
            t     = k * Ts
            F_ref = _get_reference(scenario, t)

            # Dynamic stiffness switch (scenario s04)
            plant.k_L = _get_kL(scenario, t, params['k_L'])

            P_A, P_B, xv, vp, xp = plant.state()
            F = plant.force()
            # Sensor fault: corrupt force measurement seen by controller
            F_obs = fi.inject_sensor(t, F) if fi else F

            u_v = controller.compute(F_ref, F_obs, vp, P_A, P_B, xv, t)
            # Actuator fault: corrupt valve command
            u_v = fi.inject_actuator(t, u_v) if fi else u_v
            u_v = max(-1.0, min(1.0, u_v))

            error = F_ref - F
            iae  += abs(error) * Ts
            if abs(error) > max_error:
                max_error = abs(error)

            # Phase error (approximate atan2 of error vs. reference magnitude)
            if scenario['ref_type'] in ('sine', 'chirp') and abs(F_ref) > 1.0:
                phase_err = math.degrees(math.atan2(error, max(abs(F_ref), 1.0)))
            else:
                phase_err = 0.0

            # Settling detection (post-step transient)
            if scenario['ref_type'] == 'step' and not settled and t > scenario.get('step_time', 0.0) + 0.01:
                if abs(error) < settle_thr:
                    settled     = True
                    settle_time = t

            writer.writerow([
                f"{t:.5f}",
                f"{F_ref:.2f}",
                f"{F:.2f}",
                f"{xp*1000:.4f}",      # mm
                f"{vp*1000:.4f}",      # mm/s
                f"{P_A*1e-5:.3f}",     # bar
                f"{P_B*1e-5:.3f}",     # bar
                f"{u_v:.5f}",
                f"{phase_err:.3f}",
                f"{iae:.2f}",
            ])

            # Advance plant
            plant.step(u_v)

    return {
        'name':         ctrl_name,
        'scenario_id':  scenario_id,
        'IAE':          iae,
        'max_error':    max_error,
        'settling_time': settle_time,
        'csv':          csv_path,
    }
