"""
simulation_runner.py
Run one (scenario, controller) pair for the FOWT+WEC case study.

CSV columns:
  time, x_ref, z, zdot, x_rel, xrel_dot, F_pto, power, fowt_rms_cumul
"""

import csv
import math
import os

from wind_wave_plant import FOWTWECPlant, make_wave_fn


def _make_x_ref_fn(scenario: dict, plant_params: dict):
    """Return x_ref(t) callable - sinusoidal WEC arm reference for power extraction."""
    A_ref_scale = float(scenario.get('A_ref_scale', 1.0))
    wave_type   = scenario['wave_type']

    M_f = plant_params['M_f']
    M_w = plant_params['M_w']
    K_w = plant_params['K_w']

    if wave_type in ('regular', 'short_period'):
        T_w    = scenario['T_wave']
        H      = scenario['H_wave']
        omega_w = 2.0 * math.pi / T_w
        F_amp   = M_f * omega_w ** 2 * (H / 2.0)
        # Resonant amplitude: A_res = gamma*F_amp / |K_w - M_w*omega_w^2 + j*B_w*omega_w|
        gamma  = plant_params['gamma']
        B_w    = plant_params['B_w']
        dyn    = abs(complex(K_w - M_w * omega_w**2, B_w * omega_w))
        A_res  = gamma * F_amp / (dyn + 1.0)   # +1 avoids div/0 away from resonance
        A_ref  = A_ref_scale * A_res
        def x_ref_fn(t):
            return A_ref * math.sin(omega_w * t)
        return x_ref_fn

    elif wave_type == 'bichromatic':
        T1 = scenario['T_wave'];  H1 = scenario['H_wave']
        T2 = scenario['T_wave2']; H2 = scenario['H_wave2']
        o1 = 2.0 * math.pi / T1;  o2 = 2.0 * math.pi / T2
        F1 = M_f * o1**2 * (H1/2); F2 = M_f * o2**2 * (H2/2)
        gamma  = plant_params['gamma']
        B_w    = plant_params['B_w']
        def _A(o, F):
            dyn = abs(complex(K_w - M_w*o**2, B_w*o))
            return A_ref_scale * gamma * F / (dyn + 1.0)
        A1 = _A(o1, F1); A2 = _A(o2, F2)
        def x_ref_fn(t):
            return A1 * math.sin(o1 * t) + A2 * math.sin(o2 * t)
        return x_ref_fn

    elif wave_type == 'step_freq':
        T1  = scenario['T_wave_1']; T2 = scenario['T_wave_2']
        t_sw = scenario['step_time']
        H   = scenario['H_wave']
        o1  = 2.0 * math.pi / T1;  o2 = 2.0 * math.pi / T2
        F1  = M_f * o1**2 * (H/2); F2 = M_f * o2**2 * (H/2)
        gamma  = plant_params['gamma']
        B_w    = plant_params['B_w']
        def _A(o, F):
            dyn = abs(complex(K_w - M_w*o**2, B_w*o))
            return A_ref_scale * gamma * F / (dyn + 1.0)
        A1 = _A(o1, F1); A2 = _A(o2, F2)
        phase_sw = o1 * t_sw
        def x_ref_fn(t):
            if t < t_sw:
                return A1 * math.sin(o1 * t)
            return A2 * math.sin(o2 * (t - t_sw) + phase_sw)
        return x_ref_fn

    return lambda t: 0.0


def run_simulation(plant_params: dict,
                   scenario:     dict,
                   controller,
                   log_dir:      str,
                   fault_injector=None) -> dict:
    """
    Simulate the FOWT+WEC for one scenario/controller pair.

    Returns summary dict: {name, scenario_id, mean_power, IAE, fowt_rms, csv}
    Optional fault_injector: tools.fault_injector.FaultInjector instance.
    """
    plant = FOWTWECPlant(plant_params)
    wave_fn  = make_wave_fn(scenario, plant_params)
    x_ref_fn = _make_x_ref_fn(scenario, plant_params)
    plant.set_wave(wave_fn)
    controller.reset()
    fi = fault_injector

    Ts    = plant_params['Ts']
    T_sim = float(scenario['T_sim'])
    N     = int(round(T_sim / Ts))

    ctrl_name   = controller.name()
    scenario_id = scenario['id']
    csv_path    = os.path.join(log_dir, f"run_{scenario_id}_{ctrl_name}.csv")

    total_power = 0.0
    fowt_sq_sum = 0.0
    iae         = 0.0

    with open(csv_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['time', 'x_ref', 'z', 'zdot', 'x_rel', 'xrel_dot',
                         'F_pto', 'power', 'fowt_rms_cumul'])

        for k in range(N):
            t       = k * Ts
            x_ref   = x_ref_fn(t)
            ps      = plant.state()     # (z, zdot, x_rel, xrel_dot)

            # Sensor fault: corrupt WEC arm displacement measurement
            if fi:
                _z, _zd, _xr, _xrd = ps
                ps_obs = (_z, _zd, fi.inject_sensor(t, _xr), _xrd)
            else:
                ps_obs = ps

            F_pto = controller.compute(x_ref, ps_obs, t)
            # Actuator fault: corrupt PTO force command
            F_pto = fi.inject_actuator(t, F_pto) if fi else F_pto
            F_pto = max(-plant_params['F_max'], min(plant_params['F_max'], F_pto))

            z, zd, xr, xrd = ps  # true state for metrics
            power        = F_pto * xrd
            total_power += power * Ts
            fowt_sq_sum += z ** 2 * Ts
            iae         += abs(x_ref - xr) * Ts

            fowt_rms = math.sqrt(fowt_sq_sum / max(t + Ts, Ts))
            writer.writerow([f"{t:.2f}", f"{x_ref:.4f}",
                             f"{z:.4f}", f"{zd:.4f}",
                             f"{xr:.4f}", f"{xrd:.4f}",
                             f"{F_pto:.2f}", f"{power:.2f}",
                             f"{fowt_rms:.4f}"])

            plant.step(F_pto, t)

    mean_power = total_power / T_sim
    fowt_rms   = math.sqrt(fowt_sq_sum / T_sim)
    return {
        'name':        ctrl_name,
        'scenario_id': scenario_id,
        'mean_power':  mean_power,
        'IAE':         iae,
        'fowt_rms':    fowt_rms,
        'csv':         csv_path,
    }
