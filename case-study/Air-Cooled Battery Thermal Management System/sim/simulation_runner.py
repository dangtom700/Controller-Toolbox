"""
simulation_runner.py
Run one (scenario, controller) pair for the Air-Cooled BTMS case study.

CSV columns:
  t, DeltaT_ref, DeltaT, T_max, T_min, T_avg, flow_pattern, u_ctrl,
  phi_b_kWm3, soc, n_switches, iae_cumulative

u_ctrl is the controller's continuous control signal (threshold adjustment in
[-0.5, 0.5] K for the ctrl_toolbox controllers, flow-pattern encoding for the
rule-based ones). It is the continuous actuator counterpart to the discrete
flow_pattern, and the (DeltaT, u_ctrl) pair is what tools/mu_analysis.py fits.
"""

import csv
import math
import os
import random
import time

from btms_plant import BTMSPlant


# ---------------------------------------------------------------------------
# Heat generation profiles
# ---------------------------------------------------------------------------
def _phi_profile(scenario: dict, t: float, plant: BTMSPlant) -> float:
    """Return volumetric heat generation rate [W/m^3] at time t."""
    profile = scenario.get('phi_profile', 'constant_c_rate')

    if profile == 'constant_c_rate':
        c_rate = float(scenario.get('c_rate', 5.0))
        return plant.heat_gen(t, c_rate)

    if profile == 'varying_random':
        # Pseudo-random between phi_lo and phi_hi, seed from t
        phi_lo = float(scenario.get('phi_lo', 5e4))
        phi_hi = float(scenario.get('phi_hi', 8e4))
        # Slowly varying: change every 60 s
        period = float(scenario.get('phi_period', 60.0))
        rng = random.Random(int(t / period) + scenario.get('rng_seed', 7))
        return rng.uniform(phi_lo, phi_hi)

    if profile == 'pulse_then_low':
        t_switch = float(scenario.get('t_switch', 300.0))
        c_hi = float(scenario.get('c_rate_hi', 7.0))
        c_lo = float(scenario.get('c_rate_lo', 2.0))
        c    = c_hi if t < t_switch else c_lo
        return plant.heat_gen(t, c)

    if profile == 'aged_battery':
        # Same C-rate but resistance is R_factor * nominal
        c_rate = float(scenario.get('c_rate', 5.0))
        r_factor = float(scenario.get('R_factor', 1.5))
        soc  = max(0.0, 1.0 - c_rate * t / 3600.0)
        from btms_plant import _resistance, DUE_DT, V_BAT, I_1C
        R    = _resistance(soc) * r_factor
        I    = c_rate * I_1C
        T_b  = sum(plant.temperatures()) / len(plant.temperatures())
        phi  = (I * I * R - I * T_b * DUE_DT) / V_BAT
        return max(phi, 0.0)

    # Fallback: constant moderate rate
    return plant.heat_gen(t, 3.0)


def _soc_from_scenario(scenario: dict, t: float) -> float:
    """Approximate SOC for gain-scheduling purposes."""
    profile = scenario.get('phi_profile', 'constant_c_rate')
    if profile == 'pulse_then_low':
        t_switch = float(scenario.get('t_switch', 300.0))
        c_hi = float(scenario.get('c_rate_hi', 7.0))
        c_lo = float(scenario.get('c_rate_lo', 2.0))
        if t < t_switch:
            return max(0.0, 1.0 - c_hi * t / 3600.0)
        else:
            soc_switch = max(0.0, 1.0 - c_hi * t_switch / 3600.0)
            return max(0.0, soc_switch - c_lo * (t - t_switch) / 3600.0)
    c_rate = float(scenario.get('c_rate', 5.0))
    return max(0.0, 1.0 - c_rate * t / 3600.0)


# ---------------------------------------------------------------------------
# run_simulation
# ---------------------------------------------------------------------------
def run_simulation(plant_params: dict,
                   scenario:     dict,
                   controller,
                   log_dir:      str,
                   fault_injector=None,
                   wcet_sink:    list = None) -> dict:
    """
    Simulate one (scenario, controller) pair.
    Returns summary dict: {name, scenario_id, IAE, max_delta_T, avg_delta_T,
                           T_max_final, n_switches, csv}
    Optional fault_injector: tools.fault_injector.FaultInjector instance.
      - sensor fault: biases DeltaT reading seen by controller
      - actuator fault: not applied (output is discrete flow pattern string)
    Optional wcet_sink: if a list is passed, one dict per step
      {"controller": name, "step_time_us": float, "step_index": k} timing
      controller.compute() is appended to it (see tools/wcet_report.py).
    """
    plant = BTMSPlant(plant_params)
    plant.set_pattern('J')        # always start with J-type (lowest DeltaT at 5C, per paper)
    controller.reset()
    fi = fault_injector

    # Inject plant reference for MPC look-ahead
    if hasattr(controller, 'set_plant_ref'):
        controller.set_plant_ref(plant)

    Ts    = float(plant_params.get('Ts', 1.0))
    T_sim = float(scenario.get('T_sim', 720.0))
    N     = int(round(T_sim / Ts))

    ctrl_name   = controller.name()
    scenario_id = scenario['id']
    csv_path    = os.path.join(log_dir, f"run_{scenario_id}_{ctrl_name}.csv")

    iae        = 0.0
    sum_dt     = 0.0
    max_dt     = 0.0
    DT_REF     = 0.0   # ideal: zero temperature difference

    with open(csv_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow([
            't', 'DeltaT_ref', 'DeltaT', 'T_max', 'T_min', 'T_avg',
            'flow_pattern', 'u_ctrl', 'phi_b_kWm3', 'soc', 'n_switches',
            'iae_cumulative'
        ])

        for k in range(N):
            t   = k * Ts
            soc = _soc_from_scenario(scenario, t)
            phi = _phi_profile(scenario, t, plant)

            # Cache phi for MPC prediction
            plant._phi_cache = phi

            DeltaT    = plant.delta_T()
            T_max     = plant.T_max()
            T_min     = plant.T_min()
            T_avg_val = sum(plant.temperatures()) / 9
            x_hot     = plant.x_hotspot_norm()

            # Sensor fault: corrupt temperature non-uniformity reading
            DeltaT_obs = fi.inject_sensor(t, DeltaT) if fi else DeltaT

            # Controller decision (actuator output is discrete - no actuator fault)
            if wcet_sink is not None:
                t0 = time.perf_counter()
                new_pattern = controller.compute(DeltaT_obs, x_hot, soc, t)
                dt_us = (time.perf_counter() - t0) * 1e6
                wcet_sink.append({"controller": ctrl_name, "step_time_us": dt_us, "step_index": k})
            else:
                new_pattern = controller.compute(DeltaT_obs, x_hot, soc, t)
            plant.set_pattern(new_pattern)
            u_ctrl = controller.last_u()

            # Metrics
            iae    += abs(DeltaT - DT_REF) * Ts
            sum_dt += DeltaT
            if DeltaT > max_dt:
                max_dt = DeltaT

            writer.writerow([
                f"{t:.1f}",
                f"{DT_REF:.3f}",
                f"{DeltaT:.4f}",
                f"{T_max:.4f}",
                f"{T_min:.4f}",
                f"{T_avg_val:.4f}",
                new_pattern,
                f"{u_ctrl:.5f}",
                f"{phi / 1000.0:.2f}",    # kW/m^3
                f"{soc:.4f}",
                str(plant.n_switches()),
                f"{iae:.2f}",
            ])

            # Advance plant
            plant.step(phi, Ts)

    avg_dt = sum_dt / max(N, 1)

    return {
        'name':        ctrl_name,
        'scenario_id': scenario_id,
        'IAE':         iae,
        'max_delta_T': max_dt,
        'avg_delta_T': avg_dt,
        'T_max_final': plant.T_max(),
        'n_switches':  plant.n_switches(),
        'csv':         csv_path,
    }
