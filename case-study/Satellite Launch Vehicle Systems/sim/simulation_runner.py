"""
simulation_runner.py
Run one (scenario, controller) pair for the Satellite Launch Vehicle (SLV)
pitch-plane attitude case study.

CSV columns:
  time, theta_ref, theta, theta_dot, delta, alpha_w, error, iae_cumulative

The commanded gimbal is rate-limited (delta_rate_max) and position-limited
(delta_max, enforced inside the plant) - a reduction of the paper's second-order
actuator with acceleration/slew/position limits (Sec. 4.1 assumption 2).

Metrics returned:
  IAE   - integral of |error| dt (primary; drives the tracker semaphore)
  ISE   - integral of error^2 dt (paper's "integral absolute square error")
  max_error         - peak |theta_ref - theta| [rad]
  max_control_demand- peak |delta| [rad]
"""

import csv
import os
import time

from satellite_launch_vehicle_systems_plant import SLVPlant, make_wind_fn


def _get_reference(scenario: dict, t: float) -> float:
    """Guidance attitude command theta_ref(t) [rad]."""
    rt = scenario.get("ref_type", "step")

    if rt == "step":
        return float(scenario["ref_final"]) if t >= scenario.get("step_time", 1.0) \
            else float(scenario.get("ref_init", 0.0))

    if rt == "ramp":
        t0 = float(scenario.get("ramp_start", 1.0))
        dur = float(scenario.get("ramp_duration", 60.0))
        r0 = float(scenario.get("ref_init", 0.0))
        rf = float(scenario.get("ref_final", 0.15))
        if t < t0:
            return r0
        if t >= t0 + dur:
            return rf
        return r0 + (rf - r0) * (t - t0) / dur

    if rt == "guidance":
        # Piecewise-constant guidance program: list of {t_start, value}.
        segs = scenario["segments"]
        val = segs[0]["value"]
        for s in segs:
            if t >= s["t_start"]:
                val = s["value"]
            else:
                break
        return float(val)

    return float(scenario.get("ref_final", 0.0))


def run_simulation(plant_params: dict,
                   scenario: dict,
                   controller,
                   log_dir: str,
                   fault_injector=None,
                   wcet_sink: list = None) -> dict:
    # A scenario may override plant parameters (e.g. a perturbed transonic
    # aerodynamic profile for the robustness scenario, paper Sec. 4.2.1).
    run_params = {**plant_params, **scenario.get("plant_overrides", {})}
    plant = SLVPlant(run_params)
    plant.set_wind_fn(make_wind_fn(scenario))
    controller.reset()
    fi = fault_injector

    Ts = run_params["Ts"]
    T_sim = float(scenario["T_sim"])
    N = int(round(T_sim / Ts))
    delta_max = run_params["delta_max"]
    delta_rate_max = run_params.get("delta_rate_max", 8.0)   # rad/s slew limit

    ctrl_name = controller.name()
    scenario_id = scenario["id"]
    csv_path = os.path.join(log_dir, f"run_{scenario_id}_{ctrl_name}.csv")

    iae = 0.0
    ise = 0.0
    max_error = 0.0
    max_ctrl = 0.0
    delta_prev = 0.0

    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["time", "theta_ref", "theta", "theta_dot", "delta",
                    "alpha_w", "error", "iae_cumulative"])

        for k in range(N):
            t = k * Ts
            ref = _get_reference(scenario, t)
            theta, theta_dot = plant.state()

            # Sensor fault corrupts the measured attitude/rate seen by the controller.
            theta_obs = fi.inject_sensor(t, theta) if fi else theta
            rate_obs = theta_dot

            if wcet_sink is not None:
                t0 = time.perf_counter()
                delta_cmd = controller.compute(ref, theta_obs, rate_obs, t)
                wcet_sink.append({"controller": ctrl_name,
                                  "step_time_us": (time.perf_counter() - t0) * 1e6,
                                  "step_index": k})
            else:
                delta_cmd = controller.compute(ref, theta_obs, rate_obs, t)

            # Actuator fault, then slew-rate limit, then position limit.
            if fi:
                delta_cmd = fi.inject_actuator(t, delta_cmd)
            dmax = delta_rate_max * Ts
            delta = max(delta_prev - dmax, min(delta_prev + dmax, delta_cmd))
            delta = max(-delta_max, min(delta_max, delta))
            delta_prev = delta

            error = ref - theta          # true state for metrics
            iae += abs(error) * Ts
            ise += error * error * Ts
            max_error = max(max_error, abs(error))
            max_ctrl = max(max_ctrl, abs(delta))
            alpha_w = plant._wind_fn(t)

            w.writerow([f"{t:.3f}", f"{ref:.5f}", f"{theta:.5f}",
                        f"{theta_dot:.5f}", f"{delta:.5f}", f"{alpha_w:.5f}",
                        f"{error:.5f}", f"{iae:.5f}"])

            plant.step(delta, t)

    return {
        "name": ctrl_name,
        "scenario_id": scenario_id,
        "IAE": iae,
        "ISE": ise,
        "max_error": max_error,
        "max_control_demand": max_ctrl,
        "csv": csv_path,
    }
