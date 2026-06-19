"""
simulation_runner.py
Run one (scenario, planner) pair for the aerial firefighting bag drop study.

Each run simulates N_mc independent bag trajectories with random wind
perturbations drawn from the scenario's wind statistics.  Impact points
are collected and accuracy metrics are computed.

CSV columns (one row per Monte Carlo sample):
  sample_id, x_impact, y_impact, t_flight,
  wx_true, wy_true, error_x, error_y, radial_error, iae

iae is CEP repeated on every row (this run's primary metric, constant across
trials) so shared tooling (tools/metrics.py::extract_final_iae) recognises a
per-(scenario, controller) IAE-equivalent value for this study, alongside the
trailing '# CEP' summary row below which carries the same value in that slot.

Metrics returned:
  IAE          -- CEP [m] (50th-percentile radial error, used as primary metric)
  max_error    -- 95th-percentile radial error [m]
  pattern_length -- 80th-20th percentile of downrange errors [m]
  pattern_width  -- 95th percentile of |lateral error| [m]
"""

import csv
import math
import os
import random
import time

from bag_drop_plant import BagDropPlant
from planners import IterativeRefinementDrop, AdaptiveRLSDrop


def run_simulation(plant_params: dict,
                   scenario:     dict,
                   planner,
                   log_dir:      str,
                   fault_injector=None,
                   wcet_sink:    list = None) -> dict:
    """
    Evaluate planner on the given scenario using N_mc random wind realizations.

    Returns dict:
      name, scenario_id, IAE (=CEP [m]), max_error, pattern_length,
      pattern_width, csv (path)
    Optional fault_injector: tools.fault_injector.FaultInjector instance.
      - sensor fault: biases the wind estimate (wx_mean, wy_mean) seen by planner
      - actuator fault: scales the planned release offsets (x_off, y_off)
    Optional wcet_sink: if a list is passed, one dict per Monte Carlo sample
      {"controller": name, "step_time_us": float, "step_index": k} timing
      traj_plant.simulate() is appended to it (see tools/wcet_report.py).
      planner.plan() itself runs once per scenario (not in a per-step loop
      like the other case studies), so the repeated per-trajectory physics
      call below is timed instead - see tools/study_protocol.py.
    """
    planner.reset()
    fi = fault_injector

    h_drop    = float(scenario.get('h_drop', plant_params['h_drop_nom']))
    V_aircraft = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
    wx_mean   = float(scenario.get('wx_mean', 0.0))
    wy_mean   = float(scenario.get('wy_mean', 0.0))
    sigma_w   = float(scenario.get('wind_sigma', 0.5))
    N_mc      = int(scenario.get('N_mc', plant_params.get('N_mc', 30)))

    ctrl_name   = planner.name()
    scenario_id = scenario['id']
    csv_path    = os.path.join(log_dir, f"run_{scenario_id}_{ctrl_name}.csv")

    # Reference: nominal impact point (zero wind)
    ref_plant = BagDropPlant(plant_params)
    x_nom, y_nom, t_nom, _ = ref_plant.nominal_impact(h_drop, V_aircraft)

    # --- Planner decision (sensor fault biases wind estimate) -------------
    if fi:
        sc_obs = dict(scenario,
                      wx_mean=fi.inject_sensor(0.0, wx_mean),
                      wy_mean=fi.inject_sensor(0.0, wy_mean))
    else:
        sc_obs = scenario
    x_off, y_off = planner.plan(sc_obs, plant_params)
    # Actuator fault: scale planning offsets (navigation/execution error)
    if fi:
        x_off = fi.inject_actuator(0.0, x_off)
        y_off = fi.inject_actuator(0.0, y_off)

    # --- Monte Carlo evaluation -------------------------------------------
    rng = random.Random(scenario.get('rng_seed', 0) + hash(ctrl_name) % 997)
    impacts   = []   # list of (x_i, y_i, t_i, wx_i, wy_i)
    traj_plant = BagDropPlant(plant_params)

    for idx in range(N_mc):
        wx_s = wx_mean + sigma_w * rng.gauss(0, 1)
        wy_s = wy_mean + sigma_w * rng.gauss(0, 1)
        if wcet_sink is not None:
            t0 = time.perf_counter()
            xi, yi, ti, _ = traj_plant.simulate(
                h_drop, V_aircraft, (wx_s, wy_s, 0.0),
                x0=x_off, y0=y_off)
            dt_us = (time.perf_counter() - t0) * 1e6
            wcet_sink.append({"controller": ctrl_name, "step_time_us": dt_us, "step_index": idx})
        else:
            xi, yi, ti, _ = traj_plant.simulate(
                h_drop, V_aircraft, (wx_s, wy_s, 0.0),
                x0=x_off, y0=y_off)
        impacts.append((xi, yi, ti, wx_s, wy_s))

    # --- Metrics ----------------------------------------------------------
    # Errors relative to nominal target (x_nom, 0)
    errors_x  = [imp[0] - x_nom for imp in impacts]
    errors_y  = [imp[1]          for imp in impacts]  # target lateral = 0
    radii     = [math.sqrt(ex * ex + ey * ey)
                 for ex, ey in zip(errors_x, errors_y)]

    radii_s   = sorted(radii)
    abs_ey_s  = sorted(abs(ey) for ey in errors_y)
    ex_s      = sorted(errors_x)

    n50  = max(0, int(0.50 * N_mc) - 1)
    n80  = max(0, int(0.80 * N_mc) - 1)
    n20  = max(0, int(0.20 * N_mc) - 1)
    n95  = max(0, int(0.95 * N_mc) - 1)

    cep            = radii_s[n50]
    max_radial     = radii_s[n95]
    pattern_length = ex_s[n80] - ex_s[n20]
    pattern_width  = abs_ey_s[n95]

    # Centroid (for IterativeRefinement / AdaptiveRLS feedback)
    x_centroid = sum(imp[0] for imp in impacts) / N_mc
    y_centroid = sum(imp[1] for imp in impacts) / N_mc
    wx_centroid = sum(imp[3] for imp in impacts) / N_mc
    wy_centroid = sum(imp[4] for imp in impacts) / N_mc
    x_drift_mean = x_centroid - x_nom
    y_drift_mean = y_centroid

    # Notify learning planners
    if isinstance(planner, IterativeRefinementDrop):
        planner.update(scenario, plant_params,
                       x_off, y_off,
                       x_centroid, y_centroid, x_nom)
    if isinstance(planner, AdaptiveRLSDrop):
        planner.update(wx_centroid, wy_centroid,
                       x_drift_mean, y_drift_mean)

    # --- Write CSV --------------------------------------------------------
    with open(csv_path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['sample_id', 'x_impact', 'y_impact', 't_flight',
                         'wx_true', 'wy_true', 'error_x', 'error_y',
                         'radial_error', 'iae'])
        for idx, (xi, yi, ti, wx_s, wy_s) in enumerate(impacts):
            ex = xi - x_nom
            ey = yi
            r  = math.sqrt(ex * ex + ey * ey)
            writer.writerow([
                idx,
                f"{xi:.3f}",
                f"{yi:.3f}",
                f"{ti:.3f}",
                f"{wx_s:.3f}",
                f"{wy_s:.3f}",
                f"{ex:.3f}",
                f"{ey:.3f}",
                f"{r:.3f}",
                f"{cep:.3f}",
            ])
        # Summary row - padded to the same 10-column width as the data rows
        # above so the 'iae' column (index 9) still resolves to the CEP value
        # for tools that read the file's last row (e.g. extract_final_iae).
        writer.writerow(['# CEP', f"{cep:.3f}", 'P95', f"{max_radial:.3f}",
                         'L_pat', f"{pattern_length:.3f}",
                         'W_pat', f"{pattern_width:.3f}",
                         '', f"{cep:.3f}"])

    return {
        'name':           ctrl_name,
        'scenario_id':    scenario_id,
        'IAE':            cep,           # CEP used as primary metric
        'max_error':      max_radial,
        'pattern_length': pattern_length,
        'pattern_width':  pattern_width,
        'csv':            csv_path,
    }
