"""
simulation_runner.py
Run one (scenario, controller) pair for the Aircraft Engine Thermal
Management (Intermediate Circulation Loop) case study.

CSV columns:
  time, Thout1_ref, Thout1, Thout2_ref, Thout2, TT, m1, m2, m0,
  u1, u2, Thin1, Thin2, override_active, iae_cumulative

Safety supervisor (paper Section 3.3 / Fig. 8 "forced heat exchange cooling"):
wraps every controller, not controller-specific. When TT exceeds
(T_boil_limit - T_boil_margin), the supervisor overrides the controller's
command and forces maximum cooling flow-rate increase, exactly mirroring the
paper's own framing: "a hybrid control architecture, where the rule-based
anti-boiling logic overrides the MPC when temperatures approach the
critical limit."
"""

import csv
import os

from aircraft_engine_thermal_management_plant import AETMPlant, make_boundary_fn


def run_simulation(plant_params: dict,
                    scenario: dict,
                    controller,
                    log_dir: str,
                    fault_injector=None) -> dict:
    """Simulate the Intermediate Circulation Loop for one (scenario,
    controller) pair. Returns a summary dict and writes CSV telemetry."""
    run_params = plant_params
    if "mh3" in scenario or "Tcin3" in scenario:
        run_params = {**plant_params,
                      **{k: scenario[k] for k in ("mh3", "Tcin3") if k in scenario}}
    plant = AETMPlant(run_params)
    bc_fn = make_boundary_fn(scenario)
    plant.set_boundary_fn(bc_fn)
    controller.reset()
    fi = fault_injector

    Ts = plant_params["Ts"]
    T_sim = float(scenario["T_sim"])
    N = int(round(T_sim / Ts))

    Thout1_ref0 = plant_params.get("Thout1_ref", 400.0)
    Thout2_ref0 = plant_params.get("Thout2_ref", 400.0)
    ref_step = scenario.get("ref_step")

    T_boil_limit = plant_params.get("T_boil_limit", 503.0)
    T_boil_margin = plant_params.get("T_boil_margin", 10.0)
    boil_trigger = T_boil_limit - T_boil_margin
    u_rate_max = plant_params["u_rate_max"]

    ctrl_name = controller.name()
    scenario_id = scenario["id"]
    csv_path = os.path.join(log_dir, f"run_{scenario_id}_{ctrl_name}.csv")

    iae = 0.0
    max_TT = plant.state()[0]
    override_count = 0
    boil_violations = 0

    with open(csv_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["time", "Thout1_ref", "Thout1", "Thout2_ref", "Thout2",
                          "TT", "m1", "m2", "m0", "u1", "u2", "Thin1", "Thin2",
                          "override_active", "iae_cumulative"])

        for k in range(N):
            t = k * Ts

            if ref_step and t >= ref_step["t_step"]:
                Thout1_ref = ref_step["Thout1_ref_final"]
                Thout2_ref = ref_step["Thout2_ref_final"]
            else:
                Thout1_ref = Thout1_ref0
                Thout2_ref = Thout2_ref0
            refs = (Thout1_ref, Thout2_ref)

            x = plant.state()        # (TT, m1, m2)
            x_obs = x
            if fi:
                TT_obs = fi.inject_sensor(t, x[0])
                x_obs = (TT_obs, x[1], x[2])

            Thout1, Thout2, m0, Tmix, TTi = plant.outputs(t, x_obs)
            outs = (Thout1, Thout2, m0, Tmix, TTi)

            u1, u2 = controller.compute(refs, x_obs, t, outs)

            override_active = 0
            if x[0] > boil_trigger:
                u1 = u_rate_max
                u2 = u_rate_max
                override_active = 1
                override_count += 1

            if fi:
                u1 = fi.inject_actuator(t, u1)
                u2 = fi.inject_actuator(t, u2)

            e1 = Thout1 - Thout1_ref
            e2 = Thout2 - Thout2_ref
            iae += (abs(e1) + abs(e2)) * Ts

            Thin1, Thin2, _, _ = bc_fn(t)
            max_TT = max(max_TT, x[0])
            if x[0] >= T_boil_limit:
                boil_violations += 1

            writer.writerow([f"{t:.2f}", f"{Thout1_ref:.2f}", f"{Thout1:.3f}",
                              f"{Thout2_ref:.2f}", f"{Thout2:.3f}",
                              f"{x[0]:.3f}", f"{x[1]:.4f}", f"{x[2]:.4f}",
                              f"{m0:.4f}", f"{u1:.6f}", f"{u2:.6f}",
                              f"{Thin1:.2f}", f"{Thin2:.2f}",
                              override_active, f"{iae:.4f}"])

            plant.step((u1, u2), t)

    return {
        "name": ctrl_name,
        "scenario_id": scenario_id,
        "IAE": iae,
        "max_TT": max_TT,
        "override_count": override_count,
        "boil_violations": boil_violations,
        "csv": csv_path,
    }
