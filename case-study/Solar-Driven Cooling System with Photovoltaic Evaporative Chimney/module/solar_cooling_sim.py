"""
solar_cooling_sim.py - Simulation runner for the Solar-Driven Cooling System.

Runs the plant under a set of scenario weather profiles and controller modes,
writing one CSV log per (scenario, controller) pair to ../logs/.

Usage:
    python solar_cooling_sim.py                  # all scenarios, all controllers
    python solar_cooling_sim.py --ctrl PID       # only PID controller
    python solar_cooling_sim.py --scenario s01   # only scenario s01

CSV columns:
    t_s, G_Wm2, T_amb_C, phi_amb, v_w_ms,
    m_dot_w_kgs, kr,
    Tw1_C, Tw2_C, T_ai_C, ma_dot_kgs,
    Q_cond_kW, W_comp_kW, EER,
    Tc_C, eta_PV, W_PV_W,
    Q_op_ls, W_pump_W, eta_pump,
    EER_grid, W_net_kW,
    ref_Tw1_C, error_Tw1_C
"""

from __future__ import annotations

import csv
import argparse
import math
import sys
from pathlib import Path

# Make module directory importable when run directly
sys.path.insert(0, str(Path(__file__).parent))

from solar_cooling_plant import (
    PlantParams, SolarCoolingPlant, WeatherInput, ControlInput, PlantOutput
)
from solar_cooling_controller import (
    PIDController, FFPIDController, MPCController,
    PIDParams, FFPIDParams, MPCParams, PIDGains,
    make_pid, make_ffpid, make_mpc,
)


# ---------------------------------------------------------------------------
# Scenario definitions
# ---------------------------------------------------------------------------
def _sine_irradiance(t: float, G_peak: float = 920.0, day_s: float = 43200.0) -> float:
    """Simple sunrise-to-sunset bell: 0 at t=0, peak at t=day_s/2, 0 at t=day_s."""
    phase = math.pi * t / day_s
    return max(0.0, G_peak * math.sin(phase))


SCENARIOS: dict[str, dict] = {
    "s01_design_steady": {
        "label": "Design point steady-state regulation",
        "ref_Tw1_C": 40.0,
        "weather_fn": lambda t: WeatherInput(G=920.0, T_amb=35.6, phi_amb=0.35, v_w=3.028),
    },
    "s02_irradiance_ramp": {
        "label": "Rising irradiance (FF-PID anticipates load)",
        "ref_Tw1_C": 40.0,
        "weather_fn": lambda t: WeatherInput(
            G=min(1000.0, 250.0 + t / 3.6),      # 250 -> 1000 over 1 h
            T_amb=30.0, phi_amb=0.50, v_w=5.0
        ),
    },
    "s03_cloudy_disturbance": {
        "label": "Partial cloud (irradiance step drop then recovery)",
        "ref_Tw1_C": 38.0,
        "weather_fn": lambda t: WeatherInput(
            G=500.0 if 600 < t < 1800 else 900.0,
            T_amb=28.0, phi_amb=0.60, v_w=4.0
        ),
    },
    "s04_setpoint_step": {
        "label": "Setpoint step change from 42 -> 38 ^\circC at t=1800 s",
        "ref_Tw1_fn": lambda t: 42.0 if t < 1800 else 38.0,
        "weather_fn": lambda t: WeatherInput(G=750.0, T_amb=32.0, phi_amb=0.45, v_w=3.5),
    },
    "s05_high_humidity": {
        "label": "High humidity reduces evaporative effectiveness",
        "ref_Tw1_C": 40.0,
        "weather_fn": lambda t: WeatherInput(G=800.0, T_amb=38.0, phi_amb=0.90, v_w=2.0),
    },
}


def _get_ref(sc: dict, t: float) -> float:
    if "ref_Tw1_fn" in sc:
        return sc["ref_Tw1_fn"](t)
    return sc["ref_Tw1_C"]


# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------
def run_scenario(scenario_key: str, scenario: dict, plant: SolarCoolingPlant,
                 ctrl_name: str, controller, log_dir: Path, Ts: float) -> None:
    duration = plant.p.duration
    n_steps  = int(duration / Ts)

    log_dir.mkdir(parents=True, exist_ok=True)
    csv_path = log_dir / f"run_{scenario_key}_{ctrl_name}.csv"

    is_ffpid = isinstance(controller, FFPIDController)

    fieldnames = [
        "t_s", "G_Wm2", "T_amb_C", "phi_amb", "v_w_ms",
        "m_dot_w_kgs", "kr",
        "Tw1_C", "Tw2_C", "T_ai_C", "ma_dot_kgs",
        "Q_cond_kW", "W_comp_kW", "EER",
        "Me_calc", "Me_target",
        "Tc_C", "eta_PV", "W_PV_W", "vi_ms",
        "Q_op_ls", "W_pump_W", "eta_pump",
        "EER_grid", "W_net_kW",
        "ref_Tw1_C", "error_Tw1_C",
    ]

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        # Initialise: compute steady state at t=0 to prime controller
        weather = scenario["weather_fn"](0.0)
        ref_Tw1 = _get_ref(scenario, 0.0)

        # Warm-start: nominal chimney spray flow
        u = ControlInput(m_dot_w=0.10, kr=0.85)
        out = plant.step(weather, u)

        controller.reset()

        for k in range(n_steps):
            t = k * Ts
            weather = scenario["weather_fn"](t)
            ref_Tw1 = _get_ref(scenario, t)

            if is_ffpid:
                controller.set_irradiance(weather.G)

            u = controller.compute(reference=ref_Tw1, measurement=out.Tw1_C)
            out = plant.step(weather, u)

            writer.writerow({
                "t_s":          t,
                "G_Wm2":        weather.G,
                "T_amb_C":      weather.T_amb,
                "phi_amb":      weather.phi_amb,
                "v_w_ms":       weather.v_w,
                "m_dot_w_kgs":  u.m_dot_w,
                "kr":           u.kr,
                "Tw1_C":        out.Tw1_C,
                "Tw2_C":        out.Tw2_C,
                "T_ai_C":       out.T_ai_C,
                "ma_dot_kgs":   out.ma_dot_kgs,
                "Q_cond_kW":    out.Q_cond_kW,
                "W_comp_kW":    out.W_comp_kW,
                "EER":          out.EER,
                "Me_calc":      out.Me_calc,
                "Me_target":    out.Me_target,
                "Tc_C":         out.Tc_C,
                "eta_PV":       out.eta_PV,
                "W_PV_W":       out.W_PV_W,
                "vi_ms":        out.vi_ms,
                "Q_op_ls":      out.Q_op_ls,
                "W_pump_W":     out.W_pump_W,
                "eta_pump":     out.eta_pump,
                "EER_grid":     out.EER_grid if out.EER_grid != float("inf") else 999.0,
                "W_net_kW":     out.W_net_kW,
                "ref_Tw1_C":    ref_Tw1,
                "error_Tw1_C":  ref_Tw1 - out.Tw1_C,
            })

    print(f"  Logged: {csv_path.name}")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def _make_controller(name: str, Ts: float) -> object:
    if name == "PID":
        return make_pid(Ts)
    if name == "FFPID":
        return make_ffpid(Ts)
    if name == "MPC":
        return make_mpc(Ts, Tw1_nom=40.0, a=0.018, b=4.5)
    raise ValueError(f"Unknown controller: {name}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Solar Cooling Simulation Runner")
    parser.add_argument("--scenario", default=None,
                        help="Scenario key prefix filter (e.g. s01)")
    parser.add_argument("--ctrl", default=None,
                        help="Controller name: PID, FFPID, MPC")
    args = parser.parse_args()

    cfg_path = Path(__file__).resolve().parent.parent / "config" / "plant_params.json"
    log_dir  = Path(__file__).resolve().parent.parent / "logs"

    params = PlantParams.from_json(cfg_path)
    Ts     = params.dt

    ctrl_names = ["PID", "FFPID", "MPC"]
    if args.ctrl:
        ctrl_names = [args.ctrl]

    scenarios = SCENARIOS
    if args.scenario:
        scenarios = {k: v for k, v in SCENARIOS.items() if k.startswith(args.scenario)}

    print(f"Solar Cooling Simulation  |  Ts={Ts}s  |  duration={params.duration}s")
    print(f"Controllers : {ctrl_names}")
    print(f"Scenarios   : {list(scenarios.keys())}")
    print()

    for sc_key, sc in scenarios.items():
        print(f"[{sc_key}] {sc['label']}")
        for cn in ctrl_names:
            ctrl  = _make_controller(cn, Ts)
            plant = SolarCoolingPlant(params)   # fresh plant per run
            run_scenario(sc_key, sc, plant, cn, ctrl, log_dir, Ts)
        print()

    print("Done.")


if __name__ == "__main__":
    main()
