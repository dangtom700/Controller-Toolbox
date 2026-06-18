# Satellite Launch Vehicle Systems

> Scaffolded 2026-06-16 by `tools/new_case_study.py` from `case-study/Satellite Launch Vehicle Systems/Lyapunov based PD-PID in MPC for satellite launch vehicle systems.pdf`.
> This is a TEMPLATE - fill every TODO before counting it as implemented.

## Source

- Paper PDF: `case-study/Satellite Launch Vehicle Systems/Lyapunov based PD-PID in MPC for satellite launch vehicle systems.pdf`
- Extracted text: [`extracted_text.txt`](extracted_text.txt) (verify; PDF extraction is lossy)

## Plant model

TODO: state vector, inputs, outputs, governing equations, parameters
(mirror `config/plant_params.json`), integrator + sample time `Ts`.

## Control objective

TODO: reference/regulation goal, constraints, primary metric (IAE / RMS / CEP / ...).

## Controllers (12)

TODO: list the controllers. The paper's proposed method must be one of them.

## Scenarios (5)

TODO: describe each scenario in `config/scenarios/`.

## Run

conda run -n soft_robotics -- python "case-study/Satellite Launch Vehicle Systems/sim/main.py"
