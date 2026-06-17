# Aircraft Engine Thermal Management

> Scaffolded 2026-06-16 by `tools/new_case_study.py` from `case-study/Aircraft Engine Thermal Management/MPC for aircraft engine thermal management under nonlinear heat transfer and time delay.pdf`.
> This is a TEMPLATE - fill every TODO before counting it as implemented.

## Source

- Paper PDF: `case-study/Aircraft Engine Thermal Management/MPC for aircraft engine thermal management under nonlinear heat transfer and time delay.pdf`
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

conda run -n soft_robotics -- python "case-study/Aircraft Engine Thermal Management/sim/main.py"
