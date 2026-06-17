# Bouyancy-Driven Airship in Vertical Plane

> Scaffolded 2026-06-16 by `tools/new_case_study.py` from `case-study/Bouyancy-Driven Airship in Vertical Plane/full model of a bouyancy-driven airship and its control in the vertical plane.pdf`.
> This is a TEMPLATE - fill every TODO before counting it as implemented.

## Source

- Paper PDF: `case-study/Bouyancy-Driven Airship in Vertical Plane/full model of a bouyancy-driven airship and its control in the vertical plane.pdf`
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

Build target `bouyancy_driven_airship_in_vertical_plan_sim` (registered in compile.bat); runs in run.py Phase 4.
