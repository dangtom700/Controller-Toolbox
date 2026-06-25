# Documentation Index

One map into all committed docs, organized by what you're trying to do rather than where
the file happens to live. Nothing here duplicates content — every line links to a single
source of truth.

---

## New to the repo

| Doc | What it's for |
|---|---|
| [README.md](../README.md) | Quick start, minimal example, repo layout, controller inventory |
| `.\setup.ps1` (Windows) / `./setup.sh` (Linux/macOS) | One-time toolchain + conda env + bindings bootstrap |
| [docs/handoff.md](handoff.md) | Onboarding tribal knowledge — current, verified, kept in sync. Read this before trusting any "status" doc below |

## Using the library

| Doc | What it's for |
|---|---|
| [docs/DOCUMENTATION.md](DOCUMENTATION.md) | Full API reference, class-by-class |
| [docs/control_strategies_deep_dive.md](control_strategies_deep_dive.md) | Strategy-family taxonomy, plant/controller interference, decision framework |
| [cheatsheet/controller_categories.md](../cheatsheet/controller_categories.md) | Controllers grouped by category |
| [cheatsheet/controller_list.md](../cheatsheet/controller_list.md) | Master algorithm list, implemented vs. not |
| [cheatsheet/control_design_pipeline.md](../cheatsheet/control_design_pipeline.md) | Requirements -> modeling -> design -> validation workflow |
| [cheatsheet/controller-tuning-reference.md](../cheatsheet/controller-tuning-reference.md) | Per-controller tuning parameter guide |
| [cheatsheet/tuning_methods.md](../cheatsheet/tuning_methods.md) | Tuning method notes (Ziegler-Nichols, relay, optimisation-based, ...) |
| [cheatsheet/system_identification.md](../cheatsheet/system_identification.md) | System ID method overview |
| [cheatsheet/system_identification/](../cheatsheet/system_identification/) | Worked examples: `fopdt.md`, `armax.md`, `n4sid.md` |
| [cheatsheet/advanced_model_estimation.md](../cheatsheet/advanced_model_estimation.md) | Grey-box / recursive grey-box / GP residual estimation |
| [cheatsheet/model_evaluation.md](../cheatsheet/model_evaluation.md) | Validating an identified model against data |
| [cheatsheet/mismatch_detection.md](../cheatsheet/mismatch_detection.md) | Runtime model-mismatch monitoring (CUSUM on innovations) |
| [cheatsheet/phase2_hybrid_modeling.md](../cheatsheet/phase2_hybrid_modeling.md) | Hybrid (physical + data-correction) modeling |
| [cheatsheet/embedded_and_realtime.md](../cheatsheet/embedded_and_realtime.md) | `lib/embedded/` header-only subset notes |

## Case studies

| Doc | What it's for |
|---|---|
| [case-study/](../case-study/) | The 18 end-to-end physics studies — start with the study's own `README.md` |
| [docs/case_study_status.md](case_study_status.md) | Auto-generated per-study status (regenerate via `tools/case_study_tracker.py`; never hand-edit) |
| [docs/case_study_copilot_reference.md](case_study_copilot_reference.md) | Condensed public-API map for scaffolding a new study |

## Deploying

| Doc | What it's for |
|---|---|
| [docs/deployment.md](deployment.md) | Parameter constraints, zero-allocation/RTOS guidance, lock-free parameter buffer, troubleshooting |
| [ros2/ctrl_toolbox_ros2/README.md](../ros2/ctrl_toolbox_ros2/README.md) | ROS 2 lifecycle node adapter |

## Contributing

| Doc | What it's for |
|---|---|
| [CONTRIBUTING.md](../CONTRIBUTING.md) | Build/test workflow, adding a controller/case study, sign conventions, numerical safety, PR checklist |

## Backlog / planned work

| Doc | What it's for |
|---|---|
| [docs/algorithm_backlog.md](algorithm_backlog.md) | Candidate algorithm list for future phases, deduped against current `lib/` contents |
| [docs/ALGORITHM_ROADMAP_PHASE3.md](ALGORITHM_ROADMAP_PHASE3.md) | Phase-3 roadmap — 32 designs sequencing every open `algorithm_backlog.md` item by value/ROI into 5 phases |
| [docs/superpowers/specs/](superpowers/specs/) | Approved design specs awaiting or under implementation |

## Historical / internal records

These describe a point-in-time snapshot or a now-closed-out plan, not current state. If a
historical doc disagrees with [docs/handoff.md](handoff.md), `docs/case_study_status.md`,
or the source itself, trust the latter.

| Doc | What it's for |
|---|---|
| [docs/PROJECT_MASTER_STATE.md](PROJECT_MASTER_STATE.md) | Project state snapshot — flagged stale in its own header; see `handoff.md` instead |
| [docs/ALGORITHM_ROADMAP_PHASE2.md](ALGORITHM_ROADMAP_PHASE2.md) | Phase-2 roadmap — 12/13 items shipped; kept as original design rationale |
| [docs/robust_implementation_plan.md](robust_implementation_plan.md) | Robustness-analysis implementation plan — all 5 phases complete |
| [docs/cumulative_bug_report.md](cumulative_bug_report.md) | Long-form, Part-numbered change/bug history (Part 51+) |
| [docs/compact_bug_report_parts_1-25.md](compact_bug_report_parts_1-25.md) / [26-50.md](compact_bug_report_parts_26-50.md) | Condensed history for earlier parts |
| [docs/archived/](../docs/archived/) | Superseded docs kept for reference (`test_update.md`, `audit_report.md`, `roadmap_deployment_frontend.md`) |
