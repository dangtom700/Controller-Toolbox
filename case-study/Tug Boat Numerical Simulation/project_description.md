# Tug Boat Numerical Simulation - Project Description

**Document:** Project Description Report
**Audience:** Technical engineers familiar with control systems and C++ programming
**Date:** 2026-05-23

---

## 1. Overview

This project implements a standalone C++ numerical simulation of a four-tugboat cooperative
barge-pushing operation, grounded in the mathematical model of Li et al. (2026, *Ocean
Engineering* 357, 125514). The simulation serves as a rigorous, deterministic benchmark
environment for comparing multiple closed-loop controller designs on a physically meaningful
marine positioning task.

The simulation is implemented entirely in C++ using the Controller Toolbox library. Output
data - position histories, control effort, and performance metrics - are written to CSV for
post-processing and visualization in Python. No visualization logic resides in C++.

---

## 2. Problem Statement

A large ocean barge (200 m * 50 m) must be held at a fixed target position and heading in
the presence of persistent environmental disturbances: wind, ocean current, and wave drift.
Four independent tugboats, each with limited thrust capacity and slew rate, push the barge
from fixed attachment points. A coordinating controller issues a three-axis generalized
force/moment command that a thrust allocator distributes among the four tugs.

The key engineering challenge addressed by Li et al. is that each tugboat is an independent
dynamic body subject to its own environmental loading. Its main thruster must simultaneously
push the barge *and* compensate for its own wind/current disturbance. This physical coupling
is captured by the unified reconstructed mass, damping, and Coriolis matrices of the combined
barge-tug system.

---

## 3. Simulation Objectives

- Implement the unified 3-DOF plant equation (Li et al. Eq. 21) faithfully in C++.
- Implement environmental disturbances (wind load, current load, wave drift) matching the
  paper's disturbance models (Eqs. 3-6).
- Implement a pseudo-inverse thrust allocator with per-tug box constraints and rate limits.
- Integrate five controller designs drawn from the Controller Toolbox and evaluate each
  under three standardized test scenarios.
- Record full state trajectories and performance metrics (IAE, thrust energy, saturation
  count) to CSV for offline analysis.

---

## 4. Relationship to the Controller Toolbox

The Controller Toolbox provides production-ready C++ implementations of all controller
classes used in this study: `DiscretePID`, `KalmanFilter`, `DiscreteSMC`, `DiscreteMPC`,
and `ExtremumSeeker`. The simulation instantiates these as black-box modules through the
`IController` interface, feeding each the same plant state and recording its commanded
generalized force vector. This design allows fair, side-by-side comparison under identical
plant dynamics and disturbance realizations.

---

## 5. Scope

**In scope:**
- 3-DOF low-frequency plant dynamics (surge x, sway y, yaw ψ)
- Four fixed-position tugboats per paper Figure 5
- Wind, current, and simplified JONSWAP wave-drift disturbances
- Pseudo-inverse thrust allocation with box and rate constraints
- Five controller modes: PID, KF-PID, SMC, MPC, ESC
- Ideal and noisy measurement modes (Gaussian sensor noise injection)
- Per-tick CSV telemetry logging
- Three standardized test scenarios matching paper Table 5 conditions

**Out of scope:**
- Visualization (handled in Python via `plot_run.py`)
- Behavioral cloning / ML inference
- Fuzzy logic controller (requires external dependency not in Toolbox)
- Full-3D contact physics (rubber fender elasticity)
- Shallow-water hydrodynamic depth modification
- Real-time interactive input (human pilot mode)

---

## 6. Expected Deliverables

- `sim/` - C++ simulation core (plant, environment, allocator, controller wrappers, logger)
- `config/` - JSON parameter files (plant, scenarios, controller gains)
- `logs/` - CSV output files (one per scenario * controller run)
- `analysis/plot_run.py` - Python visualization scripts (separate from C++ build)
- Validation against paper Table 7 IAE values within +/-10% tolerance for the SMC baseline

---

## 7. Reference

Li et al. (2026). *A novel mathematical modeling and simulation of multi-tugboat cooperative
pushing operation for a large barge with environmental disturbances.* Ocean Engineering 357,
125514.
