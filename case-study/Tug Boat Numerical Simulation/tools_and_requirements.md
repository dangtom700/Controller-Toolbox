# Tug Boat Numerical Simulation - Tools and Requirements

**Document:** Tools and Requirements
**Audience:** Technical engineers familiar with control systems and C++ programming
**Date:** 2026-05-23

---

## 1. Language and Standard

- **C++17** - primary implementation language for all simulation and controller logic
- **Python 3.10+** (conda environment `soft_robotics`) - post-processing and visualization only

---

## 2. Controller Toolbox Components Used

The following headers from the Controller Toolbox `lib/` directory are direct dependencies:

- `DiscretePID.h` - three-channel decoupled PID baseline (Mode 1)
- `KalmanFilter.h` - discrete linear Kalman filter for state estimation (Mode 2)
- `DiscreteSMC.h` - sliding mode controller with boundary-layer saturation (Mode 3)
- `DiscreteMPC.h` - constrained model predictive controller, condensed QP (Mode 4)
- `ExtremumSeeker.h` - model-free dither-based gradient descent (Mode 5)
- `IController.h` - abstract controller interface (all modes)
- `PlantModel.h` - linearized state-space plant model (used by MPC and KF linearization)
- `ControllerTuner.h` - offline gain tuning utility (PID Bryson's method)
- `MetricsAnalyzer.h` - IAE, variance, and energy accumulation utilities
- `KalmanFilter.h` - linear discrete KF (used standalone for KF-PID)

---

## 3. Third-Party Libraries

- **Eigen 3.4** (header-only) - matrix arithmetic for M_re, C_re, D_re assembly and RK4
  integration. No dynamic allocation in the simulation hot path.
- **nlohmann/json** (header-only, single file) - JSON parsing for `plant_params.json` and
  scenario config files
- **OSQP 0.6.x** (optional) - if `DiscreteMPC.h` QP backend is OSQP-based; otherwise
  the Toolbox condensed unconstrained QP solver is used

---

## 4. Build System

- **CMake 3.20+** - primary build system
- **Ninja** or **MSBuild** - backend (platform-dependent)
- Compiler: MSVC 19.x (Windows, x64) or GCC 12+ / Clang 15+ (Linux)
- Build types: `Debug` (assertions enabled, verbose logging) and `Release` (optimized)

---

## 5. Python Analysis Stack (Post-Processing Only)

- `numpy` - numerical post-processing of CSV data
- `matplotlib` - position traces, thruster traces, IAE bar charts
- `pandas` - CSV loading and manipulation
- Activation: `conda activate soft_robotics` before any Python script execution

---

## 6. Configuration Files

- `config/plant_params.json` - barge/tug mass, damping, added-mass coefficients, tug
  station positions, environmental coefficient tables, simulation timestep
- `config/scenarios/s2_90deg.json`, `s3_135deg.json`, `s4_180deg.json` - scenario
  presets (wind speed, direction, current, wave parameters, target pose, seed, duration)
- `config/pid_params.json` - PID gains (Kp, Ki, Kd per axis, anti-windup limits)
- `config/smc_params.json` - SMC parameters (Lambda, K_sw, phi boundary layer)
- `config/mpc_params.json` - MPC horizons (N_p, N_c), Q/R weights
- `config/kf_params.json` - Kalman filter covariance matrices (Q_kf, R_kf)
- `config/esc_params.json` - ESC dither frequency, amplitude, filter bandwidths

---

## 7. Output Format

- `logs/run_{scenario}_{controller}_{timestamp}.csv` - per-tick telemetry
- CSV columns: `t, x, y, psi, u, v, r, tau_x_cmd, tau_y_cmd, tau_psi_cmd,`
  `T1, T2, T3, T4, IAE_x, IAE_y, IAE_psi, E_fuel, sat_count`
- One file per (scenario, controller) pair; all five controllers run sequentially per scenario

---

## 8. Hardware Requirements

- x86-64 CPU with at least 2 cores (simulation is single-threaded per run; five runs may
  be launched in parallel via shell)
- 4 GB RAM minimum
- Windows 10/11 x64 or Ubuntu 22.04 LTS
- No GPU required

---

## 9. Version Control

- Git repository with `.gitignore` excluding `logs/`, `build/`, and Eigen/nlohmann headers
  if vendored locally
- `config/` and `lib/` (Controller Toolbox) tracked in-repo
