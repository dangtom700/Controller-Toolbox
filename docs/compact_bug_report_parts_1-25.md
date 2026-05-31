# Controller Toolbox - Compact Reference: Parts 1-25

**Covers:** Parts 1-25 (2026-05-19 through 2026-05-30)
**Full history:** See git log; original reports archived in git history.
**Purpose:** Quick-reference for algorithm inventory, critical caveats, and tribal knowledge
accumulated over 25 development sessions. Use this when the full cumulative report is too
large to read; read cumulative_bug_report.md (Part 26+) for active issues.

---

## 1. Project State (as of Part 25)

- **89 C++ executables** pass | 0 failed
- **88 Python examples** pass | 0 failed (including SKIP guards for unbuilt bindings)
- **Catch2 test cases:** 65 (test_catch2_advanced) + 5 (pilot) + 9 (autoscheduling) + 3 (stability_margins) + 1 (tugsim) = 83 total
- **Python bindings:** All lib/ classes bound (ctrl_toolbox.pyd); rebuild in Release to silence MPC/GPC QP warnings
- **21 runtime warnings** normal: 15 from old .pyd (no NDEBUG), 4 from TC-REG-05 (intentional), 2 from old .pyd c2d

---

## 2. Complete Algorithm Inventory

### Core Controllers

| Class | File | Added | Notes |
|-------|------|-------|-------|
| DiscretePID | lib/DiscretePID.{h,cpp} | Part 1 | 2DOF, DoM, back-calc anti-windup (Kb), N-filter. compute(r-y) |
| DiscreteLeadLag | lib/DiscreteLeadLag.{h,cpp} | Part 1 | phaseAt() returns RADIANS (fixed P12-21) |
| DiscreteSMC | lib/DiscreteSMC.{h,cpp} | Part 1 | compute(y-ref) convention (NOT r-y); sign corrected Part 21 |
| DiscreteADRC | lib/DiscreteADRC.{h,cpp} | Part 1 | r handled internally; omega_o*Ts < 0.5 required (backward Euler) |
| DiscreteLQR | lib/DiscreteLQR.{h,cpp} | Part 2 | DARE doubling; DareResult{P,converged,iters}; PBH stability check |
| DiscreteMPC | lib/DiscreteMPC.{h,cpp} | Part 3 | condensed QP (FISTA); compute(r-y); Nu renamed from Nc in params |
| DiscreteLQG | lib/DiscreteLQG.{h,cpp} | Part 4 | LQR + KF combined |
| ExtremumSeeker | lib/ExtremumSeeker.{h,cpp} | Part 5 | ESC; ESCParams (C++) = ExtremumSeekerParams (Python) |
| SmithPredictor | lib/SmithPredictor.{h,cpp} | Part 5 | e_sp = error - (y_now - y_delayed) (sign fixed P17-1) |
| RepetitiveController | lib/RepetitiveController.{h,cpp} | Part 6 | periodic disturbance rejection |
| GeneralizedPredictiveControl | lib/GeneralizedPredictiveControl.{h,cpp} | Part 7 | GPC CARIMA; GPCParams: Nu (not Nc) |
| ControllerStack | lib/ControllerStack.{h,cpp} | Part 8 | Supervisory/Additive/Weighted; notifyObserver auto-wired |
| FeedforwardController | lib/FeedforwardController.h | Part 17 | header-only; u_ff = G_ff(z)*r |
| AdaptiveSmithPredictor | lib/AdaptiveSmithPredictor.{h,cpp} | Part 22 | cross-corr delay ID; setPlantOutput(y) before compute() |
| NonlinearMPC | lib/NonlinearMPC.{h,cpp} | Part 22 | RTI; setState+setReference+computeRef pattern; lastOutput() NOT override |
| MRACController | lib/MRACController.{h,cpp} | Part 19 | Lyapunov + sigma-mod; compute(y_plant) NOT compute(error) |
| FeedbackLinearisation | lib/FeedbackLinearisation.{h,cpp} | Part 19 | setState(x) required before compute() |
| GainScheduledController | lib/GainScheduledController.h | Part 20 | header-only; lastOutput() NOT IController override; NearestNeighbor bumpless Part 23 |
| AntiWindupWrapper | lib/AntiWindupWrapper.h | Part 24 | header-only; conditioning (Hanus 1987); do NOT wrap DiscretePID (has own Kb) |
| TubeMPC | lib/TubeMPC.{h,cpp} | Part 25 | mRPI tube; setState+computeRef pattern; y_ss = Q/(Q+R)*r bias without integral |
| DiscreteHinf | lib/DiscreteHinf.{h,cpp} | Part 18 | DGKF 2-Riccati; solveMuSyn DK-iteration; MuSynParams/MuSynResult |

### Estimators

| Class | File | Added | Notes |
|-------|------|-------|-------|
| KalmanFilter | lib/KalmanFilter.{h,cpp} | Part 1 | covariance() = P_updated (post-update, NOT pre-update DARE solution) |
| ExtendedKalmanFilter | lib/ExtendedKalmanFilter.{h,cpp} | Part 2 | numerical Jacobians (scaled eps); analyticalJacobian override |
| UnscentedKalmanFilter | lib/UnscentedKalmanFilter.{h,cpp} | Part 2 | alpha=sqrt((n+kappa)/n) to avoid negative Wc0; alpha=1e-3 default |
| MovingHorizonEstimator | lib/MovingHorizonEstimator.{h,cpp} | Part 18 | condensed QP; box constraints on w; horizon ramp-up |
| ParticleFilter | lib/ParticleFilter.{h,cpp} | Part 25 | SIR; systematic resampling; log-sum-exp weights; Kitagawa benchmark RMSE 4-10 normal |

### System Identification

| Class | File | Added | Notes |
|-------|------|-------|-------|
| RecursiveLeastSquares | lib/RecursiveLeastSquares.{h,cpp} | Part 3 | ARX; forgetting factor |
| SubspaceID | lib/SubspaceID.{h,cpp} | Part 6 | N4SID; suggest_order: TWO overloads (SubspaceID VectorXd AND BalancedTruncation TruncationResult) |
| FOPDTIdentifier | lib/FOPDTIdentifier.{h,cpp} | Part 17 | graphical + optimisation; imcTuning -> PIDParams |
| SOPDTIdentifier | lib/SOPDTIdentifier.{h,cpp} | Part 18 | tau1 >= tau2 always; Rivera 1986 IMC extension |
| LPVSystemID | lib/LPVSystemID.{h,cpp} | Part 20 | identifyLPV takes column-major (n x N) matrices NOT (N x n) |

### Analysis & Reduction

| Class | File | Added | Notes |
|-------|------|-------|-------|
| SystemAnalysis | lib/SystemAnalysis.{h,cpp} | Part 4 | calculateMargins: returns inf when no crossover found |
| MetricsAnalyzer | lib/MetricsAnalyzer.{h,cpp} | Part 4 | IAE/ISE/ITAE/settling time |
| BalancedTruncation | lib/BalancedTruncation.{h,cpp} | Part 19 | balancedTruncate; suggestOrder; H-inf error bound |
| ZeroPhaseTrackingFilter | lib/ZeroPhaseTrackingFilter.{h,cpp} | Part 19 | ZPETC; transmissionZeros; designZPETC |
| LinearisationHelper | lib/LinearisationHelper.{h,cpp} | Part 19 | jacobianX/U central diff; lineariseAtPoint ZOH c2d |
| GapMetric | lib/GapMetric.{h,cpp} | Part 20 | nuGap SISO only (throws for MIMO); nuGapMatrix symmetric |
| LinearModelCluster | lib/LinearModelCluster.h | Part 20 | header-only; single-linkage agglomerative |

### Gain Scheduling

| Class | File | Added | Notes |
|-------|------|-------|-------|
| AutoGainScheduler | lib/AutoGainScheduler.h | Part 20 | header-only; design_fn lambda MUST have trailing return type -> shared_ptr<IController> |
| GainScheduledController | lib/GainScheduledController.h | Part 20 | LinearBlend advances BOTH adjacent controllers |

### Tuning

| Class | File | Added | Notes |
|-------|------|-------|-------|
| AutoTuner | lib/AutoTuner.h | Part 22 | header-only CMA-ES; box constraints via clip |
| RelayAutoTuner | lib/ControllerTuner.{h,cpp} | Part 4 | ZN/Tyreus-Luyben/IMC/AMIGO rules |
| StepResponseTuner | lib/ControllerTuner.{h,cpp} | Part 4 | FOPDT identification + IMC |

### Fuzzy Logic

| Class | File | Notes |
|-------|------|-------|
| FuzzySystem, FuzzyPD, FuzzyPID, FuzzySupervisor | lib/FuzzyLogic.{h,cpp} | Part 6; Mamdani/TS; conditional CTRL_HAS_FUZZY |

### HAL

| Component | File | Notes |
|-----------|------|-------|
| IScheduler | lib/hal/IScheduler.h | abstract interface |
| SimScheduler | lib/hal/SimScheduler.h | host/test scheduler |
| FreeRTOSScheduler | lib/hal/FreeRTOSScheduler.h | Part 24; #if FREERTOS_VERSION |
| ZephyrScheduler | lib/hal/ZephyrScheduler.h | Part 24; #if CONFIG_ZEPHYR |
| SimPlant, SimSensor, SimActuator, SafeSensor | lib/hal/ | simulation adapters |
| StdTimer | lib/hal/StdTimer.h | std::chrono steady_clock |

---

## 3. Critical Caveats (Tribal Knowledge)

These are things that LOOK correct but are wrong, or that catch contributors repeatedly.
Every item here represents a bug that was filed and fixed.

### Sign Conventions
```
DiscretePID:         compute(r - y)         [tracking error, positive]
DiscreteSMC:         compute(y - ref)       [OPPOSITE to PID; fixed Part 21]
DiscreteADRC:        compute(r - y)         [r handled via setReference internally too]
SmithPredictor:      compute(r - y)         [e_sp = error - (y_now - y_delayed); + sign was bug P17-1]
MRACController:      compute(y_plant)       [NOT error; reference set separately]
FeedbackLinearisation: compute(error)       [but setState(x) required first]
NonlinearMPC:        computeRef(x, y_ref)   [NOT compute(error); setState pattern]
TubeMPC:             computeRef(x, y_ref)   [same pattern as NonlinearMPC]
ExtremumSeeker:      compute(J)             [cost at dithered point, not the estimate]
```

### Frequency/Discretisation
```
DiscreteADRC:  omega_o * Ts < 0.5 (backward Euler stability). At Ts=1s, omega_o=0.75 diverges.
phaseAt():     returns RADIANS (was documented as degrees until Part 12 fix P12-21)
c2d():         ZOH and Tustin give different results; neither is "wrong"
KalmanFilter:  covariance() = P_updated (post-innovation). scipy DARE gives P_pred (pre-update).
               P_upd = (I-KC)*P_pred*(I-KC)' + K*R*K' (Joseph form)
```

### API Gotchas
```
LPVSystemID:   identifyLPV(X, U, Y, ...) takes (n x N) column-major, NOT (N x n)
SubspaceID:    suggest_order has TWO overloads: VectorXd (SubspaceID) and TruncationResult (BalancedTruncation)
GainScheduledController: lastOutput() is NOT virtual override (no virtual in IController base)
NonlinearMPC:  lastOutput() is NOT override
TubeMPC:       TubeMPCParams.K uses u_tube = K*(x-x_nom) convention.
               For MATLAB lqr() gain (u=-K*x), negate: K = -K_lqr
AntiWindupWrapper: Do NOT wrap DiscretePID. PID already has built-in Kb. Double-conditioning destabilises.
TubeMPC SS bias: y_ss = Q/(Q+R)*r without integral action. Use Q=10, R=0.05 for ~0.5% error.
ParticleFilter: Kitagawa benchmark (y=x^2/20) has RMSE 4-10 normally. This is NOT a bug.
```

### Python Binding Patterns
```
All IController subclasses:   py::class_<T, ctrl::IController, std::shared_ptr<T>>
                               Missing 3rd arg causes "Unable to load custom holder" RuntimeError
std::function lambdas:        Use py::object capture (NOT py::cpp_function)
EKF/UKF functors:             Captured as py::object in lambda -> VectorXd/MatrixXd
ControllerStack.add_controller condition: lambda(error, last_output) -> bool (TWO args)
UKF alpha:                    Use alpha=sqrt((n+kappa)/n); alpha=1 with kappa=0 gives negative Wc0
ESCParams (C++):              ExtremumSeekerParams (Python)
GPCParams:                    Nu (not Nc), Np, rho_y, rho_u
```

### NumPy 2.x Compatibility
```
float(np.array)     -> TypeError in NumPy >= 2.0
Fix:                float(np.squeeze(arr)) or float(arr[0])
Applies to:         Any place where -K @ x gives shape (m,) not scalar
```

### QP Solver Notes
```
GradientProjectionQP: FISTA O(1/k^2). Pass L = lambda_max(H) (NOT 1/L).
DiscreteMPC/GPC:      QP warnings suppressed by #ifndef NDEBUG. Use Release build for production.
                      lastQPConverged() / isHealthy() for runtime health check.
qpMaxIter:            Set to 500-1000 for ill-conditioned Hessians (long Np, high rho_y/rho_u).
```

---

## 4. Open Items (after Part 25)

| ID  | Description                                           | Priority |
|-----|-------------------------------------------------------|----------|
| T2  | MIMO nu-gap (subspace chordal distance)               | Low      |
| T3  | Full DK-iteration with vector-fitting rational D(jw)  | Low      |
| T4  | MHE state constraints (linear inequalities)           | Low      |
| T5  | GainScheduledController bumpless for LinearBlend mode | Low      |
| T7  | tools/compare_controllers.py IAE/ISE table            | Low      |
| -   | FrequencyResponseID (Levy's method)                   | Low      |
| -   | SDREController (pointwise DARE)                       | Low      |
| -   | RL bridge (Python gym.Env wrapper)                    | Low      |
| -   | ROS 2 hardware interface                              | Low      |
| -   | FMU / co-simulation support                           | Low      |
| -   | CI cross-platform PyPI wheels                         | Low      |
| -   | Python binding rebuild in Release (silences QP warns) | Low      |

---

## 5. Build & Run Reference

```
Build:   conda run -n soft_robotics -- python run.py
Python:  conda run -n soft_robotics -- python <script>
Binding: cmake -S . -B build -DCTRL_BUILD_PYTHON_BINDINGS=ON -G Ninja
         cmake --build build --target ctrl_toolbox
Smoke:   conda run -n soft_robotics -- python bindings/smoke_test.py
```

Expected passing: C++ 89 | Python 88 (as of Part 25).
Runtime warnings: 21 normal (see Section 1).
Log: `run_YYYYMMDD_HHMMSS.log` written after every run.py.

---

## 6. Key File Paths

```
lib/ControllerToolbox.h    Umbrella include (single #include for users)
lib/Features.h             ctrl::features() runtime flags
lib/IController.h          Base interface; compute/reset/sampleTime/bumplessInit/isHealthy
lib/GradientProjectionQP.h FISTA solver (header-only, used by MPC/GPC/MHE/NonlinearMPC/TubeMPC)
lib/hal/HAL.h              HAL umbrella (incl. RTOS stubs)
tests/test_catch2_advanced.cpp  Main Catch2 suite (65 cases)
tests/test_stability_margins.cpp Stability margins regression (3 cases) [Part 25]
bindings/smoke_test.py     All-classes binding smoke test
docs/DOCUMENTATION.md      API reference (class guide, parameter descriptions)
CONTRIBUTING.md            Coding conventions, new-controller checklist, PR checklist
prompt/prompt_enhanced.txt Session handoff prompt (update each Part)
```

---

*Compact report covers Parts 1-25. Active issues tracked in cumulative_bug_report.md Part 26+.*
