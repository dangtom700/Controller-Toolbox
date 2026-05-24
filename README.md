# Controller Toolbox

A discrete-time C++17 control library with PID, LQR, LQG, MPC, ADRC, SMC, Lead-Lag, Smith Predictor, Extremum Seeking, Kalman filtering, Fuzzy Logic inference, plus an integrated tuner suite and analysis layer.

Twelve controller implementations, eight tuning families, frequency- and time-domain analysis, controller composition (Supervisory / Additive / Weighted), a lock-free parameter buffer for RT updates, and a hardware abstraction layer for simulation.

---

## Quick Start

### Native build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Requires **C++17**, **CMake >= 3.16**, and **Eigen >= 3.4**.

### Docker build

```bash
docker build -t controller-toolbox .
docker run --rm controller-toolbox          # runs the test suite
docker run --rm controller-toolbox ex02_ss_lqr   # runs a specific example
```

See [Docker Usage](#docker-usage) for more.

---

## Minimal Example

```cpp
#include "ControllerToolbox.h"

const double Ts = 0.01;
ctrl::TransferFunction G({0.0048, 0.0047}, {1.0, -1.81, 0.819}, Ts);
ctrl::StateSpace sys = ctrl::tf2ss(G);

ctrl::PIDParams pp;
pp.Kp = 1.0; pp.Ki = 0.1; pp.Kd = 0.05; pp.N = 100.0;
ctrl::DiscretePID pid(pp, Ts);

Eigen::VectorXd x = Eigen::VectorXd::Zero(sys.stateSize());
double r = 1.0, y = 0.0;
for (int k = 0; k < 500; ++k) {
    double u = pid.compute(r - y);
    Eigen::VectorXd uv(1); uv << u;
    y = ctrl::ssStep(sys, x, uv)(0);
}
```

---

## Documentation

| Document | Purpose |
|---|---|
| [docs/DOCUMENTATION.md](docs/DOCUMENTATION.md) | Full API reference, class-by-class breakdown, usage workflows |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Parameter constraints, RT/RTOS integration, troubleshooting recipes |
| [cheatsheet/](cheatsheet/) | Tuning methods, controller categories, system identification notes |
| [case-study/](case-study/) | Nonlinear 3*3 boiler-turbine multivariable study |

---

## Repository Layout

```
|-- lib/             # Library sources -> target: controller_toolbox
|-- examples/        # ex01..ex26 single-file demos + cpp/ MIMO examples
|-- case-study/      # Boiler-turbine MIMO benchmark + Tug Boat numerical simulation
|-- tests/           # CTest-driven unit + integration tests
|-- scripts/         # tune_all / simulate_all / realtime_all
|-- cheatsheet/      # Reference notes
|-- docs/            # Documentation & deployment guides
```

---

## Docker Usage

The included [`Dockerfile`](Dockerfile) uses a two-stage build:

- **Stage 1 (builder)** - Debian Bookworm slim + CMake + g++ + libeigen3-dev, compiles every target with the root [`CMakeLists.txt`](CMakeLists.txt).
- **Stage 2 (runtime)** - Slim image containing only the compiled binaries, ready to run examples, tests, or your own application.

### Build the image

```bash
docker build -t controller-toolbox .
```

### Run the test suite (default `CMD`)

```bash
docker run --rm controller-toolbox
```

### Run any example or script

```bash
docker run --rm controller-toolbox ex07_lqg_kalman
docker run --rm controller-toolbox boiler_turbine_case_study
docker run --rm controller-toolbox simulate_all
```

### Interactive shell for development

```bash
docker run --rm -it --entrypoint /bin/bash controller-toolbox
```

### Mount your own source for in-container builds

```bash
docker run --rm -it -v "$(pwd):/work" -w /work \
    --entrypoint /bin/bash controller-toolbox:builder \
    -c "cmake -S . -B build && cmake --build build --parallel"
```

(Use the `builder` stage tag - see the Dockerfile for details.)

---

## Tested Compilers

| Compiler | Version | Status |
|---|---|---|
| GCC | 9, 11, 12, 13 | OK |
| Clang | 10, 14, 16 | OK |
| MSVC | 19.20+ (VS 2019/2022) | OK |

---

## Project Status

Twelve controllers implemented and unit-tested. `FuzzyLogic` module added 2026-05-23: Mamdani and Takagi-Sugeno inference engines, `FuzzyPD`, `FuzzyPID`, and `FuzzySupervisor` classes — see [docs/DOCUMENTATION.md Section 5.2](docs/DOCUMENTATION.md). Four fuzzy application examples added (`ex23`–`ex26`). Five earlier math corrections (2026-05-22): MPC condensed prediction formula (`G_u.u_prev` term), PID backward-Euler integral law, LQG D!=0 staleness warning, Smith Predictor D.u_prev feedthrough, and SMC `c_de` Ts-absorption documentation — details in [docs/bug_report.md Section 3](docs/bug_report.md). Real-time deployment guidance in [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).
