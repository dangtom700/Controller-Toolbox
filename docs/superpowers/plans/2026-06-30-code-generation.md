# Code Generation (DT1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Emit dependency-free, allocation-free C99 source for a single tuned, step-based
controller (`DiscretePID`, `DiscreteSMC`, or `DiscreteLeadLag`), optionally fused with one
`AntiWindupWrapper` corrector, so it can be deployed on a bare-metal MCU without linking Eigen or
`lib/`.

**Architecture:** Three free functions `ctrl::generateControllerC(...)` (no class, no
inheritance) in a new `lib/CodeGenC.h`/`.cpp`, one overload per controller-params type, each
returning a `GeneratedCode{header, source}` pair of plain C strings. Emitted C has zero structs:
gains are `static const double`, controller state is file-scope `static double`, one function
`double controller_step(double error)` plus `void controller_reset(void)`. All three controller
types share this "step-based" shape - a fixed, single-pass, O(1) arithmetic update with no
internal loop or iteration, chosen deliberately over `FuzzyPD`/`FuzzyPID` (101-point CoG grid
search per call) and `DiscreteMPC` (iterative FISTA QP solve per call, larger static footprint)
because it is the most predictable in both CPU cycles and memory for constrained MCU targets.
Golden-file Catch2 tests compile each emission with a discovered system C compiler and diff its
output against the live C++ controller.

**Tech Stack:** C++20 (the generator itself, host-side only), C99 (emitted code only), Catch2
v3.5.4, CMake `find_program` for C-compiler discovery.

## Global Constraints

- No class, no inheritance, anywhere in the emitted C or in the generator's own C++ API (per
  project instruction: exactly one controller, and optionally one corrector, ever exists on the
  target MCU).
- Emitted C: C99, no dynamic allocation (`malloc`/`new`/STL containers), no Eigen, no `std::`.
- Double precision by default (enables exact golden-file comparison against the live C++
  controllers); `float` emission is out of scope for this plan.
- In scope: `DiscretePID`, `DiscreteSMC` (first-order only), `DiscreteLeadLag` - the three
  "step-based" controllers with a single-pass, O(1), no-internal-iteration update. One optional
  corrector: `AntiWindupWrapper`.
- Out of scope: `FuzzyPD`/`FuzzyPID` (iterative CoG grid search per call), `DiscreteMPC` (iterative
  QP solve per call), `SuperTwistingSMC`, general `FuzzySystem`, any MIMO controller, any
  corrector other than `AntiWindupWrapper`, Python bindings, `float` emission.
- Design reference: `docs/superpowers/specs/2026-06-30-code-generation-design.md` (approved,
  revised 2026-06-30 to narrow scope to step-based controllers only).

---

## Task 1: `CodeGenC` skeleton + `DiscretePID` emission + golden-file test harness

**Files:**
- Create: `lib/CodeGenC.h`
- Create: `lib/CodeGenC.cpp` (only the `PIDParams` overload defined in this task; the other two
  are declared but defined in later tasks)
- Modify: `lib/CMakeLists.txt` (register `CodeGenC.cpp`)
- Modify: `lib/ControllerToolbox.h` (umbrella include)
- Modify: `tests/CMakeLists.txt` (C-compiler discovery)
- Modify: `tests/test_catch2_advanced.cpp` (shared golden-file test harness + first `[code_generation]` test)

**Interfaces:**
- Produces: `ctrl::AntiWindupConfig`, `ctrl::CodeGenParams`, `ctrl::GeneratedCode`, and the three
  `ctrl::generateControllerC(...)` declarations - all later tasks build on these exact names.
  Produces the test-file helper `codegen_test::runGeneratedC(const ctrl::GeneratedCode&, const
  std::vector<double>&, const std::string&) -> std::optional<std::vector<double>>` and the
  runtime-skip constant `codegen_test::kCCompiler` - used by every `[code_generation]` test in
  Tasks 2-5.

### Step 1: Write `lib/CodeGenC.h`

- [ ] Create `lib/CodeGenC.h`:

```cpp
#pragma once
#include "ControllerRegistry.h"
#include "DiscretePID.h"
#include "DiscreteSMC.h"
#include "DiscreteLeadLag.h"
#include <optional>
#include <string>

/**
 * @file CodeGenC.h
 * @brief Flat, boilerplate-free C99 code generation for a single tuned, step-based controller.
 *
 * Each `generateControllerC()` overload takes a controller's tuning-parameter struct (the same
 * struct that already configures the live `IController`) and emits a self-contained `.h`/`.c`
 * pair with zero dynamic allocation, zero structs, and zero dependency on Eigen or `lib/` - one
 * `double controller_step(double error)` function plus `void controller_reset(void)`, with the
 * tuned gains baked in as `static const double` and controller state as file-scope
 * `static double`.
 *
 * Scoped to "step-based" controllers only - a fixed, single-pass, O(1) update with no internal
 * loop or iteration (`DiscretePID`, `DiscreteSMC`, `DiscreteLeadLag`). `FuzzyPD`/`FuzzyPID`
 * (iterative CoG grid search per call) and `DiscreteMPC` (iterative QP solve per call) are
 * deliberately out of scope - both cost more CPU cycles per step and more static memory than a
 * memory-constrained MCU target should have to budget for, and neither fits this generator's
 * "no internal loop" shape.
 *
 * There is no `ControllerCodeGenerator` class: on the target MCU exactly one controller (and
 * optionally one corrector wrapped around it) ever exists at a time, so there is nothing to
 * dispatch polymorphically - the generator itself is three independent "params in, C string out"
 * functions.
 *
 * @see docs/superpowers/specs/2026-06-30-code-generation-design.md
 */

namespace ctrl {

/**
 * @brief Configuration for fusing one `AntiWindupWrapper`-equivalent corrector into the emitted
 *        controller's step function (inline, not a second wrapping function).
 */
struct AntiWindupConfig {
    double uMin;
    double uMax;
    double Kb = 1.0;
};

/**
 * @brief Options common to every `generateControllerC()` overload.
 */
struct CodeGenParams {
    std::string function_name = "controller_step"; ///< Name of the emitted step function.
    std::optional<AntiWindupConfig> corrector;      ///< nullopt = no corrector fused in.
};

/**
 * @brief A generated `.h`/`.c` pair. Plain data - not a class with behavior.
 */
struct GeneratedCode {
    std::string header; ///< Include-guarded prototypes only.
    std::string source;  ///< Static gains/state + controller_step()/controller_reset().
};

/**
 * @brief Emit flat C for a tuned `DiscretePID`.
 * @throws std::invalid_argument If @p cfg.corrector is set and @p p.Kb != 0 (PID already has
 *         built-in anti-windup; wrapping it doubles the correction, matching
 *         `AntiWindupWrapper`'s own constructor guard).
 */
GeneratedCode generateControllerC(const PIDParams &p, double Ts, const CodeGenParams &cfg = {});

/** @brief Emit flat C for a tuned `DiscreteSMC` (first-order boundary-layer variant). */
GeneratedCode generateControllerC(const SMCParams &p, double Ts, const CodeGenParams &cfg = {});

/** @brief Emit flat C for a tuned `DiscreteLeadLag`. */
GeneratedCode generateControllerC(const LeadLagParams &p, double Ts, const CodeGenParams &cfg = {});

} // namespace ctrl

CTRL_REGISTER_FEATURE(code_gen_c)
```

### Step 2: Write `lib/CodeGenC.cpp` skeleton + `PIDParams` overload

- [ ] Create `lib/CodeGenC.cpp`:

```cpp
#include "CodeGenC.h"
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ctrl {
namespace {

// Formats a double as a C99 floating-point literal that round-trips exactly
// (max_digits10 = 17 for IEEE-754 double).
std::string fmtD(double v)
{
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10) << v;
    std::string s = oss.str();
    if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
        s.find('E') == std::string::npos && s.find("inf") == std::string::npos &&
        s.find("nan") == std::string::npos)
        s += ".0";
    return s;
}

std::string headerFor(const std::string &function_name)
{
    std::ostringstream h;
    h << "#ifndef CONTROLLER_GEN_H\n"
      << "#define CONTROLLER_GEN_H\n\n"
      << "double " << function_name << "(double input);\n"
      << "void controller_reset(void);\n\n"
      << "#endif\n";
    return h.str();
}

} // namespace

GeneratedCode generateControllerC(const PIDParams &p, double Ts, const CodeGenParams &cfg)
{
    if (cfg.corrector && p.Kb != 0.0)
        throw std::invalid_argument(
            "generateControllerC(PIDParams): a corrector was requested but PIDParams::Kb != 0 "
            "(DiscretePID already has built-in anti-windup). Set Kb = 0 before requesting a "
            "corrector.");

    std::ostringstream c;
    c << "#include \"controller_gen.h\"\n"
      << "#include <math.h>\n\n"
      << "static const double KP = " << fmtD(p.Kp) << ";\n"
      << "static const double KI = " << fmtD(p.Ki) << ";\n"
      << "static const double KD = " << fmtD(p.Kd) << ";\n"
      << "static const double ND = " << fmtD(p.N) << ";\n"
      << "static const double KB = " << fmtD(p.Kb) << ";\n"
      << "static const double U_MIN = " << fmtD(p.uMin) << ";\n"
      << "static const double U_MAX = " << fmtD(p.uMax) << ";\n"
      << "static const double TS = " << fmtD(Ts) << ";\n\n";

    if (cfg.corrector)
        c << "static const double CORR_U_MIN = " << fmtD(cfg.corrector->uMin) << ";\n"
          << "static const double CORR_U_MAX = " << fmtD(cfg.corrector->uMax) << ";\n"
          << "static const double CORR_KB = " << fmtD(cfg.corrector->Kb) << ";\n"
          << "static double s_corr = 0.0;\n\n";

    c << "static double s_integral = 0.0;\n"
      << "static double s_deriv = 0.0;\n"
      << "static double s_e_prev = 0.0;\n"
      << "static double s_u_prev = 0.0;\n\n"
      << "static double clampd(double v, double lo, double hi) {\n"
      << "    return v < lo ? lo : (v > hi ? hi : v);\n"
      << "}\n\n"
      << "double " << cfg.function_name << "(double error) {\n"
      << "    if (!isfinite(error)) return s_u_prev;\n";

    const std::string input = cfg.corrector ? "e_in" : "error";
    if (cfg.corrector)
        c << "    const double e_in = error + s_corr;\n";

    c << "    const double alpha = 1.0 / (1.0 + ND * TS);\n"
      << "    const double d_new = alpha * s_deriv + KD * ND * alpha * (" << input
      << " - s_e_prev);\n"
      << "    const double ki_update = KI * TS * " << input << ";\n"
      << "    const double u_unsat = KP * " << input << " + (s_integral + ki_update) + d_new;\n"
      << "    const double u_raw = clampd(u_unsat, U_MIN, U_MAX);\n"
      << "    s_integral += ki_update + KB * (u_raw - u_unsat);\n"
      << "    s_deriv = d_new;\n"
      << "    s_e_prev = " << input << ";\n";

    if (cfg.corrector)
        c << "    const double u_sat = clampd(u_raw, CORR_U_MIN, CORR_U_MAX);\n"
          << "    s_corr = CORR_KB * (u_sat - u_raw);\n"
          << "    s_u_prev = u_sat;\n"
          << "    return u_sat;\n";
    else
        c << "    s_u_prev = u_raw;\n"
          << "    return u_raw;\n";

    c << "}\n\n"
      << "void controller_reset(void) {\n"
      << "    s_integral = 0.0;\n"
      << "    s_deriv = 0.0;\n"
      << "    s_e_prev = 0.0;\n"
      << "    s_u_prev = 0.0;\n";
    if (cfg.corrector)
        c << "    s_corr = 0.0;\n";
    c << "}\n";

    return GeneratedCode{headerFor(cfg.function_name), c.str()};
}

} // namespace ctrl
```

### Step 3: Wire into the build

- [ ] In `lib/CMakeLists.txt`, find the `CTRL_CORE_SOURCES` list's last entry:

```cmake
    LPMPC.cpp
)
```

Change to:

```cmake
    LPMPC.cpp
    CodeGenC.cpp
)
```

(Unconditional - `CodeGenC.h` only depends on `DiscretePID.h`/`DiscreteSMC.h`/`DiscreteLeadLag.h`,
all always-on core sources, so no `CTRL_ENABLE_*` gating is needed.)

- [ ] In `lib/ControllerToolbox.h`, find:

```cpp
#include "LPMPC.h"                   ///< LPMPC - SISO L1-cost linear MPC solved via LPSolver per step (Phase 3 OC4).
```

Add immediately after it:

```cpp
#include "CodeGenC.h"                ///< CodeGenC - flat C99 code generation for a single tuned, step-based controller (Phase 4 DT1).
```

- [ ] In `tests/CMakeLists.txt`, immediately after the existing block:

```cmake
add_executable(test_catch2_advanced test_catch2_advanced.cpp)
target_link_libraries(test_catch2_advanced PRIVATE controller_toolbox Catch2::Catch2WithMain)
target_include_directories(test_catch2_advanced PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
```

add:

```cmake

# Code-generation golden-file tests compile emitted C with a discovered system C compiler.
# Tests SKIP (not fail) at runtime when no compiler is found.
find_program(CTRL_C_COMPILER NAMES gcc cc clang)
if(NOT CTRL_C_COMPILER)
    set(CTRL_C_COMPILER "")
endif()
target_compile_definitions(test_catch2_advanced PRIVATE CTRL_C_COMPILER_PATH="${CTRL_C_COMPILER}")
```

### Step 4: Write the shared golden-file test harness + first failing test

- [ ] At the top of `tests/test_catch2_advanced.cpp`, add to the existing include block:

```cpp
#include "CodeGenC.h"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
```

- [ ] Append to the end of `tests/test_catch2_advanced.cpp`:

```cpp
// =============================================================================
// Code generation (Phase 4 DT1) - golden-file tests
// =============================================================================
namespace codegen_test {

namespace fs = std::filesystem;

inline const std::string kCCompiler = CTRL_C_COMPILER_PATH;

// Writes the generated .h/.c pair plus a small harness main() that reads doubles from
// inputs.txt (one per line), calls controller_step() on each, and writes outputs.txt
// (one per line, full double precision). Compiles standalone (no Eigen/lib link) and runs it.
// Returns std::nullopt if the compiler invocation or the generated program fails.
inline std::optional<std::vector<double>> runGeneratedC(
    const ctrl::GeneratedCode &code,
    const std::vector<double> &inputs,
    const std::string &tag)
{
    fs::path dir = fs::temp_directory_path() / ("ctrl_codegen_" + tag);
    fs::create_directories(dir);

    {
        std::ofstream h(dir / "controller_gen.h");
        h << code.header;
    }
    {
        std::ofstream c(dir / "controller_gen.c");
        c << code.source;
    }
    {
        std::ofstream m(dir / "harness.c");
        m << "#include \"controller_gen.h\"\n"
          << "#include <stdio.h>\n"
          << "int main(int argc, char** argv) {\n"
          << "    FILE* in = fopen(argv[1], \"r\");\n"
          << "    FILE* out = fopen(argv[2], \"w\");\n"
          << "    double x;\n"
          << "    while (fscanf(in, \"%lf\", &x) == 1) {\n"
          << "        fprintf(out, \"%.17g\\n\", controller_step(x));\n"
          << "    }\n"
          << "    fclose(in);\n"
          << "    fclose(out);\n"
          << "    return 0;\n"
          << "}\n";
    }
    {
        std::ofstream inf(dir / "inputs.txt");
        for (double x : inputs) inf << std::setprecision(17) << x << "\n";
    }

    const fs::path exe = dir / "harness_exe";
    std::ostringstream cmd;
    cmd << "\"" << kCCompiler << "\" -std=c99 -o \"" << exe.string() << "\" \""
        << (dir / "controller_gen.c").string() << "\" \"" << (dir / "harness.c").string() << "\"";
    if (std::system(cmd.str().c_str()) != 0) return std::nullopt;

    const fs::path outputs = dir / "outputs.txt";
    std::ostringstream runCmd;
    runCmd << "\"" << exe.string() << "\" \"" << (dir / "inputs.txt").string() << "\" \""
           << outputs.string() << "\"";
    if (std::system(runCmd.str().c_str()) != 0) return std::nullopt;

    std::vector<double> result;
    std::ifstream of(outputs);
    double v;
    while (of >> v) result.push_back(v);
    return result;
}

} // namespace codegen_test

TEST_CASE("Code generation: PID golden file matches DiscretePID", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::PIDParams p;
    p.Kp = 2.0; p.Ki = 0.5; p.Kd = 0.1; p.N = 50.0; p.Kb = 1.0;
    p.uMin = -5.0; p.uMax = 5.0;
    const double Ts = 0.01;

    ctrl::DiscretePID pid(p, Ts);
    const std::vector<double> inputs = {0.1, 0.5, 1.0, 3.0, 6.0, 6.0, 6.0, -6.0, -6.0, 0.0, 0.2};
    std::vector<double> expected;
    for (double e : inputs) expected.push_back(pid.compute(e));

    const auto code = ctrl::generateControllerC(p, Ts);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "pid");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE(actual->at(i) == Catch::Approx(expected[i]).margin(1e-9));
}
```

- [ ] **Step 5: Build to confirm it currently fails to link/compile**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target test_catch2_advanced`

Expected before this task's code exists: FAIL (undefined reference / no such header). This is
the RED step - Task 1 Step 2's `CodeGenC.cpp` above is what makes it pass, so after writing
both Step 2 and Step 4, this build should now succeed.

- [ ] **Step 6: Run the new test**

Run: `build/tests/test_catch2_advanced.exe "[code_generation]"`
Expected: `1 assertion` (or more) - PASS, `All tests passed`. If it prints
"no C compiler ... skipping", verify MSYS2 UCRT64's `gcc` is on `PATH` per
`CLAUDE.md`'s toolchain note, then re-run `cmake -S . -B build` to re-trigger `find_program`.

- [ ] **Step 7: Commit**

```bash
git add lib/CodeGenC.h lib/CodeGenC.cpp lib/CMakeLists.txt lib/ControllerToolbox.h tests/CMakeLists.txt tests/test_catch2_advanced.cpp
git commit -m "Add CodeGenC skeleton, PID emission, and code-generation golden-file test harness"
```

---

## Task 2: `DiscreteSMC` emission

**Files:**
- Modify: `lib/CodeGenC.cpp` (add the `SMCParams` overload)
- Modify: `tests/test_catch2_advanced.cpp` (add the SMC golden-file test)

**Interfaces:**
- Consumes: `codegen_test::runGeneratedC`, `codegen_test::kCCompiler` (Task 1).
- Produces: the `SMCParams` overload of `generateControllerC` (declared in Task 1's header,
  defined here).

- [ ] **Step 1: Write the failing test**

Append to `tests/test_catch2_advanced.cpp`:

```cpp
TEST_CASE("Code generation: SMC golden file matches DiscreteSMC", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::SMCParams p;
    p.c_e = 1.0; p.c_de = 0.05; p.K = 3.0; p.phi = 0.2;
    p.uMin = -4.0; p.uMax = 4.0;
    const double Ts = 0.01;

    ctrl::DiscreteSMC smc(p, Ts);
    const std::vector<double> inputs = {0.05, 0.3, 0.9, 2.0, -0.5, -2.0, 0.0, 0.01, -0.01};
    std::vector<double> expected;
    for (double e : inputs) expected.push_back(smc.compute(e));

    const auto code = ctrl::generateControllerC(p, Ts);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "smc");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE(actual->at(i) == Catch::Approx(expected[i]).margin(1e-9));
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target test_catch2_advanced`
Expected: link error - `generateControllerC(const ctrl::SMCParams&, ...)` is declared but not
defined.

- [ ] **Step 3: Implement the SMC emission**

In `lib/CodeGenC.cpp`, add after the `PIDParams` overload:

```cpp
GeneratedCode generateControllerC(const SMCParams &p, double /*Ts*/, const CodeGenParams &cfg)
{
    std::ostringstream c;
    c << "#include \"controller_gen.h\"\n"
      << "#include <math.h>\n\n"
      << "static const double C_E = " << fmtD(p.c_e) << ";\n"
      << "static const double C_DE = " << fmtD(p.c_de) << ";\n"
      << "static const double K = " << fmtD(p.K) << ";\n"
      << "static const double PHI = " << fmtD(p.phi) << ";\n"
      << "static const double U_MIN = " << fmtD(p.uMin) << ";\n"
      << "static const double U_MAX = " << fmtD(p.uMax) << ";\n\n";

    if (cfg.corrector)
        c << "static const double CORR_U_MIN = " << fmtD(cfg.corrector->uMin) << ";\n"
          << "static const double CORR_U_MAX = " << fmtD(cfg.corrector->uMax) << ";\n"
          << "static const double CORR_KB = " << fmtD(cfg.corrector->Kb) << ";\n"
          << "static double s_corr = 0.0;\n\n";

    c << "static double s_e_prev = 0.0;\n"
      << "static double s_u_prev = 0.0;\n\n"
      << "static double clampd(double v, double lo, double hi) {\n"
      << "    return v < lo ? lo : (v > hi ? hi : v);\n"
      << "}\n\n"
      << "double " << cfg.function_name << "(double error) {\n"
      << "    if (!isfinite(error)) return s_u_prev;\n";

    const std::string input = cfg.corrector ? "e_in" : "error";
    if (cfg.corrector)
        c << "    const double e_in = error + s_corr;\n";

    c << "    const double s = C_E * " << input << " + C_DE * (" << input << " - s_e_prev);\n"
      << "    double sat_val;\n"
      << "    if (PHI > 1e-12) {\n"
      << "        sat_val = s / PHI;\n"
      << "        if (sat_val < -1.0) sat_val = -1.0;\n"
      << "        if (sat_val > 1.0) sat_val = 1.0;\n"
      << "    } else {\n"
      << "        sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);\n"
      << "    }\n"
      << "    const double u_raw = clampd(-K * sat_val, U_MIN, U_MAX);\n"
      << "    s_e_prev = " << input << ";\n";

    if (cfg.corrector)
        c << "    const double u_sat = clampd(u_raw, CORR_U_MIN, CORR_U_MAX);\n"
          << "    s_corr = CORR_KB * (u_sat - u_raw);\n"
          << "    s_u_prev = u_sat;\n"
          << "    return u_sat;\n";
    else
        c << "    s_u_prev = u_raw;\n"
          << "    return u_raw;\n";

    c << "}\n\n"
      << "void controller_reset(void) {\n"
      << "    s_e_prev = 0.0;\n"
      << "    s_u_prev = 0.0;\n";
    if (cfg.corrector)
        c << "    s_corr = 0.0;\n";
    c << "}\n";

    return GeneratedCode{headerFor(cfg.function_name), c.str()};
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cmake --build build --target test_catch2_advanced && build/tests/test_catch2_advanced.exe "[code_generation]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add lib/CodeGenC.cpp tests/test_catch2_advanced.cpp
git commit -m "Add DiscreteSMC code generation"
```

---

## Task 3: `DiscreteLeadLag` emission

**Files:**
- Modify: `lib/CodeGenC.cpp` (add the `LeadLagParams` overload)
- Modify: `tests/test_catch2_advanced.cpp` (add the LeadLag golden-file test)

**Interfaces:**
- Consumes: `codegen_test::runGeneratedC`, `codegen_test::kCCompiler` (Task 1).
- Produces: the `LeadLagParams` overload of `generateControllerC`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_catch2_advanced.cpp`:

```cpp
TEST_CASE("Code generation: LeadLag golden file matches DiscreteLeadLag", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::LeadLagParams p;
    p.continuousZero = 1.0; p.continuousPole = 10.0; p.gain = 2.0;
    const double Ts = 0.01;

    ctrl::DiscreteLeadLag ll(p, Ts);
    const std::vector<double> inputs = {0.0, 1.0, 0.5, -0.5, -1.0, 2.0, 0.0, 0.3};
    std::vector<double> expected;
    for (double u : inputs) expected.push_back(ll.compute(u));

    const auto code = ctrl::generateControllerC(p, Ts);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "leadlag");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE(actual->at(i) == Catch::Approx(expected[i]).margin(1e-9));
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target test_catch2_advanced`
Expected: link error - `generateControllerC(const ctrl::LeadLagParams&, ...)` undefined.

- [ ] **Step 3: Implement the LeadLag emission**

In `lib/CodeGenC.cpp`, add after the `SMCParams` overload:

```cpp
GeneratedCode generateControllerC(const LeadLagParams &p, double Ts, const CodeGenParams &cfg)
{
    const double two_over_Ts = 2.0 / Ts;
    const double denom = two_over_Ts + p.continuousPole;
    const double b0 = p.gain * (two_over_Ts + p.continuousZero) / denom;
    const double b1 = p.gain * (p.continuousZero - two_over_Ts) / denom;
    const double a1 = (p.continuousPole - two_over_Ts) / denom;

    std::ostringstream c;
    c << "#include \"controller_gen.h\"\n"
      << "#include <math.h>\n\n"
      << "static const double B0 = " << fmtD(b0) << ";\n"
      << "static const double B1 = " << fmtD(b1) << ";\n"
      << "static const double A1 = " << fmtD(a1) << ";\n\n";

    if (cfg.corrector)
        c << "static const double CORR_U_MIN = " << fmtD(cfg.corrector->uMin) << ";\n"
          << "static const double CORR_U_MAX = " << fmtD(cfg.corrector->uMax) << ";\n"
          << "static const double CORR_KB = " << fmtD(cfg.corrector->Kb) << ";\n"
          << "static double s_corr = 0.0;\n\n";

    c << "static double s_u_prev = 0.0;\n"
      << "static double s_y_prev = 0.0;\n\n"
      << "static double clampd(double v, double lo, double hi) {\n"
      << "    return v < lo ? lo : (v > hi ? hi : v);\n"
      << "}\n\n"
      << "double " << cfg.function_name << "(double u_in) {\n"
      << "    if (!isfinite(u_in)) return s_y_prev;\n";

    const std::string input = cfg.corrector ? "e_in" : "u_in";
    if (cfg.corrector)
        c << "    const double e_in = u_in + s_corr;\n";

    c << "    const double y_raw = B0 * " << input << " + B1 * s_u_prev - A1 * s_y_prev;\n"
      << "    s_u_prev = " << input << ";\n";

    if (cfg.corrector)
        c << "    const double y_sat = clampd(y_raw, CORR_U_MIN, CORR_U_MAX);\n"
          << "    s_corr = CORR_KB * (y_sat - y_raw);\n"
          << "    s_y_prev = y_sat;\n"
          << "    return y_sat;\n";
    else
        c << "    s_y_prev = y_raw;\n"
          << "    return y_raw;\n";

    c << "}\n\n"
      << "void controller_reset(void) {\n"
      << "    s_u_prev = 0.0;\n"
      << "    s_y_prev = 0.0;\n";
    if (cfg.corrector)
        c << "    s_corr = 0.0;\n";
    c << "}\n";

    return GeneratedCode{headerFor(cfg.function_name), c.str()};
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cmake --build build --target test_catch2_advanced && build/tests/test_catch2_advanced.exe "[code_generation]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add lib/CodeGenC.cpp tests/test_catch2_advanced.cpp
git commit -m "Add DiscreteLeadLag code generation"
```

---

## Task 4: Corrector golden-file test + PID native-anti-windup guard test

**Files:**
- Modify: `tests/test_catch2_advanced.cpp` (add the corrector golden-file test and the PID guard
  test)

**Interfaces:**
- Consumes: `generateControllerC(const SMCParams&, ...)` with `CodeGenParams::corrector` set
  (Task 2), `ctrl::AntiWindupWrapper` (existing `lib/AntiWindupWrapper.h`).

Tasks 1-3 already implement corrector fusion inline in every overload. This task adds the two
tests the design's testing plan calls out that aren't covered yet: a corrector golden-file
comparison against a *live* `AntiWindupWrapper`, and the `PIDParams::Kb != 0` rejection guard.

- [ ] **Step 1: Write the tests**

Append to `tests/test_catch2_advanced.cpp`:

```cpp
TEST_CASE("Code generation: PID rejects a corrector when native anti-windup is active", "[code_generation]")
{
    ctrl::PIDParams p;
    p.Kp = 1.0; p.Ki = 0.5; p.Kb = 1.0; // native anti-windup active
    ctrl::CodeGenParams cfg;
    cfg.corrector = ctrl::AntiWindupConfig{-5.0, 5.0, 1.0};
    REQUIRE_THROWS_AS(ctrl::generateControllerC(p, 0.01, cfg), std::invalid_argument);
}

TEST_CASE("Code generation: SMC+corrector golden file matches AntiWindupWrapper(DiscreteSMC)", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::SMCParams p;
    p.c_e = 1.0; p.c_de = 0.05; p.K = 3.0; p.phi = 0.2;
    p.uMin = -10.0; p.uMax = 10.0; // inner has wide limits; corrector applies the real bound
    const double Ts = 0.01;

    auto smc = std::make_shared<ctrl::DiscreteSMC>(p, Ts);
    ctrl::AntiWindupWrapper wrapper(smc, -4.0, 4.0, 0.8);

    const std::vector<double> inputs = {0.05, 0.3, 0.9, 2.0, -0.5, -2.0, 0.0, 0.01, -0.01};
    std::vector<double> expected;
    for (double e : inputs) expected.push_back(wrapper.compute(e));

    ctrl::CodeGenParams cfg;
    cfg.corrector = ctrl::AntiWindupConfig{-4.0, 4.0, 0.8};
    const auto code = ctrl::generateControllerC(p, Ts, cfg);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "smc_corrector");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE(actual->at(i) == Catch::Approx(expected[i]).margin(1e-9));
}
```

- [ ] **Step 2: Add the missing include**

At the top of `tests/test_catch2_advanced.cpp`, if `AntiWindupWrapper.h` is not already included
(check the existing include block first - other test cases earlier in the file may already use
`ctrl::AntiWindupWrapper`), add:

```cpp
#include "AntiWindupWrapper.h"
```

- [ ] **Step 3: Run tests to verify they pass**

Run: `cmake --build build --target test_catch2_advanced && build/tests/test_catch2_advanced.exe "[code_generation]"`
Expected: PASS (both new tests, plus all prior `[code_generation]` tests still passing).

- [ ] **Step 4: Commit**

```bash
git add tests/test_catch2_advanced.cpp
git commit -m "Add corrector golden-file test and PID anti-windup rejection guard test"
```

---

## Task 5: Zero-allocation / freestanding grep check

**Files:**
- Modify: `tests/test_catch2_advanced.cpp` (add the freestanding-source check)

**Interfaces:**
- Consumes: all three `generateControllerC(...)` overloads (Tasks 1-3).

- [ ] **Step 1: Write the test**

Append to `tests/test_catch2_advanced.cpp`:

```cpp
TEST_CASE("Code generation: emitted sources are freestanding (no malloc/new/STL/Eigen)", "[code_generation]")
{
    static const std::vector<std::string> kForbidden = {
        "malloc", "calloc", "realloc", "new ", "std::", "#include <vector",
        "#include <memory", "#include <Eigen", "#include <string"
    };

    auto checkFreestanding = [](const ctrl::GeneratedCode &code, const std::string &tag) {
        for (const auto &token : kForbidden) {
            INFO(tag << ": forbidden token \"" << token << "\"");
            CHECK(code.header.find(token) == std::string::npos);
            CHECK(code.source.find(token) == std::string::npos);
        }
    };

    checkFreestanding(ctrl::generateControllerC(ctrl::PIDParams{}, 0.01), "PID");
    checkFreestanding(ctrl::generateControllerC(ctrl::SMCParams{}, 0.01), "SMC");
    checkFreestanding(ctrl::generateControllerC(ctrl::LeadLagParams{}, 0.01), "LeadLag");
}
```

- [ ] **Step 2: Run to verify it passes**

Run: `cmake --build build --target test_catch2_advanced && build/tests/test_catch2_advanced.exe "[code_generation]"`
Expected: PASS. This test needs no C compiler (it inspects the generated `std::string`s
directly, not compiled output), so it always runs.

- [ ] **Step 3: Commit**

```bash
git add tests/test_catch2_advanced.cpp
git commit -m "Add freestanding-source (no malloc/STL/Eigen) check for code generation"
```

---

## Task 6: Example + build registration

**Files:**
- Create: `examples/ex120_code_generation.cpp`
- Modify: `examples/CMakeLists.txt`
- Modify: `compile.bat`
- Modify: `compile.sh`

**Interfaces:**
- Consumes: all three `generateControllerC(...)` overloads (Tasks 1-3).

- [ ] **Step 1: Write the example**

Create `examples/ex120_code_generation.cpp`:

```cpp
/**
 * @file ex120_code_generation.cpp
 * @brief Phase 4 (DT1): generate flat, dependency-free C99 for a tuned step-based controller of
 *        each supported type, plus one corrector-fused example, and write the .h/.c pairs to
 *        disk.
 *
 * @see docs/superpowers/specs/2026-06-30-code-generation-design.md
 */

#include "ControllerToolbox.h"
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

bool writePair(const std::string &tag, const ctrl::GeneratedCode &code)
{
    namespace fs = std::filesystem;
    const fs::path dir = fs::path("generated") / tag;
    fs::create_directories(dir);
    std::ofstream(dir / "controller_gen.h") << code.header;
    std::ofstream(dir / "controller_gen.c") << code.source;
    const bool ok = !code.header.empty() && !code.source.empty() &&
                     code.header.find("controller_step") != std::string::npos;
    std::cout << "  [" << tag << "] " << (ok ? "generated" : "FAILED") << " -> "
              << dir.string() << "\n";
    return ok;
}

} // namespace

int main()
{
    bool ok = true;

    ctrl::PIDParams pid;
    pid.Kp = 2.0; pid.Ki = 0.5; pid.Kd = 0.1; pid.uMin = -5.0; pid.uMax = 5.0;
    ok &= writePair("pid", ctrl::generateControllerC(pid, 0.01));

    ctrl::SMCParams smc;
    smc.c_e = 1.0; smc.c_de = 0.05; smc.K = 3.0; smc.phi = 0.2; smc.uMin = -10.0; smc.uMax = 10.0;
    ok &= writePair("smc", ctrl::generateControllerC(smc, 0.01));

    ctrl::LeadLagParams ll;
    ll.continuousZero = 1.0; ll.continuousPole = 10.0; ll.gain = 2.0;
    ok &= writePair("leadlag", ctrl::generateControllerC(ll, 0.01));

    // One corrector-fused example: SMC + AntiWindupWrapper-equivalent, fused inline.
    ctrl::CodeGenParams cfg;
    cfg.corrector = ctrl::AntiWindupConfig{-4.0, 4.0, 0.8};
    ok &= writePair("smc_corrector", ctrl::generateControllerC(smc, 0.01, cfg));

    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
```

- [ ] **Step 2: Register the example**

In `examples/CMakeLists.txt`, find:

```cmake
add_example(ex119_lp_mpc)
```

Add immediately after it:

```cmake
add_example(ex120_code_generation)
```

In `compile.bat`, find the block ending with:

```
    ex118_lp_solver
    ex119_lp_mpc
```

Add `ex120_code_generation` as the next line:

```
    ex118_lp_solver
    ex119_lp_mpc
    ex120_code_generation
```

In `compile.sh`, apply the same addition to its matching list.

- [ ] **Step 3: Build and run**

Run: `cmake --build build --target ex120_code_generation`
Run: `build/examples/ex120_code_generation.exe` (or the platform-appropriate output path used by
the other `exNN_*` examples in this build)
Expected: 4 "generated" lines (pid, smc, leadlag, smc_corrector) and a final `PASS`.

- [ ] **Step 4: Commit**

```bash
git add examples/ex120_code_generation.cpp examples/CMakeLists.txt compile.bat compile.sh
git commit -m "Add code-generation example (ex120) covering PID/SMC/LeadLag + corrector"
```

---

## Task 7: Documentation updates

**Files:**
- Modify: `docs/cumulative_bug_report.md` (new Part 71 section)
- Modify: `docs/ALGORITHM_ROADMAP_PHASE3.md` (DT1 status)
- Modify: `docs/algorithm_backlog.md` (move "Code generation" to "Already done")

- [ ] **Step 1: Run the full verification suite**

Run: `conda run -n soft_robotics -- python run.py`
Expected: all 7 phases pass, no regressions. Record the assertion/test-case counts from the
`test_catch2_advanced` phase for the bug-report entry below.

- [ ] **Step 2: Add the cumulative bug report entry**

Append to `docs/cumulative_bug_report.md` (following the exact structure of the existing "Part
70" section - header, one-paragraph summary, "Verification" paragraph with the real numbers from
Step 1, a "Non-obvious facts added" fenced block, and a "Docs updated" paragraph):

```markdown
## Part 71 - Algorithm Roadmap Phase 3, Phase 4 (DT1: ControllerCodeGen) - 2026-06-30

Implemented `lib/CodeGenC.h`/`.cpp`: three free functions (no class, no inheritance) that emit
flat, dependency-free C99 for a single tuned `DiscretePID`/`DiscreteSMC`/`DiscreteLeadLag`,
optionally fused with one `AntiWindupWrapper`-equivalent corrector inline. Scoped deliberately to
"step-based" controllers only (fixed, single-pass, O(1) update, no internal loop) -
`FuzzyPD`/`FuzzyPID` (iterative CoG grid search per call) and `DiscreteMPC` (iterative QP solve
per call, larger static footprint) were both considered during design and dropped for
memory-/CPU-cycle predictability on constrained MCU targets; see the design doc's revision note.

**Verification:** [fill in with the actual `test_catch2_advanced` assertion/test-case counts and
overall `run.py` phase results from Step 1 - do not leave this placeholder in the committed
report; this bracketed instruction itself must not appear in the final text].

**Non-obvious facts added (Part 71):**
```
CodeGenC scope       -> DiscretePID/DiscreteSMC(1st-order)/DiscreteLeadLag only ("step-based":
                         single-pass, O(1), no internal loop); FuzzyPD/FuzzyPID and DiscreteMPC
                         deliberately excluded (iterative per-step cost, larger memory footprint)
CodeGenC correctors   -> AntiWindupWrapper only, fused inline (no wrapping function); rejected at
                         generation time if PIDParams::Kb != 0 (native anti-windup already active)
CodeGenC build gating -> unconditional in CTRL_CORE_SOURCES (no CTRL_ENABLE_* gate needed, since
                         PID/SMC/LeadLag are all always-on core sources)
Golden-file tests     -> SKIP (not fail) at runtime via Catch2 SKIP() when no gcc/cc/clang is
                         found on PATH at CMake configure time (tests/CMakeLists.txt find_program)
```

**Docs updated:** `docs/ALGORITHM_ROADMAP_PHASE3.md` status table (DT1 Open -> Done);
`docs/algorithm_backlog.md` ("Code generation" moved to "Already done");
`docs/superpowers/specs/2026-06-30-code-generation-design.md` (design, already committed); this
report's new Part 71 section.
```

- [ ] **Step 3: Update the roadmap status table**

In `docs/ALGORITHM_ROADMAP_PHASE3.md`, find the status table row:

```
| DT1 | Code Generation | 4 | Open |
```

Change to:

```
| DT1 | Code Generation | 4 | Done |
```

Also update the header's shipped count. Find:

```
**Status:** Planning - 25 of 32 items shipped (Phase 1, Phase 2, and Phase 3 complete:
ML1/ML2/NC3/SI4/SI3/ML3/FD2 all done; Phase 4 underway: OC2, OC4 done).
```

Change to:

```
**Status:** Planning - 26 of 32 items shipped (Phase 1, Phase 2, and Phase 3 complete:
ML1/ML2/NC3/SI4/SI3/ML3/FD2 all done; Phase 4 underway: OC2, OC4, DT1 done).
```

- [ ] **Step 4: Move the backlog entry**

In `docs/algorithm_backlog.md`, find and remove this line from its current location (near line
192, in the Deployment/Production section):

```
| Code generation (C/C++ from controller design) | Heavy lift; highest production value of this category per the original wishlist's own priority table. |
```

Add a new row to the "Already done" table (near line 56, alongside the `Linear-programming-based
control` row), matching that table's exact column format:

```
| Code generation (C/C++ from controller design) | `lib/CodeGenC.h`/`.cpp` - flat C99 emission for DiscretePID/DiscreteSMC/DiscreteLeadLag ("step-based": single-pass, O(1), no internal loop), with an optional AntiWindupWrapper corrector fused inline - Phase 4 DT1. No class/inheritance in the generator or the emitted code (matches the target's one-controller-one-corrector MCU constraint); FuzzyPD/FuzzyPID (iterative CoG grid search), DiscreteMPC (iterative QP solve), `float` emission, and Python bindings for the generator are explicitly out of scope for CPU-cycle/memory predictability on constrained MCU targets; see the design doc. |
```

- [ ] **Step 5: Commit**

```bash
git add docs/cumulative_bug_report.md docs/ALGORITHM_ROADMAP_PHASE3.md docs/algorithm_backlog.md
git commit -m "Document DT1 code generation completion (Part 71, roadmap status, backlog)"
```

---

## Self-review notes (for whoever executes this plan)

- **Spec coverage:** every decision-log item in
  `docs/superpowers/specs/2026-06-30-code-generation-design.md` maps to a task: items 1-4 ->
  Task 1; item 5 (corrector fusion) -> Tasks 1-4.
- **`StateSpace` note (moot for this scope):** `DiscreteMPC` and its `StateSpace` plant are no
  longer touched by this plan at all (dropped from scope) - no accessor changes to `lib/`
  controller classes are needed for this phase.
- **`LeadLagParams` has no `uMin`/`uMax`:** `DiscreteLeadLag` is a pure IIR filter with no internal
  saturation - Task 3's emission correctly has no clamp on the un-corrected path; only a requested
  corrector's own `CORR_U_MIN`/`CORR_U_MAX` ever clamp its output.
