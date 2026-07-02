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
