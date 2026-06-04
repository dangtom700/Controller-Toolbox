#pragma once
#include "ControllerRegistry.h"

/**
 * @file Features.h
 * @brief Backward-compatible `ctrl::features()` wrapper over ControllerRegistry.
 *
 * Part 33 (M2): the hand-maintained literal map has been replaced by the
 * self-registration system in `ControllerRegistry.h`.  Controllers register
 * themselves via `CTRL_REGISTER_FEATURE(name)` at the bottom of their own
 * header.  Pre-M2 controllers are registered centrally in `ControllerRegistrations.h`
 * (included by the `ControllerToolbox.h` umbrella).
 *
 * The `features()` function below is kept for backward compatibility: any code
 * that called `ctrl::features()` continues to work unchanged.
 *
 * @code
 *   // Check whether a feature is available at runtime
 *   if (ctrl::ControllerRegistry::has("dyna"))
 *       std::cout << "DynaController available\n";
 *
 *   // Enumerate all registered features (same as before)
 *   for (const auto& [name, on] : ctrl::features())
 *       std::printf("  %-24s %s\n", name.c_str(), on ? "yes" : "no");
 * @endcode
 *
 * @see ControllerRegistry.h  -- singleton + CTRL_REGISTER_FEATURE macro.
 * @see ControllerRegistrations.h -- centralized registrations for pre-M2 controllers.
 */

namespace ctrl {

/**
 * @brief Return a snapshot of all registered features.
 *
 * Equivalent to `ControllerRegistry::all()`.
 * Returns only the features whose headers have been included in this TU;
 * using the `ControllerToolbox.h` umbrella ensures the full set is present.
 */
inline std::unordered_map<std::string, bool> features()
{
    return ctrl::ControllerRegistry::all();
}

} // namespace ctrl
