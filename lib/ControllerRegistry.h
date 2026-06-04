#pragma once
#include <string>
#include <string_view>
#include <unordered_map>

/**
 * @file ControllerRegistry.h
 * @brief Self-registration registry for controller feature flags.
 *
 * Replaces the hand-maintained `Features.h` map (M2 from the Part 26 review).
 * Controllers register themselves via `CTRL_REGISTER_FEATURE(name)` placed at the
 * bottom of their own header.  The macro uses an `inline` variable (C++17) for
 * one-time initialisation across all translation units, so each registration fires
 * exactly once per program regardless of how many TUs include the header.
 *
 * **Self-registration in a new header (the only step required):**
 * @code
 *   // At the bottom of MyController.h, after the class definition:
 *   CTRL_REGISTER_FEATURE(my_controller)
 * @endcode
 *
 * **Reading the registry:**
 * @code
 *   for (const auto& [name, on] : ctrl::ControllerRegistry::all())
 *       std::printf("  %-22s %s\n", name.c_str(), on ? "yes" : "no");
 *
 *   // or via the thin features() wrapper (backward-compatible with Features.h):
 *   auto f = ctrl::features();
 * @endcode
 *
 * **Design notes:**
 * - Meyers singleton inner map: thread-safe initialization (C++11 guarantee).
 * - `addFeature()` is called at static-init time from `inline const bool`
 *   declarations, which are safe because the singleton is always initialized
 *   before any static objects in the same TU.
 * - Features registered in individual headers are only present when those headers
 *   have been included.  The `ControllerToolbox.h` umbrella includes everything,
 *   so `ctrl::features()` returns the full set when called after that include.
 *
 * @see Features.h  -- backward-compatible `features()` wrapper.
 * @see ControllerRegistrations.h -- centralized registrations for pre-M2 controllers.
 */

namespace ctrl {

/**
 * @brief Singleton registry mapping feature names to availability flags.
 */
class ControllerRegistry {
public:
    /**
     * @brief Register a feature flag.  Returns @p value for use in static init.
     * @param name  Feature name (lowercase, underscore-separated).
     * @param value Availability flag (almost always @c true when called from a header).
     */
    static bool addFeature(const std::string& name, bool value = true) {
        map_()[name] = value;
        return value;
    }

    /** @brief Return a snapshot copy of all registered features. */
    static std::unordered_map<std::string, bool> all() {
        return map_();
    }

    /** @brief Return @c true if @p name is registered and marked available. */
    static bool has(std::string_view name) {
        const auto& m = map_();
        auto it = m.find(std::string(name));
        return it != m.end() && it->second;
    }

    /** @brief Number of registered features. */
    static int count() {
        return static_cast<int>(map_().size());
    }

private:
    static std::unordered_map<std::string, bool>& map_() {
        static std::unordered_map<std::string, bool> m;
        return m;
    }
};

} // namespace ctrl

// ---------------------------------------------------------------------------
// Self-registration macro
//
// Usage (once per header, after the class definition):
//   CTRL_REGISTER_FEATURE(my_feature_name)
//
// The inline variable is initialised exactly once across all TUs (C++17 rule),
// so addFeature() is called once at program startup.
// ---------------------------------------------------------------------------
#define CTRL_REGISTER_FEATURE(fname)                                        \
    namespace ctrl::detail {                                                 \
        inline const bool _registered_##fname =                             \
            ::ctrl::ControllerRegistry::addFeature(#fname);                  \
    }
