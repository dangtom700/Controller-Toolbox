#pragma once
#include "IController.h"
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <stdexcept>

// ControllerStack - runs multiple discrete controllers in supervisory or complementary modes.
//
// StackMode::Supervisory
//   Only one controller is active per step.  Entries are evaluated in insertion order;
//   the first whose activationCondition returns true AND whose isHealthy() returns true
//   is selected.  If no condition is registered for an entry, that entry is always eligible.
//   isHealthy() == false causes the entry to be skipped with a stderr warning, enabling
//   automatic fallback: e.g., MPC (QP diverged) -> GPC -> PID without manual intervention.
//   Edge case: if no entry is active, eligible, and healthy, the output holds at the
//   previous value (bumpless hold) and a warning is emitted on stderr. Ensure at least
//   one always-eligible, always-healthy fallback entry (e.g., a PID) to avoid this.
//   Use for: gain-scheduled switching, mode-based selection, fallback chains.
//
//   Supervisory mode pseudocode:
//     for each entry e in stack (insertion order):
//         if e.active AND (e.condition == nullptr OR e.condition(error, lastOutput))
//                     AND e.controller->isHealthy():
//             if e.controller != last_active:
//                 e.controller->bumplessInit(lastOutput, error)   // bump-free handover
//                 last_active = e.controller
//             return e.controller->compute(error)
//     // no eligible entry:
//     std::cerr << "ControllerStack::Supervisory: no eligible entry. Holding last output."
//     return lastOutput   // bumpless hold
//
// StackMode::Additive
//   All enabled entries contribute; their outputs are summed.
//   Use for: inner/outer cascade (fast PID + slow MPC trim), complementary power splitting.
//
// StackMode::Weighted
//   Normalised weighted average of all enabled, gate-passing entries:
//     u = (Sigma w_i . u_i(e)) / (Sigma w_i)   over active, gate-passing entries only.
//   Note: the denominator is the sum of *active and gate-passing* weights, not all
//   stated weights.  If an entry's activationCondition returns false, its weight is
//   excluded from the denominator, so the remaining controllers automatically "fill" the
//   full output range.  Set all activationConditions to nullptr for a static blend.
//   Edge case: if all entries are inactive or gate out, the output holds at the
//   previous value (bumpless hold) and a warning is emitted on stderr.
//
//   Weighted mode pseudocode:
//     total_w = 0;  out = 0;
//     for each entry e in stack:
//         if e.active AND (e.condition == nullptr OR e.condition(error, lastOutput)):
//             out     += e.weight * e.controller->compute(error)
//             total_w += e.weight
//     if total_w > 1e-12:
//         return out / total_w
//     // all entries inactive or gated out:
//     std::cerr << "ControllerStack::Weighted: no active entry. Holding last output."
//     return lastOutput   // bumpless hold
//
//   Example - two controllers with weights [0.7, 0.3]:
//     Both active:   u = (0.7*u0 + 0.3*u1) / (0.7 + 0.3) = 0.7*u0 + 0.3*u1
//     Entry 0 gates out: u = (0.3*u1) / (0.3) = u1   (entry 1 gets full authority)
//   This means "weight" is a relative preference, not a fixed gain.
//
//   Use for: blended controller transitions, fuzzy membership weighting.
//
// Ref: Astrom "Control System Design" Ch 9 (Gain Scheduling);
//      MATLAB supervisory control patterns.
namespace ctrl
{

    enum class StackMode
    {
        Supervisory,
        Additive,
        Weighted
    };

    struct StackEntry
    {
        std::shared_ptr<IController> controller;
        std::string name;
        bool active = true;
        double weight = 1.0;

        // Optional activation gate (Supervisory / Weighted modes).
        // Receives current error and last composite output; returns true if eligible.
        std::function<bool(double error, double lastOutput)> activationCondition;
    };

    class ControllerStack : public IController
    {
    public:
        explicit ControllerStack(StackMode mode, double sampleTime);

        // Register a controller at the end of the priority queue.
        // condition = nullptr  ->  always eligible.
        void addController(std::shared_ptr<IController> controller,
                           const std::string &name,
                           double weight = 1.0,
                           std::function<bool(double, double)> condition = nullptr);

        void removeController(const std::string &name);
        void setActive(const std::string &name, bool active);
        void setWeight(const std::string &name, double weight);

        // Compute composite output per StackMode.
        double compute(double error) override;

        // Reset all controllers.
        void reset() override;

        double sampleTime() const override { return Ts_; }
        StackMode mode() const { return mode_; }
        const std::string &activeControllerName() const { return activeName_; }
        const std::vector<StackEntry> &entries() const { return entries_; }

    private:
        StackMode mode_;
        double Ts_;
        std::vector<StackEntry> entries_;
        std::string activeName_;
        std::string prevActiveName_; // tracks last selected controller for bumpless transfer
        double lastOutput_ = 0.0;

        StackEntry *findEntry(const std::string &name);
    };

} // namespace ctrl
