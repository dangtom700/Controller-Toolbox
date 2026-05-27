#pragma once
#include "IController.h"
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <stdexcept>

/**
 * @file ControllerStack.h
 * @brief Multi-controller orchestration — Supervisory, Additive, and Weighted modes.
 *
 * ControllerStack composes multiple IController instances and dispatches compute() calls
 * according to the selected StackMode. All modes implement bumpless output holding when
 * no entry is eligible.
 *
 * @see Åström, "Control System Design" Ch. 9 (Gain Scheduling).
 * @see MATLAB supervisory control patterns.
 */

namespace ctrl
{

/**
 * @brief Orchestration strategy for ControllerStack.
 */
enum class StackMode
{
    /**
     * @brief Only one controller is active per step.
     *
     * Entries are evaluated in insertion order. The first entry whose activationCondition
     * returns @c true **and** whose isHealthy() returns @c true is selected. An unhealthy
     * controller is skipped with a stderr warning, enabling automatic fallback (e.g.,
     * MPC → GPC → PID when the QP consistently fails).
     *
     * On controller change, bumplessInit() is called on the newly selected entry to
     * prevent an output bump.
     *
     * If no entry is active, eligible, and healthy, the output holds at the previous value
     * and a warning is emitted. Ensure at least one always-eligible, always-healthy entry
     * (e.g., a PID) to avoid this.
     *
     * Use for: gain-scheduled switching, mode-based selection, fallback chains.
     */
    Supervisory,

    /**
     * @brief All enabled entries contribute; their outputs are summed.
     *
     * Use for: inner/outer cascade (fast PID + slow MPC trim), complementary power splitting.
     */
    Additive,

    /**
     * @brief Normalised weighted average of all enabled, gate-passing entries.
     *
     * @code
     *   u = (Σ wᵢ·uᵢ(e)) / (Σ wᵢ)   over active, gate-passing entries only
     * @endcode
     *
     * The denominator is the sum of *active and gate-passing* weights only. When an entry's
     * activationCondition returns @c false, its weight is excluded from the denominator, so
     * the remaining controllers automatically fill the full output range. Set all conditions
     * to @c nullptr for a static blend.
     *
     * Example with weights [0.7, 0.3]:
     * - Both active:   u = 0.7·u0 + 0.3·u1
     * - Entry 0 gates out: u = u1 (entry 1 gets full authority)
     *
     * If all entries are inactive, the output holds and a warning is emitted.
     *
     * Use for: blended controller transitions, fuzzy membership weighting.
     */
    Weighted
};

/**
 * @brief A single entry in the ControllerStack priority queue.
 */
struct StackEntry
{
    std::shared_ptr<IController> controller; ///< Owned controller instance.
    std::string name;                        ///< Unique name for lookup and logging.
    bool   active = true;                    ///< Whether the entry participates in dispatch.
    double weight = 1.0;                     ///< Relative weight (Weighted mode only).

    /**
     * @brief Optional activation gate (Supervisory and Weighted modes).
     *
     * Receives the current error and last composite output; returns @c true if this entry
     * is eligible. @c nullptr means always eligible.
     */
    std::function<bool(double error, double lastOutput)> activationCondition;
};

/**
 * @brief Composite controller that dispatches to multiple IController entries.
 *
 * Itself satisfies the IController interface, so stacks can be nested.
 *
 * @code
 * // Example: supervisory fallback chain
 * auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, Ts);
 * stack->addController(mpc,  "MPC",  1.0, [](double, double) { return mpc->isHealthy(); });
 * stack->addController(pid,  "PID");   // always-eligible fallback
 *
 * // step
 * double u = stack->compute(error);
 * @endcode
 */
class ControllerStack : public IController
{
public:
    /**
     * @brief Construct an empty stack.
     * @param mode       Dispatch strategy (Supervisory, Additive, or Weighted).
     * @param sampleTime Sample period Ts [s] shared across all entries.
     */
    explicit ControllerStack(StackMode mode, double sampleTime);

    /**
     * @brief Append a controller to the priority queue.
     *
     * In Supervisory mode the insertion order determines fallback priority (first = highest).
     *
     * @param controller Shared ownership of the controller.
     * @param name       Unique name for lookup (removeController, setActive, setWeight).
     * @param weight     Relative weight used in Weighted mode (ignored in other modes).
     * @param condition  Activation gate; @c nullptr means always eligible.
     */
    void addController(std::shared_ptr<IController> controller,
                       const std::string &name,
                       double weight = 1.0,
                       std::function<bool(double, double)> condition = nullptr);

    /**
     * @brief Remove the entry with the given name.
     * @param name Controller name as passed to addController().
     * @throws std::out_of_range If no entry with that name exists.
     */
    void removeController(const std::string &name);

    /**
     * @brief Enable or disable an entry without removing it.
     * @param name   Controller name.
     * @param active @c true to enable, @c false to exclude from dispatch.
     */
    void setActive(const std::string &name, bool active);

    /**
     * @brief Update the weight of an entry (Weighted mode only).
     * @param name   Controller name.
     * @param weight New relative weight (must be ≥ 0).
     */
    void setWeight(const std::string &name, double weight);

    /**
     * @brief Compute the composite output according to the current StackMode.
     * @param error Current tracking error e[k] (or signal as defined by IController).
     * @return Composite control output u[k].
     */
    double compute(double error) override;

    /** @brief Reset all registered controllers. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Current dispatch mode. */
    StackMode mode() const { return mode_; }

    /** @brief Name of the controller selected in the most recent compute() call (Supervisory mode). */
    const std::string &activeControllerName() const { return activeName_; }

    /** @brief Read-only view of all registered entries. */
    const std::vector<StackEntry> &entries() const { return entries_; }

private:
    StackMode mode_;
    double Ts_;
    std::vector<StackEntry> entries_;
    std::string activeName_;
    std::string prevActiveName_;
    double lastOutput_ = 0.0;

    StackEntry *findEntry(const std::string &name);
};

} // namespace ctrl
