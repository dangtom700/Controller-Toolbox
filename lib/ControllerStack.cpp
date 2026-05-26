#include "ControllerStack.h"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace ctrl
{

    ControllerStack::ControllerStack(StackMode mode, double sampleTime)
        : mode_(mode), Ts_(sampleTime)
    {
    }

    void ControllerStack::addController(std::shared_ptr<IController> controller,
                                        const std::string &name,
                                        double weight,
                                        std::function<bool(double, double)> condition)
    {
        entries_.push_back({std::move(controller), name, true, weight, std::move(condition)});
    }

    void ControllerStack::removeController(const std::string &name)
    {
        entries_.erase(
            std::remove_if(entries_.begin(), entries_.end(),
                           [&](const StackEntry &e)
                           { return e.name == name; }),
            entries_.end());
    }

    StackEntry *ControllerStack::findEntry(const std::string &name)
    {
        for (auto &e : entries_)
            if (e.name == name)
                return &e;
        return nullptr;
    }

    void ControllerStack::setActive(const std::string &name, bool active)
    {
        if (auto *e = findEntry(name))
            e->active = active;
    }

    void ControllerStack::setWeight(const std::string &name, double weight)
    {
        if (auto *e = findEntry(name))
            e->weight = weight;
    }

    double ControllerStack::compute(double error)
    {
        double out = 0.0;

        switch (mode_)
        {
        // ----------------------------------------------------------------
        // Supervisory: first eligible controller wins.
        // On a controller switch, bumplessInit() is called on the incoming
        // controller so its first compute() returns approx = lastOutput_, preventing
        // an output bump at the transition.
        //
        // Edge case: if no entry is active and eligible, the output holds at
        // lastOutput_ (bumpless hold). This avoids a silent 0.0 being sent to the
        // actuator when all conditions fail simultaneously (e.g., a condition array
        // is not exhaustive). activeName_ is empty in this case.
        // ----------------------------------------------------------------
        case StackMode::Supervisory:
        {
            activeName_.clear();
            bool found = false;
            out = lastOutput_; // hold previous output if no entry qualifies
            for (auto &e : entries_)
            {
                if (!e.active)
                    continue;
                bool eligible = !e.activationCondition ||
                                e.activationCondition(error, lastOutput_);
                if (eligible)
                {
                    if (!prevActiveName_.empty() && e.name != prevActiveName_)
                        e.controller->bumplessInit(lastOutput_, error);
                    out = e.controller->compute(error);
                    activeName_ = e.name;
                    found = true;
                    break;
                }
            }
            if (!found)
                std::cerr << "[ControllerStack] WARNING: Supervisory mode - no eligible entry "
                             "found. Holding last output " << lastOutput_ << ".\n";
            prevActiveName_ = activeName_;
            break;
        }

        // ----------------------------------------------------------------
        // Additive: sum all active controllers
        // ----------------------------------------------------------------
        case StackMode::Additive:
        {
            for (auto &e : entries_)
            {
                if (!e.active)
                    continue;
                bool gate = !e.activationCondition ||
                            e.activationCondition(error, lastOutput_);
                if (gate)
                    out += e.controller->compute(error);
            }
            break;
        }

        // ----------------------------------------------------------------
        // Weighted: normalised weighted sum of all active, gate-passing entries.
        //
        // Edge case: if all entries are inactive or gate out, total_w == 0 and
        // the output holds at lastOutput_ (bumpless hold) to avoid a NaN from 0/0
        // or a spurious 0.0 being sent to the actuator.
        // ----------------------------------------------------------------
        case StackMode::Weighted:
        {
            double total_w = 0.0;
            for (auto &e : entries_)
            {
                if (!e.active)
                    continue;
                bool gate = !e.activationCondition ||
                            e.activationCondition(error, lastOutput_);
                if (gate)
                {
                    out += e.weight * e.controller->compute(error);
                    total_w += e.weight;
                }
            }
            if (total_w > 1e-12)
                out /= total_w;
            else
            {
                out = lastOutput_; // hold last output - no active, gate-passing entry
                std::cerr << "[ControllerStack] WARNING: Weighted mode - no active, gate-passing "
                             "entry found. Holding last output " << lastOutput_ << ".\n";
            }
            break;
        }
        }

        lastOutput_ = out;
        return out;
    }

    void ControllerStack::reset()
    {
        for (auto &e : entries_)
            e.controller->reset();
        lastOutput_ = 0.0;
        activeName_.clear();
        prevActiveName_.clear();
    }

} // namespace ctrl
