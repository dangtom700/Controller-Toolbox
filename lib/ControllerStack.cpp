#include "ControllerStack.h"
#include <algorithm>
#include <stdexcept>
#include <numeric>

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
        // Supervisory: first eligible controller wins
        // ----------------------------------------------------------------
        case StackMode::Supervisory:
        {
            activeName_.clear();
            for (auto &e : entries_)
            {
                if (!e.active)
                    continue;
                bool eligible = !e.activationCondition ||
                                e.activationCondition(error, lastOutput_);
                if (eligible)
                {
                    out = e.controller->compute(error);
                    activeName_ = e.name;
                    break;
                }
            }
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
        // Weighted: weighted sum of all active controllers
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
    }

} // namespace ctrl
