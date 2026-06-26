#include "FTCSupervisor.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

FTCSupervisor::FTCSupervisor(std::shared_ptr<ControllerStack> stack,
                              const FaultDetectorParams &fp, double Ts)
    : stack_(std::move(stack)), classifier_(fp), Ts_(Ts)
{
    if (!stack_)
        throw std::invalid_argument("FTCSupervisor: stack must not be null");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("FTCSupervisor: Ts must be > 0");
}

void FTCSupervisor::registerFaultResponse(FaultType fault, const std::string &controllerName)
{
    const auto &entries = stack_->entries();
    const bool found = std::any_of(entries.begin(), entries.end(),
                                    [&](const StackEntry &e) { return e.name == controllerName; });
    if (!found)
        throw std::invalid_argument(
            "FTCSupervisor::registerFaultResponse: '" + controllerName +
            "' was not found in the stack - add it via stack->addController() first");
    faultResponse_[static_cast<size_t>(fault)] = controllerName;
}

void FTCSupervisor::feedResidual(double innovation, double u_cmd, double y_meas)
{
    lastInnovation_ = innovation;
    lastUCmd_ = u_cmd;
    lastYMeas_ = y_meas;
    currentFault_ = classifier_.classify(innovation, u_cmd, y_meas);
}

double FTCSupervisor::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev_;

    // Only reconfigure when the newly classified fault has a *registered* response. A heuristic
    // classifier reading a noisy/small-sample residual window can transiently report a fault
    // type nobody registered a response for (e.g. SensorNoise/ActuatorLoss blips while the real
    // condition is SensorBias); reconfiguring on those would deactivate every stack entry
    // (silent output freeze), then bumplessly re-engage against a further-diverged error once
    // the classification flickers back - compounding instability instead of tolerating it. An
    // unregistered classification is therefore ignored: keep whatever response was last applied.
    const bool actionable = !faultResponse_[static_cast<size_t>(currentFault_)].empty();
    if (actionable && (!everApplied_ || currentFault_ != lastApplied_))
    {
        for (size_t f = 0; f < faultResponse_.size(); ++f)
        {
            const std::string &name = faultResponse_[f];
            if (!name.empty())
                stack_->setActive(name, static_cast<FaultType>(f) == currentFault_);
        }
        lastApplied_ = currentFault_;
        everApplied_ = true;
    }
    const double u = stack_->compute(error);
    u_prev_ = u;
    notifyObserver(u, error);
    return u;
}

void FTCSupervisor::reset()
{
    classifier_.reset();
    stack_->reset();
    currentFault_ = FaultType::None;
    lastApplied_ = FaultType::None;
    everApplied_ = false;
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
