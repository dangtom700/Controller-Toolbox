#pragma once
#include "ControllerStack.h"
#include "FaultClassifier.h"
#include "IController.h"
#include <array>
#include <memory>
#include <string>

/**
 * @file FTCSupervisor.h
 * @brief Fault-tolerant control reconfiguration on top of ControllerStack (Phase 3 Roadmap
 *        Phase 2 DT4).
 *
 * Closes the loop from fault detection to controller reconfiguration: a FaultClassifier drives
 * which single ControllerStack entry is `active` via ControllerStack::setActive(), reusing its
 * existing Supervisory-mode bumpless transfer wholesale - no ControllerStack changes needed.
 *
 * @see docs/superpowers/specs/2026-06-25-ftc-reconfiguration-design.md
 */

namespace ctrl
{

/**
 * @brief Reconfiguring supervisor: classifies faults and switches ControllerStack entries.
 */
class FTCSupervisor : public IController
{
public:
    /**
     * @param stack Pre-populated ControllerStack (Supervisory mode); every entry that will be
     *        referenced by registerFaultResponse() must already be added via addController(),
     *        with no activationCondition (eligibility is driven entirely by this class).
     * @param fp Fault classifier parameters.
     * @param Ts Sample time [s].
     */
    FTCSupervisor(std::shared_ptr<ControllerStack> stack, const FaultDetectorParams &fp,
                  double Ts);

    /**
     * @brief Register which stack entry handles a given fault. FaultType::None registers the
     *        nominal/default controller - required exactly like every other fault type.
     * @param controllerName Must already have been added to the stack via addController().
     * @throws std::invalid_argument if controllerName is not found in the stack -
     *         ControllerStack::setActive() itself silently no-ops on an unknown name, so this
     *         class validates explicitly rather than letting a typo fail silently at runtime.
     */
    void registerFaultResponse(FaultType fault, const std::string &controllerName);

    /**
     * @brief Feed the latest residual triple. Call once per step BEFORE compute(), typically
     *        right after a KalmanFilter::update() call:
     *        feedResidual(kf.mismatch_score()-style innovation, u_prev, y_meas).
     */
    void feedResidual(double innovation, double u_cmd, double y_meas);

    double compute(double error) override;
    void reset() override;
    double sampleTime() const override { return Ts_; }

    FaultType currentFault() const { return currentFault_; }

private:
    std::shared_ptr<ControllerStack> stack_;
    FaultClassifier classifier_;
    std::array<std::string, 5> faultResponse_; // indexed by static_cast<int>(FaultType)
    FaultType currentFault_ = FaultType::None;
    FaultType lastApplied_ = FaultType::None;
    bool everApplied_ = false;
    double lastInnovation_ = 0.0, lastUCmd_ = 0.0, lastYMeas_ = 0.0;
    double u_prev_ = 0.0;
    double Ts_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(ftc_supervisor)
