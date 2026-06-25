#pragma once
#include "ControllerRegistry.h"
#include <Eigen/Dense>

/**
 * @file FaultClassifier.h
 * @brief Heuristic fault-type classifier over rolling residual statistics (Phase 3 Roadmap
 *        Phase 2 DT4).
 *
 * Distinguishes the four `tools/fault_injector.py` fault kinds using two structurally different
 * signatures recoverable from a scalar residual stream:
 *  - Actuator faults (loss, stuck) break the causal link between commanded u and measured y.
 *  - Sensor faults (bias, noise) leave that causal link intact but corrupt the residual's own
 *    distribution (persistent offset vs. elevated variance).
 *
 * This is a **heuristic** classifier, not a rigorous structured-residual FDI observer bank - a
 * genuinely ambiguous fault can still misclassify; see the design spec for the full rationale.
 *
 * @see docs/superpowers/specs/2026-06-25-ftc-reconfiguration-design.md
 */

namespace ctrl
{

/** @brief Fault categories FaultClassifier distinguishes. */
enum class FaultType
{
    None,
    SensorBias,
    SensorNoise,
    ActuatorLoss,
    ActuatorStuck
};

/** @brief Parameters for FaultClassifier. */
struct FaultDetectorParams
{
    double residual_threshold = 3.0; ///< Absolute RMS(innovation) above which a fault is suspected.
    int confirm_window = 5;           ///< Rolling history length (>= 3).
    double corr_threshold = 0.3;      ///< |corr(du_cmd, dy_meas)| below this -> actuator suspected.
    double bias_threshold = 2.0;      ///< |mean(innov)|/stddev(innov) above this -> SensorBias.
    double stuck_du_threshold = 1e-3; ///< stddev(u_cmd) below this -> ActuatorStuck.
};

/**
 * @brief Heuristic per-step fault classifier over a rolling (innovation, u_cmd, y_meas) window.
 */
class FaultClassifier
{
public:
    explicit FaultClassifier(const FaultDetectorParams &p = {});

    /** @brief Feed one new residual sample and return the classified fault type. */
    FaultType classify(double innovation, double u_cmd, double y_meas);

    /** @brief Clear rolling history. */
    void reset();

private:
    FaultDetectorParams p_;
    Eigen::VectorXd innovHist_, uHist_, yHist_; // fixed-size circular buffers, size confirm_window
    int count_ = 0;
    int head_ = 0;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(fault_classifier)
