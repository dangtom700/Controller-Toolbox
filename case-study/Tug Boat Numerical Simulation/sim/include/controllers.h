#pragma once
#include "plant_parameters.h"
#include "DiscretePID.h"
#include "KalmanFilter.h"
#include "DiscreteSMC.h"
#include "DiscreteMPC.h"
#include "ExtremumSeeker.h"
#include "FuzzyLogic.h"
#include "PlantModel.h"
#include "RecursiveLeastSquares.h"
#include "SystemAnalysis.h"
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>

// Controller wrappers for the tug-barge simulation.
// All controllers share the same interface:
//
//   compute(ref, state) -> tau_c = [tau_x, tau_y, tau_psi]
//
// ref   = [x_target, y_target, psi_target]
// state = [x, y, psi, u, v, r]   (measured or filtered)
//
// Output is the body-frame generalised force command BEFORE saturation.
// Saturation to [-2e6, 2e6] N and [-5e7, 5e7] N.m is applied in the runner.

namespace tug {

// -- Output saturation limits -------------------------------------------------
static constexpr double TAU_XY_MAX  = 2.0e6;   // N
static constexpr double TAU_PSI_MAX = 5.0e7;   // N.m

inline Eigen::Vector3d saturateTau(const Eigen::Vector3d& tau)
{
    return Eigen::Vector3d(
        std::clamp(tau(0), -TAU_XY_MAX,  TAU_XY_MAX),
        std::clamp(tau(1), -TAU_XY_MAX,  TAU_XY_MAX),
        std::clamp(tau(2), -TAU_PSI_MAX, TAU_PSI_MAX));
}

// -- Base class ----------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                                    const Eigen::Matrix<double,6,1>& state) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};

// -- Mode 1: PID Baseline ------------------------------------------------------
class PIDController : public ControllerBase {
public:
    explicit PIDController(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "PID"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::DiscretePID, 3> pids_;  // x, y, psi
};

// -- Mode 2: Kalman Filter + PID -----------------------------------------------
class KFPIDController : public ControllerBase {
public:
    explicit KFPIDController(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "KF-PID"; }
private:
    const PlantParameters& pp_;
    std::unique_ptr<ctrl::KalmanFilter> kf_;
    std::array<ctrl::DiscretePID, 3>    pids_;
    Eigen::Vector3d u_prev_;     // last tau_c / scale for KF input

    ctrl::StateSpace buildPlantSS() const;
};

// -- Mode 3: Sliding Mode Controller ------------------------------------------
// Implements paper Eqs. (24-27): sliding surface with integral, equivalent
// control (model-cancellation), and switching control with boundary layer.
class SMCController : public ControllerBase {
public:
    explicit SMCController(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "SMC"; }
private:
    const PlantParameters& pp_;
    // Per-axis parameters matching paper Table 4
    Eigen::Vector3d Lambda_;    // [0.05, 0.05, 0.10]
    Eigen::Vector3d Ki_s_;      // [1e-4, 1e-4, 1e-4]
    Eigen::Vector3d K_sw_;      // [8e5, 8e5, 2e7]
    Eigen::Vector3d Phi_;       // [0.5, 0.5, 0.05]
    Eigen::Vector3d e_prev_;
    Eigen::Vector3d integral_;

    static double sat(double s, double phi);
};

// -- Mode 4: Model Predictive Control -----------------------------------------
class MPCController : public ControllerBase {
public:
    explicit MPCController(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "MPC"; }
private:
    const PlantParameters& pp_;
    // One DiscreteMPC per axis (SISO, decoupled linearized model)
    std::array<std::unique_ptr<ctrl::DiscreteMPC>, 3> mpcs_;

    ctrl::StateSpace buildAxisSS(int axis) const;
};

// -- Mode 5: Extremum Seeking Control -----------------------------------------
// Model-free gradient descent on per-axis IAE cost.
class ESCController : public ControllerBase {
public:
    explicit ESCController(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "ESC"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::ExtremumSeeker, 3> escs_;  // x, y, psi
    Eigen::Vector3d e_prev_;
};

// -- Mode 6: Fuzzy PID Controller ---------------------------------------------
// Three independent FuzzyPID loops (one per axis) using the Toolbox's
// ctrl::FuzzyPID.  Each loop runs a 25-rule Mamdani FuzzyPD block plus a
// crisp integral accumulator with back-calculation anti-windup.
//
// Scaling parameters are chosen so the normalised universe [-1,1] maps to
// physically meaningful error ranges for this barge system.
class FuzzyPIDController : public ControllerBase {
public:
    explicit FuzzyPIDController(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "FuzzyPID"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::FuzzyPID, 3> fuzzy_pids_;  // x, y, psi
};

// -- Fuzzy Supervisor + MPC  ---------------------------------------------------
// Wraps the MPC controller and attaches a per-axis FuzzySupervisor.
// At each step the supervisor evaluates whether the closed-loop error magnitude
// and its trend indicate that the MPC's internal linearised plant model has
// drifted far enough from current dynamics to warrant re-linearisation.
//
// Re-linearisation trigger:
//   1. FuzzySupervisor fires (signal > threshold AND cooldown expired).
//   2. Runner calls relinearize() which rebuilds the per-axis state-space model
//      from M_re, D_re evaluated at the CURRENT operating velocity nu.
//   3. New StateSpace is pushed to each DiscreteMPC via setPlant().
//
// This answers the question "does the plant have to be re-linearized?" at runtime
// rather than at a fixed schedule, adapting to actual disturbance conditions.
class FuzzySupervised_MPC : public ControllerBase {
public:
    explicit FuzzySupervised_MPC(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "FuzzySup-MPC"; }

    // Expose last supervisor decisions for logging
    std::array<ctrl::SupervisorDecision, 3> lastDecisions() const { return decisions_; }
    int relinearizeCount() const { return relinearize_count_; }

private:
    const PlantParameters& pp_;

    // One MPC per axis (same structure as MPCController)
    std::array<std::unique_ptr<ctrl::DiscreteMPC>, 3> mpcs_;

    // One FuzzySupervisor per axis - they watch independent error channels
    std::array<ctrl::FuzzySupervisor, 3> supervisors_;

    // Last decisions (for logging / diagnostics)
    std::array<ctrl::SupervisorDecision, 3> decisions_;
    int relinearize_count_ = 0;

    ctrl::StateSpace buildAxisSS(int axis, const Eigen::Vector3d& nu) const;
    void relinearize(int axis, const Eigen::Vector3d& nu);
};

} // namespace tug
