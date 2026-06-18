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
#include "DiscreteADRC.h"
#include "RepetitiveController.h"
#include "DiscreteLQR.h"
#include "DiscreteLQG.h"
#include "ControllerTuner.h"
#include "TubeMPC.h"
#include "ExtendedKalmanFilter.h"
#include "MRACController.h"
#include "NonlinearMPC.h"
#include "AutoGainScheduler.h"
#include "L1AdaptiveController.h"
#include "ScenarioMPC.h"
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <optional>
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

    // Called after thrust allocation, with the body-frame force actually delivered
    // to the plant (tau_applied = B * T_clamped).  MPC controllers use this to
    // correct their internal u_prev_ so x_hat_ stays synchronised with the real
    // plant even when the allocator saturates or redistributes thrust.
    // Default is a no-op; only MPC-based controllers override this.
    virtual void notifyApplied(const Eigen::Vector3d& /*tau_applied*/) {}
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

    // Feeds the allocator-applied force back to each per-axis DiscreteMPC so
    // u_prev_ reflects the actual plant input rather than the commanded value.
    void notifyApplied(const Eigen::Vector3d& tau_applied) override;

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

    void notifyApplied(const Eigen::Vector3d& tau_applied) override;

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

// -- Mode 8: ADRC (3-axis) ----------------------------------------------------
// 2nd-order ADRC per axis. ESO estimates coupling + drag as total disturbance.
// b0_xy = 1/M_re(i,i); b0_psi = 1/M_re(2,2).
// omega_o=0.5 rad/s satisfies omega_o*Ts = 0.25 < 0.5 (backward Euler stable).
class ADRCTugCtrl : public ControllerBase {
public:
    explicit ADRCTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "ADRC"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::DiscreteADRC, 3> adrcs_;
};

// -- Mode 9: RepetitiveController (3-axis) ------------------------------------
// Plug-in repetitive controller on top of PID baseline.
// Period estimated from JONSWAP peak period Tp (from plant_params.json).
// Cancels periodic wave drift in S3/S4 scenarios.
class RepetitiveTugCtrl : public ControllerBase {
public:
    explicit RepetitiveTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "RepetitiveCtrl"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::RepetitiveController, 3> rcs_;
};

// -- Mode 10: LQR (6-state MIMO) -----------------------------------------------
// Bryson-tuned full-state LQR on the linearised 6-state model.
// xmax=[10,10,0.1,1,1,0.05]; umax=[TAU_XY_MAX,TAU_XY_MAX,TAU_PSI_MAX].
// x_ref = [ref(0), ref(1), ref(2), 0, 0, 0] (position track, zero velocity).
class LQRTugCtrl : public ControllerBase {
public:
    explicit LQRTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override {}   // LQR is stateless at runtime
    std::string name() const override { return "LQR"; }
private:
    const PlantParameters& pp_;
    ctrl::DiscreteLQR lqr_;
};

// -- Mode 11: LQG (6-state KF + LQR) ------------------------------------------
// DiscreteLQG: Kalman filter estimates 6-state, LQR provides optimal feedback.
// Same Bryson weights as LQRTugCtrl; KF noise tuned to GPS/IMU spec.
class LQGTugCtrl : public ControllerBase {
public:
    explicit LQGTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "LQG"; }
private:
    const PlantParameters& pp_;
    ctrl::DiscreteLQG lqg_;
    Eigen::VectorXd u_prev_;
};

// -- Mode 12: TubeMPC (3x per-axis robust MPC) ---------------------------------
// Tube-MPC on the same decoupled 2-state per-axis model as MPCController.
// K = -K_lqr (LQR stabilising feedback); wMax sized to JONSWAP wave disturbance.
// Guarantees state stays within computed tube under bounded wave forcing.
class TubeMPCTugCtrl : public ControllerBase {
public:
    explicit TubeMPCTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "TubeMPC"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::TubeMPC, 3> tmpcs_;
};

// -- Mode 13: EKF-LQR ----------------------------------------------------------
// EKF propagates nonlinear J(psi)*nu kinematics and body-frame dynamics.
// LQR uses the linearised 6-state model from KFPIDController.buildPlantSS().
// Provides better state estimates when heading angle is significant.
class EKFLQRTugCtrl : public ControllerBase {
public:
    explicit EKFLQRTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "EKF-LQR"; }
private:
    const PlantParameters& pp_;
    ctrl::ExtendedKalmanFilter ekf_;
    ctrl::DiscreteLQR lqr_;
    Eigen::VectorXd u_prev_;
};

// -- Mode 14: MRAC (3-axis adaptive) ------------------------------------------
// 3x MRACController, one per axis. Very conservative adaptation (gamma=1e-8)
// and large theta_max to prevent instability on the slow double-integrator-like
// ship dynamics (relative degree 2 from force to position).
class MRACTugCtrl : public ControllerBase {
public:
    explicit MRACTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "MRAC"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::MRACController, 3> mracs_;
};

// -- Mode 15: AutoGS (automated gain-scheduled LQR on surge axis) -------------
// buildAutoGainScheduler sweeps surge speed [0,1.5] m/s, clusters by nu-gap,
// and designs DiscreteLQR at each representative.  Scheduling variable = |u_v|.
// The other axes remain as PID loops.
class AutoGSTugCtrl : public ControllerBase {
public:
    explicit AutoGSTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "AutoGS-LQR"; }
private:
    const PlantParameters& pp_;
    std::shared_ptr<ctrl::GainScheduledController> sched_surge_;
    std::array<ctrl::DiscretePID, 2> pids_yw_;  // y and psi remain PID
    // Capture current surge-axis state for LQRAdapter callbacks
    mutable Eigen::VectorXd x_surge_;  // [e_surge, u_vel]
};

// -- Mode 16: NonlinearMPC (6-state ship dynamics) ----------------------------
// RTI-NMPC on the discrete-time nonlinear tug model (Euler-integrated J(psi)*nu
// kinematics + linear body-force dynamics). Np=20, Nu=5.
// C = [I_3, 0_{3x3}] tracks position only; input in kN for numerical stability.
class NMPCTugCtrl : public ControllerBase {
public:
    explicit NMPCTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "NMPC"; }
private:
    const PlantParameters& pp_;
    ctrl::NonlinearMPC nmpc_;
};

// -- Mode 17: L1Adaptive (3-axis) ---------------------------------------------
// One L1AdaptiveController per axis. Reference model: a_m=0.97, b_m=0.03
// (same as MRACTugCtrl). omega_c=0.05 rad/s LP filter; sigma_max=1e8.
// Output in N/N.m; saturateTau applied.
class L1AdaptiveTugCtrl : public ControllerBase {
public:
    explicit L1AdaptiveTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::L1AdaptiveController, 3> l1s_;
};

// -- Mode 18: ScenarioMPC (3-axis per-axis stochastic MPC) --------------------
// One ScenarioMPC per axis using the same 2-state [e,nu] model as TubeMPCTugCtrl.
// Input in kN; Np=60, Nu=5, N_samples=30. Sigma_w from JONSWAP wave disturbance.
// Output scaled back to N/N.m; saturateTau applied.
class ScenarioMPCTugCtrl : public ControllerBase {
public:
    explicit ScenarioMPCTugCtrl(const PlantParameters& p);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                            const Eigen::Matrix<double,6,1>& state) override;
    void reset() override;
    std::string name() const override { return "ScenarioMPC"; }
private:
    const PlantParameters& pp_;
    std::array<ctrl::ScenarioMPC, 3> smpcs_;
};

} // namespace tug
