#pragma once
#include "boiler_plant.h"
#include "linearizer.h"
#include <ControllerToolbox.h>
#include "NonlinearMPC.h"
#include "FeedbackLinearisation.h"
#include "MovingHorizonEstimator.h"
#include "LPVSystemID.h"
#include "SubspaceID.h"
#include "AutoGainScheduler.h"
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <optional>
#include <string>

// All boiler controllers share this interface:
//   compute(ref_dy, dy) -> du
//
//   ref_dy = desired output deviation [dy1, dy2, dy3]
//   dy     = measured output deviation [y1-y1_op, y2-y2_op, y3-y3_op]
//   return = control increment du = [du1, du2, du3]
//
// Absolute valve applied in runner: u = clamp(u_op + du, 0, 1)

namespace boiler {

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                                    const Eigen::Vector3d& dy) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};

// -- 1. PID -------------------------------------------------------------------
class PIDController : public ControllerBase {
public:
    explicit PIDController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "PID"; }
private:
    std::array<ctrl::DiscretePID, 3> pids_;
};

// -- 2. LQR -------------------------------------------------------------------
class LQRController : public ControllerBase {
public:
    explicit LQRController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "LQR"; }
private:
    ctrl::DiscreteLQR lqr_;
    Eigen::Matrix3d   Nbar_;
};

// -- 3. LQG -------------------------------------------------------------------
class LQGController : public ControllerBase {
public:
    explicit LQGController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "LQG"; }
private:
    ctrl::DiscreteLQG lqg_;
    Eigen::Vector3d   du_prev_;
};

// -- 4. MPC -------------------------------------------------------------------
class MPCController : public ControllerBase {
public:
    explicit MPCController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "MPC"; }
private:
    ctrl::DiscreteMPC mpc_;
};

// -- 5. SMC -------------------------------------------------------------------
class SMCController : public ControllerBase {
public:
    explicit SMCController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "SMC"; }
private:
    std::array<ctrl::DiscreteSMC, 3> smcs_;
};

// -- 6. ESC -------------------------------------------------------------------
class ESCController : public ControllerBase {
public:
    explicit ESCController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "ESC"; }
private:
    ctrl::ExtremumSeeker esc_;
    const OperatingPoint& op_;
};

// -- 7. ADRC ------------------------------------------------------------------
class ADRCController : public ControllerBase {
public:
    explicit ADRCController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "ADRC"; }
private:
    std::array<ctrl::DiscreteADRC, 3> adrcs_;
    const OperatingPoint& op_;
};

// -- 8. LeadLag + PID ---------------------------------------------------------
class LeadLagPIDController : public ControllerBase {
public:
    explicit LeadLagPIDController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "LeadLag-PID"; }
private:
    std::array<ctrl::DiscreteLeadLag, 3> leadlags_;
    std::array<ctrl::DiscretePID, 3>     pids_;
};

// -- 9. Smith Predictor -------------------------------------------------------
class SmithPredictorController : public ControllerBase {
public:
    explicit SmithPredictorController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "SmithPredictor"; }
private:
    std::array<ctrl::SmithPredictor, 3> sps_;
};

// -- 10. GPC + RLS ------------------------------------------------------------
class GPCController : public ControllerBase {
public:
    explicit GPCController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "GPC-RLS"; }
private:
    std::array<ctrl::GeneralizedPredictiveController, 3> gpcs_;
    std::array<ctrl::RecursiveLeastSquares, 3>           rls_;
    Eigen::Vector3d u_prev_;
    int step_count_ = 0;
    static constexpr int kRLSUpdateInterval = 50;
    static constexpr int kRLSWarmup         = 100;
};

// -- 11. EKF-LQR --------------------------------------------------------------
class EKFLQRController : public ControllerBase {
public:
    explicit EKFLQRController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "EKF-LQR"; }
private:
    ctrl::DiscreteLQR lqr_;
    ctrl::ExtendedKalmanFilter ekf_;
    Eigen::Vector3d du_prev_;
    const OperatingPoint& op_;
};

// -- 12. UKF-LQR --------------------------------------------------------------
class UKFLQRController : public ControllerBase {
public:
    explicit UKFLQRController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "UKF-LQR"; }
private:
    ctrl::DiscreteLQR lqr_;
    ctrl::UnscentedKalmanFilter ukf_;
    Eigen::Vector3d du_prev_;
    const OperatingPoint& op_;
};

// -- 13. FuzzyPID -------------------------------------------------------------
class FuzzyPIDController : public ControllerBase {
public:
    explicit FuzzyPIDController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "FuzzyPID"; }
private:
    std::array<ctrl::FuzzyPID, 3> fuzzies_;
};

// -- 14. FuzzySup-MPC ---------------------------------------------------------
class FuzzySupMPCController : public ControllerBase {
public:
    explicit FuzzySupMPCController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "FuzzySup-MPC"; }
private:
    ctrl::StateSpace ss_;
    const OperatingPoint& op_;
    std::array<ctrl::DiscreteMPC, 3>        mpcs_;
    std::array<ctrl::FuzzySupervisor, 3>    supervisors_;
    // current plant state for re-linearisation (updated from dy each step)
    Eigen::Vector3d x_current_;
};

// -- 15. Supervisory Stack (SMC -> LQR) ---------------------------------------
class SupervisoryStackController : public ControllerBase {
public:
    explicit SupervisoryStackController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "SupervisoryStack"; }
private:
    std::array<ctrl::ControllerStack, 3> stacks_;
};

// -- 16. Additive Stack (PID + LeadLag) ---------------------------------------
class AdditiveStackController : public ControllerBase {
public:
    explicit AdditiveStackController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "AdditiveStack"; }
private:
    std::array<ctrl::ControllerStack, 3> stacks_;
    int step_count_ = 0;
    static constexpr int kFadeSteps = 300;
};

// -- 17. Weighted Stack (PID + LQR blended by pressure) -----------------------
class WeightedStackController : public ControllerBase {
public:
    explicit WeightedStackController(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "WeightedStack"; }
private:
    std::array<ctrl::ControllerStack, 3> stacks_;
    const OperatingPoint& op_;
};

// -- 18. Repetitive Controller ------------------------------------------------
class RepetitiveCtrl : public ControllerBase {
public:
    explicit RepetitiveCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "RepetitiveCtrl"; }
private:
    std::array<ctrl::RepetitiveController, 3> rcs_;
};

// -- 19. MRACBoilerCtrl -------------------------------------------------------
// 3* MRAC (one per diagonal channel); adapts to gain changes across operating points.
// Reference model: a_m=0.80, b_m=0.20 (DC gain = 1). compute() passes absolute y/u.
class MRACBoilerCtrl : public ControllerBase {
public:
    explicit MRACBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "MRAC"; }
private:
    std::array<ctrl::MRACController, 3> mracs_;
    OperatingPoint op_;
};

// -- 20. HinfBoilerCtrl -------------------------------------------------------
// 3* H-inf mixed-sensitivity per diagonal channel; robust to pressure disturbances.
// Falls back to PID per-channel if synthesis is infeasible.
class HinfBoilerCtrl : public ControllerBase {
public:
    explicit HinfBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "H-inf"; }
private:
    std::array<std::optional<ctrl::DiscreteHinf>, 3> hinfs_;
    std::array<ctrl::DiscretePID, 3>                  fallback_pids_;
};

// -- 21. AdaptiveSPBoilerCtrl -------------------------------------------------
// 3* Adaptive Smith Predictor; online cross-correlation estimates dead-time per channel.
// Compensates for valve-rate-limited effective delay that changes with operating point.
class AdaptiveSPBoilerCtrl : public ControllerBase {
public:
    explicit AdaptiveSPBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "AdaptiveSP"; }
private:
    std::array<ctrl::AdaptiveSmithPredictor, 3> sps_;
    OperatingPoint op_;
};

// -- 22. NMPCBoilerCtrl -------------------------------------------------------
// NonlinearMPC on Bell-Astrom 3-state deviation dynamics (Euler-integrated).
// RTI horizon Np=10, Nu=3. C = ss.C maps state to output for tracking.
// Uses dy as state proxy (valid near op where Cd approx = I for channels 0,1).
class NMPCBoilerCtrl : public ControllerBase {
public:
    explicit NMPCBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "NMPC"; }
private:
    ctrl::NonlinearMPC nmpc_;
    OperatingPoint op_;
};

// -- 23. FLBoilerCtrl ---------------------------------------------------------
// 3* FeedbackLinearisationController, one per diagonal channel.
// Channel 0 (u1->y1=x1): g=0.9, nonlinear f.
// Channel 1 (u2->y2=x2): g=0.073*x1^(9/8), nonlinear f.
// Channel 2 (u3->x3):    g=141/85, linear f (x3 water level proxy for y3).
// Inner PID: Kp=0.5 for the linearized virtual plant.
class FLBoilerCtrl : public ControllerBase {
public:
    explicit FLBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "FL"; }
private:
    std::array<ctrl::FeedbackLinearisationController, 3> fls_;
    OperatingPoint op_;
};

// -- 24. MHELQRBoilerCtrl -----------------------------------------------------
// Moving Horizon Estimator (N=10) for state estimation + DiscreteLQR feedback.
// MHE uses process-noise box constraints [-0.5, 0.5] on w.
class MHELQRBoilerCtrl : public ControllerBase {
public:
    explicit MHELQRBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "MHE-LQR"; }
private:
    ctrl::MovingHorizonEstimator mhe_;
    ctrl::DiscreteLQR            lqr_;
    Eigen::Vector3d              du_prev_;
    bool                         initialised_ = false;
};

// -- 25. LPVGSBoilerCtrl ------------------------------------------------------
// LPV gain-scheduled controller: identifies A(rho), B(rho) polynomial
// dependence on pressure x1, then designs GainScheduledController over
// pressure range [60, 150] bar with NearestNeighbor switching.
class LPVGSBoilerCtrl : public ControllerBase {
public:
    explicit LPVGSBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "LPV-GS"; }
private:
    std::shared_ptr<ctrl::GainScheduledController> sched_;
    const OperatingPoint& op_;
};

// -- 26. SubspaceIDLQGBoilerCtrl -----------------------------------------------
// SubspaceID (N4SID/MOESP) identifies a 3rd-order model from simulated
// step-response data, then designs DiscreteLQG on the identified model.
// Demonstrates closed-loop model identification pipeline.
class SubspaceIDLQGBoilerCtrl : public ControllerBase {
public:
    explicit SubspaceIDLQGBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "SubspaceID-LQG"; }
private:
    ctrl::DiscreteLQG lqg_;
    Eigen::Vector3d   du_prev_;
};

// -- 27. AutoGSBoilerCtrl ------------------------------------------------------
// Automated gain-scheduled LQR built by buildAutoGainScheduler.
// Sweeps pressure x1 \in [60, 150] bar, clusters by nu-gap, designs LQR per cluster.
// Scheduling variable = current x1 = x1_op + dy[0].
class AutoGSBoilerCtrl : public ControllerBase {
public:
    explicit AutoGSBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op);
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_dy,
                            const Eigen::Vector3d& dy) override;
    void reset() override;
    std::string name() const override { return "AutoGS"; }
private:
    std::shared_ptr<ctrl::GainScheduledController> sched_;
    OperatingPoint op_;
};

} // namespace boiler
