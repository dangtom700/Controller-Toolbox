#pragma once
#include "boiler_plant.h"
#include "linearizer.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <array>
#include <memory>
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

} // namespace boiler
