#pragma once
#include "smismo_plant.h"
#include <ControllerToolbox.h>
#include "NonlinearMPC.h"
#include "FeedbackLinearisation.h"
#include <Eigen/Dense>
#include <optional>
#include <string>

// All controllers share the interface:
//   compute(y, r) -> u_cmd  in [-1, 1]
//
// y = plant.measure() = [x, v, p1, p2]  (raw)
// r = scalar position reference [m]
//
// u_cmd is applied as u1=u2=u_cmd (coupled meter-in/meter-out).
// All linear controllers use the 2-state linearised model [x, v] to avoid
// DARE issues from the indirect (pressure-mediated) position actuation.

namespace smismo {

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double compute(const Eigen::Vector4d& y, double r) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};

// ---- 1. PID -----------------------------------------------------------------
// PI controller on position error (derivative disabled to prevent reference kick).
class PIDCtrl : public ControllerBase {
public:
    PIDCtrl(const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "PID"; }
private:
    ctrl::DiscretePID pid_;
};

// ---- 2. LQR -----------------------------------------------------------------
// LQR on 2-state [x,v] model + position integral for zero steady-state error.
class LQRCtrl : public ControllerBase {
public:
    LQRCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "LQR"; }
private:
    ctrl::DiscreteLQR lqr_;
    SmismoOperatingPoint op_;
    double Ts_;
    double pos_integral_{0.0};
    static constexpr double Ki = 0.4;   // outer integral gain
};

// ---- 3. LQG -----------------------------------------------------------------
// LQG on 2-state [x,v] model with position-only measurement (Kalman estimates v).
class LQGCtrl : public ControllerBase {
public:
    LQGCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "LQG"; }
private:
    ctrl::DiscreteLQG lqg_;
    SmismoOperatingPoint op_;
    double Ts_;
    double pos_integral_{0.0};
    Eigen::VectorXd u_prev_;
    static constexpr double Ki = 0.4;
};

// ---- 4. SMC -----------------------------------------------------------------
// Sliding mode on position error.
class SMCCtrl : public ControllerBase {
public:
    SMCCtrl(const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "SMC"; }
private:
    ctrl::DiscreteSMC smc_;
};

// ---- 5. ADRC ----------------------------------------------------------------
// 2nd-order ADRC treating valve/pressure dynamics as total disturbance.
class ADRCCtrl : public ControllerBase {
public:
    ADRCCtrl(double b0, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
};

// ---- 6. TubeMPC -------------------------------------------------------------
// Robust MPC on 2-state model with position tracking and bounded load disturbance.
class TubeMPCCtrl : public ControllerBase {
public:
    TubeMPCCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "TubeMPC"; }
private:
    ctrl::TubeMPC tmpc_;
    SmismoOperatingPoint op_;
};

// ---- 7. LeadLagPIDCtrl -------------------------------------------------------
// Phase-lead pre-filter (zero=10, pole=50 rad/s) in series with PI.
// Boosts bandwidth without derivative kick on reference steps.
class LeadLagPIDCtrl : public ControllerBase {
public:
    LeadLagPIDCtrl(const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "LeadLag-PID"; }
private:
    ctrl::DiscreteLeadLag leadlag_;
    ctrl::DiscretePID     pid_;
};

// ---- 8. GPCRLSCtrl -----------------------------------------------------------
// Generalized Predictive Control (Np=50, Nu=10) with online RLS identification.
// RLS(na=2, nb=1, lambda=0.995) tracks gain variation as cylinder position changes.
// Model hot-swapped via setPlant() every kRLSUpdateInterval steps after warmup.
class GPCRLSCtrl : public ControllerBase {
public:
    GPCRLSCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "GPC-RLS"; }
private:
    ctrl::GeneralizedPredictiveController gpc_;
    ctrl::RecursiveLeastSquares           rls_;
    SmismoOperatingPoint op_;
    double u_prev_    = 0.0;
    int    step_count_ = 0;
    static constexpr int kRLSUpdateInterval = 50;
    static constexpr int kRLSWarmup         = 100;
};

// ---- 9. MRACSmismoCtrl -------------------------------------------------------
// MRAC with sigma-modification; adapts to hydraulic gain variation with position.
// Reference model: a_m=0.85, b_m=0.15 (approx. 2-state closed-loop pole at 0.85).
// compute(y, r): takes plant measurement, not error (MRACController API).
class MRACSmismoCtrl : public ControllerBase {
public:
    MRACSmismoCtrl(const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "MRAC"; }
private:
    ctrl::MRACController mrac_;
};

// ---- 10. GainScheduledSmismoCtrl ---------------------------------------------
// Five position-dependent PI controllers (x=0.05..0.25m), NearestNeighbor mode.
// Gains scaled inversely with hydraulic gain Kv(x) from linearise2() at each point.
class GainScheduledSmismoCtrl : public ControllerBase {
public:
    GainScheduledSmismoCtrl(const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "GainScheduledLQR"; }
private:
    ctrl::GainScheduledController sched_;
    SmismoOperatingPoint          op_;
};

// ---- 11. EKFLQRSmismoCtrl ----------------------------------------------------
// EKF on 4-state linear model (linearise4) for velocity/pressure estimation.
// LQR on 2-state [x,v] design (avoids DARE instability from 4-state coupling).
// Outer position integral ensures zero steady-state error.
class EKFLQRSmismoCtrl : public ControllerBase {
public:
    EKFLQRSmismoCtrl(const ctrl::StateSpace& ss4, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "EKF-LQR"; }
private:
    ctrl::ExtendedKalmanFilter ekf_;
    ctrl::DiscreteLQR          lqr_;
    SmismoOperatingPoint       op_;
    double Ts_;
    double u_prev_       = 0.0;
    double pos_integral_ = 0.0;
    static constexpr double Ki = 0.4;
};

// ---- 12. HinfSmismoCtrl ------------------------------------------------------
// H-infinity mixed-sensitivity (W1=tracking, W2=effort, W3=robustness) on 2-state model.
// Robust to bounded 1 kN load disturbance (S3 scenario).
// Falls back to PID if synthesis is infeasible.
class HinfSmismoCtrl : public ControllerBase {
public:
    HinfSmismoCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "H-inf"; }
private:
    std::optional<ctrl::DiscreteHinf> hinf_;
    ctrl::DiscretePID                 fallback_pid_;  // used if synthesis fails
};

// ---- 13. NMPCSmismoCtrl -------------------------------------------------------
// NonlinearMPC on 4-state model with position-dependent volume nonlinearity.
// State: [pos_dev, vel_dev, p1_dev, p2_dev]. Output: position only (C=[1,0,0,0]).
// Np=30, Nu=5. Captures V1(x), V2(x) volume variation across stroke.
class NMPCSmismoCtrl : public ControllerBase {
public:
    NMPCSmismoCtrl(const ctrl::StateSpace& ss4, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "NMPC"; }
private:
    ctrl::NonlinearMPC nmpc_;
    SmismoOperatingPoint op_;
};

// ---- 14. FLSmismoCtrl --------------------------------------------------------
// FeedbackLinearisationController as velocity controller with outer P position loop.
// f(x, u_prev) = -Bv/M * vel  (velocity damping drift term)
// g(x, u_prev) = b0            (effective acceleration gain, constant)
// Inner PD tracks velocity reference; outer P generates velocity reference from
// position error: v_ref = Kp_pos * (r - x).
class FLSmismoCtrl : public ControllerBase {
public:
    FLSmismoCtrl(double b0, const SmismoOperatingPoint& op, double Ts);
    double compute(const Eigen::Vector4d& y, double r) override;
    void reset() override;
    std::string name() const override { return "FL"; }
private:
    ctrl::FeedbackLinearisationController fl_;
    SmismoOperatingPoint op_;
    static constexpr double Kp_pos = 20.0;   // outer position loop gain [1/s]
};

} // namespace smismo
