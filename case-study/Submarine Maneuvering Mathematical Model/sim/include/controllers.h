#pragma once
#include "submarine_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

// Controller wrappers for the MARIN BB2 submarine autopilot case study.
//
// All controllers share:
//   delta_r = computeHeading(psi_ref, psi, v, r)   [rad]  rudder command
//   delta_s = computeDepth  (z_ref,   z,   theta)   [rad]  stern-plane command
//
// Depth control (computeDepth) is a shared two-loop PD/P structure identical
// for all 10 controllers.  Heading control (computeHeading) uses a
// study-specific lib/ algorithm.
//
// Sign conventions:
//   delta_r > 0  ->  starboard turn (ψ increases)
//   delta_s > 0  ->  nose down (theta increases, submarine dives)

namespace sub {

// Utility: wrap angle to (-pi, pi]
inline double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double computeHeading(double psi_ref, double psi,
                                  double v, double r) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;

    // Shared depth two-loop: outer P on z -> theta_ref; inner PID on theta -> deltas
    double computeDepth(double z_ref, double z, double theta, double Ts);
    void   resetDepth();

private:
    ctrl::DiscretePID depth_pid_{buildDepthPID(), 0.05};
    static ctrl::PIDParams buildDepthPID();
};

// ---------------------------------------------------------------------------
// 1. PID_ZN - Aggressive Ziegler-Nichols style PID
// ---------------------------------------------------------------------------
class PIDZNCtrl : public ControllerBase {
public:
    explicit PIDZNCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "PID_ZN"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 2. PID_Cons - Conservative PID (slower bandwidth, robust)
// ---------------------------------------------------------------------------
class PIDConsCtrl : public ControllerBase {
public:
    explicit PIDConsCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "PID_Cons"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 3. MPC - Discrete MPC on linearised 3-state heading model [v, r, ψ]
// ---------------------------------------------------------------------------
class MPCHeadingCtrl : public ControllerBase {
public:
    explicit MPCHeadingCtrl(const PlantParams& p, double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "MPC"; }
private:
    ctrl::DiscreteMPC mpc_;
    double u_prev_ = 0.0;
    Eigen::VectorXd x_hat_;   // estimated state [v, r, ψ]
};

// ---------------------------------------------------------------------------
// 4. LQR - Full-state LQR on [v, r, ψ_error]
// ---------------------------------------------------------------------------
class LQRHeadingCtrl : public ControllerBase {
public:
    explicit LQRHeadingCtrl(const PlantParams& p, double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override {}  // LQR is stateless
    std::string name() const override { return "LQR"; }
private:
    ctrl::DiscreteLQR lqr_;
};

// ---------------------------------------------------------------------------
// 5. ADRC - 2nd-order Active Disturbance Rejection Control
// ---------------------------------------------------------------------------
class ADRCHeadingCtrl : public ControllerBase {
public:
    explicit ADRCHeadingCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
};

// ---------------------------------------------------------------------------
// 6. SMC - Sliding Mode Control
//    Note: DiscreteSMC::compute() takes (y - ref) = (ψ - ψ_ref) per CLAUDE.md
// ---------------------------------------------------------------------------
class SMCHeadingCtrl : public ControllerBase {
public:
    explicit SMCHeadingCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "SMC"; }
private:
    ctrl::DiscreteSMC smc_;
};

// ---------------------------------------------------------------------------
// 7. MRAC - Model Reference Adaptive Control
//    Note: MRACController::compute() takes y_plant (not error). Call
//    setReference(r) before each compute().
// ---------------------------------------------------------------------------
class MRACHeadingCtrl : public ControllerBase {
public:
    explicit MRACHeadingCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "MRAC"; }
private:
    ctrl::MRACController mrac_;
};

// ---------------------------------------------------------------------------
// 8. L1Adaptive - L1 Adaptive Control
//    Note: L1AdaptiveController::compute() takes y_plant. Call
//    setReference(r) before each compute().
// ---------------------------------------------------------------------------
class L1HeadingCtrl : public ControllerBase {
public:
    explicit L1HeadingCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    ctrl::L1AdaptiveController l1_;
};

// ---------------------------------------------------------------------------
// 9. GainSched - Gain-scheduled PID (3 operating points on |ψ_error|)
//    Large error -> aggressive gains; small error -> conservative gains.
// ---------------------------------------------------------------------------
class GainSchedHeadingCtrl : public ControllerBase {
public:
    explicit GainSchedHeadingCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "GainSched"; }
private:
    ctrl::GainScheduledController sched_;
};

// ---------------------------------------------------------------------------
// 10. Smith - Smith Predictor (compensates 1-step heading sensor delay)
// ---------------------------------------------------------------------------
class SmithHeadingCtrl : public ControllerBase {
public:
    explicit SmithHeadingCtrl(double Ts);
    double computeHeading(double psi_ref, double psi,
                          double v, double r) override;
    void reset() override;
    std::string name() const override { return "Smith"; }
private:
    ctrl::SmithPredictor smith_;
};

} // namespace sub
