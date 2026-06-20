#pragma once
// controllers.h - 12-controller roster for the Bouyancy-Driven Airship in Vertical Plane.
//
// Interface: compute(x, theta_ref, rp1_ref, m0) -> u [N]
//   x        : full 6-state [theta, q, rp1, w, v1, v3]
//   theta_ref: pitch attitude setpoint [rad]
//   rp1_ref  : slider trim/design reference [m] (used by LQR/MPC/AirshipFBLCtrl; ignored by
//              the purely-error-driven controllers)
//   m0       : current net-lift parameter [kg] (only AirshipFBLCtrl's exact-dynamics probe
//              needs this; every other controller ignores it)
//
// Sign convention (see README "Governing Equations" / CLAUDE.md Sign Conventions table):
//   compute(theta_ref - theta)  for PID/ADRC/NeuralPID/GainScheduled/ILC's inner PID
//   compute(theta - theta_ref)  for SMC (this repo's documented SMC convention, y - ref)
//   set_reference(theta_ref) + compute(theta)  for MRAC/L1Adaptive
// Each wrapper class below encapsulates its own sign handling internally - the simulation
// runner just calls compute(x, theta_ref, rp1_ref, m0) uniformly.
#include "bouyancy_driven_airship_in_vertical_plan_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <limits>

namespace bouyancydrivenairshipinverticalplan {

inline double clampU(const PlantParams& p, double u) { return std::clamp(u, p.u_min, p.u_max); }

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double      compute(const State& x, double theta_ref, double rp1_ref, double m0) = 0;
    virtual void        reset() = 0;
    virtual std::string name() const = 0;
};

// ---------------------------------------------------------------------------
// 1. OpenLoop - u = 0 baseline
// ---------------------------------------------------------------------------
class OpenLoopCtrl : public ControllerBase {
public:
    explicit OpenLoopCtrl(const PlantParams&) {}
    double      compute(const State&, double, double, double) override { return 0.0; }
    void        reset() override {}
    std::string name() const override { return "OpenLoop"; }
};

// ---------------------------------------------------------------------------
// 2. PID - compute(theta_ref - theta)
// ---------------------------------------------------------------------------
class PIDAirshipCtrl : public ControllerBase {
public:
    explicit PIDAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "PID"; }
private:
    PlantParams p_;
    ctrl::DiscretePID pid_;
    double last_theta_ref_ = std::numeric_limits<double>::quiet_NaN();
};

// ---------------------------------------------------------------------------
// 3. ADRC - compute(theta_ref - theta); b0 negative (negative-gain plant)
// ---------------------------------------------------------------------------
class ADRCAirshipCtrl : public ControllerBase {
public:
    explicit ADRCAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "ADRC"; }
private:
    PlantParams p_;
    ctrl::DiscreteADRC adrc_;
};

// ---------------------------------------------------------------------------
// 4. SMC - compute(theta - theta_ref): this repo's documented SMC convention (y - ref)
// ---------------------------------------------------------------------------
class SMCAirshipCtrl : public ControllerBase {
public:
    explicit SMCAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "SMC"; }
private:
    PlantParams p_;
    ctrl::DiscreteSMC smc_;
};

// ---------------------------------------------------------------------------
// 5. LQR - makeLQRController on a 4-state [theta,q,rp1,w] design model (Eq. 24),
//    re-linearized + re-trimmed whenever (theta_ref, rp1_ref) changes (e.g. sawtooth).
// ---------------------------------------------------------------------------
class LQRAirshipCtrl : public ControllerBase {
public:
    explicit LQRAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "LQR"; }
private:
    PlantParams p_;
    double theta_ref_design_;
    double rp1_ref_design_;
    double u_ss_ = 0.0;
    std::shared_ptr<ctrl::DiscreteLQR> lqr_;
    void rebuildIfNeeded(double theta_ref, double rp1_ref);
};

// ---------------------------------------------------------------------------
// 6. MPC - same 4-state linearized design model as LQR, deviation-form DiscreteMPC.
// ---------------------------------------------------------------------------
class MPCAirshipCtrl : public ControllerBase {
public:
    explicit MPCAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "MPC"; }
private:
    PlantParams p_;
    double theta_ref_design_;
    double rp1_ref_design_;
    double u_ss_ = 0.0;
    std::unique_ptr<ctrl::DiscreteMPC> mpc_;
    void rebuildIfNeeded(double theta_ref, double rp1_ref);
};

// ---------------------------------------------------------------------------
// 7. MRAC - set_reference(theta_ref), compute(theta); negative gamma_r/gamma_y
// ---------------------------------------------------------------------------
class MRACAirshipCtrl : public ControllerBase {
public:
    explicit MRACAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "MRAC"; }
private:
    PlantParams p_;
    ctrl::MRACController mrac_;
    bool ym_initialized_ = false;
};

// ---------------------------------------------------------------------------
// 8. GainScheduled - 3-point PID schedule on |theta_ref - theta|
// ---------------------------------------------------------------------------
class GainScheduledAirshipCtrl : public ControllerBase {
public:
    explicit GainScheduledAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "GainScheduled"; }
private:
    PlantParams p_;
    ctrl::GainScheduledController gs_;
    // Direct references to the 3 schedule-point PIDs, kept alongside gs_'s own copies so the
    // wrapper can defensively bumplessInit() *all* of them on a theta_ref change (not just
    // whichever one GainScheduledController itself judges "newly entering the active
    // bracket" - see the .cpp comment for why that built-in protection alone is insufficient
    // here).
    std::shared_ptr<ctrl::DiscretePID> pid_gentle_, pid_mid_, pid_aggr_;
    double last_theta_ref_ = std::numeric_limits<double>::quiet_NaN();
};

// ---------------------------------------------------------------------------
// 9. L1Adaptive - set_reference(theta_ref), compute(theta); negative Gamma.
//    Architectural ceiling expected (relative-degree-1 law on a relative-degree-2 plant,
//    see Stewart Platform precedent in CLAUDE.md) - a steady-state plateau is not a bug.
// ---------------------------------------------------------------------------
class L1AdaptiveAirshipCtrl : public ControllerBase {
public:
    explicit L1AdaptiveAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    PlantParams p_;
    ctrl::L1AdaptiveController l1_;
};

// ---------------------------------------------------------------------------
// 10. NeuralPID - compute(theta_ref - theta); negative plant_gain
// ---------------------------------------------------------------------------
class NeuralPIDAirshipCtrl : public ControllerBase {
public:
    explicit NeuralPIDAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "NeuralPID"; }
private:
    PlantParams p_;
    ctrl::NeuralPID npid_;
};

// ---------------------------------------------------------------------------
// 11. ILC - two-phase: PID feedback while learning (phase 1), PID + learned
//    feedforward afterwards (phase 2). Natural fit for s05's repeated ascend/descend
//    cycles; runs as a 1-trial case on the step scenarios.
// ---------------------------------------------------------------------------
class ILCAirshipCtrl : public ControllerBase {
public:
    explicit ILCAirshipCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "ILC"; }
private:
    PlantParams p_;
    ctrl::DiscretePID pid_;
    ctrl::ILC ilc_;
    int  k_ = 0;
    bool phase2_ = false;
    double last_theta_ref_ = std::numeric_limits<double>::quiet_NaN();
};

// ---------------------------------------------------------------------------
// 12. AirshipFBLCtrl - paper's headline feedback-linearization law (Sec. 4.3.2,
//    Theorem 2) on the composite output y = phi1~ + k*phi2~. alpha/beta (the 3rd-order
//    drift/control-gain terms) are obtained NUMERICALLY via finite differences on the
//    plant's own exact closed-form dynamics rather than the paper's unpublished
//    Appendix-A symbolic expressions (see README "Implementation Notes").
// ---------------------------------------------------------------------------
class AirshipFBLCtrl : public ControllerBase {
public:
    explicit AirshipFBLCtrl(const PlantParams& p);
    double      compute(const State& x, double theta_ref, double rp1_ref, double m0) override;
    void        reset() override;
    std::string name() const override { return "AirshipFBLCtrl"; }

    // Composite output y(x) = phi1_tilde(x) + k*phi2_tilde(x) (exposed for unit testing the
    // numerical alpha/beta probe against a hand-computed value at one fixed state).
    double yOutput(const State& x) const;
    // Numerically estimate [xi1, xi2, xi3, alpha, beta] at (x, m0) via a short constant-input
    // RK4 rollout + forward finite-difference stencils (exposed for unit testing).
    void normalForm(const State& x, double m0,
                     double& xi1, double& xi2, double& xi3,
                     double& alpha, double& beta) const;

private:
    PlantParams p_;
    double k_gain_   = 50.0;
    double lambda0_  = 1.0;
    double lambda1_  = 2.0;
    double lambda2_  = 2.0;
};

// Builds all 12 controllers in roster order.
std::vector<std::unique_ptr<ControllerBase>> makeControllers(const PlantParams& p);

}  // namespace bouyancydrivenairshipinverticalplan
