#pragma once
#include "susp_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

// Active suspension controller wrappers.
//
// All controllers share:
//   compute(state, z_r) -> F_act [N]
//
//   state : [z_s, dz_s, z_u, dz_u]  (full 4-state quarter-car)
//   z_r   : current road surface height [m]  (most controllers ignore this)
//
// The returned F_act is clamped to [-F_max, F_max] in the runner after compute().
// Controllers may apply internal clamping as well.

namespace susp {

static constexpr double F_ACT_MAX = 2000.0;  // N

inline double clampForce(double f) {
    return std::clamp(f, -F_ACT_MAX, F_ACT_MAX);
}

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double compute(const Eigen::Vector4d& state, double z_r) = 0;
    virtual void   reset() = 0;
    virtual std::string name() const = 0;
};

// ---------------------------------------------------------------------------
// 1. Passive - no actuator (baseline reference)
// ---------------------------------------------------------------------------
class PassiveCtrl : public ControllerBase {
public:
    explicit PassiveCtrl(const PlantParams&) {}
    double  compute(const Eigen::Vector4d&, double) override { return 0.0; }
    void    reset() override {}
    std::string name() const override { return "Passive"; }
};

// ---------------------------------------------------------------------------
// 2. PID - body displacement feedback, r = 0
// ---------------------------------------------------------------------------
class PIDSuspCtrl : public ControllerBase {
public:
    explicit PIDSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "PID"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 3. ADRC - treats wheel/road coupling as total unknown disturbance
// ---------------------------------------------------------------------------
class ADRCSuspCtrl : public ControllerBase {
public:
    explicit ADRCSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
};

// ---------------------------------------------------------------------------
// 4. SMC - 1st-order sliding surface on body displacement
// ---------------------------------------------------------------------------
class SMCSuspCtrl : public ControllerBase {
public:
    explicit SMCSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "SMC"; }
private:
    ctrl::DiscreteSMC smc_;
};

// ---------------------------------------------------------------------------
// 5. LQR - full-state optimal feedback (Bryson-tuned)
// ---------------------------------------------------------------------------
class LQRSuspCtrl : public ControllerBase {
public:
    explicit LQRSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override {}   // LQR is stateless
    std::string name() const override { return "LQR"; }
private:
    ctrl::DiscreteLQR lqr_;
};

// ---------------------------------------------------------------------------
// 6. LQG - Kalman filter estimates full state; LQR provides feedback
// ---------------------------------------------------------------------------
class LQGSuspCtrl : public ControllerBase {
public:
    explicit LQGSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "LQG"; }
private:
    ctrl::DiscreteLQG lqg_;
    Eigen::VectorXd u_prev_;
};

// ---------------------------------------------------------------------------
// 7. MPC - 2-state body model, constraint-aware
// ---------------------------------------------------------------------------
class MPCSuspCtrl : public ControllerBase {
public:
    explicit MPCSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "MPC"; }
private:
    std::unique_ptr<ctrl::DiscreteMPC> mpc_;
    static ctrl::StateSpace buildBodySS(const PlantParams& p);
};

// ---------------------------------------------------------------------------
// 8. MRAC - sigma-modified adaptive control; adapts to road profile changes
// ---------------------------------------------------------------------------
class MRACSuspCtrl : public ControllerBase {
public:
    explicit MRACSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "MRAC"; }
private:
    ctrl::MRACController mrac_;
};

// ---------------------------------------------------------------------------
// 9. FuzzyPID - Mamdani FuzzyPD + integral, scaled for suspension ranges
// ---------------------------------------------------------------------------
class FuzzyPIDSuspCtrl : public ControllerBase {
public:
    explicit FuzzyPIDSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "FuzzyPID"; }
private:
    ctrl::FuzzyPID fpid_;
};

// ---------------------------------------------------------------------------
// 10. TubeMPC - robust MPC; 2-state body model, tube sized to wheel disturbance
// ---------------------------------------------------------------------------
class TubeMPCSuspCtrl : public ControllerBase {
public:
    explicit TubeMPCSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "TubeMPC"; }
private:
    ctrl::TubeMPC tmpc_;
    static ctrl::StateSpace buildBodySS(const PlantParams& p);
};

// ---------------------------------------------------------------------------
// 11. ILC - two-phase iterative learning on body displacement
//     Phase 1 (first N_TRIAL steps): PID only, record errors.
//     Phase 2 (remaining steps): PID + learned feedforward correction.
// ---------------------------------------------------------------------------
class ILCSuspCtrl : public ControllerBase {
public:
    explicit ILCSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "ILC"; }
private:
    ctrl::ILC          ilc_;
    ctrl::DiscretePID  pid_;
    int                k_      = 0;
    bool               phase2_ = false;
    static constexpr int N_TRIAL = 1000;  // 5 s at Ts=0.005
};

// ---------------------------------------------------------------------------
// 12. CBFSafety - body velocity barrier (h = v_max - dz_s) wrapping PID
// ---------------------------------------------------------------------------
class CBFSuspCtrl : public ControllerBase {
public:
    explicit CBFSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "CBFSafety"; }
private:
    ctrl::CBFSafetyFilter cbf_;
};

// ---------------------------------------------------------------------------
// 13. L1AdaptiveSuspCtrl - L1 adaptive on body displacement z_s.
// Reference model: a_m = exp(-4*Ts), b_m = 1-a_m (same as MRACSuspCtrl).
// setReference(0.0), compute(z_s) -> F_act [N].
// ---------------------------------------------------------------------------
class L1AdaptiveSuspCtrl : public ControllerBase {
public:
    explicit L1AdaptiveSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    ctrl::L1AdaptiveController l1_;
};

// ---------------------------------------------------------------------------
// 14. ScenarioMPCSuspCtrl - scenario-based stochastic MPC on 2-state body model.
// Mirrors TubeMPCSuspCtrl: same 2-state SS, Np=10, Nu=3.
// Sigma_w models wheel-coupling disturbance (small process noise).
// ---------------------------------------------------------------------------
class ScenarioMPCSuspCtrl : public ControllerBase {
public:
    explicit ScenarioMPCSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "ScenarioMPC"; }
private:
    ctrl::ScenarioMPC smpc_;
    static ctrl::StateSpace buildBodySS(const PlantParams& p);
};

// ---------------------------------------------------------------------------
// 15. DynaSuspCtrl - Dyna MBRL wrapping PID on body displacement.
// error = 0 - z_s = -state(0). compute(error) -> F_act [N].
// ---------------------------------------------------------------------------
class DynaSuspCtrl : public ControllerBase {
public:
    explicit DynaSuspCtrl(const PlantParams& p);
    double  compute(const Eigen::Vector4d& state, double z_r) override;
    void    reset() override;
    std::string name() const override { return "DynaCtrl"; }
private:
    ctrl::DynaController dyna_;
};

} // namespace susp
