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

} // namespace susp
