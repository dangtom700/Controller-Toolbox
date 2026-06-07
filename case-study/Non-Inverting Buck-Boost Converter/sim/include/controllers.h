#pragma once
#include "buck_boost_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <string>

// Buck-Boost converter controller wrappers.
//
// Interface: compute(state, v_ref, V_in) -> duty cycle d in [0, 1]
//   state : [i_L, v_C]  (inductor current, capacitor voltage)
//   v_ref : reference output voltage [V]
//   V_in  : current input voltage [V]
//
// The runner applies no additional clamping; each controller must return d in [0,1].

namespace conv {

static constexpr double D_MIN = 0.0;
static constexpr double D_MAX = 1.0;

inline double clampD(double d) { return std::clamp(d, D_MIN, D_MAX); }

// Steady-state duty cycle feedforward
inline double dSS(double v_ref, double V_in, ConvMode mode) {
    if (mode == ConvMode::BUCK)
        return clampD(v_ref / V_in);
    else
        return clampD(1.0 - V_in / std::max(v_ref, V_in + 0.01));
}

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) = 0;
    virtual void        reset() = 0;
    virtual std::string name() const = 0;

protected:
    // Hysteresis-based mode detection; updates mode_ in-place.
    void updateMode(double v_ref, double V_in, double hysteresis = 0.1);
    ConvMode mode_ = ConvMode::BUCK;
};

// ---------------------------------------------------------------------------
// 1. OpenLoop - steady-state duty cycle only; no feedback
// ---------------------------------------------------------------------------
class OpenLoopCtrl : public ControllerBase {
public:
    explicit OpenLoopCtrl(const PlantParams& p) : hyst_(p.hysteresis) {}
    double      compute(const Eigen::Vector2d&, double v_ref, double V_in) override;
    void        reset() override { mode_ = ConvMode::BUCK; }
    std::string name() const override { return "OpenLoop"; }
private:
    double hyst_;
};

// ---------------------------------------------------------------------------
// 2. PI_Buck - single classic PI tuned for buck mode (applied across all modes)
// ---------------------------------------------------------------------------
class PIBuckCtrl : public ControllerBase {
public:
    explicit PIBuckCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "PI_Buck"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 3. PI_Boost - single classic PI tuned for boost mode (applied across all modes)
// ---------------------------------------------------------------------------
class PIBoostCtrl : public ControllerBase {
public:
    explicit PIBoostCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "PI_Boost"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 4. TLCS_ClassicPI - two classic PIs with bumpless mode-switch transfer
// ---------------------------------------------------------------------------
class TLCSClassicPICtrl : public ControllerBase {
public:
    explicit TLCSClassicPICtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "TLCS_ClassicPI"; }
private:
    ctrl::DiscretePID pid_buck_;
    ctrl::DiscretePID pid_boost_;
    double hyst_;
};

// ---------------------------------------------------------------------------
// 5. FuzzyPD - FuzzyPD + steady-state feedforward (shows PD-only limitation)
// ---------------------------------------------------------------------------
class FuzzyPDCtrl : public ControllerBase {
public:
    explicit FuzzyPDCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "FuzzyPD"; }
private:
    ctrl::FuzzyPD fpd_;
    double hyst_;
};

// ---------------------------------------------------------------------------
// 6. FuzzyPID_Buck - FuzzyPID with buck gains, single mode (comparison)
// ---------------------------------------------------------------------------
class FuzzyPIDBuckCtrl : public ControllerBase {
public:
    explicit FuzzyPIDBuckCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "FuzzyPID_Buck"; }
private:
    ctrl::FuzzyPID fpid_;
};

// ---------------------------------------------------------------------------
// 7. FuzzyPID_Boost - FuzzyPID with boost gains, single mode (comparison)
// ---------------------------------------------------------------------------
class FuzzyPIDBoostCtrl : public ControllerBase {
public:
    explicit FuzzyPIDBoostCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "FuzzyPID_Boost"; }
private:
    ctrl::FuzzyPID fpid_;
};

// ---------------------------------------------------------------------------
// 8. TLCS_FuzzyPI - paper's main result: two FuzzyPIDs with bumpless transfer
//    Buck FuzzyPID active when V_ref < V_in; Boost FuzzyPID when V_ref > V_in.
//    Inactive controller is synchronised every step via bumplessInit().
// ---------------------------------------------------------------------------
class TLCSFuzzyPICtrl : public ControllerBase {
public:
    explicit TLCSFuzzyPICtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "TLCS_FuzzyPI"; }
private:
    ctrl::FuzzyPID fpid_buck_;
    ctrl::FuzzyPID fpid_boost_;
    double hyst_;
};

// ---------------------------------------------------------------------------
// 9. GainScheduled - scheduling variable xi = V_in/V_ref (0=boost, 2=buck)
// ---------------------------------------------------------------------------
class GainScheduledCtrl : public ControllerBase {
public:
    explicit GainScheduledCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "GainScheduled"; }
private:
    ctrl::GainScheduledController gs_;
};

// ---------------------------------------------------------------------------
// 10. ADRC - 2nd-order LADRC; ESO treats mode nonlinearity as total disturbance
// ---------------------------------------------------------------------------
class ADRCConvCtrl : public ControllerBase {
public:
    explicit ADRCConvCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
};

// ---------------------------------------------------------------------------
// 11. MPC - DiscreteMPC with buck-mode linearised SS model
// ---------------------------------------------------------------------------
class MPCConvCtrl : public ControllerBase {
public:
    explicit MPCConvCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override;
    std::string name() const override { return "MPC"; }
private:
    std::unique_ptr<ctrl::DiscreteMPC> mpc_;
    static ctrl::StateSpace buildBuckSS(const PlantParams& p);
};

// ---------------------------------------------------------------------------
// 12. LQR - DiscreteLQR on buck model + steady-state feed-forward
// ---------------------------------------------------------------------------
class LQRConvCtrl : public ControllerBase {
public:
    explicit LQRConvCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double v_ref, double V_in) override;
    void        reset() override {}
    std::string name() const override { return "LQR"; }
private:
    ctrl::DiscreteLQR lqr_;
    double R_load_;
    static ctrl::StateSpace buildBuckSS(const PlantParams& p);
};

} // namespace conv
