#pragma once
#include "solar_plant.h"
#include <ControllerToolbox.h>
#include <string>

// All solar-cooling controllers share:
//   compute(ref_Tw1, measured_Tw1, G) -> ControlInput
//
//   ref_Tw1      : desired condenser warm-water temperature [^\circC]
//   measured_Tw1 : measured Tw1 from last plant step        [^\circC]
//   G            : current solar irradiance (used by FF-PID) [W/m^2]
//
// The returned ControlInput carries {m_dot_w [kg/s], kr [-]}.

namespace solar {

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual ControlInput compute(double ref_Tw1, double measured_Tw1, double G) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};

// ---------------------------------------------------------------------------
// 1. PID - decoupled scalar PIDs on Tw1 -> m_dot_w; kr held at nominal
// ---------------------------------------------------------------------------
class PIDController : public ControllerBase {
public:
    explicit PIDController(double Ts);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "PID"; }

private:
    ctrl::DiscretePID pid_;
    double kr_nom_ = 0.85;
};

// ---------------------------------------------------------------------------
// 2. FF-PID - solar-irradiance feedforward on m_dot_w, PID for feedback trim
//             pump speed scheduled as a linear function of the tracking error
// ---------------------------------------------------------------------------
class FFPIDController : public ControllerBase {
public:
    explicit FFPIDController(double Ts);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "FFPID"; }

private:
    ctrl::DiscretePID pid_;
    double mw_nom_      = 0.10;    // nominal spray flow [kg/s]
    double G_to_mw_     = 1.5e-4;  // FF gain [kg/s / (W/m^2)]
    double kr_base_     = 0.80;
    double kr_e_slope_  = 0.35;    // kr = kr_base + slope * (e / 10)
};

// ---------------------------------------------------------------------------
// 3. MPC - FOPDT model of Tw1 vs m_dot_w, condensed unconstrained QP
//          Linearisation: Tw1[k+1] = (1-a*Ts)*Tw1[k] + b*Ts*m_dot_w[k]
//          Pump kr held at nominal
// ---------------------------------------------------------------------------
class MPCController : public ControllerBase {
public:
    explicit MPCController(double Ts, double Tw1_nom = 40.0,
                           double a = 0.018, double b = 4.5);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "MPC"; }

private:
    ctrl::DiscreteMPC mpc_;
    double Tw1_nom_;
    double kr_nom_ = 0.85;

    // Bounds on m_dot_w [kg/s] expressed as delta from nominal midpoint
    static constexpr double kMwMid  = 0.12;
    static constexpr double kMwHalf = 0.10;
};

} // namespace solar
