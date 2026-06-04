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

    // Called by the runner after each plant step with the computed EER_grid.
    // ESCSolarCtrl overrides this to feed the efficiency metric back to the
    // ExtremumSeeker. Default is a no-op.
    virtual void setLastEER(double /*eer_grid*/) {}
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
                           double a = 0.018, double b = 100.0);
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

// ---------------------------------------------------------------------------
// 4. ADRCSolarCtrl
// Active disturbance rejection: ESO estimates irradiance-driven lumped disturbance.
// b0 = 4.5 (FOPDT continuous input gain). omega_o*Ts = 0.04*10 = 0.4 < 0.5 (stable).
// ---------------------------------------------------------------------------
class ADRCSolarCtrl : public ControllerBase {
public:
    explicit ADRCSolarCtrl(double Ts);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
    static constexpr double kr_nom_ = 0.85;
};

// ---------------------------------------------------------------------------
// 5. FuzzyPIDSolarCtrl
// 25-rule Mamdani FuzzyPD + crisp integral on Tw1 error.
// e_scale=10 ^\circC, de_scale=1 ^\circC/step, u_scale=0.10 kg/s.
// ---------------------------------------------------------------------------
class FuzzyPIDSolarCtrl : public ControllerBase {
public:
    explicit FuzzyPIDSolarCtrl(double Ts);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "FuzzyPID"; }
private:
    ctrl::FuzzyPID fuzzy_;
    static constexpr double kr_nom_ = 0.85;
};

// ---------------------------------------------------------------------------
// 6. SmithPredictorSolarCtrl
// Smith Predictor with 2-step dead-time (approx = 20 s thermal measurement lag).
// FOPDT inner model: a=0.018, b=4.5. Inner PID: Kp=0.003, Ki=0.0003.
// ---------------------------------------------------------------------------
class SmithPredictorSolarCtrl : public ControllerBase {
public:
    explicit SmithPredictorSolarCtrl(double Ts);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "SmithPredictor"; }
private:
    ctrl::SmithPredictor sp_;
    static constexpr double kr_nom_ = 0.85;
};

// ---------------------------------------------------------------------------
// 7. MRACSolarCtrl
// MRAC with sigma-modification; adapts spray-flow gain as conditions vary.
// Reference model: a_m=0.70, b_m=0.30. compute() takes plant output, NOT error.
// ---------------------------------------------------------------------------
class MRACSolarCtrl : public ControllerBase {
public:
    explicit MRACSolarCtrl(double Ts);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "MRAC"; }
private:
    ctrl::MRACController mrac_;
    static constexpr double kr_nom_ = 0.85;
};

// ---------------------------------------------------------------------------
// 8. ESCSolarCtrl
// ExtremumSeeker on EER_grid feedback. Perturbation on m_dot_w; seeker
// maximises EER_grid (seekMinimum=false, cost=-EER_grid fed as positive J).
// Runner must call setLastEER(out.EER_grid) after each plant.step().
// ---------------------------------------------------------------------------
class ESCSolarCtrl : public ControllerBase {
public:
    explicit ESCSolarCtrl(double Ts);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "ESC"; }
    void setLastEER(double eer_grid) override { last_eer_ = eer_grid; }

private:
    ctrl::ExtremumSeeker esc_;
    ctrl::DiscretePID    pid_Tw1_;   // inner PID to keep Tw1 near ref
    double kr_nom_   = 0.85;
    double last_eer_ = 3.0;         // initial guess for EER_grid
    static constexpr double kMwMin = 0.02;
    static constexpr double kMwMax = 0.22;
};

// ---------------------------------------------------------------------------
// 9. GPCRLSSolarCtrl
// GPC (Np=20, Nu=5) with online RLS identification of FOPDT SISO model.
// RLS(na=1, nb=1, lambda=0.98) tracks gain variation with irradiance.
// ---------------------------------------------------------------------------
class GPCRLSSolarCtrl : public ControllerBase {
public:
    explicit GPCRLSSolarCtrl(double Ts, double Tw1_nom = 40.0);
    ControlInput compute(double ref_Tw1, double measured_Tw1, double G) override;
    void reset() override;
    std::string name() const override { return "GPC-RLS"; }

private:
    ctrl::GeneralizedPredictiveController gpc_;
    ctrl::RecursiveLeastSquares           rls_;
    double Tw1_nom_;
    double kr_nom_ = 0.85;
    double u_prev_ = 0.12;    // m_dot_w previous step [kg/s]
    int    step_   = 0;
    static constexpr int kWarmup         = 50;
    static constexpr int kRLSInterval    = 20;
    static constexpr double kMwMin = 0.02;
    static constexpr double kMwMax = 0.22;
};

} // namespace solar
