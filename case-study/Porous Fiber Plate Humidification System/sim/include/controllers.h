#pragma once
#include "humid_plant.h"
#include <ControllerToolbox.h>
#include <string>

// All humidification controllers share:
//   compute(phi_measured, ref_phi) -> ControlInput
//
//   phi_measured : delayed sensor reading from plant [fraction 0-1]
//   ref_phi      : humidity setpoint [fraction 0-1]
//
// The returned ControlInput carries {u_fan [m/s], Ta_sp [K]}.
// Single-input controllers leave Ta_sp at the default 40^\circC (313.15 K).

namespace humid {

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual ControlInput compute(double phi_measured, double ref_phi) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};

// Clamp fan speed to valid range.
static inline double clampFan(double v)  { return std::clamp(v, 1.0, 3.5); }
static inline double clampTa(double v)   { return std::clamp(v, 303.15, 323.15); }
static constexpr double kTa_nom = 313.15;  // 40^\circC default heater setpoint
static constexpr double kFanMid = 2.25;    // midpoint of valid fan range

// ---------------------------------------------------------------------------
// 1. PID - basic PI on phi_room error; fan is the sole MV
// ---------------------------------------------------------------------------
class PIDHumidCtrl : public ControllerBase {
public:
    explicit PIDHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "PID"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 2. PID_AW - same as PID but tests that Kb back-calculation AW fires at
//             saturation (phi_out=0.55 scenario the fan will hit u_min=1.0)
// ---------------------------------------------------------------------------
class PID_AWHumidCtrl : public ControllerBase {
public:
    explicit PID_AWHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "PID_AW"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 3. Feedforward + PID - static FF from outdoor phi estimate + PI trim
//    FF: lower outdoor phi -> higher u_fan demanded
// ---------------------------------------------------------------------------
class FFPIDHumidCtrl : public ControllerBase {
public:
    explicit FFPIDHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "FFPID"; }

    // Runner updates outdoor conditions each step for FF path
    void setOutdoorPhi(double phi_out) { phi_out_ = phi_out; }

private:
    ctrl::DiscretePID pid_;
    double phi_out_ = 0.35;
};

// ---------------------------------------------------------------------------
// 4. Cascade PID - outer PI: phi_room -> H_ref [g/h];
//                  inner: physics inversion H_ref -> u_fan
// ---------------------------------------------------------------------------
class CascadePIDHumidCtrl : public ControllerBase {
public:
    explicit CascadePIDHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "Cascade"; }
private:
    ctrl::DiscretePID outer_pi_;
    double H_nom_  = 275.0;   // nominal H at kFanMid [g/h]
    double u_fan_prev_ = kFanMid;
};

// ---------------------------------------------------------------------------
// 5. Gain-scheduled PID - gains scheduled on phi_room (3 operating points)
// ---------------------------------------------------------------------------
class GainSchedHumidCtrl : public ControllerBase {
public:
    explicit GainSchedHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "GainSched"; }
private:
    ctrl::GainScheduledController sched_;
};

// ---------------------------------------------------------------------------
// 6. Smith Predictor - compensates for 2-step (60 s) RH sensor dead-time
// ---------------------------------------------------------------------------
class SmithHumidCtrl : public ControllerBase {
public:
    explicit SmithHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "Smith"; }
private:
    ctrl::SmithPredictor sp_;
};

// ---------------------------------------------------------------------------
// 7. ADRC - ESO lumps infiltration / occupancy as unknown disturbance
// ---------------------------------------------------------------------------
class ADRCHumidCtrl : public ControllerBase {
public:
    explicit ADRCHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
};

// ---------------------------------------------------------------------------
// 8. MPC - FOPDT linearised model; co-optimises u_fan; Ta fixed
// ---------------------------------------------------------------------------
class MPCHumidCtrl : public ControllerBase {
public:
    explicit MPCHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "MPC"; }
private:
    ctrl::DiscreteMPC mpc_;
    static constexpr double kPhiNom = 0.45;  // operating point for linearisation
};

// ---------------------------------------------------------------------------
// 9. MRAC - sigma-modification; adapts to changing K_hum (season / occupancy)
// ---------------------------------------------------------------------------
class MRACHumidCtrl : public ControllerBase {
public:
    explicit MRACHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "MRAC"; }
private:
    ctrl::MRACController mrac_;
};

// ---------------------------------------------------------------------------
// 10. GPC + RLS - GPC (Np=15, Nu=4) with online RLS model adaptation
// ---------------------------------------------------------------------------
class GPCRLSHumidCtrl : public ControllerBase {
public:
    explicit GPCRLSHumidCtrl(double Ts);
    ControlInput compute(double phi_measured, double ref_phi) override;
    void reset() override;
    std::string name() const override { return "GPC_RLS"; }
private:
    ctrl::GeneralizedPredictiveController gpc_;
    ctrl::RecursiveLeastSquares           rls_;
    double u_prev_   = kFanMid;
    int    step_     = 0;
    static constexpr int kWarmup      = 60;
    static constexpr int kRLSInterval = 20;
};

} // namespace humid
