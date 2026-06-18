#pragma once
#include "sotec_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

// S-OTEC controller wrappers.
//
// Interface: compute(state, T_h_ref, T_c, G) -> CtrlOutput {m_dot_f, m_dot_wf}
//   state   : [T_h, T_coll]  [^\circC]
//   T_h_ref : hot reservoir setpoint [^\circC]
//   T_c     : cold ocean temperature [^\circC] (for ORC feedforward)
//   G       : solar irradiance [W/m^2] (for feedforward)
//
// Primary loop: T_h regulated via m_dot_f (collector pump).
//   Positive gain: more m_dot_f -> more heat delivery -> higher T_h.
//   Error convention for SISO controllers: e = T_h_ref - T_h.
//
// Secondary loop: m_dot_wf set by pressure-constrained feedforward.
//   m_dot_wf = min(0.9 * m_dot_wf_max(T_h), m_dot_wf_max_phys)

namespace sotec {

struct CtrlOutput {
    double m_dot_f;   // collector pump flow [0.1, 0.5 kg/s]
    double m_dot_wf;  // WF pump flow [0.01, 0.08 kg/s]
};

inline double clampMdotF(double v, const PlantParams& p) {
    return std::clamp(v, p.m_dot_f_min, p.m_dot_f_max);
}
inline double clampMdotWf(double v, const PlantParams& p) {
    return std::clamp(v, p.m_dot_wf_min, p.m_dot_wf_max);
}

// Pressure-constrained m_dot_wf feedforward: 90% of safety limit
inline double m_dot_wf_ff(double T_h, const PlantParams& p) {
    const double m_wf_max = (p.P_inlet_max - p.a0 - p.a1 * T_h) / p.a2;
    return clampMdotWf(m_wf_max * 0.9, p);
}

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual CtrlOutput  compute(const Eigen::Vector2d& state,
                                double T_h_ref, double T_c, double G) = 0;
    virtual void        reset() = 0;
    virtual std::string name() const = 0;
};

// ---------------------------------------------------------------------------
// 1. OpenLoop - fixed m_dot_f=0.3, m_dot_wf=0.04
// ---------------------------------------------------------------------------
class OpenLoopCtrl : public ControllerBase {
public:
    explicit OpenLoopCtrl(const PlantParams& p) : p_(p) {}
    CtrlOutput  compute(const Eigen::Vector2d&, double, double, double) override;
    void        reset() override {}
    std::string name() const override { return "OpenLoop"; }
private:
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 2. PID_Th - T_h regulation via m_dot_f; m_dot_wf by feedforward
// ---------------------------------------------------------------------------
class PIDThCtrl : public ControllerBase {
public:
    explicit PIDThCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "PID"; }
private:
    ctrl::DiscretePID pid_;
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 3. ADRC - T_h via m_dot_f; m_dot_wf feedforward
// ---------------------------------------------------------------------------
class ADRCCtrl : public ControllerBase {
public:
    explicit ADRCCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 4. MPC - FOPDT T_h model; outputs m_dot_f deviation + m_dot_wf feedforward
// ---------------------------------------------------------------------------
class MPCCtrl : public ControllerBase {
public:
    explicit MPCCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "MPC"; }
private:
    std::unique_ptr<ctrl::DiscreteMPC> mpc_;
    PlantParams p_;
    double T_h_nom_;
};

// ---------------------------------------------------------------------------
// 5. LQR - 2-state Bryson-optimal; computes gain via DiscreteLQR in ctor
// ---------------------------------------------------------------------------
class LQRCtrl : public ControllerBase {
public:
    explicit LQRCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override {}
    std::string name() const override { return "LQR"; }
private:
    Eigen::MatrixXd K_;       // 1x2 gain matrix
    Eigen::Vector2d x_nom_;   // [T_h_nom, T_coll_nom]
    double m_dot_f_nom_;
    PlantParams p_;
    double T_h_nom_;
    double T_coll_nom_;
    static ctrl::StateSpace buildSS(const PlantParams& p,
                                    double T_h_nom, double T_coll_nom);
};

// ---------------------------------------------------------------------------
// 6. FuzzyPID - T_h regulation; m_dot_wf feedforward
// ---------------------------------------------------------------------------
class FuzzyPIDCtrl : public ControllerBase {
public:
    explicit FuzzyPIDCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "FuzzyPID"; }
private:
    ctrl::FuzzyPID fpid_;
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 7. MRAC - setReference(T_h_ref), compute(T_h); m_dot_wf feedforward
// ---------------------------------------------------------------------------
class MRACCtrl : public ControllerBase {
public:
    explicit MRACCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "MRAC"; }
private:
    ctrl::MRACController mrac_;
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 8. L1Adaptive - setReference(T_h_ref), compute(T_h); m_dot_wf feedforward
// ---------------------------------------------------------------------------
class L1AdaptiveCtrl : public ControllerBase {
public:
    explicit L1AdaptiveCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    ctrl::L1AdaptiveController l1_;
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 9. GainScheduled - 3 brackets on T_h: <58^\circC, 58-66^\circC, >66^\circC
// ---------------------------------------------------------------------------
class GainScheduledCtrl : public ControllerBase {
public:
    explicit GainScheduledCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "GainScheduled"; }
private:
    ctrl::GainScheduledController gs_;
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 10. ScenarioMPC - 2-state model; P_inlet capped at 1.30 MPa
// ---------------------------------------------------------------------------
class ScenarioMPCCtrl : public ControllerBase {
public:
    explicit ScenarioMPCCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "ScenarioMPC"; }
private:
    ctrl::ScenarioMPC smpc_;
    PlantParams p_;
    double T_h_nom_;
    double T_coll_nom_;
};

// ---------------------------------------------------------------------------
// 11. DynaCtrl - DynaController wrapping PID; m_dot_wf feedforward
// ---------------------------------------------------------------------------
class DynaCtrl : public ControllerBase {
public:
    explicit DynaCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "DynaCtrl"; }
private:
    ctrl::DynaController dyna_;
    PlantParams p_;
};

// ---------------------------------------------------------------------------
// 12. NeuralPID - compute(T_h_ref - T_h); m_dot_wf feedforward
// ---------------------------------------------------------------------------
class NeuralPIDCtrl : public ControllerBase {
public:
    explicit NeuralPIDCtrl(const PlantParams& p);
    CtrlOutput  compute(const Eigen::Vector2d& state, double T_h_ref,
                        double T_c, double G) override;
    void        reset() override;
    std::string name() const override { return "NeuralPID"; }
private:
    ctrl::NeuralPID npid_;
    PlantParams p_;
};

} // namespace sotec
