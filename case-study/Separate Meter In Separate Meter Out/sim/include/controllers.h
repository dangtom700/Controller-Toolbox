#pragma once
#include "smismo_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <string>

// SMISMO controller wrappers.
//
// Interface: compute(state, x_ref) -> u_ctrl [V] in [-10, 10]
//   state : full 8-state plant vector [x_L, v_L, P_1, P_2, xv_1, dxv_1, xv_2, dxv_2]
//   x_ref : load position reference [m]
//
// u_ctrl is the WORKING-SIDE valve command (sign = motion direction). The shared
// ValveAllocator (Liu 2009 Fig. 10 dual-loop structure, generalised to both
// directions) maps u_ctrl to the two PDCV commands (u_1, u_2) and regulates the
// off-side chamber to the desired backpressure P_bd via feedforward + PI.
//
// Working-side plant gain for tuning: v_L/u ~ 0.14 (m/s)/V at nominal load
// pressure, with an effective velocity lag tau_v ~ 25 ms (valve + hydraulics).

namespace smismo {

// Nominal working-side plant model used by the model-based controllers
static constexpr double K_V   = 0.14;    // velocity gain [(m/s)/V]
static constexpr double TAU_V = 0.025;   // velocity lag [s]
static constexpr double U_MAX = 10.0;    // command range [V]
static constexpr double V_REF_MAX = 0.45; // inner velocity command clamp [m/s]

inline double clampU(double u) { return std::clamp(u, -U_MAX, U_MAX); }

// ---------------------------------------------------------------------------
// ValveAllocator - mode selection + off-side backpressure regulation
// (shared by all controllers; lives in the simulation runner)
// ---------------------------------------------------------------------------
class ValveAllocator {
public:
    struct Cmd { double u1; double u2; };

    explicit ValveAllocator(const PlantParams& p);

    // u_ctrl [V]: working-side command; state: full plant state.
    Cmd  allocate(double u_ctrl, const SmismoPlant::State& state);
    void reset();

    int mode() const { return mode_; }

private:
    PlantParams p_;
    int    mode_  = +1;     // +1 extend (PDCV1 working), -1 retract (PDCV2 working)
    double integ_ = 0.0;    // backpressure PI integrator [V]

    static constexpr double U_HYST = 0.05;   // mode hysteresis threshold [V]
    static constexpr double KP_BP  = 1.0e-5; // backpressure P gain [V/Pa]
    static constexpr double KI_BP  = 1.0e-3; // backpressure I gain [V/(Pa.s)]
};

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double compute(const SmismoPlant::State& state, double x_ref) = 0;
    virtual void   reset() = 0;
    virtual std::string name() const = 0;

    // Called by the runner AFTER compute() but BEFORE plant.step().
    // Override to adjust dynamic plant parameters (e.g., setSupplyPressure).
    virtual void beforePlantStep(SmismoPlant& /*plant*/) {}
};

// ---------------------------------------------------------------------------
// 1. PID - single position loop
// ---------------------------------------------------------------------------
class PIDPosCtrl : public ControllerBase {
public:
    explicit PIDPosCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "PID"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 2. CascadePID - outer position P -> v_ref; inner velocity PI -> u
// ---------------------------------------------------------------------------
class CascadePIDCtrl : public ControllerBase {
public:
    explicit CascadePIDCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "CascadePID"; }
private:
    ctrl::DiscretePID pid_v_;   // inner velocity PI
    double Kp_pos_ = 8.0;       // outer position gain [1/s]
};

// ---------------------------------------------------------------------------
// 3. LQR - 2-state [x_L, v_L] Bryson design; u = -K (x - x_ref)
// ---------------------------------------------------------------------------
class LQRCtrl : public ControllerBase {
public:
    explicit LQRCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override {}
    std::string name() const override { return "LQR"; }
private:
    Eigen::MatrixXd K_;   // 1x2
};

// ---------------------------------------------------------------------------
// 4. LQG - Kalman filter estimates v_L from x_L only; LQR feedback
// ---------------------------------------------------------------------------
class LQGCtrl : public ControllerBase {
public:
    explicit LQGCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "LQG"; }
private:
    ctrl::DiscreteLQG lqg_;
    Eigen::VectorXd   u_prev_;
};

// ---------------------------------------------------------------------------
// 5. MPC - 2-state ZOH model, Np=60 (60 ms), Nc=5
// ---------------------------------------------------------------------------
class MPCCtrl : public ControllerBase {
public:
    explicit MPCCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "MPC"; }
private:
    std::unique_ptr<ctrl::DiscreteMPC> mpc_;
};

// ---------------------------------------------------------------------------
// 6. ADRC - 2nd-order LADRC on position; b0 = K_V/TAU_V
// ---------------------------------------------------------------------------
class ADRCCtrl : public ControllerBase {
public:
    explicit ADRCCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
};

// ---------------------------------------------------------------------------
// 7. SMC - sliding surface on position error; compute(y - ref)
// ---------------------------------------------------------------------------
class SMCCtrl : public ControllerBase {
public:
    explicit SMCCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "SMC"; }
private:
    ctrl::DiscreteSMC smc_;
};

// ---------------------------------------------------------------------------
// 8. FeedbackLinearisation - g(x) = K_q/(u_max*A)*sqrt(DP), inner PID -> v_cmd
//    Compensates the sqrt(DP) valve-gain variation (Liu calc-flow control).
// ---------------------------------------------------------------------------
class FLCtrl : public ControllerBase {
public:
    explicit FLCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "FeedbackLin"; }
private:
    std::unique_ptr<ctrl::FeedbackLinearisationController> fl_;
};

// ---------------------------------------------------------------------------
// 9. TubeMPC - robust MPC on the 2-state model; LQR tube gain (negated)
// ---------------------------------------------------------------------------
class TubeMPCCtrl : public ControllerBase {
public:
    explicit TubeMPCCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "TubeMPC"; }
private:
    std::unique_ptr<ctrl::TubeMPC> tmpc_;
};

// ---------------------------------------------------------------------------
// 10. L1Adaptive - setReference(x_ref) + compute(x_L)
// ---------------------------------------------------------------------------
class L1Ctrl : public ControllerBase {
public:
    explicit L1Ctrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    ctrl::L1AdaptiveController l1_;
};

// ---------------------------------------------------------------------------
// 11. GainScheduled - 3 PIDs scheduled on v_L (resistive vs. overrunning)
// ---------------------------------------------------------------------------
class GainSchedCtrl : public ControllerBase {
public:
    explicit GainSchedCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "GainScheduled"; }
private:
    ctrl::GainScheduledController gs_;
};

// ---------------------------------------------------------------------------
// 12. NonlinearMPC - RTI on 2-state model with flow-saturation nonlinearity,
//     internal 10 ms prediction step (Np=12 -> 120 ms horizon)
// ---------------------------------------------------------------------------
class NMPCCtrl : public ControllerBase {
public:
    explicit NMPCCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    std::string name() const override { return "NonlinearMPC"; }
private:
    std::unique_ptr<ctrl::NonlinearMPC> nmpc_;
};

// ---------------------------------------------------------------------------
// 13. DOBEnergyCtrl - Disturbance Observer (Chen 2018 Eq. 29-30) estimating
//     load force F_hat, then adapting supply pressure P_s adaptively to
//     minimise hydraulic power while satisfying backpressure guard P_bd.
//
// Note: grey predictor GM(1,1) omitted (requires pump dynamics not in this
// plant model; P_s is a boundary condition, not a pump state).
//
// The controller calls plant.setSupplyPressure() each step; other controllers
// leave plant supply pressure unchanged (fixed at nominal p_.P_s = 60 bar).
// ---------------------------------------------------------------------------
class DOBEnergyCtrl : public ControllerBase {
public:
    explicit DOBEnergyCtrl(const PlantParams& p);
    double compute(const SmismoPlant::State& state, double x_ref) override;
    void   reset() override;
    void   beforePlantStep(SmismoPlant& plant) override;
    std::string name() const override { return "DOBEnergy"; }
private:
    PlantParams       p_;
    ctrl::DiscretePID pid_;     // position controller (same gains as PIDPosCtrl)

    // Second-order DOB state (Chen 2018 Eq. 29-30)
    double z_obs_   = 0.0;     // observer internal state
    double F_hat_   = 0.0;     // estimated load force [N]
    double P_s_cmd_ = 0.0;     // adaptive supply pressure computed last step [Pa]

    // Adaptive pressure constants
    static constexpr double L_OBS     = 30.0;    // observer gain [1/s]
    static constexpr double P_MARGIN  = 5.0e5;   // backpressure guard [Pa] (5 bar)
    static constexpr double P_MIN_CMD = 22.0e5;  // min supply [Pa] (22 bar > P_bd=20bar)
    static constexpr double P_MAX_CMD = 65.0e5;  // max supply [Pa] (65 bar)
};

} // namespace smismo
