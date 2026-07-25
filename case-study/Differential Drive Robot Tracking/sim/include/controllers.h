#pragma once
// controllers.h - 12-controller roster for the Differential Drive Robot Tracking case study.
//
// Interface: every controller is an OUTER KINEMATIC loop
//
//     compute(BodyError e, v_r, w_r) -> [v_cmd, w_cmd]
//
// The paper's PI wheel-torque actuation layer is shared and lives in the simulation runner,
// so no controller here emits torques directly. Body-frame errors follow the paper:
//
//     [e1, e2, e3]^T = Rz(theta)^T * (q_r - q),   e3 = wrap(theta_r - theta)
//
// Sign conventions (CONTRIBUTING.md#sign-conventions) are handled INSIDE each wrapper so the
// runner can call one uniform signature:
//   - DiscretePID          : compute(r - y)             -> pass +e directly
//   - DiscreteADRC         : computeTracking(y, r)
//   - DiscreteSMC/AdaptiveSMC : compute(y - r)          -> pass -e (sign flipped)
//   - L1AdaptiveController : setReference(r) then compute(y_plant)
//   - NonlinearMPC         : setState(x) then compute(r - y)
//   - DiscreteLQR          : compute(x) directly (not an IController)
#include "differential_drive_robot_tracking_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace differentialdriverobottracking {

/// Body-frame tracking error handed to every controller each fast tick.
struct BodyError {
    double e1  = 0.0;  ///< longitudinal error [m]
    double e2  = 0.0;  ///< lateral error [m]
    double e3  = 0.0;  ///< orientation error, wrapped to (-pi, pi] [rad]
    double de1 = 0.0;  ///< d(e1)/dt, backward difference over Tf [m/s]
    double de2 = 0.0;  ///< d(e2)/dt [m/s]
    double de3 = 0.0;  ///< d(e3)/dt [rad/s]
};

/// Optional per-step internals published for the CSV (NaN when a controller has no such state).
struct CtrlTelemetry {
    double alpha = std::numeric_limits<double>::quiet_NaN();  ///< FUHAC blending coefficient
    double Ks    = std::numeric_limits<double>::quiet_NaN();  ///< adaptive sliding gain
    double d_hat = std::numeric_limits<double>::quiet_NaN();  ///< disturbance estimate
    double V     = std::numeric_limits<double>::quiet_NaN();  ///< composite Lyapunov value
};

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;

    /// Fast loop (Tf = 30 ms): returns [v_cmd, w_cmd].
    virtual Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) = 0;

    /// Slow loop (Ts_slow = 150 ms): parameter-drift relaxation. Default no-op; only FUHAC
    /// overrides it. The paper's time-scale separation is eps = Tf/Ts_slow = 0.2.
    virtual void slowTick() {}

    virtual void reset() = 0;
    virtual std::string name() const = 0;

    virtual CtrlTelemetry telemetry() const { return CtrlTelemetry{}; }
};

using ControllerPtr = std::unique_ptr<ControllerBase>;

/// Build the full 12-controller roster. Ts is the fast-loop period Tf.
std::vector<ControllerPtr> makeControllers(const PlantParams& p);

// ---------------------------------------------------------------------------
// 1. OpenLoop - pure feedforward (v_r, w_r), no feedback. Baseline.
// ---------------------------------------------------------------------------
class OpenLoopCtrl : public ControllerBase {
public:
    explicit OpenLoopCtrl(const PlantParams&) {}
    Eigen::Vector2d compute(const BodyError&, double v_r, double w_r) override {
        return {v_r, w_r};
    }
    void reset() override {}
    std::string name() const override { return "OpenLoop"; }
};

// ---------------------------------------------------------------------------
// 2. PID - paper Table 3 "Standard PID" (reported ISE 14.7, worst of the four).
//    Two ctrl::DiscretePID loops: e1 -> delta v, (e2 + e3 blend) -> delta w.
// ---------------------------------------------------------------------------
class PIDCtrl : public ControllerBase {
public:
    explicit PIDCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "PID"; }
private:
    PlantParams       p_;
    ctrl::DiscretePID pid_v_;
    ctrl::DiscretePID pid_w_;
};

// ---------------------------------------------------------------------------
// 3. Backstepping - paper Table 3 "Pure backstepping" (reported ISE 3.3).
//    The paper's Lyapunov virtual controls u1base / u2base, implemented directly:
//    ctrl::BacksteppingController targets SISO strict-feedback chains and cannot express
//    the nonholonomic 3-error structure (CONTRIBUTING.md:164).
// ---------------------------------------------------------------------------
class BacksteppingCtrl : public ControllerBase {
public:
    explicit BacksteppingCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override {}
    std::string name() const override { return "Backstepping"; }

    /// Shared with FUHAC, which uses this exact law as its u_base term.
    static Eigen::Vector2d law(const BodyError& e, double v_r, double w_r,
                               double k1, double k2, double k3);
private:
    PlantParams p_;
    double k1_, k2_, k3_;   // paper Table 1: K1, K2, K3 = 2.0, 4.0, 2.0
};

// ---------------------------------------------------------------------------
// 4. SMC - paper Table 3 "Pure SMC" (reported ISE 9.0, "very high" torque chattering).
//    ctrl::DiscreteSMC with c_de = 0 so the surface collapses to s = c_e*e_input; feeding
//    e_input = e2 + 0.8*e3 reproduces the paper's s = e2 + 0.8 e3 exactly.
// ---------------------------------------------------------------------------
class SMCCtrl : public ControllerBase {
public:
    explicit SMCCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "SMC"; }
private:
    PlantParams       p_;
    ctrl::DiscretePID pid_v_;   // longitudinal channel stays PID; SMC drives w
    ctrl::DiscreteSMC smc_w_;
};

// ---------------------------------------------------------------------------
// 5. AdaptiveSMC - same surface as (4) but with ctrl::AdaptiveSMC's Plestan gain adaptation
//    K[k+1] = clamp(K + Ts*gamma*(|s| - eps), Kmin, Kmax). Isolates exactly the
//    Ks-adaptation contribution that FUHAC also uses (paper reports Ks: 3.0 -> 3.7-4.6).
// ---------------------------------------------------------------------------
class AdaptiveSMCCtrl : public ControllerBase {
public:
    explicit AdaptiveSMCCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "AdaptiveSMC"; }
    CtrlTelemetry telemetry() const override;
private:
    PlantParams       p_;
    ctrl::DiscretePID pid_v_;
    ctrl::AdaptiveSMC asmc_w_;
};

// ---------------------------------------------------------------------------
// 6. ADRC - the lib-native ESO alternative to the paper's first-order disturbance observer.
//    Two ctrl::DiscreteADRC loops via computeTracking(y, r).
// ---------------------------------------------------------------------------
class ADRCCtrl : public ControllerBase {
public:
    explicit ADRCCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "ADRC"; }
private:
    PlantParams        p_;
    ctrl::DiscreteADRC adrc_v_;
    ctrl::DiscreteADRC adrc_w_;
};

// ---------------------------------------------------------------------------
// 7. FuzzyTSK - the paper's neuro-fuzzy structure with FIXED rule weights, built on
//    ctrl::FuzzySystem (TakagiSugeno + Gaussian MFs + WeightedAverage defuzz). Rule weights
//    are immutable after addRule(), so this entry deliberately has NO online learning -
//    it isolates precisely what FUHAC's adaptive weight update buys.
// ---------------------------------------------------------------------------
class FuzzyTSKCtrl : public ControllerBase {
public:
    explicit FuzzyTSKCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "FuzzyTSK"; }
private:
    PlantParams        p_;
    ctrl::FuzzySystem  fs_v_;
    ctrl::FuzzySystem  fs_w_;
    std::vector<double> in_v_, in_w_;   // pre-sized evaluate() argument buffers
};

// ---------------------------------------------------------------------------
// 8. LQR - ctrl::DiscreteLQR on the error model linearised about the nominal cruise
//    (v_r0, w_r0), frozen at construction. Regulates [e1, e2, e3] to zero.
// ---------------------------------------------------------------------------
class LQRCtrl : public ControllerBase {
public:
    explicit LQRCtrl(const PlantParams& p, double v_r0, double w_r0);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override {}
    std::string name() const override { return "LQR"; }
private:
    PlantParams p_;
    std::shared_ptr<ctrl::DiscreteLQR> lqr_;
    Eigen::VectorXd x_;   // pre-allocated [e1, e2, e3]
};

// ---------------------------------------------------------------------------
// 9. NMPC - ctrl::NonlinearMPC on the 3-state body-frame error model with the true
//    nonholonomic error dynamics as its DiscreteDynamics callback.
// ---------------------------------------------------------------------------
class NMPCCtrl : public ControllerBase {
public:
    explicit NMPCCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "NMPC"; }
private:
    PlantParams p_;
    std::shared_ptr<ctrl::NonlinearMPC> nmpc_;
    Eigen::VectorXd x_, yref_;
    // v_r / w_r vary per step but the DiscreteDynamics callback signature is fixed;
    // the callback reads these through the shared_ptr so they stay in sync.
    std::shared_ptr<double> vr_ref_, wr_ref_;
};

// ---------------------------------------------------------------------------
// 10. L1Adaptive - two ctrl::L1AdaptiveController channels driven by the (negated) errors
//     as pseudo plant outputs regulated to zero.
// ---------------------------------------------------------------------------
class L1AdaptiveCtrl : public ControllerBase {
public:
    explicit L1AdaptiveCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    PlantParams                 p_;
    ctrl::L1AdaptiveController  l1_v_;
    ctrl::L1AdaptiveController  l1_w_;
};

// ---------------------------------------------------------------------------
// 11. GainScheduled - ctrl::GainScheduledController on the angular channel, scheduled on
//     reference path curvature |w_r / v_r|: gentle PID on straights, aggressive at corners.
// ---------------------------------------------------------------------------
class GainScheduledCtrl : public ControllerBase {
public:
    explicit GainScheduledCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void reset() override;
    std::string name() const override { return "GainScheduled"; }
private:
    PlantParams p_;
    ctrl::DiscretePID pid_v_;
    std::shared_ptr<ctrl::GainScheduledController> sched_w_;
};

// ---------------------------------------------------------------------------
// 12. FUHAC - the paper's proposed Fixed Ultra-Hybrid Adaptive Controller.
//
//     u1 = alpha*u1base + (1 - alpha)*u1NF
//     u2 = alpha*u2base + (1 - alpha)*u2Stab + u2SMC
//
//     - u_base   : the paper's Lyapunov backstepping law (shared with BacksteppingCtrl)
//     - u1NF     : 5x5 Gaussian Takagi-Sugeno grid on (e1, de1) with ONLINE gradient-descent-
//                  with-momentum weight updates. Study-local because ctrl::FuzzySystem fixes
//                  rule weights at addRule() and exposes no mutator, and
//                  ctrl::NNAdaptiveController has no Gaussian/RBF activation
//                  (lib/NeuralNetworkController.h:38).
//     - u2SMC    : ctrl::AdaptiveSMC with c_de = 0 and input e2 + 0.8*e3, giving the paper's
//                  s = e2 + 0.8 e3, u = -K*sat(s/Phi) and its adaptive Ks verbatim.
//     - DOB      : first-order dhat' = L*(y - yhat) plus a gamma-filtered clamp.
//     - predictor: e_pred(k+i) = [e(k) + de(k)*i*dt]*exp(-0.5*i), H = 5, dt = 0.1 s.
//     - blending : alpha[k] = 0.97*alpha[k-1] + 0.03*alpha_raw[k] (paper Table 1 beta = 0.97).
// ---------------------------------------------------------------------------
class FUHACCtrl : public ControllerBase {
public:
    static constexpr int NF_GRID = 5;   ///< 5x5 = 25 fuzzy rules
    static constexpr int H_PRED  = 5;   ///< prediction horizon (paper: H = 5, dt = 0.1 s)

    explicit FUHACCtrl(const PlantParams& p);
    Eigen::Vector2d compute(const BodyError& e, double v_r, double w_r) override;
    void slowTick() override;
    void reset() override;
    std::string name() const override { return "FUHAC"; }
    CtrlTelemetry telemetry() const override;

private:
    double neuroFuzzy(double e1, double de1);       ///< u1NF with online weight update
    double disturbanceObserver(double e1, double u_applied);
    double predictiveGain(const BodyError& e) const;
    double updateAlpha(const BodyError& e);

    PlantParams p_;

    // -- backstepping gains (paper Table 1: K1, K2, K3 = 2.0, 4.0, 2.0) ------
    double k1_ = 2.0, k2_ = 4.0, k3_ = 2.0;

    // -- neuro-fuzzy layer ---------------------------------------------------
    std::array<double, NF_GRID> c_e_{},  c_de_{};    ///< Gaussian centres
    double sigma_e_ = 0.5, sigma_de_ = 0.5;          ///< Gaussian widths
    std::array<double, NF_GRID * NF_GRID> w_{};      ///< consequent singletons y_ij
    std::array<double, NF_GRID * NF_GRID> dw_prev_{};///< momentum term
    std::array<double, NF_GRID> mu_e_{}, mu_de_{};   ///< pre-allocated membership buffers
    std::array<double, NF_GRID * NF_GRID> phi_{};    ///< pre-allocated rule activations
    double eta1_     = 0.05;    ///< paper Table 1 learning rate
    // Momentum 0.9 gives a steady-state amplification of 1/(1-0.9) = 10x, which drives the
    // weights straight into their projection bound within a second at Tf = 30 ms. 0.5 (2x)
    // keeps the learned residual in the same order as the velocity correction it augments.
    double momentum_ = 0.5;
    double w_max_    = 1.0;     ///< projection bound [m/s]; the NFS output is a residual
    double u1nf_prev_ = 0.0;    ///< paper's 0.8/0.2 output filter state

    // -- sliding mode (paper Table 1: Ks 3.0-4.6, Phi = 0.2) -----------------
    ctrl::AdaptiveSMC smc_;

    // -- disturbance observer ------------------------------------------------
    double L_dob_   = 20.0;   ///< omega_dynamics (~3.3 rad/s) << L << omega_noise (~105 rad/s)
    double gamma_   = 0.85;   ///< output filter pole
    double d_max_   = 2.0;    ///< clamp on the estimate
    double d_hat_   = 0.0;
    double y_hat_   = 0.0;

    // -- predictive error compensator ----------------------------------------
    double dt_pred_ = 0.1;    ///< paper's dt (H*dt = 0.5 s ~ tau_m)

    // -- adaptive blending ----------------------------------------------------
    double alpha_     = 1.0;
    double alpha_min_ = 0.25;
    double alpha_max_ = 0.95;
    double beta_      = 0.97;  ///< paper Table 1
    double wt_ = 0.15, we_ = 0.45, wp_ = 0.25, wo_ = 0.15;  ///< indicator weights (sum = 1)
    double t_elapsed_ = 0.0;
    double perf_index_ = 0.0;   ///< running normalised ISE, feeds P(I)
    double osc_        = 0.0;   ///< running |du| average, feeds O(l)
    double u2_prev_    = 0.0;

    // -- Lyapunov trace -------------------------------------------------------
    double V_ = 0.0;
};

}  // namespace differentialdriverobottracking
