#include "smismo_controllers.h"
#include <algorithm>
#include <cmath>

namespace smismo {

// ============================================================================
// 1. PID  (pure PI - no derivative to avoid reference-step kick)
// ============================================================================

PIDCtrl::PIDCtrl(const SmismoOperatingPoint& op, double Ts)
    : pid_([&]() {
        ctrl::PIDParams p;
        p.Kp   = 8.0;
        p.Ki   = 1.5;
        p.Kd   = 0.0;
        p.Kb   = 1.0;
        p.uMin = -1.0;
        p.uMax =  1.0;
        return ctrl::DiscretePID(p, Ts);
    }())
{
    (void)op;
}

double PIDCtrl::compute(const Eigen::Vector4d& y, double r)
{
    return pid_.compute(r - y[0]);
}

void PIDCtrl::reset()
{
    pid_.reset();
}

// ============================================================================
// 2. LQR  (2-state [x,v] + outer position integral)
// ============================================================================

LQRCtrl::LQRCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts)
    : lqr_([&]() {
        ctrl::LQRParams p;
        p.Q = Eigen::Vector2d(1e4, 10.0).asDiagonal();
        p.R = Eigen::Matrix<double,1,1>::Identity() * 1.0;
        return ctrl::DiscreteLQR(ss2, p);
    }())
    , op_(op), Ts_(Ts)
{}

double LQRCtrl::compute(const Eigen::Vector4d& y, double r)
{
    Eigen::Vector2d x2_dev(y[0] - op_.x_eq, y[1] - op_.v_eq);
    Eigen::Vector2d x2_ref(r - op_.x_eq, 0.0);

    Eigen::VectorXd u_lqr = lqr_.compute(x2_dev, x2_ref);

    // Outer integral for zero steady-state error
    pos_integral_ += Ts_ * (r - y[0]);
    pos_integral_ = std::clamp(pos_integral_, -2.0, 2.0);   // anti-windup

    return std::clamp(u_lqr(0) + Ki * pos_integral_, -1.0, 1.0);
}

void LQRCtrl::reset()
{
    pos_integral_ = 0.0;
}

// ============================================================================
// 3. LQG  (2-state Kalman + LQR + outer integral; observes position only)
// ============================================================================

LQGCtrl::LQGCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts)
    : lqg_([&]() {
        // Position-only output model for the Kalman filter
        Eigen::Matrix<double,1,2> C_pos;
        C_pos << 1.0, 0.0;
        Eigen::Matrix<double,1,1> D_zero = Eigen::Matrix<double,1,1>::Zero();
        ctrl::StateSpace ss_obs(ss2.A, ss2.B, C_pos, D_zero, ss2.Ts);

        ctrl::LQRParams lqr_p;
        lqr_p.Q = Eigen::Vector2d(1e4, 10.0).asDiagonal();
        lqr_p.R = Eigen::Matrix<double,1,1>::Identity() * 1.0;

        // Process noise: small position drift, larger velocity noise
        Eigen::Matrix2d Qn = Eigen::Vector2d(1e-6, 1e-2).asDiagonal();
        // Measurement noise: 0.1mm position sensor noise
        Eigen::Matrix<double,1,1> Rn;
        Rn(0,0) = 1e-8;

        return ctrl::DiscreteLQG(ss_obs, lqr_p, Qn, Rn);
    }())
    , op_(op), Ts_(Ts)
    , u_prev_(Eigen::VectorXd::Zero(1))
{}

double LQGCtrl::compute(const Eigen::Vector4d& y, double r)
{
    // Kalman input: position deviation only
    Eigen::Matrix<double,1,1> y_pos;
    y_pos(0) = y[0] - op_.x_eq;

    Eigen::Vector2d x2_ref(r - op_.x_eq, 0.0);

    // LQG step: Kalman update + LQR on estimated state
    Eigen::VectorXd x2_ref_v = x2_ref;
    Eigen::VectorXd u = lqg_.step(y_pos, u_prev_, x2_ref_v);
    u_prev_ = u;

    // Outer integral
    pos_integral_ += Ts_ * (r - y[0]);
    pos_integral_ = std::clamp(pos_integral_, -2.0, 2.0);

    return std::clamp(u(0) + Ki * pos_integral_, -1.0, 1.0);
}

void LQGCtrl::reset()
{
    lqg_.reset();
    u_prev_.setZero();
    pos_integral_ = 0.0;
}

// ============================================================================
// 4. SMC
// ============================================================================

SMCCtrl::SMCCtrl(const SmismoOperatingPoint& op, double Ts)
    : smc_([&]() {
        ctrl::SMCParams p;
        p.c_e  = 1.0;
        p.c_de = 30.0 * Ts;
        p.K    = 2.0;
        p.phi  = 0.05;
        p.uMin = -1.0;
        p.uMax =  1.0;
        return ctrl::DiscreteSMC(p, Ts);
    }())
{
    (void)op;
}

double SMCCtrl::compute(const Eigen::Vector4d& y, double r)
{
    return smc_.compute(y[0] - r);
}

void SMCCtrl::reset()
{
    smc_.reset();
}

// ============================================================================
// 5. ADRC
// ============================================================================

ADRCCtrl::ADRCCtrl(double b0, const SmismoOperatingPoint& op, double Ts)
    : adrc_([&]() {
        ctrl::ADRCParams p;
        p.omega_c = 18.0;
        p.omega_o = 80.0;   // 80*Ts = 0.40 < 0.5 (check)
        // b0 = (Bv/M)*Kv  (quasi-static velocity-servo model gain, ~3 m/s^2/unit)
        p.b0  = b0;
        p.uMin = -1.0;
        p.uMax =  1.0;
        return ctrl::DiscreteADRC(p, Ts);
    }())
{
    (void)op;
}

double ADRCCtrl::compute(const Eigen::Vector4d& y, double r)
{
    return adrc_.compute(r - y[0]);
}

void ADRCCtrl::reset()
{
    adrc_.reset();
}

// ============================================================================
// 6. TubeMPC  (2-state model, position-only output)
// ============================================================================

TubeMPCCtrl::TubeMPCCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op)
    : tmpc_([&]() {
        // 1-state SISO position integrator model:  x[k+1] = x[k] + Kv*Ts*u[k]
        //
        // The 2-state velocity-servo model cannot be used for TubeMPC because
        // B[1]/B[0] approx = 386 forces large velocity-to-position coupling in |A_cl|,
        // making rho(|A_cl|) > 1 and the mRPI series diverge.
        //
        // Terminal velocity Kv = b0 / (Bv/M) approx = 0.0575 m/s per unit.
        // Per-step position gain: Kv*Ts = 2.875e-4 m/unit.
        const double Kv_terminal = ss2.B(1,0) / (1.0 - ss2.A(1,1));  // b0 / a
        const double Kp_step     = Kv_terminal * ss2.Ts;              // m/step/unit

        Eigen::Matrix<double,1,1> A1, B1, C1, D1;
        A1(0,0) = 1.0;
        B1(0,0) = Kp_step;
        C1(0,0) = 1.0;
        D1(0,0) = 0.0;
        ctrl::StateSpace ss1(A1, B1, C1, D1, ss2.Ts);

        ctrl::TubeMPCParams p;
        p.Np = 200;        // needs long horizon: 10cm / (Kv*Ts) approx = 350 steps
        p.Nu = 10;
        p.Q  = Eigen::Matrix<double,1,1>::Identity() * 1e4;
        p.R  = Eigen::Matrix<double,1,1>::Identity() * 0.1;

        // Tube K: place A_cl at 0.90 -> K = (0.90-1) / Kp_step
        // For scalar system: rho(|A_cl|) = 0.90 < 1 (check)
        Eigen::MatrixXd K1(1, 1);
        K1(0,0) = (0.90 - 1.0) / Kp_step;   // negative (stabilising for A=1)
        p.K = K1;

        // Disturbance bound: 0.15mm position
        p.wMax = Eigen::Matrix<double,1,1>::Constant(1.5e-4);
        p.uMin = Eigen::Matrix<double,1,1>::Constant(-1.0);
        p.uMax = Eigen::Matrix<double,1,1>::Constant( 1.0);
        p.Ts   = ss2.Ts;

        return ctrl::TubeMPC(ss1, p);
    }())
    , op_(op)
{}

double TubeMPCCtrl::compute(const Eigen::Vector4d& y, double r)
{
    // 1-state model: state = position deviation
    Eigen::Matrix<double,1,1> x1_dev;
    x1_dev(0) = y[0] - op_.x_eq;
    Eigen::Matrix<double,1,1> y_ref;
    y_ref(0) = r - op_.x_eq;

    Eigen::VectorXd u = tmpc_.computeRef(x1_dev, y_ref);
    return std::clamp(u(0), -1.0, 1.0);
}

void TubeMPCCtrl::reset()
{
    tmpc_.reset();
}

// ============================================================================
// 7. LeadLagPIDCtrl
// DiscreteLeadLag (zero=10, pole=50 rad/s) feeds into PI.
// Lead ratio = 5x adds ~60^\circ extra phase at crossover.
// ============================================================================

LeadLagPIDCtrl::LeadLagPIDCtrl(const SmismoOperatingPoint& op, double Ts)
    : leadlag_(ctrl::LeadLagParams{
        .continuousZero = 10.0,
        .continuousPole = 50.0,
        .gain           = 1.0
      }, Ts)
    , pid_([&]() {
        ctrl::PIDParams p;
        p.Kp   = 5.0;
        p.Ki   = 1.0;
        p.Kd   = 0.0;
        p.Kb   = 1.0;
        p.uMin = -1.0;
        p.uMax =  1.0;
        return ctrl::DiscretePID(p, Ts);
    }())
{
    (void)op;
}

double LeadLagPIDCtrl::compute(const Eigen::Vector4d& y, double r)
{
    double e    = r - y[0];
    double e_ll = leadlag_.compute(e);   // phase-led error
    return std::clamp(pid_.compute(e_ll), -1.0, 1.0);
}

void LeadLagPIDCtrl::reset()
{
    leadlag_.reset();
    pid_.reset();
}

// ============================================================================
// 8. GPCRLSCtrl
// Np=50 (250 ms look-ahead), Nu=10, alpha=0.8 soft reference.
// RLS(na=2, nb=1): identifies 2nd-order ARX model; half-life approx = 138 steps.
// ============================================================================

GPCRLSCtrl::GPCRLSCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts)
    : gpc_(ss2, []() {
        ctrl::GPCParams p;
        p.Np    = 50;
        p.Nu    = 10;
        p.rho_y = 1.0;
        p.rho_u = 0.5;
        p.alpha = 0.8;
        p.uMin  = -1.0;
        p.uMax  =  1.0;
        return p;
      }())
    , rls_(2, 1, Ts, 0.995)
    , op_(op)
{}

double GPCRLSCtrl::compute(const Eigen::Vector4d& y, double r)
{
    double y_dev = y[0] - op_.x_eq;
    double r_dev = r    - op_.x_eq;

    rls_.update(y_dev, u_prev_);
    ++step_count_;

    if (step_count_ > kRLSWarmup && step_count_ % kRLSUpdateInterval == 0)
        gpc_.setPlant(rls_.toStateSpace());

    double u = std::clamp(gpc_.computeRef(y_dev, r_dev), -1.0, 1.0);
    u_prev_ = u;
    return u;
}

void GPCRLSCtrl::reset()
{
    gpc_.reset();
    rls_.reset();
    u_prev_    = 0.0;
    step_count_ = 0;
}

// ============================================================================
// 9. MRACSmismoCtrl
// Reference model pole a_m=0.85 -> reference time constant approx = Ts/(-ln(0.85)) approx = 31 ms.
// Plant is minimum-phase (no zeros for first-order effective position dynamics).
// compute() receives plant output y[0], NOT error (MRACController API convention).
// ============================================================================

MRACSmismoCtrl::MRACSmismoCtrl(const SmismoOperatingPoint& op, double Ts)
    : mrac_(ctrl::MRACParams{
        .a_m       = 0.85,
        .b_m       = 0.15,
        .gamma_r   = 1.0,
        .gamma_y   = 1.0,
        .sigma     = 0.02,
        .theta_max = 50.0,
        .uMin      = -1.0,
        .uMax      =  1.0
      }, Ts)
{
    (void)op;
}

double MRACSmismoCtrl::compute(const Eigen::Vector4d& y, double r)
{
    mrac_.setReference(r);
    return std::clamp(mrac_.compute(y[0]), -1.0, 1.0);
}

void MRACSmismoCtrl::reset() { mrac_.reset(); }

// ============================================================================
// 10. GainScheduledSmismoCtrl
// Five PI schedule points (x = 0.05..0.25 m), NearestNeighbor bumpless switching.
// Gains inversely scaled with hydraulic velocity gain Kv(x) = B[1]/(1-A[1,1])
// from linearise2() at each x, compensating for volume-dependent gain variation.
// ============================================================================

GainScheduledSmismoCtrl::GainScheduledSmismoCtrl(const SmismoOperatingPoint& op, double Ts)
    : sched_(Ts, ctrl::GainScheduleMode::NearestNeighbor), op_(op)
{
    // Nominal gain at op.x_eq for normalisation
    SmismoOperatingPoint op_nom = op;
    SmismoPlant plant_nom(SmismoParams{}, op_nom);
    ctrl::StateSpace ss_nom = plant_nom.linearise2(Ts);
    const double Kv_nom = ss_nom.B(1, 0) / (1.0 - ss_nom.A(1, 1));

    const double xs[] = {0.05, 0.10, 0.15, 0.20, 0.25};
    for (double x_sched : xs) {
        SmismoOperatingPoint op_x = op;
        op_x.x_eq = x_sched;
        SmismoPlant plant_x(SmismoParams{}, op_x);
        ctrl::StateSpace ss_x = plant_x.linearise2(Ts);
        const double Kv_x   = ss_x.B(1, 0) / (1.0 - ss_x.A(1, 1));
        const double g_ratio = Kv_nom / Kv_x;   // ~1 at nominal, varies +/-15%

        ctrl::PIDParams p;
        p.Kp   = 8.0  * g_ratio;
        p.Ki   = 1.5  * g_ratio;
        p.Kd   = 0.0;
        p.Kb   = 1.0;
        p.uMin = -1.0;
        p.uMax =  1.0;
        sched_.addSchedulePoint(x_sched,
            std::make_shared<ctrl::DiscretePID>(p, Ts));
    }
}

double GainScheduledSmismoCtrl::compute(const Eigen::Vector4d& y, double r)
{
    sched_.setSchedulingParam(y[0]);
    return std::clamp(sched_.compute(r - y[0]), -1.0, 1.0);
}

void GainScheduledSmismoCtrl::reset() { sched_.reset(); }

// ============================================================================
// 11. EKFLQRSmismoCtrl
// EKF on 4-state linear model (from linearise4) for pressure estimation.
// Measurement: position only (p=1). LQR uses full 4-state estimate.
// Using linear A4/B4 as process model makes this equivalent to a Kalman filter
// but demonstrates the EKF class and enables future nonlinear extension.
// ============================================================================

EKFLQRSmismoCtrl::EKFLQRSmismoCtrl(const ctrl::StateSpace& ss4,
                                     const SmismoOperatingPoint& op, double Ts)
    : ekf_([&]() {
        const Eigen::MatrixXd A4 = ss4.A;
        const Eigen::MatrixXd B4 = ss4.B;

        // Process: x_dev[k+1] = A4*x_dev[k] + B4*u_dev[k]
        ctrl::StateFunc f = [A4, B4](const Eigen::VectorXd& x,
                                      const Eigen::VectorXd& u) -> Eigen::VectorXd {
            return A4 * x + B4 * u;
        };
        // Measurement: position deviation only
        ctrl::MeasFunc h = [](const Eigen::VectorXd& x,
                               const Eigen::VectorXd&) -> Eigen::VectorXd {
            return Eigen::Matrix<double, 1, 1>(x(0));
        };
        ctrl::JacobianFn Fjac = [A4](const Eigen::VectorXd&,
                                      const Eigen::VectorXd&) -> Eigen::MatrixXd {
            return A4;
        };
        ctrl::JacobianFn Hjac = [](const Eigen::VectorXd&,
                                    const Eigen::VectorXd&) -> Eigen::MatrixXd {
            return (Eigen::Matrix<double, 1, 4>() << 1.0, 0.0, 0.0, 0.0).finished();
        };

        // Process noise: position + velocity dominant, pressures low
        Eigen::Matrix4d Qn = Eigen::Vector4d(1e-6, 1e-3, 1.0, 1.0).asDiagonal();
        // Measurement noise: 0.1 mm position sensor
        Eigen::Matrix<double, 1, 1> Rn;
        Rn(0, 0) = 1e-8;

        return ctrl::ExtendedKalmanFilter(4, 1, f, h, Fjac, Hjac, Qn, Rn, Ts);
    }())
    , lqr_([&]() {
        // Use 2-state model [x,v] for LQR to avoid DARE instability with 4-state model.
        // (4-state pressure coupling makes some modes non-stabilizable via linearise4.)
        SmismoPlant plant(SmismoParams{}, op);
        ctrl::StateSpace ss2 = plant.linearise2(ss4.Ts);
        ctrl::LQRParams p;
        p.Q = Eigen::Vector2d(1e4, 10.0).asDiagonal();
        p.R = Eigen::Matrix<double, 1, 1>::Identity() * 1.0;
        return ctrl::DiscreteLQR(ss2, p);
    }())
    , op_(op)
    , Ts_(ss4.Ts)
{
    // EKF state is deviation: initialise at zero (= at operating point)
    ekf_.setState(Eigen::Vector4d::Zero());
}

double EKFLQRSmismoCtrl::compute(const Eigen::Vector4d& y, double r)
{
    // EKF step in deviation space
    Eigen::Matrix<double, 1, 1> y_meas(y[0] - op_.x_eq);
    Eigen::Matrix<double, 1, 1> u_in(u_prev_);
    ekf_.step(y_meas, u_in);

    // Use only [x_dev, v_dev] from 4-state EKF estimate for 2-state LQR
    Eigen::Vector2d x2_dev(ekf_.state()(0), ekf_.state()(1));
    Eigen::Vector2d x2_ref(r - op_.x_eq, 0.0);

    Eigen::VectorXd u_lqr = lqr_.compute(x2_dev, x2_ref);

    // Outer position integral for zero steady-state error (same pattern as LQRCtrl)
    pos_integral_ += Ts_ * (r - y[0]);
    pos_integral_ = std::clamp(pos_integral_, -2.0, 2.0);

    double u_cmd = std::clamp(u_lqr(0) + Ki * pos_integral_, -1.0, 1.0);
    u_prev_ = u_cmd;
    return u_cmd;
}

void EKFLQRSmismoCtrl::reset()
{
    ekf_.reset();
    ekf_.setState(Eigen::Vector4d::Zero());
    u_prev_       = 0.0;
    pos_integral_ = 0.0;
}

// ============================================================================
// 12. HinfSmismoCtrl
// Mixed-sensitivity design on 2-state SISO model.
// W1 = tracking (omega_B=2 rad/s), W2 = effort (gain=1), W3 = robustness (omega_T=20).
// Falls back to PID if H-inf synthesis is infeasible.
// ============================================================================

HinfSmismoCtrl::HinfSmismoCtrl(const ctrl::StateSpace& ss2, const SmismoOperatingPoint& op, double Ts)
    : fallback_pid_([&]() {
        ctrl::PIDParams p;
        p.Kp   = 8.0;
        p.Ki   = 1.5;
        p.Kd   = 0.0;
        p.Kb   = 1.0;
        p.uMin = -1.0;
        p.uMax =  1.0;
        return ctrl::DiscretePID(p, Ts);
    }())
{
    (void)op;

    // ss2 has C=I2 (MIMO). Build SISO position-only model for mixed-sensitivity design.
    Eigen::Matrix<double,1,2> C_pos;
    C_pos << 1.0, 0.0;
    Eigen::Matrix<double,1,1> D_zero = Eigen::Matrix<double,1,1>::Zero();
    ctrl::StateSpace G_siso(ss2.A, ss2.B, C_pos, D_zero, Ts);

    try {
        auto W1 = ctrl::MixedSensitivity::makeW1(2.0, 1.5, 0.001, Ts);
        auto W2 = ctrl::MixedSensitivity::makeW2constant(1.0, Ts);
        auto W3 = ctrl::MixedSensitivity::makeW3(20.0, 1.5, 0.001, Ts);
        auto P  = ctrl::MixedSensitivity::build(G_siso, W1, W2, W3);

        ctrl::HinfParams hp;
        hp.gammaInit = 2.0;

        ctrl::HinfResult result = ctrl::DiscreteHinf::solve(P, hp);
        if (result.feasible)
            hinf_.emplace(result);
    } catch (const std::exception&) {
        // synthesis failed; fallback_pid_ used
    }
}

double HinfSmismoCtrl::compute(const Eigen::Vector4d& y, double r)
{
    double e = r - y[0];
    if (hinf_)
        return std::clamp(hinf_->compute(e), -1.0, 1.0);
    return std::clamp(fallback_pid_.compute(e), -1.0, 1.0);
}

void HinfSmismoCtrl::reset()
{
    if (hinf_) hinf_->reset();
    fallback_pid_.reset();
}

// ============================================================================
// 13. NMPCSmismoCtrl
// ============================================================================

NMPCSmismoCtrl::NMPCSmismoCtrl(const ctrl::StateSpace& ss4,
                                 const SmismoOperatingPoint& op,
                                 double Ts)
    : nmpc_([&]() {
        ctrl::NMPCParams np;
        np.n_states  = 4;
        np.n_inputs  = 1;
        np.n_outputs = 1;    // position only
        np.Np        = 30;
        np.Nu        = 5;
        np.rho_y     = 1e6;   // position error in m^2, force in deviation [-1,1]
        np.rho_u     = 0.01;
        np.uMin      = -1.0;
        np.uMax      =  1.0;
        np.Ts        = Ts;

        // Capture SmismoParams for the closure
        SmismoParams p = SmismoParams{};
        const double x_eq  = op.x_eq;
        const double v_eq  = op.v_eq;
        const double p1_eq = op.p1_eq;
        const double p2_eq = op.p2_eq;

        // Nonlinear 4-state discrete dynamics with position-dependent volumes.
        // State: [dx, dv, dp1, dp2]; input: [u_cmd] deviation \in [-1,1].
        // The volume nonlinearity V1(x), V2(x) is the key captured nonlinearity.
        auto f_4state = [p, x_eq, v_eq, p1_eq, p2_eq, Ts]
                        (const Eigen::VectorXd& x_dev, const Eigen::VectorXd& u_dev)
                        -> Eigen::VectorXd {
            double pos = x_eq  + x_dev(0);
            double vel = v_eq  + x_dev(1);
            double pp1 = p1_eq + x_dev(2);
            double pp2 = p2_eq + x_dev(3);
            double u   = std::clamp(u_dev(0), -1.0, 1.0);

            // Position-dependent volumes
            double V1 = p.V10 + p.A1 * std::clamp(pos, 0.0, p.L);
            double V2 = p.V20 + p.A2 * (p.L - std::clamp(pos, 0.0, p.L));

            // Orifice flow (linearized at operating point using signed sqrt)
            double dP1 = std::max(15.0e6 - pp1, 0.0);  // ps - p1
            double dP2 = std::max(pp2, 0.0);             // p2 - pt (Pt=0)
            double sign_u = (u >= 0) ? 1.0 : -1.0;
            double Q1 = sign_u * p.Cd * p.Amax * std::abs(u) * std::sqrt(2.0 * dP1 / p.rho);
            double Q2 = sign_u * p.Cd * p.Amax * std::abs(u) * std::sqrt(2.0 * dP2 / p.rho);

            // State derivatives
            double aF   = (p.A1 * pp1 - p.A2 * pp2 - p.Bv * vel - p.F_ext) / p.M;
            double ap1  = p.beta_e / V1 * (Q1 - p.A1 * vel);
            double ap2  = p.beta_e / V2 * (p.A2 * vel - Q2);

            // Euler step in deviation space
            Eigen::VectorXd x_next(4);
            x_next(0) = x_dev(0) + Ts * (vel - v_eq);
            x_next(1) = x_dev(1) + Ts * (aF  - (p.A1 * p1_eq - p.A2 * p2_eq - p.Bv * v_eq - p.F_ext) / p.M);
            x_next(2) = x_dev(2) + Ts * ap1;
            x_next(3) = x_dev(3) + Ts * ap2;
            return x_next;
        };

        // C = [1, 0, 0, 0]: output = position deviation
        Eigen::MatrixXd C_out(1, 4);
        C_out << 1.0, 0.0, 0.0, 0.0;

        return ctrl::NonlinearMPC(np, f_4state, C_out);
    }())
    , op_(op)
{
    (void)ss4;
}

double NMPCSmismoCtrl::compute(const Eigen::Vector4d& y, double r)
{
    // State deviation: [dx, dv, dp1, dp2]
    Eigen::VectorXd x_dev(4);
    x_dev(0) = y(0) - op_.x_eq;
    x_dev(1) = y(1) - op_.v_eq;
    x_dev(2) = y(2) - op_.p1_eq;
    x_dev(3) = y(3) - op_.p2_eq;

    // Reference: position deviation
    Eigen::VectorXd y_ref(1);
    y_ref(0) = r - op_.x_eq;

    Eigen::VectorXd u_opt = nmpc_.computeRef(x_dev, y_ref);
    return std::clamp(u_opt(0), -1.0, 1.0);
}

void NMPCSmismoCtrl::reset()
{
    nmpc_.reset();
}

// ============================================================================
// 14. FLSmismoCtrl
// ============================================================================

FLSmismoCtrl::FLSmismoCtrl(double b0, const SmismoOperatingPoint& op, double Ts)
    : fl_([&]() -> ctrl::FeedbackLinearisationController {
        // Model: v_dot = -Bv/M * v + b0 * u  (relative degree 1 from u to v)
        SmismoParams p_def{};

        auto f_drift = [bv = p_def.Bv, M = p_def.M](const Eigen::VectorXd& x, double) {
            double vel = x(1);   // velocity is state index 1 (2D state [pos, vel])
            return -(bv / M) * vel;
        };

        auto g_gain = [b0](const Eigen::VectorXd&, double) { return b0; };

        // Inner PD: tracks velocity error; output is virtual acceleration
        ctrl::PIDParams pp;
        pp.Kp   = 50.0;    // proportional velocity control
        pp.Ki   = 1.0;
        pp.Kd   = 0.5;
        pp.N    = 20.0;
        pp.Kb   = 1.0;
        pp.uMin = -5.0;    // virtual acceleration [m/s^2]
        pp.uMax =  5.0;
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);

        ctrl::FLParams flp;
        flp.uMin              = -1.0;
        flp.uMax              =  1.0;
        flp.regularisationEps = 1e-6;

        return ctrl::FeedbackLinearisationController(f_drift, g_gain, inner, flp, Ts);
    }())
    , op_(op)
{}

double FLSmismoCtrl::compute(const Eigen::Vector4d& y, double r)
{
    double pos = y(0);
    double vel = y(1);

    // Outer proportional position loop -> velocity reference
    double v_ref = Kp_pos * (r - pos);
    v_ref = std::clamp(v_ref, -0.5, 0.5);   // cap velocity reference to avoid windup

    // Set 2D state [pos, vel] for FL controller
    Eigen::VectorXd x_2d(2);
    x_2d(0) = pos;
    x_2d(1) = vel;
    fl_.setState(x_2d);

    // FL computes u from velocity error
    return fl_.compute(v_ref - vel);
}

void FLSmismoCtrl::reset()
{
    fl_.reset();
}

} // namespace smismo
