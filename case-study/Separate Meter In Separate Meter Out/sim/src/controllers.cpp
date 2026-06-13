#include "controllers.h"
#include <cmath>
#include <algorithm>

using namespace Eigen;

namespace smismo {

// ---------------------------------------------------------------------------
// Shared helper: discrete 2-state working-side model [x_L, v_L], input u [V]
//   dx_L/dt = v_L
//   dv_L/dt = (K_V*u - v_L) / TAU_V
// ---------------------------------------------------------------------------
static ctrl::StateSpace makeVelSS(const PlantParams& p)
{
    MatrixXd Ac(2, 2); Ac << 0.0, 1.0,
                             0.0, -1.0 / TAU_V;
    MatrixXd Bc(2, 1); Bc << 0.0, K_V / TAU_V;
    MatrixXd Cc(1, 2); Cc << 1.0, 0.0;
    MatrixXd Dc(1, 1); Dc << 0.0;
    ctrl::StateSpace ss_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ss_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// Bryson LQR weights shared by LQR/LQG/TubeMPC designs
static ctrl::LQRParams brysonLQR()
{
    ctrl::LQRParams lp;
    lp.Q = MatrixXd::Zero(2, 2);
    lp.Q(0, 0) = 1.0 / (0.02 * 0.02);  // x_L: 2 cm max deviation
    lp.Q(1, 1) = 1.0 / (0.4 * 0.4);    // v_L: 0.4 m/s max deviation
    lp.R = MatrixXd::Constant(1, 1, 1.0 / (U_MAX * U_MAX));
    return lp;
}

// ===========================================================================
// ValveAllocator
//
// Liu (2009) Fig. 10 dual-loop structure, generalised to both directions:
//   working side  = chamber metered IN from supply (PDCV1 extend / PDCV2 retract)
//   off side      = discharging chamber, regulated to P_bd by FF + PI throttling
//                   to tank (u <= 0 on the off-side valve).
// ===========================================================================
ValveAllocator::ValveAllocator(const PlantParams& p) : p_(p) {}

void ValveAllocator::reset() { mode_ = +1; integ_ = 0.0; }

ValveAllocator::Cmd ValveAllocator::allocate(double u_ctrl,
                                             const SmismoPlant::State& state)
{
    const double v  = state(1);
    const double P1 = state(2);
    const double P2 = state(3);

    // Mode hysteresis on the working-side command
    int new_mode = mode_;
    if      (u_ctrl >  U_HYST) new_mode = +1;
    else if (u_ctrl < -U_HYST) new_mode = -1;
    if (new_mode != mode_) { mode_ = new_mode; integ_ = 0.0; }  // bumpless-ish

    // Off-side chamber quantities
    const double P_off   = (mode_ > 0) ? P2 : P1;
    const double A_off   = (mode_ > 0) ? p_.A2 : p_.A1;
    const double Q_nom_o = (mode_ > 0) ? p_.Q_nom2 : p_.Q_nom1;
    const double K_q     = Q_nom_o / std::sqrt(p_.DP_nom);

    // Flow-matching feedforward (Liu Fig. 10 "feed forward control signal"):
    // discharge flow A_off*|v| through the off-side orifice at DP = P_bd - P_r
    const double v_dis = (mode_ > 0) ? std::max(v, 0.0) : std::max(-v, 0.0);
    const double dp_bd = std::max(p_.P_bd - p_.P_r, 1.0e4);
    double u_ff = p_.u_max * (A_off * v_dis) / (K_q * std::sqrt(dp_bd));
    u_ff = std::clamp(u_ff, 0.0, p_.u_max);

    // Backpressure PI (positive output = more opening to tank)
    const double e_bp  = P_off - p_.P_bd;
    const double u_raw = u_ff + KP_BP * e_bp + integ_;
    const double u_bp  = std::clamp(u_raw, 0.0, p_.u_max);

    // Conditional integration anti-windup
    const bool sat_lo = (u_raw <= 0.0       && e_bp < 0.0);
    const bool sat_hi = (u_raw >= p_.u_max  && e_bp > 0.0);
    if (!sat_lo && !sat_hi) integ_ += KI_BP * e_bp * p_.Ts;

    Cmd cmd{0.0, 0.0};
    if (mode_ > 0) {  // extend: PDCV1 working (supply->cap), PDCV2 off-side
        cmd.u1 = std::clamp(u_ctrl, 0.0, p_.u_max);
        cmd.u2 = -u_bp;
    } else {          // retract: PDCV2 working (supply->rod), PDCV1 off-side
        cmd.u2 = std::clamp(-u_ctrl, 0.0, p_.u_max);
        cmd.u1 = -u_bp;
    }
    return cmd;
}

// ===========================================================================
// 1. PID
// Crossover ~ Kp*K_V = 8.4 rad/s; Kd zero at 15 rad/s; Ki corner 0.67 rad/s.
// ===========================================================================
PIDPosCtrl::PIDPosCtrl(const PlantParams& p)
    : pid_(ctrl::PIDParams{
        .Kp   = 60.0,
        .Ki   = 40.0,
        .Kd   = 4.0,
        .N    = 10.0,
        .uMin = -U_MAX,
        .uMax =  U_MAX,
        .Kb   = 1.0
      }, p.Ts)
{}

double PIDPosCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    return clampU(pid_.compute(x_ref - state(0)));
}

void PIDPosCtrl::reset() { pid_.reset(); }

// ===========================================================================
// 2. CascadePID
// Outer P: v_ref = Kp_pos*(x_ref - x_L), clamped to +/-V_REF_MAX.
// Inner velocity PI: ~39 rad/s crossover (Kp=10 -> loop gain 1.4 at DC).
// ===========================================================================
CascadePIDCtrl::CascadePIDCtrl(const PlantParams& p)
    : pid_v_(ctrl::PIDParams{
        .Kp   = 10.0,
        .Ki   = 100.0,
        .Kd   = 0.0,
        .N    = 10.0,
        .uMin = -U_MAX,
        .uMax =  U_MAX,
        .Kb   = 1.0
      }, p.Ts)
{}

double CascadePIDCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    const double v_ref = std::clamp(Kp_pos_ * (x_ref - state(0)),
                                    -V_REF_MAX, V_REF_MAX);
    return clampU(pid_v_.compute(v_ref - state(1)));
}

void CascadePIDCtrl::reset() { pid_v_.reset(); }

// ===========================================================================
// 3. LQR
// ===========================================================================
LQRCtrl::LQRCtrl(const PlantParams& p)
{
    ctrl::DiscreteLQR lqr_design(makeVelSS(p), brysonLQR());
    K_ = lqr_design.gainMatrix();   // 1x2
}

double LQRCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    Vector2d x_dev(state(0) - x_ref, state(1));
    return clampU(-(K_ * x_dev)(0));
}

// ===========================================================================
// 4. LQG
// Position-only measurement; KF reconstructs v_L. Same Bryson LQR weights.
// ===========================================================================
LQGCtrl::LQGCtrl(const PlantParams& p)
    : lqg_([&]() {
        MatrixXd Q_kf = MatrixXd::Zero(2, 2);
        Q_kf(0, 0) = 1.0e-9;   // position process noise [m^2]
        Q_kf(1, 1) = 1.0e-4;   // velocity process noise [(m/s)^2] (load/friction)
        MatrixXd R_kf = MatrixXd::Constant(1, 1, 1.0e-10);  // 10 um sensor std
        return ctrl::DiscreteLQG(makeVelSS(p), brysonLQR(), Q_kf, R_kf);
      }())
    , u_prev_(VectorXd::Zero(1))
{}

double LQGCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    VectorXd y(1);     y(0) = state(0);
    VectorXd x_ref_v(2); x_ref_v << x_ref, 0.0;
    VectorXd u = lqg_.step(y, u_prev_, x_ref_v);
    u(0) = clampU(u(0));
    u_prev_ = u;
    return u(0);
}

void LQGCtrl::reset()
{
    lqg_.reset();
    u_prev_.setZero();
}

// ===========================================================================
// 5. MPC
// ===========================================================================
MPCCtrl::MPCCtrl(const PlantParams& p)
{
    ctrl::MPCParams mp;
    mp.Np    = 60;     // 60 ms prediction at Ts = 1 ms
    mp.Nc    = 5;
    mp.rho_y = 1.0;
    mp.rho_u = 0.01;
    mp.uMin  = -U_MAX;
    mp.uMax  =  U_MAX;
    mp.duMin = -2.0;
    mp.duMax =  2.0;
    mp.qpMaxIter = 200;
    mpc_ = std::make_unique<ctrl::DiscreteMPC>(makeVelSS(p), mp);
}

double MPCCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    VectorXd x2(2); x2 << state(0), state(1);
    VectorXd r(1);  r(0) = x_ref;
    mpc_->setState(x2);
    VectorXd u = mpc_->computeRef(x2, r);
    return clampU(u(0));
}

void MPCCtrl::reset() { mpc_->reset(); }

// ===========================================================================
// 6. ADRC
// omega_o*Ts = 200*0.001 = 0.2 < 0.5; b0 = K_V/TAU_V = 5.6 (m/s^2)/V.
// ===========================================================================
ADRCCtrl::ADRCCtrl(const PlantParams& p)
    : adrc_(ctrl::ADRCParams{
        .omega_o = 200.0,
        .omega_c = 30.0,
        .b0      = K_V / TAU_V,
        .uMin    = -U_MAX,
        .uMax    =  U_MAX
      }, p.Ts)
{}

double ADRCCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    return clampU(adrc_.computeTracking(state(0), x_ref));
}

void ADRCCtrl::reset() { adrc_.reset(); }

// ===========================================================================
// 7. SMC
// s = e + c_de*(e[k]-e[k-1]) with c_de = 50 -> lead zero at 20 rad/s
// (c_de*Delta_e = 50*Ts*de/dt = 0.05*de/dt at Ts = 1 ms).
// Inside boundary layer: u = -(K/phi)*s = -80*s -> ~11 rad/s crossover.
// DiscreteSMC convention: compute(y - ref).
// ===========================================================================
SMCCtrl::SMCCtrl(const PlantParams& p)
    : smc_(ctrl::SMCParams{
        .c_e  = 1.0,
        .c_de = 50.0,
        .K    = 4.0,
        .phi  = 0.05,
        .uMin = -U_MAX,
        .uMax =  U_MAX
      }, p.Ts)
{}

double SMCCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    return clampU(smc_.compute(state(0) - x_ref));
}

void SMCCtrl::reset() { smc_.reset(); }

// ===========================================================================
// 8. FeedbackLinearisation
// Velocity-level linearisation of the orifice equation: x_L_dot = g(x)*u with
//   g(x) = K_q,work * sqrt(P_s - P_work) / (u_max * A_work)
// (direction from u_prev). Inner PID produces the virtual velocity command.
// This is the Liu (2009) calculation-flow-rate control idea: measured chamber
// pressures compensate the sqrt(DP) flow-gain variation.
// ===========================================================================
FLCtrl::FLCtrl(const PlantParams& p)
{
    auto f = [](const VectorXd&, double) -> double { return 0.0; };

    auto g = [p](const VectorXd& x, double u_prev) -> double {
        // x = [x_L, v_L, P1, P2]
        double dp, K_q, A;
        if (u_prev >= 0.0) {
            dp  = p.P_s - x(2);
            K_q = p.Q_nom1 / std::sqrt(p.DP_nom);
            A   = p.A1;
        } else {
            dp  = p.P_s - x(3);
            K_q = p.Q_nom2 / std::sqrt(p.DP_nom);
            A   = p.A2;
        }
        const double gain = K_q * std::sqrt(std::max(dp, 0.0)) / (p.u_max * A);
        return std::max(gain, 0.02);  // keep invertible near DP = 0
    };

    auto inner = std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
        .Kp   = 8.0,
        .Ki   = 2.0,
        .Kd   = 0.05,
        .N    = 10.0,
        .uMin = -V_REF_MAX,
        .uMax =  V_REF_MAX,
        .Kb   = 1.0
      }, p.Ts);

    ctrl::FLParams fp;
    fp.uMin = -U_MAX;
    fp.uMax =  U_MAX;

    fl_ = std::make_unique<ctrl::FeedbackLinearisationController>(
        f, g, inner, fp, p.Ts);
}

double FLCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    VectorXd x4(4); x4 << state(0), state(1), state(2), state(3);
    fl_->setState(x4);
    return clampU(fl_->compute(x_ref - state(0)));
}

void FLCtrl::reset() { fl_->reset(); }

// ===========================================================================
// 9. TubeMPC
// ===========================================================================
TubeMPCCtrl::TubeMPCCtrl(const PlantParams& p)
{
    ctrl::StateSpace sys = makeVelSS(p);

    ctrl::DiscreteLQR lqr_design(sys, brysonLQR());
    MatrixXd K_tube = -lqr_design.gainMatrix();  // u_tube = K*(x - x_nom)

    ctrl::TubeMPCParams tp;
    tp.Np   = 10;
    tp.Nu   = 3;
    tp.Q    = MatrixXd::Constant(1, 1, 1.0 / (0.02 * 0.02));  // output (x_L)
    tp.R    = MatrixXd::Constant(1, 1, 1.0 / (U_MAX * U_MAX));
    tp.K    = K_tube;
    tp.wMax = VectorXd(2); tp.wMax << 1.0e-4, 5.0e-3;  // per-step model error
    tp.uMin = VectorXd::Constant(1, -U_MAX);
    tp.uMax = VectorXd::Constant(1,  U_MAX);
    tp.Ts   = p.Ts;

    tmpc_ = std::make_unique<ctrl::TubeMPC>(sys, tp);
}

double TubeMPCCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    VectorXd x2(2); x2 << state(0), state(1);
    VectorXd r(1);  r(0) = x_ref;
    VectorXd u = tmpc_->computeRef(x2, r);
    return clampU(u(0));
}

void TubeMPCCtrl::reset() { tmpc_->reset(); }

// ===========================================================================
// 10. L1Adaptive
// Reference model pole 5 rad/s; LP filter omega_c = 20 rad/s.
// Convention: setReference(x_ref) then compute(x_L) -> absolute u [V].
// ===========================================================================
L1Ctrl::L1Ctrl(const PlantParams& p)
    : l1_(ctrl::L1AdaptiveController::Params{
        .a_m       = std::exp(-5.0 * p.Ts),
        .b_m       = 1.0 - std::exp(-5.0 * p.Ts),
        .k_g       = 1.0,
        .Gamma     = 100.0,
        .omega_c   = 20.0,
        .sigma_max = 50.0,
        .Q_lyap    = 1.0,
        .uMin      = -U_MAX,
        .uMax      =  U_MAX
      }, p.Ts)
{}

double L1Ctrl::compute(const SmismoPlant::State& state, double x_ref)
{
    l1_.setReference(x_ref);
    return clampU(l1_.compute(state(0)));
}

void L1Ctrl::reset() { l1_.reset(); }

// ===========================================================================
// 11. GainScheduled
// Scheduled on v_L: motion gains at +/-0.3 m/s (resistive/overrunning sides),
// precision gains near v = 0.
// ===========================================================================
GainSchedCtrl::GainSchedCtrl(const PlantParams& p)
    : gs_(p.Ts)
{
    auto motion_pid = [&]() {
        return std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
            .Kp = 45.0, .Ki = 20.0, .Kd = 3.0, .N = 10.0,
            .uMin = -U_MAX, .uMax = U_MAX, .Kb = 1.0}, p.Ts);
    };
    gs_.addSchedulePoint(-0.3, motion_pid());
    gs_.addSchedulePoint(0.0,
        std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
            .Kp = 60.0, .Ki = 40.0, .Kd = 4.0, .N = 10.0,
            .uMin = -U_MAX, .uMax = U_MAX, .Kb = 1.0}, p.Ts));
    gs_.addSchedulePoint(0.3, motion_pid());
}

double GainSchedCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    gs_.setSchedulingParam(state(1));   // schedule on measured velocity
    return clampU(gs_.compute(x_ref - state(0)));
}

void GainSchedCtrl::reset() { gs_.reset(); }

// ===========================================================================
// 12. NonlinearMPC
// 2-state model with flow-saturation nonlinearity (tanh on commanded velocity)
// at an internal 10 ms prediction step; recomputed every control step (RTI).
// ===========================================================================
NMPCCtrl::NMPCCtrl(const PlantParams& p)
{
    static constexpr double dt_m  = 0.010;  // internal model step [s]
    static constexpr double v_sat = 1.0;    // flow-limited velocity [m/s]

    ctrl::NonlinearMPC::DiscreteDynamics f =
        [](const VectorXd& x, const VectorXd& u) -> VectorXd {
            double v_cmd = K_V * u(0);
            v_cmd = v_sat * std::tanh(v_cmd / v_sat);
            VectorXd xn(2);
            xn(0) = x(0) + dt_m * x(1);
            xn(1) = x(1) + dt_m / TAU_V * (v_cmd - x(1));
            return xn;
        };

    ctrl::NMPCParams np;
    np.Np        = 12;     // 120 ms horizon
    np.Nu        = 3;
    np.rho_y     = 1.0;
    np.rho_u     = 0.05;
    np.uMin      = -U_MAX;
    np.uMax      =  U_MAX;
    np.Ts        = dt_m;
    np.n_states  = 2;
    np.n_inputs  = 1;
    np.n_outputs = 1;

    MatrixXd C(1, 2); C << 1.0, 0.0;
    nmpc_ = std::make_unique<ctrl::NonlinearMPC>(np, f, C);
}

double NMPCCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    VectorXd x2(2); x2 << state(0), state(1);
    VectorXd r(1);  r(0) = x_ref;
    nmpc_->setState(x2);
    VectorXd u = nmpc_->computeRef(x2, r);
    return clampU(u(0));
}

void NMPCCtrl::reset() { nmpc_->reset(); }

// ===========================================================================
// 13. DOBEnergyCtrl
//
// Second-order disturbance observer (Chen 2018 Eq. 29-30) estimates the
// external load force F_hat from P_1, P_2, v_L without differentiating
// acceleration. Adaptive supply pressure reduces hydraulic energy in s05.
//
// Observer (forward Euler at Ts):
//   z_dot = -L * z - L^2 * (A1*P1 - A2*P2)
//   F_hat = z + L * (A1*P1 - A2*P2)
//
// Adaptive supply pressure (Eq. 37 spirit):
//   P_s_cmd = |F_hat|/(A1+A2) + |v_L|*B_v/(A_avg) + P_margin
//   P_s_cmd clamped to [P_MIN_CMD, P_MAX_CMD]
// ===========================================================================

DOBEnergyCtrl::DOBEnergyCtrl(const PlantParams& p)
    : p_(p),
      P_s_cmd_(p.P_s),
      pid_(ctrl::PIDParams{
          .Kp   = 12.0,
          .Ki   = 0.5,
          .Kd   = 0.0,
          .N    = 5.0,
          .uMin = -U_MAX,
          .uMax =  U_MAX,
          .Kb   = 1.0
      }, p.Ts)
{}

double DOBEnergyCtrl::compute(const SmismoPlant::State& state, double x_ref)
{
    const double P1  = state(2);
    const double P2  = state(3);
    const double v_L = state(1);

    // Force balance proxy: F_cyl = A1*P1 - A2*P2  (gravity + friction included)
    const double F_cyl = p_.A1 * P1 - p_.A2 * P2;

    // Observer step (forward Euler, Ts = 1 ms)
    const double z_dot = -L_OBS * z_obs_ - L_OBS * L_OBS * F_cyl;
    z_obs_  += z_dot * p_.Ts;
    F_hat_   = z_obs_ + L_OBS * F_cyl;

    // Adaptive supply pressure — store for beforePlantStep() to apply
    P_s_cmd_ = std::clamp(
        std::abs(F_hat_) / (p_.A1 + p_.A2) + P_MARGIN,
        P_MIN_CMD, P_MAX_CMD);

    // Position control: same cascade structure as PIDPosCtrl
    return pid_.compute(x_ref - state(0));
}

void DOBEnergyCtrl::beforePlantStep(SmismoPlant& plant)
{
    plant.setSupplyPressure(P_s_cmd_);
}

void DOBEnergyCtrl::reset()
{
    pid_.reset();
    z_obs_   = 0.0;
    F_hat_   = 0.0;
    P_s_cmd_ = p_.P_s;
}

} // namespace smismo
