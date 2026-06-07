#include "controllers.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace Eigen;

namespace susp {

// ---------------------------------------------------------------------------
// Shared helper: build discrete-time 2-state body SS  (z_s, dz_s)
//
// Continuous plant ignoring wheel dynamics:
//   m_s * ddz_s = -k_s*z_s - c_s*dz_s + F_act
//   (wheel treated as disturbance absorbed by ADRC/tube)
//
// State: [z_s, dz_s],  Input: F_act [N],  Output: z_s
// ---------------------------------------------------------------------------
static ctrl::StateSpace makeBodySS(const PlantParams& p)
{
    MatrixXd Ac(2,2); Ac << 0.0,           1.0,
                             -p.k_s/p.m_s, -p.c_s/p.m_s;
    MatrixXd Bc(2,1); Bc << 0.0, 1.0/p.m_s;
    MatrixXd Cc(1,2); Cc << 1.0, 0.0;
    MatrixXd Dc(1,1); Dc << 0.0;
    ctrl::StateSpace ss_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ss_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// Full 4-state continuous SS:  [z_s, dz_s, z_u, dz_u], input F_act, output z_s
static ctrl::StateSpace makeFull4SS(const PlantParams& p)
{
    MatrixXd Ac(4,4);
    Ac << 0,           1,          0,           0,
          -p.k_s/p.m_s, -p.c_s/p.m_s, p.k_s/p.m_s, p.c_s/p.m_s,
          0,           0,          0,           1,
          p.k_s/p.m_u, p.c_s/p.m_u, -(p.k_s+p.k_t)/p.m_u, -p.c_s/p.m_u;

    MatrixXd Bc(4,1);
    Bc << 0.0, 1.0/p.m_s, 0.0, -1.0/p.m_u;

    MatrixXd Cc(1,4); Cc << 1.0, 0.0, 0.0, 0.0;
    MatrixXd Dc(1,1); Dc << 0.0;

    ctrl::StateSpace ss_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ss_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// 4-state SS with 2 outputs: z_s and z_u (for LQG measurement model)
static ctrl::StateSpace makeFull4SS_2out(const PlantParams& p)
{
    MatrixXd Ac(4,4);
    Ac << 0,           1,          0,           0,
          -p.k_s/p.m_s, -p.c_s/p.m_s, p.k_s/p.m_s, p.c_s/p.m_s,
          0,           0,          0,           1,
          p.k_s/p.m_u, p.c_s/p.m_u, -(p.k_s+p.k_t)/p.m_u, -p.c_s/p.m_u;

    MatrixXd Bc(4,1);
    Bc << 0.0, 1.0/p.m_s, 0.0, -1.0/p.m_u;

    // Measure both body and wheel displacement
    MatrixXd Cc(2,4);
    Cc << 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0;
    MatrixXd Dc(2,1); Dc << 0.0, 0.0;

    ctrl::StateSpace ss_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ss_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// ===========================================================================
// 2. PID
// ===========================================================================

PIDSuspCtrl::PIDSuspCtrl(const PlantParams& p)
    : pid_(ctrl::PIDParams{
        .Kp   = 2000.0,      // N/m - adds to k_s for effective stiffness
        .Ki   = 30.0,        // N/(m.s)
        .Kd   = 500.0,       // N.s/m - additional damping
        .N    = 10.0,        // derivative filter coefficient
        .uMin = -F_ACT_MAX,
        .uMax =  F_ACT_MAX,
        .Kb   = 1.0          // anti-windup back-calculation
      }, p.Ts)
{}

double PIDSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    // error = r - y = 0 - z_s = -z_s  (track stationary body)
    return pid_.compute(-state(0));
}

void PIDSuspCtrl::reset() { pid_.reset(); }

// ===========================================================================
// 3. ADRC
//
// 2nd-order LADRC on body displacement z_s.
// b0 = 1/m_s (actuator-to-body gain).
// ESO treats road-induced wheel coupling as "total disturbance".
// omega_o*Ts = 20*0.005 = 0.10 < 0.5  (backward-Euler stable).
// ===========================================================================

ADRCSuspCtrl::ADRCSuspCtrl(const PlantParams& p)
    : adrc_(ctrl::ADRCParams{
        .omega_o = 20.0,            // ESO bandwidth [rad/s]
        .omega_c =  8.0,            // controller bandwidth [rad/s]
        .b0      = 1.0 / p.m_s,    // ~4.17e-3
        .uMin    = -F_ACT_MAX,
        .uMax    =  F_ACT_MAX
      }, p.Ts)
{}

double ADRCSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    // ADRC convention: compute(r - y) = compute(0 - z_s) = compute(-z_s)
    return adrc_.compute(-state(0));
}

void ADRCSuspCtrl::reset() { adrc_.reset(); }

// ===========================================================================
// 4. SMC
//
// Sliding surface: s = c_e*e + c_de*(de/dt)
//   e = z_s - 0 = z_s  (DiscreteSMC convention: compute(y - ref))
//   c_e = 1, c_de = lambda*Ts with lambda = 5 rad/s
// Boundary layer phi = 0.005 m; switching gain K = 1200 N.
// ===========================================================================

SMCSuspCtrl::SMCSuspCtrl(const PlantParams& p)
    : smc_(ctrl::SMCParams{
        .c_e  = 1.0,
        .c_de = 5.0 * p.Ts,   // lambda * Ts = 5 * 0.005 = 0.025
        .K    = 1200.0,
        .phi  = 0.005,
        .uMin = -F_ACT_MAX,
        .uMax =  F_ACT_MAX
      }, p.Ts)
{}

double SMCSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    // DiscreteSMC convention: compute(y - ref) = compute(z_s - 0) = compute(z_s)
    return smc_.compute(state(0));
}

void SMCSuspCtrl::reset() { smc_.reset(); }

// ===========================================================================
// 5. LQR
//
// Bryson weights on full 4-state model.
//   x_max = [0.025 m, 0.5 m/s, 0.025 m, 1.0 m/s]
//   u_max = 2000 N
//   Q = diag(1/x_max^2),  R = 1/u_max^2
// Control: u = -K * (x - x_ref), x_ref = 0 (regulate to zero)
// ===========================================================================

LQRSuspCtrl::LQRSuspCtrl(const PlantParams& p)
    : lqr_([&]() {
        ctrl::StateSpace sys = makeFull4SS(p);
        ctrl::LQRParams  lp;
        lp.Q = MatrixXd::Zero(4, 4);
        lp.Q(0,0) = 1.0 / (0.025 * 0.025);  // z_s   weight
        lp.Q(1,1) = 1.0 / (0.5   * 0.5);    // dz_s  weight
        lp.Q(2,2) = 1.0 / (0.025 * 0.025);  // z_u   weight
        lp.Q(3,3) = 1.0 / (1.0   * 1.0);    // dz_u  weight
        lp.R = MatrixXd::Constant(1,1, 1.0 / (p.F_max * p.F_max));
        return ctrl::DiscreteLQR(sys, lp);
      }())
{}

double LQRSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    // LQR compute(x) returns -K*(x - x_ref) as VectorXd; x_ref = 0 by default
    VectorXd xs = state;  // fixed -> dynamic for the VectorXd& API
    VectorXd u = lqr_.compute(xs);
    return std::clamp(u(0), -F_ACT_MAX, F_ACT_MAX);
}

// ===========================================================================
// 6. LQG
//
// DiscreteLQG on the full 4-state model with 2 measurements (z_s, z_u).
// Same Bryson Q/R as LQR; KF noise tuned to sensor precision.
// ===========================================================================

LQGSuspCtrl::LQGSuspCtrl(const PlantParams& p)
    : lqg_([&]() {
        ctrl::StateSpace sys = makeFull4SS_2out(p);
        ctrl::LQRParams  lp;
        lp.Q = MatrixXd::Zero(4, 4);
        lp.Q(0,0) = 1.0 / (0.025 * 0.025);
        lp.Q(1,1) = 1.0 / (0.5   * 0.5);
        lp.Q(2,2) = 1.0 / (0.025 * 0.025);
        lp.Q(3,3) = 1.0 / (1.0   * 1.0);
        lp.R = MatrixXd::Constant(1,1, 1.0 / (p.F_max * p.F_max));

        // Process noise: mainly from unmodelled road disturbance
        MatrixXd Q_kf = MatrixXd::Identity(4,4) * 1e-4;
        Q_kf(2,2) = 1e-3; Q_kf(3,3) = 1e-3;  // more uncertainty on wheel states

        // Measurement noise: typical MEMS accelerometer / position sensor
        MatrixXd R_kf = MatrixXd::Identity(2,2) * 1e-6; // very low sensor noise

        return ctrl::DiscreteLQG(sys, lp, Q_kf, R_kf);
      }())
    , u_prev_(VectorXd::Zero(1))
{}

double LQGSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    // LQG: step(y, u_prev, x_ref) where y = [z_s, z_u]
    VectorXd y(2); y(0) = state(0); y(1) = state(2);
    VectorXd x_ref = VectorXd::Zero(4);
    VectorXd u = lqg_.step(y, u_prev_, x_ref);
    u_prev_ = u;
    return std::clamp(u(0), -F_ACT_MAX, F_ACT_MAX);
}

void LQGSuspCtrl::reset()
{
    lqg_.reset();
    u_prev_.setZero();
}

// ===========================================================================
// 7. MPC
//
// 2-state body model [z_s, dz_s].  Tracks z_s = 0 with constraint on F_act.
// Np = 20 steps (0.1 s prediction), Nu = 5.
// Numerical scaling: F_max = 2000 N, model output in meters.
// ===========================================================================

ctrl::StateSpace MPCSuspCtrl::buildBodySS(const PlantParams& p)
{
    return makeBodySS(p);
}

MPCSuspCtrl::MPCSuspCtrl(const PlantParams& p)
{
    ctrl::StateSpace sys = buildBodySS(p);

    ctrl::MPCParams mp;
    mp.Np    = 20;
    mp.Nc    = 5;
    mp.rho_y = 1.0;         // output (z_s) tracking weight [1/m^2 * scale]
    mp.rho_u = 1e-7;        // input (F_act) move weight [1/N^2 * scale]
    mp.uMin  = -p.F_max;
    mp.uMax  =  p.F_max;
    mp.duMin = -p.F_max;
    mp.duMax =  p.F_max;
    mp.qpMaxIter = 200;

    mpc_ = std::make_unique<ctrl::DiscreteMPC>(sys, mp);
}

double MPCSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    VectorXd x2(2); x2 << state(0), state(1);  // [z_s, dz_s]
    VectorXd r(1);  r(0) = 0.0;

    mpc_->setState(x2);
    VectorXd u = mpc_->computeRef(x2, r);
    return std::clamp(u(0), -F_ACT_MAX, F_ACT_MAX);
}

void MPCSuspCtrl::reset() { mpc_->reset(); }

// ===========================================================================
// 8. MRAC
//
// Discrete-time sigma-modified MRAC on body displacement z_s.
// Reference model: first-order with pole a_m = exp(-4*Ts) ~ 0.980 (fast enough
//   to track the ~1.3 Hz body resonance period within ~0.25 s).
// b_m = 1 - a_m for unity DC gain.
// Conservative adaptation (gamma = 0.01) to handle 4th-order plant coupling.
// Convention: compute(y_plant) = compute(z_s);  setReference(0.0) each step.
// ===========================================================================

MRACSuspCtrl::MRACSuspCtrl(const PlantParams& p)
    : mrac_([&]() {
        const double a_m = std::exp(-4.0 * p.Ts);  // ~ 0.9802 at Ts=0.005
        ctrl::MRACParams mp;
        mp.a_m       = a_m;
        mp.b_m       = 1.0 - a_m;    // DC gain = b_m/(1-a_m) = 1.0
        mp.gamma_r   = 0.01;          // adaptation rate (conservative)
        mp.gamma_y   = 0.01;
        mp.sigma     = 0.01;          // sigma-modification
        mp.theta_max = 5000.0;        // max adaptive gain [N/m]
        mp.uMin      = -F_ACT_MAX;
        mp.uMax      =  F_ACT_MAX;
        return ctrl::MRACController(mp, p.Ts);
      }())
{}

double MRACSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    mrac_.setReference(0.0);          // track stationary body (z_s_ref = 0)
    return mrac_.compute(state(0));   // pass plant output z_s directly
}

void MRACSuspCtrl::reset() { mrac_.reset(); }

// ===========================================================================
// 9. FuzzyPID
//
// 25-rule Mamdani FuzzyPD + integral on body displacement error.
// Scaling: e_scale = 0.03 m (universe [-1,1] = +/-3 cm body displacement)
//          de_scale = 0.5 m/s (universe [-1,1] = +/-0.5 m/s body velocity)
//          u_scale  = 2000 N
// ===========================================================================

FuzzyPIDSuspCtrl::FuzzyPIDSuspCtrl(const PlantParams& p)
    : fpid_([&]() {
        ctrl::FuzzyPIDParams fp;
        fp.pd.e_scale  = 0.03;   // 3 cm -> normalized
        fp.pd.de_scale = 0.50;   // 0.5 m/s -> normalized
        fp.pd.u_scale  = p.F_max;
        fp.pd.uMin     = -p.F_max;
        fp.pd.uMax     =  p.F_max;
        fp.Ki          = 50.0;   // N/(m.s) integral gain
        fp.Kb          = 1.0;
        fp.uMin        = -p.F_max;
        fp.uMax        =  p.F_max;
        return fp;
      }(), p.Ts)
{}

double FuzzyPIDSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    // FuzzyPID computes from error: e = 0 - z_s = -z_s
    return fpid_.compute(-state(0));
}

void FuzzyPIDSuspCtrl::reset() { fpid_.reset(); }

// ===========================================================================
// 10. TubeMPC
//
// Robust tube MPC on the 2-state body model.
// Tube sized to bound wheel-dynamics coupling disturbance.
//
// LQR-designed tube feedback K (negated per toolbox convention).
// wMax element-wise: [2 mm position uncertainty, 40 mm/s velocity per step].
// ===========================================================================

ctrl::StateSpace TubeMPCSuspCtrl::buildBodySS(const PlantParams& p)
{
    return makeBodySS(p);
}

TubeMPCSuspCtrl::TubeMPCSuspCtrl(const PlantParams& p)
    : tmpc_([&]() {
        ctrl::StateSpace sys = buildBodySS(p);

        // Design LQR for tube feedback gain
        ctrl::LQRParams lp;
        lp.Q = MatrixXd::Zero(2,2);
        lp.Q(0,0) = 1.0 / (0.025 * 0.025);  // z_s
        lp.Q(1,1) = 1.0 / (0.5   * 0.5);    // dz_s
        lp.R = MatrixXd::Constant(1,1, 1.0/(p.F_max * p.F_max));

        ctrl::DiscreteLQR lqr_design(sys, lp);
        MatrixXd K_lqr = lqr_design.gainMatrix();   // K: 1 x 2
        MatrixXd K_tube = -K_lqr;            // negate: u_tube = K*(x-x_nom)

        ctrl::TubeMPCParams tp;
        tp.Np   = 10;
        tp.Nu   = 3;
        // Q is p*p (output weight), R is m*m (input weight) - NOT LQR state weights
        tp.Q    = MatrixXd::Constant(1,1, 1.0 / (0.025 * 0.025)); // z_s output tracking
        tp.R    = lp.R; // 1*1 input effort weight
        tp.K    = K_tube;
        // Disturbance bound: wheel coupling to 2-state body model per step
        tp.wMax = VectorXd(2); tp.wMax << 0.002, 0.040;
        tp.uMin = VectorXd(1); tp.uMin(0) = -p.F_max;
        tp.uMax = VectorXd(1); tp.uMax(0) =  p.F_max;
        tp.Ts   = p.Ts;

        return ctrl::TubeMPC(sys, tp);
      }())
{}

double TubeMPCSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    VectorXd x2(2); x2 << state(0), state(1);
    VectorXd r(1);  r(0) = 0.0;
    VectorXd u = tmpc_.computeRef(x2, r);
    return std::clamp(u(0), -F_ACT_MAX, F_ACT_MAX);
}

void TubeMPCSuspCtrl::reset() { tmpc_.reset(); }

// ===========================================================================
// 11. ILC
//
// Two-phase iterative learning on body displacement error (e = 0 - z_s).
// Phase 1 (steps 0..N_TRIAL-1): PID feedback only; error is recorded.
// At k == N_TRIAL: updateFeedforward() computes the learned correction.
// Phase 2 (remaining steps): PID + ILC feedforward added to F_act.
//
// N_TRIAL = 1000 steps = 5 s at Ts=0.005 s.
// Lp=0.6, Q_filter=0.95: moderate learning rate, stable across 5 scenarios.
// ===========================================================================

ILCSuspCtrl::ILCSuspCtrl(const PlantParams& p)
    : ilc_([&]() {
        ctrl::ILC::Params ip;
        ip.N        = N_TRIAL;
        ip.mode     = ctrl::ILC::Mode::PType;
        ip.Lp       = 0.6;
        ip.Ts       = p.Ts;
        ip.Q_filter = 0.95;
        ip.uMin     = -F_ACT_MAX;
        ip.uMax     =  F_ACT_MAX;
        return ctrl::ILC(ip);
      }())
    , pid_(ctrl::PIDParams{
        .Kp   = 2000.0,
        .Ki   = 30.0,
        .Kd   = 500.0,
        .N    = 10.0,
        .uMin = -F_ACT_MAX,
        .uMax =  F_ACT_MAX,
        .Kb   = 1.0
      }, p.Ts)
{}

double ILCSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    double e     = -state(0);          // error = 0 - z_s
    double u_pid = pid_.compute(e);

    if (!phase2_) {
        if (k_ < N_TRIAL)
            ilc_.recordError(k_, e);
        if (++k_ == N_TRIAL) {
            ilc_.updateFeedforward();
            phase2_ = true;
            k_      = 0;
        }
        return std::clamp(u_pid, -F_ACT_MAX, F_ACT_MAX);
    } else {
        int    kc   = std::min(k_++, N_TRIAL - 1);
        double u_ff = ilc_.feedforward(kc);
        return std::clamp(u_pid + u_ff, -F_ACT_MAX, F_ACT_MAX);
    }
}

void ILCSuspCtrl::reset()
{
    ilc_.reset();
    pid_.reset();
    k_      = 0;
    phase2_ = false;
}

// ===========================================================================
// 12. CBFSafety
//
// CBFSafetyFilter with barrier on body velocity dz_s (state index 1).
//   h(v)    = v_max - v        (upper velocity limit)
//   dh/dx   = -1
//   f0(v)   = 0                (drift ignored; wheel coupling treated as disturbance)
//   g(v)    = 1/m_s            (F_act directly drives body acceleration)
//
// When |dz_s| > v_max the CBF limits F_act to decelerate the body.
// The PID nominal provides baseline force; CBF modifies only when barrier
// would be violated. fit is approximate (ignores spring/damper coupling).
// ===========================================================================

CBFSuspCtrl::CBFSuspCtrl(const PlantParams& p)
    : cbf_([&]() {
        auto nominal = std::make_shared<ctrl::DiscretePID>(
            ctrl::PIDParams{
                .Kp   = 2000.0,
                .Ki   = 30.0,
                .Kd   = 500.0,
                .N    = 10.0,
                .uMin = -F_ACT_MAX,
                .uMax =  F_ACT_MAX,
                .Kb   = 1.0
            }, p.Ts);

        const double v_max   = 0.5;           // [m/s] maximum body velocity
        const double g_val   = 1.0 / p.m_s;  // F_act -> d(dz_s)/dt

        ctrl::CBFSafetyFilter::Params cp;
        cp.alpha = 5.0;
        cp.uMin  = -F_ACT_MAX;
        cp.uMax  =  F_ACT_MAX;

        return ctrl::CBFSafetyFilter(
            nominal,
            [v_max](double v) { return v_max - v; },   // h
            [](double)        { return -1.0; },          // dh/dx
            [](double)        { return 0.0; },            // f0 (drift)
            [g_val](double)   { return g_val; },          // g (input gain)
            cp,
            p.Ts);
      }())
{}

double CBFSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    cbf_.setState(state(1));        // CBF state = dz_s (body velocity)
    return cbf_.compute(-state(0)); // nominal error = 0 - z_s
}

void CBFSuspCtrl::reset() { cbf_.reset(); }

// ===========================================================================
// 13. L1Adaptive
//
// L1 adaptive on body displacement z_s.
// Reference model poles matched to MRACSuspCtrl: a_m = exp(-4*Ts).
// Gamma large (fast adaptation); LP filter omega_c = 2.0 rad/s provides robustness.
// sigma_max = 5000 bounds the estimated disturbance (force-scale).
// ===========================================================================

L1AdaptiveSuspCtrl::L1AdaptiveSuspCtrl(const PlantParams& p)
    : l1_([&]() {
        const double a_m = std::exp(-4.0 * p.Ts);
        ctrl::L1AdaptiveController::Params lp;
        lp.a_m       = a_m;
        lp.b_m       = 1.0 - a_m;
        lp.k_g       = 1.0;
        lp.Gamma     = 200.0;
        lp.omega_c   = 2.0;
        lp.sigma_max = 5000.0;
        lp.Q_lyap    = 1.0;
        lp.uMin      = -F_ACT_MAX;
        lp.uMax      =  F_ACT_MAX;
        return ctrl::L1AdaptiveController(lp, p.Ts);
      }())
{}

double L1AdaptiveSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    l1_.setReference(0.0);
    return std::clamp(l1_.compute(state(0)), -F_ACT_MAX, F_ACT_MAX);
}

void L1AdaptiveSuspCtrl::reset() { l1_.reset(); }

// ===========================================================================
// 14. ScenarioMPC
//
// Scenario-based stochastic MPC on 2-state body model.
// Mirrors TubeMPCSuspCtrl: same SS, Np=10, Nu=3.
// Sigma_w tuned to wheel-coupling disturbance bound (2 mm / 40 mm/s per step).
// ===========================================================================

ctrl::StateSpace ScenarioMPCSuspCtrl::buildBodySS(const PlantParams& p)
{
    return makeBodySS(p);
}

ScenarioMPCSuspCtrl::ScenarioMPCSuspCtrl(const PlantParams& p)
    : smpc_([&]() {
        ctrl::StateSpace sys = buildBodySS(p);

        ctrl::ScenarioMPCParams sp;
        sp.Np        = 10;
        sp.Nu        = 3;
        sp.Q         = Eigen::MatrixXd::Constant(1, 1, 1.0 / (0.025 * 0.025));
        sp.R         = Eigen::MatrixXd::Constant(1, 1, 1.0 / (p.F_max * p.F_max));
        sp.Sigma_w   = Eigen::MatrixXd::Zero(2, 2);
        sp.Sigma_w(0,0) = 4e-6;    // position variance (2 mm std)
        sp.Sigma_w(1,1) = 1.6e-3;  // velocity variance (40 mm/s std)
        sp.N_samples = 30;
        sp.uMin      = Eigen::VectorXd::Constant(1, -p.F_max);
        sp.uMax      = Eigen::VectorXd::Constant(1,  p.F_max);
        sp.Ts        = p.Ts;
        return ctrl::ScenarioMPC(sys, sp);
      }())
{}

double ScenarioMPCSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    Eigen::VectorXd x2(2); x2 << state(0), state(1);
    Eigen::VectorXd r(1);  r(0) = 0.0;
    Eigen::VectorXd u = smpc_.computeRef(x2, r);
    return std::clamp(u(0), -F_ACT_MAX, F_ACT_MAX);
}

void ScenarioMPCSuspCtrl::reset() { smpc_.reset(); }

// ===========================================================================
// 15. DynaCtrl
//
// Dyna MBRL wrapping PID on body displacement. Same PID gains as PIDSuspCtrl.
// After n_collect=50 steps SINDy fits error dynamics and supplements PID.
// ===========================================================================

DynaSuspCtrl::DynaSuspCtrl(const PlantParams& p)
    : dyna_(ctrl::DynaController::Params{
        .n_collect     = 50,
        .n_refit_every = 25,
        .Ts            = p.Ts
      },
      std::make_shared<ctrl::DiscretePID>(
          ctrl::PIDParams{
              .Kp   = 2000.0,
              .Ki   = 30.0,
              .Kd   = 500.0,
              .N    = 10.0,
              .uMin = -F_ACT_MAX,
              .uMax =  F_ACT_MAX,
              .Kb   = 1.0
          }, p.Ts))
{}

double DynaSuspCtrl::compute(const Vector4d& state, double /*z_r*/)
{
    return std::clamp(dyna_.compute(-state(0)), -F_ACT_MAX, F_ACT_MAX);
}

void DynaSuspCtrl::reset() { dyna_.reset(); }

} // namespace susp
