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

} // namespace susp
