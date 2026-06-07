#include "controllers.h"
#include <cmath>
#include <algorithm>

using namespace Eigen;

namespace tug {

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

// Body-frame error: e = R^T(psi) * (eta_ref - eta)
static Vector3d bodyError(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    double psi = state(2);
    double cp = std::cos(psi), sp = std::sin(psi);
    Vector3d e_world = ref - state.head<3>();

    // Rotate position error into body frame; heading error is already scalar
    Vector3d e;
    e(0) =  cp * e_world(0) + sp * e_world(1);
    e(1) = -sp * e_world(0) + cp * e_world(1);

    // Heading error: wrap to (-pi, pi]
    e(2) = std::remainder(e_world(2), 2.0 * M_PI);

    return e;
}

// -- PID gains (Bryson's method, see doc 04_controller_choices.md) -------------
static ctrl::PIDParams pidParamsFor(int axis)
{
    ctrl::PIDParams p;
    p.N  = 10.0;
    p.Kb = 1.0;
    if (axis == 0 || axis == 1) {   // surge / sway
        p.Kp   = 3.0e5;
        p.Ki   = 1.0e4;
        p.Kd   = 8.0e5;
        p.uMin = -TAU_XY_MAX;
        p.uMax =  TAU_XY_MAX;
    } else {                         // yaw
        p.Kp   = 8.0e6;
        p.Ki   = 5.0e5;
        p.Kd   = 2.0e7;
        p.uMin = -TAU_PSI_MAX;
        p.uMax =  TAU_PSI_MAX;
    }
    return p;
}

// ===========================================================================
// Mode 1 - PID
// ===========================================================================

PIDController::PIDController(const PlantParameters& p)
    : pp_(p)
    , pids_{ ctrl::DiscretePID(pidParamsFor(0), p.dt),
             ctrl::DiscretePID(pidParamsFor(1), p.dt),
             ctrl::DiscretePID(pidParamsFor(2), p.dt) }
{}

Vector3d PIDController::compute(const Vector3d& ref,
                                 const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);
    return Vector3d(
        pids_[0].compute(e(0)),
        pids_[1].compute(e(1)),
        pids_[2].compute(e(2)));
}

void PIDController::reset()
{
    for (auto& pid : pids_) pid.reset();
}

// ===========================================================================
// Shared helpers (plant SS construction used by multiple controllers)
// ===========================================================================

// 6-state linearised SS about zero velocity (used by KF-PID, LQR, LQG, EKF-LQR).
// Continuous: M_re * nu_dot = -D_re * nu + tau,  eta_dot = nu (zero-psi approx).
// State: [x, y, psi, u, v, r].  Input: tau [N/N.m].
static ctrl::StateSpace makePlantSS(const PlantParameters& pp)
{
    int n = 6, m = 3;
    Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(n, m);
    Ac.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();
    Ac.block<3,3>(3, 3) = -(pp.M_re_inv * pp.D_re);
    Bc.block<3,3>(3, 0) = pp.M_re_inv;

    Eigen::MatrixXd Cc = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd Dc = Eigen::MatrixXd::Zero(n, m);

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, pp.dt, ctrl::C2dMethod::ZOH);
}

// 2-state per-axis model [e, nu] (used by MPC, TubeMPC).
// Input in kN for QP numerical stability (same as MPCController::buildAxisSS).
static ctrl::StateSpace makeAxisSS(int axis, const PlantParameters& pp)
{
    double m = pp.M_re(axis, axis);
    double d = pp.D_re(axis, axis);
    const double kN2N = 1.0e3;

    Eigen::MatrixXd Ac(2, 2); Ac << 0, -1, 0, -d/m;
    Eigen::MatrixXd Bc(2, 1); Bc << 0, kN2N/m;
    Eigen::MatrixXd Cc(1, 2); Cc << 1, 0;
    Eigen::MatrixXd Dc(1, 1); Dc << 0;

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, pp.dt, ctrl::C2dMethod::ZOH);
}

// ===========================================================================
// Mode 2 - KF-PID
// ===========================================================================

ctrl::StateSpace KFPIDController::buildPlantSS() const
{
    return makePlantSS(pp_);
}

KFPIDController::KFPIDController(const PlantParameters& p)
    : pp_(p)
    , pids_{ ctrl::DiscretePID(pidParamsFor(0), p.dt),
             ctrl::DiscretePID(pidParamsFor(1), p.dt),
             ctrl::DiscretePID(pidParamsFor(2), p.dt) }
    , u_prev_(Vector3d::Zero())
{
    ctrl::StateSpace sys = buildPlantSS();

    MatrixXd Q_kf = MatrixXd::Zero(6,6);
    Q_kf.diagonal() << 1e-3, 1e-3, 1e-5, 1e-2, 1e-2, 1e-4;

    MatrixXd R_kf = MatrixXd::Zero(6,6);
    R_kf.diagonal() << 1e-2, 1e-2, 1e-4, 1e-1, 1e-1, 1e-3;

    kf_ = std::make_unique<ctrl::KalmanFilter>(sys, Q_kf, R_kf);
}

Vector3d KFPIDController::compute(const Vector3d& ref,
                                   const Matrix<double,6,1>& state)
{
    // KF step: predict with previous tau_c, update with current measurement
    VectorXd y = state;                   // 6-element measurement
    VectorXd u_k(3); u_k = u_prev_;
    kf_->step(y, u_k);

    // Use filtered state for PID
    const VectorXd& x_hat = kf_->state();
    Matrix<double,6,1> state_f = x_hat;

    Vector3d e = bodyError(ref, state_f);
    Vector3d tau(
        pids_[0].compute(e(0)),
        pids_[1].compute(e(1)),
        pids_[2].compute(e(2)));

    u_prev_ = tau;
    return tau;
}

void KFPIDController::reset()
{
    kf_->reset();
    for (auto& pid : pids_) pid.reset();
    u_prev_.setZero();
}

// ===========================================================================
// Mode 3 - SMC (paper Eqs. 24-27)
// ===========================================================================

SMCController::SMCController(const PlantParameters& p)
    : pp_(p)
    // Gains rescaled for physical feasibility on this barge-tug plant.
    // Lambda(i) sets the sliding surface slope; M_re(i)*Lambda(i)*|e_dot| must be < TAU_max.
    // For xy: TAU_XY_MAX/M_re(0,0) = 2e6/1.35e8 ~ 1.5e-2 rad/s -> Lambda=1e-2.
    // For yaw: TAU_PSI_MAX/M_re(2,2) = 5e7/4.67e13 ~ 1.07e-6 -> Lambda=1e-6.
    //
    // K_sw sizing accounts for the allocator geometry factor:
    // With 4 symmetric tugs, pure sway command tau_y maps to T_unc = tau_y/(2*sqrt(3)).
    // Two tugs active (T3,T4 clamped to 0): achieved_y = 0.5 * tau_y.
    // Max sway disturbance (S2: wind+current+wave) ~ 470 kN.
    // Need achieved_y = K_sw * 0.5 > 470 kN -> K_sw > 940 kN.
    // Set K_sw = TAU_XY_MAX = 2e6 N to guarantee full saturation under all disturbances.
    , Lambda_(1e-2, 1e-2, 1e-6)
    , Ki_s_  (0.0,  0.0,  0.0)    // disable integral - avoid windup during saturation
    , K_sw_  (2e6,  2e6,  5e6)    // xy at TAU_XY_MAX for disturbance rejection; yaw below TAU_PSI_MAX
    , Phi_   (0.5,  0.5,  0.05)
    , e_prev_(Vector3d::Zero())
    , integral_(Vector3d::Zero())
{}

double SMCController::sat(double s, double phi)
{
    if (phi < 1e-12) return (s >= 0.0) ? 1.0 : -1.0;
    double x = s / phi;
    return std::clamp(x, -1.0, 1.0);
}

Vector3d SMCController::compute(const Vector3d& ref,
                                 const Matrix<double,6,1>& state)
{
    const double dt = pp_.dt;
    Vector3d nu = state.tail<3>();
    Vector3d e  = bodyError(ref, state);

    // Error derivative (backward difference)
    Vector3d e_dot = (e - e_prev_) / dt;

    // Sliding surface: s = e_dot + Lambda*e + Ki_s * integral (integral updated below)
    Vector3d s = e_dot + Lambda_.cwiseProduct(e) + Ki_s_.cwiseProduct(integral_);

    // Equivalent control - model cancellation (paper Eq. 26).
    // Only the Lambda*e_dot term is included; the D_re*nu (damping cancellation) term
    // is omitted because D_re(2,2)=3.12e12 N.m.s/rad makes it 62x larger than
    // TAU_PSI_MAX for any non-trivial yaw rate, preventing physical implementation.
    // The switching control alone provides damping through the sliding surface.
    Vector3d tau_eq_raw = -pp_.M_re * Lambda_.cwiseProduct(e_dot);
    Vector3d tau_eq(
        std::clamp(tau_eq_raw(0), -TAU_XY_MAX,  TAU_XY_MAX),
        std::clamp(tau_eq_raw(1), -TAU_XY_MAX,  TAU_XY_MAX),
        std::clamp(tau_eq_raw(2), -TAU_PSI_MAX, TAU_PSI_MAX));

    // Switching control: tau_sw = +K_sw * sat(s/Phi).
    // Sign: s_dot = e_ddot + Lambda*e_dot = -tau/m + ... so s_dot < 0 when tau > 0.
    // For s < 0 (need s_dot > 0): require tau < 0 -> tau_sw = -K_sw*sat(s) gives +tau when s<0.
    // BUT the original form tau = -K_sw*sat(s) is WRONG for this sign convention because
    // it gives positive tau when s<0, making s_dot more negative.
    // Corrected: tau_sw = +K_sw * sat(s/Phi) so that negative tau is applied for s<0.
    // Equivalently: when s < 0, sat(s/Phi) < 0, so tau_sw = +K_sw * (negative) < 0. Correct!
    Vector3d tau_sw;
    for (int j = 0; j < 3; ++j)
        tau_sw(j) = K_sw_(j) * sat(s(j), Phi_(j));

    Vector3d tau = tau_eq + tau_sw;

    // Anti-windup: only accumulate integral when the total command is not saturated.
    // This prevents the sliding-surface integral from growing during actuator saturation,
    // which would otherwise prevent the surface from converging once saturation ends.
    bool sat_x   = std::abs(tau(0)) >= TAU_XY_MAX  - 1.0;
    bool sat_y   = std::abs(tau(1)) >= TAU_XY_MAX  - 1.0;
    bool sat_psi = std::abs(tau(2)) >= TAU_PSI_MAX - 1.0;
    if (!sat_x)   integral_(0) += 0.5 * (e(0) + e_prev_(0)) * dt;
    if (!sat_y)   integral_(1) += 0.5 * (e(1) + e_prev_(1)) * dt;
    if (!sat_psi) integral_(2) += 0.5 * (e(2) + e_prev_(2)) * dt;

    e_prev_ = e;
    return tau;
}

void SMCController::reset()
{
    e_prev_.setZero();
    integral_.setZero();
}

// ===========================================================================
// Mode 4 - MPC (decoupled per-axis, linearised about zero velocity)
// ===========================================================================

ctrl::StateSpace MPCController::buildAxisSS(int axis) const
{
    return makeAxisSS(axis, pp_);
}

MPCController::MPCController(const PlantParameters& p) : pp_(p)
{
    ctrl::MPCParams mp;
    // Horizon must cover ~3 plant time constants per axis.
    // For xy: tau_xy = M_re/D_re ~ 1.35e8/6e5 = 225s = 450 steps - impractical.
    // Use a moderate horizon (60 steps = 30s) to capture short-term dynamics.
    // Yaw time constant: 4.67e13/3.12e12 = 15s = 30 steps - Np=60 is sufficient.
    mp.Np        = 60;
    mp.Nc        = 5;
    mp.qpMaxIter = 500;  // well-conditioned xy/yaw Hessians; FISTA converges well under 100

    // Input in kN (see buildAxisSS). Bd_sway ~ 5.75e-6 m/s per kN per step.
    // Bd_yaw ~ 1.07e-11 rad/s per kN-torque per step.
    // For yaw axis: rho_y=1e5 is extremely aggressive given Bd_psi is tiny.
    // The QP saturates every step, causing large unintended yaw.
    // Use small rho_y(2) and large rho_u(2) to make yaw MPC conservative.
    // rho_y/rho_u should be comparable to 1/Bd^2 to keep the gain reasonable.
    // Bd_psi ~ 1.07e-8 m/kN (kN scale) -> 1/Bd^2 ~ 8.7e15; rho_y/rho_u = 1e5/1.0.
    const double kN = 1.0e3;
    const double rho_y[3] = {1e3, 1e3, 1e3};
    const double rho_u[3] = {1e-3, 1e-3, 1.0};
    const double u_lim_kN[3] = {TAU_XY_MAX/kN, TAU_XY_MAX/kN, TAU_PSI_MAX/kN};

    for (int axis = 0; axis < 3; ++axis) {
        mp.rho_y = rho_y[axis];
        mp.rho_u = rho_u[axis];
        mp.uMin  = -u_lim_kN[axis];
        mp.uMax  =  u_lim_kN[axis];
        mp.duMin = -u_lim_kN[axis];
        mp.duMax =  u_lim_kN[axis];
        mpcs_[axis] = std::make_unique<ctrl::DiscreteMPC>(buildAxisSS(axis), mp);
    }
}

Vector3d MPCController::compute(const Vector3d& ref,
                                 const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);
    Vector3d tau;

    for (int axis = 0; axis < 3; ++axis) {
        // State: [error (m), velocity (m/s or rad/s)]
        VectorXd x_ax(2);
        x_ax(0) = e(axis);
        x_ax(1) = state(3 + axis);

        VectorXd r_ax(1); r_ax(0) = 0.0;

        mpcs_[axis]->setState(x_ax);
        VectorXd u_kN = mpcs_[axis]->computeRef(x_ax, r_ax);
        tau(axis) = u_kN(0) * 1.0e3;   // kN -> N
    }

    return tau;
}

void MPCController::reset()
{
    for (auto& m : mpcs_) m->reset();
}

void MPCController::notifyApplied(const Vector3d& tau_applied)
{
    for (int axis = 0; axis < 3; ++axis) {
        VectorXd u_ax(1); u_ax(0) = tau_applied(axis) * 1.0e-3;  // N -> kN
        mpcs_[axis]->setLastApplied(u_ax);
    }
}

// ===========================================================================
// Mode 5 - ESC (model-free, per-axis IAE gradient descent)
// ===========================================================================

ESCController::ESCController(const PlantParameters& p)
    : pp_(p)
    , escs_{
        ctrl::ExtremumSeeker(
            {.perturbAmp = 5e3,  .perturbFreq = 0.016,
             .lpfCutoff = 0.02,  .hpfCutoff = 0.05,
             .integGain = 1.0,   .seekMinimum = true}, p.dt),
        ctrl::ExtremumSeeker(
            {.perturbAmp = 5e3,  .perturbFreq = 0.018,
             .lpfCutoff = 0.02,  .hpfCutoff = 0.05,
             .integGain = 1.0,   .seekMinimum = true}, p.dt),
        ctrl::ExtremumSeeker(
            {.perturbAmp = 1e5,  .perturbFreq = 0.020,
             .lpfCutoff = 0.02,  .hpfCutoff = 0.05,
             .integGain = 1.0,   .seekMinimum = true}, p.dt)
    }
    , e_prev_(Vector3d::Zero())
{}

Vector3d ESCController::compute(const Vector3d& ref,
                                 const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);

    // Cost signal for each axis: |e| (instantaneous absolute error)
    Vector3d tau;
    for (int axis = 0; axis < 3; ++axis)
        tau(axis) = escs_[axis].compute(std::abs(e(axis)));

    e_prev_ = e;
    return tau;
}

void ESCController::reset()
{
    for (auto& esc : escs_) esc.reset();
    e_prev_.setZero();
}

// ===========================================================================
// Mode 6 - FuzzyPID  (three independent FuzzyPID loops, one per axis)
// ===========================================================================

// Scaling rationale:
//   Surge/sway error up to ~50 m is "large"  -> e_scale = 25 m
//   Error rate up to ~1 m/s is "large"        -> de_scale = 0.5 m/s
//   Output universe [-1,1] maps to +/-TAU_XY_MAX -> u_scale = 2e6 N
//   Yaw: error up to ~0.5 rad, rate ~0.02 rad/s, output +/-5e7 N.m

static ctrl::FuzzyPIDParams fuzzyPIDParamsFor(int axis, double dt)
{
    ctrl::FuzzyPIDParams p;
    p.Kb = 1.0;
    if (axis == 0 || axis == 1) {   // surge / sway
        p.pd.e_scale  = 25.0;
        p.pd.de_scale =  0.5;
        p.pd.u_scale  = TAU_XY_MAX;
        p.pd.uMin     = -TAU_XY_MAX;
        p.pd.uMax     =  TAU_XY_MAX;
        p.Ki          = 1.0e4;
        p.uMin        = -TAU_XY_MAX;
        p.uMax        =  TAU_XY_MAX;
    } else {                         // yaw
        p.pd.e_scale  = 0.50;
        p.pd.de_scale = 0.02;
        p.pd.u_scale  = TAU_PSI_MAX;
        p.pd.uMin     = -TAU_PSI_MAX;
        p.pd.uMax     =  TAU_PSI_MAX;
        p.Ki          = 5.0e5;
        p.uMin        = -TAU_PSI_MAX;
        p.uMax        =  TAU_PSI_MAX;
    }
    (void)dt;
    return p;
}

FuzzyPIDController::FuzzyPIDController(const PlantParameters& p)
    : pp_(p)
    , fuzzy_pids_{
        ctrl::FuzzyPID(fuzzyPIDParamsFor(0, p.dt), p.dt),
        ctrl::FuzzyPID(fuzzyPIDParamsFor(1, p.dt), p.dt),
        ctrl::FuzzyPID(fuzzyPIDParamsFor(2, p.dt), p.dt) }
{}

Vector3d FuzzyPIDController::compute(const Vector3d& ref,
                                      const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);
    return Vector3d(
        fuzzy_pids_[0].compute(e(0)),
        fuzzy_pids_[1].compute(e(1)),
        fuzzy_pids_[2].compute(e(2)));
}

void FuzzyPIDController::reset()
{
    for (auto& f : fuzzy_pids_) f.reset();
}

// ===========================================================================
// FuzzySupervised_MPC - MPC with fuzzy re-linearisation supervisor
// ===========================================================================

ctrl::StateSpace FuzzySupervised_MPC::buildAxisSS(int axis,
                                                   const Vector3d& nu) const
{
    double m = pp_.M_re(axis, axis);
    double d = pp_.D_re(axis, axis);

    // Velocity-dependent Coriolis correction
    double d_extra = 0.0;
    if (axis == 0) d_extra = pp_.M_b(1,1) * std::abs(nu(1));
    if (axis == 1) d_extra = pp_.M_b(0,0) * std::abs(nu(0));

    double d_eff = d + d_extra;
    const double kN2N = 1.0e3;

    // A(0,1) = -1 because e_dot = -nu (positive motion reduces error)
    MatrixXd Ac(2,2); Ac << 0, -1, 0, -d_eff/m;
    MatrixXd Bc(2,1); Bc << 0, kN2N/m;
    MatrixXd Cc(1,2); Cc << 1, 0;
    MatrixXd Dc(1,1); Dc << 0;

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, pp_.dt, ctrl::C2dMethod::ZOH);
}

void FuzzySupervised_MPC::relinearize(int axis, const Vector3d& nu)
{
    ctrl::StateSpace ss = buildAxisSS(axis, nu);
    mpcs_[axis]->setPlant(ss);
    ++relinearize_count_;
}

FuzzySupervised_MPC::FuzzySupervised_MPC(const PlantParameters& p)
    : pp_(p)
    , supervisors_{
        ctrl::FuzzySupervisor({.e_threshold    = 10.0,
                               .trend_threshold = 0.2,
                               .signal_threshold = 0.5,
                               .cooldown_steps  = 40}, p.dt),
        ctrl::FuzzySupervisor({.e_threshold    = 10.0,
                               .trend_threshold = 0.2,
                               .signal_threshold = 0.5,
                               .cooldown_steps  = 40}, p.dt),
        ctrl::FuzzySupervisor({.e_threshold    = 0.1,
                               .trend_threshold = 0.005,
                               .signal_threshold = 0.5,
                               .cooldown_steps  = 40}, p.dt) }
{
    // Build MPC instances at zero-velocity linearisation (same as MPCController)
    ctrl::MPCParams mp;
    mp.Np        = 60;
    mp.Nc        = 5;
    mp.qpMaxIter = 500;  // consistent with MPCController
    const double kN = 1.0e3;
    const double rho_y[3] = {1e3, 1e3, 1e3};
    const double rho_u[3] = {1e-3, 1e-3, 1.0};
    const double u_lim[3] = {TAU_XY_MAX/kN, TAU_XY_MAX/kN, TAU_PSI_MAX/kN};

    Vector3d nu_zero = Vector3d::Zero();
    for (int axis = 0; axis < 3; ++axis) {
        mp.rho_y = rho_y[axis];
        mp.rho_u = rho_u[axis];
        mp.uMin  = -u_lim[axis];
        mp.uMax  =  u_lim[axis];
        mp.duMin = -u_lim[axis];
        mp.duMax =  u_lim[axis];
        mpcs_[axis] = std::make_unique<ctrl::DiscreteMPC>(
            buildAxisSS(axis, nu_zero), mp);
    }

    decisions_.fill(ctrl::SupervisorDecision{0.0, false, 0.0, 0.0});
}

Vector3d FuzzySupervised_MPC::compute(const Vector3d& ref,
                                       const Matrix<double,6,1>& state)
{
    Vector3d e  = bodyError(ref, state);
    Vector3d nu = state.tail<3>();
    Vector3d tau;

    for (int axis = 0; axis < 3; ++axis) {
        // Supervisor check - uses absolute error for this axis
        decisions_[axis] = supervisors_[axis].update(std::abs(e(axis)));

        if (decisions_[axis].relinearize)
            relinearize(axis, nu);

        // MPC compute (same as MPCController)
        VectorXd x_ax(2);
        x_ax(0) = e(axis);
        x_ax(1) = nu(axis);

        VectorXd r_ax(1); r_ax(0) = 0.0;
        mpcs_[axis]->setState(x_ax);
        VectorXd u_kN = mpcs_[axis]->computeRef(x_ax, r_ax);
        tau(axis) = u_kN(0) * 1.0e3;   // kN -> N
    }

    return tau;
}

void FuzzySupervised_MPC::reset()
{
    for (auto& m : mpcs_) m->reset();
    for (auto& s : supervisors_) s.reset();
    decisions_.fill(ctrl::SupervisorDecision{0.0, false, 0.0, 0.0});
    relinearize_count_ = 0;
}

void FuzzySupervised_MPC::notifyApplied(const Vector3d& tau_applied)
{
    for (int axis = 0; axis < 3; ++axis) {
        VectorXd u_ax(1); u_ax(0) = tau_applied(axis) * 1.0e-3;  // N -> kN
        mpcs_[axis]->setLastApplied(u_ax);
    }
}

// ===========================================================================
// Mode 8 - ADRC (3-axis)
// b0 = 1/M_re(i,i) [N^-^1.m.s^-^2] - effective inverse inertia per axis.
// omega_o = 0.5 rad/s -> omega_o*Ts = 0.25 < 0.5 (backward Euler stable).
// omega_c = 0.1 rad/s -> closed-loop bandwidth approx = 0.1 rad/s.
// ===========================================================================

static ctrl::ADRCParams adrcParamsFor(int axis, const PlantParameters& p)
{
    ctrl::ADRCParams ap;
    ap.omega_o = 0.5;    // rad/s; omega_o * Ts = 0.25 < 0.5 (check)
    ap.omega_c = 0.1;
    if (axis < 2) {      // surge / sway
        ap.b0   = 1.0 / p.M_re(axis, axis);
        ap.uMin = -TAU_XY_MAX;
        ap.uMax =  TAU_XY_MAX;
    } else {             // yaw
        ap.b0   = 1.0 / p.M_re(2, 2);
        ap.uMin = -TAU_PSI_MAX;
        ap.uMax =  TAU_PSI_MAX;
    }
    return ap;
}

ADRCTugCtrl::ADRCTugCtrl(const PlantParameters& p)
    : pp_(p)
    , adrcs_{ ctrl::DiscreteADRC(adrcParamsFor(0, p), p.dt),
               ctrl::DiscreteADRC(adrcParamsFor(1, p), p.dt),
               ctrl::DiscreteADRC(adrcParamsFor(2, p), p.dt) }
{}

Vector3d ADRCTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);
    return Vector3d(adrcs_[0].compute(e(0)),
                    adrcs_[1].compute(e(1)),
                    adrcs_[2].compute(e(2)));
}

void ADRCTugCtrl::reset()
{
    for (auto& a : adrcs_) a.reset();
}

// ===========================================================================
// Mode 9 - RepetitiveController (3-axis)
// Period from JONSWAP peak period Tp (from plant_params.json).
// Krc=0.4 (conservative learning gain), Q=0.97 (robustness).
// Inner PID identical to Mode 1 (same gains, same saturation limits).
// ===========================================================================

RepetitiveTugCtrl::RepetitiveTugCtrl(const PlantParameters& p)
    : pp_(p)
    , rcs_{ [&]() {
        const int steps = std::max(1, static_cast<int>(std::round(p.Tp / p.dt)));
        return ctrl::RepetitiveController(
            std::make_shared<ctrl::DiscretePID>(pidParamsFor(0), p.dt),
            ctrl::RepetitiveParams{.periodSteps = steps, .Krc = 0.4, .Q = 0.97,
                                   .uMin = -TAU_XY_MAX, .uMax = TAU_XY_MAX},
            p.dt);
    }(),
    [&]() {
        const int steps = std::max(1, static_cast<int>(std::round(p.Tp / p.dt)));
        return ctrl::RepetitiveController(
            std::make_shared<ctrl::DiscretePID>(pidParamsFor(1), p.dt),
            ctrl::RepetitiveParams{.periodSteps = steps, .Krc = 0.4, .Q = 0.97,
                                   .uMin = -TAU_XY_MAX, .uMax = TAU_XY_MAX},
            p.dt);
    }(),
    [&]() {
        const int steps = std::max(1, static_cast<int>(std::round(p.Tp / p.dt)));
        return ctrl::RepetitiveController(
            std::make_shared<ctrl::DiscretePID>(pidParamsFor(2), p.dt),
            ctrl::RepetitiveParams{.periodSteps = steps, .Krc = 0.4, .Q = 0.97,
                                   .uMin = -TAU_PSI_MAX, .uMax = TAU_PSI_MAX},
            p.dt);
    }() }
{}

Vector3d RepetitiveTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);
    return Vector3d(rcs_[0].compute(e(0)),
                    rcs_[1].compute(e(1)),
                    rcs_[2].compute(e(2)));
}

void RepetitiveTugCtrl::reset()
{
    for (auto& rc : rcs_) rc.reset();
}

// ===========================================================================
// Shared Bryson LQR parameters for 6-state tug model
// ===========================================================================

static ctrl::LQRParams tugLQRParams()
{
    Eigen::VectorXd xmax(6), umax(3);
    xmax << 10.0, 10.0, 0.1, 1.0, 1.0, 0.05;
    umax << TAU_XY_MAX, TAU_XY_MAX, TAU_PSI_MAX;
    return ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
}

// ===========================================================================
// Mode 10 - LQR (6-state MIMO, stateless at runtime)
// ===========================================================================

LQRTugCtrl::LQRTugCtrl(const PlantParameters& p)
    : pp_(p)
    , lqr_(makePlantSS(p), tugLQRParams())
{}

Vector3d LQRTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    Eigen::VectorXd x_ref(6);
    x_ref << ref(0), ref(1), ref(2), 0.0, 0.0, 0.0;
    Eigen::VectorXd u = lqr_.compute(state.cast<double>(), x_ref);
    return u.head<3>();
}

// ===========================================================================
// Mode 11 - LQG (6-state Kalman filter + LQR)
// ===========================================================================

LQGTugCtrl::LQGTugCtrl(const PlantParameters& p)
    : pp_(p)
    , lqg_([&]() {
        ctrl::StateSpace sys = makePlantSS(p);

        // Process noise: position 1e-4 m^2, heading 1e-6 rad^2, velocity 1e-2 (m/s)^2
        Eigen::MatrixXd Q_n = Eigen::MatrixXd::Zero(6, 6);
        Q_n.diagonal() << 1e-4, 1e-4, 1e-6, 1e-2, 1e-2, 1e-4;

        // Measurement noise: GPS 0.01 m^2, heading 1e-4 rad^2, velocity 0.1 (m/s)^2
        Eigen::MatrixXd R_n = Eigen::MatrixXd::Zero(6, 6);
        R_n.diagonal() << 1e-2, 1e-2, 1e-4, 1e-1, 1e-1, 1e-3;

        return ctrl::DiscreteLQG(sys, tugLQRParams(), Q_n, R_n);
    }())
    , u_prev_(Eigen::VectorXd::Zero(3))
{}

Vector3d LQGTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    Eigen::VectorXd x_ref(6);
    x_ref << ref(0), ref(1), ref(2), 0.0, 0.0, 0.0;

    Eigen::VectorXd y(6);
    y = state.cast<double>();

    Eigen::VectorXd u = lqg_.step(y, u_prev_, x_ref);
    u_prev_ = u;
    return u.head<3>();
}

void LQGTugCtrl::reset()
{
    lqg_.reset();
    u_prev_.setZero();
}

// ===========================================================================
// Mode 12 - TubeMPC (3x per-axis robust MPC)
// ===========================================================================

// Build TubeMPC for one axis.  K_tube = -K_lqr (CLAUDE.md caveat).
static ctrl::TubeMPC makeTubeMPCForAxis(int axis, const PlantParameters& pp)
{
    ctrl::StateSpace sys = makeAxisSS(axis, pp);
    const double kN = 1.0e3;
    const double tau_max_kN = (axis < 2) ? TAU_XY_MAX / kN : TAU_PSI_MAX / kN;

    // LQR gain for stabilising tube feedback
    ctrl::LQRParams lp;
    lp.Q = Eigen::MatrixXd::Identity(2, 2);
    lp.Q(0, 0) = 1e-3;   // weight position (m^2)
    lp.Q(1, 1) = 1.0;    // weight velocity (m/s or rad/s)^2
    lp.R = Eigen::MatrixXd::Identity(1, 1) * 1.0;   // kN^2 or kN.m^2

    ctrl::DiscreteLQR lqr_axis(sys, lp);
    Eigen::MatrixXd K_tube = -lqr_axis.gainMatrix();   // negate for TubeMPC convention

    // Disturbance bound: position ~ 0.001 m, velocity from wave force / mass
    double wv = (axis < 2)
        ? 50.0e3 / pp.M_re(axis, axis)   // 50 kN wave force -> velocity step
        : 50.0e3 / pp.M_re(2, 2);        // 50 kN.m wave moment -> rate step
    wv *= pp.dt;  // m/s per step

    Eigen::VectorXd wMax(2);
    wMax << 1e-3, std::max(wv, 1e-9);

    ctrl::TubeMPCParams tp;
    tp.Np   = 60;
    tp.Nu   = 5;
    tp.Q    = Eigen::MatrixXd::Identity(1, 1) * 1.0;   // output (error)
    tp.R    = Eigen::MatrixXd::Identity(1, 1) * 1.0;   // input (kN)
    tp.K    = K_tube;
    tp.wMax = wMax;
    tp.uMin = Eigen::VectorXd::Constant(1, -tau_max_kN);
    tp.uMax = Eigen::VectorXd::Constant(1,  tau_max_kN);
    tp.Ts   = pp.dt;

    return ctrl::TubeMPC(sys, tp);
}

TubeMPCTugCtrl::TubeMPCTugCtrl(const PlantParameters& p)
    : pp_(p)
    , tmpcs_{ makeTubeMPCForAxis(0, p),
              makeTubeMPCForAxis(1, p),
              makeTubeMPCForAxis(2, p) }
{}

Vector3d TubeMPCTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);
    Vector3d tau;

    for (int axis = 0; axis < 3; ++axis) {
        Eigen::VectorXd x_ax(2);
        x_ax(0) = e(axis);
        x_ax(1) = state(3 + axis);

        Eigen::VectorXd r_ax(1); r_ax(0) = 0.0;

        Eigen::VectorXd u_kN = tmpcs_[axis].computeRef(x_ax, r_ax);
        tau(axis) = u_kN(0) * 1.0e3;   // kN -> N
    }
    return tau;
}

void TubeMPCTugCtrl::reset()
{
    for (auto& t : tmpcs_) t.reset();
}

// ===========================================================================
// Mode 13 - EKF-LQR
// ===========================================================================

// Nonlinear discrete-time dynamics: eta_next = eta + dt*J(psi)*nu,
//                                   nu_next  = nu + dt*M_re_inv*(-D_re*nu + tau)
static Eigen::VectorXd tugDynamics(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u,
                                    const PlantParameters& pp)
{
    double psi = x(2);
    double cp = std::cos(psi), sp = std::sin(psi);
    Vector3d nu = x.tail<3>();

    // J(psi)*nu (world-frame velocity)
    Vector3d eta_dot(cp*nu(0) - sp*nu(1),
                     sp*nu(0) + cp*nu(1),
                     nu(2));

    // M_re_inv * (-D_re*nu + tau)
    Vector3d nu_dot = pp.M_re_inv * (-pp.D_re * nu + u.head<3>());

    // Forward-Euler integration at the control sample time pp.dt.
    // Valid for the tug's low-frequency maneuvering dynamics (dominant time constants >> dt).
    // If dt is increased or higher-frequency modes are added, replace with RK4.
    Eigen::VectorXd x_next(6);
    x_next.head<3>() = x.head<3>() + pp.dt * eta_dot;
    x_next.tail<3>() = nu + pp.dt * nu_dot;
    return x_next;
}

EKFLQRTugCtrl::EKFLQRTugCtrl(const PlantParameters& p)
    : pp_(p)
    , ekf_([&]() {
        // EKF: 6-state, 6-measurement, numerical Jacobians
        int n = 6, meas = 6;

        auto f_fn = [&pp = p](const Eigen::VectorXd& x, const Eigen::VectorXd& u)
                    { return tugDynamics(x, u, pp); };

        auto h_fn = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u)
                    { (void)u; return x; };  // full-state measurement

        auto Fjac = [f_fn](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
            return ctrl::ExtendedKalmanFilter::numericalJacobian(
                [&](const Eigen::VectorXd& xx){ return f_fn(xx, u); }, x);
        };

        auto Hjac = [meas, n](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
            (void)x; (void)u;
            return Eigen::MatrixXd::Identity(meas, n);
        };

        Eigen::MatrixXd Q_e = Eigen::MatrixXd::Zero(6, 6);
        Q_e.diagonal() << 1e-4, 1e-4, 1e-6, 1e-2, 1e-2, 1e-4;
        Eigen::MatrixXd R_e = Eigen::MatrixXd::Zero(6, 6);
        R_e.diagonal() << 1e-2, 1e-2, 1e-4, 1e-1, 1e-1, 1e-3;

        return ctrl::ExtendedKalmanFilter(n, meas, f_fn, h_fn, Fjac, Hjac,
                                          Q_e, R_e, p.dt);
    }())
    , lqr_(makePlantSS(p), tugLQRParams())
    , u_prev_(Eigen::VectorXd::Zero(3))
{}

Vector3d EKFLQRTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    // EKF step: predict with prev tau, update with current measurement
    Eigen::VectorXd y(6); y = state.cast<double>();
    ekf_.step(y, u_prev_);

    // LQR on EKF estimate
    Eigen::VectorXd x_ref(6);
    x_ref << ref(0), ref(1), ref(2), 0.0, 0.0, 0.0;
    Eigen::VectorXd u = lqr_.compute(ekf_.state(), x_ref);

    Vector3d tau = u.head<3>();
    tau = saturateTau(tau);
    u_prev_ = tau.cast<double>();
    return tau;
}

void EKFLQRTugCtrl::reset()
{
    ekf_.reset();
    u_prev_.setZero();
}

// ===========================================================================
// Mode 14 - MRAC (3-axis adaptive, conservative)
// ===========================================================================

static ctrl::MRACParams mracParamsForAxis(int axis)
{
    ctrl::MRACParams mp;
    if (axis < 2) {   // surge / sway
        mp.a_m       = 0.97;      // slow reference model pole (dt=0.5s, tau_ref~16.5s)
        mp.b_m       = 0.03;
        mp.gamma_r   = 1e-8;      // very slow adaptation (double-integrator-like plant)
        mp.gamma_y   = 1e-8;
        mp.sigma     = 0.05;
        mp.theta_max = 1.0e7;     // [N/m] scale
        mp.uMin      = -TAU_XY_MAX;
        mp.uMax      =  TAU_XY_MAX;
    } else {          // yaw
        mp.a_m       = 0.97;
        mp.b_m       = 0.03;
        mp.gamma_r   = 1e-10;
        mp.gamma_y   = 1e-10;
        mp.sigma     = 0.05;
        mp.theta_max = 1.0e10;
        mp.uMin      = -TAU_PSI_MAX;
        mp.uMax      =  TAU_PSI_MAX;
    }
    return mp;
}

MRACTugCtrl::MRACTugCtrl(const PlantParameters& p)
    : pp_(p)
    , mracs_{ ctrl::MRACController(mracParamsForAxis(0), p.dt),
               ctrl::MRACController(mracParamsForAxis(1), p.dt),
               ctrl::MRACController(mracParamsForAxis(2), p.dt) }
{}

Vector3d MRACTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    // MRAC takes plant output (position), setReference first
    Vector3d tau;
    for (int i = 0; i < 3; ++i) {
        mracs_[i].setReference(ref(i));
        tau(i) = mracs_[i].compute(state(i));
    }
    return saturateTau(tau);
}

void MRACTugCtrl::reset()
{
    for (auto& m : mracs_) m.reset();
}

// ===========================================================================
// Mode 15 - AutoGS-LQR (surge axis gain scheduling)
// ===========================================================================

AutoGSTugCtrl::AutoGSTugCtrl(const PlantParameters& p)
    : pp_(p)
    , x_surge_(Eigen::VectorXd::Zero(2))
    , pids_yw_{ ctrl::DiscretePID(pidParamsFor(1), p.dt),
                ctrl::DiscretePID(pidParamsFor(2), p.dt) }
{
    // Surge axis: 2-state [e_x, u_vel] with speed-dependent Coriolis damping.
    // Continuous model: [e_dot; u_dot] = [0,-1; 0, -(D+D_cor)/M]*[e;u] + [0; 1/M]*tau_kN*1e3
    // D_cor = M_b(1,1)*|v_vel| captures Coriolis coupling effect.
    // Scheduling on |u_vel| \in [0, 1.5] m/s.

    auto& pp_ref = pp_;  // capture by ref
    auto& x_ref  = x_surge_;

    // Nonlinear continuous dynamics for surge axis (2-state [e_x, u_vel])
    // p_sched = |u_vel| is the scheduling variable
    // u_eq(p_sched) = D_re(0,0)*p_sched (equilibrium force to hold speed p_sched)
    // x0_fn(p_sched) = [0.0, p_sched] (surge at speed p_sched, zero error)

    auto f_surge = [&pp_ref](const Eigen::VectorXd& xs, const Eigen::VectorXd& u_in)
                   -> Eigen::VectorXd {
        double vel = xs(1);
        double M   = pp_ref.M_re(0, 0);
        double D   = pp_ref.D_re(0, 0);
        Eigen::VectorXd xdot(2);
        xdot(0) = -vel;                            // e_dot = -u_vel
        xdot(1) = (-D * vel + u_in(0)) / M;       // u_vel_dot = accel
        return xdot;
    };

    auto u_eq_fn = [&pp_ref](double p_sched) -> Eigen::VectorXd {
        Eigen::VectorXd u(1);
        u(0) = pp_ref.D_re(0, 0) * p_sched;   // balance damping at speed p_sched
        return u;
    };

    auto x0_fn = [](double p_sched) -> Eigen::VectorXd {
        Eigen::VectorXd x(2);
        x(0) = 0.0;           // zero position error
        x(1) = p_sched;       // surge velocity = scheduling param
        return x;
    };

    auto design_fn = [](const ctrl::StateSpace& sys, double /*p*/)
                     -> std::shared_ptr<ctrl::IController> {
        // Extract LQR proportional gain and use a PD controller as SISO proxy.
        // The GainScheduledController only supports scalar compute(error), so we
        // extract K[0,0] (position error gain) and K[0,1] (velocity gain) from LQR.
        ctrl::LQRParams lp;
        Eigen::VectorXd xm(2), um(1);
        xm << 10.0, 1.0;   // max pos error 10m, max speed 1 m/s
        um << TAU_XY_MAX;
        lp = ctrl::LQRWeightTuner::brysonMethod(xm, um);
        ctrl::DiscreteLQR lqr(sys, lp);

        // Build PID with Kp from LQR position gain, Kd from velocity gain
        ctrl::PIDParams pp;
        pp.Kp   = lqr.gainMatrix()(0, 0);
        pp.Ki   = pp.Kp * 0.001;   // small integral for zero steady-state
        pp.Kd   = lqr.gainMatrix()(0, 1);
        pp.N    = 5.0;
        pp.Kb   = 1.0;
        pp.uMin = -TAU_XY_MAX;
        pp.uMax =  TAU_XY_MAX;
        return std::make_shared<ctrl::DiscretePID>(pp, sys.Ts);
    };

    // Build auto gain scheduler (5 grid points, gap threshold 0.3)
    try {
        sched_surge_ = ctrl::buildAutoGainScheduler(
            f_surge, 0.0, 1.5, 5, u_eq_fn, x0_fn, design_fn, p.dt,
            /*gap_threshold=*/0.3, /*freq_points=*/100);
    } catch (const std::exception& e) {
        // Fallback: create single-point scheduler with PID at zero speed
        sched_surge_ = std::make_shared<ctrl::GainScheduledController>(
            p.dt, ctrl::GainScheduleMode::NearestNeighbor);
        sched_surge_->addSchedulePoint(
            0.0, std::make_shared<ctrl::DiscretePID>(pidParamsFor(0), p.dt));
        (void)e;
    }
}

Vector3d AutoGSTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);

    // Surge axis via AutoGS LQR: schedule on |u_vel|
    double u_vel = state(3);   // surge velocity
    sched_surge_->setSchedulingParam(std::abs(u_vel));

    // For the LQR adapter inside sched_, state = [e_x, u_vel]
    // We need to route the error computation via the scheduler.
    // The GainScheduledController::compute(error) passes error directly to inner ctrl.
    // For DiscreteLQR (not LQRAdapter), compute(error) goes through IController interface
    // which ignores the MIMO nature. Use a scalar proxy: pass e(0) as the "error".
    double tau_x_kN = sched_surge_->compute(e(0));   // kN (Bryson xmax basis)
    // Scale: scheduler designed with TAU_XY_MAX umax but returns force in N
    double tau_x = std::clamp(tau_x_kN, -TAU_XY_MAX, TAU_XY_MAX);

    // y and psi remain PID
    double tau_y   = pids_yw_[0].compute(e(1));
    double tau_psi = pids_yw_[1].compute(e(2));

    return Vector3d(tau_x, tau_y, tau_psi);
}

void AutoGSTugCtrl::reset()
{
    if (sched_surge_) sched_surge_->reset();
    for (auto& pid : pids_yw_) pid.reset();
    x_surge_.setZero();
}

// ===========================================================================
// Mode 16 - NonlinearMPC (6-state ship dynamics)
// ===========================================================================

NMPCTugCtrl::NMPCTugCtrl(const PlantParameters& p)
    : pp_(p)
    , nmpc_([&]() {
        ctrl::NMPCParams np;
        np.n_states  = 6;
        np.n_inputs  = 3;
        np.n_outputs = 3;    // track [x, y, psi] only
        np.Np        = 20;
        np.Nu        = 5;
        np.rho_y     = 1.0;
        np.rho_u     = 1.0e-12;   // input in N (very large), minimal regularisation
        np.uMin      = -TAU_XY_MAX;  // tau_x, tau_y
        np.uMax      =  TAU_XY_MAX;
        np.Ts        = p.dt;
        // Note: uMin/uMax are scalar here; NMPC applies element-wise.
        // For mixed xy/psi bounds, we saturate in compute() after NMPC.

        auto f_nmpc = [&pp = p](const Eigen::VectorXd& x, const Eigen::VectorXd& u)
                      { return tugDynamics(x, u, pp); };

        // C = [I_3, 0_{3x3}]: output = position only
        Eigen::MatrixXd C_out = Eigen::MatrixXd::Zero(3, 6);
        C_out.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();

        return ctrl::NonlinearMPC(np, f_nmpc, C_out);
    }())
{}

Vector3d NMPCTugCtrl::compute(const Vector3d& ref, const Matrix<double,6,1>& state)
{
    Eigen::VectorXd x_nmpc(6);
    x_nmpc = state.cast<double>();

    Eigen::VectorXd y_ref(3);
    y_ref = ref.cast<double>();

    Eigen::VectorXd u_opt = nmpc_.computeRef(x_nmpc, y_ref);

    // Apply per-axis saturation (NMPC uMin/uMax is scalar; psi bound differs)
    Vector3d tau(
        std::clamp(u_opt(0), -TAU_XY_MAX,  TAU_XY_MAX),
        std::clamp(u_opt(1), -TAU_XY_MAX,  TAU_XY_MAX),
        std::clamp(u_opt(2), -TAU_PSI_MAX, TAU_PSI_MAX));
    return tau;
}

void NMPCTugCtrl::reset()
{
    nmpc_.reset();
}

// ===========================================================================
// Mode 17 - L1Adaptive (3-axis)
// ===========================================================================

static ctrl::L1AdaptiveController::Params l1ParamsForAxis(int axis,
                                                           const PlantParameters& pp)
{
    ctrl::L1AdaptiveController::Params lp;
    lp.a_m       = 0.97;
    lp.b_m       = 0.03;
    lp.k_g       = 1.0;
    lp.Gamma     = 1.0;
    lp.omega_c   = 0.05;
    lp.sigma_max = 1.0e8;
    lp.Q_lyap    = 1.0;
    if (axis < 2) {
        lp.uMin = -TAU_XY_MAX;
        lp.uMax =  TAU_XY_MAX;
    } else {
        lp.uMin = -TAU_PSI_MAX;
        lp.uMax =  TAU_PSI_MAX;
    }
    return lp;
}

L1AdaptiveTugCtrl::L1AdaptiveTugCtrl(const PlantParameters& p)
    : pp_(p)
    , l1s_{ ctrl::L1AdaptiveController(l1ParamsForAxis(0, p), p.dt),
            ctrl::L1AdaptiveController(l1ParamsForAxis(1, p), p.dt),
            ctrl::L1AdaptiveController(l1ParamsForAxis(2, p), p.dt) }
{}

Vector3d L1AdaptiveTugCtrl::compute(const Vector3d& ref,
                                    const Matrix<double,6,1>& state)
{
    Vector3d tau;
    for (int i = 0; i < 3; ++i) {
        l1s_[i].setReference(ref(i));
        tau(i) = l1s_[i].compute(state(i));
    }
    return saturateTau(tau);
}

void L1AdaptiveTugCtrl::reset()
{
    for (auto& l : l1s_) l.reset();
}

// ===========================================================================
// Mode 18 - ScenarioMPC (3x per-axis stochastic MPC)
// ===========================================================================

static ctrl::ScenarioMPC makeScenarioMPCForAxis(int axis, const PlantParameters& pp)
{
    ctrl::StateSpace sys = makeAxisSS(axis, pp);
    const double kN = 1.0e3;
    const double tau_max_kN = (axis < 2) ? TAU_XY_MAX / kN : TAU_PSI_MAX / kN;

    // Wave disturbance in kN units: 50 kN / mass -> velocity step per step
    double wv_kN = (axis < 2)
        ? (50.0e3 / kN) / pp.M_re(axis, axis) * pp.dt   // (kN) / mass -> kN velocity
        : (50.0e3 / kN) / pp.M_re(2, 2) * pp.dt;
    wv_kN = std::max(wv_kN, 1.0e-9);

    ctrl::ScenarioMPCParams sp;
    sp.Np        = 60;
    sp.Nu        = 5;
    sp.Q         = Eigen::MatrixXd::Identity(1, 1);
    sp.R         = Eigen::MatrixXd::Identity(1, 1);
    sp.Sigma_w   = Eigen::MatrixXd::Zero(2, 2);
    sp.Sigma_w(0,0) = 1e-6;             // position noise variance
    sp.Sigma_w(1,1) = wv_kN * wv_kN;   // velocity noise variance
    sp.N_samples = 30;
    sp.uMin      = Eigen::VectorXd::Constant(1, -tau_max_kN);
    sp.uMax      = Eigen::VectorXd::Constant(1,  tau_max_kN);
    sp.Ts        = pp.dt;
    return ctrl::ScenarioMPC(sys, sp);
}

ScenarioMPCTugCtrl::ScenarioMPCTugCtrl(const PlantParameters& p)
    : pp_(p)
    , smpcs_{ makeScenarioMPCForAxis(0, p),
              makeScenarioMPCForAxis(1, p),
              makeScenarioMPCForAxis(2, p) }
{}

Vector3d ScenarioMPCTugCtrl::compute(const Vector3d& ref,
                                     const Matrix<double,6,1>& state)
{
    Vector3d e = bodyError(ref, state);
    Vector3d tau;

    for (int axis = 0; axis < 3; ++axis) {
        Eigen::VectorXd x_ax(2);
        x_ax(0) = e(axis);
        x_ax(1) = state(3 + axis);

        Eigen::VectorXd r_ax(1); r_ax(0) = 0.0;

        Eigen::VectorXd u_kN = smpcs_[axis].computeRef(x_ax, r_ax);
        tau(axis) = u_kN(0) * 1.0e3;   // kN -> N
    }
    return saturateTau(tau);
}

void ScenarioMPCTugCtrl::reset()
{
    for (auto& s : smpcs_) s.reset();
}

} // namespace tug
