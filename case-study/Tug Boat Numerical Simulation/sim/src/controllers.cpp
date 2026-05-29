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
// Mode 2 - KF-PID
// ===========================================================================

ctrl::StateSpace KFPIDController::buildPlantSS() const
{
    // Linearise paper Eq. (21) about zero velocity.
    // Continuous: M_re * nu_dot = -D_re * nu  ->  nu_dot = -M_re^-1 * D_re * nu
    // State: [x, y, psi, u, v, r]
    // A_c = [0_{3x3}  I_{3x3}; 0_{3x3}  -M_re_inv * D_re]
    // B_c = [0_{3x3}; M_re_inv]  (control = tau_c in N/N.m)
    int n = 6, m = 3;
    MatrixXd Ac = MatrixXd::Zero(n, n);
    MatrixXd Bc = MatrixXd::Zero(n, m);
    Ac.block<3,3>(0, 3) = Matrix3d::Identity();
    Ac.block<3,3>(3, 3) = -(pp_.M_re_inv * pp_.D_re);
    Bc.block<3,3>(3, 0) = pp_.M_re_inv;

    // Scale B so KF input is normalised to ~1 (avoid numerical issues)
    // We pass tau_c / 1e5 as input, so Bc_scaled = Bc * 1e5
    // Actually we keep unscaled; KF Q/R tuning absorbs it.

    MatrixXd Cc = MatrixXd::Identity(n, n);  // full-state measurement
    MatrixXd Dc = MatrixXd::Zero(n, m);

    // Continuous SS (Ts=0 signals c2d to discretise)
    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, pp_.dt, ctrl::C2dMethod::ZOH);
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
    // Single-axis error-velocity model.  State = [e, nu] where e = ref - position.
    // Since e_dot = -nu (positive motion reduces error), A(0,1) = -1.
    // Input tau in kN to keep Bd magnitude ~1e-5 and avoid QP Hessian ill-conditioning.
    //
    // Continuous: [e_dot; nu_dot] = [0 -1; 0 -d/m] * [e; nu] + [0; kN2N/m] * tau_kN
    double m = pp_.M_re(axis, axis);
    double d = pp_.D_re(axis, axis);
    const double kN2N = 1.0e3;

    MatrixXd Ac(2, 2); Ac << 0, -1, 0, -d/m;
    MatrixXd Bc(2, 1); Bc << 0, kN2N/m;
    MatrixXd Cc(1, 2); Cc << 1, 0;
    MatrixXd Dc(1, 1); Dc << 0;

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, pp_.dt, ctrl::C2dMethod::ZOH);
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

// Build a per-axis 2-state SS model linearised about operating velocity nu.
// Continuous: [e_dot; nu_dot] = [0 1; 0 -(D+d_nu)/m] [e; nu] + [0; 1/m] tau
// where d_nu is velocity-dependent damping correction at operating point:
//   The nonlinear Coriolis coupling means effective damping at nu != 0 differs
//   from the zero-velocity linearisation.  We approximate the Coriolis
//   contribution as an additive damping correction on the diagonal.
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

} // namespace tug
