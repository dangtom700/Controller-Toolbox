#include "controllers.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace Eigen;

namespace boiler {

// ============================================================================
// Helper: build LQR parameters (Bryson's method) shared across controllers
// ============================================================================

static ctrl::LQRParams brysonLQRParams()
{
    Eigen::VectorXd xmax(3), umax(3);
    xmax << 5.0, 10.0, 1.0;
    umax << 0.3,  0.3,  0.1;
    return ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
}

// ============================================================================
// 1. PID
// ============================================================================

static ctrl::PIDParams pidParamsFor(int axis)
{
    ctrl::PIDParams p;
    p.Kp   = 0.05;
    p.Ki   = 0.002;
    p.Kd   = 0.02;
    p.N    = 5.0;
    p.Kb   = 1.0 / (p.Ki > 0 ? p.Ki : 1.0);
    p.uMin = -0.5;
    p.uMax =  0.5;
    (void)axis;
    return p;
}

PIDController::PIDController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : pids_{ ctrl::DiscretePID(pidParamsFor(0), ss.Ts),
             ctrl::DiscretePID(pidParamsFor(1), ss.Ts),
             ctrl::DiscretePID(pidParamsFor(2), ss.Ts) }
{
    (void)op;
}

Vector3d PIDController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    return Vector3d(
        pids_[0].compute(e(0)),
        pids_[1].compute(e(1)),
        pids_[2].compute(e(2)));
}

void PIDController::reset()
{
    for (auto& p : pids_) p.reset();
}

// ============================================================================
// 2. LQR
// ============================================================================

LQRController::LQRController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : lqr_(ss, brysonLQRParams())
    , Nbar_(computeNbar(ss))
{
    (void)op;
}

Vector3d LQRController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // LQR: u = -K*x + Nbar*ref  (deviation space: x = dy state estimate)
    // For output feedback we use dy as a proxy for state dx.
    Eigen::VectorXd x_dev = dy;
    Eigen::VectorXd r_vec = ref_dy;
    Eigen::VectorXd du = lqr_.compute(x_dev, r_vec);
    return du.head<3>();
}

void LQRController::reset()
{
    // DiscreteLQR is stateless; nothing to reset
}

// ============================================================================
// 3. LQG
// ============================================================================

LQGController::LQGController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : lqg_(ss,
           brysonLQRParams(),
           1e-4 * Eigen::Matrix3d::Identity(),
           (Eigen::Matrix3d() << 0.25, 0, 0,
                                  0, 1.0, 0,
                                  0, 0, 25.0).finished())
    , du_prev_(Vector3d::Zero())
{
    (void)op;
}

Vector3d LQGController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Eigen::VectorXd x_ref = ref_dy;
    Eigen::VectorXd du    = lqg_.step(dy, du_prev_, x_ref);
    du_prev_ = du.head<3>();
    return du_prev_;
}

void LQGController::reset()
{
    lqg_.reset();
    du_prev_.setZero();
}

// ============================================================================
// 4. MPC
// ============================================================================

MPCController::MPCController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : mpc_([&] {
        auto rec = ctrl::MPCHorizonTuner::recommend(ss, ss.Ts);
        ctrl::MPCParams mp;
        mp.Np        = std::min(rec.Np, 20);
        mp.Nc        = std::min(rec.Nc,  5);
        mp.rho_y     = rec.rho_y;
        mp.rho_u     = rec.rho_u;
        mp.uMin      = -0.5;
        mp.uMax      =  0.5;
        // Rate constraints from plant_params.json du_max values (most restrictive = 0.020 steam valve).
        // Prevents the MPC from requesting physically impossible fast valve movements.
        mp.duMin     = -0.020;
        mp.duMax     =  0.020;
        mp.qpMaxIter = 1000;  // boiler Hessian kappa~50; FISTA needs ~sqrt(50)*log(1/tol) < 500
        return ctrl::DiscreteMPC(ss, mp);
    }())
{
    (void)op;
}

Vector3d MPCController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Eigen::VectorXd r_ref = ref_dy;
    Eigen::VectorXd du    = mpc_.computeRef(dy, r_ref);
    return du.head<3>();
}

void MPCController::reset()
{
    mpc_.reset();
}

// ============================================================================
// 5. SMC
// ============================================================================

// Per-axis SMC params: boundary layer phi sized to expected steady-state |s| = c_e*|e_ss|.
// Axis 0 (drum pressure): |e_ss| ~ 0.03 -> phi = 0.10 (3x headroom)
// Axis 1 (steam flow rate): larger excursions |e_ss| ~ 0.07 -> phi = 0.20
// Axis 2 (evaporation rate): |e_ss| ~ 0.01 -> phi = 0.05
// Larger phi suppresses chattering; K/phi sets the effective P-gain in the boundary layer.
static ctrl::SMCParams smcParamsFor(int axis)
{
    ctrl::SMCParams p;
    p.c_e  = 1.0;
    p.c_de = 0.2;  // lambda = c_de/Ts = 0.2 rad/s sliding-surface derivative weight
    p.K    = 0.10; // switching gain; K/phi = effective P-gain when |s| < phi
    p.uMin = -0.5;
    p.uMax =  0.5;
    switch (axis) {
        case 0:  p.phi = 0.10; break;  // drum pressure axis
        case 1:  p.phi = 0.20; break;  // steam flow axis (larger fluctuations -> wider layer)
        default: p.phi = 0.05; break;  // evaporation rate (tight, small deviations)
    }
    return p;
}

SMCController::SMCController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : smcs_{ ctrl::DiscreteSMC(smcParamsFor(0), ss.Ts),
             ctrl::DiscreteSMC(smcParamsFor(1), ss.Ts),
             ctrl::DiscreteSMC(smcParamsFor(2), ss.Ts) }
{
    (void)op;
}

Vector3d SMCController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // DiscreteSMC::compute() expects error = y - ref (sliding surface s = c_e*(y-r) + c_de*ds).
    // A positive s -> output above ref -> u = -K*sat(s) drives output down. Pass dy-ref_dy.
    Vector3d e_smc = dy - ref_dy;
    return Vector3d(
        smcs_[0].compute(e_smc(0)),
        smcs_[1].compute(e_smc(1)),
        smcs_[2].compute(e_smc(2)));
}

void SMCController::reset()
{
    for (auto& s : smcs_) s.reset();
}

// ============================================================================
// 6. ESC
// ============================================================================

ESCController::ESCController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : esc_([&] {
        ctrl::ExtremumSeekerParams p;
        p.perturbAmp  = 0.005;
        p.perturbFreq = 0.02;
        p.lpfCutoff   = 0.005;
        p.hpfCutoff   = 0.002;
        p.integGain   = 0.5;
        p.seekMinimum = false;  // maximise y3
        return ctrl::ExtremumSeeker(p, ss.Ts);
    }())
    , op_(op)
{
}

Vector3d ESCController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    (void)ref_dy;
    // ESC on u3: maximise y3 (absolute output)
    double y3_abs = op_.y3 + dy(2);
    double u3_dev = esc_.compute(y3_abs) - op_.u3;
    // u1, u2 stay at operating point (du=0)
    return Vector3d(0.0, 0.0, u3_dev);
}

void ESCController::reset()
{
    esc_.reset();
}

// ============================================================================
// 7. ADRC
// ============================================================================

ADRCController::ADRCController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : adrcs_([&] {
        // b0 estimates from diagonal of Bc (continuous, before ZOH scaling)
        // axis 0: Bc(0,0) = 0.9  -> b0 ~ 0.9*Ts
        // axis 1: Bc(1,1) = 0.073*x1^(9/8) -> approximate at op
        // axis 2: Bc(2,2) = 141/85 -> b0 ~ (141/85)*Ts
        const double x1_98 = std::pow(op.x1, 9.0 / 8.0);
        const double Ts    = ss.Ts;

        // Bandwidth tuning (Ts=1s). Stability constraint for backward-Euler ADRC:
        // omega_o*Ts < ~0.5 and omega_c*Ts < ~0.2 to avoid closed-loop instability from
        // discretisation error in the ESO (verified empirically: omega_o=0.75 diverges).
        // Safe choice: omega_o=0.5/Ts=0.5, omega_c=omega_o/5=0.10.
        ctrl::ADRCParams p0; p0.omega_o=0.50; p0.omega_c=0.10; p0.b0=0.9*Ts;         p0.uMin=-0.5; p0.uMax=0.5;
        ctrl::ADRCParams p1; p1.omega_o=0.40; p1.omega_c=0.08; p1.b0=0.073*x1_98*Ts; p1.uMin=-0.5; p1.uMax=0.5;
        ctrl::ADRCParams p2; p2.omega_o=0.40; p2.omega_c=0.08; p2.b0=(141.0/85.0)*Ts; p2.uMin=-0.5; p2.uMax=0.5;

        return std::array<ctrl::DiscreteADRC, 3>{
            ctrl::DiscreteADRC(p0, Ts),
            ctrl::DiscreteADRC(p1, Ts),
            ctrl::DiscreteADRC(p2, Ts)
        };
    }())
    , op_(op)
{
}

Vector3d ADRCController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // ADRC takes absolute y and absolute reference
    Vector3d y_abs   = Eigen::Vector3d(op_.y1, op_.y2, op_.y3) + dy;
    Vector3d ref_abs = Eigen::Vector3d(op_.y1, op_.y2, op_.y3) + ref_dy;

    return Vector3d(
        adrcs_[0].computeTracking(y_abs(0), ref_abs(0)),
        adrcs_[1].computeTracking(y_abs(1), ref_abs(1)),
        adrcs_[2].computeTracking(y_abs(2), ref_abs(2)));
}

void ADRCController::reset()
{
    for (auto& a : adrcs_) a.reset();
}

// ============================================================================
// 8. LeadLag + PID
// ============================================================================

LeadLagPIDController::LeadLagPIDController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : leadlags_([&] {
        // Simple fixed lead-lag: zero at 0.01 rad/s, pole at 0.05 rad/s, gain 1
        ctrl::LeadLagParams ll;
        ll.continuousZero = 0.01;
        ll.continuousPole = 0.05;
        ll.gain           = 1.0;
        return std::array<ctrl::DiscreteLeadLag, 3>{
            ctrl::DiscreteLeadLag(ll, ss.Ts),
            ctrl::DiscreteLeadLag(ll, ss.Ts),
            ctrl::DiscreteLeadLag(ll, ss.Ts)
        };
    }())
    , pids_([&] {
        auto p = pidParamsFor(0);
        return std::array<ctrl::DiscretePID, 3>{
            ctrl::DiscretePID(p, ss.Ts),
            ctrl::DiscretePID(p, ss.Ts),
            ctrl::DiscretePID(p, ss.Ts)
        };
    }())
{
    (void)op;
}

Vector3d LeadLagPIDController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    Vector3d du;
    for (int i = 0; i < 3; ++i) {
        double e_filtered = leadlags_[i].compute(e(i));
        du(i) = pids_[i].compute(e_filtered);
    }
    return du;
}

void LeadLagPIDController::reset()
{
    for (auto& ll : leadlags_) ll.reset();
    for (auto& p  : pids_)     p.reset();
}

// ============================================================================
// 9. Smith Predictor
// ============================================================================

static ctrl::StateSpace buildSmithDelayModel(const ctrl::StateSpace& ss, int axis)
{
    // Use the diagonal channel for the Smith Predictor delay-free model
    return diagonalChannel(ss, axis);
}

SmithPredictorController::SmithPredictorController(const ctrl::StateSpace& ss,
                                                   const OperatingPoint& op)
    : sps_([&] {
        // Estimate delay: 2 steps (2 s) as representative steam path delay
        const int d_steps = 2;
        auto make_sp = [&](int axis) {
            auto inner_pid = std::make_shared<ctrl::DiscretePID>(pidParamsFor(axis), ss.Ts);
            return ctrl::SmithPredictor(inner_pid, buildSmithDelayModel(ss, axis), d_steps);
        };
        return std::array<ctrl::SmithPredictor, 3>{
            make_sp(0), make_sp(1), make_sp(2)
        };
    }())
{
    (void)op;
}

Vector3d SmithPredictorController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    return Vector3d(
        sps_[0].compute(e(0)),
        sps_[1].compute(e(1)),
        sps_[2].compute(e(2)));
}

void SmithPredictorController::reset()
{
    for (auto& sp : sps_) sp.reset();
}

// ============================================================================
// 10. GPC + RLS
// ============================================================================

GPCController::GPCController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : gpcs_([&] {
        ctrl::GPCParams gp;
        // Np=20: covers ~20 steps for a boiler with 100s time constant (need at least tau/Ts=100 steps,
        // but Np=20 is a practical compromise - more steps improves tracking at increased compute cost).
        // Nu=3: 3 control moves balances lookahead vs computation.
        // alpha=0.8: reference trajectory filter. alpha=0.1 reaches setpoint in ~1 step (physically
        // impossible for slow boiler -> saturates every step -> diverges). 0.8 gives ~10-step approach.
        // rho_u=0.5: higher than 0.1 to penalise fast valve movements and prevent saturation.
        gp.Np        = 20;
        gp.Nu        = 3;
        gp.rho_y     = 1.0;
        gp.rho_u     = 0.5;
        gp.alpha     = 0.8;
        gp.uMin      = -0.5;
        gp.uMax      =  0.5;
        gp.qpMaxIter = 1000;  // GPC Hessian; FISTA converges well under 200 with these params
        auto ch0 = diagonalChannel(ss, 0);
        auto ch1 = diagonalChannel(ss, 1);
        auto ch2 = diagonalChannel(ss, 2);
        return std::array<ctrl::GeneralizedPredictiveController, 3>{
            ctrl::GeneralizedPredictiveController(ch0, gp),
            ctrl::GeneralizedPredictiveController(ch1, gp),
            ctrl::GeneralizedPredictiveController(ch2, gp)
        };
    }())
    // Forgetting factor 0.98 -> RLS half-life ~ 34 steps (too aggressive; model drifts under
    // large setpoint changes). 0.995 -> half-life ~ 138 steps for better stability.
    , rls_{ ctrl::RecursiveLeastSquares(2, 2, ss.Ts, 0.995),
            ctrl::RecursiveLeastSquares(2, 2, ss.Ts, 0.995),
            ctrl::RecursiveLeastSquares(2, 2, ss.Ts, 0.995) }
    , u_prev_(Vector3d::Zero())
{
    (void)op;
}

Vector3d GPCController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d du;
    for (int i = 0; i < 3; ++i) {
        // Update RLS with latest I/O
        rls_[i].update(dy(i), u_prev_(i));

        // Hot-swap GPC plant every kRLSUpdateInterval steps after warmup.
        // Guard: only swap if the identified model is stable (all poles inside
        // the unit disk). Closed-loop RLS can identify unstable models during
        // large setpoint transitions, which causes NaN in the GPC Hessian.
        if (step_count_ >= kRLSWarmup &&
            step_count_ % kRLSUpdateInterval == 0) {
            auto plant = rls_[i].toStateSpace();
            const Eigen::VectorXcd eigs = plant.A.eigenvalues();
            bool stable = true;
            for (int k = 0; k < eigs.size(); ++k)
                if (std::abs(eigs(k)) >= 1.0) { stable = false; break; }
            if (stable)
                gpcs_[i].setPlant(plant);
        }

        du(i) = gpcs_[i].computeRef(dy(i), ref_dy(i));
    }
    u_prev_ = du;
    ++step_count_;
    return du;
}

void GPCController::reset()
{
    for (auto& g : gpcs_) g.reset();
    for (auto& r : rls_)  r.reset();
    u_prev_.setZero();
    step_count_ = 0;
}

// ============================================================================
// 11. EKF-LQR
// ============================================================================

// Nonlinear process function: Euler step of Bell-Astrom ODE
static VectorXd boilerF(const VectorXd& x, const VectorXd& u)
{
    const double Ts   = 1.0;
    const double x1   = x(0), x2 = x(1), x3 = x(2);
    const double u1   = u(0), u2 = u(1), u3 = u(2);
    const double x1_98 = std::pow(std::max(x1, 1.0), 9.0 / 8.0);

    VectorXd x_next(3);
    x_next(0) = x1 + Ts * (-0.0018 * u2 * x1_98 + 0.9 * u1 - 0.15 * u3);
    x_next(1) = x2 + Ts * ((0.073 * u2 - 0.016) * x1_98 - 0.1 * x2);
    x_next(2) = x3 + Ts * ((141.0 * u3 - (1.1 * u2 - 0.19) * x1) / 85.0);
    return x_next;
}

// Measurement function: y = [x1, x2, y3(x,u)]
static VectorXd boilerH(const VectorXd& x, const VectorXd& u)
{
    VectorXd y(3);
    y(0) = x(0);
    y(1) = x(1);
    y(2) = computeY3(x(0), x(1), x(2), u(0), u(1), u(2));
    return y;
}

EKFLQRController::EKFLQRController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : lqr_(ss, brysonLQRParams())
    , ekf_([&] {
        auto Qn = 1e-4 * Eigen::Matrix3d::Identity();
        auto Rn = (Eigen::Matrix3d() <<
            0.25,  0,   0,
            0,   1.0,   0,
            0,   0,  25.0).finished();

        // Jacobian F = df/dx (analytical via linearize)
        auto Fjac = [op](const VectorXd& x, const VectorXd& u) -> MatrixXd {
            OperatingPoint cur_op = op;
            cur_op.x1 = x(0); cur_op.x2 = x(1); cur_op.x3 = x(2);
            cur_op.u1 = u(0); cur_op.u2 = u(1); cur_op.u3 = u(2);
            ctrl::StateSpace lin = linearize(cur_op, 1.0);
            // Return the discrete Ad as the Jacobian approximation
            return lin.A;
        };

        // Jacobian H = dh/dx (from Cc of linearization)
        auto Hjac = [op](const VectorXd& x, const VectorXd& u) -> MatrixXd {
            OperatingPoint cur_op = op;
            cur_op.x1 = x(0); cur_op.x2 = x(1); cur_op.x3 = x(2);
            cur_op.u1 = u(0); cur_op.u2 = u(1); cur_op.u3 = u(2);
            ctrl::StateSpace lin = linearize(cur_op, 1.0);
            return lin.C;
        };

        return ctrl::ExtendedKalmanFilter(3, 3, boilerF, boilerH, Fjac, Hjac, Qn, Rn, ss.Ts);
    }())
    , du_prev_(Vector3d::Zero())
    , op_(op)
{
}

Vector3d EKFLQRController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // EKF: absolute measurements and absolute u
    VectorXd y_abs(3);
    y_abs << op_.y1 + dy(0), op_.y2 + dy(1), op_.y3 + dy(2);

    VectorXd u_abs(3);
    u_abs << op_.u1 + du_prev_(0),
             op_.u2 + du_prev_(1),
             op_.u3 + du_prev_(2);

    ekf_.step(y_abs, u_abs);

    // LQR on estimated state deviation
    VectorXd x_est(3);
    x_est << ekf_.state()(0) - op_.x1,
             ekf_.state()(1) - op_.x2,
             ekf_.state()(2) - op_.x3;

    VectorXd r_vec = ref_dy;
    VectorXd du    = lqr_.compute(x_est, r_vec);
    du_prev_ = du.head<3>();
    return du_prev_;
}

void EKFLQRController::reset()
{
    ekf_.reset();
    du_prev_.setZero();
    // Re-initialise EKF state at operating point
    VectorXd x0(3); x0 << op_.x1, op_.x2, op_.x3;
    ekf_.setState(x0);
}

// ============================================================================
// 12. UKF-LQR
// ============================================================================

UKFLQRController::UKFLQRController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : lqr_(ss, brysonLQRParams())
    , ukf_([&] {
        auto Qn = 1e-4 * Eigen::Matrix3d::Identity();
        auto Rn = (Eigen::Matrix3d() <<
            0.25,  0,   0,
            0,   1.0,   0,
            0,   0,  25.0).finished();
        // alpha = sqrt((n+kappa)/n) with n=3, kappa=0 gives Wc0 = 0 (avoids negative weights).
        // Default alpha=1e-3 gives Wc0 << 0 for n=3, causing covariance indefiniteness.
        // alpha = sqrt((n+kappa)/n) = 1.0 for n=3, kappa=0 -> Wc0 = 0 (avoids negativity).
        const Eigen::MatrixXd Qn_d = Qn;
        const Eigen::MatrixXd Rn_d = Rn;
        return ctrl::UnscentedKalmanFilter(3, 3, boilerF, boilerH, Qn_d, Rn_d, ss.Ts,
                                           Eigen::MatrixXd{},  // default P0
                                           1.0, 2.0, 0.0);    // alpha, beta, kappa
    }())
    , du_prev_(Vector3d::Zero())
    , op_(op)
{
}

Vector3d UKFLQRController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    VectorXd y_abs(3);
    y_abs << op_.y1 + dy(0), op_.y2 + dy(1), op_.y3 + dy(2);

    VectorXd u_abs(3);
    u_abs << op_.u1 + du_prev_(0),
             op_.u2 + du_prev_(1),
             op_.u3 + du_prev_(2);

    ukf_.step(y_abs, u_abs);

    VectorXd x_est(3);
    x_est << ukf_.state()(0) - op_.x1,
             ukf_.state()(1) - op_.x2,
             ukf_.state()(2) - op_.x3;

    VectorXd r_vec = ref_dy;
    VectorXd du    = lqr_.compute(x_est, r_vec);
    du_prev_ = du.head<3>();
    return du_prev_;
}

void UKFLQRController::reset()
{
    ukf_.reset();
    du_prev_.setZero();
    VectorXd x0(3); x0 << op_.x1, op_.x2, op_.x3;
    ukf_.setState(x0);
}

// ============================================================================
// 13. FuzzyPID
// ============================================================================

static ctrl::FuzzyPIDParams fuzzyPIDParamsFor(int axis)
{
    ctrl::FuzzyPIDParams p;
    p.Kb   = 0.8;
    p.Ki   = 0.001;
    p.uMin = -0.5;
    p.uMax =  0.5;

    if (axis == 0) {        // y1: pressure
        p.pd.e_scale  = 10.0;
        p.pd.de_scale =  2.0;
        p.pd.u_scale  =  0.3;
        p.pd.uMin     = -0.5;
        p.pd.uMax     =  0.5;
    } else if (axis == 1) { // y2: power
        p.pd.e_scale  = 20.0;
        p.pd.de_scale =  5.0;
        p.pd.u_scale  =  0.3;
        p.pd.uMin     = -0.5;
        p.pd.uMax     =  0.5;
    } else {                // y3: efficiency
        p.pd.e_scale  = 0.05;
        p.pd.de_scale = 0.01;
        p.pd.u_scale  = 0.1;
        p.pd.uMin     = -0.1;
        p.pd.uMax     =  0.1;
    }
    return p;
}

FuzzyPIDController::FuzzyPIDController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : fuzzies_{ ctrl::FuzzyPID(fuzzyPIDParamsFor(0), ss.Ts),
                ctrl::FuzzyPID(fuzzyPIDParamsFor(1), ss.Ts),
                ctrl::FuzzyPID(fuzzyPIDParamsFor(2), ss.Ts) }
{
    (void)op;
}

Vector3d FuzzyPIDController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    return Vector3d(
        fuzzies_[0].compute(e(0)),
        fuzzies_[1].compute(e(1)),
        fuzzies_[2].compute(e(2)));
}

void FuzzyPIDController::reset()
{
    for (auto& f : fuzzies_) f.reset();
}

// ============================================================================
// 14. FuzzySup-MPC
// ============================================================================

FuzzySupMPCController::FuzzySupMPCController(const ctrl::StateSpace& ss,
                                             const OperatingPoint& op)
    : ss_(ss)
    , op_(op)
    , mpcs_([&] {
        auto rec = ctrl::MPCHorizonTuner::recommend(ss, ss.Ts);
        ctrl::MPCParams mp;
        mp.Np        = std::min(rec.Np, 20);
        mp.Nc        = std::min(rec.Nc,  5);
        mp.rho_y     = rec.rho_y;
        mp.rho_u     = rec.rho_u;
        mp.uMin      = -0.5;
        mp.uMax      =  0.5;
        mp.duMin     = -0.020;
        mp.duMax     =  0.020;
        mp.qpMaxIter = 1000;  // match MPCController; FISTA needs <500 for this Hessian
        return std::array<ctrl::DiscreteMPC, 3>{
            ctrl::DiscreteMPC(ss, mp),
            ctrl::DiscreteMPC(ss, mp),
            ctrl::DiscreteMPC(ss, mp)
        };
    }())
    , supervisors_([&] {
        ctrl::SupervisorParams sp;
        sp.e_threshold     = 5.0;
        sp.trend_threshold = 0.5;
        sp.signal_threshold = 0.5;
        sp.cooldown_steps  = 120;
        return std::array<ctrl::FuzzySupervisor, 3>{
            ctrl::FuzzySupervisor(sp, ss.Ts),
            ctrl::FuzzySupervisor(sp, ss.Ts),
            ctrl::FuzzySupervisor(sp, ss.Ts)
        };
    }())
    , x_current_(op.x1, op.x2, op.x3)
{
}

Vector3d FuzzySupMPCController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // Update current state estimate (x_op + dx = x_op + dy for output = state channels 0,1)
    x_current_(0) = op_.x1 + dy(0);
    x_current_(1) = op_.x2 + dy(1);
    // x3 not directly observable from y3 alone; keep at op
    x_current_(2) = op_.x3;

    // Check fuzzy supervisor per axis; re-linearise if triggered
    for (int i = 0; i < 3; ++i) {
        auto dec = supervisors_[i].update(std::abs(dy(i) - ref_dy(i)));
        if (dec.relinearize) {
            OperatingPoint cur_op = op_;
            cur_op.x1 = x_current_(0);
            cur_op.x2 = x_current_(1);
            cur_op.x3 = x_current_(2);
            cur_op.u1 = op_.u1; cur_op.u2 = op_.u2; cur_op.u3 = op_.u3;
            ctrl::StateSpace new_ss = linearize(cur_op, ss_.Ts);
            mpcs_[i].setPlant(new_ss);
        }
    }

    // Each MPC channel independently optimises its own axis.
    // Bug fix: was erroneously calling only mpcs_[0] for all three axes.
    // Each per-axis MPC is a 3-state MIMO controller (all outputs), so we still
    // call computeRef with the full dy/ref vectors, but use the axis-specific MPC
    // that has been re-linearised (if triggered) by its own FuzzySupervisor.
    Eigen::VectorXd r_ref = ref_dy;
    Eigen::VectorXd du(3);
    du.setZero();
    for (int i = 0; i < 3; ++i) {
        Eigen::VectorXd u_i = mpcs_[i].computeRef(dy, r_ref);
        // Take only the i-th output of each MPC (the axis this MPC is responsible for)
        du(i) = u_i(i);
    }
    return du.head<3>();
}

void FuzzySupMPCController::reset()
{
    for (auto& m : mpcs_)       m.reset();
    for (auto& s : supervisors_) s.reset();
    x_current_ = Eigen::Vector3d(op_.x1, op_.x2, op_.x3);
}

// ============================================================================
// 15. Supervisory Stack (SMC -> LQR)
// ============================================================================

SupervisoryStackController::SupervisoryStackController(const ctrl::StateSpace& ss,
                                                       const OperatingPoint& op)
    : stacks_([&] {
        auto make_stack = [&](int axis) {
            ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, ss.Ts);

            auto smc_ctrl = std::make_shared<ctrl::DiscreteSMC>(smcParamsFor(axis), ss.Ts);
            auto lqr_p    = brysonLQRParams();
            // SISO LQR approximation via PID with LQR-tuned gains
            auto pid_p    = pidParamsFor(axis);
            auto lqr_ctrl = std::make_shared<ctrl::DiscretePID>(pid_p, ss.Ts);

            // SMC activates when |error| > 5
            stack.addController(smc_ctrl, "SMC",
                1.0, [](double e, double) { return std::abs(e) > 5.0; });
            // LQR/PID fallback (always eligible)
            stack.addController(lqr_ctrl, "LQR", 1.0, nullptr);
            return stack;
        };
        return std::array<ctrl::ControllerStack, 3>{
            make_stack(0), make_stack(1), make_stack(2)
        };
    }())
{
    (void)op;
}

Vector3d SupervisoryStackController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    return Vector3d(
        stacks_[0].compute(e(0)),
        stacks_[1].compute(e(1)),
        stacks_[2].compute(e(2)));
}

void SupervisoryStackController::reset()
{
    for (auto& s : stacks_) s.reset();
}

// ============================================================================
// 16. Additive Stack (PID + LeadLag, fade LeadLag out over 300 steps)
// ============================================================================

AdditiveStackController::AdditiveStackController(const ctrl::StateSpace& ss,
                                                  const OperatingPoint& op)
    : stacks_([&] {
        ctrl::LeadLagParams ll;
        ll.continuousZero = 0.01;
        ll.continuousPole = 0.05;
        ll.gain           = 1.0;

        auto make_stack = [&](int) {
            ctrl::ControllerStack stack(ctrl::StackMode::Additive, ss.Ts);
            auto pid_c = std::make_shared<ctrl::DiscretePID>(pidParamsFor(0), ss.Ts);
            auto ll_c  = std::make_shared<ctrl::DiscreteLeadLag>(ll, ss.Ts);
            stack.addController(pid_c, "PID",     1.0, nullptr);
            stack.addController(ll_c,  "LeadLag", 1.0, nullptr);
            return stack;
        };
        return std::array<ctrl::ControllerStack, 3>{
            make_stack(0), make_stack(1), make_stack(2)
        };
    }())
{
    (void)op;
}

Vector3d AdditiveStackController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // Fade LeadLag weight from 1 -> 0 over first kFadeSteps
    double ll_weight = (step_count_ < kFadeSteps)
                       ? 1.0 - static_cast<double>(step_count_) / kFadeSteps
                       : 0.0;
    for (auto& s : stacks_) s.setWeight("LeadLag", ll_weight);

    ++step_count_;

    Vector3d e = ref_dy - dy;
    return Vector3d(
        stacks_[0].compute(e(0)),
        stacks_[1].compute(e(1)),
        stacks_[2].compute(e(2)));
}

void AdditiveStackController::reset()
{
    for (auto& s : stacks_) s.reset();
    step_count_ = 0;
}

// ============================================================================
// 17. Weighted Stack (PID + LQR-tuned PID, weights by pressure deviation)
// ============================================================================

WeightedStackController::WeightedStackController(const ctrl::StateSpace& ss,
                                                  const OperatingPoint& op)
    : stacks_([&] {
        auto make_stack = [&](int) {
            ctrl::ControllerStack stack(ctrl::StackMode::Weighted, ss.Ts);
            auto pid_c = std::make_shared<ctrl::DiscretePID>(pidParamsFor(0), ss.Ts);
            auto lqr_c = std::make_shared<ctrl::DiscretePID>(pidParamsFor(0), ss.Ts);
            stack.addController(pid_c, "PID", 0.5, nullptr);
            stack.addController(lqr_c, "LQR", 0.5, nullptr);
            return stack;
        };
        return std::array<ctrl::ControllerStack, 3>{
            make_stack(0), make_stack(1), make_stack(2)
        };
    }())
    , op_(op)
{
}

Vector3d WeightedStackController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // Weight based on y1 deviation from operating point (normalised to [0,1])
    // w_pid = clamp((x1 - 75) / 65, 0, 1),  w_lqr = 1 - w_pid
    double x1_approx = op_.x1 + dy(0);
    double w_pid = std::clamp((x1_approx - 75.0) / 65.0, 0.0, 1.0);
    double w_lqr = 1.0 - w_pid;

    for (auto& s : stacks_) {
        s.setWeight("PID", w_pid);
        s.setWeight("LQR", w_lqr);
    }

    Vector3d e = ref_dy - dy;
    return Vector3d(
        stacks_[0].compute(e(0)),
        stacks_[1].compute(e(1)),
        stacks_[2].compute(e(2)));
}

void WeightedStackController::reset()
{
    for (auto& s : stacks_) s.reset();
}

// ============================================================================
// 18. Repetitive Controller
// ============================================================================

RepetitiveCtrl::RepetitiveCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : rcs_([&] {
        ctrl::RepetitiveParams rp;
        rp.periodSteps = 600;   // 10 minutes at Ts=1s
        rp.Krc         = 0.5;
        rp.Q           = 0.98;
        rp.uMin        = -0.5;
        rp.uMax        =  0.5;

        auto make_rc = [&](int axis) {
            auto inner = std::make_shared<ctrl::DiscretePID>(pidParamsFor(axis), ss.Ts);
            return ctrl::RepetitiveController(inner, rp, ss.Ts);
        };
        return std::array<ctrl::RepetitiveController, 3>{
            make_rc(0), make_rc(1), make_rc(2)
        };
    }())
{
    (void)op;
}

Vector3d RepetitiveCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    return Vector3d(
        rcs_[0].compute(e(0)),
        rcs_[1].compute(e(1)),
        rcs_[2].compute(e(2)));
}

void RepetitiveCtrl::reset()
{
    for (auto& r : rcs_) r.reset();
}

// ============================================================================
// 19. MRACBoilerCtrl
// Per-channel reference model: y_m[k+1] = 0.80*y_m[k] + 0.20*r[k]  (DC gain=1).
// Absolute mode: setReference(y_op + ref_dy), compute(y_op + dy), du = u_abs - u_op.
// gamma_r = gamma_y = 2.0 > 0 (plant input gains are all positive for diagonal channels).
// ============================================================================

MRACBoilerCtrl::MRACBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : mracs_([&] {
        ctrl::MRACParams mp;
        mp.a_m       = 0.80;
        mp.b_m       = 0.20;
        mp.gamma_r   = 2.0;
        mp.gamma_y   = 2.0;
        mp.sigma     = 0.02;
        mp.theta_max = 50.0;
        mp.uMin      = 0.0;   // absolute valve position
        mp.uMax      = 1.0;
        return std::array<ctrl::MRACController, 3>{
            ctrl::MRACController(mp, ss.Ts),
            ctrl::MRACController(mp, ss.Ts),
            ctrl::MRACController(mp, ss.Ts)
        };
    }())
    , op_(op)
{}

Vector3d MRACBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    const double op_y[] = {op_.y1, op_.y2, op_.y3};
    const double op_u[] = {op_.u1, op_.u2, op_.u3};
    Vector3d du;
    for (int i = 0; i < 3; ++i) {
        mracs_[i].setReference(op_y[i] + ref_dy(i));
        double u_abs = mracs_[i].compute(op_y[i] + dy(i));
        du(i) = u_abs - op_u[i];
    }
    return du;
}

void MRACBoilerCtrl::reset()
{
    for (auto& m : mracs_) m.reset();
}

// ============================================================================
// 20. HinfBoilerCtrl
// Per-channel mixed-sensitivity synthesis on diagonal SISO model.
// W1 tracks low-frequency setpoints; W3 guards against high-frequency noise.
// std::optional avoids storing DiscreteHinf objects without a valid synthesis.
// ============================================================================

HinfBoilerCtrl::HinfBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : fallback_pids_{ ctrl::DiscretePID(pidParamsFor(0), ss.Ts),
                      ctrl::DiscretePID(pidParamsFor(1), ss.Ts),
                      ctrl::DiscretePID(pidParamsFor(2), ss.Ts) }
{
    (void)op;

    ctrl::HinfParams hp;
    hp.gammaInit = 2.0;

    for (int i = 0; i < 3; ++i) {
        try {
            auto G = diagonalChannel(ss, i);
            G.D.setZero();   // DGKF requires D22=0; small feedthrough from ZOH discretisation
            auto W1 = ctrl::MixedSensitivity::makeW1(0.05, 1.5, 0.001, ss.Ts);
            auto W2 = ctrl::MixedSensitivity::makeW2constant(2.0, ss.Ts);
            auto W3 = ctrl::MixedSensitivity::makeW3(0.5, 1.5, 0.001, ss.Ts);
            auto P  = ctrl::MixedSensitivity::build(G, W1, W2, W3);
            auto result = ctrl::DiscreteHinf::solve(P, hp);
            if (result.feasible)
                hinfs_[i].emplace(result);
        } catch (const std::exception&) {
            // synthesis failed; fallback_pids_[i] used
        }
    }
}

Vector3d HinfBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    Vector3d du;
    for (int i = 0; i < 3; ++i) {
        du(i) = hinfs_[i] ? hinfs_[i]->compute(e(i))
                           : fallback_pids_[i].compute(e(i));
    }
    return du;
}

void HinfBoilerCtrl::reset()
{
    for (int i = 0; i < 3; ++i) {
        if (hinfs_[i]) hinfs_[i]->reset();
        fallback_pids_[i].reset();
    }
}

// ============================================================================
// 21. AdaptiveSPBoilerCtrl
// Online cross-correlation re-estimates dead-time (search range 0..8 steps).
// setPlantOutput(y_abs) must be called before compute() each step.
// initialDelay=2 steps (2 s) matching SmithPredictorController.
// ============================================================================

AdaptiveSPBoilerCtrl::AdaptiveSPBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : sps_([&] {
        ctrl::AdaptiveSPParams asp;
        asp.maxDelaySteps    = 8;
        asp.estimateInterval = 100;
        asp.bufferLen        = 200;

        auto make_asp = [&](int axis) {
            auto inner = std::make_shared<ctrl::DiscretePID>(pidParamsFor(axis), ss.Ts);
            return ctrl::AdaptiveSmithPredictor(
                inner, buildSmithDelayModel(ss, axis), 2, ss.Ts, asp);
        };
        return std::array<ctrl::AdaptiveSmithPredictor, 3>{
            make_asp(0), make_asp(1), make_asp(2)
        };
    }())
    , op_(op)
{}

Vector3d AdaptiveSPBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    const double op_y[] = {op_.y1, op_.y2, op_.y3};
    Vector3d du;
    for (int i = 0; i < 3; ++i) {
        sps_[i].setPlantOutput(op_y[i] + dy(i));
        du(i) = sps_[i].compute(ref_dy(i) - dy(i));
    }
    return du;
}

void AdaptiveSPBoilerCtrl::reset()
{
    for (auto& sp : sps_) sp.reset();
}

// ============================================================================
// 22. NMPCBoilerCtrl
// ============================================================================

NMPCBoilerCtrl::NMPCBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : nmpc_([&]() {
        ctrl::NMPCParams np;
        np.n_states  = 3;
        np.n_inputs  = 3;
        np.n_outputs = 3;
        np.Np        = 10;
        np.Nu        = 3;
        np.rho_y     = 1.0;
        np.rho_u     = 0.1;
        np.uMin      = -0.5;    // max du per step [valve fraction]
        np.uMax      =  0.5;
        np.Ts        = ss.Ts;

        // Capture operating point for the closure
        const double x1_op = op.x1, x2_op = op.x2, x3_op = op.x3;
        const double u1_op = op.u1, u2_op = op.u2, u3_op = op.u3;
        const double Ts    = ss.Ts;

        // Nonlinear discrete dynamics in deviation space (Euler integration, Ts=1s)
        auto f_dev = [x1_op, x2_op, x3_op, u1_op, u2_op, u3_op, Ts]
                     (const Eigen::VectorXd& x_dev, const Eigen::VectorXd& u_dev)
                     -> Eigen::VectorXd {
            double x1 = x1_op + x_dev(0);
            double x2 = x2_op + x_dev(1);
            double x3 = x3_op + x_dev(2);
            double u1 = std::clamp(u1_op + u_dev(0), 0.0, 1.0);
            double u2 = std::clamp(u2_op + u_dev(1), 0.0, 1.0);
            double u3 = std::clamp(u3_op + u_dev(2), 0.0, 1.0);

            double x1_98 = std::pow(std::max(x1, 1.0), 9.0 / 8.0);
            double dx1 = -0.0018 * u2 * x1_98 + 0.9 * u1 - 0.15 * u3;
            double dx2 = (0.073 * u2 - 0.016) * x1_98 - 0.1 * x2;
            double dx3 = (141.0 * u3 - (1.1 * u2 - 0.19) * x1) / 85.0;

            Eigen::VectorXd x_next(3);
            x_next(0) = x_dev(0) + Ts * dx1;
            x_next(1) = x_dev(1) + Ts * dx2;
            x_next(2) = x_dev(2) + Ts * dx3;
            return x_next;
        };

        // Use ss.C as output matrix (maps [dx1,dx2,dx3] to [dy1,dy2,dy3])
        return ctrl::NonlinearMPC(np, f_dev, ss.C);
    }())
    , op_(op)
{}

Vector3d NMPCBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // Use dy as state proxy (Cd approx = I near operating point)
    Eigen::VectorXd x_dev(3); x_dev = dy;
    Eigen::VectorXd y_ref(3); y_ref = ref_dy;

    Eigen::VectorXd du = nmpc_.computeRef(x_dev, y_ref);
    return du.head<3>();
}

void NMPCBoilerCtrl::reset()
{
    nmpc_.reset();
}

// ============================================================================
// 23. FLBoilerCtrl
// ============================================================================

// Build one FeedbackLinearisationController for a diagonal boiler channel.
// Channel 0: u1->y1=x1  (g=0.9, nonlinear f)
// Channel 1: u2->y2=x2  (g=0.073*x1^(9/8), nonlinear f)
// Channel 2: u3->x3      (g=141/85, linear f for water level)
static ctrl::FeedbackLinearisationController makeFLForChannel(
    int ch, const OperatingPoint& op, double Ts)
{
    const double x1_op = op.x1, x2_op = op.x2, x3_op = op.x3;
    const double u1_op = op.u1, u2_op = op.u2, u3_op = op.u3;

    ctrl::FeedbackLinearisationController::DriftFn f_fn;
    ctrl::FeedbackLinearisationController::GainFn  g_fn;

    if (ch == 0) {
        // y1 = x1;  dy1/dt = -0.0018*u2*x1^(9/8) + 0.9*u1 - 0.15*u3
        // In deviation: f0(x_dev) = drift at u1=u1_op
        f_fn = [x1_op, u1_op, u2_op, u3_op](const Eigen::VectorXd& x_dev, double) {
            double x1 = x1_op + x_dev(0);
            double x1_98 = std::pow(std::max(x1, 1.0), 9.0 / 8.0);
            return -0.0018 * u2_op * x1_98 + 0.9 * u1_op - 0.15 * u3_op;
        };
        g_fn = [](const Eigen::VectorXd&, double) { return 0.9; };

    } else if (ch == 1) {
        // y2 = x2;  dy2/dt = (0.073*u2 - 0.016)*x1^(9/8) - 0.1*x2
        f_fn = [x1_op, x2_op, u2_op](const Eigen::VectorXd& x_dev, double) {
            double x1 = x1_op + x_dev(0);
            double x2 = x2_op + x_dev(1);
            double x1_98 = std::pow(std::max(x1, 1.0), 9.0 / 8.0);
            return (0.073 * u2_op - 0.016) * x1_98 - 0.1 * x2;
        };
        g_fn = [x1_op](const Eigen::VectorXd& x_dev, double) {
            double x1 = x1_op + x_dev(0);
            return 0.073 * std::pow(std::max(x1, 1.0), 9.0 / 8.0);
        };

    } else {
        // x3 water level proxy for y3 (channel 2)
        // dx3/dt = (141*u3 - (1.1*u2_op - 0.19)*x1) / 85
        f_fn = [x1_op, u3_op, u2_op](const Eigen::VectorXd& x_dev, double) {
            double x1 = x1_op + x_dev(0);
            return (141.0 * u3_op - (1.1 * u2_op - 0.19) * x1) / 85.0;
        };
        g_fn = [](const Eigen::VectorXd&, double) { return 141.0 / 85.0; };
    }

    // Inner PID: proportional control on the linearized virtual plant
    ctrl::PIDParams pp;
    pp.Kp   = 0.5;
    pp.Ki   = 0.005;
    pp.Kd   = 0.0;
    pp.N    = 5.0;
    pp.Kb   = 1.0;
    pp.uMin = -2.0;
    pp.uMax =  2.0;
    auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::FLParams flp;
    flp.uMin              = -0.5;
    flp.uMax              =  0.5;
    flp.regularisationEps = 1e-6;

    return ctrl::FeedbackLinearisationController(f_fn, g_fn, inner, flp, Ts);
}

FLBoilerCtrl::FLBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : fls_{ makeFLForChannel(0, op, ss.Ts),
            makeFLForChannel(1, op, ss.Ts),
            makeFLForChannel(2, op, ss.Ts) }
    , op_(op)
{}

Vector3d FLBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Eigen::VectorXd x_dev(3); x_dev = dy;
    for (auto& fl : fls_) fl.setState(x_dev);

    Vector3d du;
    for (int i = 0; i < 3; ++i)
        du(i) = fls_[i].compute(ref_dy(i) - dy(i));
    return du;
}

void FLBoilerCtrl::reset()
{
    for (auto& fl : fls_) fl.reset();
}

// ============================================================================
// 24. MHELQRBoilerCtrl
// ============================================================================

MHELQRBoilerCtrl::MHELQRBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : mhe_([&]() {
        ctrl::MHEParams mp;
        mp.N        = 10;
        mp.wMin     = -0.5;
        mp.wMax     =  0.5;
        mp.qpMaxIter = 300;
        return ctrl::MovingHorizonEstimator(
            ss,
            1e-3 * Eigen::Matrix3d::Identity(),
            (Eigen::Matrix3d() << 0.25, 0, 0, 0, 1.0, 0, 0, 0, 25.0).finished(),
            mp);
    }())
    , lqr_(ss, brysonLQRParams())
    , du_prev_(Vector3d::Zero())
{
    (void)op;
}

Vector3d MHELQRBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    if (!initialised_) {
        mhe_.initialize(dy.cast<double>(),
                        1e-3 * Eigen::Matrix3d::Identity());
        initialised_ = true;
    }

    Eigen::VectorXd x_est = mhe_.estimate(dy.cast<double>(),
                                           du_prev_.cast<double>());
    Eigen::VectorXd du = lqr_.compute(x_est, ref_dy.cast<double>());
    du_prev_ = du.head<3>();
    return du.head<3>();
}

void MHELQRBoilerCtrl::reset()
{
    mhe_.reset();
    du_prev_.setZero();
    initialised_ = false;
}

// ============================================================================
// 25. LPVGSBoilerCtrl
// ============================================================================

LPVGSBoilerCtrl::LPVGSBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : op_(op)
{
    // Sweep pressure x1 \in [60, 150] bar; linearise at each point.
    // Design DiscreteLQR per linearisation; assemble GainScheduledController.
    const double p_min = 60.0, p_max = 150.0;
    const int    n_pts = 6;

    sched_ = std::make_shared<ctrl::GainScheduledController>(
        ss.Ts, ctrl::GainScheduleMode::NearestNeighbor);

    for (int k = 0; k < n_pts; ++k) {
        double x1_k = p_min + k * (p_max - p_min) / (n_pts - 1);

        // Build operating point at this pressure
        OperatingPoint op_k = op;
        op_k.x1 = x1_k;

        try {
            ctrl::StateSpace ss_k = linearize(op_k, ss.Ts);
            ctrl::DiscreteLQR lqr_k(ss_k, brysonLQRParams());
            // Wrap LQR gain in PID (DiscreteLQR doesn't implement IController)
            ctrl::PIDParams pp;
            pp.Kp   = lqr_k.gainMatrix()(0, 0);  // channel-0 gain as proxy
            pp.Ki   = pp.Kp * 0.05;
            pp.Kd   = 0.0;
            pp.N    = 5.0;
            pp.Kb   = 1.0;
            pp.uMin = -0.5;
            pp.uMax =  0.5;
            sched_->addSchedulePoint(x1_k,
                std::make_shared<ctrl::DiscretePID>(pp, ss.Ts));
        } catch (const std::exception&) {
            // If linearization fails at this point, skip it
        }
    }

    // Ensure at least one point exists (fallback PID)
    if (sched_->numPoints() == 0) {
        sched_->addSchedulePoint((p_min + p_max) / 2.0,
            std::make_shared<ctrl::DiscretePID>(pidParamsFor(0), ss.Ts));
    }
}

Vector3d LPVGSBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // Schedule on current pressure deviation from op + op.x1
    double x1_current = op_.x1 + dy(0);
    sched_->setSchedulingParam(x1_current);

    // The DiscreteLQR inside the scheduler uses IController::compute(error).
    // For MIMO: it returns u[0] only. We call per-channel as proxy.
    Vector3d e = ref_dy - dy;
    Vector3d du;
    for (int i = 0; i < 3; ++i)
        du(i) = sched_->compute(e(i));
    return du;
}

void LPVGSBoilerCtrl::reset()
{
    if (sched_) sched_->reset();
}

// ============================================================================
// 26. SubspaceIDLQGBoilerCtrl
// ============================================================================

SubspaceIDLQGBoilerCtrl::SubspaceIDLQGBoilerCtrl(const ctrl::StateSpace& ss,
                                                   const OperatingPoint& op)
    : lqg_([&]() {
        // Generate simulated PRBS I/O data for SubspaceID identification.
        const int N_id = 300;
        Eigen::MatrixXd U(3, N_id), Y(3, N_id);

        Eigen::VectorXd x_sim = Eigen::VectorXd::Zero(3);
        for (int k = 0; k < N_id; ++k) {
            double t = k * ss.Ts;
            Eigen::VectorXd u_id(3);
            u_id << 0.05 * std::sin(0.10 * t),
                    0.05 * std::sin(0.13 * t + 0.5),
                    0.02 * std::sin(0.07 * t + 1.0);
            U.col(k) = u_id;
            Y.col(k) = ss.C * x_sim + ss.D * u_id;
            x_sim    = ss.A * x_sim + ss.B * u_id;
        }

        // n4sid: identify 3rd-order model, horizon i=15
        ctrl::SubspaceIDResult res = ctrl::n4sid(Y, U, 3, 15, ss.Ts);

        // Use identified model if successful, fall back to linearised ss
        ctrl::StateSpace ss_id = (res.success && res.model.has_value())
                                     ? res.model.value()
                                     : ss;

        ctrl::LQRParams lp = brysonLQRParams();
        Eigen::MatrixXd Q_n = 1e-3 * Eigen::MatrixXd::Identity(ss_id.A.rows(), ss_id.A.rows());
        Eigen::MatrixXd R_n = (Eigen::Matrix3d() << 0.25, 0, 0,
                                                      0,  1.0, 0,
                                                      0,  0, 25.0).finished();
        return ctrl::DiscreteLQG(ss_id, lp, Q_n, R_n);
    }())
    , du_prev_(Vector3d::Zero())
{
    (void)op;
}

Vector3d SubspaceIDLQGBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Eigen::VectorXd x_ref(3); x_ref = ref_dy.cast<double>();
    Eigen::VectorXd y(3);     y     = dy.cast<double>();

    Eigen::VectorXd u = lqg_.step(y, du_prev_.cast<double>(), x_ref);
    du_prev_ = u.head<3>();
    return u.head<3>();
}

void SubspaceIDLQGBoilerCtrl::reset()
{
    lqg_.reset();
    du_prev_.setZero();
}

// ============================================================================
// 27. AutoGSBoilerCtrl
// ============================================================================

AutoGSBoilerCtrl::AutoGSBoilerCtrl(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : op_(op)
{
    // Boiler channel 0 (pressure x1) nonlinear dynamics for AutoGS.
    // State: 1D [dx1], input: 1D [du1], scheduling: x1 = x1_op + p_sched.
    // Continuous: dx1/dt = -0.0018*u2_op*(x1_op+p)^(9/8) + 0.9*(u1_op+du1) - 0.15*u3_op
    //           = f_drift(p) + 0.9*du1
    // This is a 1D SISO affine system. Schedule on x1 = x1_op + p, p \in [-30, 40] bar.

    const double x1_op = op.x1, u1_op = op.u1, u2_op = op.u2, u3_op = op.u3;

    auto f_cont = [x1_op, u1_op, u2_op, u3_op](const Eigen::VectorXd& xs,
                                                  const Eigen::VectorXd& u_in)
                  -> Eigen::VectorXd {
        double x1 = x1_op + xs(0);
        double x1_98 = std::pow(std::max(x1, 1.0), 9.0 / 8.0);
        Eigen::VectorXd xdot(1);
        xdot(0) = -0.0018 * u2_op * x1_98 + 0.9 * (u1_op + u_in(0)) - 0.15 * u3_op;
        return xdot;
    };

    auto u_eq_fn = [x1_op, u1_op, u2_op, u3_op](double p_sched)
                   -> Eigen::VectorXd {
        // At equilibrium: f_cont([p_sched], [0]) = 0 -> solve for du1
        double x1_98 = std::pow(std::max(x1_op + p_sched, 1.0), 9.0 / 8.0);
        double du1_eq = (0.0018 * u2_op * x1_98 + 0.15 * u3_op - 0.9 * u1_op) / 0.9;
        Eigen::VectorXd u(1);
        u(0) = std::clamp(du1_eq, -0.5, 0.5);
        return u;
    };

    auto x0_fn = [](double p_sched) -> Eigen::VectorXd {
        Eigen::VectorXd x(1);
        x(0) = p_sched;   // pressure deviation = scheduling param
        return x;
    };

    auto design_fn = [](const ctrl::StateSpace& sys_k, double /*p*/)
                     -> std::shared_ptr<ctrl::IController> {
        ctrl::LQRParams lp;
        Eigen::VectorXd xm(1), um(1);
        xm << 5.0;    // max pressure deviation 5 bar
        um << 0.3;    // max valve increment 0.3
        lp = ctrl::LQRWeightTuner::brysonMethod(xm, um);
        ctrl::DiscreteLQR lqr(sys_k, lp);

        // DiscreteLQR doesn't implement IController; extract gain into PID
        ctrl::PIDParams pp;
        pp.Kp   = lqr.gainMatrix()(0, 0);
        pp.Ki   = pp.Kp * 0.01;
        pp.Kd   = 0.0;
        pp.N    = 5.0;
        pp.Kb   = 1.0;
        pp.uMin = -0.5;
        pp.uMax =  0.5;
        return std::make_shared<ctrl::DiscretePID>(pp, sys_k.Ts);
    };

    try {
        sched_ = ctrl::buildAutoGainScheduler(
            f_cont, -30.0, 40.0, 6, u_eq_fn, x0_fn, design_fn, ss.Ts,
            /*gap_threshold=*/0.4, /*freq_points=*/80);
    } catch (const std::exception&) {
        // Fallback to single-point PID-like controller
        sched_ = std::make_shared<ctrl::GainScheduledController>(
            ss.Ts, ctrl::GainScheduleMode::NearestNeighbor);
        sched_->addSchedulePoint(0.0,
            std::make_shared<ctrl::DiscretePID>(pidParamsFor(0), ss.Ts));
    }
}

Vector3d AutoGSBoilerCtrl::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    // Schedule on current pressure deviation (channel 0)
    sched_->setSchedulingParam(dy(0));

    Vector3d e = ref_dy - dy;
    Vector3d du;
    // AutoGS only controls channel 0 (pressure); channels 1,2 use PID gains
    du(0) = sched_->compute(e(0));
    // Channels 1,2: simple proportional fallback
    du(1) = 0.05 * e(1);
    du(2) = 0.05 * e(2);
    return du;
}

void AutoGSBoilerCtrl::reset()
{
    if (sched_) sched_->reset();
}

} // namespace boiler
