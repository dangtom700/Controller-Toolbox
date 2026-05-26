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
        mp.Np   = std::min(rec.Np, 20);
        mp.Nc   = std::min(rec.Nc,  5);
        mp.rho_y = rec.rho_y;
        mp.rho_u = rec.rho_u;
        mp.uMin  = -0.5;
        mp.uMax  =  0.5;
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

static ctrl::SMCParams smcParams()
{
    ctrl::SMCParams p;
    p.c_e  = 1.0;
    p.c_de = 0.2;
    p.K    = 0.05;
    p.phi  = 0.3;
    p.uMin = -0.5;
    p.uMax =  0.5;
    return p;
}

SMCController::SMCController(const ctrl::StateSpace& ss, const OperatingPoint& op)
    : smcs_{ ctrl::DiscreteSMC(smcParams(), ss.Ts),
             ctrl::DiscreteSMC(smcParams(), ss.Ts),
             ctrl::DiscreteSMC(smcParams(), ss.Ts) }
{
    (void)op;
}

Vector3d SMCController::compute(const Vector3d& ref_dy, const Vector3d& dy)
{
    Vector3d e = ref_dy - dy;
    return Vector3d(
        smcs_[0].compute(e(0)),
        smcs_[1].compute(e(1)),
        smcs_[2].compute(e(2)));
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

        ctrl::ADRCParams p0; p0.omega_o=0.10; p0.omega_c=0.02; p0.b0=0.9*Ts;       p0.uMin=-0.5; p0.uMax=0.5;
        ctrl::ADRCParams p1; p1.omega_o=0.05; p1.omega_c=0.01; p1.b0=0.073*x1_98*Ts; p1.uMin=-0.5; p1.uMax=0.5;
        ctrl::ADRCParams p2; p2.omega_o=0.05; p2.omega_c=0.01; p2.b0=(141.0/85.0)*Ts; p2.uMin=-0.5; p2.uMax=0.5;

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
        gp.Np    = 10;
        gp.Nu    = 3;
        gp.rho_y = 1.0;
        gp.rho_u = 0.1;
        gp.alpha = 0.1;
        gp.uMin  = -0.5;
        gp.uMax  =  0.5;
        auto ch0 = diagonalChannel(ss, 0);
        auto ch1 = diagonalChannel(ss, 1);
        auto ch2 = diagonalChannel(ss, 2);
        return std::array<ctrl::GeneralizedPredictiveController, 3>{
            ctrl::GeneralizedPredictiveController(ch0, gp),
            ctrl::GeneralizedPredictiveController(ch1, gp),
            ctrl::GeneralizedPredictiveController(ch2, gp)
        };
    }())
    , rls_{ ctrl::RecursiveLeastSquares(2, 2, ss.Ts, 0.98),
            ctrl::RecursiveLeastSquares(2, 2, ss.Ts, 0.98),
            ctrl::RecursiveLeastSquares(2, 2, ss.Ts, 0.98) }
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

        // Hot-swap GPC plant every kRLSUpdateInterval steps after warmup
        if (step_count_ >= kRLSWarmup &&
            step_count_ % kRLSUpdateInterval == 0) {
            gpcs_[i].setPlant(rls_[i].toStateSpace());
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
        return ctrl::UnscentedKalmanFilter(3, 3, boilerF, boilerH, Qn, Rn, ss.Ts);
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
        mp.Np   = std::min(rec.Np, 20);
        mp.Nc   = std::min(rec.Nc,  5);
        mp.rho_y = rec.rho_y;
        mp.rho_u = rec.rho_u;
        mp.uMin  = -0.5;
        mp.uMax  =  0.5;
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

            auto smc_ctrl = std::make_shared<ctrl::DiscreteSMC>(smcParams(), ss.Ts);
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

} // namespace boiler
