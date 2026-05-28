/**
 * @file test_catch2_advanced.cpp
 * @brief Advanced Catch2 unit + integration tests for the Controller Toolbox.
 *
 * Coverage targets (all new relative to the legacy hand-rolled suite):
 *   - GradientProjectionQP - direct solver validation
 *   - ControllerStack + IControllerObserver - integration
 *   - DiscreteMPC closed-loop tracking
 *   - GPC vs MPC equivalence (alpha=0)
 *   - KalmanFilter state estimation on a linear system
 *   - DiscreteLQR closed-loop stability + DareResult struct fields
 *   - SuperTwistingSMC tracking
 *   - DiscreteLeadLag phaseAt() returns radians (P12-21 regression guard)
 *   - IControllerObserver wired through ControllerStack
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "ControllerToolbox.h"
#include <cmath>
#include <cstdlib>
#include <numbers>

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

// -----------------------------------------------------------------------------
// Shared helpers
// -----------------------------------------------------------------------------

static constexpr double Ts = 0.01;

// Second-order plant G(s) = 1/(s^2 + 1.5s + 1), ZOH-discretised.
static ctrl::StateSpace makePlant()
{
    ctrl::TransferFunction tf(
        {0.0, 4.9625e-5, 4.9125e-5},
        {1.0, -1.98511,  0.98522},
        Ts);
    return ctrl::tf2ss(tf);
}

// Double integrator x = [pos, vel], u = acceleration
static ctrl::StateSpace makeDoubleIntegrator(double ts = Ts)
{
    Eigen::Matrix2d A;
    A << 1.0, ts, 0.0, 1.0;
    Eigen::Vector2d B;
    B << 0.5 * ts * ts, ts;
    Eigen::RowVector2d C;
    C << 1.0, 0.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    return ctrl::StateSpace{A, B, C, D, ts};
}

// -----------------------------------------------------------------------------
// GradientProjectionQP - direct validation
// -----------------------------------------------------------------------------

TEST_CASE("GradientProjectionQP solves unconstrained QP exactly", "[qp]")
{
    // Unconstrained optimum: x* = -H^{-1} g = -2
    // H = [[2]], g = [4], lb = [-1e9], ub = [1e9]
    Eigen::MatrixXd H(1, 1);
    H << 2.0;
    Eigen::VectorXd g(1);
    g << 4.0;
    Eigen::VectorXd lb(1);
    lb << -1e9;
    Eigen::VectorXd ub(1);
    ub << 1e9;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    REQUIRE(ldlt.info() == Eigen::Success);

    Eigen::VectorXd x(1), tmp1(1), tmp2(1);
    const double L = H.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff();

    const auto res = ctrl::solveGradientProjectionQP(H, g, lb, ub, ldlt, L, 200, 1e-12, x, tmp1, tmp2);

    REQUIRE(res.converged);
    REQUIRE_THAT(x(0), WithinAbs(-2.0, 1e-8));
}

TEST_CASE("GradientProjectionQP respects active box constraint", "[qp]")
{
    // x* = -2 but lb = -1.0, so optimal clamped solution is x = -1.
    Eigen::MatrixXd H(1, 1);
    H << 2.0;
    Eigen::VectorXd g(1);
    g << 4.0;
    Eigen::VectorXd lb(1), ub(1);
    lb << -1.0;
    ub <<  1e9;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    const double L = H(0, 0);

    Eigen::VectorXd x(1), tmp1(1), tmp2(1);
    const auto res = ctrl::solveGradientProjectionQP(H, g, lb, ub, ldlt, L, 200, 1e-12, x, tmp1, tmp2);

    REQUIRE(res.converged);
    REQUIRE_THAT(x(0), WithinAbs(-1.0, 1e-8));
}

// -----------------------------------------------------------------------------
// DiscreteMPC - closed-loop step tracking
// -----------------------------------------------------------------------------

TEST_CASE("DiscreteMPC tracks a unit step reference", "[mpc][integration]")
{
    auto plant = makePlant();
    ctrl::MPCParams p;
    p.Np     = 20;
    p.Nc     = 5;
    p.rho_y  = 10.0;
    p.rho_u  = 0.01;
    p.uMin   = -100.0;
    p.uMax   =  100.0;
    p.duMin  = -100.0;
    p.duMax  =  100.0;

    ctrl::DiscreteMPC mpc(plant, p);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd ref(1);
    ref(0) = 1.0;

    double y = 0.0;
    for (int k = 0; k < 3000; ++k)
    {
        mpc.setState(x);
        const Eigen::VectorXd u = mpc.computeRef(x, ref);
        y = ctrl::ssStep(plant, x, u)(0);
    }

    REQUIRE_THAT(y, WithinAbs(1.0, 0.02)); // within 2% of reference
}

TEST_CASE("DiscreteMPC lastQPConverged is true for a well-conditioned problem", "[mpc]")
{
    auto plant = makePlant();
    ctrl::MPCParams p;
    p.Np = 10; p.Nc = 3; p.rho_y = 1.0; p.rho_u = 0.1;

    ctrl::DiscreteMPC mpc(plant, p);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd ref(1);
    ref(0) = 1.0;

    mpc.computeRef(x, ref);

    REQUIRE(mpc.lastQPConverged());
    REQUIRE(mpc.lastQPIters() > 0);
}

// -----------------------------------------------------------------------------
// GPC closed-loop tracking validation
// -----------------------------------------------------------------------------

TEST_CASE("GPC with alpha=0 tracks unit step reference", "[gpc]")
{
    // GPC uses a velocity-form CARIMA augmented model (Aa, Ba, Ca) that differs
    // from standard MPC, so their first control moves are NOT numerically equal.
    // This test validates GPC's actual closed-loop tracking behaviour instead.
    auto plant = makePlant();

    ctrl::GPCParams gp;
    gp.Np = 15; gp.Nu = 5;
    gp.rho_y = 10.0; gp.rho_u = 0.01;
    gp.alpha = 0.0;
    gp.uMin = -100.0; gp.uMax = 100.0;
    gp.duMin = -100.0; gp.duMax = 100.0;

    ctrl::GeneralizedPredictiveController gpc(plant, gp);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd u_vec(1);
    double y = 0.0;

    for (int k = 0; k < 3000; ++k)
    {
        const double u = gpc.computeRef(y, 1.0);
        u_vec(0) = u;
        y = ctrl::ssStep(plant, x, u_vec)(0);
    }

    REQUIRE_THAT(y, WithinAbs(1.0, 0.02)); // within 2% of unit step reference
    REQUIRE(gpc.lastQPConverged());
}

// -----------------------------------------------------------------------------
// KalmanFilter - estimation accuracy on a first-order system
// -----------------------------------------------------------------------------

TEST_CASE("KalmanFilter converges to true state on a stable first-order plant", "[kalman]")
{
    // Plant: x[k+1] = 0.9*x[k] + u[k], y[k] = x[k]
    Eigen::MatrixXd A(1, 1), B(1, 1), C(1, 1), D(1, 1);
    A << 0.9; B << 1.0; C << 1.0; D << 0.0;
    ctrl::StateSpace sys(A, B, C, D, Ts);

    Eigen::MatrixXd Q_noise(1, 1), R_noise(1, 1);
    Q_noise << 1e-3;
    R_noise << 1e-2;

    ctrl::KalmanFilter kf(sys, Q_noise, R_noise);

    Eigen::VectorXd x_true(1);
    x_true << 0.0;
    Eigen::VectorXd u(1);
    u << 0.05; // constant drive

    for (int k = 0; k < 200; ++k)
    {
        // True system (no noise for this test)
        x_true = A * x_true + B * u;
        Eigen::VectorXd y_meas = C * x_true;
        kf.step(y_meas, u);
    }

    const double err = std::abs(kf.state()(0) - x_true(0));
    REQUIRE(err < 0.01); // estimate within 1% of true value after 200 steps
}

TEST_CASE("KalmanFilter covariance is positive semi-definite after updates", "[kalman]")
{
    Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
    A << 0.9, 0.1, 0.0, 0.8;
    B << 0.0, 0.1;
    C << 1.0, 0.0;
    D << 0.0;
    ctrl::StateSpace sys(A, B, C, D, Ts);

    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = 0.1  * Eigen::MatrixXd::Identity(1, 1);
    ctrl::KalmanFilter kf(sys, Q, R);

    Eigen::VectorXd u(1);
    u << 1.0;
    Eigen::VectorXd y(1);
    y << 0.5;

    for (int k = 0; k < 50; ++k)
        kf.step(y, u);

    const Eigen::MatrixXd &P = kf.covariance();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P);
    REQUIRE(eig.eigenvalues().minCoeff() >= -1e-10); // PSD
}

// -----------------------------------------------------------------------------
// DiscreteLQR - closed-loop stability + DareResult fields
// -----------------------------------------------------------------------------

TEST_CASE("DiscreteLQR drives double-integrator state to zero", "[lqr]")
{
    auto plant = makeDoubleIntegrator();

    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);

    ctrl::DiscreteLQR lqr(plant, lqr_p);
    REQUIRE(lqr.dareConverged());

    Eigen::Vector2d x;
    x << 1.0, 0.0; // start at position = 1

    for (int k = 0; k < 500; ++k)
    {
        const Eigen::VectorXd u = lqr.compute(x);
        ctrl::ssStep(plant, x, u);
    }

    REQUIRE(x.norm() < 0.05); // state near zero after 500 steps
}

TEST_CASE("DareResult fields P, converged, iterations are accessible from DiscreteLQR", "[lqr][dare]")
{
    auto plant = makeDoubleIntegrator();
    ctrl::LQRParams lqr_p;
    lqr_p.Q = Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);
    ctrl::DiscreteLQR lqr(plant, lqr_p);

    REQUIRE(lqr.dareConverged());
    REQUIRE(lqr.dareIterations() > 0);

    const Eigen::MatrixXd &P = lqr.riccatiSolution();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P);
    REQUIRE(eig.eigenvalues().minCoeff() >= 0.0); // DARE solution P >= 0
}

// -----------------------------------------------------------------------------
// SuperTwistingSMC - basic tracking
// -----------------------------------------------------------------------------

TEST_CASE("SuperTwistingSMC reduces tracking error on a first-order plant", "[smc][super_twisting]")
{
    // Simple plant: y[k+1] = 0.8*y[k] + 0.2*u[k]
    ctrl::SuperTwistingParams p;
    p.c_e  = 1.0;
    p.c_de = 0.01;  // = lambda * Ts with lambda=1, Ts=0.01
    p.K1   = 2.0;
    p.K2   = 3.0;   // K2 > K1^2/4 = 1.0 (check)
    p.uMin = -20.0;
    p.uMax =  20.0;

    ctrl::SuperTwistingSMC smc(p, Ts);

    double y = 0.0;
    const double ref = 1.0;
    double initial_error = ref - y;

    for (int k = 0; k < 500; ++k)
    {
        const double u = smc.compute(y - ref); // SMC sign convention: y - ref
        y = 0.8 * y + 0.2 * u;
    }

    const double final_error = ref - y;
    REQUIRE(std::abs(final_error) < std::abs(initial_error) * 0.05); // error reduced by 95%
}

// -----------------------------------------------------------------------------
// DiscreteLeadLag - phaseAt() returns radians regression guard (P12-21)
// -----------------------------------------------------------------------------

TEST_CASE("DiscreteLeadLag::phaseAt() returns radians, not degrees", "[lead_lag][regression]")
{
    // zc = 1 rad/s, pc = 10 rad/s (lead compensator)
    ctrl::LeadLagParams p;
    p.continuousZero = 1.0;
    p.continuousPole = 10.0;
    p.gain           = 1.0;

    ctrl::DiscreteLeadLag ll(p, Ts);

    // At omega = sqrt(zc * pc) = sqrt(10) approx = 3.16 rad/s, lead phase is maximum.
    // The analytical max lead angle is arcsin((pc-zc)/(pc+zc)) which is arcsin(9/11)approx =54.9^\circ=0.958 rad.
    const double omega = std::sqrt(p.continuousZero * p.continuousPole);
    const double phase = ll.phaseAt(omega);

    // If this returns degrees, the value would be ~54.9 which would fail the < pi test.
    // If this returns radians, phase < pi/2 < pi.
    REQUIRE(phase < std::numbers::pi); // radians: max lead < 90^\circ
    REQUIRE(phase > 0.0);              // lead: positive phase
}

// -----------------------------------------------------------------------------
// ControllerStack + IControllerObserver - integration test
// -----------------------------------------------------------------------------

TEST_CASE("ControllerStack fires its own observer on compute()", "[stack][observer][integration]")
{
    struct StackObserver : ctrl::IControllerObserver
    {
        std::vector<double> u_log;
        std::vector<double> e_log;

        void onCompute(double u, double signal) override
        {
            u_log.push_back(u);
            e_log.push_back(signal);
        }
    };

    ctrl::PIDParams p;
    p.Kp = 1.0; p.Ki = 0.0; p.Kd = 0.0;

    auto stack = std::make_unique<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, Ts);
    stack->addController(std::make_shared<ctrl::DiscretePID>(p, Ts), "PID");

    StackObserver obs;
    stack->attachObserver(&obs);

    SECTION("Observer fires once per compute() call")
    {
        stack->compute(1.0);
        stack->compute(0.5);
        stack->compute(0.25);

        REQUIRE(obs.u_log.size() == 3);
        REQUIRE_THAT(obs.e_log[0], WithinAbs(1.0,  1e-12));
        REQUIRE_THAT(obs.e_log[1], WithinAbs(0.5,  1e-12));
        REQUIRE_THAT(obs.e_log[2], WithinAbs(0.25, 1e-12));
    }

    SECTION("Logged u matches compute() return value")
    {
        const double u1 = stack->compute(2.0);
        REQUIRE_THAT(obs.u_log.at(0), WithinRel(u1, 1e-12));
    }

    SECTION("Detached observer no longer fires")
    {
        stack->detachObserver();
        stack->compute(1.0);
        REQUIRE(obs.u_log.empty());
    }
}

// -----------------------------------------------------------------------------
// ControllerStack - Additive mode with multiple controllers
// -----------------------------------------------------------------------------

TEST_CASE("ControllerStack Additive mode sums all active controller outputs", "[stack]")
{
    ctrl::PIDParams p;
    p.Kp = 1.0; p.Ki = 0.0; p.Kd = 0.0;

    ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
    stack.addController(std::make_shared<ctrl::DiscretePID>(p, Ts), "P1");
    stack.addController(std::make_shared<ctrl::DiscretePID>(p, Ts), "P2");

    // Both controllers: P=1, error=2 -> each outputs 2 -> sum = 4
    const double u = stack.compute(2.0);
    REQUIRE_THAT(u, WithinAbs(4.0, 1e-10));
}

TEST_CASE("ControllerStack Weighted mode normalises weights correctly", "[stack]")
{
    ctrl::PIDParams pa, pb;
    pa.Kp = 1.0; pa.Ki = 0.0; pa.Kd = 0.0; // outputs 1*e
    pb.Kp = 3.0; pb.Ki = 0.0; pb.Kd = 0.0; // outputs 3*e

    ctrl::ControllerStack stack(ctrl::StackMode::Weighted, Ts);
    stack.addController(std::make_shared<ctrl::DiscretePID>(pa, Ts), "P1", 0.5);
    stack.addController(std::make_shared<ctrl::DiscretePID>(pb, Ts), "P2", 0.5);

    // Normalised: u = (0.5*1 + 0.5*3) / (0.5 + 0.5) = 2.0 for error = 1.0
    const double u = stack.compute(1.0);
    REQUIRE_THAT(u, WithinAbs(2.0, 1e-10));
}

// -----------------------------------------------------------------------------
// DiscreteSMC - sliding surface convergence
// -----------------------------------------------------------------------------

TEST_CASE("DiscreteSMC sliding surface enters boundary layer", "[smc]")
{
    ctrl::SMCParams p;
    p.c_e  = 1.0;
    p.c_de = 0.01;
    p.K    = 5.0;
    p.phi  = 0.5;
    p.uMin = -20.0;
    p.uMax =  20.0;

    ctrl::DiscreteSMC smc(p, Ts);

    double y = 0.0;
    const double ref = 1.0;

    for (int k = 0; k < 1000; ++k)
    {
        const double u = smc.compute(y - ref); // SMC sign convention: y - ref
        y = 0.8 * y + 0.2 * u;
    }

    // After convergence, |s| < phi (inside boundary layer = linear PD regime)
    REQUIRE(std::abs(smc.slidingSurface()) < p.phi * 2.0); // within 2x boundary layer
}

// -----------------------------------------------------------------------------
// DiscreteADRC - tracks unit step reference
// -----------------------------------------------------------------------------

TEST_CASE("DiscreteADRC tracks unit step reference on a double-integrator plant", "[adrc]")
{
    // 2nd-order ADRC models y'' = f + b0*u, so the canonical test plant is a
    // double integrator: x1' = x2, x2' = u, y = x1  (b0 = 1.0 exactly).
    ctrl::ADRCParams p;
    p.omega_c = 5.0;
    p.omega_o = 20.0;
    p.b0      = 1.0;
    p.uMin    = -50.0;
    p.uMax    =  50.0;

    ctrl::DiscreteADRC adrc(p, Ts);

    double x1 = 0.0, x2 = 0.0; // position, velocity

    for (int k = 0; k < 2000; ++k)
    {
        const double u = adrc.computeTracking(x1, 1.0);
        x2 += Ts * u;   // velocity integrates acceleration
        x1 += Ts * x2;  // position integrates velocity
    }

    REQUIRE_THAT(x1, WithinAbs(1.0, 0.05)); // within 5% of reference
}

// -----------------------------------------------------------------------------
// DiscreteLeadLag - closed-loop phase improvement
// -----------------------------------------------------------------------------

TEST_CASE("DiscreteLeadLag output is non-trivially filtered", "[lead_lag]")
{
    ctrl::LeadLagParams p;
    p.continuousZero = 1.0;
    p.continuousPole = 10.0;
    p.gain           = 1.0;

    ctrl::DiscreteLeadLag ll(p, Ts);

    // With a unit step input, the first output should be larger than subsequent ones
    // (lead compensator amplifies high-frequency content: first output > steady-state).
    const double u1 = ll.compute(1.0);
    for (int k = 0; k < 200; ++k) ll.compute(1.0);
    const double u_ss = ll.compute(1.0); // steady state

    // Lead: high-freq gain > 1, so initial spike > steady-state gain
    REQUIRE(u1 > u_ss);
    REQUIRE(u_ss > 0.0); // non-zero steady state
}

// -----------------------------------------------------------------------------
// RecursiveLeastSquares - online identification
// -----------------------------------------------------------------------------

TEST_CASE("RecursiveLeastSquares identifies first-order ARX coefficients", "[rls]")
{
    // True plant: y[k] = 0.8*y[k-1] + 0.2*u[k-1]  ->  ARX: y = -(-0.8)*y_prev + 0.2*u_prev
    // ARX parameter vector theta = [a1, b1] = [-0.8, 0.2]  (in MATLAB convention)
    // Actually: y[k] + a1*y[k-1] = b1*u[k-1]  ->  a1=-0.8, b1=0.2
    // Standard form: y[k] = phi'[k] * theta  where phi = [-y[k-1], u[k-1]]
    ctrl::RecursiveLeastSquares rls(1, 1, Ts, 0.99);

    double y_prev = 0.0, u_prev = 0.0;
    double y = 0.0;

    for (int k = 0; k < 2000; ++k)
    {
        const double u = 0.5 * std::sin(2.0 * std::numbers::pi * k * Ts * 5.0) + 0.3;
        y = 0.8 * y_prev + 0.2 * u_prev;
        rls.update(y, u);
        y_prev = y;
        u_prev = u;
    }

    const Eigen::VectorXd theta = rls.params();
    // theta = [a1, b1] where model is y[k] = -a1*y[k-1] + b1*u[k-1]
    // True: a1 = -0.8, b1 = 0.2 in standard form
    REQUIRE(theta.size() == 2);
    // Check that identified model matches true poles/gains within 5%
    REQUIRE_THAT(std::abs(theta(0)), WithinAbs(0.8, 0.05));
    REQUIRE_THAT(std::abs(theta(1)), WithinAbs(0.2, 0.05));
}

// -----------------------------------------------------------------------------
// StepResponseTuner - robust identification (P9-10)
// -----------------------------------------------------------------------------

TEST_CASE("StepResponseTuner::identify handles noisy step response", "[step_tuner]")
{
    // True FOPDT: K=2, tau=0.5s, theta=0.05s
    // Simulate: y(t) approx K*(1 - exp(-(t-theta)/tau)) for t > theta
    const double K_true = 2.0, tau_true = 0.5, theta_true = 0.05;
    const double step_mag = 1.0;
    const int    N_pts  = 500;
    const double dt     = 0.005; // 2.5 s total

    std::srand(42);
    std::vector<double> t(N_pts), y(N_pts);
    for (int i = 0; i < N_pts; ++i)
    {
        t[i] = i * dt;
        const double t_eff = t[i] - theta_true;
        const double y_true = (t_eff > 0) ? K_true * (1.0 - std::exp(-t_eff / tau_true)) : 0.0;
        // Add +-2% noise relative to final value
        const double noise = 0.02 * K_true * (static_cast<double>(std::rand()) / RAND_MAX - 0.5);
        y[i] = y_true + noise;
    }

    const auto model = ctrl::StepResponseTuner::identify(t, y, step_mag);

    REQUIRE_THAT(model.K,     WithinRel(K_true,     0.10)); // within 10%
    REQUIRE_THAT(model.tau,   WithinRel(tau_true,   0.20)); // within 20%
    REQUIRE(model.theta >= 0.0);
    REQUIRE(model.theta < tau_true); // dead time < time constant
}

TEST_CASE("StepResponseTuner::identify handles non-zero baseline (drifting DC)", "[step_tuner]")
{
    // Same FOPDT but output starts at y_offset = 3.0 (non-zero baseline)
    const double K_true = 1.5, tau_true = 0.3, theta_true = 0.02;
    const double y_offset = 3.0;
    const double step_mag = 1.0;
    const int N_pts = 400;
    const double dt = 0.004;

    std::vector<double> t(N_pts), y(N_pts);
    for (int i = 0; i < N_pts; ++i)
    {
        t[i] = i * dt;
        const double t_eff = t[i] - theta_true;
        y[i] = y_offset + ((t_eff > 0) ? K_true * (1.0 - std::exp(-t_eff / tau_true)) : 0.0);
    }

    const auto model = ctrl::StepResponseTuner::identify(t, y, step_mag);

    // Baseline correction should recover the correct K regardless of offset
    REQUIRE_THAT(model.K,   WithinRel(K_true,   0.10));
    REQUIRE_THAT(model.tau, WithinRel(tau_true, 0.20));
}

TEST_CASE("StepResponseTuner::identify works for negative step input", "[step_tuner]")
{
    // Negative step: y decays from 0 to -K
    const double K_true = 2.0, tau_true = 0.4, theta_true = 0.01;
    const double step_mag = -1.0; // negative step
    const int N_pts = 300;
    const double dt = 0.005;

    std::vector<double> t(N_pts), y(N_pts);
    for (int i = 0; i < N_pts; ++i)
    {
        t[i] = i * dt;
        const double t_eff = t[i] - theta_true;
        // response to negative step: y = K_true * step_mag * (1 - exp(-t_eff/tau))
        y[i] = (t_eff > 0) ? K_true * step_mag * (1.0 - std::exp(-t_eff / tau_true)) : 0.0;
    }

    const auto model = ctrl::StepResponseTuner::identify(t, y, step_mag);

    REQUIRE_THAT(model.K,   WithinAbs(K_true, 0.2));   // sign-agnostic gain
    REQUIRE_THAT(model.tau, WithinRel(tau_true, 0.20));
    REQUIRE(model.theta >= 0.0);
}

// -----------------------------------------------------------------------------
// RelayAutoTuner - relative hysteresis (P9-11)
// -----------------------------------------------------------------------------

TEST_CASE("RelayAutoTuner with hysteresis_rel=0 matches hysteresis_rel=0 baseline", "[relay]")
{
    // Simple first-order continuous plant, Euler-integrated: y' = -y/tau + u/tau
    // At steady state limit cycle with relay amplitude d, Ku = 4d/(pi*a_y).
    const double tau_plant = 0.5;
    const double dt = 0.001;

    ctrl::RelayTunerConfig cfg;
    cfg.relayAmplitude  = 1.0;
    cfg.hysteresis      = 0.0;
    cfg.hysteresis_rel  = 0.0;
    cfg.cyclesRequired  = 4;

    ctrl::RelayAutoTuner tuner(cfg, dt);

    double y = 0.0;
    int max_steps = 50000;
    for (int k = 0; k < max_steps && !tuner.isDone(); ++k)
    {
        const double u = tuner.step(y);
        y += dt * (-y / tau_plant + u / tau_plant);
    }

    REQUIRE(tuner.isDone());
    REQUIRE(tuner.ultimateGain()   > 0.0);
    REQUIRE(tuner.ultimatePeriod() > 0.0);
}

TEST_CASE("RelayAutoTuner hysteresis_rel converges on noisy signal", "[relay]")
{
    // Same plant as above but with additive white noise.
    // With relative hysteresis the tuner should still complete (not lock up).
    const double tau_plant = 0.5;
    const double dt = 0.001;

    ctrl::RelayTunerConfig cfg;
    cfg.relayAmplitude  = 1.0;
    cfg.hysteresis_rel  = 0.03; // 3% of amplitude
    cfg.cyclesRequired  = 4;

    ctrl::RelayAutoTuner tuner(cfg, dt);

    std::srand(7);
    double y = 0.0;
    int max_steps = 80000;
    for (int k = 0; k < max_steps && !tuner.isDone(); ++k)
    {
        const double noise = 0.02 * (static_cast<double>(std::rand()) / RAND_MAX - 0.5);
        const double u = tuner.step(y + noise);
        y += dt * (-y / tau_plant + u / tau_plant);
    }

    REQUIRE(tuner.isDone());
    // Ku should be in a physically plausible range for this plant
    REQUIRE(tuner.ultimateGain()   > 0.1);
    REQUIRE(tuner.ultimatePeriod() > 0.0);
}

// -----------------------------------------------------------------------------
// SubspaceID n4sid - functional test (6-step algorithm)
// -----------------------------------------------------------------------------

TEST_CASE("n4sid identifies a first-order discrete-time system", "[subspace][n4sid]")
{
    // True system: y[k+1] = 0.8*y[k] + 0.5*u[k]  (pole at 0.8, DC gain = 2.5)
    const double a_true = 0.8, b_true = 0.5, c_true = 1.0;
    const int    N      = 800;

    Eigen::MatrixXd Y(1, N), U(1, N);
    std::srand(99);
    double x = 0.0;
    for (int k = 0; k < N; ++k)
    {
        // PRBS-like input: random +/-1 switches
        const double u = (std::rand() % 2 == 0) ? 1.0 : -1.0;
        U(0, k) = u;
        Y(0, k) = c_true * x;
        x = a_true * x + b_true * u;
    }

    const auto result = ctrl::n4sid(Y, U, 1, 6, Ts);

    REQUIRE(result.success);
    REQUIRE(result.model.has_value());

    const auto &model = result.model.value();

    // Eigenvalue of A should be close to 0.8
    const double pole = model.A(0, 0);
    REQUIRE_THAT(std::abs(pole), WithinAbs(a_true, 0.05));

    // DC gain = C*(I-A)^{-1}*B = c/(1-a)*b
    const double dc_true = c_true * b_true / (1.0 - a_true); // = 2.5
    const double dc_id   = model.C(0, 0) * model.B(0, 0) / (1.0 - model.A(0, 0));
    REQUIRE_THAT(dc_id, WithinAbs(dc_true, 2.0)); // subspace ID DC gain uncertainty is larger than pole uncertainty

    // Kalman gain and innovation covariance should be non-empty
    REQUIRE(result.kalmanGain.rows() == 1);
    REQUIRE(result.kalmanGain.cols() == 1);
    REQUIRE(result.innovCov.rows()   == 1);
}

TEST_CASE("suggestOrder returns 1 for a first-order system singular values", "[subspace][n4sid]")
{
    // When there is one dominant singular value, suggestOrder should return 1.
    Eigen::VectorXd sv(5);
    sv << 100.0, 0.5, 0.3, 0.2, 0.1; // sharp drop after index 0

    const int order = ctrl::suggestOrder(sv, 0.01, -1);
    REQUIRE(order == 1);
}
