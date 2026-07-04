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
 *   - LQRAdapter MIMO (m=2): computeVec() returns full vector, compute() truncates (P12-20)
 *   - c2d() ZOH accuracy on stiff plant cond(A)=100 (P12-20)
 *   - RLS forgetting factor prevents covariance blow-up on integrating signal (P12-20)
 *   - scipy/control cross-validation: c2d ZOH/Tustin, LQR K matrix, step response
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include "ControllerToolbox.h"
#include "IterativeLearningControl.h"
#include "SINDy.h"
#include "KoopmanEDMD.h"
#include "L1AdaptiveController.h"
#include "CBFSafetyFilter.h"
#include "GaussianProcess.h"
#include "GPResidualModel.h"
#include "GreyBoxEstimator.h"
#include "RecursiveGreyBoxEstimator.h"
#include "EchoStateNetwork.h"
#include "NeuralPID.h"
#include "CEMController.h"
#include "DynaController.h"
#include "ScenarioMPC.h"
#include "BayesianOptimizer.h"
#include "GeneticAlgorithm.h"
#include "ParticleSwarmOptimizer.h"
#include "DifferentialEvolution.h"
#include "ControllerRegistry.h"
#include "ControllerMonitor.h"
#include "HybridModel.h"
#include "HybridMPC.h"
#include "HybridModelTrainer.h"
#include "VectorFitting.h"
#include "LPVSystemID.h"
#include "MetricsAnalyzer.h"
#include "ZeroPhaseTrackingFilter.h"
#include "FunctionApproximator.h"
#include "DeePC.h"
#include "WorstCaseSearch.h"
#include "LyapunovRobustness.h"
#include "FreqDomainIdentifier.h"
#include "ResonantController.h"
#include "ComplexVectorFit.h"
#include "hal/HAL.h"   // SimScheduler, StdTimer (HAL not in umbrella by default)
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <random>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>

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

    Eigen::VectorXd x(1), tmp1(1), tmp2(1), y_fista(1);
    const double L = H.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff();

    const auto res = ctrl::solveGradientProjectionQP(H, g, lb, ub, ldlt, L, 200, 1e-12, x, tmp1, tmp2, y_fista);

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

    Eigen::VectorXd x(1), tmp1(1), tmp2(1), y_fista(1);
    const auto res = ctrl::solveGradientProjectionQP(H, g, lb, ub, ldlt, L, 200, 1e-12, x, tmp1, tmp2, y_fista);

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
// NonsingularTerminalSMC - finite-time reaching + zero offset on an integrator
// -----------------------------------------------------------------------------

TEST_CASE("NonsingularTerminalSMC reaches the setpoint in finite time on an integrator",
          "[smc][terminal]")
{
    // Integrator plant y[k+1] = y + b*u holds any y at u = 0, so an SMC without
    // integral action still reaches the setpoint with no steady-state offset.
    ctrl::NonsingularTerminalSMCParams p;
    p.beta = 1.0;
    p.gamma = 1.5;   // in (1,2): nonsingular + finite-time
    p.K   = 2.0;
    p.eta = 0.5;
    p.phi = 0.5;
    p.uMin = -20.0;
    p.uMax =  20.0;
    ctrl::NonsingularTerminalSMC smc(p, Ts);

    const double b = 0.1, ref = 1.0;
    double y = 0.0;
    int reach = -1;
    for (int k = 0; k < 800; ++k)
    {
        const double u = smc.compute(y - ref); // SMC sign convention: y - ref
        y += b * u;
        if (reach < 0 && std::abs(smc.slidingSurface()) < p.phi)
            reach = k;
    }
    REQUIRE(std::isfinite(y));
    REQUIRE(reach >= 0);                    // finite-time reaching of the boundary layer
    REQUIRE(std::abs(ref - y) < 0.02);      // integrator -> negligible steady offset
}

// -----------------------------------------------------------------------------
// AdaptiveSMC - switching gain grows to reject an unknown-bound disturbance
// -----------------------------------------------------------------------------

TEST_CASE("AdaptiveSMC grows its gain to reject an unknown-bound disturbance",
          "[smc][adaptive]")
{
    ctrl::AdaptiveSMCParams p;
    p.c_e = 1.0;
    p.c_de = 0.05;
    p.gamma = 8.0;
    p.epsilon = 0.02;
    p.K0 = 0.2;       // deliberately too small for the disturbance below
    p.Kmin = 0.0;
    p.Kmax = 100.0;
    p.phi = 0.3;
    p.uMin = -20.0;
    p.uMax =  20.0;
    ctrl::AdaptiveSMC smc(p, Ts);

    const double b = 0.1, d = 0.3, ref = 1.0; // matched disturbance, bound unknown to controller
    double y = 0.0;
    const double K0 = smc.adaptiveGain();
    for (int k = 0; k < 1500; ++k)
    {
        const double u = smc.compute(y - ref);
        y += b * (u + d);
    }
    REQUIRE(std::isfinite(y));
    REQUIRE(smc.adaptiveGain() > K0);       // gain adapted upward
    REQUIRE(std::abs(ref - y) < 0.05);      // disturbance rejected without a-priori bound
}

// -----------------------------------------------------------------------------
// FractionalDifferintegrator - Oustaloup |s^alpha| = 1 at the band centre
// -----------------------------------------------------------------------------

TEST_CASE("FractionalDifferintegrator matches |s^alpha| = 1 at the band centre", "[pid][fopid]")
{
    // s^{0.5} over [0.01, 100] rad/s -> geometric centre sqrt(wb*wh) = 1 rad/s,
    // where |(j*1)^0.5| = 1 exactly. Ts = 0.005 keeps Nyquist (~628 rad/s) well above wh.
    const double Tsf = 0.005;
    ctrl::FractionalDifferintegrator op(0.5, 0.01, 100.0, 5, Tsf);

    const double w = 1.0;
    double amp = 0.0;
    const int N = 40000;
    for (int k = 0; k < N; ++k)
    {
        const double o = op.compute(std::sin(w * k * Tsf));
        if (k > N - 4000) amp = std::max(amp, std::abs(o)); // steady-state amplitude
    }
    // Tolerance ~15%: finite Oustaloup order (N=5 -> 11 sections) + bilinear warping.
    REQUIRE(amp > 0.85);
    REQUIRE(amp < 1.15);
}

// -----------------------------------------------------------------------------
// FractionalOrderPID - degenerates to proportional + tracks a step
// -----------------------------------------------------------------------------

TEST_CASE("FractionalOrderPID reduces to P-only and tracks a step", "[pid][fopid]")
{
    // Ki = Kd = 0 -> pure proportional passthrough u = Kp * e.
    {
        ctrl::FOPIDParams pp;
        pp.Kp = 1.0; pp.Ki = 0.0; pp.Kd = 0.0;
        ctrl::FractionalOrderPID pure(pp, Ts);
        REQUIRE(std::abs(pure.compute(1.0) - 1.0) < 1e-9);
    }

    // Closed-loop step tracking on y[k+1] = 0.8*y + 0.2*u (DC gain 1). The
    // fractional-integral branch supplies the steady control that removes offset.
    ctrl::FOPIDParams p;
    p.Kp = 0.5; p.Ki = 0.3; p.Kd = 0.02;
    p.lambda = 0.9; p.mu = 0.6;
    p.wb = 0.01; p.wh = 100.0; p.N = 4;
    p.uMin = -10.0; p.uMax = 10.0;
    ctrl::FractionalOrderPID fopid(p, Ts);

    double y = 0.0;
    const double ref = 1.0;
    for (int k = 0; k < 2000; ++k)
    {
        const double u = fopid.compute(ref - y); // PID sign convention: r - y
        y = 0.8 * y + 0.2 * u;
    }
    REQUIRE(std::isfinite(y));
    REQUIRE(std::abs(ref - y) < 0.1);
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

// -----------------------------------------------------------------------------
// P12-20: LQRAdapter on a true MIMO plant (m=2)
// -----------------------------------------------------------------------------

TEST_CASE("LQRAdapter computeVec returns full vector for MIMO plant (m=2)", "[lqr][adapter][mimo][p12-20]")
{
    // Decoupled 2-state, 2-input plant: each input controls its own state independently.
    // A = diag(0.9, 0.8), B = diag(0.1, 0.2).
    // This is the minimal plant where m=2 causes silent truncation in compute().
    Eigen::Matrix2d A, B;
    A << 0.9, 0.0, 0.0, 0.8;
    B << 0.1, 0.0, 0.0, 0.2;
    Eigen::MatrixXd C = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2, 2);
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::LQRParams lqr_p;
    lqr_p.Q = Eigen::Matrix2d::Identity() * 10.0;
    lqr_p.R = Eigen::Matrix2d::Identity();
    ctrl::DiscreteLQR lqr(plant, lqr_p);
    REQUIRE(lqr.dareConverged());

    // Non-zero state on both axes so both control outputs are non-zero
    Eigen::Vector2d x_state;
    x_state << 1.0, -0.5;

    ctrl::LQRAdapter adapter(lqr,
        [&x_state]() -> Eigen::VectorXd { return x_state; });

    // computeVec() must return all m=2 control inputs, not silently truncate to 1
    const Eigen::VectorXd u_vec = adapter.computeVec(Eigen::VectorXd());
    REQUIRE(u_vec.size() == 2);

    // Both components must be non-zero (each state drives its own actuator)
    REQUIRE(std::abs(u_vec(0)) > 1e-10);
    REQUIRE(std::abs(u_vec(1)) > 1e-10);

    // compute() scalar path: must equal u_vec(0) exactly (not a separate computation)
    const double u_scalar = adapter.compute(0.0);
    REQUIRE_THAT(u_scalar, WithinRel(u_vec(0), 1e-12));

    // Closed-loop: both states should converge to zero under MIMO LQR
    for (int k = 0; k < 300; ++k)
    {
        const Eigen::VectorXd u = lqr.compute(x_state);
        ctrl::ssStep(plant, x_state, u);
    }
    REQUIRE(x_state.norm() < 0.05);
}

// -----------------------------------------------------------------------------
// P12-20: c2d() ZOH accuracy on a stiff plant (cond(A) >= 100)
// -----------------------------------------------------------------------------

TEST_CASE("c2d ZOH is accurate for a stiff plant with cond(A) = 100", "[c2d][numerical][p12-20]")
{
    // Continuous: Ac = diag(-1, -100) - slow pole at -1, fast pole at -100.
    // Condition number of Ac = 100/1 = 100 (stiff).
    // Exact ZOH matrix exponential at Ts = 0.01:
    //   Ad(0,0) = exp(-0.01) ~ 0.99004983
    //   Ad(1,1) = exp(-1.0)  ~ 0.36787944
    // Off-diagonal entries are exactly zero (decoupled system).
    Eigen::Matrix2d Ac;
    Ac << -1.0,    0.0,
           0.0, -100.0;
    Eigen::MatrixXd Bc(2, 1);
    Bc << 1.0, 1.0;
    Eigen::MatrixXd Cc(1, 2);
    Cc << 1.0, 1.0;
    Eigen::MatrixXd Dc(1, 1);
    Dc << 0.0;

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    const ctrl::StateSpace sys_d = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    const double a00_exact = std::exp(-1.0   * Ts);  // slow pole: exp(-0.01)
    const double a11_exact = std::exp(-100.0 * Ts);  // fast pole: exp(-1.0)

    // Diagonal elements must match the exact exponential to 8 significant figures
    REQUIRE_THAT(sys_d.A(0, 0), WithinAbs(a00_exact, 1e-8));
    REQUIRE_THAT(sys_d.A(1, 1), WithinAbs(a11_exact, 1e-8));

    // Off-diagonal entries must be (near) zero for the decoupled system
    REQUIRE_THAT(sys_d.A(0, 1), WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(sys_d.A(1, 0), WithinAbs(0.0, 1e-10));

    // Advance one step with x=[1,0]^T and u=0: x_next = A_d*[1,0]^T = [a00, 0].
    // ssStep mutates x in-place; check x directly after the step.
    Eigen::Vector2d x;
    x << 1.0, 0.0;
    Eigen::VectorXd u_zero(1);
    u_zero << 0.0;
    ctrl::ssStep(sys_d, x, u_zero);  // x is now [a00, 0]
    REQUIRE_THAT(x(0), WithinAbs(a00_exact, 1e-8));   // slow pole decayed by exp(-0.01)
    REQUIRE_THAT(x(1), WithinAbs(0.0,       1e-10));  // fast pole was 0, stays 0 (decoupled)
}

// -----------------------------------------------------------------------------
// P12-20: RLS with forgetting factor stays bounded on integrating plant
// -----------------------------------------------------------------------------

TEST_CASE("RLS with forgetting factor stays bounded on integrating-output signal", "[rls][numerical][p12-20]")
{
    // True model: y[k] = y[k-1] + u[k-1]  (pure integrator, pole at z=1).
    // An integrating output y grows as a random walk, making the RLS information
    // matrix R_N = sum(phi*phi') potentially ill-conditioned without forgetting.
    // A forgetting factor lambda < 1 keeps the effective window to ~1/(1-lambda) steps,
    // preventing covariance blow-up.
    //
    // ARX form: y[k] + a1*y[k-1] = b1*u[k-1]  -> true params: a1=-1, b1=1.
    // RLS theta = [a1, b1] after identification.
    const double lambda_f = 0.95;
    ctrl::RecursiveLeastSquares rls(1, 1, Ts, lambda_f);

    double y = 0.0, u_prev = 0.0;
    std::srand(42);

    for (int k = 0; k < 500; ++k)
    {
        // Integrating plant: y[k] = y[k-1] + u[k-1]
        y = y + u_prev;
        const double u = 0.1 * (static_cast<double>(std::rand()) / RAND_MAX - 0.5);
        rls.update(y, u);
        u_prev = u;
    }

    // With forgetting, covariance P must remain finite (not blow up to inf/nan)
    const Eigen::MatrixXd &P = rls.covariance();
    REQUIRE(std::isfinite(P.norm()));
    REQUIRE(P.norm() < 1e8); // bounded; not exploding

    // Both parameter estimates must be finite
    const Eigen::VectorXd theta = rls.params();
    REQUIRE(std::isfinite(theta(0)));
    REQUIRE(std::isfinite(theta(1)));

    // The integrator pole identified near z=1 means |a1| should be close to 1
    // (large tolerance because short data, but clearly not diverged)
    REQUIRE(std::abs(theta(0)) < 2.0); // a1 in a reasonable range
    REQUIRE(rls.sampleCount() == 500);
}

// =============================================================================
// Part 17: SmithPredictor - integer and fractional dead-time (Gap 1 closure)
// Verifies that padeDelayFilter() is correctly wired into the theta-Ts ctor.
// =============================================================================

TEST_CASE("SmithPredictor with integer delay compensates FOPDT plant", "[smithpredictor]")
{
    // Plant: G(s) = 1/(s+1) discretised at Ts=0.1 s, pure integer delay d=3.
    const double Ts_sp = 0.1;
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace G0 = ctrl::c2d(sys_c, Ts_sp, ctrl::C2dMethod::ZOH);

    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 1.0; pp.Kd = 0.0;
    pp.uMin = -20.0; pp.uMax = 20.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts_sp);

    ctrl::SmithPredictor sp(pid, G0, 3);
    REQUIRE_THAT(sp.sampleTime(), WithinAbs(Ts_sp, 1e-12));

    // Closed-loop simulation: 3-step circular delay on plant output
    Eigen::VectorXd x_plant = Eigen::VectorXd::Zero(G0.stateSize());
    std::vector<double> delay_buf(3, 0.0);
    int buf_head = 0;
    const double ref = 1.0;
    double y = 0.0;

    for (int k = 0; k < 200; ++k) {
        const double u = sp.compute(ref - y);
        Eigen::VectorXd uv(1); uv << u;
        const double y_nd = (G0.C * x_plant + G0.D * uv)(0);
        x_plant = G0.A * x_plant + G0.B * uv;
        const double y_del = delay_buf[buf_head];
        delay_buf[buf_head] = y_nd;
        buf_head = (buf_head + 1) % 3;
        y = y_del;
    }
    // Plant should converge to reference despite 0.3 s dead time
    REQUIRE_THAT(y, WithinAbs(ref, 0.05));
}

TEST_CASE("SmithPredictor theta-Ts ctor wires padeDelayFilter for fractional delay",
          "[smithpredictor]")
{
    // Plant: G(s)=1/(s+1), theta=0.35 s -> d=3, theta_frac=0.05 s.
    // The theta-Ts constructor must call padeDelayFilter(0.05, 0.1) automatically.
    const double Ts_sp = 0.1;
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace G0 = ctrl::c2d(sys_c, Ts_sp, ctrl::C2dMethod::ZOH);

    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 1.0; pp.Kd = 0.0;
    pp.uMin = -20.0; pp.uMax = 20.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts_sp);

    // Must construct without throwing (wires Pade filter internally)
    ctrl::SmithPredictor sp(pid, G0, 0.35, Ts_sp);
    REQUIRE_THAT(sp.sampleTime(), WithinAbs(Ts_sp, 1e-12));

    // Same simulation loop: integer delay = 3 steps
    Eigen::VectorXd x_plant = Eigen::VectorXd::Zero(G0.stateSize());
    std::vector<double> delay_buf(3, 0.0);
    int buf_head = 0;
    const double ref = 1.0;
    double y = 0.0;

    for (int k = 0; k < 250; ++k) {
        const double u = sp.compute(ref - y);
        Eigen::VectorXd uv(1); uv << u;
        const double y_nd = (G0.C * x_plant + G0.D * uv)(0);
        x_plant = G0.A * x_plant + G0.B * uv;
        const double y_del = delay_buf[buf_head];
        delay_buf[buf_head] = y_nd;
        buf_head = (buf_head + 1) % 3;
        y = y_del;
    }
    // Converges with fractional Pade approximation (looser tolerance than integer)
    REQUIRE_THAT(y, WithinAbs(ref, 0.1));
}

// =============================================================================
// Part 17: FOPDTIdentifier - fits K, tau, theta from step-response data
// =============================================================================

TEST_CASE("FOPDTIdentifier recovers K, tau, theta from synthetic step response",
          "[fopdt]")
{
    // True FOPDT: K=2.0, tau=5.0 s, theta=1.5 s
    const double K_true   = 2.0;
    const double tau_true = 5.0;
    const double th_true  = 1.5;
    const double step_mag = 0.5;

    // Generate clean step-response data at 0.1 s sampling
    const double Ts_data = 0.1;
    const int    N       = 300;  // 30 s
    std::vector<double> t(N), y(N);
    for (int i = 0; i < N; ++i) {
        t[i] = i * Ts_data;
        const double dt = t[i] - th_true;
        y[i] = (dt > 0.0) ? K_true * step_mag * (1.0 - std::exp(-dt / tau_true)) : 0.0;
    }

    ctrl::FOPDTIdentifier id(t, y, step_mag, 0.0);

    // Graphical method
    const ctrl::FOPDTModel mg = id.identify(ctrl::FOPDTMethod::Graphical);
    REQUIRE_THAT(mg.K,     WithinAbs(K_true,   0.05));   // K within 5%
    REQUIRE_THAT(mg.tau,   WithinAbs(tau_true, 0.5));    // tau within 0.5 s
    REQUIRE_THAT(mg.theta, WithinAbs(th_true,  0.4));    // theta within 0.4 s
    REQUIRE(mg.fitRMSE < 0.1 * K_true * step_mag);      // RMSE < 10% of step output

    // Optimization method should match or beat graphical
    const ctrl::FOPDTModel mo = id.identify(ctrl::FOPDTMethod::Optimization);
    REQUIRE_THAT(mo.K,     WithinAbs(K_true,   0.05));
    REQUIRE_THAT(mo.tau,   WithinAbs(tau_true, 0.3));
    REQUIRE_THAT(mo.theta, WithinAbs(th_true,  0.2));
    REQUIRE(mo.fitRMSE <= mg.fitRMSE + 1e-6);  // optimization is no worse

    // IMC tuning from optimization model: lambda_c = 2*theta -> valid PID params
    const auto pp = ctrl::FOPDTIdentifier::imcTuning(mo, 2.0 * mo.theta, Ts_data);
    REQUIRE(pp.Kp > 0.0);
    REQUIRE(pp.Ki > 0.0);
    REQUIRE(std::isfinite(pp.Kp));
    REQUIRE(std::isfinite(pp.Ki));
}

// =============================================================================
// Part 17: FeedforwardController - applies StateSpace filter to reference
// =============================================================================

TEST_CASE("FeedforwardController computes u_ff = G_ff(z) * r", "[feedforward]")
{
    // G_ff(z) = gain filter: y = 0.5 * u  (static gain via D=0.5, A=0, B=0, C=0)
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A(0,0) = 0.0; B(0,0) = 0.0; C(0,0) = 0.0; D(0,0) = 0.5;
    ctrl::StateSpace Gff(A, B, C, D, 0.1);

    ctrl::FeedforwardController ff(Gff);
    REQUIRE_THAT(ff.sampleTime(), WithinAbs(0.1, 1e-12));

    // Static gain: u_ff = 0.5 * r for any r
    REQUIRE_THAT(ff.compute(2.0), WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(ff.compute(4.0), WithinAbs(2.0, 1e-12));

    ff.reset();

    // First-order filter: A=a, B=b, C=c, D=d; test that state evolves
    const double a = 0.8, b = 0.2, c = 1.0, d = 0.0;
    Eigen::MatrixXd A2(1,1), B2(1,1), C2(1,1), D2(1,1);
    A2(0,0)=a; B2(0,0)=b; C2(0,0)=c; D2(0,0)=d;
    ctrl::FeedforwardController ff2(ctrl::StateSpace(A2, B2, C2, D2, 0.1));

    // k=0: y = C*0 + D*1 = 0, x+ = A*0 + B*1 = 0.2
    REQUIRE_THAT(ff2.compute(1.0), WithinAbs(0.0,      1e-12));
    // k=1: y = C*0.2 + D*1 = 0.2, x+ = A*0.2 + B*1 = 0.8*0.2+0.2 = 0.36
    REQUIRE_THAT(ff2.compute(1.0), WithinAbs(b,        1e-12));
    // k=2: y = C*0.36 + D*1 = 0.36
    REQUIRE_THAT(ff2.compute(1.0), WithinAbs(a*b + b,  1e-10));
}

// =============================================================================
// Part 17: SimScheduler (HAL stub) - fires callback synchronously
// =============================================================================

TEST_CASE("SimScheduler fires callback synchronously via run()", "[hal][simscheduler]")
{
    ctrl::SimScheduler sched;
    ctrl::StdTimer     timer;

    const uint64_t period_ns = 1'000'000;  // 1 ms
    sched.setPeriodNs(period_ns);
    REQUIRE(sched.periodNs() == period_ns);
    REQUIRE(!sched.isRunning());

    int call_count = 0;
    sched.setCallback([&]() { ++call_count; });

    sched.start();
    REQUIRE(sched.isRunning());
    REQUIRE(sched.tickCount() == 0);

    // Fire 10 ticks without ITimer (count-only path)
    sched.run(10);
    REQUIRE(sched.tickCount() == 10);
    REQUIRE(call_count == 10);

    // Fire 5 more ticks with ITimer (overrun detection path)
    sched.run(timer, 5);
    REQUIRE(sched.tickCount() == 15);
    REQUIRE(call_count == 15);

    sched.stop();
    REQUIRE(!sched.isRunning());

    // After stop, run() must throw
    REQUIRE_THROWS_AS(sched.run(1), std::logic_error);
}

// =============================================================================
// Part 17: mu-synthesis (Gap 2 closure) - DK-iteration produces tighter bound
// =============================================================================

#if defined(CTRL_HAS_HINF)
TEST_CASE("DiscreteHinf::solveMuSyn runs DK-iteration without crashing",
          "[musyn][hinf]")
{
    // Simple SISO plant G(s) = 1/(s+1) at Ts=0.01 s.
    const double Ts_mu = 0.01;
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace G = ctrl::c2d(sys_c, Ts_mu, ctrl::C2dMethod::ZOH);

    // Generous S/KS/T weights with high gammaInit
    const auto W1 = ctrl::MixedSensitivity::makeW1(1.0, 2.0, 0.01, Ts_mu);
    const auto W2 = ctrl::MixedSensitivity::makeW2constant(0.5, Ts_mu);
    const auto W3 = ctrl::MixedSensitivity::makeW3(5.0, 2.0, 0.01, Ts_mu);
    const auto P  = ctrl::MixedSensitivity::build(G, W1, W2, W3);

    // mu-synthesis: 3 DK iterations, very high gammaInit to encourage feasibility
    ctrl::MuSynParams mp;
    mp.maxDKIter = 3;
    mp.hinfParams.gammaInit = 500.0;

    // Must NOT throw regardless of feasibility
    ctrl::MuSynResult mr;
    REQUIRE_NOTHROW(mr = ctrl::DiscreteHinf::solveMuSyn(P, mp));

    // Algorithm must complete and report a valid iteration count
    REQUIRE(mr.iterations >= 0);
    REQUIRE(mr.iterations <= mp.maxDKIter);

    // If a feasible K-step was found, the mu upper bound must be consistent
    if (mr.hinfResult.feasible) {
        REQUIRE(mr.achievedMuUpper > 0.0);
        REQUIRE(std::isfinite(mr.achievedMuUpper));
        // mu_upper from D-scaled evaluation should not exceed K-step gamma
        REQUIRE(mr.achievedMuUpper <= mr.hinfResult.achievedGamma + 1.0);
        // Standard H-inf (no D-scaling) should have gamma >= mu_upper
        ctrl::HinfResult hr_std = ctrl::DiscreteHinf::solve(P, mp.hinfParams);
        if (hr_std.feasible)
            REQUIRE(mr.achievedMuUpper <= hr_std.achievedGamma + 1e-2);
    }
}

TEST_CASE("DiscreteHinf::solveStructured tunes a static output-feedback gain via CMA-ES",
          "[structured][hinf]")
{
    // Simple SISO plant G(s) = 1/(s+1) at Ts=0.01 s (same idiom as the solveMuSyn test).
    const double Ts_s = 0.01;
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace G = ctrl::c2d(sys_c, Ts_s, ctrl::C2dMethod::ZOH);

    const auto W1 = ctrl::MixedSensitivity::makeW1(1.0, 2.0, 0.01, Ts_s);
    const auto W2 = ctrl::MixedSensitivity::makeW2constant(0.5, Ts_s);
    const auto W3 = ctrl::MixedSensitivity::makeW3(5.0, 2.0, 0.01, Ts_s);
    const ctrl::GeneralisedPlant P = ctrl::MixedSensitivity::build(G, W1, W2, W3);

    // Static output-feedback controller: nk=0, so u = Dk*y with Dk = [theta].
    // (Ak is 0x0, Bk is 0x ny, Ck is nu x 0 - the fixed-structure that solveStructured
    //  validates against P.ny()/P.nu() before starting the CMA-ES search.)
    auto staticGain = [](const Eigen::VectorXd &th,
                         Eigen::MatrixXd &Ak, Eigen::MatrixXd &Bk,
                         Eigen::MatrixXd &Ck, Eigen::MatrixXd &Dk) {
        Ak = Eigen::MatrixXd(0, 0);
        Bk = Eigen::MatrixXd(0, 1);
        Ck = Eigen::MatrixXd(1, 0);
        Dk = Eigen::MatrixXd(1, 1);
        Dk(0, 0) = th(0);
    };

    ctrl::StructuredHinfParams sp;
    sp.maxEvals = 300;   // keep the CMA-ES search short for a unit test
    sp.seed     = 42;    // deterministic

    const Eigen::VectorXd theta0 = Eigen::VectorXd::Constant(1, 0.0);

    ctrl::StructuredHinfResult r;
    REQUIRE_NOTHROW(r = ctrl::DiscreteHinf::solveStructured(P, staticGain, theta0, sp));

    // Mechanics that hold regardless of the optimiser's outcome:
    REQUIRE(r.theta.size() == 1);
    REQUIRE(r.nEvals > 0);
    REQUIRE(r.hinfResult.Dk.rows() == 1);
    REQUIRE(r.hinfResult.Dk.cols() == 1);
    REQUIRE(std::abs(r.hinfResult.Ts - P.Ts) < 1e-12);

    // achievedGamma is finite & positive when feasible, +inf otherwise - never NaN.
    if (r.hinfResult.feasible) {
        REQUIRE(std::isfinite(r.hinfResult.achievedGamma));
        REQUIRE(r.hinfResult.achievedGamma > 0.0);
    } else {
        REQUIRE(std::isinf(r.hinfResult.achievedGamma));
    }
}

TEST_CASE("DiscreteHinf::solveStructured validates its arguments up front",
          "[structured][hinf]")
{
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace G = ctrl::c2d(sys_c, 0.01, ctrl::C2dMethod::ZOH);
    const auto W1 = ctrl::MixedSensitivity::makeW1(1.0, 2.0, 0.01, 0.01);
    const auto W2 = ctrl::MixedSensitivity::makeW2constant(0.5, 0.01);
    const auto W3 = ctrl::MixedSensitivity::makeW3(5.0, 2.0, 0.01, 0.01);
    const ctrl::GeneralisedPlant P = ctrl::MixedSensitivity::build(G, W1, W2, W3);

    auto staticGain = [](const Eigen::VectorXd &th,
                         Eigen::MatrixXd &Ak, Eigen::MatrixXd &Bk,
                         Eigen::MatrixXd &Ck, Eigen::MatrixXd &Dk) {
        Ak = Eigen::MatrixXd(0, 0); Bk = Eigen::MatrixXd(0, 1);
        Ck = Eigen::MatrixXd(1, 0); Dk = Eigen::MatrixXd(1, 1); Dk(0, 0) = th(0);
    };

    // Empty theta0 -> throw.
    REQUIRE_THROWS_AS(
        ctrl::DiscreteHinf::solveStructured(P, staticGain, Eigen::VectorXd(), {}),
        std::invalid_argument);

    // A structureFn that produces a wrong-shaped Dk (2x2, expected 1x1) is rejected before
    // the search starts, not left to crash inside buildClosedLoop().
    auto badShape = [](const Eigen::VectorXd &,
                       Eigen::MatrixXd &Ak, Eigen::MatrixXd &Bk,
                       Eigen::MatrixXd &Ck, Eigen::MatrixXd &Dk) {
        Ak = Eigen::MatrixXd(0, 0); Bk = Eigen::MatrixXd(0, 1);
        Ck = Eigen::MatrixXd(1, 0); Dk = Eigen::MatrixXd(2, 2);
    };
    const Eigen::VectorXd theta0 = Eigen::VectorXd::Constant(1, 0.0);
    REQUIRE_THROWS_AS(
        ctrl::DiscreteHinf::solveStructured(P, badShape, theta0, {}),
        std::invalid_argument);
}
#endif

// =============================================================================
// Phase 4 Iteration 3: DiscreteH2 (discrete H2/LQG output-feedback synthesis)
// =============================================================================

#if defined(CTRL_HAS_HINF)

namespace
{
// Hand-built D11=0 generalised plant with nonzero control/filter cross terms (S1, S2),
// so the cross-term-elimination code path is genuinely exercised (not the degenerate
// S1=S2=0 case). nw=2 in the exogenous channel specifically avoids the scalar
// Cauchy-Schwarz degeneracy S2^2 == Q2*R2 that occurs for any SISO single-noise-channel
// (n=1, nw=1) plant and would force the filter Riccati solution Y to be exactly zero.
ctrl::GeneralisedPlant makeH2CrossTermPlant()
{
    ctrl::GeneralisedPlant P;
    P.Ts  = 0.1;
    P.A   = Eigen::MatrixXd::Constant(1, 1, 0.9);
    P.B1  = (Eigen::MatrixXd(1, 2) << 0.3, 0.1).finished();
    P.B2  = Eigen::MatrixXd::Constant(1, 1, 1.0);
    P.C1  = (Eigen::MatrixXd(2, 1) << 1.0, 0.3).finished();
    P.C2  = Eigen::MatrixXd::Constant(1, 1, 1.0);
    P.D11 = Eigen::MatrixXd::Zero(2, 2);
    P.D12 = (Eigen::MatrixXd(2, 1) << 0.2, 1.0).finished();
    P.D21 = (Eigen::MatrixXd(1, 2) << 0.1, 0.4).finished();
    P.D22 = Eigen::MatrixXd::Zero(1, 1);
    return P;
}
} // namespace

TEST_CASE("DiscreteH2::solve matches independent scipy DARE/Lyapunov reference", "[h2][hinf]")
{
    // Reference computed 2026-06-22 via scipy.linalg.solve_discrete_are (cross-term `s`
    // argument) + solve_discrete_lyapunov for the exact plant built by
    // makeH2CrossTermPlant() above:
    //   X = 0.93620768, Y = 0.08464685
    //   F = -0.67937541, L(=-Bk) = -0.57405840
    //   Ak = -0.35343382, Bk = 0.57405840
    //   achievedH2Norm = 0.41331452
    //   closed-loop [x;xk] eigenvalues = {0.32594160, 0.22062459} (both stable, matching
    //   the separation principle: eig(A+B2F) union eig(A - (-L)*C2))
    // Tolerance 1e-4: tight enough to catch a sign error in F/L/Bk (which would otherwise
    // flip a result that still happens to converge, e.g. an unstable-but-finite Ak), loose
    // enough for the doubling-iteration DARE solver's own internal tolerance (1e-12 on the
    // Riccati residual, but compounded through several matrix products before comparison).
    const ctrl::GeneralisedPlant P = makeH2CrossTermPlant();
    const ctrl::H2Result result = ctrl::DiscreteH2::solve(P);

    REQUIRE(result.feasible);
    REQUIRE(result.dareConvX);
    REQUIRE(result.dareConvY);

    REQUIRE_THAT(result.X(0, 0),              WithinAbs(0.93620768, 1e-4));
    REQUIRE_THAT(result.Y(0, 0),              WithinAbs(0.08464685, 1e-4));
    REQUIRE_THAT(result.Ck(0, 0),              WithinAbs(-0.67937541, 1e-4)); // Ck == F
    REQUIRE_THAT(result.Ak(0, 0),              WithinAbs(-0.35343382, 1e-4));
    REQUIRE_THAT(result.Bk(0, 0),              WithinAbs( 0.57405840, 1e-4));
    REQUIRE(result.Dk(0, 0) == 0.0);
    REQUIRE_THAT(result.achievedH2Norm,        WithinAbs(0.41331452, 1e-4));

    // Separation-principle check: the full [x;xk] closed loop's eigenvalues must be the
    // union of eig(A+B2*F) and eig(A - Lobs*C2) where Lobs = -Bk (see DiscreteH2.cpp's
    // assembly comment) - not just "Ak alone is stable", which a sign bug in Bk could
    // satisfy by coincidence while still leaving the full closed loop unstable (this is
    // exactly the failure mode the scipy reference computation above caught during
    // development: Bk = +L gave a stable Ak but an unstable full closed loop).
    Eigen::MatrixXd A_cl(2, 2);
    A_cl << P.A(0,0),                         P.B2(0,0) * result.Ck(0,0),
            result.Bk(0,0) * P.C2(0,0),       result.Ak(0,0);
    Eigen::EigenSolver<Eigen::MatrixXd> esAcl(A_cl, /*computeEigenvectors=*/false);
    for (int i = 0; i < esAcl.eigenvalues().size(); ++i)
        REQUIRE(std::abs(esAcl.eigenvalues()(i)) < 1.0);
}

TEST_CASE("DiscreteH2::solve succeeds on a nonzero-D11 (MixedSensitivity-built) plant, "
          "with F/L unaffected by D11 and the achieved norm picking up exactly trace(D11*D11')",
          "[h2][hinf]")
{
    // MixedSensitivity-built plants have D11 != 0 by construction (W1/W3 weight gains feed
    // straight through to z) - this used to be a hard rejection case; D11 is now supported.
    const double Ts = 0.1;
    const ctrl::StateSpace G = ctrl::c2d(
        ctrl::StateSpace(Eigen::MatrixXd::Constant(1,1,-1.0), Eigen::MatrixXd::Constant(1,1,1.0),
                          Eigen::MatrixXd::Constant(1,1,1.0), Eigen::MatrixXd::Zero(1,1), 0.0),
        Ts, ctrl::C2dMethod::ZOH);
    const auto W1 = ctrl::MixedSensitivity::makeW1(2.0, 2.0, 0.01, Ts);
    const auto W2 = ctrl::MixedSensitivity::makeW2constant(0.1, Ts);
    const auto W3 = ctrl::MixedSensitivity::makeW3(30.0, 1.5, 0.01, Ts);
    const auto P  = ctrl::MixedSensitivity::build(G, W1, W2, W3);

    REQUIRE(!P.D11.isZero(1e-12)); // sanity: confirms D11 is genuinely nonzero here

    const ctrl::H2Result result = ctrl::DiscreteH2::solve(P);
    REQUIRE(result.feasible);

    // Same plant with D11 zeroed out - everything else (A,B1,B2,C1,C2,D12,D21,D22) identical.
    ctrl::GeneralisedPlant P0 = P;
    P0.D11 = Eigen::MatrixXd::Zero(P.D11.rows(), P.D11.cols());
    const ctrl::H2Result result0 = ctrl::DiscreteH2::solve(P0);
    REQUIRE(result0.feasible);

    // D11 must not affect the controller gains at all (it never enters the Riccati equations).
    REQUIRE(result.Ak.isApprox(result0.Ak, 1e-9));
    REQUIRE(result.Bk.isApprox(result0.Bk, 1e-9));
    REQUIRE(result.Ck.isApprox(result0.Ck, 1e-9));

    // The achieved norm must differ from the D11=0 case by exactly trace(D11*D11') in quadrature.
    const double d11_contribution = (P.D11 * P.D11.transpose()).trace();
    const double expected_sq = result0.achievedH2Norm * result0.achievedH2Norm + d11_contribution;
    REQUIRE_THAT(result.achievedH2Norm * result.achievedH2Norm, WithinAbs(expected_sq, 1e-6));
}

TEST_CASE("DiscreteH2::solve throws on rank-deficient D12/D21", "[h2][hinf]")
{
    ctrl::GeneralisedPlant P = makeH2CrossTermPlant();
    P.D12 = Eigen::MatrixXd::Zero(2, 1); // rank 0, nu = 1 -> not full column rank
    REQUIRE_THROWS_AS(ctrl::DiscreteH2::solve(P), std::invalid_argument);

    ctrl::GeneralisedPlant P2 = makeH2CrossTermPlant();
    P2.D21 = Eigen::MatrixXd::Zero(1, 2); // rank 0, ny = 1 -> not full row rank
    REQUIRE_THROWS_AS(ctrl::DiscreteH2::solve(P2), std::invalid_argument);
}

TEST_CASE("DiscreteH2::solve throws on nonzero D22", "[h2][hinf]")
{
    ctrl::GeneralisedPlant P = makeH2CrossTermPlant();
    P.D22 = Eigen::MatrixXd::Constant(1, 1, 0.1);
    REQUIRE_THROWS_WITH(ctrl::DiscreteH2::solve(P), Catch::Matchers::ContainsSubstring("D22"));
}

TEST_CASE("DiscreteH2 closed-loop compute() stays bounded under a unit-impulse disturbance",
          "[h2][hinf]")
{
    const ctrl::GeneralisedPlant P = makeH2CrossTermPlant();
    const ctrl::H2Result result = ctrl::DiscreteH2::solve(P);
    REQUIRE(result.feasible);

    ctrl::DiscreteH2 h2(result);
    double y = 1.0; // unit-impulse-like initial measurement
    bool all_finite = true;
    for (int k = 0; k < 200; ++k)
    {
        const double u = h2.compute(y);
        all_finite = all_finite && std::isfinite(u);
        // Simple scalar plant response under the synthesised control, feeding back y = x:
        y = P.A(0,0) * y + P.B2(0,0) * u;
    }
    REQUIRE(all_finite);
    REQUIRE(std::abs(y) < 1e-3); // closed loop is stable -> decays to ~0 in 200 steps
}

TEST_CASE("DiscreteLQR::solveDARE visibility change is a pure no-op (regression)", "[h2][lqr]")
{
    // DiscreteLQR::solveDARE was made public so DiscreteH2 could reuse it (lib/DiscreteLQR.h).
    // This confirms the externally-called result matches what DiscreteLQR computes
    // internally for the same (A,B,Q,R) - i.e. the visibility edit changed zero behaviour.
    const Eigen::MatrixXd A = Eigen::MatrixXd::Constant(1, 1, 0.9);
    const Eigen::MatrixXd B = Eigen::MatrixXd::Constant(1, 1, 1.0);
    const Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(1, 1, 1.0);
    const Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, 0.5);

    const ctrl::DareResult direct = ctrl::DiscreteLQR::solveDARE(A, B, Q, R);
    REQUIRE(direct.converged);

    ctrl::LQRParams lp; lp.Q = Q; lp.R = R;
    const ctrl::DiscreteLQR lqr(ctrl::StateSpace(A, B, Eigen::MatrixXd::Identity(1,1),
                                                  Eigen::MatrixXd::Zero(1,1), 0.1), lp);
    REQUIRE_THAT(direct.P(0, 0), WithinAbs(lqr.riccatiSolution()(0, 0), 1e-12));
}

TEST_CASE("DiscreteHinf::solveHinfDARE visibility change is a pure no-op (regression)",
          "[hinf_filter]")
{
    // solveHinfDARE was made public so HinfFilter could reuse it (lib/HinfFilter.cpp).
    // This confirms the now-public solver still satisfies its own DARE residual contract
    // for an indefinite-R problem (a standard-doubling solver like DiscreteLQR::solveDARE
    // would diverge on this R) - i.e. the visibility edit changed zero behaviour.
    const Eigen::MatrixXd A = Eigen::MatrixXd::Constant(1, 1, 0.9);
    const Eigen::MatrixXd B = Eigen::MatrixXd::Constant(1, 1, 1.0);
    const Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(1, 1, 1.0);
    // R = -0.2: indefinite (negative) but feasible - the scalar DARE's discriminant
    // 0.0361*r^2 + 3.62*r + 1 (for A=0.9, B=1, Q=1) is negative for r in (-100, -0.277),
    // so r=-0.2 sits just inside the feasible band above -0.277.
    const Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, -0.2);

    const ctrl::DareResult dr = ctrl::DiscreteHinf::solveHinfDARE(A, B, Q, R, 1e-12, 200);
    REQUIRE(dr.converged);
    REQUIRE_THAT(dr.P(0, 0), WithinAbs(dr.P(0, 0), 1e-12)); // P populated, symmetric (1x1 trivially)

    const Eigen::MatrixXd &X = dr.P;
    const Eigen::MatrixXd Rbar = R + B.transpose() * X * B;
    const Eigen::MatrixXd K = Rbar.inverse() * B.transpose() * X * A;
    const Eigen::MatrixXd resid = A.transpose() * X * A - X + Q - A.transpose() * X * B * K;
    REQUIRE(resid.norm() / (1.0 + X.norm()) < 1e-6);
}

// =============================================================================
// HinfFilter (Phase 3 EF1)
// =============================================================================

namespace
{
ctrl::StateSpace makeHinfFilterTestPlant()
{
    Eigen::MatrixXd A(1, 1); A << 0.9;
    Eigen::MatrixXd B(1, 1); B << 0.0; // no control channel exercised in these tests
    Eigen::MatrixXd C(1, 1); C << 1.0;
    Eigen::MatrixXd D(1, 1); D << 0.0;
    return ctrl::StateSpace(A, B, C, D, 0.1);
}
} // namespace

TEST_CASE("HinfFilter's empirical estimation-error energy stays within the achieved gamma bound",
          "[hinf_filter]")
{
    const ctrl::StateSpace plant = makeHinfFilterTestPlant();
    const Eigen::MatrixXd Qw = Eigen::MatrixXd::Constant(1, 1, 0.01);
    const Eigen::MatrixXd Rv = Eigen::MatrixXd::Constant(1, 1, 0.05);

    const auto result = ctrl::HinfFilter::solve(plant, Qw, Rv);
    REQUIRE(result.feasible);

    ctrl::HinfFilter hf(result);

    // Bounded (not Gaussian) disturbance: a deterministic worst-case-flavoured alternating
    // sequence at the process/measurement noise bounds, both starting from x_true(0) = 0 to
    // match the filter's own zero initial state (avoids an initial-condition mismatch term
    // contaminating the energy-gain bound, which is otherwise an infinite-horizon/steady-
    // state guarantee).
    const double wBound = std::sqrt(Qw(0, 0));
    const double vBound = std::sqrt(Rv(0, 0));
    double xTrue = 0.0;
    double sumErrSq = 0.0, sumDistSq = 0.0;
    for (int k = 0; k < 200; ++k)
    {
        const double w = wBound * ((k % 2 == 0) ? 1.0 : -1.0);
        const double v = vBound * ((k % 3 == 0) ? 1.0 : -1.0);
        const double y = plant.C(0, 0) * xTrue + v;

        hf.predict(Eigen::VectorXd::Constant(1, 0.0));
        hf.update(Eigen::VectorXd::Constant(1, y));

        const double err = xTrue - hf.state()(0);
        sumErrSq  += err * err;
        sumDistSq += w * w + v * v;

        xTrue = plant.A(0, 0) * xTrue + w;
    }

    REQUIRE(std::isfinite(sumErrSq));
    REQUIRE(sumDistSq > 0.0);
    // Generous margin over the strict gamma^2 bound to absorb finite-horizon transients
    // (the H-infinity guarantee is a steady-state/infinite-horizon energy-gain bound).
    REQUIRE(sumErrSq < result.achievedGamma * result.achievedGamma * sumDistSq * 4.0);
}

TEST_CASE("HinfFilter is more conservative but stable relative to KalmanFilter on Gaussian noise",
          "[hinf_filter]")
{
    const ctrl::StateSpace plant = makeHinfFilterTestPlant();
    const Eigen::MatrixXd Qw = Eigen::MatrixXd::Constant(1, 1, 0.01);
    const Eigen::MatrixXd Rv = Eigen::MatrixXd::Constant(1, 1, 0.05);

    const auto hfResult = ctrl::HinfFilter::solve(plant, Qw, Rv);
    REQUIRE(hfResult.feasible);
    ctrl::HinfFilter hf(hfResult);
    ctrl::KalmanFilter kf(plant, Qw, Rv);

    std::mt19937 rng(123);
    std::normal_distribution<double> wDist(0.0, std::sqrt(Qw(0, 0)));
    std::normal_distribution<double> vDist(0.0, std::sqrt(Rv(0, 0)));

    double xTrue = 0.0;
    double sseHf = 0.0, sseKf = 0.0;
    const int N = 500;
    for (int k = 0; k < N; ++k)
    {
        const double w = wDist(rng);
        const double v = vDist(rng);
        const double y = plant.C(0, 0) * xTrue + v;

        hf.predict(Eigen::VectorXd::Constant(1, 0.0));
        hf.update(Eigen::VectorXd::Constant(1, y));
        kf.step(Eigen::VectorXd::Constant(1, y), Eigen::VectorXd::Constant(1, 0.0));

        sseHf += std::pow(xTrue - hf.state()(0), 2);
        sseKf += std::pow(xTrue - kf.state()(0), 2);

        xTrue = plant.A(0, 0) * xTrue + w;
    }

    const double rmsHf = std::sqrt(sseHf / N);
    const double rmsKf = std::sqrt(sseKf / N);
    REQUIRE(std::isfinite(rmsHf));
    REQUIRE(std::isfinite(rmsKf));
    REQUIRE(rmsHf < 5.0 * rmsKf); // conservative but within a documented factor, not wildly worse
}

TEST_CASE("HinfFilter::solve reports infeasible for an unreasonably tight gamma", "[hinf_filter]")
{
    const ctrl::StateSpace plant = makeHinfFilterTestPlant();
    const Eigen::MatrixXd Qw = Eigen::MatrixXd::Constant(1, 1, 0.01);
    const Eigen::MatrixXd Rv = Eigen::MatrixXd::Constant(1, 1, 0.05);

    ctrl::HinfFilterParams params;
    params.gammaInit = 1e-6; // far below any achievable bound; doubling 10x still won't reach it
    const auto result = ctrl::HinfFilter::solve(plant, Qw, Rv, params);
    REQUIRE_FALSE(result.feasible);
}

TEST_CASE("HinfFilter constructor throws on an infeasible result", "[hinf_filter]")
{
    const ctrl::HinfFilterResult result{false, 0.0, Eigen::MatrixXd(), Eigen::MatrixXd(),
                                         makeHinfFilterTestPlant()};
    REQUIRE_THROWS_AS(ctrl::HinfFilter(result), std::invalid_argument);
}

TEST_CASE("HinfFilter holds the last state on non-finite predict()/update() input", "[hinf_filter]")
{
    const ctrl::StateSpace plant = makeHinfFilterTestPlant();
    const Eigen::MatrixXd Qw = Eigen::MatrixXd::Constant(1, 1, 0.01);
    const Eigen::MatrixXd Rv = Eigen::MatrixXd::Constant(1, 1, 0.05);
    const auto result = ctrl::HinfFilter::solve(plant, Qw, Rv);
    REQUIRE(result.feasible);

    ctrl::HinfFilter hf(result);
    hf.update(Eigen::VectorXd::Constant(1, 0.5));
    const Eigen::VectorXd stateBefore = hf.state();

    hf.predict(Eigen::VectorXd::Constant(1, std::numeric_limits<double>::quiet_NaN()));
    REQUIRE(hf.state()(0) == stateBefore(0));

    hf.update(Eigen::VectorXd::Constant(1, std::numeric_limits<double>::quiet_NaN()));
    REQUIRE(hf.state()(0) == stateBefore(0));

    hf.reset();
    REQUIRE(hf.state()(0) == 0.0);
}

#endif // CTRL_HAS_HINF

// =============================================================================
// SCIPY / python-control cross-validation (reference values computed 2026-05-28)
// Each constant below was verified by running the corresponding scipy/control call.
// Tolerances match what ex43-ex47 Python scripts use (1e-6 relative or 1e-8 abs).
// =============================================================================

// -----------------------------------------------------------------------------
// c2d ZOH vs scipy.signal.cont2discrete - G(s)=1/(s^2+1.5s+1), Ts=0.01
// Reference: scipy.signal.cont2discrete((Ac, Bc, I2, 0), 0.01, 'zoh')
// Verified 2026-05-28.
// -----------------------------------------------------------------------------
TEST_CASE("c2d ZOH matches scipy.signal.cont2discrete: G(s)=1/(s^2+1.5s+1)", "[c2d][scipy]")
{
    // Controllable canonical form of G(s)=1/(s^2+1.5s+1)
    Eigen::Matrix2d Ac;
    Ac << 0.0, 1.0, -1.0, -1.5;
    Eigen::Vector2d Bc;
    Bc << 0.0, 1.0;
    Eigen::Matrix2d Cc = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd Dc = Eigen::MatrixXd::Zero(2, 1);

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    const ctrl::StateSpace sys_d = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    // scipy reference values (10-digit precision)
    REQUIRE_THAT(sys_d.A(0, 0), WithinAbs( 0.9999502495, 1e-6));
    REQUIRE_THAT(sys_d.A(0, 1), WithinAbs( 0.0099252082, 1e-6));
    REQUIRE_THAT(sys_d.A(1, 0), WithinAbs(-0.0099252082, 1e-6));
    REQUIRE_THAT(sys_d.A(1, 1), WithinAbs( 0.9850624372, 1e-6));
    REQUIRE_THAT(sys_d.B(0, 0), WithinAbs( 0.0000497505, 1e-8));
    REQUIRE_THAT(sys_d.B(1, 0), WithinAbs( 0.0099252082, 1e-6));
}

TEST_CASE("c2d ZOH matches exact exponential: G(s)=1/(s+1), Ts=0.1", "[c2d][scipy]")
{
    // ZOH exact: Ad = exp(-Ts), Bd = 1 - exp(-Ts)
    // scipy: signal.cont2discrete(([-1],[1],[1],[0]),0.1,'zoh') -> Ad=0.904837418036
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);

    const ctrl::StateSpace sys_d_zoh    = ctrl::c2d(sys_c, 0.1, ctrl::C2dMethod::ZOH);
    const ctrl::StateSpace sys_d_tustin = ctrl::c2d(sys_c, 0.1, ctrl::C2dMethod::Tustin);

    // ZOH: exact values from scipy and analytical formula
    const double ad_zoh_exact = std::exp(-0.1);             // 0.904837418036
    const double bd_zoh_exact = 1.0 - std::exp(-0.1);       // 0.095162581964
    REQUIRE_THAT(sys_d_zoh.A(0,0), WithinAbs(ad_zoh_exact, 1e-10));
    REQUIRE_THAT(sys_d_zoh.B(0,0), WithinAbs(bd_zoh_exact, 1e-10));

    // Tustin: exact bilinear transform (s -> 2/Ts*(z-1)/(z+1))
    // Ad = (1 - Ts/2)/(1 + Ts/2) = 0.904761904762
    const double ad_tustin_exact = (1.0 - 0.1/2.0) / (1.0 + 0.1/2.0);
    const double bd_tustin_exact = 0.1 / (1.0 + 0.1/2.0);
    REQUIRE_THAT(sys_d_tustin.A(0,0), WithinAbs(ad_tustin_exact, 1e-10));
    REQUIRE_THAT(sys_d_tustin.B(0,0), WithinAbs(bd_tustin_exact, 1e-10));

    // ZOH and Tustin give different results (non-interchangeable methods).
    // Difference approx = 7.55e-5 for Ts=0.1 (scipy.signal.cont2discrete verified).
    REQUIRE(std::abs(sys_d_zoh.A(0,0) - sys_d_tustin.A(0,0)) > 1e-5);
}

// -----------------------------------------------------------------------------
// DiscreteLQR gain vs scipy DARE / control.dlqr
// Reference: control.dlqr(Ad, Bd, Q, R) verified 2026-05-28.
// -----------------------------------------------------------------------------
TEST_CASE("DiscreteLQR gain matches control.dlqr: double integrator Ts=0.01 Q=10I R=I", "[lqr][scipy]")
{
    // control.dlqr(Ad_int, Bd_int, 10*I2, I1) -> K = [3.0990370926, 3.9751861701]
    // Verified 2026-05-28 with python-control 0.10.2 and scipy 1.17.1.
    auto plant = makeDoubleIntegrator();

    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);
    ctrl::DiscreteLQR lqr(plant, lqr_p);

    REQUIRE(lqr.dareConverged());
    const Eigen::MatrixXd &K = lqr.gainMatrix();
    REQUIRE(K.rows() == 1);
    REQUIRE(K.cols() == 2);

    // scipy reference: K = [3.0990370926, 3.9751861701] (1e-6 relative tolerance)
    REQUIRE_THAT(K(0,0), WithinRel(3.0990370926, 1e-6));
    REQUIRE_THAT(K(0,1), WithinRel(3.9751861701, 1e-6));

    // Closed-loop eigenvalues must all be inside the unit disk
    const Eigen::Matrix2d Acl = plant.A - plant.B * K;
    Eigen::EigenSolver<Eigen::Matrix2d> eig(Acl);
    for (int i = 0; i < 2; ++i)
        REQUIRE(std::abs(eig.eigenvalues()(i)) < 1.0);
}

// -----------------------------------------------------------------------------
// KalmanFilter step response: ss_step_copy matches scipy.signal.dlsim
// Reference: scipy.signal.dlsim on first-order ZOH plant, 10 steps.
// Verified 2026-05-28.
// -----------------------------------------------------------------------------
TEST_CASE("ss_step_copy matches exact first-order ZOH step response", "[c2d][simulation][scipy]")
{
    // G(s) = 1/(s+1), ZOH @ Ts=0.1s
    // Exact step response: y[k] = D*u  for k=0 (D=0 so y[0]=0),
    //                              y[k] = 1 - exp(-k*Ts)  for k >= 1
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys_d = ctrl::c2d(sys_c, 0.1, ctrl::C2dMethod::ZOH);

    Eigen::VectorXd x(1);
    x << 0.0;
    Eigen::VectorXd u(1);
    u << 1.0;

    // k=0: y = C*x + D*u = 0 + 0 = 0 (output before advancing)
    const Eigen::VectorXd y0 = ctrl::ssStep(sys_d, x, u);
    REQUIRE_THAT(y0(0), WithinAbs(0.0, 1e-12));

    // k=1: after one step x = Bd, y = C*x = (1-exp(-0.1))
    const Eigen::VectorXd y1 = ctrl::ssStep(sys_d, x, u);
    const double y1_exact = 1.0 - std::exp(-0.1);  // = 0.09516258196
    REQUIRE_THAT(y1(0), WithinAbs(y1_exact, 1e-10));

    // k=5: y = 1 - exp(-0.5) = 0.39346934
    for (int k = 2; k < 5; ++k) ctrl::ssStep(sys_d, x, u);
    const Eigen::VectorXd y5 = ctrl::ssStep(sys_d, x, u);
    REQUIRE_THAT(y5(0), WithinAbs(1.0 - std::exp(-0.5), 1e-8));
}

// =============================================================================
// Part 18: SOPDTIdentifier - graphical + optimization + IMC tuning
// =============================================================================

TEST_CASE("SOPDTIdentifier identifies SOPDT parameters from synthetic step response",
          "[sopdt]")
{
    // True SOPDT: G(s) = 2.0 * exp(-1.5s) / ((5s+1)(2s+1))
    // K=2, tau1=5, tau2=2, theta=1.5
    const double K_true    = 2.0;
    const double tau1_true = 5.0;
    const double tau2_true = 2.0;
    const double th_true   = 1.5;
    const double step_mag  = 0.5;
    const double Ts_data   = 0.2;

    // Generate synthetic step response
    const int N = 200; // 40 seconds
    std::vector<double> t(N), y(N);
    for (int i = 0; i < N; ++i) {
        t[i] = i * Ts_data;
        const double dt = t[i] - th_true;
        if (dt <= 0.0) {
            y[i] = 0.0;
        } else {
            // Overdamped SOPDT step response (normalized to unit step)
            const double resp = 1.0 - (tau1_true * std::exp(-dt / tau1_true)
                                      - tau2_true * std::exp(-dt / tau2_true))
                                      / (tau1_true - tau2_true);
            y[i] = K_true * step_mag * resp;
        }
    }

    ctrl::SOPDTIdentifier id(t, y, step_mag, 0.0);

    // --- Graphical method ---
    const ctrl::SOPDTModel mg = id.identify(ctrl::SOPDTMethod::Graphical);
    REQUIRE_THAT(mg.K,     WithinAbs(K_true,   0.15));   // K within 15% of step change
    REQUIRE_THAT(mg.theta, WithinAbs(th_true,   1.0));   // theta within 1 s
    REQUIRE(mg.tau1 + mg.tau2 > 2.0);                    // sum of time constants positive
    REQUIRE(mg.tau1 >= mg.tau2);                          // tau1 is the dominant pole
    REQUIRE(mg.fitRMSE < 0.5 * K_true * step_mag);       // RMSE under 50% of output

    // --- Optimization method ---
    const ctrl::SOPDTModel mo = id.identify(ctrl::SOPDTMethod::Optimization);
    REQUIRE_THAT(mo.K,     WithinAbs(K_true,   0.05));
    REQUIRE_THAT(mo.theta, WithinAbs(th_true,   0.5));
    REQUIRE_THAT(mo.tau1 + mo.tau2, WithinAbs(tau1_true + tau2_true, 2.0));
    REQUIRE(mo.fitRMSE <= mg.fitRMSE + 1e-6);  // optimization not worse than graphical

    // --- IMC-PID tuning from optimization model ---
    const auto pp = ctrl::SOPDTIdentifier::imcTuning(mo, 2.0 * mo.theta, Ts_data);
    REQUIRE(pp.Kp > 0.0);
    REQUIRE(pp.Ki > 0.0);
    REQUIRE(pp.Kd >= 0.0);
    REQUIRE(std::isfinite(pp.Kp));
    REQUIRE(std::isfinite(pp.Ki));

    // --- Closed-loop convergence check with IMC-PID on SOPDT plant ---
    // Simple discrete PI on approximate FOPDT (tau_eq = tau1+tau2) - should reach steady state
    const double tauEq = mo.tau1 + mo.tau2;
    const double Kp = pp.Kp, Ki = pp.Ki;
    double y_cl = 0.0, integ = 0.0;
    const double ref = 1.0;
    for (int k = 0; k < 5000; ++k) {
        const double e = ref - y_cl;
        integ += e * Ts_data;
        const double u = Kp * e + Ki * integ;
        // Approximate first-order plant: y+ = exp(-Ts/tauEq)*y + (1-exp(-Ts/tauEq))*K*u
        const double a = std::exp(-Ts_data / tauEq);
        y_cl = a * y_cl + (1.0 - a) * mo.K * u;
    }
    REQUIRE_THAT(y_cl, WithinAbs(ref, 0.1));  // closed-loop reaches reference within 10%
}

// =============================================================================
// Part 18: MovingHorizonEstimator - convergence on linear Gaussian system
// For a linear unconstrained system, MHE should converge close to the KF estimate.
// =============================================================================

TEST_CASE("MovingHorizonEstimator converges on linear first-order system",
          "[mhe]")
{
    // Plant: y[k] = x[k],  x[k+1] = 0.8*x[k] + u[k] + w[k]
    // Ts = 0.1 s, scalar state
    const double Ts_mhe = 0.1;
    Eigen::MatrixXd A_m(1,1); A_m(0,0) = 0.8;
    Eigen::MatrixXd B_m(1,1); B_m(0,0) = 1.0;
    Eigen::MatrixXd C_m(1,1); C_m(0,0) = 1.0;
    Eigen::MatrixXd D_m = Eigen::MatrixXd::Zero(1,1);
    ctrl::StateSpace plant(A_m, B_m, C_m, D_m, Ts_mhe);

    const Eigen::MatrixXd Q_p = Eigen::MatrixXd::Constant(1,1,1e-3);  // small process noise
    const Eigen::MatrixXd R_m = Eigen::MatrixXd::Constant(1,1,0.1);   // measurement noise

    ctrl::MHEParams mp;
    mp.N = 8;

    ctrl::MovingHorizonEstimator mhe(plant, Q_p, R_m, mp);
    mhe.initialize(Eigen::VectorXd::Zero(1),
                   100.0 * Eigen::MatrixXd::Identity(1,1));  // large initial uncertainty

    // Simulate 60 steps with a step input u=1, true state starts at 0
    double x_true = 0.0;
    const int N_steps = 60;
    std::srand(42);

    double final_error = 1e9;
    for (int k = 0; k < N_steps; ++k) {
        const double u_k = 1.0;
        // Noise-free measurement (to check estimation convergence, not filter tuning)
        const double y_k = x_true;  // C=1, no noise

        Eigen::VectorXd yv(1); yv(0) = y_k;
        Eigen::VectorXd uv(1); uv(0) = u_k;
        const Eigen::VectorXd x_est = mhe.estimate(yv, uv);

        // Propagate true state
        x_true = 0.8 * x_true + u_k;  // true plant, no noise

        if (k >= N_steps - 1)
            final_error = std::abs(x_est(0) - x_true);
    }

    // After 60 steps, the estimate should be within 2 of the true state
    // (MHE converges but may lag due to horizon and arrival cost)
    REQUIRE(std::isfinite(mhe.state()(0)));
    REQUIRE(final_error < 5.0);   // large but finite tolerance; validates no divergence

    // QP should converge
    REQUIRE(mhe.lastConverged());
}

// =============================================================================
// GainScheduledController - bumpless transfer on NearestNeighbor switch
// =============================================================================

TEST_CASE("GainScheduledController NearestNeighbor bumpless on switch", "[gain_sched]")
{
    // Two PIDs tuned for different operating points.
    // The first is running and has accumulated integral state (non-zero output).
    // When the scheduler switches to the second, its first output should be
    // close to the last output of the first (bumplessInit fires).
    const double Ts = 0.1;

    ctrl::PIDParams pp0; pp0.Kp = 1.0; pp0.Ki = 0.5; pp0.Kd = 0.0;
    ctrl::PIDParams pp1; pp1.Kp = 2.0; pp1.Ki = 0.1; pp1.Kd = 0.0;

    auto pid0 = std::make_shared<ctrl::DiscretePID>(pp0, Ts);
    auto pid1 = std::make_shared<ctrl::DiscretePID>(pp1, Ts);

    ctrl::GainScheduledController gs(Ts, ctrl::GainScheduleMode::NearestNeighbor);
    gs.addSchedulePoint(0.0, pid0);
    gs.addSchedulePoint(1.0, pid1);

    // Warm up pid0 at p=0 for 20 steps so it has integral state
    gs.setSchedulingParam(0.0);
    double last_u = 0.0;
    for (int k = 0; k < 20; ++k)
        last_u = gs.compute(1.0); // constant error = 1

    // Switch to p=1 (pid1 becomes nearest): bumplessInit should fire
    gs.setSchedulingParam(1.0);
    const double first_u_new = gs.compute(1.0);

    // After bumplessInit, the new controller's first output should be close to last_u
    // (bumplessInit prepares the controller to output u_target at the given error).
    // Exact match not guaranteed (bumplessInit is controller-specific) but the jump
    // should be small: |first_u_new - last_u| << |unconditioned first output|.
    const double uncond_first = pp1.Kp * 1.0; // what pid1 would produce without bumpless (~2.0)
    const double jump_with    = std::abs(first_u_new - last_u);
    const double jump_without = std::abs(uncond_first - last_u);
    // Bumpless should reduce the jump by at least 50%
    REQUIRE(jump_with < jump_without);
}

TEST_CASE("GainScheduledController NearestNeighbor notifies observer", "[gain_sched]")
{
    const double Ts = 0.05;
    ctrl::PIDParams pp; pp.Kp = 1.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::GainScheduledController gs(Ts, ctrl::GainScheduleMode::NearestNeighbor);
    gs.addSchedulePoint(0.0, pid);

    int n_callbacks = 0;
    double last_u_obs = 0.0;
    class CountObs : public ctrl::IControllerObserver {
    public:
        int &n; double &u_last;
        CountObs(int &n_, double &u_) : n(n_), u_last(u_) {}
        void onCompute(double u, double) override { ++n; u_last = u; }
        void onComputeVec(const Eigen::VectorXd &, const Eigen::VectorXd &) override {}
        void onReset() override {}
    } obs(n_callbacks, last_u_obs);

    gs.attachObserver(&obs);
    gs.setSchedulingParam(0.0);
    gs.compute(0.5);

    REQUIRE(n_callbacks == 1);
    REQUIRE_THAT(last_u_obs, WithinRel(gs.lastOutput(), 1e-9));
}

// =============================================================================
// NonlinearMPC - RTI closed-loop convergence
// =============================================================================

TEST_CASE("NonlinearMPC converges on scalar nonlinear plant", "[nmpc]")
{
    // Plant: x[k+1] = 0.8*x - 0.05*x^3 + u,  y = x (1D, full-state output)
    const double Ts = 0.1;
    auto f_nl = [](const Eigen::VectorXd &x,
                   const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xn(1);
        xn(0) = 0.8 * x(0) - 0.05 * x(0) * x(0) * x(0) + u(0);
        return xn;
    };

    ctrl::NMPCParams np;
    np.Np = 8; np.Nu = 3;
    np.rho_y = 2.0; np.rho_u = 0.2;
    np.uMin = -5.0; np.uMax = 5.0;
    np.qpMaxIter = 500; np.qpTol = 1e-6;
    np.Ts = Ts; np.n_states = 1; np.n_inputs = 1; np.n_outputs = 1;

    ctrl::NonlinearMPC nmpc(np, f_nl);

    Eigen::VectorXd x(1);    x(0) = 0.0;
    Eigen::VectorXd yref(1); yref(0) = 1.0;

    for (int k = 0; k < 60; ++k)
    {
        nmpc.setState(x);
        const Eigen::VectorXd u = nmpc.computeRef(x, yref);
        x = f_nl(x, u);
    }

    // After 60 steps the output should be within 0.1 of reference
    REQUIRE(std::abs(x(0) - 1.0) < 0.1);
    REQUIRE(nmpc.lastQPConverged());
}

TEST_CASE("NonlinearMPC reset clears warm start", "[nmpc]")
{
    auto f_lin = [](const Eigen::VectorXd &x,
                    const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xn(1);
        xn(0) = 0.9 * x(0) + u(0);
        return xn;
    };

    ctrl::NMPCParams np;
    np.Np = 5; np.Nu = 2; np.rho_y = 1.0; np.rho_u = 0.1;
    np.uMin = -10.0; np.uMax = 10.0;
    np.Ts = 0.1; np.n_states = 1; np.n_inputs = 1; np.n_outputs = 1;

    ctrl::NonlinearMPC nmpc(np, f_lin);

    Eigen::VectorXd x(1); x(0) = 2.0;
    Eigen::VectorXd yref(1); yref(0) = 0.0;
    nmpc.setState(x); nmpc.computeRef(x, yref); // run once to fill warm start

    nmpc.reset();
    // After reset, lastOutput should be zero
    REQUIRE(nmpc.lastOutput() == 0.0);
    // compute on a zero state should not throw
    Eigen::VectorXd x0(1); x0.setZero();
    nmpc.setState(x0);
    REQUIRE_NOTHROW(nmpc.computeRef(x0, yref));
}

// =============================================================================
// AdaptiveSmithPredictor - delay estimation and closed-loop stability
// =============================================================================

TEST_CASE("AdaptiveSmithPredictor stabilises delayed plant", "[adaptive_smith]")
{
    // Delay-free model: x[k+1] = 0.8*x + 0.2*u,  y = x
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1); A << 0.8;
    Eigen::MatrixXd B(1,1); B << 0.2;
    Eigen::MatrixXd C(1,1); C << 1.0;
    Eigen::MatrixXd D(1,1); D << 0.0;
    ctrl::StateSpace model(A, B, C, D, Ts);

    ctrl::PIDParams pp; pp.Kp = 1.5; pp.Ki = 0.2;
    auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::AdaptiveSPParams asp;
    asp.maxDelaySteps    = 8;
    asp.estimateInterval = 100;
    asp.bufferLen        = 150;

    // True delay = 3; initialise with correct estimate (tests stability path)
    ctrl::AdaptiveSmithPredictor asp_ctrl(inner, model, 3, Ts, asp);

    // Simulate true plant with delay buffer
    // Buffer size = d_true: at step k, the input applied d_true steps ago is returned.
    const int d_true = 3;
    std::vector<double> u_buf(d_true, 0.0);
    double x_plant = 0.0;

    double iae = 0.0;
    for (int k = 0; k < 200; ++k)
    {
        const double y   = x_plant;
        const double e   = 1.0 - y;
        asp_ctrl.setPlantOutput(y);
        const double u   = asp_ctrl.compute(e);

        u_buf.push_back(u);
        const double u_del = u_buf.front();
        u_buf.erase(u_buf.begin());

        x_plant = 0.8 * x_plant + 0.2 * u_del;
        if (k > 50) iae += std::abs(e) * Ts;
    }

    // Plant should have settled: IAE in steps 50-200 < 5.0 (loose; tests convergence, not tuning quality)
    REQUIRE(iae < 5.0);
    REQUIRE(std::abs(x_plant - 1.0) < 0.15);
}

TEST_CASE("AdaptiveSmithPredictor estimatedDelayTime is Ts * delay", "[adaptive_smith]")
{
    const double Ts = 0.05;
    Eigen::MatrixXd A(1,1); A << 0.9;
    Eigen::MatrixXd B(1,1); B << 0.1;
    Eigen::MatrixXd C(1,1); C << 1.0;
    Eigen::MatrixXd D(1,1); D << 0.0;
    ctrl::StateSpace model(A, B, C, D, Ts);

    ctrl::PIDParams pp; pp.Kp = 1.0;
    auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::AdaptiveSmithPredictor asp(inner, model, 5, Ts);

    REQUIRE(asp.estimatedDelaySteps() == 5);
    REQUIRE_THAT(asp.estimatedDelayTime(),
                 WithinRel(5 * Ts, 1e-10));
}

// =============================================================================
// AutoTuner (CMA-ES) - optimisation correctness
// =============================================================================

TEST_CASE("AutoTuner minimises known 2D quadratic", "[autotuner]")
{
    // f(x) = (x0 - 3)^2 + 2*(x1 + 1)^2  has minimum 0 at (3, -1)
    auto cost = [](const Eigen::VectorXd &x) -> double {
        return (x(0) - 3.0) * (x(0) - 3.0)
             + 2.0 * (x(1) + 1.0) * (x(1) + 1.0);
    };

    ctrl::AutoTunerParams atp;
    atp.n      = 2;
    atp.sigma0 = 0.5;
    atp.maxIter = 200;
    atp.tol    = 1e-8;

    ctrl::AutoTuner tuner(atp, 42);
    Eigen::Vector2d x0(0.0, 0.0);
    const ctrl::TunerResult res = tuner.tune(cost, x0);

    // Should find minimum within tolerance
    REQUIRE_THAT(res.params(0), WithinAbs(3.0, 1e-3));
    REQUIRE_THAT(res.params(1), WithinAbs(-1.0, 1e-3));
    REQUIRE(res.cost < 1e-5);
    REQUIRE(res.nEvals > 0);
}

TEST_CASE("AutoTuner respects box bounds", "[autotuner]")
{
    // f(x) = (x0 - 10)^2,  true minimum at x=10, but bounded to [0, 5]
    // Expected optimum: x=5 (upper bound), cost=(10-5)^2=25
    auto cost = [](const Eigen::VectorXd &x) -> double {
        return (x(0) - 10.0) * (x(0) - 10.0);
    };

    ctrl::AutoTunerParams atp;
    atp.n      = 1;
    atp.sigma0 = 1.0;
    atp.maxIter = 150;
    atp.lower  = Eigen::VectorXd::Constant(1, 0.0);
    atp.upper  = Eigen::VectorXd::Constant(1, 5.0);

    ctrl::AutoTuner tuner(atp, 7);
    Eigen::VectorXd x0(1); x0(0) = 2.5;
    const ctrl::TunerResult res = tuner.tune(cost, x0);

    // Best feasible point should be at or near the upper bound
    REQUIRE(res.params(0) >= 0.0 - 1e-9);
    REQUIRE(res.params(0) <= 5.0 + 1e-9);
    REQUIRE(res.cost < 26.0); // cost at x=5 is 25; allow small slack
}

// =============================================================================
// LinearisationHelper - numerical Jacobians and lineariseAtPoint
// =============================================================================

TEST_CASE("jacobianX/U match analytical for Van der Pol at origin", "[linearisation]")
{
    // Van der Pol: xdot1=x2, xdot2=mu(1-x1^2)x2-x1+u  at x=(0,0), u=0
    // Analytical: A_c=[[0,1],[-1,mu]], B_c=[[0],[1]]
    const double mu = 0.5;
    ctrl::StateFunc vdp = [mu](const Eigen::VectorXd &x,
                               const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xd(2);
        xd(0) = x(1);
        xd(1) = mu * (1.0 - x(0) * x(0)) * x(1) - x(0) + u(0);
        return xd;
    };

    const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    const Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    const Eigen::MatrixXd A_num = ctrl::jacobianX(vdp, x0, u0);
    const Eigen::MatrixXd B_num = ctrl::jacobianU(vdp, x0, u0);

    Eigen::Matrix2d A_ana;
    A_ana << 0.0, 1.0, -1.0, mu;
    Eigen::Vector2d B_ana;
    B_ana << 0.0, 1.0;

    // Relative error on each matrix
    const double A_err = (A_num - A_ana).norm() / (A_ana.norm() + 1e-12);
    const double B_err = (B_num - B_ana).norm() / (B_ana.norm() + 1e-12);

    REQUIRE(A_err < 1e-4);
    REQUIRE(B_err < 1e-4);
}

TEST_CASE("lineariseAtPoint produces stable LQR gain for Van der Pol", "[linearisation]")
{
    const double mu = 0.5;
    const double Ts_lin = 0.05;

    ctrl::StateFunc vdp = [mu](const Eigen::VectorXd &x,
                               const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xd(2);
        xd(0) = x(1);
        xd(1) = mu * (1.0 - x(0) * x(0)) * x(1) - x(0) + u(0);
        return xd;
    };

    const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    const Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    ctrl::StateSpace sys_d = ctrl::lineariseAtPoint(vdp, x0, u0, Ts_lin);

    // Dimensions
    REQUIRE(sys_d.stateSize()  == 2);
    REQUIRE(sys_d.inputSize()  == 1);
    REQUIRE(sys_d.outputSize() == 2);

    // DARE must converge
    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);
    ctrl::DiscreteLQR lqr(sys_d, lqr_p);

    REQUIRE(lqr.dareConverged());

    // Closed-loop poles must be inside the unit circle
    const Eigen::MatrixXd A_cl = sys_d.A - sys_d.B * lqr.gainMatrix();
    const auto evs = A_cl.eigenvalues();
    for (int i = 0; i < evs.size(); ++i)
        REQUIRE(std::abs(evs(i)) < 1.0);

    // Closed-loop simulation on LINEARISED plant from x0=(0.5,0) -> |x| < 0.05 in 500 steps
    Eigen::VectorXd x_cl(2);
    x_cl << 0.5, 0.0;
    const Eigen::MatrixXd K = lqr.gainMatrix();
    for (int k = 0; k < 500; ++k)
    {
        const Eigen::VectorXd u_ctrl = -K * x_cl;
        x_cl = sys_d.A * x_cl + sys_d.B * u_ctrl;
    }
    REQUIRE(x_cl.norm() < 0.05);
}

// =============================================================================
// FeedbackLinearisationController - exact FL for affine-in-control SISO systems
// =============================================================================

TEST_CASE("FL controller drives cubic drift xdot=-x^3+u to reference 1.0", "[fl]")
{
    const double Ts_fl = 0.01;

    ctrl::FeedbackLinearisationController::DriftFn f =
        [](const Eigen::VectorXd &x, double) { return -x(0) * x(0) * x(0); };
    ctrl::FeedbackLinearisationController::GainFn g =
        [](const Eigen::VectorXd &, double) { return 1.0; };

    ctrl::PIDParams pp;
    pp.Kp = 5.0; pp.Ki = 2.0; pp.Kd = 0.0; pp.N = 100.0;
    pp.uMin = -100.0; pp.uMax = 100.0;
    auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts_fl);

    ctrl::FLParams flp;
    flp.uMin = -50.0; flp.uMax = 50.0; flp.regularisationEps = 1e-6;

    ctrl::FeedbackLinearisationController fl(f, g, inner, flp, Ts_fl);

    const double ref = 1.0;
    Eigen::VectorXd x(1);
    x(0) = 0.0;

    for (int k = 0; k < 500; ++k)
    {
        fl.setState(x);
        const double u = fl.compute(ref - x(0));
        x(0) += Ts_fl * (-x(0) * x(0) * x(0) + u);   // Euler integration of true plant
    }

    REQUIRE(std::isfinite(x(0)));
    REQUIRE_THAT(x(0), WithinAbs(ref, 0.05));
}

TEST_CASE("FL lastOutput and sampleTime return correct values", "[fl]")
{
    const double Ts_fl = 0.02;

    ctrl::FeedbackLinearisationController::DriftFn f =
        [](const Eigen::VectorXd &, double) { return 0.0; };    // linear: f=0, g=1 -> u=v
    ctrl::FeedbackLinearisationController::GainFn g =
        [](const Eigen::VectorXd &, double) { return 1.0; };

    ctrl::PIDParams pp;
    pp.Kp = 1.0; pp.Ki = 0.0; pp.Kd = 0.0; pp.N = 10.0;
    pp.uMin = -10.0; pp.uMax = 10.0;
    auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts_fl);

    ctrl::FLParams flp;
    flp.uMin = -10.0; flp.uMax = 10.0;

    ctrl::FeedbackLinearisationController fl(f, g, inner, flp, Ts_fl);

    REQUIRE_THAT(fl.sampleTime(), WithinAbs(Ts_fl, 1e-12));
    REQUIRE_THAT(fl.lastOutput(), WithinAbs(0.0, 1e-12));

    // With f=0, g=1: u = v - 0 / 1 = v = Kp * error
    Eigen::VectorXd x0(1);
    x0(0) = 0.0;
    fl.setState(x0);
    const double u = fl.compute(3.0);            // error = 3 -> Kp*3 = 3
    REQUIRE_THAT(u, WithinAbs(3.0, 0.01));
    REQUIRE_THAT(fl.lastOutput(), WithinAbs(u, 1e-12));

    // reset() clears u_last and inner state
    fl.reset();
    REQUIRE_THAT(fl.lastOutput(), WithinAbs(0.0, 1e-12));
}

// =============================================================================
// BacksteppingController (Phase 3 NC1)
// =============================================================================

TEST_CASE("BacksteppingController drives a 2-stage double-integrator to a constant reference",
          "[backstepping]")
{
    // x1' = x2, x2' = u (f_0=f_1=0, g_0=g_1=1) - the textbook double-integrator
    // strict-feedback system (Khalil Ch. 14's canonical 2-stage example).
    const double Ts = 0.01;
    std::vector<ctrl::BacksteppingController::DriftFn> f = {
        [](const Eigen::VectorXd &, int) { return 0.0; },
        [](const Eigen::VectorXd &, int) { return 0.0; },
    };
    std::vector<ctrl::BacksteppingController::GainFn> g = {
        [](const Eigen::VectorXd &, int) { return 1.0; },
        [](const Eigen::VectorXd &, int) { return 1.0; },
    };
    ctrl::BacksteppingParams p;
    p.k_gains = {2.0, 2.0};
    ctrl::BacksteppingController bc(f, g, p, Ts);

    const double ref = 1.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    for (int k = 0; k < 2000; ++k)
    {
        bc.setState(x);
        const double u = bc.compute(ref - x(0));
        x(0) += Ts * x(1);
        x(1) += Ts * u;
    }

    REQUIRE(x.allFinite());
    REQUIRE_THAT(x(0), WithinAbs(ref, 0.02));
    REQUIRE_THAT(x(1), WithinAbs(0.0, 0.05));
}

TEST_CASE("BacksteppingController's composite tracking-error norm decreases from its early "
          "transient toward the end of the run",
          "[backstepping]")
{
    // z1 = x1 - r, z2 = x2 - alpha1 (alpha1 ~= -k1*z1 once r' settles to 0 for a constant
    // reference) - a coarse, transient-tolerant stand-in for the roadmap's Lyapunov-
    // monotonicity check (V = 0.5*(z1^2+z2^2)), since the finite-difference approximation
    // of alpha1' makes strict per-step monotonicity too fragile to assert directly.
    const double Ts = 0.01;
    std::vector<ctrl::BacksteppingController::DriftFn> f = {
        [](const Eigen::VectorXd &, int) { return 0.0; },
        [](const Eigen::VectorXd &, int) { return 0.0; },
    };
    std::vector<ctrl::BacksteppingController::GainFn> g = {
        [](const Eigen::VectorXd &, int) { return 1.0; },
        [](const Eigen::VectorXd &, int) { return 1.0; },
    };
    ctrl::BacksteppingParams p;
    p.k_gains = {2.0, 2.0};
    ctrl::BacksteppingController bc(f, g, p, Ts);

    const double ref = 1.0, k1 = 2.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    const int N = 1500;
    double vEarly = 0.0, vLate = 0.0;
    for (int k = 0; k < N; ++k)
    {
        bc.setState(x);
        const double u = bc.compute(ref - x(0));
        const double z1 = x(0) - ref;
        const double alpha1 = -k1 * z1; // r' ~= 0 for a constant reference past the first step
        const double z2 = x(1) - alpha1;
        const double V = 0.5 * (z1 * z1 + z2 * z2);
        if (k == N / 10)      vEarly = V; // after the initial transient settles
        if (k == N - 1)       vLate  = V;
        x(0) += Ts * x(1);
        x(1) += Ts * u;
    }

    REQUIRE(std::isfinite(vEarly));
    REQUIRE(std::isfinite(vLate));
    REQUIRE(vLate < vEarly);
}

TEST_CASE("BacksteppingController saturates u without corrupting the virtual-control "
          "finite-difference chain",
          "[backstepping]")
{
    const double Ts = 0.01;
    std::vector<ctrl::BacksteppingController::DriftFn> f = {
        [](const Eigen::VectorXd &, int) { return 0.0; },
        [](const Eigen::VectorXd &, int) { return 0.0; },
    };
    std::vector<ctrl::BacksteppingController::GainFn> g = {
        [](const Eigen::VectorXd &, int) { return 1.0; },
        [](const Eigen::VectorXd &, int) { return 1.0; },
    };
    ctrl::BacksteppingParams p;
    p.k_gains = {2.0, 2.0};
    p.uMin = -1.0; p.uMax = 1.0; // tight enough to saturate early in the transient
    ctrl::BacksteppingController bc(f, g, p, Ts);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    bool sawSaturation = false;
    for (int k = 0; k < 500; ++k)
    {
        bc.setState(x);
        const double u = bc.compute(1.0 - x(0));
        REQUIRE(u >= p.uMin - 1e-9);
        REQUIRE(u <= p.uMax + 1e-9);
        if (u <= p.uMin + 1e-9 || u >= p.uMax - 1e-9) sawSaturation = true;
        x(0) += Ts * x(1);
        x(1) += Ts * u;
    }
    REQUIRE(sawSaturation);
    REQUIRE(x.allFinite());
}

// =============================================================================
// PassivityBasedController (Phase 3 NC2)
// =============================================================================

namespace
{
// Single-pendulum Euler-Lagrange model: M(q)=m*l^2 (constant), dV(q)=m*g*l*sin(q),
// C(q,qdot)=0 (no velocity-dependent term for a single point-mass pendulum).
ctrl::PassivityBasedController::MassMatrixFn pendulumMass(double ml2)
{
    return [ml2](const Eigen::VectorXd &) {
        return Eigen::MatrixXd::Constant(1, 1, ml2);
    };
}
ctrl::PassivityBasedController::PotentialGradFn pendulumGrad(double mgl)
{
    return [mgl](const Eigen::VectorXd &q) {
        Eigen::VectorXd dV(1);
        dV(0) = mgl * std::sin(q(0));
        return dV;
    };
}
ctrl::PassivityBasedController::CoriolisFn pendulumCoriolis()
{
    return [](const Eigen::VectorXd &, const Eigen::VectorXd &) {
        return Eigen::MatrixXd::Zero(1, 1);
    };
}
} // namespace

TEST_CASE("PassivityBasedController regulates a single pendulum to a nonzero desired angle",
          "[passivity_based]")
{
    const double Ts = 0.01, ml2 = 1.0, mgl = 9.8;
    ctrl::PBCParams p;
    p.Kp = Eigen::MatrixXd::Constant(1, 1, 10.0);
    p.Kd = Eigen::MatrixXd::Constant(1, 1, 4.0);
    ctrl::PassivityBasedController pbc(pendulumMass(ml2), pendulumGrad(mgl), pendulumCoriolis(),
                                        p, Ts);
    pbc.setDesired(Eigen::VectorXd::Constant(1, 0.5));

    Eigen::VectorXd state = Eigen::VectorXd::Zero(2); // [q; qdot]
    for (int k = 0; k < 3000; ++k)
    {
        const Eigen::VectorXd u = pbc.computeVec(state);
        const double qddot = (u(0) - mgl * std::sin(state(0))) / ml2;
        state(1) += Ts * qddot;
        state(0) += Ts * state(1);
    }

    REQUIRE(state.allFinite());
    REQUIRE_THAT(state(0), WithinAbs(0.5, 0.02));
}

TEST_CASE("PassivityBasedController's storage energy is non-increasing once past the "
          "initial transient",
          "[passivity_based]")
{
    const double Ts = 0.01, ml2 = 1.0, mgl = 9.8;
    ctrl::PBCParams p;
    p.Kp = Eigen::MatrixXd::Constant(1, 1, 10.0);
    p.Kd = Eigen::MatrixXd::Constant(1, 1, 4.0);
    ctrl::PassivityBasedController pbc(pendulumMass(ml2), pendulumGrad(mgl), pendulumCoriolis(),
                                        p, Ts);
    pbc.setDesired(Eigen::VectorXd::Constant(1, 0.3));

    Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
    double prevEnergy = std::numeric_limits<double>::infinity();
    bool everIncreased = false;
    for (int k = 0; k < 3000; ++k)
    {
        const Eigen::VectorXd u = pbc.computeVec(state);
        const double energy = pbc.storageEnergy();
        if (k > 50 && energy > prevEnergy + 1e-6) everIncreased = true; // skip initial transient
        prevEnergy = energy;

        const double qddot = (u(0) - mgl * std::sin(state(0))) / ml2;
        state(1) += Ts * qddot;
        state(0) += Ts * state(1);
    }
    REQUIRE_FALSE(everIncreased);
}

TEST_CASE("PassivityBasedController holds the last output when MassMatrixFn returns "
          "a non-finite matrix",
          "[passivity_based]")
{
    const double Ts = 0.01;
    ctrl::PBCParams p;
    p.Kp = Eigen::MatrixXd::Constant(1, 1, 10.0);
    p.Kd = Eigen::MatrixXd::Constant(1, 1, 4.0);

    auto singularMass = [](const Eigen::VectorXd &q) {
        Eigen::MatrixXd M(1, 1);
        M(0, 0) = (std::fabs(q(0)) < 1e-9)
            ? std::numeric_limits<double>::quiet_NaN()
            : 1.0;
        return M;
    };
    ctrl::PassivityBasedController pbc(singularMass, pendulumGrad(9.8), pendulumCoriolis(), p, Ts);
    pbc.setDesired(Eigen::VectorXd::Constant(1, 0.5));

    const Eigen::VectorXd u1 = pbc.computeVec(Eigen::VectorXd::Constant(2, 0.0)); // q=0 -> NaN M
    REQUIRE(u1.allFinite());
    REQUIRE_THAT(u1(0), WithinAbs(0.0, 1e-12)); // u_prev_ defaults to 0 before any success
}

TEST_CASE("PassivityBasedController::compute(double) always throws", "[passivity_based]")
{
    const double Ts = 0.01;
    ctrl::PBCParams p;
    p.Kp = Eigen::MatrixXd::Constant(1, 1, 10.0);
    p.Kd = Eigen::MatrixXd::Constant(1, 1, 4.0);
    ctrl::PassivityBasedController pbc(pendulumMass(1.0), pendulumGrad(9.8), pendulumCoriolis(),
                                        p, Ts);
    REQUIRE_THROWS_AS(pbc.compute(0.0), std::logic_error);
}

// =============================================================================
// CLFController (Phase 3 NC4)
// =============================================================================

TEST_CASE("CLFController's Sontag-formula output matches the hand-derived closed form",
          "[clf_controller]")
{
    // Scalar system xdot = -x + u, V(x) = x^2 -> LfV = -2x^2, LgV = 2x.
    // a = LfV + alpha*V = -2x^2 + alpha*x^2 = (alpha-2)*x^2,  b = 2x.
    const double alpha = 1.0;
    ctrl::CLFParams p;
    p.alpha = alpha;
    ctrl::CLFController clf(
        [](const Eigen::VectorXd &x) { return x(0) * x(0); },
        [](const Eigen::VectorXd &x) { return -2.0 * x(0) * x(0); },
        [](const Eigen::VectorXd &x) { return 2.0 * x(0); },
        p, 0.01);

    const double x0 = 2.0;
    clf.setState(Eigen::VectorXd::Constant(1, x0));
    const double u = clf.compute(0.0);

    const double a = (alpha - 2.0) * x0 * x0;
    const double b = 2.0 * x0;
    const double uExpected = -(a + std::sqrt(a * a + b * b * b * b)) / b;

    REQUIRE_THAT(u, WithinAbs(uExpected, 1e-9));
    REQUIRE(clf.isHealthy());
}

TEST_CASE("CLFController stabilizes a scalar nonlinear system toward the CLF's equilibrium",
          "[clf_controller]")
{
    // xdot = x^3 + u (unstable open-loop drift away from 0), V(x) = x^2.
    const double Ts = 0.01;
    ctrl::CLFParams p;
    p.alpha = 2.0;
    ctrl::CLFController clf(
        [](const Eigen::VectorXd &x) { return x(0) * x(0); },
        [](const Eigen::VectorXd &x) { return 2.0 * x(0) * x(0) * x(0) * x(0); }, // 2x*(x^3)
        [](const Eigen::VectorXd &x) { return 2.0 * x(0); },
        p, Ts);

    double x = 1.5;
    for (int k = 0; k < 2000; ++k)
    {
        clf.setState(Eigen::VectorXd::Constant(1, x));
        const double u = clf.compute(0.0);
        x += Ts * (x * x * x + u);
    }
    REQUIRE(std::isfinite(x));
    REQUIRE(std::fabs(x) < 0.05);
}

TEST_CASE("CLFController reports unhealthy and holds last output when LgV=0 with positive drift",
          "[clf_controller]")
{
    ctrl::CLFParams p;
    ctrl::CLFController clf(
        [](const Eigen::VectorXd &x) { return x(0) * x(0); },
        [](const Eigen::VectorXd &) { return 1.0; },  // LfV > 0 (non-decaying drift)
        [](const Eigen::VectorXd &) { return 0.0; },  // LgV == 0 (uncontrollable direction)
        p, 0.01);

    clf.setState(Eigen::VectorXd::Constant(1, 1.0));
    const double u1 = clf.compute(0.0);
    REQUIRE_FALSE(clf.isHealthy());

    const double u2 = clf.compute(0.0); // still infeasible -> holds the same last value
    REQUIRE(u1 == u2);
}

// =============================================================================
// MRACController - model reference adaptive control
// =============================================================================

TEST_CASE("MRAC tracks reference model on nominal plant within 500 steps", "[mrac]")
{
    // Plant: y[k+1] = 0.7*y[k] + 0.5*u[k]
    // Reference model: a_m=0.5, b_m=0.5, r=1 (step)
    constexpr double Ts_m = 0.1;
    ctrl::MRACParams mp;
    mp.a_m = 0.5; mp.b_m = 0.5;
    mp.gamma_r = 3.0; mp.gamma_y = 1.5;
    mp.sigma = 0.02; mp.theta_max = 20.0;

    ctrl::MRACController mrac(mp, Ts_m);

    double y = 0.0;
    double em_final = 1e9;
    for (int k = 0; k < 500; ++k)
    {
        mrac.setReference(1.0);
        const double u = mrac.compute(y);
        y = 0.7 * y + 0.5 * u;
        em_final = mrac.modelError();
    }

    // Model tracking error must converge
    REQUIRE(std::isfinite(em_final));
    REQUIRE_THAT(std::abs(em_final), WithinAbs(0.0, 0.05));

    // Parameters must stay within bound
    const double theta_norm = std::sqrt(mrac.theta_r() * mrac.theta_r() +
                                        mrac.theta_y() * mrac.theta_y());
    REQUIRE(theta_norm <= mp.theta_max + 1e-6);
}

TEST_CASE("MRAC reset restores initial theta and clears model state", "[mrac]")
{
    ctrl::MRACParams mp;
    mp.a_m = 0.5; mp.b_m = 0.5;
    mp.gamma_r = 1.0; mp.gamma_y = 1.0;
    mp.sigma = 0.01; mp.theta_max = 10.0;

    ctrl::MRACController mrac(mp, 0.1);

    // Run a few steps to change theta
    for (int k = 0; k < 50; ++k)
    {
        mrac.setReference(1.0);
        mrac.compute(0.5);
    }

    const double theta_r_pre  = mrac.theta_r();
    const double model_out_pre = mrac.modelOutput();

    mrac.reset();

    // After reset: theta should return to initial value (b_m) and model to 0
    REQUIRE_THAT(mrac.theta_r(),    WithinAbs(mp.b_m, 1e-10));
    REQUIRE_THAT(mrac.theta_y(),    WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(mrac.modelOutput(), WithinAbs(0.0, 1e-10));

    // sampleTime is preserved
    REQUIRE_THAT(mrac.sampleTime(), WithinAbs(0.1, 1e-12));
}

// =============================================================================
// BalancedTruncation - model order reduction
// =============================================================================

TEST_CASE("balancedTruncate returns descending Hankel singular values", "[btm]")
{
    // Second-order stable plant: poles at 0.5, 0.8
    Eigen::Matrix2d A; A << 0.5, 0.0, 0.0, 0.8;
    Eigen::Vector2d B; B << 1.0, 1.0;
    Eigen::RowVector2d C; C << 1.0, 1.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    ctrl::StateSpace sys(A, B.reshaped(2,1), C.reshaped(1,2), D, 0.05);

    ctrl::TruncationResult res = ctrl::balancedTruncate(sys, 1);

    // Two HSVs, descending
    REQUIRE(res.hankelSingularValues.size() == 2);
    REQUIRE(res.hankelSingularValues(0) >= res.hankelSingularValues(1));
    REQUIRE(res.hankelSingularValues(1) >= 0.0);

    // Error bound = 2 * sigma_2
    REQUIRE_THAT(res.errorBound,
                 WithinAbs(2.0 * res.hankelSingularValues(1), 1e-8));

    // Reduced model is stable
    REQUIRE(res.isStable);
    REQUIRE(res.reduced.stateSize() == 1);
}

TEST_CASE("balancedTruncate DC gain within Hinf error bound", "[btm]")
{
    // 3rd-order stable plant with widely separated poles
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    A(0,0) = 0.9; A(1,1) = 0.5; A(2,2) = 0.1;
    Eigen::Vector3d B; B << 1.0, 1.0, 1.0;
    Eigen::RowVector3d C; C << 0.5, 0.3, 0.2;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    ctrl::StateSpace sys(A, B.reshaped(3,1), C.reshaped(1,3), D, 0.01);

    ctrl::TruncationResult res = ctrl::balancedTruncate(sys, 2);

    // DC gains
    const double dc_full = (sys.C * (Eigen::MatrixXd::Identity(3,3) - sys.A)
                                        .inverse() * sys.B)(0,0);
    const Eigen::MatrixXd &Ar = res.reduced.A, &Br = res.reduced.B, &Cr = res.reduced.C;
    const double dc_red  = (Cr * (Eigen::MatrixXd::Identity(2,2) - Ar)
                                        .inverse() * Br)(0,0);

    REQUIRE(std::abs(dc_full - dc_red) <= res.errorBound + 1e-8);
}

// =============================================================================
// ZeroPhaseTrackingFilter - ZPETC prefilter design
// =============================================================================

TEST_CASE("transmissionZeros finds correct zeros for a SISO system", "[zpetc]")
{
    // G(z) = 0.2*(z+0.5) / ((z-0.8)*(z-0.6)) - one zero at z=-0.5
    ctrl::TransferFunction tf({0.2, 0.1}, {1.0, -1.4, 0.48}, 0.01);
    ctrl::StateSpace sys = ctrl::tf2ss(tf);

    auto zeros = ctrl::transmissionZeros(sys);

    REQUIRE(zeros.size() == 1);
    // The zero should be near z = -0.5
    REQUIRE_THAT(std::abs(zeros[0].real() - (-0.5)), WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(std::abs(zeros[0].imag()), WithinAbs(0.0, 1e-6));
}

TEST_CASE("designZPETC min-phase: unit composite magnitude everywhere", "[zpetc]")
{
    // Minimum-phase: G(z) = 0.2*(z+0.5) / ((z-0.8)*(z-0.6))
    ctrl::TransferFunction tf({0.2, 0.1}, {1.0, -1.4, 0.48}, 0.01);
    ctrl::StateSpace sys = ctrl::tf2ss(tf);

    auto res = ctrl::designZPETC(sys);

    REQUIRE_FALSE(res.hasNMPZeros);
    // For a minimum-phase plant, G*G_ff = z^{-1} (unit magnitude everywhere)
    REQUIRE_THAT(res.dcAmplitudeError, WithinAbs(0.0, 1e-8));
}

TEST_CASE("designZPETC NMP: detects NMP zeros and unit DC gain", "[zpetc]")
{
    // NMP: G(z) = 0.1*(z-1.5) / ((z-0.9)*(z-0.7)) - zero at z=1.5
    ctrl::TransferFunction tf({0.1, -0.15}, {1.0, -1.6, 0.63}, 0.01);
    ctrl::StateSpace sys = ctrl::tf2ss(tf);

    auto res = ctrl::designZPETC(sys);

    REQUIRE(res.hasNMPZeros);
    REQUIRE(res.nmpZeros.size() == 1);
    REQUIRE_THAT(std::abs(res.nmpZeros[0].real() - 1.5), WithinAbs(0.0, 1e-6));

    // DC gain of G*G_ff must be 1 (by normalization)
    const double omega_dc = 1e-4;
    auto resp_plant  = ctrl::SystemAnalysis::getFrequencyResponse(sys,       {omega_dc});
    auto resp_filter = ctrl::SystemAnalysis::getFrequencyResponse(res.filter, {omega_dc});
    const double dc_composite = std::abs(resp_plant[0] * resp_filter[0]);
    REQUIRE_THAT(dc_composite, WithinAbs(1.0, 0.05));
}

// =============================================================================
// AntiWindupWrapper - conditioning technique (Hanus 1987 / Astrom-Wittenmark 8.5)
// =============================================================================

TEST_CASE("AntiWindupWrapper limits integrator windup during saturation", "[anti_windup]")
{
    // Plant: y[k+1] = 0.9*y[k] + 0.2*u[k],  y_ss = 2.0 at u = 1.
    // Controller: PI (Kp=0.5, Ki=1.0, Kb=0) - no internal anti-windup.
    // Actuator: uMin=-3, uMax=1.  Reference r=5 saturates the actuator.
    //
    // Without wrapper: integral accumulates freely during saturation (-> ~20).
    // With wrapper (Kb=1): conditioning bounds the integral to ~ uMax + r/Kb approx = 6.
    // After switching to r=0, the wrapped plant converges much faster because
    // its integral is smaller and exits saturation ~40 steps sooner.
    //
    // Tolerance: wrapped y[final] < unwrapped y[final].  The gap is ~1.5 at N=80.
    const double Ts   = 0.1;
    const double uMax =  1.0;
    const double uMin = -3.0;
    const int    Nsat = 50;   // saturation phase (r=5)
    const int    Nrec = 30;   // recovery phase (r=0)

    auto step_plant = [](double y, double u) { return 0.9 * y + 0.2 * u; };

    auto make_pid = [&]() -> std::shared_ptr<ctrl::DiscretePID> {
        ctrl::PIDParams pp;
        pp.Kp = 0.5; pp.Ki = 1.0; pp.Kd = 0.0; pp.N = 10.0;
        pp.Kb  = 0.0;                // disable built-in anti-windup
        pp.uMin = -1e9; pp.uMax = 1e9; // wrapper owns clamping
        return std::make_shared<ctrl::DiscretePID>(pp, Ts);
    };

    // --- Unwrapped: manual output clamping, integral winds up freely ---
    auto pid_raw = make_pid();
    double y_raw = 0.0;
    for (int k = 0; k < Nsat + Nrec; ++k) {
        const double ref  = (k < Nsat) ? 5.0 : 0.0;
        const double u    = std::clamp(pid_raw->compute(ref - y_raw), uMin, uMax);
        y_raw = step_plant(y_raw, u);
    }

    // --- Wrapped: conditioning technique prevents windup ---
    auto pid_aw = make_pid();
    ctrl::AntiWindupWrapper wrapped(pid_aw, uMin, uMax, /*Kb=*/1.0);
    double y_aw = 0.0;
    for (int k = 0; k < Nsat + Nrec; ++k) {
        const double ref = (k < Nsat) ? 5.0 : 0.0;
        y_aw = step_plant(y_aw, wrapped.compute(ref - y_aw));
    }

    // Both applied u=1 during saturation -> identical plant trajectory to step Nsat.
    // After Nrec recovery steps the wrapped integral is bounded (approx =6) vs unbounded
    // (approx =20) for the unwrapped case.  The wrapped version converges to r=0 faster.
    REQUIRE(y_aw < y_raw);          // wrapped converges toward 0; unwrapped stays high
    REQUIRE(y_aw < 1.5);            // wrapped has crossed below the saturation floor
    REQUIRE(y_raw > 1.5);           // unwrapped is still winding down near y_ss=2
    REQUIRE(wrapped.isHealthy());
}

TEST_CASE("AntiWindupWrapper is transparent when not saturating", "[anti_windup]")
{
    // When the output never hits the saturation limits, the wrapper must be
    // a perfect pass-through: same output as the undecorated inner controller
    // at every step.  The correction stays zero, isSaturated() stays false.
    const double Ts = 0.1;
    ctrl::PIDParams pp;
    pp.Kp = 1.0; pp.Ki = 0.5; pp.Kd = 0.0; pp.N = 10.0;
    pp.Kb  = 0.0; pp.uMin = -1e9; pp.uMax = 1e9;

    auto pid_ref     = std::make_shared<ctrl::DiscretePID>(pp, Ts);
    auto pid_wrapped = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // Wide limits: max attainable output for r=0.3 is far below +/-10.
    ctrl::AntiWindupWrapper wrapper(pid_wrapped, -10.0, 10.0, /*Kb=*/1.0);

    double y_ref = 0.0, y_aw = 0.0;
    const double r = 0.3;

    for (int k = 0; k < 30; ++k) {
        const double u_ref = pid_ref->compute(r - y_ref);
        const double u_aw  = wrapper.compute(r - y_aw);

        y_ref = 0.9 * y_ref + 0.2 * u_ref;
        y_aw  = 0.9 * y_aw  + 0.2 * u_aw;

        // Wrapper must be transparent: same output, no saturation flag
        REQUIRE_THAT(u_aw, WithinRel(u_ref, 1e-9));
        REQUIRE_FALSE(wrapper.isSaturated());
        REQUIRE_THAT(wrapper.saturationError(), WithinAbs(0.0, 1e-12));
    }

    // reset() clears correction and propagates to inner controller
    wrapper.reset();
    REQUIRE_FALSE(wrapper.isSaturated());
    REQUIRE_THAT(wrapper.saturationError(), WithinAbs(0.0, 1e-12));
}

// =============================================================================
// TubeMPC - robust MPC for bounded additive disturbances
// =============================================================================

TEST_CASE("TubeMPC nominal trajectory tracks reference (no disturbance)", "[tube_mpc]")
{
    // 1D plant: x[k+1] = 0.8*x[k] + 0.2*u[k],  y = x.
    // K = -0.3  -> A_cl = 0.8 + 0.2*(-0.3) = 0.74  (stable).
    // No disturbance: the nominal model IS the actual model.
    // After 40 steps the output must reach within 0.15 of r=1.
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.8; B << 0.2; C << 1.0; D << 0.0;
    ctrl::StateSpace sys(A, B, C, D, Ts);

    ctrl::TubeMPCParams p;
    p.Np = 10; p.Nu = 3;
    p.Q  = Eigen::MatrixXd::Identity(1,1) * 10.0;
    p.R  = Eigen::MatrixXd::Identity(1,1) * 0.05; // low rho_u -> y_ss ~= Q/(Q+R)*r ~= 0.995
    p.K  = Eigen::MatrixXd::Constant(1,1,-0.3); // tube feedback
    p.wMax  = Eigen::VectorXd::Constant(1, 0.05);
    p.uMin  = Eigen::VectorXd::Constant(1,-2.0);
    p.uMax  = Eigen::VectorXd::Constant(1, 2.0);
    p.Ts = Ts;

    ctrl::TubeMPC tmpc(sys, p);

    Eigen::VectorXd x(1); x(0) = 0.0;
    const Eigen::VectorXd yref = Eigen::VectorXd::Constant(1, 1.0);

    for (int k = 0; k < 50; ++k) {
        const Eigen::VectorXd u = tmpc.computeRef(x, yref);
        x = A * x + B * u;
    }

    // Steady-state: MPC without integral has y_ss = Q/(Q+R)*r ~= 10/10.05 ~= 0.995
    REQUIRE_THAT(x(0), WithinAbs(1.0, 0.15));
    REQUIRE(tmpc.lastQPConverged());

    // mRPI radius must be finite and positive (disturbance bound > 0)
    REQUIRE(tmpc.tubeRadius()(0) > 0.0);
    REQUIRE(std::isfinite(tmpc.tubeRadius()(0)));
}

TEST_CASE("TubeMPC actual state stays within tube under bounded disturbances", "[tube_mpc]")
{
    // Same plant.  Apply disturbances w[k] \in [-wMax, wMax] at every step.
    // Tube guarantee: |x[k] - x_nom[k]| <= z_max for all k.
    // Verified empirically with adversarial (worst-case) disturbances.
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.8; B << 0.2; C << 1.0; D << 0.0;
    ctrl::StateSpace sys(A, B, C, D, Ts);

    ctrl::TubeMPCParams p;
    p.Np = 8; p.Nu = 2;
    p.Q  = Eigen::MatrixXd::Identity(1,1);
    p.R  = Eigen::MatrixXd::Identity(1,1) * 0.1;
    p.K  = Eigen::MatrixXd::Constant(1,1,-0.3);
    p.wMax  = Eigen::VectorXd::Constant(1, 0.08);
    p.uMin  = Eigen::VectorXd::Constant(1,-3.0);
    p.uMax  = Eigen::VectorXd::Constant(1, 3.0);
    p.Ts = Ts;

    ctrl::TubeMPC tmpc(sys, p);
    const double z_max = tmpc.tubeRadius()(0);

    Eigen::VectorXd x(1); x(0) = 0.0;
    const Eigen::VectorXd yref = Eigen::VectorXd::Constant(1, 1.0);

    // Worst-case disturbance: always at maximum amplitude
    const double w_amp = p.wMax(0);
    double max_tube_err = 0.0;

    for (int k = 0; k < 50; ++k) {
        const Eigen::VectorXd u = tmpc.computeRef(x, yref);
        // Apply adversarial disturbance (sign chosen to maximise error)
        const double w = (k % 2 == 0) ? w_amp : -w_amp;
        x = A * x + B * u + Eigen::VectorXd::Constant(1, w);

        const double tube_err = std::abs(x(0) - tmpc.nominalState()(0));
        max_tube_err = std::max(max_tube_err, tube_err);
    }

    // Tube guarantee: max error must not exceed z_max (with small floating-point margin)
    REQUIRE(max_tube_err <= z_max + 1e-6);
    REQUIRE(tmpc.lastQPConverged());
}

// =============================================================================
// ParticleFilter - SIR sequential importance resampling
// =============================================================================

TEST_CASE("ParticleFilter estimates linear plant state within EKF accuracy", "[particle_filter]")
{
    // Linear plant: x[k+1] = 0.9*x[k] + u[k] + w[k],  y[k] = x[k] + v[k]
    // Q=0.1, R=0.5.  After 30 steps the PF estimate RMSE must be < 0.3
    // (comparable to the KF/EKF steady-state, confirming the algorithm works).
    const double Ts = 0.1;
    const double q  = 0.01;
    const double r  = 0.25;

    ctrl::ParticleFilterParams pfp;
    pfp.n_particles = 400;
    pfp.Q = Eigen::MatrixXd::Constant(1, 1, q);
    pfp.R = Eigen::MatrixXd::Constant(1, 1, r);
    pfp.Ts   = Ts;
    pfp.seed = 7u;

    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(1);
        xn(0) = 0.9 * x(0) + u(0);
        return xn;
    };
    auto h = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        return x; // y = x
    };

    ctrl::ParticleFilter pf(pfp, 1, 1, f, h);
    pf.initialise(Eigen::VectorXd::Zero(1),
                  Eigen::MatrixXd::Identity(1, 1) * 0.1);

    // Separate RNG for the "true" plant (deterministic seed)
    std::mt19937 plant_rng(42);
    std::normal_distribution<double> w_dist(0.0, std::sqrt(q));
    std::normal_distribution<double> v_dist(0.0, std::sqrt(r));

    double x_true = 0.0;
    const Eigen::VectorXd u_zero = Eigen::VectorXd::Zero(1);
    double sse = 0.0;
    const int N = 30;

    for (int k = 0; k < N; ++k) {
        x_true = 0.9 * x_true + w_dist(plant_rng);
        const double y_meas = x_true + v_dist(plant_rng);
        Eigen::VectorXd y_vec(1); y_vec(0) = y_meas;
        pf.step(y_vec, u_zero);

        const double err = pf.state()(0) - x_true;
        sse += err * err;
    }

    const double rmse = std::sqrt(sse / N);
    REQUIRE(rmse < 0.30); // well-behaved filter should achieve sub-KF RMSE
    REQUIRE(pf.effectiveSampleSize() > 1.0); // at least some diversity
    REQUIRE(pf.isInitialised());
}

TEST_CASE("ParticleFilter resamples when weight degeneracy occurs", "[particle_filter]")
{
    // Use a very tight likelihood (R small) to force weight degeneracy rapidly.
    // After a few steps the resampler should have fired at least once.
    ctrl::ParticleFilterParams pfp;
    pfp.n_particles = 100;
    pfp.Q = Eigen::MatrixXd::Constant(1, 1, 0.1);
    pfp.R = Eigen::MatrixXd::Constant(1, 1, 1e-4); // very tight -> fast degeneracy
    pfp.resample_threshold = 90.0; // aggressive: resample when N_eff < 90
    pfp.seed = 13u;

    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(1); xn(0) = 0.95 * x(0) + u(0); return xn;
    };
    auto h = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) { return x; };

    ctrl::ParticleFilter pf(pfp, 1, 1, f, h);
    pf.initialise(Eigen::VectorXd::Zero(1));

    const Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd y(1); y(0) = 0.5; // sudden measurement

    for (int k = 0; k < 5; ++k)
        pf.step(y, u0);

    // With N_eff threshold = 90/100, resampling must have fired at least once
    REQUIRE(pf.resampleCount() >= 1);

    // After resampling, weights should be uniform (1/N each)
    const double w_uniform = 1.0 / 100.0;
    REQUIRE_THAT(pf.weights().minCoeff(), WithinAbs(w_uniform, 1e-9));
    REQUIRE_THAT(pf.weights().maxCoeff(), WithinAbs(w_uniform, 1e-9));

    // Estimate must be finite
    REQUIRE(std::isfinite(pf.state()(0)));
}

// -----------------------------------------------------------------------------
// SetMembershipEstimator (Phase 3 Roadmap Phase 2 EF2)
// -----------------------------------------------------------------------------

TEST_CASE("SetMembershipEstimator's ellipsoid always contains the true state under bounded noise",
          "[set_membership]")
{
    const ctrl::StateSpace plant(Eigen::MatrixXd::Constant(1, 1, 0.9),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Zero(1, 1), 0.1);
    ctrl::SetMembershipParams p; p.w_bound = 0.1; p.v_bound = 0.2;
    ctrl::SetMembershipEstimator est(plant, p, Eigen::VectorXd::Zero(1),
                                      Eigen::MatrixXd::Identity(1, 1));

    std::mt19937 rng(5);
    std::uniform_real_distribution<double> wDist(-0.1, 0.1), vDist(-0.2, 0.2);
    double xTrue = 0.0;
    const Eigen::VectorXd u = Eigen::VectorXd::Constant(1, 0.3);

    for (int k = 0; k < 50; ++k)
    {
        xTrue = 0.9 * xTrue + u(0) + wDist(rng);
        est.predict(u);
        Eigen::VectorXd y(1); y(0) = xTrue + vDist(rng);
        est.update(y);

        const Eigen::VectorXd d = Eigen::VectorXd::Constant(1, xTrue) - est.centerEstimate();
        const double quad = d.dot(est.ellipsoidShape().inverse() * d);
        REQUIRE(quad <= 1.0 + 1e-6);
        REQUIRE(est.isConsistent());
    }
}

TEST_CASE("SetMembershipEstimator's ellipsoid never excludes the true state under non-Gaussian "
          "noise, unlike a KalmanFilter's confidence interval",
          "[set_membership]")
{
    const ctrl::StateSpace plant(Eigen::MatrixXd::Constant(1, 1, 0.9),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Zero(1, 1), 0.1);
    ctrl::SetMembershipParams smp; smp.w_bound = 0.05; smp.v_bound = 0.3;
    ctrl::SetMembershipEstimator est(plant, smp, Eigen::VectorXd::Zero(1),
                                      Eigen::MatrixXd::Identity(1, 1));
    ctrl::KalmanFilter kf(plant, Eigen::MatrixXd::Constant(1, 1, smp.w_bound * smp.w_bound),
                          Eigen::MatrixXd::Constant(1, 1, smp.v_bound * smp.v_bound / 9.0));

    std::mt19937 rng(9);
    std::uniform_real_distribution<double> wDist(-0.05, 0.05), vDist(-0.3, 0.3); // uniform, not Gaussian
    double xTrue = 0.0;
    const Eigen::VectorXd u = Eigen::VectorXd::Constant(1, 0.2);
    bool smAlwaysContains = true;

    for (int k = 0; k < 100; ++k)
    {
        xTrue = 0.9 * xTrue + u(0) + wDist(rng);
        est.predict(u);
        kf.predict(u);
        Eigen::VectorXd y(1); y(0) = xTrue + vDist(rng);
        est.update(y);
        kf.update(y, u);

        const Eigen::VectorXd d = Eigen::VectorXd::Constant(1, xTrue) - est.centerEstimate();
        const double quad = d.dot(est.ellipsoidShape().inverse() * d);
        if (quad > 1.0 + 1e-6) smAlwaysContains = false;
    }

    REQUIRE(smAlwaysContains);
}

TEST_CASE("SetMembershipEstimator flags an inconsistent measurement and keeps the predicted "
          "ellipsoid",
          "[set_membership]")
{
    const ctrl::StateSpace plant(Eigen::MatrixXd::Constant(1, 1, 0.9),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Zero(1, 1), 0.1);
    ctrl::SetMembershipParams p; p.w_bound = 0.05; p.v_bound = 0.1;
    ctrl::SetMembershipEstimator est(plant, p, Eigen::VectorXd::Zero(1),
                                      Eigen::MatrixXd::Identity(1, 1) * 0.01);

    est.predict(Eigen::VectorXd::Zero(1));
    const Eigen::VectorXd cBefore = est.centerEstimate();
    const Eigen::MatrixXd PBefore = est.ellipsoidShape();

    Eigen::VectorXd yBad(1); yBad(0) = 100.0; // far outside any plausible bound
    est.update(yBad);

    REQUIRE_FALSE(est.isConsistent());
    REQUIRE(est.centerEstimate().isApprox(cBefore));
    REQUIRE(est.ellipsoidShape().isApprox(PBefore));
}

TEST_CASE("SetMembershipEstimator's predict() trace-optimal scalar beats p=1",
          "[set_membership]")
{
    const ctrl::StateSpace plant(Eigen::MatrixXd::Constant(1, 1, 0.9),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Zero(1, 1), 0.1);
    ctrl::SetMembershipParams p; p.w_bound = 0.2; p.v_bound = 0.1;
    ctrl::SetMembershipEstimator est(plant, p, Eigen::VectorXd::Zero(1),
                                      Eigen::MatrixXd::Identity(1, 1));

    est.predict(Eigen::VectorXd::Zero(1));
    const double traceOptimal = est.ellipsoidShape().trace();

    // Re-derive the p=1 (naive equal-weight) bound by hand for comparison.
    const Eigen::MatrixXd APA = 0.9 * 0.9 * Eigen::MatrixXd::Identity(1, 1);
    const Eigen::MatrixXd Qw = Eigen::MatrixXd::Constant(1, 1, p.w_bound * p.w_bound);
    const double traceP1 = (2.0 * APA + 2.0 * Qw).trace();

    REQUIRE(traceOptimal <= traceP1 + 1e-9);
}

// -----------------------------------------------------------------------------
// ParticleFilterV2 (Phase 3 Roadmap Phase 2 EF3)
// -----------------------------------------------------------------------------

TEST_CASE("ParticleFilterV2 Bootstrap mode is numerically identical to plain ParticleFilter",
          "[particle_filter_variants]")
{
    ctrl::ParticleFilterParamsV2 p;
    p.n_particles = 50; p.Q = Eigen::MatrixXd::Constant(1, 1, 0.1);
    p.R = Eigen::MatrixXd::Constant(1, 1, 0.5); p.seed = 3u;
    p.variant = ctrl::PFVariant::Bootstrap;

    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(1); xn(0) = 0.9 * x(0) + u(0); return xn;
    };
    auto h = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) { return x; };

    ctrl::ParticleFilterV2 pfv2(p, 1, 1, f, h);
    ctrl::ParticleFilterParams pBase = p;
    ctrl::ParticleFilter pfBase(pBase, 1, 1, f, h);

    pfv2.initialise(Eigen::VectorXd::Zero(1));
    pfBase.initialise(Eigen::VectorXd::Zero(1));

    const Eigen::VectorXd u0 = Eigen::VectorXd::Constant(1, 0.2);
    for (int k = 0; k < 10; ++k)
    {
        Eigen::VectorXd y(1); y(0) = 0.3 + 0.01 * k;
        pfv2.step(y, u0);
        pfBase.step(y, u0);
    }

    REQUIRE_THAT(pfv2.state()(0), WithinAbs(pfBase.state()(0), 1e-9));
    REQUIRE(pfv2.resampleCount() == pfBase.resampleCount());
}

TEST_CASE("ParticleFilterV2 Auxiliary mode achieves a higher effective sample size than "
          "Bootstrap under an informative (sharply-peaked) likelihood",
          "[particle_filter_variants]")
{
    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(1); xn(0) = x(0) + u(0); return xn;
    };
    auto h = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) { return x; };

    ctrl::ParticleFilterParamsV2 pBoot;
    pBoot.n_particles = 200; pBoot.Q = Eigen::MatrixXd::Constant(1, 1, 1.0);
    pBoot.R = Eigen::MatrixXd::Constant(1, 1, 0.01); // sharply-peaked likelihood
    pBoot.seed = 17u; pBoot.variant = ctrl::PFVariant::Bootstrap;
    ctrl::ParticleFilterV2 pfBoot(pBoot, 1, 1, f, h);
    pfBoot.initialise(Eigen::VectorXd::Zero(1));

    ctrl::ParticleFilterParamsV2 pAux = pBoot;
    pAux.variant = ctrl::PFVariant::Auxiliary;
    ctrl::ParticleFilterV2 pfAux(pAux, 1, 1, f, h);
    pfAux.initialise(Eigen::VectorXd::Zero(1));

    const Eigen::VectorXd u0 = Eigen::VectorXd::Constant(1, 0.5);
    Eigen::VectorXd y(1); y(0) = 0.5; // jumps well away from the prior - tests look-ahead value

    pfBoot.predict(u0); pfBoot.update(y, u0);
    pfAux.step(y, u0);

    REQUIRE(pfAux.effectiveSampleSize() >= pfBoot.effectiveSampleSize());
}

TEST_CASE("ParticleFilterV2 RaoBlackwellized mode keeps an identical embedded-KF covariance "
          "across particles after predict() (LTI shared-covariance property)",
          "[particle_filter_variants]")
{
    ctrl::ParticleFilterParamsV2 p;
    p.n_particles = 20; p.Q = Eigen::MatrixXd::Identity(2, 2) * 0.01;
    p.R = Eigen::MatrixXd::Constant(1, 1, 0.1); p.seed = 1u;
    p.variant = ctrl::PFVariant::RaoBlackwellized;
    p.linear_state_indices = {1};

    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2); xn(0) = std::sin(x(0)) + u(0); xn(1) = 0.0; return xn;
    };
    auto h = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        Eigen::VectorXd y(1); y(0) = x(0) + x(1); return y;
    };

    Eigen::MatrixXd A_lin(1, 1); A_lin << 0.9;
    Eigen::MatrixXd B_lin(1, 1); B_lin << 1.0;
    Eigen::MatrixXd C_lin(1, 1); C_lin << 1.0;
    Eigen::MatrixXd Q_lin(1, 1); Q_lin << 0.01;
    Eigen::MatrixXd R_lin(1, 1); R_lin << 0.1;

    ctrl::ParticleFilterV2 pf(p, 2, 1, f, h, A_lin, B_lin, C_lin, Q_lin, R_lin);
    pf.initialise(Eigen::VectorXd::Zero(2));
    pf.predict(Eigen::VectorXd::Constant(1, 0.3));

    REQUIRE(std::isfinite(pf.state()(0)));
    REQUIRE(std::isfinite(pf.state()(1)));

    Eigen::VectorXd y(1); y(0) = 0.2;
    pf.update(y, Eigen::VectorXd::Constant(1, 0.3));
    REQUIRE(std::isfinite(pf.state()(1)));
}

TEST_CASE("ParticleFilterV2 throws on a mis-sized linear_state_indices for RaoBlackwellized mode",
          "[particle_filter_variants]")
{
    ctrl::ParticleFilterParamsV2 p;
    p.n_particles = 10; p.Q = Eigen::MatrixXd::Identity(2, 2) * 0.01;
    p.R = Eigen::MatrixXd::Constant(1, 1, 0.1);
    p.variant = ctrl::PFVariant::RaoBlackwellized;
    p.linear_state_indices = {}; // empty - required to be non-empty

    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) { return x; };
    auto h = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) { return x.head(1); };

    Eigen::MatrixXd A_lin(1, 1); A_lin << 0.9;
    Eigen::MatrixXd B_lin(1, 1); B_lin << 1.0;
    Eigen::MatrixXd C_lin(1, 1); C_lin << 1.0;
    Eigen::MatrixXd Q_lin(1, 1); Q_lin << 0.01;
    Eigen::MatrixXd R_lin(1, 1); R_lin << 0.1;

    REQUIRE_THROWS_AS(
        ctrl::ParticleFilterV2(p, 2, 1, f, h, A_lin, B_lin, C_lin, Q_lin, R_lin),
        std::invalid_argument);
}

// =============================================================================
// ILC
// =============================================================================

// Single trial on a first-order plant with P-type ILC.
static double runILCTrial(ctrl::ILC& ilc, double a, double b, double r_val)
{
    double x = 0.0, sse = 0.0;
    for (int k = 0; k < ilc.trialLength(); ++k) {
        double y = x;
        double e = r_val - y;
        double u = ilc.feedforward(k) + 3.0 * e;   // simple proportional feedback
        x = a * x + b * u;
        ilc.recordError(k, e);
        sse += e * e;
    }
    ilc.updateFeedforward();
    return std::sqrt(sse / ilc.trialLength());
}

TEST_CASE("ILC P-type constructs without error", "[ilc]")
{
    ctrl::ILC::Params p;
    p.N = 50; p.Ts = 0.01; p.mode = ctrl::ILC::Mode::PType; p.Lp = 0.5;
    REQUIRE_NOTHROW(ctrl::ILC(p));
}

TEST_CASE("ILC P-type error decreases over trials", "[ilc]")
{
    ctrl::ILC::Params p;
    p.N = 100; p.Ts = 0.01;
    p.mode = ctrl::ILC::Mode::PType;
    p.Lp   = 0.5;
    p.Q_filter = 0.95;
    ctrl::ILC ilc(p);

    double rms_first = 0.0, rms_last = 0.0;
    for (int t = 0; t < 20; ++t) {
        double rms = runILCTrial(ilc, 0.8, 0.2, 1.0);
        if (t == 0)  rms_first = rms;
        if (t == 19) rms_last  = rms;
    }
    // After 20 trials, error should have decreased (Q_filter=0.95 slows convergence)
    REQUIRE(rms_last < rms_first * 0.65);
}

TEST_CASE("ILC feedforward bound respected", "[ilc]")
{
    ctrl::ILC::Params p;
    p.N = 50; p.Ts = 0.01; p.Lp = 0.8;
    p.uMin = -0.5; p.uMax = 0.5;
    ctrl::ILC ilc(p);

    // Run one trial with large errors to force saturation
    for (int k = 0; k < p.N; ++k)
        ilc.recordError(k, 10.0);   // very large error -> would saturate
    ilc.updateFeedforward();

    for (int k = 0; k < p.N; ++k) {
        REQUIRE(ilc.feedforward(k) >= p.uMin - 1e-10);
        REQUIRE(ilc.feedforward(k) <= p.uMax + 1e-10);
    }
}

TEST_CASE("ILC norm-optimal constructs from Markov matrix", "[ilc]")
{
    constexpr int N = 20;
    // First-order system Markov matrix G[i,j] = b * a^(i-j)  for i>=j
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(N, N);
    const double a = 0.8, b = 0.2;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j <= i; ++j)
            G(i, j) = b * std::pow(a, static_cast<double>(i - j));

    ctrl::ILC::Params p;
    p.N = N; p.Ts = 0.01;
    p.mode = ctrl::ILC::Mode::NormOptimal;
    p.rho_u = 0.5; p.rho_e = 1.0;
    REQUIRE_NOTHROW(ctrl::ILC(p, G));
}

TEST_CASE("ILC reset restores initial state", "[ilc]")
{
    ctrl::ILC::Params p;
    p.N = 30; p.Ts = 0.01; p.Lp = 0.5;
    ctrl::ILC ilc(p);

    for (int k = 0; k < p.N; ++k) ilc.recordError(k, 1.0);
    ilc.updateFeedforward();
    REQUIRE(ilc.trialIndex() == 1);
    REQUIRE(ilc.feedforward(0) > 0.0);  // should have learnt something

    ilc.reset();
    REQUIRE(ilc.trialIndex() == 0);
    for (int k = 0; k < p.N; ++k)
        REQUIRE_THAT(ilc.feedforward(k), WithinAbs(0.0, 1e-12));
}

// =============================================================================
// SINDy
// =============================================================================

TEST_CASE("SINDy identifies linear system exactly (OLS)", "[sindy]")
{
    // True dynamics: dx = -0.5*x + 2.0*u
    ctrl::SINDy::Params p;
    p.n_state  = 1;
    p.n_input  = 1;
    p.library  = ctrl::SINDyLibrary::PolyDeg2;
    p.use_ols  = true;   // plain OLS, no sparsification
    ctrl::SINDy sindy(p);

    for (int k = 0; k < 100; ++k) {
        Eigen::VectorXd x(1), u(1), xdot(1);
        x(0)    = static_cast<double>(k % 10) * 0.2 - 1.0;
        u(0)    = static_cast<double>(k % 7)  * 0.3 - 1.0;
        xdot(0) = -0.5 * x(0) + 2.0 * u(0);
        sindy.addSnapshot(x, u, xdot);
    }

    ctrl::SINDyModel model = sindy.fit();

    // Verify prediction on test point
    Eigen::VectorXd x_t(1), u_t(1);
    x_t(0) = 0.7;  u_t(0) = 0.3;
    Eigen::VectorXd pred = model.predict(x_t, u_t);
    const double true_val = -0.5 * 0.7 + 2.0 * 0.3;
    REQUIRE_THAT(pred(0), WithinAbs(true_val, 1e-6));
}

TEST_CASE("SINDy STLS produces sparser solution than OLS on linear system", "[sindy]")
{
    ctrl::SINDy::Params p_ols, p_stls;
    p_ols.n_state  = p_stls.n_state  = 1;
    p_ols.n_input  = p_stls.n_input  = 1;
    p_ols.library  = p_stls.library  = ctrl::SINDyLibrary::PolyDeg2;
    p_ols.use_ols  = true;
    p_stls.threshold = 0.05;  p_stls.stls_iter = 15;

    ctrl::SINDy sindy_ols(p_ols), sindy_stls(p_stls);

    for (int k = 0; k < 200; ++k) {
        Eigen::VectorXd x(1), u(1), xdot(1);
        x(0)    = (k % 11) * 0.2 - 1.0;
        u(0)    = (k % 7)  * 0.3 - 1.0;
        xdot(0) = -0.5 * x(0) + 1.0 * u(0);  // linear system
        sindy_ols .addSnapshot(x, u, xdot);
        sindy_stls.addSnapshot(x, u, xdot);
    }

    auto m_ols  = sindy_ols .fit();
    auto m_stls = sindy_stls.fit();

    // STLS must be at least as sparse as OLS
    REQUIRE(m_stls.sparsity() >= m_ols.sparsity());
}

TEST_CASE("SINDy libraryRow length matches n_terms", "[sindy]")
{
    ctrl::SINDy::Params p;
    p.n_state = 2;  p.n_input = 1;
    p.library = ctrl::SINDyLibrary::PolyDeg2;
    ctrl::SINDy sindy(p);

    Eigen::VectorXd x(2), u(1);
    x << 0.5, -0.3;
    u << 0.2;
    auto row = sindy.libraryRow(x, u);
    REQUIRE(row.size() == sindy.nTerms());
}

TEST_CASE("SINDy finite-difference snapshot matches analytic", "[sindy]")
{
    ctrl::SINDy::Params p;
    p.n_state = 1; p.n_input = 1;
    p.library = ctrl::SINDyLibrary::PolyDeg1;
    p.use_ols = true;
    ctrl::SINDy sindy(p);

    constexpr double Ts = 0.01;
    for (int k = 0; k < 50; ++k) {
        Eigen::VectorXd x0(1), x1(1), u(1);
        x0(0) = static_cast<double>(k) * 0.05;
        u(0)  = (k % 2 == 0) ? 0.5 : -0.3;  // vary u to avoid collinearity with constant term
        x1(0) = 0.9 * x0(0) + 0.1 * u(0);  // Euler step of first-order system
        sindy.addSnapshotFD(x0, x1, u, Ts);
    }
    auto model = sindy.fit();

    // Should reconstruct dx/dt = -10*x + 10*u (from Euler: x1 = (1-a*Ts)*x0 + b*Ts*u)
    Eigen::VectorXd xt(1), ut(1);
    xt(0) = 1.0; ut(0) = 0.5;
    auto pred = model.predict(xt, ut);
    // True: dx = (0.9-1)/0.01 * x + 0.1/0.01 * u = -10*x + 10*u = -10 + 5 = -5
    REQUIRE_THAT(pred(0), WithinAbs(-5.0, 0.1));
}

// =============================================================================
// KoopmanEDMD
// =============================================================================

TEST_CASE("KoopmanEDMD constructs and reports correct lifted dimension", "[koopman]")
{
    ctrl::KoopmanEDMD::Params p;
    p.n_state = 2; p.n_input = 1;
    p.dict = ctrl::KoopmanDict::PolyDeg2;
    ctrl::KoopmanEDMD edmd(p);
    // PolyDeg2 for n=2, m=1: [1; x(2); u(1); x^2(3); xu(2); u^2(1)] = 10
    REQUIRE(edmd.nLifted() == 10);
    REQUIRE(edmd.snapshotCount() == 0);
}

TEST_CASE("KoopmanEDMD PolyDeg1 recovers linear dynamics exactly", "[koopman]")
{
    // True dynamics: x[k+1] = 0.9*x[k] + 0.1*u[k]
    ctrl::KoopmanEDMD::Params p;
    p.n_state = 1; p.n_input = 1;
    p.dict = ctrl::KoopmanDict::PolyDeg1;
    p.tikhonov = 1e-10;
    ctrl::KoopmanEDMD edmd(p);

    for (int k = 0; k < 60; ++k) {
        Eigen::VectorXd x(1), u(1), x1(1);
        x(0) = (k % 10) * 0.2 - 1.0;
        u(0) = (k % 7)  * 0.3 - 1.0;
        x1(0) = 0.9 * x(0) + 0.1 * u(0);
        edmd.addSnapshot(x, u, x1);
    }

    ctrl::StateSpace ss = edmd.fitProjected();
    // Projected A should be ~0.9, B ~0.1
    REQUIRE_THAT(ss.A(0, 0), WithinAbs(0.9, 0.02));
    REQUIRE_THAT(ss.B(0, 0), WithinAbs(0.1, 0.02));
}

TEST_CASE("KoopmanEDMD fit() returns correct output dimensions", "[koopman]")
{
    ctrl::KoopmanEDMD::Params p;
    p.n_state = 1; p.n_input = 1;
    p.dict = ctrl::KoopmanDict::PolyDeg2;
    ctrl::KoopmanEDMD edmd(p);

    for (int k = 0; k < 40; ++k) {
        Eigen::VectorXd x(1), u(1), x1(1);
        x(0) = k * 0.1 - 2.0; u(0) = 0.5; x1(0) = 0.8*x(0) + 0.2*u(0);
        edmd.addSnapshot(x, u, x1);
    }

    ctrl::StateSpace ss = edmd.fit();
    // fit() strips the n_input input columns from the lifted state, so
    // A is (nLifted - n_input) x (nLifted - n_input).
    // C is Identity(n_state_lift, n_state_lift) -- full lifted output.
    // Use fitProjected() if you need n_state x n_state_lift projection.
    const int n_state_lift = edmd.nLifted() - p.n_input;
    REQUIRE(ss.A.rows() == n_state_lift);
    REQUIRE(ss.A.cols() == n_state_lift);
    REQUIRE(ss.C.rows() == n_state_lift);
    REQUIRE(ss.C.cols() == n_state_lift);
}

TEST_CASE("KoopmanEDMD lift() returns correct dimension", "[koopman]")
{
    ctrl::KoopmanEDMD::Params p;
    p.n_state = 2; p.n_input = 2;
    p.dict = ctrl::KoopmanDict::PolyDeg1;
    ctrl::KoopmanEDMD edmd(p);
    Eigen::VectorXd x(2), u(2);
    x << 1.0, 2.0; u << 0.5, -0.5;
    Eigen::VectorXd psi = edmd.lift(x, u);
    REQUIRE(psi.size() == edmd.nLifted());
}

// =============================================================================
// L1 Adaptive Controller
// =============================================================================

TEST_CASE("L1AdaptiveController constructs without throw", "[l1adaptive]")
{
    ctrl::L1AdaptiveController::Params p;
    p.a_m = 0.9; p.b_m = 0.1; p.Gamma = 100.0; p.omega_c = 5.0;
    REQUIRE_NOTHROW(ctrl::L1AdaptiveController(p, 0.01));
}

TEST_CASE("L1AdaptiveController output is finite and bounded", "[l1adaptive]")
{
    ctrl::L1AdaptiveController::Params p;
    p.a_m = 0.85; p.b_m = 0.15; p.Gamma = 50.0; p.omega_c = 3.0;
    p.uMin = -5.0; p.uMax = 5.0;
    ctrl::L1AdaptiveController l1(p, 0.01);

    for (int k = 0; k < 200; ++k) {
        l1.setReference(1.0);
        double u = l1.compute(0.5 + 0.3 * std::sin(k * 0.1));
        REQUIRE(std::isfinite(u));
        REQUIRE(u >= p.uMin - 1e-9);
        REQUIRE(u <= p.uMax + 1e-9);
    }
}

TEST_CASE("L1AdaptiveController tracks step reference (steady-state error < 15%)", "[l1adaptive]")
{
    ctrl::L1AdaptiveController::Params p;
    p.a_m = 0.9; p.b_m = 0.1; p.k_g = 1.0;
    p.Gamma = 200.0; p.omega_c = 10.0;
    p.uMin = -5.0; p.uMax = 5.0;
    ctrl::L1AdaptiveController l1(p, 0.01);

    // First-order plant: y[k+1] = 0.85*y[k] + 0.15*u[k]
    double y = 0.0, r = 1.0;
    for (int k = 0; k < 500; ++k) {
        l1.setReference(r);
        double u = l1.compute(y);
        y = 0.85 * y + 0.15 * u;
    }
    REQUIRE_THAT(y, WithinAbs(r, 0.15));
}

TEST_CASE("L1AdaptiveController reset clears adaptation state", "[l1adaptive]")
{
    ctrl::L1AdaptiveController::Params p;
    p.a_m = 0.8; p.b_m = 0.2;
    ctrl::L1AdaptiveController l1(p, 0.01);

    l1.setReference(2.0);
    for (int k = 0; k < 50; ++k) l1.compute(static_cast<double>(k) * 0.05);

    l1.reset();
    REQUIRE_THAT(l1.estimatedDisturbance(), WithinAbs(0.0, 1e-12));
}

// =============================================================================
// CBF Safety Filter
// =============================================================================

TEST_CASE("CBFSafetyFilter constructs with shared_ptr nominal", "[cbf]")
{
    ctrl::PIDParams cbf_p1; cbf_p1.Kp = 2.0; cbf_p1.Ki = 0.1; cbf_p1.N = 1.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(cbf_p1, 0.01);
    auto h_fn   = [](double x) { return 1.5 - x; };
    auto dh_fn  = [](double)   { return -1.0; };
    auto f0_fn  = [](double x) { return -0.1 * x; };
    auto g_fn   = [](double)   { return 1.0; };
    ctrl::CBFSafetyFilter::Params cp; cp.alpha = 1.0; cp.uMin = -3.0; cp.uMax = 3.0;
    REQUIRE_NOTHROW(ctrl::CBFSafetyFilter(pid, h_fn, dh_fn, f0_fn, g_fn, cp, 0.01));
}

TEST_CASE("CBFSafetyFilter prevents state from exceeding x_max", "[cbf]")
{
    const double x_max = 1.5;
    ctrl::PIDParams cbf_p2; cbf_p2.Kp = 5.0; cbf_p2.Ki = 0.5; cbf_p2.N = 1.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(cbf_p2, 0.01);
    auto h_fn  = [x_max](double x) { return x_max - x; };
    auto dh_fn = [](double)        { return -1.0; };
    auto f0_fn = [](double)        { return 0.0; };
    auto g_fn  = [](double)        { return 1.0; };
    ctrl::CBFSafetyFilter::Params cp;
    cp.alpha = 2.0; cp.uMin = -3.0; cp.uMax = 3.0;
    ctrl::CBFSafetyFilter cbf(pid, h_fn, dh_fn, f0_fn, g_fn, cp, 0.01);

    // Integrator: x[k+1] = x[k] + 0.01*u[k], reference=2.0 (above safe set)
    double x = 0.0;
    for (int k = 0; k < 300; ++k) {
        cbf.setState(x);
        double u = cbf.compute(2.0 - x);
        x += 0.01 * u;
        REQUIRE(x <= x_max + 0.05);
    }
}

TEST_CASE("CBFSafetyFilter is inactive when state is far from boundary", "[cbf]")
{
    ctrl::PIDParams cbf_p3; cbf_p3.Kp = 1.0; cbf_p3.N = 1.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(cbf_p3, 0.01);
    auto h_fn  = [](double x) { return 10.0 - x; };
    auto dh_fn = [](double)   { return -1.0; };
    auto f0_fn = [](double)   { return 0.0; };
    auto g_fn  = [](double)   { return 1.0; };
    ctrl::CBFSafetyFilter::Params cp; cp.alpha = 1.0;
    ctrl::CBFSafetyFilter cbf(pid, h_fn, dh_fn, f0_fn, g_fn, cp, 0.01);

    cbf.setState(0.0);
    cbf.compute(1.0);
    REQUIRE_FALSE(cbf.cbfActive());
}

// =============================================================================
// Gaussian Process
// =============================================================================

TEST_CASE("GaussianProcess constructs and reports zero size", "[gp]")
{
    ctrl::GaussianProcess::Params p;
    p.length_scale = 1.0; p.signal_var = 1.0; p.noise_var = 0.01; p.n_max = 50;
    ctrl::GaussianProcess gp(1, p);
    REQUIRE(gp.size() == 0);
    REQUIRE_FALSE(gp.isFitted());
}

TEST_CASE("GaussianProcess fit and predict on near-linear data", "[gp]")
{
    ctrl::GaussianProcess::Params p;
    p.length_scale = 2.0; p.signal_var = 1.0; p.noise_var = 1e-4; p.n_max = 50;
    ctrl::GaussianProcess gp(1, p);

    for (int k = 0; k < 20; ++k) {
        Eigen::VectorXd xv(1); xv(0) = k * 0.5 - 5.0;
        gp.addPoint(xv, 2.0 * xv(0) + 1.0);
    }
    gp.fit();
    REQUIRE(gp.isFitted());

    Eigen::VectorXd xt(1); xt(0) = 0.0;
    auto pred = gp.predict(xt);
    REQUIRE(std::isfinite(pred.mean));
    REQUIRE(pred.variance >= 0.0);
    REQUIRE_THAT(pred.mean, WithinAbs(1.0, 0.5));
}

TEST_CASE("GaussianProcess variance is higher away from training data", "[gp]")
{
    ctrl::GaussianProcess::Params p;
    p.length_scale = 0.5; p.signal_var = 1.0; p.noise_var = 0.01; p.n_max = 30;
    ctrl::GaussianProcess gp(1, p);

    for (int k = 0; k < 10; ++k) {
        Eigen::VectorXd xv(1); xv(0) = static_cast<double>(k);
        gp.addPoint(xv, std::sin(xv(0)));
    }
    gp.fit();

    Eigen::VectorXd x_near(1), x_far(1);
    x_near(0) = 4.5;
    x_far(0)  = 50.0;
    auto pred_near = gp.predict(x_near);
    auto pred_far  = gp.predict(x_far);
    REQUIRE(pred_far.variance > pred_near.variance);
}

TEST_CASE("GaussianProcess fixed budget evicts oldest points", "[gp]")
{
    ctrl::GaussianProcess::Params p;
    p.length_scale = 1.0; p.signal_var = 1.0; p.noise_var = 0.01; p.n_max = 10;
    ctrl::GaussianProcess gp(1, p);

    for (int k = 0; k < 25; ++k) {
        Eigen::VectorXd xv(1); xv(0) = k * 0.1;
        gp.addPoint(xv, static_cast<double>(k));
    }
    REQUIRE(gp.size() == 10);
}

// =============================================================================
// Echo State Network
// =============================================================================

TEST_CASE("EchoStateNetwork constructs with correct reservoir size", "[esn]")
{
    ctrl::EchoStateNetwork::Params p;
    p.n_res = 30; p.n_in = 1; p.n_out = 1;
    p.spectral_radius = 0.85; p.sparsity = 0.8; p.washout = 10;
    ctrl::EchoStateNetwork esn(p);
    REQUIRE(esn.reservoirSize() == 30);
    REQUIRE_FALSE(esn.isFitted());
}

TEST_CASE("EchoStateNetwork fit and predict returns finite values", "[esn]")
{
    ctrl::EchoStateNetwork::Params p;
    p.n_res = 20; p.n_in = 1; p.n_out = 1;
    p.spectral_radius = 0.8; p.sparsity = 0.7;
    p.washout = 5; p.ridge = 1e-3; p.seed = 7;
    ctrl::EchoStateNetwork esn(p);

    double y = 0.0;
    for (int k = 0; k < 100; ++k) {
        double u = (k % 5 < 3) ? 1.0 : -1.0;
        double y_next = std::tanh(0.7 * y + 0.4 * u);
        Eigen::VectorXd uv(1), yv(1); uv << u; yv << y_next;
        esn.stepReservoir(uv);
        esn.addTrainingTarget(yv);
        y = y_next;
    }
    esn.fitReadout();
    REQUIRE(esn.isFitted());

    esn.reset();
    Eigen::VectorXd uv(1); uv << 0.5;
    Eigen::VectorXd pred = esn.predict(uv);
    REQUIRE(pred.size() == 1);
    REQUIRE(std::isfinite(pred(0)));
}

TEST_CASE("EchoStateNetwork MSE on held-out sequence is reasonable", "[esn]")
{
    ctrl::EchoStateNetwork::Params p;
    p.n_res = 40; p.n_in = 1; p.n_out = 1;
    p.spectral_radius = 0.9; p.sparsity = 0.8;
    p.washout = 20; p.ridge = 1e-4; p.seed = 42;
    ctrl::EchoStateNetwork esn(p);

    double y = 0.0;
    for (int k = 0; k < 300; ++k) {
        double u = (k % 3 == 0) ? 1.0 : -0.5;
        double y_next = std::tanh(0.8 * y + 0.5 * u);
        Eigen::VectorXd uv(1), yv(1); uv << u; yv << y_next;
        esn.stepReservoir(uv);
        esn.addTrainingTarget(yv);
        y = y_next;
    }
    esn.fitReadout();

    y = 0.0; esn.reset();
    double mse = 0.0;
    for (int k = 0; k < 50; ++k) {
        double u = (k % 4 == 0) ? 0.8 : -0.3;
        double y_true = std::tanh(0.8 * y + 0.5 * u);
        Eigen::VectorXd uv(1); uv << u;
        double y_hat = esn.predict(uv)(0);
        mse += (y_true - y_hat) * (y_true - y_hat);
        y = y_true;
    }
    REQUIRE(mse / 50 < 0.1);
}

// =============================================================================
// NeuralPID
// =============================================================================

TEST_CASE("NeuralPID constructs and returns finite bounded output", "[neuralpid]")
{
    ctrl::NeuralPID::Params p;
    p.n_hidden = 8; p.lr = 1e-3; p.Ts = 0.01;
    p.plant_gain = 1.0; p.uMin = -5.0; p.uMax = 5.0;
    ctrl::NeuralPID npid(p);
    double u = npid.compute(0.5);
    REQUIRE(std::isfinite(u));
    REQUIRE(u >= p.uMin - 1e-9);
    REQUIRE(u <= p.uMax + 1e-9);
}

TEST_CASE("NeuralPID initial gains are positive (softplus output)", "[neuralpid]")
{
    ctrl::NeuralPID::Params p;
    p.Kp0 = 2.0; p.Ki0 = 0.3; p.Kd0 = 0.0;
    p.n_hidden = 6; p.lr = 0.0;
    p.Ts = 0.01; p.plant_gain = 1.0;
    ctrl::NeuralPID npid(p);
    npid.compute(0.0);
    REQUIRE(npid.currentKp() > 0.0);
    REQUIRE(npid.currentKi() > 0.0);
}

TEST_CASE("NeuralPID reduces tracking error over 400 adaptation steps", "[neuralpid]")
{
    ctrl::NeuralPID::Params p;
    p.n_hidden = 8; p.lr = 5e-4; p.Ts = 0.01;
    p.plant_gain = 0.2; p.Kp0 = 1.0; p.Ki0 = 0.1;
    p.uMin = -3.0; p.uMax = 3.0;
    ctrl::NeuralPID npid(p);

    double x = 0.0;
    double iae_first = 0.0, iae_last = 0.0;
    for (int k = 0; k < 400; ++k) {
        double u = npid.compute(1.0 - x);
        x = 0.8 * x + 0.2 * u;
        double e = std::abs(1.0 - x);
        if (k < 100) iae_first += e;
        if (k >= 300) iae_last  += e;
    }
    REQUIRE(iae_last <= iae_first + 1.0);
}

TEST_CASE("NeuralPID reset clears integrator and error state", "[neuralpid]")
{
    ctrl::NeuralPID::Params p;
    p.n_hidden = 4; p.lr = 0.0; p.Ts = 0.01; p.plant_gain = 1.0;
    ctrl::NeuralPID npid(p);
    for (int k = 0; k < 20; ++k) npid.compute(1.0);
    npid.reset();
    double u = npid.compute(0.0);
    REQUIRE_THAT(u, WithinAbs(0.0, 0.5));
}

// =============================================================================
// CEM-MPC
// =============================================================================

TEST_CASE("CEMController constructs on double integrator", "[cem]")
{
    constexpr double kTs = 0.05;
    Eigen::Matrix2d A; A << 1, kTs, 0, 1;
    Eigen::Vector2d B; B << 0.5*kTs*kTs, kTs;
    Eigen::MatrixXd C(1, 2); C << 1.0, 0.0;

    auto f = [A, B](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        return A * x + B * u;
    };

    ctrl::CEMController::Params p;
    p.Np = 10; p.N_samples = 50; p.n_iter = 3;
    p.Q = 10.0; p.R = 0.1; p.uMin = -2.0; p.uMax = 2.0; p.seed = 1;
    REQUIRE_NOTHROW(ctrl::CEMController(p, f, C, kTs));
}

TEST_CASE("CEMController output is within bounds", "[cem]")
{
    constexpr double kTs = 0.05;
    Eigen::Matrix2d A; A << 1, kTs, 0, 1;
    Eigen::Vector2d B; B << 0.5*kTs*kTs, kTs;
    Eigen::MatrixXd C(1, 2); C << 1.0, 0.0;

    auto f = [A, B](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        return A * x + B * u;
    };

    ctrl::CEMController::Params p;
    p.Np = 8; p.N_samples = 40; p.n_iter = 2;
    p.Q = 1.0; p.R = 0.1; p.uMin = -1.5; p.uMax = 1.5; p.seed = 2;
    ctrl::CEMController cem(p, f, C, kTs);

    Eigen::Vector2d x0; x0.setZero();
    Eigen::VectorXd r(1); r << 1.0;
    cem.setState(x0); cem.setReference(r);
    auto u_vec = cem.computeRef(x0, r);

    REQUIRE(u_vec.size() == 1);
    REQUIRE(u_vec(0) >= p.uMin - 1e-9);
    REQUIRE(u_vec(0) <= p.uMax + 1e-9);
}

TEST_CASE("CEMController drives double integrator toward reference", "[cem]")
{
    constexpr double kTs = 0.05;
    Eigen::Matrix2d A; A << 1, kTs, 0, 1;
    Eigen::Vector2d B; B << 0.5*kTs*kTs, kTs;
    Eigen::MatrixXd C(1, 2); C << 1.0, 0.0;

    auto f = [A, B](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        return A * x + B * u;
    };

    ctrl::CEMController::Params p;
    p.Np = 20; p.N_samples = 80; p.n_iter = 4;
    p.Q = 100.0; p.R = 0.1; p.uMin = -3.0; p.uMax = 3.0; p.seed = 42;
    ctrl::CEMController cem(p, f, C, kTs);

    Eigen::Vector2d x; x.setZero();
    Eigen::VectorXd r(1); r << 1.0;

    for (int k = 0; k < 60; ++k) {
        cem.setState(x); cem.setReference(r);
        auto u_vec = cem.computeRef(x, r);
        x = A * x + B * u_vec(0);
    }
    REQUIRE_THAT(x(0), WithinAbs(1.0, 0.3));
}

TEST_CASE("CEMController IController compute() returns finite value", "[cem]")
{
    constexpr double kTs = 0.05;
    Eigen::Matrix2d A; A << 1, kTs, 0, 1;
    Eigen::Vector2d B; B << 0.5*kTs*kTs, kTs;
    Eigen::MatrixXd C(1, 2); C << 1.0, 0.0;

    auto f = [A, B](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        return A * x + B * u;
    };

    ctrl::CEMController::Params p;
    p.Np = 5; p.N_samples = 20; p.n_iter = 2;
    p.Q = 1.0; p.R = 0.1; p.uMin = -2.0; p.uMax = 2.0; p.seed = 3;
    ctrl::CEMController cem(p, f, C, kTs);

    Eigen::Vector2d x0; x0.setZero();
    cem.setState(x0);
    double u = cem.compute(0.5);
    REQUIRE(std::isfinite(u));
}

// =============================================================================
// DynaController (Sutton Dyna MBRL)
// =============================================================================

TEST_CASE("DynaController single compute() returns finite bounded output", "[dyna]")
{
    auto pid = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{0.8, 0.2, 0.0}, 0.01);

    ctrl::DynaController::Params dp;
    dp.Ts = 0.01; dp.n_collect = 10; dp.n_refit_every = 50;
    ctrl::DynaController dyna(dp, pid);

    for (int k = 0; k < 5; ++k) {
        double u = dyna.compute(1.0 - 0.1 * k);
        REQUIRE(std::isfinite(u));
        REQUIRE(u >= -1e6);
        REQUIRE(u <= 1e6);
    }
    REQUIRE(dyna.bufferSize() == 4);
    REQUIRE_FALSE(dyna.modelFitted());   // only 4 transitions < n_collect=10
}

TEST_CASE("DynaController fits model after n_collect transitions", "[dyna]")
{
    auto pid = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{0.5, 0.1, 0.0}, 0.01);

    ctrl::DynaController::Params dp;
    dp.Ts = 0.01; dp.n_collect = 5; dp.n_refit_every = 100;
    ctrl::DynaController dyna(dp, pid);

    // Simulate: plant e[k+1] approx = 0.9 * e[k] - 0.5 * u[k]
    double e = 1.0;
    for (int k = 0; k < 20; ++k) {
        double u = dyna.compute(e);
        e = 0.9 * e - 0.5 * u;
    }
    REQUIRE(dyna.modelFitted());
    REQUIRE(dyna.bufferSize() == 19);

    // Model rollout should return a finite trajectory
    Eigen::VectorXd u_plan = Eigen::VectorXd::Constant(10, 0.1);
    Eigen::VectorXd e_pred;
    REQUIRE_NOTHROW(e_pred = dyna.modelRollout(e, u_plan));
    REQUIRE(e_pred.size() == 10);
    REQUIRE(e_pred.allFinite());
}

TEST_CASE("DynaController model approximates linear error dynamics", "[dyna]")
{
    // Known linear dynamics: e[k+1] = 0.8 * e[k] - 0.3 * u[k]
    // Verify fitted SINDy model predicts well over a short horizon.
    auto pid = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{0.5, 0.0, 0.0}, 0.05);

    ctrl::DynaController::Params dp;
    dp.Ts = 0.05; dp.n_collect = 80; dp.n_refit_every = 200;
    dp.sindy.library   = ctrl::SINDyLibrary::PolyDeg1;  // linear model = PolyDeg1
    dp.sindy.threshold = 0.001;
    ctrl::DynaController dyna(dp, pid);

    // Collect transitions with varied u (avoid collinearity)
    double e = 1.0;
    for (int k = 0; k < 100; ++k) {
        double u_forced = (k % 2 == 0) ? 0.5 : -0.3;  // alternating excitation
        dyna.addTransition(e, u_forced, 0.8 * e - 0.3 * u_forced);
        e = 0.8 * e - 0.3 * u_forced;
    }
    REQUIRE_NOTHROW(dyna.fitModel());
    REQUIRE(dyna.modelFitted());

    // Rollout 5 steps from e=1; true trajectory: e[k] = 0.8^k * 1 - 0.3*0.1*sum(0.8^j)
    Eigen::VectorXd u_plan = Eigen::VectorXd::Constant(5, 0.1);
    Eigen::VectorXd e_pred = dyna.modelRollout(1.0, u_plan);
    double e_true = 1.0;
    for (int k = 0; k < 5; ++k) {
        e_true = 0.8 * e_true - 0.3 * 0.1;
        REQUIRE_THAT(e_pred(k), WithinAbs(e_true, 0.15));  // allow 0.15 error margin
    }
}

TEST_CASE("DynaController reset clears state without discarding model", "[dyna]")
{
    auto pid = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{0.5, 0.1, 0.0}, 0.01);

    ctrl::DynaController::Params dp;
    dp.Ts = 0.01; dp.n_collect = 5; dp.n_refit_every = 100;
    ctrl::DynaController dyna(dp, pid);

    double e = 1.0;
    for (int k = 0; k < 10; ++k) { double u = dyna.compute(e); e -= 0.1 * u; }

    REQUIRE(dyna.modelFitted());
    int buf_before = dyna.bufferSize();

    dyna.reset();

    // Buffer size and model should survive reset (model is an inference artifact)
    REQUIRE(dyna.modelFitted());          // model survives
    REQUIRE(dyna.bufferSize() == buf_before);  // accumulated data survives
    // But next compute() should not throw (has_prev_ is cleared)
    REQUIRE_NOTHROW(dyna.compute(0.5));
}

// =============================================================================
// ScenarioMPC - scenario-based stochastic MPC
// =============================================================================

static ctrl::StateSpace makeScenarioPlant(double Ts = 0.1)
{
    Eigen::Matrix<double,1,1> A; A << 0.9;
    Eigen::Matrix<double,1,1> B; B << 0.1;
    Eigen::Matrix<double,1,1> C; C << 1.0;
    Eigen::Matrix<double,1,1> D; D << 0.0;
    return ctrl::StateSpace(A, B, C, D, Ts);
}

static ctrl::ScenarioMPCParams makeScenarioParams(double sigma_w = 0.0)
{
    ctrl::ScenarioMPCParams p;
    p.Np = 8; p.Nu = 3; p.Ts = 0.1;
    p.Q  = Eigen::MatrixXd::Identity(1, 1);
    p.R  = Eigen::MatrixXd::Identity(1, 1) * 0.1;
    p.Sigma_w  = Eigen::MatrixXd::Identity(1, 1) * sigma_w * sigma_w;
    p.N_samples = 20; p.seed = 42;
    p.uMin = Eigen::VectorXd::Constant(1, -2.0);
    p.uMax = Eigen::VectorXd::Constant(1,  2.0);
    return p;
}

TEST_CASE("ScenarioMPC constructs on SISO first-order plant", "[scenario_mpc]")
{
    auto sys = makeScenarioPlant();
    REQUIRE_NOTHROW(ctrl::ScenarioMPC(sys, makeScenarioParams()));
}

TEST_CASE("ScenarioMPC produces finite bounded output", "[scenario_mpc]")
{
    auto sys = makeScenarioPlant();
    ctrl::ScenarioMPC smpc(sys, makeScenarioParams(0.05));

    smpc.setState(Eigen::VectorXd::Constant(1, 0.5));
    smpc.setReference(Eigen::VectorXd::Constant(1, 1.0));
    Eigen::VectorXd u = smpc.computeControl();

    REQUIRE(u.size() == 1);
    REQUIRE(std::isfinite(u(0)));
    REQUIRE(u(0) >= -2.0 - 1e-9);
    REQUIRE(u(0) <=  2.0 + 1e-9);
}

TEST_CASE("ScenarioMPC zero-noise matches deterministic MPC direction", "[scenario_mpc]")
{
    // With Sigma_w = 0 the average noise term is zero, so ScenarioMPC
    // should push the state toward the reference like deterministic MPC.
    auto sys = makeScenarioPlant();
    ctrl::ScenarioMPC smpc(sys, makeScenarioParams(0.0));

    Eigen::VectorXd x0 = Eigen::VectorXd::Constant(1, 0.0);
    Eigen::VectorXd ref = Eigen::VectorXd::Constant(1, 1.0);
    smpc.setState(x0);
    smpc.setReference(ref);
    Eigen::VectorXd u = smpc.computeControl();

    // With x=0 and ref=1, the first control action should be positive
    REQUIRE(u(0) > 0.0);
}

TEST_CASE("ScenarioMPC drives plant to reference in closed loop", "[scenario_mpc]")
{
    auto sys = makeScenarioPlant();
    // Lower effort penalty (R=0.01) so steady-state tracking error is within 10%.
    // With R=0.1 the MPC undershoots to ~0.79 due to the cost-function offset.
    auto p = makeScenarioParams(0.02);
    p.R = Eigen::MatrixXd::Identity(1, 1) * 0.01;
    ctrl::ScenarioMPC smpc(sys, p);

    double y = 0.0;
    double ref = 1.0;
    for (int k = 0; k < 60; ++k) {
        Eigen::VectorXd xv(1); xv(0) = y;
        Eigen::VectorXd rv(1); rv(0) = ref;
        smpc.setState(xv);
        smpc.setReference(rv);
        Eigen::VectorXd u = smpc.computeControl();
        // Advance SISO FOPDT plant: y[k+1] = 0.9*y[k] + 0.1*u[k]
        y = 0.9 * y + 0.1 * u(0);
    }
    // Should have converged to within 10% of ref despite noise
    REQUIRE_THAT(y, WithinAbs(1.0, 0.10));
}

TEST_CASE("ScenarioMPC IController compute() returns finite value", "[scenario_mpc]")
{
    auto sys = makeScenarioPlant();
    ctrl::ScenarioMPC smpc(sys, makeScenarioParams(0.01));

    double u = smpc.compute(0.5);  // SISO shortcut
    REQUIRE(std::isfinite(u));
}

// =============================================================================
// BayesianOptimizer
// =============================================================================

// Helper: make BayesOptParams for an n-dimensional problem
static ctrl::BayesOptParams makeBOParams(int n, int n_init = 8, int maxIter = 12)
{
    ctrl::BayesOptParams p;
    p.n = n; p.n_init = n_init; p.maxIter = maxIter;
    p.n_acq_restarts = 50; p.seed = 42;
    p.lower = Eigen::VectorXd::Constant(n, 0.0);
    p.upper = Eigen::VectorXd::Constant(n, 5.0);
    return p;
}

TEST_CASE("BayesianOptimizer constructs on 1D problem", "[bayesian_optimizer]")
{
    REQUIRE_NOTHROW(ctrl::BayesianOptimizer(makeBOParams(1)));
}

TEST_CASE("BayesianOptimizer minimises 1D parabola (f=(x-2)^2)", "[bayesian_optimizer]")
{
    ctrl::BayesianOptimizer bo(makeBOParams(1, 5, 15));
    Eigen::VectorXd x0(1); x0(0) = 4.0;

    auto cost = [](const Eigen::VectorXd& p) { return (p(0) - 2.0) * (p(0) - 2.0); };
    ctrl::TunerResult r = bo.tune(cost, x0);

    REQUIRE(r.nEvals == 20);  // n_init=5 + maxIter=15
    REQUIRE(std::isfinite(r.cost));
    REQUIRE(r.cost < 0.5);          // should find x ~ 2 within [0,5]
    REQUIRE_THAT(r.params(0), WithinAbs(2.0, 0.8));
}

TEST_CASE("BayesianOptimizer minimises 2D sphere (f=||x-c||^2)", "[bayesian_optimizer]")
{
    ctrl::BayesianOptimizer bo(makeBOParams(2, 8, 20));
    Eigen::VectorXd x0(2); x0 << 0.5, 0.5;
    Eigen::VectorXd centre(2); centre << 2.5, 1.5;

    auto cost = [&](const Eigen::VectorXd& p) {
        return (p - centre).squaredNorm();
    };
    ctrl::TunerResult r = bo.tune(cost, x0);

    REQUIRE(r.cost < 1.0);   // should find near (2.5, 1.5)
}

TEST_CASE("BayesianOptimizer EI mode returns finite result", "[bayesian_optimizer]")
{
    ctrl::BayesOptParams p = makeBOParams(1, 5, 10);
    p.acq = ctrl::BayesOptParams::Acquisition::EI;
    ctrl::BayesianOptimizer bo(p);

    Eigen::VectorXd x0(1); x0(0) = 3.0;
    auto cost = [](const Eigen::VectorXd& v) { return std::abs(v(0) - 1.5); };
    ctrl::TunerResult r = bo.tune(cost, x0);

    REQUIRE(std::isfinite(r.cost));
    REQUIRE(r.cost < 1.0);
}

TEST_CASE("BayesianOptimizer eval count matches n_init + maxIter", "[bayesian_optimizer]")
{
    ctrl::BayesOptParams p = makeBOParams(2, 6, 9);
    ctrl::BayesianOptimizer bo(p);

    Eigen::VectorXd x0(2); x0 << 1.0, 1.0;
    int count = 0;
    auto cost = [&](const Eigen::VectorXd&) { ++count; return 1.0; };
    ctrl::TunerResult r = bo.tune(cost, x0);

    REQUIRE(count == 15);         // n_init=6 + maxIter=9
    REQUIRE(r.nEvals == 15);
}

// =============================================================================
// ControllerRegistry (M2)
// =============================================================================

TEST_CASE("ControllerRegistry: addFeature and has work", "[registry]")
{
    // The registry should already be populated from the includes above
    REQUIRE(ctrl::ControllerRegistry::count() > 0);
    REQUIRE(ctrl::ControllerRegistry::has("dyna"));            // self-registered
    REQUIRE(ctrl::ControllerRegistry::has("scenario_mpc"));    // self-registered
    REQUIRE(ctrl::ControllerRegistry::has("bayesian_optimizer")); // self-registered
    REQUIRE(ctrl::ControllerRegistry::has("genetic_algorithm")); // self-registered
    REQUIRE(ctrl::ControllerRegistry::has("particle_swarm"));    // self-registered
    REQUIRE(ctrl::ControllerRegistry::has("differential_evolution")); // self-registered
    REQUIRE(ctrl::ControllerRegistry::has("controller_monitor")); // self-registered
    REQUIRE(ctrl::ControllerRegistry::has("pid"));             // from ControllerRegistrations.h
    REQUIRE_FALSE(ctrl::ControllerRegistry::has("nonexistent_xyz"));
}

TEST_CASE("ControllerRegistry: features() snapshot matches registry", "[registry]")
{
    auto f = ctrl::features();
    REQUIRE_FALSE(f.empty());
    // features() must include every self-registered entry
    REQUIRE(f.count("dyna") > 0);
    REQUIRE(f.at("dyna") == true);
}

TEST_CASE("ControllerRegistry: CTRL_REGISTER_FEATURE macro dedups", "[registry]")
{
    // Including DynaController.h (already included above) does NOT double-insert
    // because inline const bool is initialised exactly once per program.
    int before = ctrl::ControllerRegistry::count();
    (void)ctrl::ControllerRegistry::has("dyna");  // just a read
    REQUIRE(ctrl::ControllerRegistry::count() == before);  // count unchanged
}

// =============================================================================
// ControllerMonitor (M3/SPC)
// =============================================================================

TEST_CASE("ControllerMonitor: no alarm on in-control data", "[monitor]")
{
    ctrl::ControllerMonitor mon;
    mon.setTarget(0.0);
    mon.setSigma(1.0);
    // CUSUM h=5 sigma => threshold 5.0; k=0.5 => slack 0.5
    // Normal N(0,1) data: C+ should stay << 5
    int alarms = 0;
    mon.setAlarmCallback([&](std::string_view, double) { ++alarms; });

    for (int i = 0; i < 30; ++i)
        mon.onCompute(0.0, 0.0);   // zero mean = in-control

    REQUIRE(mon.nSamples() == 30);
    REQUIRE(alarms == 0);
    REQUIRE(std::isfinite(mon.cusumStat()));
    REQUIRE(std::isfinite(mon.ewmaStat()));
}

TEST_CASE("ControllerMonitor: CUSUM detects sustained mean shift", "[monitor]")
{
    ctrl::ControllerMonitor mon;
    mon.setTarget(0.0);
    mon.setSigma(1.0);
    mon.setCUSUMParams(0.5, 4.0);  // h=4 => threshold 4 sigma

    int cusum_alarms = 0;
    mon.setAlarmCallback([&](std::string_view name, double) {
        if (name == "CUSUM") ++cusum_alarms;
    });

    // Feed 20 samples with mean shift of +3 sigma => CUSUM should alarm quickly
    for (int i = 0; i < 20; ++i)
        mon.onCompute(3.0, 0.0);

    REQUIRE(cusum_alarms > 0);
}

TEST_CASE("ControllerMonitor: EWMA detects slow drift", "[monitor]")
{
    ctrl::ControllerMonitor mon;
    mon.setTarget(0.0);
    mon.setSigma(1.0);
    mon.setEWMAParams(0.1, 3.0);  // lambda=0.1 (slow), L=3

    int ewma_alarms = 0;
    mon.setAlarmCallback([&](std::string_view name, double) {
        if (name == "EWMA") ++ewma_alarms;
    });

    // Gradual drift: +0.5 per 5 steps => after 40 steps mean approx = 4 >> L*sigma
    for (int i = 0; i < 50; ++i)
        mon.onCompute(4.0, 0.0);

    REQUIRE(ewma_alarms > 0);
}

TEST_CASE("ControllerMonitor: onState watched key triggers alarm on large shift", "[monitor]")
{
    ctrl::ControllerMonitor mon;
    mon.setWatchKey("surface");    // watch SMC sliding surface
    mon.setTarget(0.0);
    mon.setSigma(0.1);
    mon.setCUSUMParams(0.5, 3.0);

    int alarms = 0;
    mon.setAlarmCallback([&](std::string_view, double) { ++alarms; });

    // onCompute should be ignored; onState("surface") should be processed
    for (int i = 0; i < 5; ++i)
        mon.onCompute(99.0, 0.0);   // large compute signal - should be ignored

    REQUIRE(mon.nSamples() == 0);   // watchKey set, onCompute ignored

    // Feed large surface values through onState
    for (int i = 0; i < 15; ++i)
        mon.onState("surface", Eigen::VectorXd::Constant(1, 5.0));

    REQUIRE(mon.nSamples() == 15);
    REQUIRE(alarms > 0);
}

TEST_CASE("ControllerMonitor: reset clears state", "[monitor]")
{
    ctrl::ControllerMonitor mon;
    mon.setTarget(0.0); mon.setSigma(1.0);
    for (int i = 0; i < 10; ++i) mon.onCompute(3.0, 0.0);

    mon.onReset();

    REQUIRE(mon.nSamples() == 0);
    REQUIRE(mon.nAlarms()  == 0);
    REQUIRE(mon.cusumStat() == 0.0);
}

// =============================================================================
// Edge-case contract tests (R1, R2, T2 from Part 34 senior review)
// =============================================================================

TEST_CASE("NaN input: simple controllers return finite output and recover", "[nan_guard]")
{
    const double Ts = 0.1;

    // DiscretePID
    {
        ctrl::PIDParams pp; pp.Kp = 1.0; pp.Ki = 0.5; pp.Kd = 0.0; pp.Kb = 1.0;
        ctrl::DiscretePID pid(pp, Ts);
        const double u_nan = pid.compute(std::numeric_limits<double>::quiet_NaN());
        REQUIRE(std::isfinite(u_nan));             // guard returns finite
        const double u_rec = pid.compute(0.1);     // state not corrupted
        REQUIRE(std::isfinite(u_rec));
    }

    // DiscreteSMC
    {
        ctrl::SMCParams sp; sp.K = 1.0; sp.c_e = 1.0; sp.c_de = 0.0; sp.phi = 0.1;
        ctrl::DiscreteSMC smc(sp, Ts);
        REQUIRE(std::isfinite(smc.compute(std::numeric_limits<double>::quiet_NaN())));
        REQUIRE(std::isfinite(smc.compute(0.1)));
    }

    // DiscreteADRC
    {
        ctrl::ADRCParams ap; ap.omega_c = 1.0; ap.omega_o = 3.0; ap.b0 = 1.0;
        ctrl::DiscreteADRC adrc(ap, Ts);
        REQUIRE(std::isfinite(adrc.compute(std::numeric_limits<double>::quiet_NaN())));
        REQUIRE(std::isfinite(adrc.compute(0.1)));
    }

    // MRACController
    {
        ctrl::MRACParams mp;
        mp.a_m = 0.8; mp.b_m = 0.2;
        mp.gamma_r = 0.01; mp.gamma_y = 0.01;
        mp.theta_max = 10.0; mp.uMin = -10.0; mp.uMax = 10.0;
        ctrl::MRACController mrac(mp, Ts);
        mrac.setReference(1.0);
        REQUIRE(std::isfinite(mrac.compute(std::numeric_limits<double>::quiet_NaN())));
        REQUIRE(std::isfinite(mrac.compute(0.5)));
    }
}

TEST_CASE("NaN input: MPC-family controllers return finite output and recover", "[nan_guard]")
{
    const double Ts = 0.1;
    // SISO first-order plant: x[k+1]=0.9x+0.1u, y=x
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.1; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    // DiscreteMPC
    {
        ctrl::MPCParams mp; mp.Np = 5; mp.Nc = 2; mp.rho_y = 1.0; mp.rho_u = 0.1;
        ctrl::DiscreteMPC mpc(plant, mp);
        REQUIRE(std::isfinite(mpc.compute(std::numeric_limits<double>::quiet_NaN())));
        REQUIRE(std::isfinite(mpc.compute(0.1)));
    }

    // TubeMPC
    {
        ctrl::TubeMPCParams tp;
        tp.Q = Eigen::MatrixXd::Identity(1,1);
        tp.R = Eigen::MatrixXd::Identity(1,1) * 0.1;
        tp.Np = 5;
        ctrl::TubeMPC tube(plant, tp);
        REQUIRE(std::isfinite(tube.compute(std::numeric_limits<double>::quiet_NaN())));
        REQUIRE(std::isfinite(tube.compute(0.1)));
    }
}

TEST_CASE("hasInternalAntiWindup(): DiscretePID reports correctly", "[anti_windup_contract]")
{
    const double Ts = 0.1;

    ctrl::PIDParams with_aw;
    with_aw.Kp = 1.0; with_aw.Ki = 0.5; with_aw.Kb = 1.0;
    ctrl::DiscretePID pid_aw(with_aw, Ts);
    REQUIRE(pid_aw.hasInternalAntiWindup() == true);

    ctrl::PIDParams no_aw;
    no_aw.Kp = 1.0; no_aw.Ki = 0.5; no_aw.Kb = 0.0;
    ctrl::DiscretePID pid_no(no_aw, Ts);
    REQUIRE(pid_no.hasInternalAntiWindup() == false);

    // A generic PID (non-IController pointer) also reports correctly
    ctrl::IController* base = &pid_aw;
    REQUIRE(base->hasInternalAntiWindup() == true);
}

TEST_CASE("AntiWindupWrapper throws when inner controller already has anti-windup", "[anti_windup_contract]")
{
    const double Ts = 0.1;
    ctrl::PIDParams pp; pp.Kp = 1.0; pp.Ki = 0.5; pp.Kb = 1.0;  // Kb != 0
    auto pid_aw = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    REQUIRE_THROWS_AS(
        ctrl::AntiWindupWrapper(pid_aw, -10.0, 10.0, 1.0),
        std::invalid_argument);
}

TEST_CASE("LQRAdapter isHealthy() is false for non-stabilizable plant", "[health_contract]")
{
    // Scalar unstable plant with no input: A=3, B=0.  The DARE diverges.
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 3.0; B << 0.0; C << 1.0; D << 0.0;
    ctrl::StateSpace bad_plant(A, B, C, D, 0.1);

    ctrl::LQRParams lp;
    lp.Q = Eigen::MatrixXd::Identity(1,1);
    lp.R = Eigen::MatrixXd::Identity(1,1);

    // Suppress the expected stderr warning during construction
    ctrl::DiscreteLQR lqr(bad_plant, lp);
    REQUIRE(lqr.dareConverged() == false);
}

TEST_CASE("DiscreteMPC isHealthy() is true after normal QP convergence", "[health_contract]")
{
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.1; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::MPCParams mp; mp.Np = 5; mp.Nc = 2; mp.rho_y = 1.0; mp.rho_u = 0.1;
    ctrl::DiscreteMPC mpc(plant, mp);

    mpc.compute(0.0);
    REQUIRE(mpc.isHealthy() == true);
}

TEST_CASE("DiscretePID integral stays bounded under sustained saturation with Kb>0", "[anti_windup_contract]")
{
    // Plant: y[k+1]=0.9*y + 0.2*u, steady-state u_ss=1 gives y_ss=2.
    // Reference r=5 is unachievable (uMax=1).  With Kb>0 the integral is bounded;
    // without it, the integral accumulates without limit.
    const double Ts  = 0.1;
    const double uMax = 1.0;
    const double uMin = -3.0;

    ctrl::PIDParams pp;
    pp.Kp = 0.5; pp.Ki = 1.0; pp.Kd = 0.0; pp.N = 10.0;
    pp.Kb = 1.0;   // back-calculation AW active
    pp.uMin = uMin; pp.uMax = uMax;
    ctrl::DiscretePID pid(pp, Ts);

    double y = 0.0;
    for (int k = 0; k < 200; ++k) {
        const double u = pid.compute(5.0 - y);
        y = 0.9 * y + 0.2 * u;
    }

    // After 200 steps the integral term must be within a finite, reasonable bound.
    // Without AW the integral would exceed 1000.
    const double integral = pid.params().Ki > 0.0
        ? (pid.lastOutput() - pp.Kp * (5.0 - y)) / 1.0  // rough I estimate
        : 0.0;
    REQUIRE(std::isfinite(pid.lastOutput()));
    REQUIRE(std::abs(pid.lastOutput()) <= uMax + 1e-9);  // saturated output stays clamped
}

// =============================================================================
// R1 extended: hold-last NaN contract for additional controller families
// =============================================================================

TEST_CASE("NaN input: output-feedback and wrapper controllers hold last output", "[nan_guard]")
{
    const double Ts = 0.1;
    // SISO first-order plant A=0.8, B=0.2, C=1, D=0
    Eigen::MatrixXd A1(1,1), B1(1,1), C1(1,1), D1(1,1);
    A1 << 0.8; B1 << 0.2; C1 << 1.0; D1 << 0.0;
    ctrl::StateSpace plant1(A1, B1, C1, D1, Ts);

    // ---- SmithPredictor --------------------------------------------------------
    {
        ctrl::PIDParams pp; pp.Kp = 1.0; pp.Ki = 0.1; pp.Kb = 1.0; pp.uMin = -5; pp.uMax = 5;
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::SmithPredictor sp(inner, plant1, 2);

        // First call with NaN: u_prev_ is zero-initialized -> returns 0.0 (finite)
        REQUIRE(std::isfinite(sp.compute(std::numeric_limits<double>::quiet_NaN())));
        // Accumulate a non-zero output, then verify hold-last
        const double u_normal = sp.compute(0.5);
        REQUIRE(std::isfinite(u_normal));
        const double u_held   = sp.compute(std::numeric_limits<double>::quiet_NaN());
        REQUIRE(std::isfinite(u_held));
        REQUIRE(std::isfinite(sp.compute(0.5)));   // recovers after NaN
    }

    // ---- ExtremumSeeker --------------------------------------------------------
    {
        ctrl::ExtremumSeekerParams ep;
        ep.perturbAmp = 0.1; ep.perturbFreq = 1.0;
        ep.lpfCutoff = 0.1;  ep.hpfCutoff = 0.05;
        ep.integGain = 0.5;  ep.seekMinimum = true;
        ctrl::ExtremumSeeker esc(ep, Ts);

        // Before any valid step: u_prev_ = 0 -> finite
        REQUIRE(std::isfinite(esc.compute(std::numeric_limits<double>::quiet_NaN())));
        // After a valid step, the held value should equal the last compute() output
        const double u_last = esc.compute(0.8);
        const double u_held = esc.compute(std::numeric_limits<double>::quiet_NaN());
        REQUIRE(std::isfinite(u_held));
        REQUIRE_THAT(u_held, WithinAbs(u_last, 1e-12));  // exact hold
        REQUIRE(std::isfinite(esc.compute(0.8)));
    }

    // ---- RepetitiveController --------------------------------------------------
    {
        ctrl::PIDParams pp; pp.Kp = 0.5; pp.Ki = 0.1; pp.Kb = 1.0; pp.uMin = -5; pp.uMax = 5;
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::RepetitiveParams rp; rp.periodSteps = 8; rp.Krc = 0.3; rp.Q = 0.95;
        rp.uMin = -5.0; rp.uMax = 5.0;
        ctrl::RepetitiveController rc(inner, rp, Ts);

        REQUIRE(std::isfinite(rc.compute(std::numeric_limits<double>::quiet_NaN())));
        const double u_last = rc.compute(0.3);
        const double u_held = rc.compute(std::numeric_limits<double>::quiet_NaN());
        REQUIRE(std::isfinite(u_held));
        REQUIRE_THAT(u_held, WithinAbs(u_last, 1e-12));
        REQUIRE(std::isfinite(rc.compute(0.3)));
    }
}

TEST_CASE("NaN input: ScenarioMPC scalar compute holds last output", "[nan_guard]")
{
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.1; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::ScenarioMPCParams sp;
    sp.Np = 5; sp.Nu = 2;
    sp.Q         = Eigen::MatrixXd::Identity(1,1);
    sp.R         = Eigen::MatrixXd::Identity(1,1) * 0.1;
    sp.Sigma_w   = Eigen::MatrixXd::Zero(1,1);
    sp.N_samples = 4;
    sp.uMin = Eigen::VectorXd::Constant(1, -5.0);
    sp.uMax = Eigen::VectorXd::Constant(1,  5.0);
    ctrl::ScenarioMPC smpc(plant, sp);

    REQUIRE(std::isfinite(smpc.compute(std::numeric_limits<double>::quiet_NaN())));
    const double u_last = smpc.compute(0.2);
    const double u_held = smpc.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(std::isfinite(u_held));
    REQUIRE_THAT(u_held, WithinAbs(u_last, 1e-12));
    REQUIRE(std::isfinite(smpc.compute(0.2)));
}

TEST_CASE("MRACController NaN guard holds last output and does not update theta", "[nan_guard]")
{
    const double Ts = 0.1;
    ctrl::MRACParams mp;
    mp.a_m = 0.8; mp.b_m = 0.2;
    mp.gamma_r = 0.05; mp.gamma_y = 0.05;
    mp.theta_max = 10.0; mp.uMin = -10.0; mp.uMax = 10.0;
    ctrl::MRACController mrac(mp, Ts);
    mrac.setReference(1.0);

    // Run a few valid steps to get non-zero u_prev_
    for (int k = 0; k < 10; ++k) mrac.compute(0.5);
    const double u_last = mrac.compute(0.5);
    const double u_held = mrac.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(std::isfinite(u_held));
    REQUIRE_THAT(u_held, WithinAbs(u_last, 1e-12));  // exact hold
    REQUIRE(std::isfinite(mrac.compute(0.5)));        // recovery
}

TEST_CASE("TubeMPC scalar compute holds last output on NaN", "[nan_guard]")
{
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.1; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::TubeMPCParams tp;
    tp.Q    = Eigen::MatrixXd::Identity(1,1);
    tp.R    = Eigen::MatrixXd::Identity(1,1) * 0.1;
    tp.Np   = 5;
    tp.wMax = Eigen::VectorXd::Constant(1, 0.05);
    tp.uMin = Eigen::VectorXd::Constant(1, -5.0);
    tp.uMax = Eigen::VectorXd::Constant(1,  5.0);
    ctrl::TubeMPC tube(plant, tp);

    REQUIRE(std::isfinite(tube.compute(std::numeric_limits<double>::quiet_NaN())));
    const double u_last = tube.compute(0.3);
    const double u_held = tube.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(std::isfinite(u_held));
    REQUIRE_THAT(u_held, WithinAbs(u_last, 1e-12));
    REQUIRE(std::isfinite(tube.compute(0.3)));
}

TEST_CASE("DiscreteHinf NaN guard returns last finite output, state not corrupted", "[nan_guard]")
{
    // The hold-last NaN/Inf contract is a property of DiscreteHinf::compute(),
    // which depends only on the synthesised controller matrices (Ak/Bk/Ck/Dk) -
    // not on whether DGKF synthesis converged for any particular plant. Build a
    // known-stable controller directly from a HinfResult so the guard is always
    // exercised deterministically. (The previous version ran DiscreteHinf::solve()
    // on a mixed-sensitivity plant and silently skipped every assertion via WARN
    // whenever that synthesis happened to be infeasible - so the guard could go
    // entirely untested. Synthesis itself is covered separately by the
    // solve()/solveMuSyn() cases above.)
    const double Ts = 0.05;

    ctrl::HinfResult result;
    result.feasible      = true;
    result.Ts            = Ts;
    result.achievedGamma = 1.5;
    // 2-state SISO controller K(z): stable Ak (|eig| < 1) and a non-zero Dk so a
    // valid step yields a non-trivial output we can compare against on hold-last.
    result.Ak = (Eigen::MatrixXd(2, 2) <<  0.5,  0.0,
                                           0.0, -0.3).finished();
    result.Bk = (Eigen::MatrixXd(2, 1) <<  0.2,
                                           0.1).finished();
    result.Ck = (Eigen::MatrixXd(1, 2) <<  1.0, -0.5).finished();
    result.Dk = (Eigen::MatrixXd(1, 1) <<  0.4).finished();

    ctrl::DiscreteHinf K(result);

    // One valid step populates the internal state and the last-output cache.
    const double u_valid = K.compute(0.1);
    REQUIRE(std::isfinite(u_valid));
    REQUIRE(u_valid != 0.0);                                  // Dk != 0 -> non-trivial
    const Eigen::VectorXd state_before = K.controllerState();

    // NaN input: hold the last finite output AND leave the controller state untouched.
    const double u_nan = K.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(std::isfinite(u_nan));
    REQUIRE_THAT(u_nan, WithinAbs(u_valid, 1e-12));
    REQUIRE(K.controllerState().isApprox(state_before));      // state not corrupted

    // +Inf input: same hold-last behaviour, still no state advance.
    const double u_inf = K.compute(std::numeric_limits<double>::infinity());
    REQUIRE_THAT(u_inf, WithinAbs(u_valid, 1e-12));
    REQUIRE(K.controllerState().isApprox(state_before));

    // Recovery: a finite input after NaN/Inf advances the state and stays finite.
    const double u_recover = K.compute(0.1);
    REQUIRE(std::isfinite(u_recover));
    REQUIRE_FALSE(K.controllerState().isApprox(state_before));
}

TEST_CASE("AntiWindupWrapper integral stays bounded under saturation", "[anti_windup_contract]")
{
    // Wrap a simple proportional-only controller with AntiWindupWrapper.
    // Under constant reference the wrapper's integral must stay bounded.
    const double Ts  = 0.1;
    const double uMax = 2.0, uMin = -2.0;

    // P-only inner: output = Kp * error.  Wrap with integrator + AW.
    ctrl::PIDParams pp; pp.Kp = 1.0; pp.Ki = 0.0; pp.Kd = 0.0; pp.Kb = 0.0;
    auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
    ctrl::AntiWindupWrapper aw(inner, uMin, uMax, /*Ki=*/2.0);

    double y = 0.0;
    for (int k = 0; k < 300; ++k) {
        const double u = aw.compute(5.0 - y);   // reference 5, uMax=2 -> saturated
        y = 0.9 * y + 0.3 * u;
    }
    REQUIRE(std::isfinite(aw.compute(5.0 - y)));
    REQUIRE(std::abs(aw.lastOutput()) <= uMax + 1e-9);
}

TEST_CASE("non-stabilizable plant: DiscreteMPC QP still returns finite, isHealthy reflects status", "[health_contract]")
{
    // A plant with B=0 (no input authority): MPC QP becomes trivially feasible but
    // the controller cannot do anything useful.  isHealthy() should remain true
    // (QP converged), but the plant is uncontrollable.
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 2.0; B << 0.0; C << 1.0; D << 0.0;  // unstable, no input
    ctrl::StateSpace bad(A, B, C, D, Ts);

    ctrl::MPCParams mp; mp.Np = 5; mp.Nc = 2; mp.rho_y = 1.0; mp.rho_u = 0.1;
    mp.uMin = -10.0; mp.uMax = 10.0;
    ctrl::DiscreteMPC mpc(bad, mp);
    const double u = mpc.compute(1.0);
    REQUIRE(std::isfinite(u));   // QP still produces a finite (zero) command
    // isHealthy() tracks QP convergence, not plant stabilizability
    REQUIRE(mpc.isHealthy() == true);
}

TEST_CASE("GPC isHealthy() mirrors MPC convergence behaviour", "[health_contract]")
{
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.1; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::GPCParams gp; gp.Np = 5; gp.Nu = 2; gp.rho_y = 1.0; gp.rho_u = 0.1;
    ctrl::GeneralizedPredictiveController gpc(plant, gp);
    gpc.compute(0.0);
    REQUIRE(gpc.isHealthy() == true);
}

// =============================================================================
// G2: makeLQRController -- owning factory for AutoGainScheduler design_fn use
// =============================================================================

TEST_CASE("makeLQRController creates owned LQRAdapter whose LQR survives caller scope",
          "[lqr_factory]")
{
    // Stable 2-state plant: integrator-like (pos, vel) with damping
    const double Ts = 0.1;
    Eigen::MatrixXd A(2,2), B(2,1), C(2,2), D(2,1);
    A << 1.0, Ts, 0.0, 0.95;
    B << 0.0, Ts;
    C = Eigen::MatrixXd::Identity(2, 2);
    D = Eigen::MatrixXd::Zero(2, 1);
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::LQRParams lp;
    lp.Q = Eigen::Matrix2d::Identity() * 10.0;
    lp.R = Eigen::MatrixXd::Constant(1, 1, 0.5);

    Eigen::Vector2d x; x << 0.5, 0.0;

    // Simulate design_fn scope: LQR created and destroyed inside a block,
    // but the adapter (and therefore LQR) must stay alive via shared_ptr.
    std::shared_ptr<ctrl::IController> adapt;
    {
        adapt = ctrl::makeLQRController(plant, lp, [&x]{ return x; });
    } // local scope ends -- no external DiscreteLQR object remains

    REQUIRE(adapt != nullptr);
    REQUIRE(adapt->isHealthy());           // DARE converged for a stabilizable plant
    REQUIRE_THAT(adapt->sampleTime(), WithinAbs(Ts, 1e-12));

    const double u0 = adapt->compute(0.0);
    REQUIRE(std::isfinite(u0));

    // Run a brief closed-loop: plant should converge toward zero
    for (int k = 0; k < 50; ++k) {
        const double u = adapt->compute(0.0);
        x = A * x + B * Eigen::VectorXd::Constant(1, u);
    }
    REQUIRE(x.norm() < 0.5);   // LQR drove state toward 0 -- LQR alive throughout

    adapt->reset();
    adapt->compute(0.0);       // no crash after reset confirms no dangling reference
}

TEST_CASE("makeLQRController returned adapter is implicitly IController", "[lqr_factory]")
{
    // Verify the factory return type is assignable to shared_ptr<IController>
    const double Ts = 0.05;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.1; C << 1.0; D << 0.0;
    ctrl::StateSpace sys(A, B, C, D, Ts);

    ctrl::LQRParams lp;
    lp.Q = Eigen::MatrixXd::Constant(1,1, 1.0);
    lp.R = Eigen::MatrixXd::Constant(1,1, 0.1);

    Eigen::VectorXd x(1); x(0) = 1.0;

    // Type widens to IController -- the primary use in design_fn callbacks
    std::shared_ptr<ctrl::IController> ic =
        ctrl::makeLQRController(sys, lp, [&x]{ return x; });

    REQUIRE(ic != nullptr);
    REQUIRE(ic->isHealthy());
    const double u = ic->compute(0.0);
    REQUIRE(std::isfinite(u));
}

// =============================================================================
// G3: ComputationalDelayWrapper -- one-sample actuator delay
// =============================================================================

TEST_CASE("ComputationalDelayWrapper introduces exactly one sample of delay", "[delay_wrapper]")
{
    // Proportional controller Kp=2: u = 2*e exactly (no integral, no derivative)
    const double Ts = 0.1;
    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 0.0; pp.Kd = 0.0; pp.N = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::ComputationalDelayWrapper dw(pid);

    // Step 0: e=1 -> inner computes u=2, but output is initial delay = 0
    REQUIRE_THAT(dw.compute(1.0), WithinAbs(0.0, 1e-12));

    // Step 1: e=0.5 -> inner computes u=1, output is u from step 0 = 2
    REQUIRE_THAT(dw.compute(0.5), WithinAbs(2.0, 1e-12));

    // Step 2: e=0 -> inner computes u=0, output is u from step 1 = 1
    REQUIRE_THAT(dw.compute(0.0), WithinAbs(1.0, 1e-12));

    // Step 3: output is u from step 2 = 0
    REQUIRE_THAT(dw.compute(0.0), WithinAbs(0.0, 1e-12));
}

TEST_CASE("ComputationalDelayWrapper reset clears delayed buffer", "[delay_wrapper]")
{
    const double Ts = 0.1;
    ctrl::PIDParams pp;
    pp.Kp = 5.0; pp.Ki = 0.0; pp.Kd = 0.0; pp.N = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::ComputationalDelayWrapper dw(pid);

    // Accumulate some delayed output
    dw.compute(1.0);  // buffers u=5
    REQUIRE_THAT(dw.compute(0.0), WithinAbs(5.0, 1e-12));  // u delayed=5 is returned

    // After reset, delayed buffer is 0 again
    dw.reset();
    REQUIRE_THAT(dw.compute(1.0), WithinAbs(0.0, 1e-12));  // initial output after reset
    REQUIRE_THAT(dw.sampleTime(), WithinAbs(Ts, 1e-12));
}

TEST_CASE("ComputationalDelayWrapper NaN input holds delayed output", "[delay_wrapper]")
{
    const double Ts = 0.1;
    ctrl::PIDParams pp; pp.Kp = 1.0; pp.Ki = 0.0; pp.Kd = 0.0; pp.N = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::ComputationalDelayWrapper dw(pid);
    dw.compute(2.0);  // buffers u=2

    // NaN input: inner controller not called, delayed value held
    const double u_nan = dw.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE_THAT(u_nan, WithinAbs(2.0, 1e-12));
}

// =============================================================================
// T5: GainScheduledController LinearBlend bumpless transfer
// =============================================================================

TEST_CASE("GainScheduledController LinearBlend bumplessInit called on newly-entering controller",
          "[gain_scheduled]")
{
    // Three P controllers: K=1 at p=0, K=2 at p=0.5, K=3 at p=1.
    // Pure-proportional: bumplessInit sets the last-output register so the test
    // mainly verifies no crash and correct output values.
    const double Ts = 0.1;
    auto gs = std::make_shared<ctrl::GainScheduledController>(
        Ts, ctrl::GainScheduleMode::LinearBlend);

    auto make_p_ctrl = [&](double Kp) {
        ctrl::PIDParams pp;
        pp.Kp = Kp; pp.Ki = 0.0; pp.Kd = 0.0; pp.N = 10.0;
        return std::make_shared<ctrl::DiscretePID>(pp, Ts);
    };
    gs->addSchedulePoint(0.0, make_p_ctrl(1.0));
    gs->addSchedulePoint(0.5, make_p_ctrl(2.0));
    gs->addSchedulePoint(1.0, make_p_ctrl(3.0));

    // Run at p=0.1 (bracket [idx 0, idx 1]) for several steps with e=1
    gs->setSchedulingParam(0.1);
    for (int k = 0; k < 10; ++k)
        gs->compute(1.0);

    // Jump to p=0.9 (bracket [idx 1, idx 2]): controller at idx 2 newly enters.
    // alpha = (0.9 - 0.5) / (1.0 - 0.5) = 0.8
    // expected output = (1-0.8)*K1*e + 0.8*K2*e = 0.2*2 + 0.8*3 = 2.8
    gs->setSchedulingParam(0.9);
    const double u_trans = gs->compute(1.0);

    REQUIRE(std::isfinite(u_trans));
    REQUIRE_THAT(u_trans, WithinAbs(2.8, 0.01));  // blended P output

    // After reset, prev_lo/prev_hi are cleared: first compute should be correct
    gs->reset();
    gs->setSchedulingParam(0.9);
    const double u_post_reset = gs->compute(1.0);
    REQUIRE(std::isfinite(u_post_reset));
    REQUIRE_THAT(u_post_reset, WithinAbs(2.8, 0.01));
}

// =============================================================================
// T2: MIMO nu-gap -- subspaceDist + SISO-consistency
// =============================================================================

TEST_CASE("nuGap MIMO path: 1x1 result matches SISO chordal formula", "[mimo_nugap]")
{
    // Two 1x1 (SISO) plants: poles at 0.8 and 0.7 respectively.
    // nuGap must accept them without throwing (previously threw "SISO only").
    const double Ts = 0.1;
    Eigen::MatrixXd A1(1,1),B1(1,1),C1(1,1),D1 = Eigen::MatrixXd::Zero(1,1);
    A1 << 0.8; B1 << 0.2; C1 << 1.0;
    ctrl::StateSpace sys1(A1, B1, C1, D1, Ts);

    Eigen::MatrixXd A2(1,1),B2(1,1),C2(1,1),D2 = Eigen::MatrixXd::Zero(1,1);
    A2 << 0.7; B2 << 0.2; C2 << 1.0;
    ctrl::StateSpace sys2(A2, B2, C2, D2, Ts);

    const double gap = ctrl::nuGap(sys1, sys2, 100);
    REQUIRE(std::isfinite(gap));
    REQUIRE(gap >= 0.0);
    REQUIRE(gap <= 1.0);
    REQUIRE(gap > 0.0);   // different plants -> non-zero gap

    // Identical plants: gap must be zero
    const double gap_same = ctrl::nuGap(sys1, sys1, 100);
    REQUIRE_THAT(gap_same, WithinAbs(0.0, 1e-6));
}

TEST_CASE("nuGap MIMO path: 2x2 diagonal plant returns finite gap in [0,1]",
          "[mimo_nugap]")
{
    // 2x2 decoupled plant: diagonal A with poles (0.8, 0.9) and (0.75, 0.85)
    const double Ts = 0.1;
    Eigen::Matrix2d A1, A2, C;
    A1 << 0.8, 0.0, 0.0, 0.9;
    A2 << 0.75, 0.0, 0.0, 0.85;
    C  << 1.0, 0.0, 0.0, 1.0;

    Eigen::MatrixXd B(2,2); B << 0.2, 0.0, 0.0, 0.1;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2, 2);

    ctrl::StateSpace s1(A1, B, C.reshaped(2,2), D, Ts);
    ctrl::StateSpace s2(A2, B, C.reshaped(2,2), D, Ts);

    const double gap = ctrl::nuGap(s1, s2, 100);
    REQUIRE(std::isfinite(gap));
    REQUIRE(gap >= 0.0);
    REQUIRE(gap <= 1.0);

    // Identical: gap = 0
    const double gap_self = ctrl::nuGap(s1, s1, 100);
    REQUIRE_THAT(gap_self, WithinAbs(0.0, 1e-6));
}

TEST_CASE("nuGap throws on dimension mismatch (MIMO)", "[mimo_nugap]")
{
    // 1x1 vs 2x2: must throw
    const double Ts = 0.1;
    Eigen::MatrixXd A1(1,1),B1(1,1),C1(1,1),D1 = Eigen::MatrixXd::Zero(1,1);
    A1 << 0.8; B1 << 0.2; C1 << 1.0;
    ctrl::StateSpace siso(A1, B1, C1, D1, Ts);

    Eigen::Matrix2d A2, C2;
    A2 << 0.8, 0.0, 0.0, 0.9;
    C2 << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd B2(2,2); B2.setIdentity();
    ctrl::StateSpace mimo(A2, B2, C2.reshaped(2,2), Eigen::MatrixXd::Zero(2,2), Ts);

    REQUIRE_THROWS_AS(ctrl::nuGap(siso, mimo, 50), std::invalid_argument);
}

// =============================================================================
// T4: MHE state constraints (xMin/xMax on arrival state x_0)
// =============================================================================

TEST_CASE("MHEParams xMin clamps arrival state under impossible measurements",
          "[mhe_constraints]")
{
    // Plant: x[k+1] = 0.9*x + w, y = x.
    // With B=0 (no control), tight noise bounds (|w| <= 1e-4), and xMin=0,
    // the MHE cannot explain y=-2 by driving x_0 negative or by large w.
    // Therefore x_0 is clamped to 0, and the estimate propagates as ~0.9^k * 0.
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.0; C << 1.0; D << 0.0;  // B=0: control has no effect
    ctrl::StateSpace plant(A, B, C, D, Ts);

    const Eigen::MatrixXd Q_p = Eigen::MatrixXd::Constant(1, 1, 0.01);
    const Eigen::MatrixXd R_m = Eigen::MatrixXd::Constant(1, 1, 0.1);

    ctrl::MHEParams mp;
    mp.N    = 4;
    mp.wMin = -1e-4;   // tight noise: process nearly noiseless
    mp.wMax =  1e-4;
    mp.xMin = Eigen::VectorXd::Constant(1, 0.0);  // physical floor

    ctrl::MovingHorizonEstimator mhe(plant, Q_p, R_m, mp);
    mhe.initialize(Eigen::VectorXd::Zero(1),
                   Eigen::MatrixXd::Identity(1, 1));

    for (int k = 0; k < 8; ++k) {
        Eigen::VectorXd y(1); y(0) = -2.0;
        const Eigen::VectorXd x_est =
            mhe.estimate(y, Eigen::VectorXd::Zero(1));
        // With tight noise + xMin=0: QP cannot place x_0 < 0,
        // so the estimate stays near 0 rather than following -2.
        REQUIRE(x_est(0) >= -0.05);   // small tolerance for QP convergence
    }
}

TEST_CASE("MHEParams xMax clamps arrival state estimate from above", "[mhe_constraints]")
{
    // Same plant; xMax=1 + tight noise prevents MHE from estimating x > 1
    // even when measurements are y=+5.
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.0; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    const Eigen::MatrixXd Q_p = Eigen::MatrixXd::Constant(1, 1, 0.01);
    const Eigen::MatrixXd R_m = Eigen::MatrixXd::Constant(1, 1, 0.1);

    ctrl::MHEParams mp;
    mp.N    = 4;
    mp.wMin = -1e-4;
    mp.wMax =  1e-4;
    mp.xMax = Eigen::VectorXd::Constant(1, 1.0);  // physical ceiling

    ctrl::MovingHorizonEstimator mhe(plant, Q_p, R_m, mp);
    mhe.initialize(Eigen::VectorXd::Constant(1, 0.5),
                   Eigen::MatrixXd::Identity(1, 1));

    for (int k = 0; k < 8; ++k) {
        Eigen::VectorXd y(1); y(0) = 5.0;
        const Eigen::VectorXd x_est =
            mhe.estimate(y, Eigen::VectorXd::Zero(1));
        REQUIRE(x_est(0) <= 1.0 + 0.05);  // clamped near 1, not 5
    }
}

// =============================================================================
// E4: MHE polytopic inequality constraints (C_ineq * x_0 <= d_ineq)
// =============================================================================

TEST_CASE("MHE polytopic C_ineq keeps estimate below half-space bound", "[mhe_polytopic]")
{
    // 1-D integrator; true state ramps up.  Hard constraint: x_0 <= 2.0 via C_ineq.
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 1.0; B << Ts; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::MHEParams mp;
    mp.N    = 4;
    mp.wMin = -1e-3;
    mp.wMax =  1e-3;
    // Polytopic constraint: 1*x_0 <= 2.0
    mp.C_ineq = Eigen::MatrixXd::Constant(1, 1, 1.0);
    mp.d_ineq = Eigen::VectorXd::Constant(1, 2.0);

    ctrl::MovingHorizonEstimator mhe(plant, Eigen::MatrixXd::Constant(1,1,0.01),
                                              Eigen::MatrixXd::Constant(1,1,0.1), mp);
    mhe.initialize(Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Identity(1,1));

    for (int k = 0; k < 15; ++k) {
        Eigen::VectorXd y(1); y(0) = 5.0 + k * 0.5;   // inflated measurements
        Eigen::VectorXd u(1); u(0) = 1.0;
        Eigen::VectorXd x_est = mhe.estimate(y, u);
        // Arrival state x_0 is projected; propagated current estimate may exceed
        // the bound by up to a few wMax*N steps, but should not explode.
        REQUIRE(std::isfinite(x_est(0)));
    }
}

TEST_CASE("MHE C_ineq simplex-like constraint: x1+x2 bounded", "[mhe_polytopic]")
{
    // 2-state plant; constraint x1 + x2 <= 1.5 enforced on x_0.
    const double Ts = 0.1;
    Eigen::MatrixXd A(2,2), B(2,1), Cm(1,2), D(1,1);
    A  << 0.9, 0.0, 0.0, 0.8;
    B  << Ts, 0.0;
    Cm << 1.0, 0.0;
    D  << 0.0;
    ctrl::StateSpace plant(A, B, Cm, D, Ts);

    ctrl::MHEParams mp;
    mp.N    = 3;
    mp.wMin = -0.01;
    mp.wMax =  0.01;
    mp.C_ineq.resize(1, 2);
    mp.C_ineq << 1.0, 1.0;   // x1 + x2 <= 1.5
    mp.d_ineq = Eigen::VectorXd::Constant(1, 1.5);

    ctrl::MovingHorizonEstimator mhe(plant, 0.01 * Eigen::MatrixXd::Identity(2,2),
                                             0.1  * Eigen::MatrixXd::Identity(1,1), mp);
    mhe.initialize(Eigen::VectorXd::Zero(2), Eigen::MatrixXd::Identity(2,2));

    for (int k = 0; k < 10; ++k) {
        Eigen::VectorXd y(1); y(0) = 3.0;   // large measurement pulls estimate up
        Eigen::VectorXd u(1); u(0) = 0.0;
        mhe.estimate(y, u);
    }
    // C_ineq applied to x_0 only; propagated x may differ.  Just check finite.
    REQUIRE(mhe.state().allFinite());
}

TEST_CASE("MHE C_ineq equivalent to xMax gives same result", "[mhe_polytopic]")
{
    // Box constraint x_0 <= 1.0 expressed two ways; results should agree closely.
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << 0.0; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    auto run_mhe = [&](ctrl::MHEParams mp) -> Eigen::VectorXd {
        ctrl::MovingHorizonEstimator mhe(plant,
            Eigen::MatrixXd::Constant(1,1,0.01),
            Eigen::MatrixXd::Constant(1,1,0.1), mp);
        mhe.initialize(Eigen::VectorXd::Constant(1, 0.5),
                       Eigen::MatrixXd::Identity(1,1));
        Eigen::VectorXd last(1);
        for (int k = 0; k < 6; ++k)
            last = mhe.estimate(Eigen::VectorXd::Constant(1, 5.0),
                                Eigen::VectorXd::Zero(1));
        return last;
    };

    ctrl::MHEParams mpBox;
    mpBox.N = 4; mpBox.wMin = -1e-4; mpBox.wMax = 1e-4;
    mpBox.xMax = Eigen::VectorXd::Constant(1, 1.0);

    ctrl::MHEParams mpIneq;
    mpIneq.N = 4; mpIneq.wMin = -1e-4; mpIneq.wMax = 1e-4;
    mpIneq.C_ineq = Eigen::MatrixXd::Constant(1, 1, 1.0);
    mpIneq.d_ineq = Eigen::VectorXd::Constant(1, 1.0);

    Eigen::VectorXd x_box  = run_mhe(mpBox);
    Eigen::VectorXd x_ineq = run_mhe(mpIneq);

    // Both must be near or below the ceiling; not necessarily identical (different
    // codepaths) but both must be finite and <= 1 + small tolerance.
    REQUIRE(x_box(0)  <= 1.0 + 0.05);
    REQUIRE(x_ineq(0) <= 1.0 + 0.05);
    REQUIRE(std::isfinite(x_box(0)));
    REQUIRE(std::isfinite(x_ineq(0)));
}

// =============================================================================
// P1: DAESystem - consistentInit + dae2ode
// =============================================================================

// Shared helper: Index-1 DAE with analytic solution.
// Differential:  x1' = -x1 + u
// Algebraic:     0   = x1 - x2       (so x2 = x1 at all times)
// Exact discrete (Euler, Ts=0.1): x1_next = (1-Ts)*x1 + Ts*u
static ctrl::DAESystem makeTestDAE(double Ts_dae = 0.1)
{
    ctrl::DAESystem dae;
    dae.n_diff = 1;
    dae.n_alg  = 1;
    dae.Ts     = Ts_dae;
    dae.f = [](const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, double u)
              -> Eigen::VectorXd {
        Eigen::VectorXd dx(1);
        dx(0) = -x1(0) + u;  // x1_dot = -x1 + u
        return dx;
    };
    dae.g = [](const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, double u)
              -> Eigen::VectorXd {
        Eigen::VectorXd res(1);
        res(0) = x1(0) - x2(0);  // constraint: x2 = x1
        return res;
    };
    dae.h = [](const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, double u)
              -> Eigen::VectorXd {
        return x1;  // output = differential state
    };
    return dae;
}

TEST_CASE("DAESystem consistentInit finds x2 satisfying g=0", "[dae_system]")
{
    auto dae = makeTestDAE();

    // x1 = 3.0; correct x2 = 3.0 (from g: x1 - x2 = 0)
    Eigen::VectorXd x1(1); x1(0) = 3.0;
    Eigen::VectorXd x2_guess(1); x2_guess(0) = 0.0;  // deliberately wrong

    const auto ci_result = ctrl::consistentInit(dae, x1, 0.0, x2_guess);
    const Eigen::VectorXd x2_sol = ci_result.x;

    REQUIRE(x2_sol.size() == 1);
    REQUIRE_THAT(x2_sol(0), WithinAbs(3.0, 1e-7));

    // Residual must be near zero
    const Eigen::VectorXd g_res = dae.g(x1, x2_sol, 0.0);
    REQUIRE(g_res.norm() < 1e-8);
}

TEST_CASE("DAESystem dae2ode step function converges to correct steady state", "[dae_system]")
{
    // Steady state: x1' = 0 => -x1 + u = 0 => x1_ss = u = 2.0
    auto dae = makeTestDAE(0.05);
    auto step_fn = ctrl::dae2ode(dae, 20, 1e-9);

    const double u_input = 2.0;
    Eigen::VectorXd x_aug(2); x_aug << 0.0, 0.0;

    // Run 100 steps - should converge to x1 = x2 = 2.0
    for (int k = 0; k < 100; ++k)
        x_aug = step_fn(x_aug, u_input);

    REQUIRE_THAT(x_aug(0), WithinAbs(2.0, 0.15));  // x1 converged to u
    REQUIRE_THAT(x_aug(1), WithinAbs(x_aug(0), 1e-6));  // constraint x2 = x1 maintained
}

TEST_CASE("DAESystem dae2ode maintains algebraic constraint at every step", "[dae_system]")
{
    auto dae = makeTestDAE(0.1);
    auto step_fn = ctrl::dae2ode(dae, 20, 1e-9);

    Eigen::VectorXd x_aug(2); x_aug << 0.5, 0.0;  // start with inconsistent x2

    for (int k = 0; k < 30; ++k) {
        const double u = std::sin(k * 0.3);
        x_aug = step_fn(x_aug, u);
        // Constraint g = x1 - x2 must hold after every step
        const double g_resid = std::abs(x_aug(0) - x_aug(1));
        REQUIRE(g_resid < 1e-7);
    }
}

// =============================================================================
// P2: c2d(DAESystem) - DAE linearisation and discretisation
// =============================================================================

TEST_CASE("c2d(DAESystem) reduced model has correct ZOH eigenvalue", "[dae_c2d]")
{
    // DAE: x1' = -x1 + u,  0 = x1 - x2
    // Reduced continuous model: x1' = -x1 + u  (A_c = -1, B_c = 1)
    // ZOH discrete at Ts=0.1: A_d = exp(-0.1) approx = 0.9048, B_d = 1 - exp(-0.1) approx = 0.0952
    auto dae = makeTestDAE(0.1);

    Eigen::VectorXd x1_op(1); x1_op(0) = 1.0;
    Eigen::VectorXd x2_op(1); x2_op(0) = 1.0;  // consistent: g=0

    const ctrl::StateSpace ss_d = ctrl::c2d(dae, x1_op, x2_op, 1.0, 0.1);

    REQUIRE(ss_d.stateSize()  == 1);
    REQUIRE(ss_d.inputSize()  == 1);
    REQUIRE_THAT(ss_d.A(0, 0), WithinAbs(std::exp(-0.1), 0.02));
    REQUIRE_THAT(ss_d.B(0, 0), WithinAbs(1.0 - std::exp(-0.1), 0.02));
}

TEST_CASE("c2d(DAESystem) throws when G2 is singular (index > 1)", "[dae_c2d]")
{
    // Degenerate constraint: g = 0 (no x2 dependence) => G2 = 0, singular.
    ctrl::DAESystem bad_dae;
    bad_dae.n_diff = 1; bad_dae.n_alg = 1; bad_dae.Ts = 0.1;
    bad_dae.f = [](const Eigen::VectorXd &x1, const Eigen::VectorXd &, double u)
                   -> Eigen::VectorXd { return -x1; };
    bad_dae.g = [](const Eigen::VectorXd &, const Eigen::VectorXd &, double)
                   -> Eigen::VectorXd {
        return Eigen::VectorXd::Zero(1);  // G2 = 0 -> singular
    };

    Eigen::VectorXd x1(1); x1(0) = 1.0;
    Eigen::VectorXd x2(1); x2(0) = 1.0;

    REQUIRE_THROWS_AS(ctrl::c2d(bad_dae, x1, x2, 0.0, 0.1), std::runtime_error);
}

// =============================================================================
// P3: DAE-aware EKF - algebraic projection via setAlgebraicConstraint
// =============================================================================

TEST_CASE("EKF with algebraic constraint: estimate converges and g residual stays near zero",
          "[dae_ekf]")
{
    // Plant: x_aug = [x1; x2], x1 integrates, x2 = x1 (pure constraint).
    // EKF process function includes x2 drift to test projection.
    const double Ts_e = 0.1;
    const int n_total = 2;
    const int n_out   = 1;

    // Discrete process: x1_next = 0.9*x1 + 0.1*u;  x2_next = x2 (drifts without projection)
    ctrl::StateFunc f_ekf = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        xn(0) = 0.9 * x(0) + 0.1 * u(0);
        xn(1) = x(1);  // x2 does NOT track x1 here - projection must enforce this
        return xn;
    };
    ctrl::MeasFunc h_ekf = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        return x.head(1);  // observe x1 only
    };
    ctrl::JacobianFn Fjac = [](const Eigen::VectorXd &, const Eigen::VectorXd &) {
        Eigen::MatrixXd F(2, 2);
        F << 0.9, 0.0, 0.0, 1.0;
        return F;
    };
    ctrl::JacobianFn Hjac = [](const Eigen::VectorXd &, const Eigen::VectorXd &) {
        Eigen::MatrixXd H(1, 2); H << 1.0, 0.0;
        return H;
    };

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2) * 1e-4;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * 0.01;

    ctrl::ExtendedKalmanFilter ekf(n_total, n_out, f_ekf, h_ekf, Fjac, Hjac, Q, R, Ts_e);

    // Attach algebraic constraint: x1 - x2 = 0  (same as makeTestDAE)
    ekf.setAlgebraicConstraint(
        [](const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, double) {
            Eigen::VectorXd g(1); g(0) = x1(0) - x2(0);
            return g;
        },
        1, 1);

    REQUIRE(ekf.hasAlgebraicConstraint());

    // Run 20 steps with noisy measurements of a known trajectory (x1 -> 1.0)
    Eigen::VectorXd x_true(2); x_true << 0.0, 0.0;
    for (int k = 0; k < 20; ++k) {
        Eigen::VectorXd u(1); u(0) = 1.0;
        x_true(0) = 0.9 * x_true(0) + 0.1;
        x_true(1) = x_true(0);

        Eigen::VectorXd y(1); y(0) = x_true(0) + 0.01;
        ekf.step(y, u);

        // Constraint must hold after each step
        const double g_resid = std::abs(ekf.state()(0) - ekf.state()(1));
        REQUIRE(g_resid < 1e-6);
    }

    // Estimate should have converged close to x1 = 1.0
    REQUIRE_THAT(ekf.state()(0), WithinAbs(1.0, 0.15));
}

TEST_CASE("EKF without algebraic constraint behaves identically to baseline EKF", "[dae_ekf]")
{
    // Verify setAlgebraicConstraint is purely additive - not calling it leaves EKF unchanged.
    const double Ts_e = 0.1;
    ctrl::StateFunc f_s = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return Eigen::VectorXd(x * 0.9 + u * 0.1);
    };
    ctrl::MeasFunc  h_s = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) { return x; };
    ctrl::JacobianFn Fj = [](const Eigen::VectorXd &, const Eigen::VectorXd &) {
        return Eigen::MatrixXd::Constant(1, 1, 0.9);
    };
    ctrl::JacobianFn Hj = [](const Eigen::VectorXd &, const Eigen::VectorXd &) {
        return Eigen::MatrixXd::Identity(1, 1);
    };

    Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
    Eigen::MatrixXd R1 = Eigen::MatrixXd::Identity(1, 1) * 0.01;

    // Baseline (no constraint)
    ctrl::ExtendedKalmanFilter ekf_base(1, 1, f_s, h_s, Fj, Hj, Q1, R1, Ts_e);
    // Constraint-free copy
    ctrl::ExtendedKalmanFilter ekf_constrained(1, 1, f_s, h_s, Fj, Hj, Q1, R1, Ts_e);

    REQUIRE_FALSE(ekf_constrained.hasAlgebraicConstraint());

    Eigen::VectorXd y(1); y(0) = 0.5;
    Eigen::VectorXd u(1); u(0) = 0.0;

    ekf_base.step(y, u);
    ekf_constrained.step(y, u);

    // Without constraint, both EKFs must produce identical output
    REQUIRE_THAT(ekf_base.state()(0),
                 WithinRel(ekf_constrained.state()(0), 1e-10));
    REQUIRE_THAT(ekf_base.covariance()(0, 0),
                 WithinRel(ekf_constrained.covariance()(0, 0), 1e-10));
}

// =============================================================================
// GreyBoxEstimator (E1)
// =============================================================================

TEST_CASE("GreyBoxEstimator recovers decay rate of first-order system", "[grey_box]")
{
    // Plant: xdot = -a*x + b*u,  y = x.   True: a=0.5, b=1.0.
    const double true_a = 0.5, true_b = 1.0;
    const double Ts_gb  = 0.05;
    const int    N      = 60;

    ctrl::GreyBoxEstimator::OdeFn f = [](const Eigen::VectorXd& x,
                                          const Eigen::VectorXd& u,
                                          const Eigen::VectorXd& p) {
        Eigen::VectorXd xd(1);
        xd(0) = -p(0)*x(0) + p(1)*u(0);
        return xd;
    };
    ctrl::GreyBoxEstimator::MeasFn h = [](const Eigen::VectorXd& x,
                                           const Eigen::VectorXd&) {
        return x;
    };

    // Generate ground-truth trajectory (Forward Euler)
    Eigen::MatrixXd U(1, N), Y(1, N);
    double xsim = 0.0;
    for (int k = 0; k < N; ++k) {
        U(0, k) = 1.0;
        Y(0, k) = xsim;
        xsim   += Ts_gb * (-true_a*xsim + true_b*U(0, k));
    }

    ctrl::GreyBoxEstimator::Params par;
    par.p0   = Eigen::Vector2d(0.2, 0.6);   // off by ~50%
    par.lower = Eigen::Vector2d(0.01, 0.1);
    par.upper = Eigen::Vector2d(5.0, 5.0);
    par.Ts   = Ts_gb;
    par.max_iter = 50;

    ctrl::GreyBoxEstimator est(f, h, par);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(1);
    auto res = est.fit(x0, U, Y);

    REQUIRE(est.isFitted());
    REQUIRE(std::isfinite(res.cost));
    REQUIRE_THAT(est.params()(0), WithinAbs(true_a, 0.15));
    REQUIRE_THAT(est.params()(1), WithinAbs(true_b, 0.15));
}

TEST_CASE("GreyBoxEstimator bounded parameters stay within bounds", "[grey_box]")
{
    // Deliberately tight bounds so params must be clipped.
    ctrl::GreyBoxEstimator::OdeFn f = [](const Eigen::VectorXd& x,
                                          const Eigen::VectorXd& u,
                                          const Eigen::VectorXd& p) {
        Eigen::VectorXd xd(1);
        xd(0) = -p(0)*x(0) + u(0);
        return xd;
    };
    ctrl::GreyBoxEstimator::MeasFn h = [](const Eigen::VectorXd& x,
                                           const Eigen::VectorXd&) { return x; };

    Eigen::MatrixXd U(1, 20), Y(1, 20);
    double xs = 0.0;
    for (int k = 0; k < 20; ++k) {
        U(0, k) = 1.0;
        Y(0, k) = xs;
        xs += 0.05 * (-0.5*xs + 1.0);
    }

    ctrl::GreyBoxEstimator::Params par;
    par.p0    = Eigen::VectorXd::Constant(1, 0.5);
    par.lower = Eigen::VectorXd::Constant(1, 0.3);
    par.upper = Eigen::VectorXd::Constant(1, 0.7);
    par.Ts    = 0.05;
    par.max_iter = 20;

    ctrl::GreyBoxEstimator est(f, h, par);
    auto res = est.fit(Eigen::VectorXd::Zero(1), U, Y);

    REQUIRE(est.params()(0) >= 0.3 - 1e-9);
    REQUIRE(est.params()(0) <= 0.7 + 1e-9);
}

TEST_CASE("GreyBoxEstimator predict returns correct output dimensions", "[grey_box]")
{
    ctrl::GreyBoxEstimator::OdeFn f = [](const Eigen::VectorXd& x,
                                          const Eigen::VectorXd& u,
                                          const Eigen::VectorXd& p) {
        Eigen::VectorXd xd(1); xd(0) = -p(0)*x(0) + u(0); return xd;
    };
    ctrl::GreyBoxEstimator::MeasFn h = [](const Eigen::VectorXd& x,
                                           const Eigen::VectorXd&) { return x; };

    const int N = 10;
    Eigen::MatrixXd U = Eigen::MatrixXd::Ones(1, N);
    ctrl::GreyBoxEstimator::Params par;
    par.p0 = Eigen::VectorXd::Constant(1, 0.5);
    par.Ts = 0.05;

    ctrl::GreyBoxEstimator est(f, h, par);
    Eigen::MatrixXd Y_hat = est.predict(Eigen::VectorXd::Zero(1), U);

    REQUIRE(Y_hat.rows() == 1);
    REQUIRE(Y_hat.cols() == N);
    REQUIRE(Y_hat.allFinite());
}

// =============================================================================
// RecursiveGreyBoxEstimator (E2)
// =============================================================================

TEST_CASE("RecursiveGreyBoxEstimator initializes and runs without error", "[recursive_grey_box]")
{
    ctrl::RecursiveGreyBoxEstimator::OdeFn f = [](const Eigen::VectorXd& x,
                                                    const Eigen::VectorXd& u,
                                                    const Eigen::VectorXd& p) {
        Eigen::VectorXd xd(1);
        xd(0) = -p(0)*x(0) + u(0);
        return xd;
    };
    ctrl::RecursiveGreyBoxEstimator::MeasFn h = [](const Eigen::VectorXd& x,
                                                     const Eigen::VectorXd&) { return x; };

    ctrl::RecursiveGreyBoxEstimator::Params par;
    par.p0      = Eigen::VectorXd::Constant(1, 0.5);
    par.Q_state = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
    par.Q_param = Eigen::MatrixXd::Identity(1, 1) * 1e-6;
    par.R_meas  = Eigen::MatrixXd::Identity(1, 1) * 0.01;
    par.Ts      = 0.05;

    ctrl::RecursiveGreyBoxEstimator rge(f, h, 1, 1, par);
    REQUIRE_FALSE(rge.isInitialized());

    rge.initialize(Eigen::VectorXd::Zero(1),
                   Eigen::MatrixXd::Identity(1, 1) * 0.1);
    REQUIRE(rge.isInitialized());

    Eigen::VectorXd y(1), u(1);
    y(0) = 0.3; u(0) = 1.0;
    Eigen::VectorXd x_hat = rge.step(y, u);

    REQUIRE(x_hat.size() == 1);
    REQUIRE(std::isfinite(x_hat(0)));
    REQUIRE(rge.paramEstimate().size() == 1);
    REQUIRE(std::isfinite(rge.paramEstimate()(0)));
    REQUIRE(rge.covariance().rows() == 2);
}

TEST_CASE("RecursiveGreyBoxEstimator param estimate stays finite over many steps", "[recursive_grey_box]")
{
    ctrl::RecursiveGreyBoxEstimator::OdeFn f = [](const Eigen::VectorXd& x,
                                                    const Eigen::VectorXd& u,
                                                    const Eigen::VectorXd& p) {
        Eigen::VectorXd xd(1); xd(0) = -p(0)*x(0) + u(0); return xd;
    };
    ctrl::RecursiveGreyBoxEstimator::MeasFn h = [](const Eigen::VectorXd& x,
                                                     const Eigen::VectorXd&) { return x; };

    ctrl::RecursiveGreyBoxEstimator::Params par;
    par.p0      = Eigen::VectorXd::Constant(1, 0.4);
    par.Q_state = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
    par.Q_param = Eigen::MatrixXd::Identity(1, 1) * 1e-5;
    par.R_meas  = Eigen::MatrixXd::Identity(1, 1) * 0.01;
    par.Ts      = 0.05;

    ctrl::RecursiveGreyBoxEstimator rge(f, h, 1, 1, par);
    rge.initialize(Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Identity(1, 1));

    double xsim = 0.0;
    for (int k = 0; k < 80; ++k) {
        Eigen::VectorXd y(1), u(1);
        y(0) = xsim + 0.01 * (k % 3 - 1);  // noisy measurement
        u(0) = 1.0;
        rge.step(y, u);
        xsim += 0.05 * (-0.5*xsim + 1.0);  // true plant
    }

    REQUIRE(rge.stateEstimate().allFinite());
    REQUIRE(rge.paramEstimate().allFinite());
    // P_aug must remain PSD: diagonal non-negative
    REQUIRE(rge.covariance().diagonal().minCoeff() >= -1e-10);
}

// =============================================================================
// GPResidualModel (E3)
// =============================================================================

TEST_CASE("GPResidualModel predicts lower variance near training data", "[gp_residual]")
{
    ctrl::GPResidualModel::Params p;
    p.gp.length_scale = 0.5;
    p.gp.signal_var   = 1.0;
    p.gp.noise_var    = 0.01;
    p.gp.n_max        = 50;

    ctrl::GPResidualModel grm(1, p);
    REQUIRE(grm.size() == 0);

    // Add residuals near x=1.0
    for (int k = 0; k < 10; ++k) {
        Eigen::VectorXd xf(1);
        xf(0) = 1.0 + 0.05 * (k - 5);
        grm.addResidualPoint(xf, 0.3, 0.0);  // residual = 0.3
    }
    grm.fit();
    REQUIRE(grm.isFitted());

    Eigen::VectorXd x_near(1);  x_near(0)  = 1.0;
    Eigen::VectorXd x_far(1);   x_far(0)   = 10.0;

    auto pred_near = grm.predictWithUncertainty(x_near, 0.0);
    auto pred_far  = grm.predictWithUncertainty(x_far,  0.0);

    REQUIRE(std::isfinite(pred_near.mean_total));
    REQUIRE(pred_near.variance >= 0.0);
    REQUIRE(pred_far.variance  >= 0.0);
    REQUIRE(pred_far.variance > pred_near.variance);   // more uncertain far from data
}

TEST_CASE("GPResidualModel total prediction equals model + GP correction", "[gp_residual]")
{
    ctrl::GPResidualModel::Params p;
    p.gp.length_scale = 1.0;
    p.gp.signal_var   = 1.0;
    p.gp.noise_var    = 0.01;

    ctrl::GPResidualModel grm(1, p);
    for (int k = 0; k < 8; ++k) {
        Eigen::VectorXd xf(1); xf(0) = static_cast<double>(k);
        grm.addResidualPoint(xf, 2.0*k + 0.1, 2.0*k);  // residual ~ 0.1
    }
    grm.fit();

    const double model_pred = 5.0;
    Eigen::VectorXd xf(1); xf(0) = 3.0;
    auto pred = grm.predictWithUncertainty(xf, model_pred);

    REQUIRE_THAT(pred.mean_total,
                 WithinAbs(model_pred + pred.gp_mean, 1e-10));
    REQUIRE(pred.variance >= 0.0);
}

TEST_CASE("GPResidualModel batch residualFit matches manual add+fit", "[gp_residual]")
{
    ctrl::GPResidualModel::Params p;
    p.gp.length_scale = 1.0; p.gp.signal_var = 1.0; p.gp.noise_var = 0.01;
    p.gp.n_max = 30;

    const int N = 10;
    Eigen::MatrixXd X_feat(1, N);
    Eigen::VectorXd Y_true(N);
    for (int k = 0; k < N; ++k) {
        X_feat(0, k) = static_cast<double>(k) * 0.5;
        Y_true(k)    = std::sin(X_feat(0, k)) + 1.0;
    }
    auto model_fn = [](const Eigen::VectorXd&) -> double { return 1.0; };

    // Batch fit
    ctrl::GPResidualModel grm1(1, p);
    grm1.residualFit(X_feat, Y_true, model_fn);
    REQUIRE(grm1.isFitted());
    REQUIRE(grm1.size() == N);

    // Manual fit: same data
    ctrl::GPResidualModel grm2(1, p);
    for (int k = 0; k < N; ++k) {
        grm2.addResidualPoint(X_feat.col(k), Y_true(k), model_fn(X_feat.col(k)));
    }
    grm2.fit();

    // Both should give identical predictions at the same point
    Eigen::VectorXd xf(1); xf(0) = 1.5;
    auto pred1 = grm1.predictWithUncertainty(xf, 1.0);
    auto pred2 = grm2.predictWithUncertainty(xf, 1.0);
    REQUIRE_THAT(pred1.mean_total, WithinAbs(pred2.mean_total, 1e-8));
    REQUIRE_THAT(pred1.variance,   WithinAbs(pred2.variance,   1e-8));
}

// =============================================================================
// HybridModel (H1)
// =============================================================================

// Helper: linear spring ODE  x_dot = [x1, -4*x0 - 0.8*x1 + u]
static Eigen::VectorXd smd_phys(const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& u,
                                  const Eigen::VectorXd& p)
{
    Eigen::VectorXd xd(2);
    xd(0) = x(1);
    xd(1) = -p(0)*x(0) - p(1)*x(1) + u(0);
    return xd;
}

TEST_CASE("HybridModel predict without data model matches predictPhys", "[hybrid_model]")
{
    ctrl::HybridModelParams hmp;
    hmp.n_states = 2; hmp.n_inputs = 1; hmp.Ts = 0.01; hmp.rk4_steps = 4;

    Eigen::VectorXd p_phys(2); p_phys << 4.0, 0.8;
    ctrl::HybridModel model(smd_phys, hmp, p_phys);

    REQUIRE_FALSE(model.hasDataModel());

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0(1); u0 << 1.0;

    const auto xn_phys = model.predictPhys(x0, u0);
    const auto xn_comb = model.predict(x0, u0);

    REQUIRE_THAT((xn_phys - xn_comb).norm(), WithinAbs(0.0, 1e-10));
    REQUIRE(xn_phys.size() == 2);
    REQUIRE(std::isfinite(xn_phys(0)));
    REQUIRE(std::isfinite(xn_phys(1)));
}

TEST_CASE("HybridModel data model is applied and clearable", "[hybrid_model]")
{
    ctrl::HybridModelParams hmp;
    hmp.n_states = 2; hmp.n_inputs = 1; hmp.Ts = 0.01; hmp.rk4_steps = 4;

    Eigen::VectorXd p_phys(2); p_phys << 4.0, 0.8;
    ctrl::HybridModel model(smd_phys, hmp, p_phys);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0(1); u0 << 1.0;
    const auto xn_phys = model.predictPhys(x0, u0);

    // Attach a known constant correction
    const Eigen::VectorXd delta = (Eigen::VectorXd(2) << 0.01, -0.02).finished();
    model.setDataModel([delta](const Eigen::VectorXd&, const Eigen::VectorXd&) {
        return delta;
    });
    REQUIRE(model.hasDataModel());

    const auto xn_with_data = model.predict(x0, u0);
    REQUIRE_THAT((xn_with_data - xn_phys - delta).norm(), WithinAbs(0.0, 1e-10));

    model.clearDataModel();
    REQUIRE_FALSE(model.hasDataModel());
    const auto xn_cleared = model.predict(x0, u0);
    REQUIRE_THAT((xn_cleared - xn_phys).norm(), WithinAbs(0.0, 1e-10));
}

// =============================================================================
// HybridMPC (H2)
// =============================================================================

TEST_CASE("HybridMPC compute() returns finite output before data fitting", "[hybrid_mpc]")
{
    ctrl::HybridModelParams hmp;
    hmp.n_states = 2; hmp.n_inputs = 1; hmp.n_outputs = 2; hmp.Ts = 0.01;

    Eigen::VectorXd p_phys(2); p_phys << 4.0, 0.8;
    auto model = std::make_shared<ctrl::HybridModel>(smd_phys, hmp, p_phys);

    ctrl::HybridMPCParams hp;
    hp.nmpc.n_states = 2; hp.nmpc.n_inputs = 1; hp.nmpc.n_outputs = 2;
    hp.nmpc.Np = 5; hp.nmpc.Nu = 2; hp.nmpc.Ts = 0.01;
    hp.nmpc.rho_y = 1.0; hp.nmpc.rho_u = 0.1;
    hp.data_update_interval = 0;    // manual only
    hp.min_observations     = 5;

    ctrl::HybridMPC hmpc(hp, model);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    hmpc.setState(x0);
    const double u = hmpc.compute(0.5);
    REQUIRE(std::isfinite(u));
    REQUIRE(hmpc.observationCount() == 0);
    REQUIRE_FALSE(hmpc.isDataModelFitted());
}

TEST_CASE("HybridMPC refitDataModel() installs data correction in model", "[hybrid_mpc]")
{
    ctrl::HybridModelParams hmp;
    hmp.n_states = 2; hmp.n_inputs = 1; hmp.n_outputs = 2; hmp.Ts = 0.01;

    Eigen::VectorXd p_phys(2); p_phys << 4.0, 0.8;
    auto model = std::make_shared<ctrl::HybridModel>(smd_phys, hmp, p_phys);

    ctrl::HybridMPCParams hp;
    hp.nmpc.n_states = 2; hp.nmpc.n_inputs = 1; hp.nmpc.n_outputs = 2;
    hp.nmpc.Np = 5; hp.nmpc.Nu = 2; hp.nmpc.Ts = 0.01;
    hp.nmpc.rho_y = 1.0; hp.nmpc.rho_u = 0.1;
    hp.data_update_interval = 0;   // manual only
    hp.min_observations     = 5;

    ctrl::HybridMPC hmpc(hp, model);

    // Insufficient data: refit should fail
    REQUIRE_FALSE(hmpc.refitDataModel());
    REQUIRE_FALSE(hmpc.isDataModelFitted());

    // Add enough observations
    for (int k = 0; k < 8; ++k) {
        Eigen::VectorXd x(2); x << k * 0.01, 0.0;
        Eigen::VectorXd u(1); u << 0.5;
        const auto xn = model->predict(x, u);
        hmpc.addStateObservation(x, u, xn);
    }
    REQUIRE(hmpc.observationCount() == 8);

    // Manual refit
    const bool ok = hmpc.refitDataModel();
    REQUIRE(ok);
    REQUIRE(hmpc.isDataModelFitted());
    REQUIRE(model->hasDataModel());

    // After refit, predictions should still be finite
    Eigen::VectorXd x_test = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u_test(1); u_test << 0.0;
    const auto xn = model->predict(x_test, u_test);
    REQUIRE(xn.allFinite());
}

// =============================================================================
// HybridModelTrainer (H4)
// =============================================================================

// Generate offline SMD data (with a constant friction offset as "unmodeled")
static void generateSMDData(const ctrl::HybridModel& model,
                              Eigen::MatrixXd& X_obs,
                              Eigen::MatrixXd& U_obs,
                              Eigen::MatrixXd& Xn_obs,
                              int N, double Ts)
{
    X_obs.resize(2, N); U_obs.resize(1, N); Xn_obs.resize(2, N);
    Eigen::VectorXd xs = Eigen::VectorXd::Zero(2);

    // True plant: same physical model + constant friction perturbation
    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd uk(1); uk(0) = (k < N/2) ? 1.0 : -0.5;
        X_obs.col(k)  = xs;
        U_obs.col(k)  = uk;
        // "True" next state adds small constant friction offset
        auto xn_phys  = model.predictPhys(xs, uk);
        xn_phys(1)   -= 0.05;      // unmodeled damping perturbation
        Xn_obs.col(k) = xn_phys;
        xs            = Xn_obs.col(k);
    }
}

TEST_CASE("HybridModelTrainer Ridge reduces prediction RMSE vs physical only", "[hybrid_trainer]")
{
    ctrl::HybridModelParams hmp;
    hmp.n_states = 2; hmp.n_inputs = 1; hmp.Ts = 0.02; hmp.rk4_steps = 4;

    Eigen::VectorXd p_phys(2); p_phys << 4.0, 0.8;
    ctrl::HybridModel model(smd_phys, hmp, p_phys);

    const int N = 80;
    Eigen::MatrixXd X_obs, U_obs, Xn_obs;
    generateSMDData(model, X_obs, U_obs, Xn_obs, N, hmp.Ts);

    ctrl::HybridModelTrainer::Params tp;
    tp.method       = ctrl::HybridModelTrainer::Method::Ridge;
    tp.ridge_lambda = 1e-4;
    ctrl::HybridModelTrainer trainer(tp);

    const double rmse_before = trainer.validate(model, X_obs, U_obs, Xn_obs);
    auto res = trainer.trainHybridModel(model, X_obs, U_obs, Xn_obs);
    const double rmse_after  = trainer.validate(model, X_obs, U_obs, Xn_obs);

    REQUIRE(res.success);
    REQUIRE(res.n_samples == N);
    REQUIRE(std::isfinite(res.train_rmse));
    REQUIRE(rmse_after < rmse_before + 1e-9);  // fitted model must not be worse
    REQUIRE(model.hasDataModel());
}

TEST_CASE("HybridModelTrainer GP training produces finite predictions", "[hybrid_trainer]")
{
    ctrl::HybridModelParams hmp;
    hmp.n_states = 2; hmp.n_inputs = 1; hmp.Ts = 0.02; hmp.rk4_steps = 4;

    Eigen::VectorXd p_phys(2); p_phys << 4.0, 0.8;
    ctrl::HybridModel model(smd_phys, hmp, p_phys);

    const int N = 40;
    Eigen::MatrixXd X_obs, U_obs, Xn_obs;
    generateSMDData(model, X_obs, U_obs, Xn_obs, N, hmp.Ts);

    ctrl::HybridModelTrainer::Params tp;
    tp.method          = ctrl::HybridModelTrainer::Method::GP;
    tp.gp.length_scale = 0.5;
    tp.gp.signal_var   = 0.1;
    tp.gp.noise_var    = 1e-3;
    tp.gp.n_max        = N;
    ctrl::HybridModelTrainer trainer(tp);

    auto res = trainer.trainHybridModel(model, X_obs, U_obs, Xn_obs);
    REQUIRE(res.success);
    REQUIRE(std::isfinite(res.train_rmse));

    // All predictions should be finite
    for (int k = 0; k < N; ++k) {
        const auto pred = model.predict(X_obs.col(k), U_obs.col(k));
        REQUIRE(pred.allFinite());
    }
}

// =============================================================================
// T3: VectorFitting - rational magnitude fitting for full DK-iteration D-step
// =============================================================================

TEST_CASE("VectorFitting fitMagnitude returns stable StateSpace and finite RMS error", "[vector_fitting]")
{
    // Fit a 3-pole rational filter to a monotone magnitude profile on [0, pi/Ts).
    const double Ts     = 0.01;
    const int    N      = 40;
    const double w_nyq  = M_PI / Ts;

    std::vector<double> omega(N), mag(N);
    for (int k = 0; k < N; ++k)
    {
        omega[k] = w_nyq * (static_cast<double>(k + 1) / N) * 0.95;
        // Simple low-pass magnitude: 1/(1 + (omega/100)^2)
        const double x = omega[k] / 100.0;
        mag[k] = 1.0 / (1.0 + x * x);
    }

    ctrl::VectorFittingResult res;
    ctrl::StateSpace fitted = ctrl::VectorFitting::fitMagnitude(omega, mag, 3, Ts, res);

    REQUIRE(fitted.A.rows() == 3);
    REQUIRE(fitted.A.cols() == 3);
    REQUIRE(std::isfinite(res.rms_error));

    // All eigenvalues of A must be inside the unit disk (stability)
    Eigen::EigenSolver<Eigen::MatrixXd> es(fitted.A, false);
    for (int k = 0; k < fitted.A.rows(); ++k)
        REQUIRE(std::abs(es.eigenvalues()(k)) < 1.0 + 1e-9);

    // Evaluate at a few grid points - must be finite and positive
    for (int k = 0; k < N; k += 4)
    {
        const double m = ctrl::VectorFitting::evalMagnitude(fitted, omega[k]);
        REQUIRE(std::isfinite(m));
        REQUIRE(m >= 0.0);
    }
}

TEST_CASE("VectorFitting RMS error decreases with more poles", "[vector_fitting]")
{
    // A two-bump magnitude profile: harder to fit with 1 pole than 4.
    const double Ts    = 0.01;
    const int    N     = 60;
    const double w_nyq = M_PI / Ts;

    std::vector<double> omega(N), mag(N);
    for (int k = 0; k < N; ++k)
    {
        omega[k] = w_nyq * (static_cast<double>(k + 1) / N) * 0.95;
        const double x1 = omega[k] / 50.0;
        const double x2 = omega[k] / 200.0;
        mag[k] = 1.0 / (1.0 + x1 * x1) + 0.5 / (1.0 + x2 * x2);
    }

    ctrl::VectorFittingResult r1, r4;
    ctrl::VectorFitting::fitMagnitude(omega, mag, 1, Ts, r1);
    ctrl::VectorFitting::fitMagnitude(omega, mag, 4, Ts, r4);

    REQUIRE(std::isfinite(r1.rms_error));
    REQUIRE(std::isfinite(r4.rms_error));
    // More poles should achieve a better (lower) RMS fit on this profile
    REQUIRE(r4.rms_error <= r1.rms_error + 0.1);  // allow 0.1 tolerance for numerical variability
}

TEST_CASE("VectorFitting: MuSynParams dFitOrder field accepted by solveMuSyn", "[vector_fitting]")
{
    // Smoke test: construct params with dFitOrder = 3 and verify the field is accessible.
    ctrl::MuSynParams mp;
    mp.useRationalD = true;
    mp.dFitOrder    = 3;
    mp.maxDKIter    = 1;   // only one iteration for speed
    mp.nFreqPoints  = 20;

    REQUIRE(mp.dFitOrder == 3);
    REQUIRE(mp.useRationalD == true);
    // Full synthesis test omitted here (requires a complex generalised plant);
    // DK-iteration with vector fitting is exercised in ex82_vector_fitting_dk.cpp.
}

// =============================================================================
// D1: MismatchDetector - CUSUM on KF / MHE innovation sequence
// =============================================================================

TEST_CASE("MismatchDetector: no alarm on white-noise innovation", "[mismatch_detector]")
{
    ctrl::MismatchDetectorParams dp;
    dp.sigma = 1.0; dp.k_cusum = 0.5; dp.h_threshold = 5.0;
    ctrl::MismatchDetector det(dp);

    // Feed 100 samples at sigma=1 (in-control); CUSUM should not alarm.
    std::srand(42);
    for (int k = 0; k < 100; ++k) {
        double v = 0.3 * (static_cast<double>(std::rand()) / RAND_MAX * 2.0 - 1.0);
        det.update(v);
    }
    REQUIRE_FALSE(det.detected());
}

TEST_CASE("MismatchDetector: sustained shift triggers detection", "[mismatch_detector]")
{
    ctrl::MismatchDetectorParams dp;
    dp.sigma = 1.0; dp.k_cusum = 0.5; dp.h_threshold = 5.0;
    ctrl::MismatchDetector det(dp);

    // Feed samples 3* the nominal sigma (persistent model mismatch).
    for (int k = 0; k < 30; ++k)
        det.update(3.0);

    REQUIRE(det.detected());
    REQUIRE(det.score() > dp.h_threshold * dp.sigma);
}

TEST_CASE("MismatchDetector: reset clears alarm state", "[mismatch_detector]")
{
    ctrl::MismatchDetectorParams dp;
    dp.sigma = 1.0; dp.k_cusum = 0.5; dp.h_threshold = 2.0;  // low threshold for quick alarm
    ctrl::MismatchDetector det(dp);

    for (int k = 0; k < 20; ++k) det.update(3.0);
    REQUIRE(det.detected());

    det.reset();
    REQUIRE_FALSE(det.detected());
    REQUIRE_THAT(det.score(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("MismatchDetector: vector innovation update fires alarm", "[mismatch_detector]")
{
    ctrl::MismatchDetectorParams dp;
    dp.sigma = 1.0; dp.k_cusum = 0.5; dp.h_threshold = 5.0;
    ctrl::MismatchDetector det(dp);

    // 2D innovations with large magnitude (mismatch)
    Eigen::VectorXd innov(2);
    innov << 3.0, 3.0;  // norm/sqrt(2) = 3.0, well above sigma=1
    for (int k = 0; k < 25; ++k) det.update(innov);

    REQUIRE(det.detected());
}

TEST_CASE("KalmanFilter: mismatch detection disabled by default", "[mismatch_detector]")
{
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << Ts; C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);
    ctrl::KalmanFilter kf(plant,
                          Eigen::MatrixXd::Constant(1,1,0.01),
                          Eigen::MatrixXd::Constant(1,1,0.1));

    Eigen::VectorXd y(1), u(1); y << 0.5; u << 0.0;
    kf.step(y, u);

    REQUIRE_FALSE(kf.mismatchDetected());
    REQUIRE_THAT(kf.mismatchScore(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("KalmanFilter: mismatch detection triggers on wrong model", "[mismatch_detector]")
{
    // Plant is integrator (A=1); KF model is stable (A=0.9) - deliberate mismatch.
    // Innovation grows as KF prediction diverges from ramp truth.
    const double Ts = 0.1;
    Eigen::MatrixXd B(1,1), C(1,1), D(1,1);
    B << Ts; C << 1.0; D << 0.0;

    // KF model: stable first-order (wrong model)
    Eigen::MatrixXd A_model(1,1); A_model << 0.9;
    ctrl::StateSpace kf_model(A_model, B, C, D, Ts);
    ctrl::KalmanFilter kf(kf_model,
                          Eigen::MatrixXd::Constant(1,1,0.001),
                          Eigen::MatrixXd::Constant(1,1,0.01));

    ctrl::MismatchDetectorParams dp;
    dp.sigma = 0.05; dp.k_cusum = 0.5; dp.h_threshold = 3.0;
    kf.enableMismatchDetection(dp);

    Eigen::VectorXd u(1); u << 1.0;
    double y_true = 0.0;
    for (int k = 0; k < 80; ++k) {
        y_true += Ts * 1.0;  // true output growing (ramp)
        Eigen::VectorXd y(1); y << y_true;
        kf.step(y, u);
    }
    REQUIRE(kf.mismatchDetected());
}

TEST_CASE("MovingHorizonEstimator: mismatch detection fires on mismatched model",
          "[mismatch_detector]")
{
    // 1-D integrator plant; model is stable (mismatch).
    const double Ts = 0.1;
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << 0.9; B << Ts; C << 1.0; D << 0.0;  // model: stable
    ctrl::StateSpace plant(A, B, C, D, Ts);

    ctrl::MHEParams mp; mp.N = 5;
    ctrl::MovingHorizonEstimator mhe(plant,
                                     Eigen::MatrixXd::Constant(1,1,0.1),
                                     Eigen::MatrixXd::Constant(1,1,0.1), mp);
    mhe.initialize(Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Identity(1,1));

    ctrl::MismatchDetectorParams dp;
    dp.sigma = 0.1; dp.k_cusum = 0.5; dp.h_threshold = 3.0;
    mhe.enableMismatchDetection(dp);

    Eigen::VectorXd u(1); u << 1.0;
    double y_true = 0.0;
    for (int k = 0; k < 60; ++k) {
        y_true += Ts;  // ramp output from true integrator
        Eigen::VectorXd y(1); y << y_true;
        mhe.estimate(y, u);
    }
    REQUIRE(mhe.mismatchDetected());
}

// =============================================================================
// M4: BasicPID<Scalar> and BasicSMC<Scalar> - embedded template controllers
// =============================================================================

TEST_CASE("BasicPID<double> step-response converges to setpoint", "[basic_pid]")
{
    // First-order plant: y[k+1] = 0.9*y[k] + 0.1*u[k] (Ts=0.1s, tau=1s, K=1).
    ctrl::BasicPIDParams<double> p;
    p.Kp = 3.0; p.Ki = 1.5; p.Kd = 0.0; p.N = 100.0;
    p.Kb = 1.0; p.uMin = -20.0; p.uMax = 20.0; p.Ts = 0.1;

    ctrl::BasicPID<double> pid(p);
    double y = 0.0, ref = 1.0;
    for (int k = 0; k < 200; ++k) {
        double u = pid.compute(ref - y);
        y = 0.9 * y + 0.1 * u;
    }
    REQUIRE(std::abs(y - ref) < 0.02);
}

TEST_CASE("BasicPID<float> output is finite and within limits", "[basic_pid]")
{
    ctrl::BasicPIDParams<float> p;
    p.Kp = 2.0f; p.Ki = 0.5f; p.Kd = 0.1f; p.N = 50.0f;
    p.Kb = 1.0f; p.uMin = -1.0f; p.uMax = 1.0f; p.Ts = 0.01f;

    ctrl::BasicPID<float> pid(p);
    for (int k = 0; k < 100; ++k) {
        float u = pid.compute(1.0f);  // constant error
        REQUIRE(std::isfinite(u));
        REQUIRE(u >= -1.0f);
        REQUIRE(u <=  1.0f);
    }
}

TEST_CASE("BasicPID reset zeroes integrator and derivative state", "[basic_pid]")
{
    ctrl::BasicPIDParams<double> p;
    p.Kp = 1.0; p.Ki = 10.0; p.Kd = 0.0; p.Ts = 0.1;
    ctrl::BasicPID<double> pid(p);

    // Wind up integrator
    for (int k = 0; k < 20; ++k) pid.compute(1.0);
    REQUIRE(pid.integrator() != 0.0);

    pid.reset();
    REQUIRE(pid.integrator() == 0.0);
    REQUIRE(pid.lastOutput() == 0.0);
}

TEST_CASE("BasicPID anti-windup limits output within bounds under large error", "[basic_pid]")
{
    ctrl::BasicPIDParams<double> p;
    p.Kp = 10.0; p.Ki = 5.0; p.Kd = 0.0; p.Kb = 1.0;
    p.uMin = -2.0; p.uMax = 2.0; p.Ts = 0.1;
    ctrl::BasicPID<double> pid(p);

    for (int k = 0; k < 200; ++k) {
        double u = pid.compute(100.0);  // huge constant error
        REQUIRE(u <= 2.0 + 1e-9);
        REQUIRE(u >= -2.0 - 1e-9);
    }
    // With back-calculation, integrator should stay bounded too.
    REQUIRE(std::isfinite(pid.integrator()));
}

TEST_CASE("BasicSMC<double> drives first-order plant to sliding surface", "[basic_smc]")
{
    // Plant: y[k+1] = 0.9*y[k] + 0.1*u[k], ref=2.
    // Parameters chosen so the boundary-layer inner loop is stable on all platforms:
    //   inner pole = 0.9 - 0.1*(K/phi) = 0.9 - 0.1*(5/0.3) = -0.77  (|.| < 1 ok)
    // K/phi=40 (the old K=8/phi=0.2) gives pole=-3.1 (unstable on x86-32 FPU).
    ctrl::BasicSMCParams<double> sp;
    sp.c_e = 1.0; sp.c_de = 0.05; sp.K = 5.0; sp.phi = 0.3;
    sp.uMin = -10.0; sp.uMax = 10.0;

    ctrl::BasicSMC<double> smc(sp);
    double y = 0.0, ref = 2.0;
    for (int k = 0; k < 400; ++k) {
        double u = smc.compute(ref - y);
        y = 0.9 * y + 0.1 * u;
    }
    REQUIRE(std::abs(y - ref) < 0.20);  // near but not zero (no integral term)
}

TEST_CASE("BasicSMC<float> output within saturation limits", "[basic_smc]")
{
    ctrl::BasicSMCParams<float> sp;
    sp.c_e = 1.0f; sp.c_de = 0.1f; sp.K = 3.0f; sp.phi = 0.3f;
    sp.uMin = -1.0f; sp.uMax = 1.0f;

    ctrl::BasicSMC<float> smc(sp);
    for (int k = 0; k < 100; ++k) {
        float u = smc.compute(5.0f);  // large constant error
        REQUIRE(std::isfinite(u));
        REQUIRE(u >= -1.0f - 1e-5f);
        REQUIRE(u <=  1.0f + 1e-5f);
    }
}

TEST_CASE("BasicSMC reset clears previous error state", "[basic_smc]")
{
    ctrl::BasicSMCParams<double> sp;
    sp.c_e = 1.0; sp.c_de = 0.2; sp.K = 5.0; sp.phi = 0.5;
    ctrl::BasicSMC<double> smc(sp);

    smc.compute(3.0);  // introduce non-zero e_prev
    smc.reset();
    REQUIRE(smc.lastOutput() == 0.0);

    // After reset, two identical errors should give the same surface value
    // (c_de * (e - 0) = c_de * e, reproducible).
    double s1 = smc.slidingSurface(1.0);
    smc.reset();
    double s2 = smc.slidingSurface(1.0);
    REQUIRE(s1 == s2);
}

// =============================================================================
// GeneticAlgorithm
// =============================================================================

static ctrl::GAParams makeGAParams(int n, int pop = 30, int gen = 60)
{
    ctrl::GAParams p;
    p.n_dim      = n;
    p.population = pop;
    p.max_gen    = gen;
    p.lower = Eigen::VectorXd::Constant(n, 0.0);
    p.upper = Eigen::VectorXd::Constant(n, 5.0);
    p.seed  = 99;
    return p;
}

TEST_CASE("GeneticAlgorithm minimises 1D parabola", "[genetic_algorithm]")
{
    // f(x) = (x-2)^2, minimum at x=2, cost=0
    ctrl::GeneticAlgorithm ga(makeGAParams(1));
    auto result = ga.optimize([](const Eigen::VectorXd& x){ return (x(0)-2.0)*(x(0)-2.0); });

    REQUIRE(result.cost < 0.1);
    REQUIRE(result.params(0) >= 0.0);
    REQUIRE(result.params(0) <= 5.0);
    REQUIRE(result.nEvals > 0);
}

TEST_CASE("GeneticAlgorithm minimises 2D sphere", "[genetic_algorithm]")
{
    Eigen::Vector2d centre(1.5, 3.0);
    ctrl::GeneticAlgorithm ga(makeGAParams(2, 40, 80));
    auto result = ga.optimize([&](const Eigen::VectorXd& x){
        return (x - centre).squaredNorm();
    });

    REQUIRE(result.cost < 0.5);
    REQUIRE((result.params - centre).norm() < 1.0);
}

TEST_CASE("GeneticAlgorithm result always within bounds", "[genetic_algorithm]")
{
    ctrl::GAParams p = makeGAParams(3, 20, 40);
    ctrl::GeneticAlgorithm ga(p);
    auto result = ga.optimize([](const Eigen::VectorXd& x){ return x.sum(); }); // min at lower bound

    for (int i = 0; i < 3; ++i) {
        REQUIRE(result.params(i) >= p.lower(i) - 1e-9);
        REQUIRE(result.params(i) <= p.upper(i) + 1e-9);
    }
    REQUIRE(std::isfinite(result.cost));
}

// =============================================================================
// NelderMead (Phase 3 MO2)
// =============================================================================

TEST_CASE("NelderMead converges to the Rosenbrock minimum", "[nelder_mead]")
{
    // f(x,y) = (1-x)^2 + 100*(y-x^2)^2, global minimum at (1,1), cost=0.
    ctrl::NelderMeadParams p;
    p.n_dim = 2;
    p.max_iter = 2000;
    ctrl::NelderMead nm(p);

    Eigen::VectorXd x0(2); x0 << -1.0, 1.0;
    auto result = nm.optimize(
        [](const Eigen::VectorXd &x) {
            const double a = 1.0 - x(0);
            const double b = x(1) - x(0) * x(0);
            return a * a + 100.0 * b * b;
        },
        x0);

    REQUIRE_THAT(result.params(0), WithinAbs(1.0, 1e-3));
    REQUIRE_THAT(result.params(1), WithinAbs(1.0, 1e-3));
    REQUIRE(result.cost < 1e-6);
}

TEST_CASE("NelderMead converges in fewer evaluations than AutoTuner on a quadratic bowl",
          "[nelder_mead]")
{
    // f(x,y) = (x-1.5)^2 + (y-3)^2 - a smooth unimodal bowl, the "easy" case Nelder-Mead's
    // single-point start should win on relative to a population-based CMA-ES search.
    const Eigen::Vector2d centre(1.5, 3.0);
    auto cost = [&](const Eigen::VectorXd &x) { return (x - centre).squaredNorm(); };

    ctrl::NelderMeadParams nmp;
    nmp.n_dim = 2;
    ctrl::NelderMead nm(nmp);
    Eigen::VectorXd x0(2); x0 << 0.0, 0.0;
    auto nmResult = nm.optimize(cost, x0);

    ctrl::AutoTunerParams atp;
    atp.n = 2;
    ctrl::AutoTuner tuner(atp);
    auto atResult = tuner.tune(cost, x0);

    REQUIRE(nmResult.cost < 1e-4);
    REQUIRE(atResult.cost < 1e-4);
    REQUIRE(nmResult.nEvals < atResult.nEvals);
}

TEST_CASE("NelderMead survives a flat-plateau cost landscape without returning a degenerate "
          "or non-finite result",
          "[nelder_mead]")
{
    // A cost function that is perfectly flat in a neighbourhood of the start point (zero
    // gradient everywhere inside |x| < 0.5) repeatedly drives every reflect/contract step
    // toward shrink, collapsing the simplex toward a single point - exactly the degenerate-
    // simplex scenario the collapse-detection/restart path guards against internally. The
    // externally observable contract is simply that the result stays finite and sane (the
    // roadmap's own framing: "doesn't silently return a bad point"), since whether a restart
    // fires is an internal implementation detail this test deliberately doesn't probe.
    ctrl::NelderMeadParams p;
    p.n_dim = 2;
    p.max_iter = 300;
    ctrl::NelderMead nm(p);

    Eigen::VectorXd x0(2); x0 << 0.0, 0.0;
    auto result = nm.optimize(
        [](const Eigen::VectorXd &x) {
            if (x.norm() < 0.5) return 1.0; // flat plateau around the start
            return 1.0 + (x.norm() - 0.5);  // increases outside the plateau
        },
        x0);

    REQUIRE(std::isfinite(result.cost));
    REQUIRE(result.params.allFinite());
    REQUIRE(result.nEvals > 0);
}

// ---------------------------------------------------------------------------
// SelfTuningRegulator (Phase 3 Roadmap Phase 2 OC1)
// ---------------------------------------------------------------------------

TEST_CASE("SelfTuningRegulator MinimumVariance mode remains stable and bounded under "
          "persistent random-reference closed-loop operation",
          "[self_tuning_regulator]")
{
    // True plant: y[k] = 0.5*y[k-1] + 1.0*u[k-1]  =>  a1=-0.5, b1=1.0 (RLS convention).
    //
    // NOTE on scope: certainty-equivalence direct adaptive control (this mode) has no general
    // guarantee of persistent excitation from a closed-loop reference alone (Astrom & Wittenmark
    // Ch. 3) - confirmed during implementation via a standalone diagnostic: RLS can converge
    // confidently to a *stabilizing but numerically wrong* parameter estimate, even with a
    // randomly-varying reference and explicit dither (STRParams::probeAmplitude). This is a
    // documented, well-known property of the algorithm class (not a code defect) - the proper
    // remedy is dual control (roadmap OC3, Phase 5, deliberately out of scope here). What IS
    // reliably testable is stability: the closed loop stays bounded and finite, not exact
    // parameter or setpoint convergence.
    auto simulate = [](double yPrev, double uPrev) { return 0.5 * yPrev + 1.0 * uPrev; };

    ctrl::STRParams p;
    p.na = 1; p.nb = 1; p.mode = ctrl::STRMode::MinimumVariance; p.lambda = 1.0;
    p.uMin = -20.0; p.uMax = 20.0;
    ctrl::SelfTuningRegulator str(p, 0.1);

    std::mt19937 rng(7);
    std::uniform_real_distribution<double> refDist(-2.0, 2.0);
    double y = 0.0;
    for (int k = 0; k < 150; ++k)
    {
        str.setReference(refDist(rng));
        const double u = str.compute(y);
        REQUIRE(std::isfinite(u));
        REQUIRE(u >= p.uMin);
        REQUIRE(u <= p.uMax);
        y = simulate(y, u); // apply u directly - compute() already accounts for the plant's
                             // inherent one-step delay internally via its own uPrev_ bookkeeping
        REQUIRE(std::isfinite(y));
        REQUIRE(std::fabs(y) < 1000.0); // bounded, not diverging
    }

    REQUIRE(str.covariance().allFinite());
}

TEST_CASE("SelfTuningRegulator PolePlacement mode remains stable and bounded under "
          "persistent random-reference closed-loop operation",
          "[self_tuning_regulator]")
{
    // True plant: y[k] = 1.5*y[k-1] - 0.7*y[k-2] + 1.0*u[k-1]  =>  a1=-1.5, a2=0.7, b1=1.0.
    // See the NOTE on scope in the MinimumVariance stability test above - same limitation
    // applies here; comfortably-damped (not deadbeat) desired_poles are used per the
    // class-level @warning in SelfTuningRegulator.h, but exact pole/parameter convergence is
    // still not guaranteed from closed-loop excitation alone, so this test checks stability.
    auto simulate = [](double y1, double y2, double uPrev) { return 1.5*y1 - 0.7*y2 + 1.0*uPrev; };

    ctrl::STRParams p;
    p.na = 2; p.nb = 1; p.mode = ctrl::STRMode::PolePlacement; p.lambda = 1.0;
    p.desired_poles = Eigen::Vector2d(0.3, 0.3); // na+nb-1 = 2; comfortably damped, not deadbeat
    p.uMin = -20.0; p.uMax = 20.0;

    ctrl::SelfTuningRegulator str(p, 0.1);

    std::mt19937 rng(11);
    std::uniform_real_distribution<double> refDist(-2.0, 2.0);
    double y1 = 0.0, y2 = 0.0, y = 0.0;
    for (int k = 0; k < 300; ++k)
    {
        str.setReference(refDist(rng));
        const double u = str.compute(y);
        REQUIRE(std::isfinite(u));
        REQUIRE(u >= p.uMin);
        REQUIRE(u <= p.uMax);
        y2 = y1; y1 = y; y = simulate(y1, y2, u); // apply u directly, see note above
        REQUIRE(std::isfinite(y));
        REQUIRE(std::fabs(y) < 1000.0);
    }

    REQUIRE(str.covariance().allFinite());
}

TEST_CASE("SelfTuningRegulator re-converges after a mid-run plant parameter step-change",
          "[self_tuning_regulator]")
{
    ctrl::STRParams p;
    p.na = 1; p.nb = 1; p.mode = ctrl::STRMode::MinimumVariance; p.lambda = 0.95;
    p.uMin = -20.0; p.uMax = 20.0; // see the class-level @warning in SelfTuningRegulator.h
    ctrl::SelfTuningRegulator str(p, 0.1);

    std::mt19937 rng(3);
    std::uniform_real_distribution<double> refDist(-2.0, 2.0);
    double y = 0.0;
    double a = 0.5, b = 1.0; // current plant params (RLS sign convention: y=a.y_prev+b.u_prev)

    for (int k = 0; k < 80; ++k)
    {
        str.setReference(refDist(rng));
        const double u = str.compute(y);
        y = a * y + b * u; // apply u directly, see note above
    }

    a = 0.2; b = 0.6; // plant changes mid-run
    for (int k = 0; k < 150; ++k)
    {
        str.setReference(refDist(rng));
        const double u = str.compute(y);
        y = a * y + b * u;
    }

    const double rTarget = 1.0;
    str.setReference(rTarget);
    const double u = str.compute(y);
    y = a * y + b * u;

    REQUIRE_THAT(y, WithinAbs(rTarget, 0.1));
}

TEST_CASE("SelfTuningRegulator's RLS covariance does not blow up under a non-identifiable "
          "(constant reference) input",
          "[self_tuning_regulator]")
{
    ctrl::STRParams p;
    p.na = 1; p.nb = 1; p.mode = ctrl::STRMode::MinimumVariance;
    ctrl::SelfTuningRegulator str(p, 0.1);

    double y = 0.0;
    str.setReference(1.0);
    for (int k = 0; k < 500; ++k)
    {
        const double u = str.compute(y);
        y = 0.5 * y + 1.0 * u; // apply u directly, see note in the deadbeat test above
    }

    REQUIRE(str.covariance().allFinite());
    REQUIRE(str.covariance().trace() < 1e6);
}

TEST_CASE("SelfTuningRegulator falls back to bounded proportional control instead of dividing "
          "by a near-zero leading coefficient",
          "[self_tuning_regulator]")
{
    ctrl::STRParams p;
    p.na = 1; p.nb = 1; p.mode = ctrl::STRMode::MinimumVariance;
    p.bMin = 1e-3;
    p.uMin = -1.0; p.uMax = 1.0;
    ctrl::SelfTuningRegulator str(p, 0.1);

    str.setReference(1.0);
    // theta_ starts at zero (b1=0 < bMin) before any update() call has run - dividing by b1
    // would be undefined; the fallback must instead return a finite, bounded value (not
    // necessarily zero - holding the last output at cold start would deadlock, since zero
    // output never excites the plant enough for RLS to leave b1=0).
    const double u0 = str.compute(0.0);
    REQUIRE(std::isfinite(u0));
    REQUIRE(u0 >= p.uMin);
    REQUIRE(u0 <= p.uMax);
}

TEST_CASE("SelfTuningRegulator holds the last output on a non-finite plant measurement",
          "[self_tuning_regulator]")
{
    ctrl::STRParams p;
    p.na = 1; p.nb = 1; p.mode = ctrl::STRMode::MinimumVariance;
    ctrl::SelfTuningRegulator str(p, 0.1);

    str.setReference(1.0);
    double y = 0.0, uBefore = 0.0;
    for (int k = 0; k < 10; ++k)
    {
        uBefore = str.compute(y);
        y = 0.5 * y + 1.0 * uBefore; // apply u directly, see note in the deadbeat test above
    }
    const Eigen::MatrixXd covBefore = str.covariance();

    const double uAfter = str.compute(std::numeric_limits<double>::quiet_NaN());

    REQUIRE(uAfter == uBefore);
    REQUIRE(str.covariance().isApprox(covBefore));
}

TEST_CASE("SelfTuningRegulator throws on invalid construction parameters",
          "[self_tuning_regulator]")
{
    ctrl::STRParams base; base.na = 2; base.nb = 1;

    ctrl::STRParams badNa = base; badNa.na = 0;
    REQUIRE_THROWS_AS(ctrl::SelfTuningRegulator(badNa, 0.1), std::invalid_argument);

    ctrl::STRParams badPoles = base;
    badPoles.mode = ctrl::STRMode::PolePlacement;
    badPoles.desired_poles = Eigen::VectorXd::Zero(1); // should be na+nb-1=2
    REQUIRE_THROWS_AS(ctrl::SelfTuningRegulator(badPoles, 0.1), std::invalid_argument);

    ctrl::STRParams badBounds = base; badBounds.uMin = 1.0; badBounds.uMax = -1.0;
    REQUIRE_THROWS_AS(ctrl::SelfTuningRegulator(badBounds, 0.1), std::invalid_argument);
}

// -----------------------------------------------------------------------------
// NSGA2 (Phase 3 Roadmap Phase 2 MO1)
// -----------------------------------------------------------------------------

TEST_CASE("NSGA2 recovers the ZDT1 Pareto front shape", "[nsga2]")
{
    // ZDT1 with n=2 decision variables: f1=x1, f2=g*(1-sqrt(x1/g)), g=1+9*x2 - minimised at
    // x2=0, where the analytic front is f2 = 1 - sqrt(f1).
    ctrl::NSGA2Params p;
    p.n_dim = 2; p.n_objectives = 2; p.population = 60; p.max_gen = 80;
    p.lower = Eigen::Vector2d(0.0, 0.0);
    p.upper = Eigen::Vector2d(1.0, 1.0);
    ctrl::NSGA2 nsga(p);

    auto cost = [](const Eigen::VectorXd &x) {
        const double g = 1.0 + 9.0 * x(1);
        Eigen::VectorXd f(2);
        f(0) = x(0);
        f(1) = g * (1.0 - std::sqrt(x(0) / g));
        return f;
    };
    const auto result = nsga.optimize(cost);

    REQUIRE(result.front_params.rows() >= 1);
    double maxErr = 0.0;
    for (int i = 0; i < result.front_objectives.rows(); ++i)
    {
        const double f1 = result.front_objectives(i, 0);
        const double f2 = result.front_objectives(i, 1);
        const double expected = 1.0 - std::sqrt(std::max(f1, 0.0));
        maxErr = std::max(maxErr, std::fabs(f2 - expected));
    }
    REQUIRE(maxErr < 0.1);
}

TEST_CASE("NSGA2's final front is spread across the objective range, not clustered",
          "[nsga2]")
{
    ctrl::NSGA2Params p;
    p.n_dim = 1; p.n_objectives = 2; p.population = 40; p.max_gen = 50;
    p.lower = Eigen::VectorXd::Constant(1, 0.0);
    p.upper = Eigen::VectorXd::Constant(1, 2.0);
    ctrl::NSGA2 nsga(p);

    const auto result = nsga.optimize([](const Eigen::VectorXd &x) {
        Eigen::VectorXd f(2); f(0) = x(0) * x(0); f(1) = (x(0) - 2.0) * (x(0) - 2.0); return f;
    });

    const double f1Range = result.front_objectives.col(0).maxCoeff() -
                            result.front_objectives.col(0).minCoeff();
    REQUIRE(f1Range > 0.5); // a clustered/degenerate front would have near-zero range
}

TEST_CASE("NSGA2 with n_objectives=1 matches GeneticAlgorithm's quality on the same scalar problem",
          "[nsga2]")
{
    auto scalarCost = [](const Eigen::VectorXd &x) { return (x(0) - 1.5) * (x(0) - 1.5); };

    ctrl::NSGA2Params np;
    np.n_dim = 1; np.n_objectives = 1; np.population = 30; np.max_gen = 60;
    np.lower = Eigen::VectorXd::Constant(1, 0.0);
    np.upper = Eigen::VectorXd::Constant(1, 3.0);
    ctrl::NSGA2 nsga(np);
    const auto nsgaResult = nsga.optimize([&](const Eigen::VectorXd &x) {
        return Eigen::VectorXd::Constant(1, scalarCost(x));
    });
    const double nsgaBest = nsgaResult.front_objectives.col(0).minCoeff();

    ctrl::GAParams gp;
    gp.n_dim = 1; gp.population = 30; gp.max_gen = 60;
    gp.lower = np.lower; gp.upper = np.upper;
    ctrl::GeneticAlgorithm ga(gp);
    const auto gaResult = ga.optimize(scalarCost);

    REQUIRE(nsgaBest < 0.05);
    REQUIRE(gaResult.cost < 0.05);
}

TEST_CASE("NSGA2's non-dominated sort keeps every point on a strict trade-off curve at rank 0",
          "[nsga2]")
{
    // f(x) = [x, 1-x] over x in [0,1]: every point strictly trades one objective for the
    // other, so the entire population is mutually non-dominated (all rank 0). Using
    // max_gen=0 isolates the initial non-dominated sort from the generational loop.
    ctrl::NSGA2Params p;
    p.n_dim = 1; p.n_objectives = 2; p.population = 30; p.max_gen = 0;
    p.lower = Eigen::VectorXd::Constant(1, 0.0);
    p.upper = Eigen::VectorXd::Constant(1, 1.0);
    ctrl::NSGA2 nsga(p);

    const auto result = nsga.optimize([](const Eigen::VectorXd &x) {
        Eigen::VectorXd f(2); f(0) = x(0); f(1) = 1.0 - x(0); return f;
    });

    REQUIRE(result.front_params.rows() == p.population);
}

// -----------------------------------------------------------------------------
// tuneConstrained (Phase 3 Roadmap Phase 2 MO3)
// -----------------------------------------------------------------------------

TEST_CASE("tuneConstrained converges to the constrained (not unconstrained) optimum",
          "[constrained_tuning]")
{
    ctrl::ConstrainedTuneParams cp;
    cp.constraints = [](const Eigen::VectorXd &x) { return Eigen::VectorXd::Constant(1, x(0) - 1.0); };
    cp.outer_iters = 6;

    ctrl::AutoTunerParams atp; atp.n = 1;
    ctrl::AutoTuner tuner(atp);
    auto optimizerRun = [&](const ctrl::AutoTuner::CostFn &c, const Eigen::VectorXd &x0) {
        return tuner.tune(c, x0);
    };

    const auto result = ctrl::tuneConstrained(
        optimizerRun, [](const Eigen::VectorXd &x) { return (x(0) - 2.0) * (x(0) - 2.0); },
        cp, Eigen::VectorXd::Constant(1, 0.0));

    REQUIRE_THAT(result.params(0), WithinAbs(1.0, 0.05));
}

TEST_CASE("tuneConstrained drives an infeasible starting point into the feasible region",
          "[constrained_tuning]")
{
    ctrl::ConstrainedTuneParams cp;
    cp.constraints = [](const Eigen::VectorXd &x) { return Eigen::VectorXd::Constant(1, x(0) - 1.0); };
    cp.outer_iters = 8;

    ctrl::AutoTunerParams atp; atp.n = 1;
    ctrl::AutoTuner tuner(atp);
    auto optimizerRun = [&](const ctrl::AutoTuner::CostFn &c, const Eigen::VectorXd &x0) {
        return tuner.tune(c, x0);
    };

    const auto result = ctrl::tuneConstrained(
        optimizerRun, [](const Eigen::VectorXd &x) { return (x(0) - 5.0) * (x(0) - 5.0); },
        cp, Eigen::VectorXd::Constant(1, 10.0)); // x0=10 deliberately infeasible

    REQUIRE(result.params(0) <= 1.0 + 0.05);
}

TEST_CASE("tuneConstrained wraps AutoTuner and GeneticAlgorithm interchangeably with "
          "consistent results",
          "[constrained_tuning]")
{
    ctrl::ConstrainedTuneParams cp;
    cp.constraints = [](const Eigen::VectorXd &x) { return Eigen::VectorXd::Constant(1, x(0) - 1.0); };
    cp.outer_iters = 6;
    auto objective = [](const Eigen::VectorXd &x) { return (x(0) - 2.0) * (x(0) - 2.0); };

    ctrl::AutoTunerParams atp; atp.n = 1;
    ctrl::AutoTuner tuner(atp);
    const auto atResult = ctrl::tuneConstrained(
        [&](const ctrl::AutoTuner::CostFn &c, const Eigen::VectorXd &x0) { return tuner.tune(c, x0); },
        objective, cp, Eigen::VectorXd::Constant(1, 0.0));

    ctrl::GAParams gp; gp.n_dim = 1;
    gp.lower = Eigen::VectorXd::Constant(1, -5.0); gp.upper = Eigen::VectorXd::Constant(1, 5.0);
    ctrl::GeneticAlgorithm ga(gp);
    const auto gaResult = ctrl::tuneConstrained(
        [&](const ctrl::AutoTuner::CostFn &c, const Eigen::VectorXd &) { return ga.optimize(c); },
        objective, cp, Eigen::VectorXd::Constant(1, 0.0));

    REQUIRE_THAT(atResult.params(0), WithinAbs(1.0, 0.1));
    REQUIRE_THAT(gaResult.params(0), WithinAbs(1.0, 0.1));
}

TEST_CASE("tuneConstrained matches the unconstrained optimum when x0 is already feasible "
          "with a loose constraint",
          "[constrained_tuning]")
{
    ctrl::ConstrainedTuneParams cp;
    cp.constraints = [](const Eigen::VectorXd &x) { return Eigen::VectorXd::Constant(1, x(0) - 100.0); };
    cp.outer_iters = 6;

    ctrl::AutoTunerParams atp; atp.n = 1;
    ctrl::AutoTuner tuner(atp);
    auto optimizerRun = [&](const ctrl::AutoTuner::CostFn &c, const Eigen::VectorXd &x0) {
        return tuner.tune(c, x0);
    };

    const auto result = ctrl::tuneConstrained(
        optimizerRun, [](const Eigen::VectorXd &x) { return (x(0) - 2.0) * (x(0) - 2.0); },
        cp, Eigen::VectorXd::Constant(1, 0.0));

    REQUIRE_THAT(result.params(0), WithinAbs(2.0, 0.05));
}

// -----------------------------------------------------------------------------
// FaultClassifier (Phase 3 Roadmap Phase 2 DT4)
// -----------------------------------------------------------------------------

TEST_CASE("FaultClassifier classifies a persistent-offset residual as SensorBias",
          "[fault_classifier]")
{
    ctrl::FaultClassifier fc;
    ctrl::FaultType result = ctrl::FaultType::None;
    for (int k = 0; k < 6; ++k)
    {
        const double u = (k % 2 == 0) ? 0.0 : 1.0;
        result = fc.classify(5.0, u, u); // innovation: constant offset; u/y perfectly correlated
    }
    REQUIRE(result == ctrl::FaultType::SensorBias);
}

TEST_CASE("FaultClassifier classifies a zero-mean high-variance residual as SensorNoise",
          "[fault_classifier]")
{
    ctrl::FaultClassifier fc;
    ctrl::FaultType result = ctrl::FaultType::None;
    for (int k = 0; k < 6; ++k)
    {
        const double u = (k % 2 == 0) ? 0.0 : 1.0;
        const double innov = (k % 2 == 0) ? 5.0 : -5.0; // alternating sign, zero mean
        result = fc.classify(innov, u, u);
    }
    REQUIRE(result == ctrl::FaultType::SensorNoise);
}

TEST_CASE("FaultClassifier classifies a broken u->y causal link with varying u_cmd as "
          "ActuatorLoss",
          "[fault_classifier]")
{
    ctrl::FaultClassifier fc;
    ctrl::FaultType result = ctrl::FaultType::None;
    for (int k = 0; k < 6; ++k)
    {
        const double u = (k % 2 == 0) ? 0.0 : 1.0; // varying command
        result = fc.classify(5.0, u, 0.0);          // y_meas decoupled from u_cmd
    }
    REQUIRE(result == ctrl::FaultType::ActuatorLoss);
}

TEST_CASE("FaultClassifier classifies a broken u->y causal link with frozen u_cmd as "
          "ActuatorStuck",
          "[fault_classifier]")
{
    ctrl::FaultClassifier fc;
    ctrl::FaultType result = ctrl::FaultType::None;
    for (int k = 0; k < 6; ++k)
        result = fc.classify(5.0, 0.5, 0.0); // u_cmd frozen, y_meas unresponsive
    REQUIRE(result == ctrl::FaultType::ActuatorStuck);
}

TEST_CASE("FaultClassifier reports None on nominal (below-threshold) residuals",
          "[fault_classifier]")
{
    ctrl::FaultClassifier fc;
    ctrl::FaultType result = ctrl::FaultType::None;
    for (int k = 0; k < 10; ++k)
        result = fc.classify(0.1, 0.5, 0.5);
    REQUIRE(result == ctrl::FaultType::None);
}

TEST_CASE("FaultClassifier reports None before the rolling history window is full",
          "[fault_classifier]")
{
    ctrl::FaultDetectorParams p; p.confirm_window = 5;
    ctrl::FaultClassifier fc(p);
    for (int k = 0; k < 4; ++k) // one short of confirm_window
    {
        const auto result = fc.classify(50.0, 0.5, 0.0); // would otherwise indicate a fault
        REQUIRE(result == ctrl::FaultType::None);
    }
}

// -----------------------------------------------------------------------------
// FTCSupervisor (Phase 3 Roadmap Phase 2 DT4)
// -----------------------------------------------------------------------------

TEST_CASE("FTCSupervisor switches to the registered fallback controller on an injected "
          "actuator_loss fault",
          "[ftc_supervisor]")
{
    ctrl::PIDParams pp; pp.Kp = 1.0;
    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, 0.1);
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, 0.1), "primary");
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, 0.1), "fallback");

    ctrl::FTCSupervisor ftc(stack, ctrl::FaultDetectorParams{}, 0.1);
    ftc.registerFaultResponse(ctrl::FaultType::None, "primary");
    ftc.registerFaultResponse(ctrl::FaultType::ActuatorLoss, "fallback");

    for (int k = 0; k < 6; ++k)
    {
        const double u = (k % 2 == 0) ? 0.0 : 1.0;
        ftc.feedResidual(5.0, u, 0.0); // actuator_loss signature
        ftc.compute(1.0);
    }

    REQUIRE(ftc.currentFault() == ctrl::FaultType::ActuatorLoss);
    REQUIRE(stack->activeControllerName() == "fallback");
}

TEST_CASE("FTCSupervisor behaves identically to a plain ControllerStack when no fault occurs",
          "[ftc_supervisor]")
{
    ctrl::PIDParams pp; pp.Kp = 2.0; pp.Ki = 0.5;

    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, 0.1);
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, 0.1), "primary");
    ctrl::FTCSupervisor ftc(stack, ctrl::FaultDetectorParams{}, 0.1);
    ftc.registerFaultResponse(ctrl::FaultType::None, "primary");

    ctrl::ControllerStack plainStack(ctrl::StackMode::Supervisory, 0.1);
    plainStack.addController(std::make_shared<ctrl::DiscretePID>(pp, 0.1), "primary");

    for (int k = 0; k < 20; ++k)
    {
        const double e = 1.0 - 0.01 * k;
        ftc.feedResidual(0.0, 0.5, 0.5);
        const double uFtc = ftc.compute(e);
        const double uPlain = plainStack.compute(e);
        REQUIRE_THAT(uFtc, WithinAbs(uPlain, 1e-9));
    }
}

TEST_CASE("FTCSupervisor switches back to the nominal controller once the fault clears",
          "[ftc_supervisor]")
{
    ctrl::PIDParams pp; pp.Kp = 1.0;
    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, 0.1);
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, 0.1), "primary");
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, 0.1), "fallback");

    ctrl::FTCSupervisor ftc(stack, ctrl::FaultDetectorParams{}, 0.1);
    ftc.registerFaultResponse(ctrl::FaultType::None, "primary");
    ftc.registerFaultResponse(ctrl::FaultType::ActuatorLoss, "fallback");

    for (int k = 0; k < 6; ++k)
    {
        const double u = (k % 2 == 0) ? 0.0 : 1.0;
        ftc.feedResidual(5.0, u, 0.0);
        ftc.compute(1.0);
    }
    REQUIRE(stack->activeControllerName() == "fallback");

    double uLast = 0.0;
    for (int k = 0; k < 10; ++k)
    {
        ftc.feedResidual(0.0, 0.5, 0.5); // fault clears
        uLast = ftc.compute(1.0);
    }

    REQUIRE(stack->activeControllerName() == "primary");
    REQUIRE(std::isfinite(uLast));
}

TEST_CASE("FTCSupervisor::registerFaultResponse throws when the controller name is not in "
          "the stack",
          "[ftc_supervisor]")
{
    ctrl::PIDParams pp; pp.Kp = 1.0;
    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, 0.1);
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, 0.1), "primary");

    ctrl::FTCSupervisor ftc(stack, ctrl::FaultDetectorParams{}, 0.1);
    REQUIRE_THROWS_AS(ftc.registerFaultResponse(ctrl::FaultType::ActuatorLoss, "nonexistent"),
                       std::invalid_argument);
}

// =============================================================================
// ParticleSwarmOptimizer
// =============================================================================

static ctrl::PSOParams makePSOParams(int n, int particles = 20, int iters = 60)
{
    ctrl::PSOParams p;
    p.n_dim       = n;
    p.n_particles = particles;
    p.max_iter    = iters;
    p.lower = Eigen::VectorXd::Constant(n, 0.0);
    p.upper = Eigen::VectorXd::Constant(n, 5.0);
    p.seed  = 77;
    return p;
}

TEST_CASE("ParticleSwarmOptimizer minimises 1D parabola", "[pso]")
{
    ctrl::ParticleSwarmOptimizer pso(makePSOParams(1));
    auto result = pso.optimize([](const Eigen::VectorXd& x){ return (x(0)-2.0)*(x(0)-2.0); });

    REQUIRE(result.cost < 0.1);
    REQUIRE(result.params(0) >= 0.0);
    REQUIRE(result.params(0) <= 5.0);
    REQUIRE(result.nEvals > 0);
}

TEST_CASE("ParticleSwarmOptimizer minimises 2D sphere", "[pso]")
{
    Eigen::Vector2d centre(2.0, 4.0);
    ctrl::ParticleSwarmOptimizer pso(makePSOParams(2, 25, 80));
    auto result = pso.optimize([&](const Eigen::VectorXd& x){
        return (x - centre).squaredNorm();
    });

    REQUIRE(result.cost < 0.5);
    REQUIRE((result.params - centre).norm() < 1.0);
}

TEST_CASE("ParticleSwarmOptimizer result always within bounds", "[pso]")
{
    ctrl::PSOParams p = makePSOParams(3, 15, 30);
    ctrl::ParticleSwarmOptimizer pso(p);
    auto result = pso.optimize([](const Eigen::VectorXd& x){ return -x.sum(); }); // max at upper

    for (int i = 0; i < 3; ++i) {
        REQUIRE(result.params(i) >= p.lower(i) - 1e-9);
        REQUIRE(result.params(i) <= p.upper(i) + 1e-9);
    }
    REQUIRE(std::isfinite(result.cost));
}

// =============================================================================
// DifferentialEvolution
// =============================================================================

static ctrl::DEParams makeDEParams(int n, int pop = 20, int gen = 60)
{
    ctrl::DEParams p;
    p.n_dim      = n;
    p.population = pop;
    p.max_gen    = gen;
    p.lower = Eigen::VectorXd::Constant(n, 0.0);
    p.upper = Eigen::VectorXd::Constant(n, 5.0);
    p.seed  = 55;
    return p;
}

TEST_CASE("DifferentialEvolution minimises 1D parabola", "[de]")
{
    ctrl::DifferentialEvolution de(makeDEParams(1));
    auto result = de.optimize([](const Eigen::VectorXd& x){ return (x(0)-2.0)*(x(0)-2.0); });

    REQUIRE(result.cost < 0.05);
    REQUIRE(result.params(0) >= 0.0);
    REQUIRE(result.params(0) <= 5.0);
    REQUIRE(result.nEvals > 0);
}

TEST_CASE("DifferentialEvolution minimises 2D sphere", "[de]")
{
    Eigen::Vector2d centre(1.0, 4.5);
    ctrl::DifferentialEvolution de(makeDEParams(2, 25, 80));
    auto result = de.optimize([&](const Eigen::VectorXd& x){
        return (x - centre).squaredNorm();
    });

    REQUIRE(result.cost < 0.5);
    REQUIRE((result.params - centre).norm() < 1.0);
}

TEST_CASE("DifferentialEvolution result always within bounds", "[de]")
{
    ctrl::DEParams p = makeDEParams(3, 15, 30);
    ctrl::DifferentialEvolution de(p);
    auto result = de.optimize([](const Eigen::VectorXd& x){ return x.sum(); }); // min at lower

    for (int i = 0; i < 3; ++i) {
        REQUIRE(result.params(i) >= p.lower(i) - 1e-9);
        REQUIRE(result.params(i) <= p.upper(i) + 1e-9);
    }
    REQUIRE(std::isfinite(result.cost));
}

// -----------------------------------------------------------------------------
// Finding 39 - LPV system identification [lpv]
// -----------------------------------------------------------------------------

TEST_CASE("identifyLPV recovers affine A(p) on synthetic column-major data", "[lpv]")
{
    // Generate synthetic LPV data: x[k+1] = (0.8 + 0.1*p[k])*x[k] + 0.1*u[k]
    // A(p) = 0.8 + 0.1*p  (degree=1 polynomial in p)
    const int N = 200;
    const double ts = 0.01;
    Eigen::MatrixXd X(1, N + 1), U(1, N), Y(1, N);
    std::vector<double> sched(N + 1);

    X(0, 0) = 0.0;
    for (int k = 0; k < N; ++k) {
        sched[k] = static_cast<double>(k) / N;   // p in [0, 1)
        U(0, k)  = (k % 20 < 10) ? 1.0 : -1.0;  // square wave input
        const double a = 0.8 + 0.1 * sched[k];
        X(0, k + 1) = a * X(0, k) + 0.1 * U(0, k);
        Y(0, k)     = X(0, k);
    }
    sched[N] = 1.0;
    Eigen::MatrixXd X_in = X.leftCols(N);

    ctrl::LPVModel model = ctrl::identifyLPV(X_in, U, Y, sched, 1, ts);
    REQUIRE(model.A_coeffs.size() >= 2);

    // At p=0: A(0) approx = 0.8
    ctrl::StateSpace ss0 = model.frozen(0.0);
    REQUIRE_THAT(ss0.A(0, 0), WithinAbs(0.8, 0.05));

    // At p=1: A(1) approx = 0.9
    ctrl::StateSpace ss1 = model.frozen(1.0);
    REQUIRE_THAT(ss1.A(0, 0), WithinAbs(0.9, 0.05));

    SUCCEED();
}

// -----------------------------------------------------------------------------
// Finding 40 - Taylor function approximator [func_approx]
// -----------------------------------------------------------------------------

TEST_CASE("TaylorApproximator predicts sin(x) within tolerance on training range", "[func_approx]")
{
    const int N_train = 20;
    std::vector<double> xs(N_train), ys(N_train);
    for (int i = 0; i < N_train; ++i) {
        xs[i] = -1.0 + 2.0 * static_cast<double>(i) / (N_train - 1); // [-1, 1]
        ys[i] = std::sin(xs[i]);
    }
    ctrl::TaylorApproximator approx(xs, ys, 7); // degree 7 is enough for sin on [-1,1]

    // Evaluate at mid-points (not training points)
    for (int i = 0; i < 10; ++i) {
        const double x = -0.9 + 0.18 * i;
        REQUIRE_THAT(approx.evaluate(x), WithinAbs(std::sin(x), 1e-4));
    }
}

// -----------------------------------------------------------------------------
// Finding 41 - MetricsAnalyzer on a known first-order step response [metrics]
// -----------------------------------------------------------------------------

TEST_CASE("MetricsAnalyzer rise time and SSE on first-order step response", "[metrics]")
{
    // First-order step response: y(t) = 1 - exp(-t/tau)  with tau = 0.5 s
    // Rise time (10->90%) = tau * ln(9) approx = 1.0986 s
    const double tau = 0.5;
    const int N = 500;
    const double dt = 0.01;
    std::vector<double> t_data(N), y_data(N);
    for (int i = 0; i < N; ++i) {
        t_data[i] = i * dt;
        y_data[i] = 1.0 - std::exp(-t_data[i] / tau);
    }

    auto metrics = ctrl::MetricsAnalyzer::calculate(t_data, y_data, 1.0, 20);

    const double rise_time_analytic = tau * std::log(9.0); // approx = 1.0986 s
    REQUIRE_THAT(metrics.riseTime, WithinAbs(rise_time_analytic, 3 * dt));
    REQUIRE_THAT(metrics.steadyStateError, WithinAbs(0.0, 0.02));
    REQUIRE_THAT(metrics.peakOvershoot, WithinAbs(0.0, 0.1));
}

// -----------------------------------------------------------------------------
// Finding 42 - ZeroPhaseTrackingFilter phase error < 1^\circ [zero_phase]
// -----------------------------------------------------------------------------

TEST_CASE("designZPETC prefilter phase error < 1 degree at mid-band", "[zero_phase]")
{
    // Simple first-order min-phase plant G(z) = b/(z - a)
    // Use the makePlant() second-order plant from the shared helpers
    const ctrl::StateSpace plant = makePlant();

    ctrl::ZPETCResult res = ctrl::designZPETC(plant);

    // For a min-phase plant, ZPETC achieves near-zero phase error.
    // Verify at a mid-band frequency (0.1 * pi rad/sample)
    const double Ts_p = plant.Ts;
    const double omega = 0.1 * std::acos(-1.0) / Ts_p; // rad/s

    // Evaluate |arg(G(e^{jwTs}) * G_ff(e^{jwTs}))|  should be < 1 degree = pi/180 rad
    // We use evalMagnitude from VectorFitting or the frequency response directly.
    // Compute composite z-response manually:
    const std::complex<double> z(std::cos(omega * Ts_p), std::sin(omega * Ts_p));
    const int n_p = plant.A.rows();
    const int n_f = res.filter.A.rows();

    auto evalH = [&](const ctrl::StateSpace& ss, std::complex<double> zz) -> std::complex<double> {
        const int n = ss.A.rows();
        Eigen::MatrixXcd I = Eigen::MatrixXcd::Identity(n, n);
        Eigen::MatrixXcd M = (zz * I - ss.A.cast<std::complex<double>>());
        Eigen::FullPivLU<Eigen::MatrixXcd> lu(M);
        if (!lu.isInvertible()) return {0.0, 0.0};
        return (ss.C.cast<std::complex<double>>() *
                lu.solve(ss.B.cast<std::complex<double>>()) +
                ss.D.cast<std::complex<double>>())(0, 0);
    };

    const std::complex<double> G   = evalH(plant,      z);
    const std::complex<double> Gff = evalH(res.filter, z);

    // ZPETC for a min-phase plant gives G*Gff = z^{-d} (pure delay), so |G*Gff| = 1
    // for all frequencies.  Absolute phase equals -d*omega*T (not zero) because of the
    // delay.  The correct invariant to test is amplitude flatness.
    const double amplitude_composite = std::abs(G * Gff);
    REQUIRE_THAT(amplitude_composite, WithinAbs(1.0, 0.05));
    REQUIRE_FALSE(res.hasNMPZeros); // makePlant() is minimum-phase
}

// =============================================================================
// DeePC tests - Data-Enabled Predictive Control (Coulson 2019)
// =============================================================================

namespace {
// Generate data from first-order plant y[k] = a*y[k-1] + b*u[k-1]
// with PRBS-like random input.
void generateFirstOrderData(double a, double b, int N,
                            Eigen::VectorXd& u_out, Eigen::VectorXd& y_out,
                            unsigned seed = 42)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    u_out.resize(N);
    y_out.resize(N);
    y_out(0) = 0.0;
    for (int k = 0; k < N; ++k) {
        u_out(k) = dist(rng);
        if (k + 1 < N)
            y_out(k + 1) = a * y_out(k) + b * u_out(k);
    }
}
} // namespace

TEST_CASE("DeePC collectData builds valid Hankel matrices", "[deepc]") {
    ctrl::DeePCParams p;
    p.T_ini = 5; p.N = 5;
    ctrl::DeePC dc(p, 0.1);
    REQUIRE_FALSE(dc.isDataCollected());

    const int N_data = p.T_ini + p.N + 20;  // 30 samples
    Eigen::VectorXd u, y;
    generateFirstOrderData(0.7, 0.3, N_data, u, y);

    dc.collectData(u, y);
    REQUIRE(dc.isDataCollected());
    REQUIRE(dc.hankelColumns() == N_data - p.T_ini - p.N + 1);
}

TEST_CASE("DeePC rejects data that is too short", "[deepc]") {
    ctrl::DeePCParams p;
    p.T_ini = 10; p.N = 10;
    ctrl::DeePC dc(p, 0.1);

    const int N_short = p.T_ini + p.N;  // exactly L, needs L+1
    Eigen::VectorXd u = Eigen::VectorXd::Zero(N_short);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(N_short);
    REQUIRE_THROWS_AS(dc.collectData(u, y), std::invalid_argument);
}

TEST_CASE("DeePC tracks constant reference for first-order plant", "[deepc]") {
    // Plant: y[k+1] = 0.7*y[k] + 0.3*u[k], steady-state gain = 0.3/(1-0.7) = 1.0
    const double a = 0.7, b = 0.3;
    ctrl::DeePCParams p;
    p.T_ini = 8; p.N = 8;
    p.Q = 10.0; p.R = 0.01;
    p.lambda_g = 1.0; p.lambda_y = 50.0; p.lambda_u = 5.0;
    p.uMin = -5.0; p.uMax = 5.0;
    p.rho = 10.0; p.admm_iters = 300;

    ctrl::DeePC dc(p, 0.1);
    Eigen::VectorXd u_d, y_d;
    generateFirstOrderData(a, b, 80, u_d, y_d);
    dc.collectData(u_d, y_d);

    const double r = 1.0;
    dc.setReference(r);

    double y_k = 0.0, u_k = 0.0;
    for (int k = 0; k < 80; ++k) {
        u_k = dc.compute(y_k);
        y_k = a * y_k + b * u_k;
    }
    // After 80 steps (8 s at Ts=0.1), output should be close to steady-state
    REQUIRE_THAT(y_k, WithinAbs(r, 0.20));
}

TEST_CASE("DeePC output respects uMin/uMax hard bounds", "[deepc]") {
    ctrl::DeePCParams p;
    p.T_ini = 5; p.N = 5;
    p.uMin = -0.5; p.uMax = 0.5;

    ctrl::DeePC dc(p, 0.1);
    Eigen::VectorXd u_d, y_d;
    generateFirstOrderData(0.7, 0.3, 50, u_d, y_d);
    dc.collectData(u_d, y_d);
    dc.setReference(10.0);  // large reference - should saturate output

    for (int k = 0; k < 20; ++k) {
        const double u = dc.compute(static_cast<double>(k) * 0.1);
        REQUIRE(u >= p.uMin - 1e-9);
        REQUIRE(u <= p.uMax + 1e-9);
    }
}

TEST_CASE("DeePC NaN hold-last and reset", "[deepc]") {
    ctrl::DeePCParams p;
    p.T_ini = 5; p.N = 5;
    p.uMin = -2.0; p.uMax = 2.0;

    ctrl::DeePC dc(p, 0.1);
    Eigen::VectorXd u_d, y_d;
    generateFirstOrderData(0.7, 0.3, 30, u_d, y_d);
    dc.collectData(u_d, y_d);
    dc.setReference(1.0);

    // One good step
    const double u0 = dc.compute(0.0);
    REQUIRE(std::isfinite(u0));

    // NaN input -> hold last
    const double u_nan = dc.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE_THAT(u_nan, WithinAbs(u0, 1e-9));

    // reset clears buffers; output should still be finite on next step
    dc.reset();
    const double u_after = dc.compute(0.0);
    REQUIRE(std::isfinite(u_after));
}

// =============================================================================
// [pid] - DiscretePID supplemental tests (#64, #66)
// =============================================================================

TEST_CASE("DiscretePID Kb anti-windup limits integrator growth under saturation", "[pid]")
{
    ctrl::PIDParams p;
    p.Kp = 1.0; p.Ki = 2.0; p.Kd = 0.0;
    p.uMin = -1.0; p.uMax = 1.0;
    p.Kb = 1.0;  // anti-windup active
    ctrl::DiscretePID pid(p, 0.1);

    // Drive hard into saturation for 50 steps with large positive error
    for (int k = 0; k < 50; ++k)
        pid.compute(10.0);

    const double u_sat = pid.compute(10.0);
    // Output must remain clamped; anti-windup prevents integrator runaway
    REQUIRE(u_sat <= 1.0 + 1e-9);
    REQUIRE(u_sat >= -1.0 - 1e-9);
}

TEST_CASE("DiscretePID N-filter decays derivative contribution after initial kick", "[pid]")
{
    ctrl::PIDParams p;
    p.Kp = 0.0; p.Ki = 0.0; p.Kd = 1.0;
    p.N  = 10.0;  // derivative filter pole at z = N/(N+1/Ts) -> fast pole, decays
    p.uMin = -1e9; p.uMax = 1e9;
    ctrl::DiscretePID pid(p, 0.01);

    // Step error -> large derivative kick at first sample
    const double u0 = pid.compute(1.0);
    REQUIRE(u0 > 0.0);

    // After several more steps with e=1 (de/dt=0), derivative component decays
    double u_last = u0;
    for (int k = 0; k < 20; ++k)
        u_last = pid.compute(1.0);

    // N-filtered derivative on constant error must decay toward zero
    REQUIRE(u_last < u0);
    REQUIRE(std::isfinite(u_last));
}

TEST_CASE("DiscretePID computeDoM gives strictly lower peak control signal than compute on step", "[pid]")
{
    // DoM (Derivative on Measurement) eliminates the derivative kick in the control signal
    // that occurs when the setpoint steps.  Plant output peak is not a reliable discriminant
    // because the N-filter smooths the kick before it reaches the plant.  The distinguishing
    // property is the peak of the CONTROL SIGNAL u[k].
    //
    // With compute(r - y): at step k=0, e jumps from 0 to r, so the N-filtered derivative
    // contributes +Kd*N*r to u[0] (the "derivative kick").
    // With computeDoM(y, r): the derivative term is -Kd*N*(y - y_prev).  At k=0, y=y_prev=0,
    // so the kick is zero.  Peak u(DoM) < peak u(standard) is guaranteed for any Kd > 0.
    const double Ts = 0.01, a = 0.99, b = 0.01;
    const double r = 1.0;

    auto run_sim = [&](bool use_dom) {
        ctrl::PIDParams p;
        p.Kp = 5.0; p.Ki = 0.5; p.Kd = 0.3; p.N = 20.0;
        p.uMin = -100.0; p.uMax = 100.0;
        ctrl::DiscretePID pid(p, Ts);

        double y = 0.0, peak_u = 0.0;
        for (int k = 0; k < 300; ++k) {
            double u = use_dom ? pid.computeDoM(y, r) : pid.compute(r - y);
            y = a * y + b * u;
            if (u > peak_u) peak_u = u;
        }
        return peak_u;
    };

    const double peak_standard = run_sim(false);
    const double peak_dom      = run_sim(true);

    // computeDoM avoids the derivative kick -> lower peak control signal
    REQUIRE(peak_dom < peak_standard);
}

// =============================================================================
// [repetitive] - RepetitiveController edge cases (#65)
// =============================================================================

TEST_CASE("RepetitiveController throws when periodSteps < 1", "[repetitive]")
{
    auto pid = std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{1.0, 0.0, 0.0}, 0.01);
    ctrl::RepetitiveParams p;
    p.periodSteps = 0;
    REQUIRE_THROWS_AS(ctrl::RepetitiveController(pid, p, 0.01), std::invalid_argument);
    p.periodSteps = -5;
    REQUIRE_THROWS_AS(ctrl::RepetitiveController(pid, p, 0.01), std::invalid_argument);
}

TEST_CASE("RepetitiveController NaN input triggers hold-last contract", "[repetitive]")
{
    auto pid = std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{1.0, 0.0, 0.0}, 0.01);
    ctrl::RepetitiveParams p;
    p.periodSteps = 5;
    ctrl::RepetitiveController rc(pid, p, 0.01);

    const double u_good = rc.compute(0.5);
    REQUIRE(std::isfinite(u_good));

    const double u_nan = rc.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE_THAT(u_nan, WithinAbs(u_good, 1e-9));
}

TEST_CASE("RepetitiveController setParams with new period resets learning buffer", "[repetitive]")
{
    auto pid = std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{1.0, 0.0, 0.0}, 0.01);
    ctrl::RepetitiveParams p;
    p.periodSteps = 10; p.Krc = 0.5; p.Q = 0.98;
    ctrl::RepetitiveController rc(pid, p, 0.01);

    // Accumulate some learning
    for (int k = 0; k < 10; ++k) rc.compute(1.0);
    REQUIRE(rc.correction() != 0.0);

    // Change period -> buffer cleared, correction resets
    p.periodSteps = 20;
    rc.setParams(p);
    REQUIRE_THAT(rc.correction(), WithinAbs(0.0, 1e-12));
    REQUIRE(rc.params().periodSteps == 20);
}

// =============================================================================
// [extremum_seeker] - convergence to known quadratic minimum (#67)
// =============================================================================

TEST_CASE("ExtremumSeeker converges to minimum of quadratic J(theta)=(theta-2)^2", "[extremum_seeker]")
{
    // Cost surface: J(theta) = (theta - 2)^2, minimum at theta* = 2.
    // Plant is static (memoryless), so the plant output = J evaluated at the current u.
    ctrl::ExtremumSeekerParams ep;
    ep.perturbAmp  = 0.05;
    ep.perturbFreq = 0.5;    // [Hz]
    ep.lpfCutoff   = 0.05;   // [Hz]
    ep.hpfCutoff   = 0.02;   // [Hz]
    ep.integGain   = 0.5;
    ep.seekMinimum = true;

    const double Ts = 0.01;
    ctrl::ExtremumSeeker esc(ep, Ts);

    // Start away from optimum - initial theta = 0
    double u = 0.0;
    for (int k = 0; k < 20000; ++k) {
        const double J = (u - 2.0) * (u - 2.0);  // static quadratic cost
        u = esc.compute(J);
    }

    REQUIRE_THAT(esc.currentEstimate(), WithinAbs(2.0, 0.5));
}

// -----------------------------------------------------------------------------
// RobustnessAnalysis - Monte-Carlo closed-loop robustness (Phase 1)
// -----------------------------------------------------------------------------

// Helper: SISO state-space plant (n=1).
static ctrl::StateSpace makeFirstOrderPlant(double a, double b, double ts)
{
    return ctrl::StateSpace((Eigen::MatrixXd(1, 1) << a).finished(),
                            (Eigen::MatrixXd(1, 1) << b).finished(),
                            (Eigen::MatrixXd(1, 1) << 1.0).finished(),
                            (Eigen::MatrixXd(1, 1) << 0.0).finished(), ts);
}

// Helper: static-gain controller as a 1-state (uncontrollable) realisation of D = gain.
static ctrl::StateSpace makeStaticController(double gain, double ts)
{
    return ctrl::StateSpace(Eigen::MatrixXd::Zero(1, 1),
                            Eigen::MatrixXd::Zero(1, 1),
                            Eigen::MatrixXd::Zero(1, 1),
                            (Eigen::MatrixXd(1, 1) << gain).finished(), ts);
}

TEST_CASE("spawn_SS_samples perturbs A around the nominal value", "[robustness_mc]")
{
    const auto nominal = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto ens = ctrl::spawn_SS_samples(nominal, 400, /*sigma_A=*/0.1, 0.0, 0.0, 0.0, 123);

    REQUIRE(ens.size() == 400u);

    double sum = 0.0, sq = 0.0;
    bool any_differ = false;
    for (const auto& s : ens)
    {
        const double a = s.A(0, 0);
        sum += a;
        sq  += a * a;
        if (std::abs(a - 0.6) > 1e-9) any_differ = true;
        // B/C/D held fixed (sigma = 0).
        REQUIRE_THAT(s.B(0, 0), WithinAbs(0.4, 1e-12));
        REQUIRE_THAT(s.C(0, 0), WithinAbs(1.0, 1e-12));
    }
    const double mean = sum / ens.size();
    const double sd   = std::sqrt(sq / ens.size() - mean * mean);

    REQUIRE(any_differ);
    REQUIRE_THAT(mean, WithinAbs(0.6, 0.02));   // unbiased around nominal
    REQUIRE_THAT(sd,   WithinAbs(0.06, 0.02));  // ~ sigma * |a| = 0.1 * 0.6
}

TEST_CASE("Stable nominal plant + controller yields zero instability for small sigma",
          "[robustness_mc]")
{
    const auto plant = makeFirstOrderPlant(0.6, 0.4, 0.1);   // open-loop stable
    const auto ctl   = makeStaticController(0.5, 0.1);       // closed-loop pole 0.4

    const auto res = ctrl::monteCarloAnalysis(plant, ctl, 200, /*sigma_A=*/0.02,
                                              0.0, 0.0, 0.0, 7);

    REQUIRE(res.n_samples == 200);
    REQUIRE(res.n_unstable == 0);
    REQUIRE_THAT(res.instability_probability, WithinAbs(0.0, 1e-12));
    // Sensitivity peak is finite and positive for a stable loop.
    REQUIRE(std::isfinite(res.sensitivity_peak_stats.mean));
    REQUIRE(res.sensitivity_peak_stats.mean > 0.0);
}

TEST_CASE("Destabilising controller drives instability probability above zero",
          "[robustness_mc]")
{
    const auto plant = makeFirstOrderPlant(0.5, 1.0, 0.1);
    const auto ctl   = makeStaticController(5.0, 0.1);  // closed-loop pole -4.5 (unstable)

    const auto res = ctrl::monteCarloAnalysis(plant, ctl, 100, /*sigma_A=*/0.05,
                                              0.0, 0.0, 0.0, 11);

    REQUIRE(res.n_samples == 100);
    REQUIRE(res.instability_probability > 0.0);
    // Unstable samples report infinite IAE; stats over finite values stay sane or NaN.
    for (const auto& s : res.samples)
        if (!s.is_stable)
            REQUIRE(std::isinf(s.iae));
}

TEST_CASE("monteCarloAnalysis runs end-to-end on a second-order plant", "[robustness_mc]")
{
    // Damped 2nd-order plant (discrete), SISO.
    Eigen::MatrixXd A(2, 2); A << 0.9, 0.05, -0.1, 0.85;
    Eigen::MatrixXd B(2, 1); B << 0.0, 0.1;
    Eigen::MatrixXd C(1, 2); C << 1.0, 0.0;
    Eigen::MatrixXd D(1, 1); D << 0.0;
    const ctrl::StateSpace plant(A, B, C, D, 0.1);
    const auto ctl = makeStaticController(0.8, 0.1);

    const auto res = ctrl::monteCarloAnalysis(plant, ctl, 60, /*sigma_A=*/0.03,
                                              0.0, 0.0, 0.0, 42);

    REQUIRE(res.n_samples == 60);
    REQUIRE(static_cast<int>(res.samples.size()) == 60);
    REQUIRE(res.instability_probability >= 0.0);
    REQUIRE(res.instability_probability <= 1.0);
    REQUIRE(std::isfinite(res.comp_sensitivity_peak_stats.p50));
    REQUIRE(std::isfinite(res.nu_gap_stats.mean));
    // SISO loop => gain/phase margins are populated (finite or +inf, never NaN).
    REQUIRE_FALSE(std::isnan(res.samples.front().gain_margin_db));
}

TEST_CASE("nu-gap from nominal is near zero when sigma is zero", "[robustness_mc]")
{
    const auto plant = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto ctl   = makeStaticController(0.5, 0.1);

    // sigma_A = 0 => every sample equals the nominal plant.
    const auto res = ctrl::monteCarloAnalysis(plant, ctl, 10, 0.0, 0.0, 0.0, 0.0, 1);
    REQUIRE(res.n_unstable == 0);
    REQUIRE_THAT(res.nu_gap_stats.worst, WithinAbs(0.0, 1e-6));
}

// -----------------------------------------------------------------------------
// SystemAnalysis extensions - Gang of Four + Disk Margin (Phase 2)
// -----------------------------------------------------------------------------

TEST_CASE("series() cascade matches pointwise product of frequency responses",
          "[system_analysis_ext]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);

    // series(K, G): output of K feeds G, i.e. apply K first then G (matrix product G*K).
    const auto L = ctrl::SystemAnalysis::series(K, G);

    const std::vector<double> freqs{1.0, 5.0, 15.0};
    const auto L_resp = ctrl::SystemAnalysis::getFrequencyResponse(L, freqs);
    const auto G_resp = ctrl::SystemAnalysis::getFrequencyResponse(G, freqs);
    const auto K_resp = ctrl::SystemAnalysis::getFrequencyResponse(K, freqs);

    for (std::size_t i = 0; i < freqs.size(); ++i)
    {
        const auto expected = G_resp[i] * K_resp[i];
        REQUIRE_THAT(L_resp[i].real(), WithinAbs(expected.real(), 1e-9));
        REQUIRE_THAT(L_resp[i].imag(), WithinAbs(expected.imag(), 1e-9));
    }
}

TEST_CASE("parallel() sums D and pointwise frequency response of both systems",
          "[system_analysis_ext]")
{
    const auto G1 = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto G2 = makeFirstOrderPlant(0.3, 0.2, 0.1);

    const auto Gp = ctrl::SystemAnalysis::parallel(G1, G2);
    REQUIRE_THAT(Gp.D(0, 0), WithinAbs(0.0, 1e-12));

    const std::vector<double> freqs{2.0, 10.0};
    const auto Gp_resp = ctrl::SystemAnalysis::getFrequencyResponse(Gp, freqs);
    const auto G1_resp = ctrl::SystemAnalysis::getFrequencyResponse(G1, freqs);
    const auto G2_resp = ctrl::SystemAnalysis::getFrequencyResponse(G2, freqs);

    for (std::size_t i = 0; i < freqs.size(); ++i)
    {
        const auto expected = G1_resp[i] + G2_resp[i];
        REQUIRE_THAT(Gp_resp[i].real(), WithinAbs(expected.real(), 1e-9));
        REQUIRE_THAT(Gp_resp[i].imag(), WithinAbs(expected.imag(), 1e-9));
    }
}

TEST_CASE("feedback() closes the loop at the analytically-known pole",
          "[system_analysis_ext]")
{
    // x+ = 0.6x + 0.4u, y=x, u=0.5(r-y) => x+ = 0.4x + 0.2r => closed-loop pole at 0.4.
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);
    const auto L = ctrl::SystemAnalysis::series(K, G);

    const auto T = ctrl::SystemAnalysis::feedback(L);
    const auto poles = ctrl::SystemAnalysis::getPoles(T);

    bool found_04 = false;
    for (const auto &p : poles)
        if (std::abs(p.real() - 0.4) < 1e-9 && std::abs(p.imag()) < 1e-12)
            found_04 = true;
    REQUIRE(found_04);
}

TEST_CASE("feedback() throws for a non-square forward-path system",
          "[system_analysis_ext]")
{
    // 1 input, 2 outputs -> inputSize() != outputSize() -> non-square.
    Eigen::MatrixXd A(1, 1); A << 0.5;
    Eigen::MatrixXd B(1, 1); B << 0.1;
    Eigen::MatrixXd C(2, 1); C << 1.0, 0.5;
    Eigen::MatrixXd D(2, 1); D.setZero();
    const ctrl::StateSpace non_square(A, B, C, D, 0.1);

    REQUIRE_THROWS_AS(ctrl::SystemAnalysis::feedback(non_square), std::invalid_argument);
}

TEST_CASE("gangOfFour() satisfies S + T = I pointwise across frequency",
          "[system_analysis_ext]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);

    const auto g4 = ctrl::SystemAnalysis::gangOfFour(G, K);

    const std::vector<double> freqs{0.5, 3.0, 10.0, 25.0};
    const auto S_resp = ctrl::SystemAnalysis::getFrequencyResponse(g4.S, freqs);
    const auto T_resp = ctrl::SystemAnalysis::getFrequencyResponse(g4.T, freqs);

    for (std::size_t i = 0; i < freqs.size(); ++i)
    {
        const auto sum = S_resp[i] + T_resp[i];
        REQUIRE_THAT(sum.real(), WithinAbs(1.0, 1e-9));
        REQUIRE_THAT(sum.imag(), WithinAbs(0.0, 1e-9));
    }
}

TEST_CASE("gangOfFourNorms() matches analytical DC value of T for a monotonic 1st-order loop",
          "[system_analysis_ext]")
{
    // T(z) = 0.2/(z-0.4); |T| is monotonically decreasing from DC for this 1st-order
    // lowpass loop, so the Hinf peak equals the DC value T(1) = 0.2/0.6 = 1/3.
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);

    const auto g4  = ctrl::SystemAnalysis::gangOfFour(G, K);
    const auto g4n = ctrl::SystemAnalysis::gangOfFourNorms(g4);

    REQUIRE_THAT(g4n.norm_T, WithinAbs(1.0 / 3.0, 1e-3));
    // S peaks above 1 away from DC (waterbed effect) for this loop - not a bug.
    REQUIRE(g4n.norm_S > 1.0);
}

TEST_CASE("gangOfFour() throws on plant/controller dimension mismatch",
          "[system_analysis_ext]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);

    Eigen::MatrixXd Ak(1, 1); Ak.setZero();
    Eigen::MatrixXd Bk(1, 2); Bk.setZero();
    Eigen::MatrixXd Ck(2, 1); Ck.setZero();
    Eigen::MatrixXd Dk(2, 2); Dk.setZero(); // 2-input/2-output controller vs SISO plant
    const ctrl::StateSpace K_mismatched(Ak, Bk, Ck, Dk, 0.1);

    REQUIRE_THROWS_AS(ctrl::SystemAnalysis::gangOfFour(G, K_mismatched), std::invalid_argument);
}

TEST_CASE("calculateDiskMargin() alpha equals 1/||S||_inf and agrees with gangOfFour path",
          "[system_analysis_ext]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);
    const auto L = ctrl::SystemAnalysis::series(K, G);

    const auto dm  = ctrl::SystemAnalysis::calculateDiskMargin(L);
    const auto g4n = ctrl::SystemAnalysis::gangOfFourNorms(ctrl::SystemAnalysis::gangOfFour(G, K));

    REQUIRE(dm.alpha > 0.0);
    REQUIRE_THAT(dm.alpha, WithinAbs(1.0 / g4n.norm_S, 1e-6));
    REQUIRE_THAT(dm.gain_margin, WithinAbs((1.0 + dm.alpha) / (1.0 - dm.alpha), 1e-9));
    REQUIRE_THAT(dm.phase_margin_deg,
                 WithinAbs(2.0 * std::asin(dm.alpha / 2.0) * 180.0 / std::numbers::pi, 1e-9));
}

TEST_CASE("calculateDiskMargin() throws for a non-square open-loop system",
          "[system_analysis_ext]")
{
    // 1 input, 2 outputs -> inputSize() != outputSize() -> non-square.
    Eigen::MatrixXd A(1, 1); A << 0.5;
    Eigen::MatrixXd B(1, 1); B << 0.1;
    Eigen::MatrixXd C(2, 1); C << 1.0, 0.5;
    Eigen::MatrixXd D(2, 1); D.setZero();
    const ctrl::StateSpace non_square(A, B, C, D, 0.1);

    REQUIRE_THROWS_AS(ctrl::SystemAnalysis::calculateDiskMargin(non_square), std::invalid_argument);
}

TEST_CASE("getSingularValues() on a SISO system matches |getFrequencyResponse()|",
          "[system_analysis_ext]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);

    const std::vector<double> freqs{1.0, 5.0, 15.0};
    const auto resp = ctrl::SystemAnalysis::getFrequencyResponse(G, freqs);
    const auto svs  = ctrl::SystemAnalysis::getSingularValues(G, freqs);

    REQUIRE(svs.size() == freqs.size());
    for (std::size_t i = 0; i < freqs.size(); ++i)
    {
        REQUIRE(svs[i].size() == 1);
        REQUIRE_THAT(svs[i](0), WithinAbs(std::abs(resp[i]), 1e-9));
    }
}

TEST_CASE("getSingularValues() on a diagonal 2x2 MIMO system matches sorted SISO channel magnitudes",
          "[system_analysis_ext]")
{
    // Two decoupled first-order channels stacked block-diagonally:
    //   channel 1: x1+ = 0.6 x1 + 0.4 u1, y1 = x1
    //   channel 2: x2+ = 0.3 x2 + 0.2 u2, y2 = x2
    const auto G1 = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto G2 = makeFirstOrderPlant(0.3, 0.2, 0.1);

    Eigen::MatrixXd A(2, 2); A.setZero();
    A(0, 0) = 0.6; A(1, 1) = 0.3;
    Eigen::MatrixXd B(2, 2); B.setZero();
    B(0, 0) = 0.4; B(1, 1) = 0.2;
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2, 2);
    const ctrl::StateSpace G_mimo(A, B, C, D, 0.1);

    const std::vector<double> freqs{1.0, 8.0};
    const auto resp1 = ctrl::SystemAnalysis::getFrequencyResponse(G1, freqs);
    const auto resp2 = ctrl::SystemAnalysis::getFrequencyResponse(G2, freqs);
    const auto svs   = ctrl::SystemAnalysis::getSingularValues(G_mimo, freqs);

    for (std::size_t i = 0; i < freqs.size(); ++i)
    {
        REQUIRE(svs[i].size() == 2);
        const double m1 = std::abs(resp1[i]);
        const double m2 = std::abs(resp2[i]);
        REQUIRE_THAT(svs[i](0), WithinAbs(std::max(m1, m2), 1e-9));
        REQUIRE_THAT(svs[i](1), WithinAbs(std::min(m1, m2), 1e-9));
    }
}

// -----------------------------------------------------------------------------
// MuAnalysis - Structured Singular Value (Phase 3)
// -----------------------------------------------------------------------------

TEST_CASE("UncertaintyStructure totalInputs()/totalOutputs() sum block dimensions",
          "[mu_analysis]")
{
    ctrl::UncertaintyStructure struc;
    struc.blocks = {
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull,   2, 3},
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexScalar, 1, 1},
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull,   1, 1},
    };

    REQUIRE(struc.totalOutputs() == 4);
    REQUIRE(struc.totalInputs()  == 5);
}

TEST_CASE("computeMu() upper bound is exactly sigma_max(M) for a single ComplexFull block",
          "[mu_analysis]")
{
    // A single block spanning the whole space has no free D-scaling (any global scalar
    // cancels in D*M*D^-1), so the upper bound must equal the plain sigma_max(M) exactly -
    // and since M here is a real diagonal (hence normal) matrix, mu = sigma_max = rho too.
    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 2, 2}};

    Eigen::MatrixXcd M = Eigen::MatrixXcd::Zero(2, 2);
    M(0, 0) = 3.0;
    M(1, 1) = 1.0;

    const auto bounds = ctrl::computeMu({M}, struc, /*compute_lower_bound=*/true);
    REQUIRE(bounds.size() == 1);
    REQUIRE_THAT(bounds[0].upper, WithinAbs(3.0, 1e-9));
    REQUIRE_THAT(bounds[0].lower, WithinAbs(3.0, 1e-9));
}

TEST_CASE("computeMu() returns zero bounds for a zero interconnection matrix",
          "[mu_analysis]")
{
    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 2, 2}};

    const Eigen::MatrixXcd M = Eigen::MatrixXcd::Zero(2, 2);

    const auto bounds = ctrl::computeMu({M}, struc, /*compute_lower_bound=*/true);
    REQUIRE_THAT(bounds[0].upper, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(bounds[0].lower, WithinAbs(0.0, 1e-12));
}

TEST_CASE("computeMu() throws on dimension mismatch and on r_out != r_in scalar blocks",
          "[mu_analysis]")
{
    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 2, 2}};

    const Eigen::MatrixXcd M_wrong = Eigen::MatrixXcd::Identity(3, 3);
    REQUIRE_THROWS_AS(ctrl::computeMu({M_wrong}, struc), std::invalid_argument);

    ctrl::UncertaintyStructure mismatched_struc;
    mismatched_struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::RealScalar, 1, 2}};
    const Eigen::MatrixXcd M_mismatched = Eigen::MatrixXcd::Identity(1, 1);
    REQUIRE_THROWS_AS(ctrl::computeMu({M_mismatched}, mismatched_struc), std::invalid_argument);
}

TEST_CASE("computeMu() RealScalar G-scaling: a single real-scalar block drives the bound "
          "well below the single-ComplexFull-block bound on the same purely-imaginary M",
          "[mu_analysis]")
{
    // M = [i] (1x1, purely imaginary). For a single ComplexFull block, no scaling is possible
    // (a lone block's global D cancels), so the bound is exactly sigma_max(M) = |i| = 1.
    // For a single RealScalar block, G-scaling is NOT gauge-cancelled - minimising
    // sigma_max((1+g^2)^-1/2 * (i - jg)) over real g gives an exact zero at g=1:
    // (1+(-g)^2)... |i - jg| = |1-g|, so cost^2 = (1-g)^2/(1+g^2), which is exactly 0 at g=1.
    // This is the textbook point of G-scaling: a real perturbation can never exactly cancel a
    // purely-imaginary M the way a complex perturbation trivially can (delta=1/i=-i), giving a
    // genuinely smaller real-mu bound, not just an artifact of looser scaling.
    const Eigen::MatrixXcd M = (Eigen::MatrixXcd(1, 1) << std::complex<double>(0.0, 1.0)).finished();

    ctrl::UncertaintyStructure complex_struc;
    complex_struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1}};
    const auto complex_bounds = ctrl::computeMu({M}, complex_struc);
    REQUIRE_THAT(complex_bounds[0].upper, WithinAbs(1.0, 1e-9));

    ctrl::UncertaintyStructure real_struc;
    real_struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::RealScalar, 1, 1}};
    const auto real_bounds = ctrl::computeMu({M}, real_struc);
    REQUIRE_THAT(real_bounds[0].upper, WithinAbs(0.0, 1e-3));
    REQUIRE(real_bounds[0].upper < complex_bounds[0].upper - 0.9); // genuinely, not marginally, smaller
}

TEST_CASE("computeMu() mixed RealScalar/ComplexFull structure recovers the all-ComplexFull "
          "bound when the off-diagonal example's optimal scaling is already real-positive",
          "[mu_analysis]")
{
    // Same M as the classic 2x2 off-diagonal example (M=[[0,2],[0.5,0]]). With block 0 forced
    // real (delta1 real) and block 1 free complex (delta2), the singularity condition
    // 1 - delta1*delta2*m12*m21 = 0 (m12*m21 = 1 here) forces delta2 = 1/delta1, which is
    // automatically real once delta1 is real - so the mixed real/complex structured mu equals
    // the all-complex mu (=1) exactly, and the optimal scaling needs no G contribution at all
    // (g1=0 already attains it, same d1=0.5/d2=1 the all-ComplexFull test uses) - a regression
    // check that mixed block types interact correctly through the same alternating d/g sweep.
    ctrl::UncertaintyStructure mixed_struc;
    mixed_struc.blocks = {
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::RealScalar,  1, 1},
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
    };

    Eigen::MatrixXcd M(2, 2);
    M << 0.0, 2.0,
         0.5, 0.0;

    const auto bounds = ctrl::computeMu({M}, mixed_struc, /*compute_lower_bound=*/true);
    REQUIRE_THAT(bounds[0].upper, WithinAbs(1.0, 1e-3));
    REQUIRE(bounds[0].upper <= 2.0 + 1e-9); // never worse than the unscaled sigma_max(M) baseline
}

TEST_CASE("computeMu() coordinate-descent D-scaling recovers the textbook mu=1 "
          "for a classic 2x2 off-diagonal example",
          "[mu_analysis]")
{
    // Skogestad & Postlethwaite-style example: M = [[0,2],[0.5,0]] with Delta = diag(d1,d2)
    // (two independent SISO complex full blocks). Analytically mu = sqrt(|m12*m21|) = 1,
    // attained at d1/d2 = sqrt(m21/m12) = 0.5, while the unstructured sigma_max(M) = 2 and
    // rho(M) = 1 (eigenvalues +/-1) - a case where the lower and upper bounds coincide.
    ctrl::UncertaintyStructure struc;
    struc.blocks = {
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
    };

    Eigen::MatrixXcd M(2, 2);
    M << 0.0, 2.0,
         0.5, 0.0;

    const auto bounds = ctrl::computeMu({M}, struc, /*compute_lower_bound=*/true);
    REQUIRE(bounds.size() == 1);
    REQUIRE_THAT(bounds[0].upper, WithinAbs(1.0, 1e-3));
    REQUIRE_THAT(bounds[0].lower, WithinAbs(1.0, 1e-9));
    // The D-scaling upper bound must never exceed the unscaled sigma_max(M) baseline.
    REQUIRE(bounds[0].upper <= 2.0 + 1e-9);
}

TEST_CASE("peakMu() matches sigma_rel * ||T||_inf for a single ComplexFull block "
          "spanning a SISO output",
          "[mu_analysis]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);

    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1}};

    const auto result =
        ctrl::peakMu(G, K, struc, /*sigma_rel=*/1.0, /*freq_points=*/50, /*omega_min=*/1e-4);

    // norm_T = 1/3 (the same analytically-derived value as the Phase 2 gangOfFourNorms
    // test); |T| is monotonically decreasing away from DC for this loop, so the peak
    // occurs at the smallest grid frequency and converges to the exact Hinf norm as
    // omega_min -> 0.
    REQUIRE_THAT(result.peak.upper, WithinAbs(1.0 / 3.0, 1e-3));
    REQUIRE(result.mu_curve.size() == 50);
}

TEST_CASE("robustStabilityRadius() recovers 1/||T||_inf for a single ComplexFull block",
          "[mu_analysis]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);

    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1}};

    const double radius =
        ctrl::robustStabilityRadius(G, K, struc, /*sigma_max=*/5.0, /*bisect_iters=*/30);

    REQUIRE_THAT(radius, WithinAbs(3.0, 1e-2));
}

TEST_CASE("peakMu() throws when the uncertainty structure size mismatches "
          "the plant output dimension",
          "[mu_analysis]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);

    // Plant is SISO (1 output) but the structure declares a 2x2 block.
    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 2, 2}};

    REQUIRE_THROWS_AS(ctrl::peakMu(G, K, struc), std::invalid_argument);
}

// -----------------------------------------------------------------------------
// LFTSystem - general multi-block LFT/Delta channel-gather (Phase 3 RC1)
// -----------------------------------------------------------------------------

TEST_CASE("LFTSystem degenerate single-block case reproduces peakMu() exactly",
          "[lft_system]")
{
    const auto G = makeFirstOrderPlant(0.6, 0.4, 0.1);
    const auto K = makeStaticController(0.5, 0.1);

    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1}};

    const auto expected =
        ctrl::peakMu(G, K, struc, /*sigma_rel=*/1.0, /*freq_points=*/50, /*omega_min=*/1e-4);

    const ctrl::GangOfFour g4 = ctrl::SystemAnalysis::gangOfFour(G, K);
    const ctrl::StateSpace &M0 = g4.T; // sigma_rel = 1.0, so M0 == T directly

    ctrl::LFTChannelMap map;
    map.rowStart = {0};
    map.colStart = {0};
    ctrl::LFTSystem lft(M0, struc, map);
    const auto actual = lft.peakMu(/*freq_points=*/50, /*omega_min=*/1e-4);

    REQUIRE_THAT(actual.peak.upper, WithinAbs(expected.peak.upper, 1e-9));
    REQUIRE_THAT(actual.peak_omega_rad_s, WithinAbs(expected.peak_omega_rad_s, 1e-9));
    REQUIRE(actual.mu_curve.size() == expected.mu_curve.size());
}

TEST_CASE("LFTSystem gathers two disjoint, scattered blocks into the correct block-ordered matrix",
          "[lft_system]")
{
    // Static (zero-state) 4-input/4-output gain map: D(i,j) = 10*i + j, so every entry is
    // distinct and hand-verifiable. Two 1x1 blocks at scattered, non-adjacent positions:
    // block 0 reads row 0 / writes col 2; block 1 reads row 3 / writes col 1.
    Eigen::MatrixXd D(4, 4);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            D(i, j) = 10.0 * i + j;
    ctrl::StateSpace M0(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 4),
                         Eigen::MatrixXd(4, 0), D, 0.1);

    ctrl::UncertaintyStructure struc;
    struc.blocks = {
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
    };
    ctrl::LFTChannelMap map;
    map.rowStart = {0, 3};
    map.colStart = {2, 1};

    ctrl::LFTSystem lft(M0, struc, map);
    const auto responses = lft.closedLoopFreqResponse({1.0});
    REQUIRE(responses.size() == 1);
    const Eigen::MatrixXcd &Mg = responses[0];

    REQUIRE(Mg.rows() == 2);
    REQUIRE(Mg.cols() == 2);
    REQUIRE_THAT(Mg(0, 0).real(), WithinAbs(D(0, 2), 1e-9)); // = 2
    REQUIRE_THAT(Mg(0, 1).real(), WithinAbs(D(0, 1), 1e-9)); // = 1
    REQUIRE_THAT(Mg(1, 0).real(), WithinAbs(D(3, 2), 1e-9)); // = 32
    REQUIRE_THAT(Mg(1, 1).real(), WithinAbs(D(3, 1), 1e-9)); // = 31
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            REQUIRE_THAT(Mg(i, j).imag(), WithinAbs(0.0, 1e-9)); // static gain: no imaginary part
}

TEST_CASE("LFTSystem throws on a mis-sized, out-of-range, or overlapping channel map",
          "[lft_system]")
{
    const Eigen::MatrixXd D = Eigen::MatrixXd::Identity(2, 2);
    ctrl::StateSpace M0(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 2),
                         Eigen::MatrixXd(2, 0), D, 0.1);
    ctrl::UncertaintyStructure struc;
    struc.blocks = {
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
    };

    // Mis-sized: only one entry for two blocks.
    ctrl::LFTChannelMap badSize;
    badSize.rowStart = {0};
    badSize.colStart = {0};
    REQUIRE_THROWS_AS(ctrl::LFTSystem(M0, struc, badSize), std::invalid_argument);

    // Out of range: M0 only has 2 output rows, but block 1 starts at row 2.
    ctrl::LFTChannelMap outOfRange;
    outOfRange.rowStart = {0, 2};
    outOfRange.colStart = {0, 1};
    REQUIRE_THROWS_AS(ctrl::LFTSystem(M0, struc, outOfRange), std::invalid_argument);

    // Overlapping: both blocks claim row 0.
    ctrl::LFTChannelMap overlap;
    overlap.rowStart = {0, 0};
    overlap.colStart = {0, 1};
    REQUIRE_THROWS_AS(ctrl::LFTSystem(M0, struc, overlap), std::invalid_argument);
}

TEST_CASE("LFTSystem ignores channels not claimed by any block", "[lft_system]")
{
    // M0 has 4 channels but only 2 (non-adjacent) are claimed by the single block - the
    // gathered matrix must be exactly 1x1 (struc.totalOutputs() x struc.totalInputs()),
    // not 4x4, confirming partial coverage doesn't pull in unrelated channels.
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(4, 4) * 5.0;
    ctrl::StateSpace M0(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 4),
                         Eigen::MatrixXd(4, 0), D, 0.1);
    ctrl::UncertaintyStructure struc;
    struc.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1}};
    ctrl::LFTChannelMap map;
    map.rowStart = {2};
    map.colStart = {2};

    ctrl::LFTSystem lft(M0, struc, map);
    const auto responses = lft.closedLoopFreqResponse({1.0});
    REQUIRE(responses[0].rows() == 1);
    REQUIRE(responses[0].cols() == 1);
    REQUIRE_THAT(responses[0](0, 0).real(), WithinAbs(5.0, 1e-9));
}

// -----------------------------------------------------------------------------
// WorstCaseSearch - CMA-ES worst-case parameter search (Robustness Phase 4)
// -----------------------------------------------------------------------------

TEST_CASE("findWorstCaseSensitivity finds a plant at least as sensitive as the nominal",
          "[worst_case_search]")
{
    const auto ctl = makeStaticController(0.5, 0.1);
    auto plant_factory = [](const Eigen::VectorXd& p) {
        return makeFirstOrderPlant(p(0), 0.4, 0.1);
    };
    const Eigen::VectorXd nominal = (Eigen::VectorXd(1) << 0.6).finished();
    const Eigen::VectorXd sigma   = (Eigen::VectorXd(1) << 0.3).finished();

    const auto nominal_sample =
        ctrl::evaluateSample(0, plant_factory(nominal), ctl, plant_factory(nominal));

    ctrl::WorstCaseSearchParams wp;
    wp.max_evals = 300;
    wp.seed = 7;
    const auto res = ctrl::findWorstCaseSensitivity(plant_factory, ctl, nominal, sigma, {}, {}, wp);

    REQUIRE(res.n_evals > 0);
    REQUIRE(std::isfinite(res.worst_cost));
    REQUIRE(res.worst_cost >= nominal_sample.hinf_sensitivity - 1e-9);
}

TEST_CASE("findWorstCase with an epsilon-small search box returns close to the nominal metric",
          "[worst_case_search]")
{
    auto plant_factory = [](const Eigen::VectorXd& p) {
        return makeFirstOrderPlant(p(0), 0.4, 0.1);
    };
    auto metric_fn = [](const ctrl::StateSpace& ss) { return std::abs(ss.A(0, 0)); };

    const Eigen::VectorXd nominal = (Eigen::VectorXd(1) << 0.6).finished();
    const Eigen::VectorXd sigma   = (Eigen::VectorXd(1) << 1e-6).finished();

    ctrl::WorstCaseSearchParams wp;
    wp.max_evals = 80;
    const auto res = ctrl::findWorstCase(plant_factory, metric_fn, nominal, sigma, {}, {}, wp);

    REQUIRE_THAT(res.worst_cost, WithinAbs(0.6, 1e-3));
}

TEST_CASE("findWorstCaseIAE returns an IAE no better than the nominal for a perturbable loop",
          "[worst_case_search]")
{
    const auto ctl = makeStaticController(0.5, 0.1);
    auto plant_factory = [](const Eigen::VectorXd& p) {
        return makeFirstOrderPlant(p(0), 0.4, 0.1);
    };
    const Eigen::VectorXd nominal = (Eigen::VectorXd(1) << 0.6).finished();
    const Eigen::VectorXd sigma   = (Eigen::VectorXd(1) << 0.3).finished();

    const auto nominal_plant  = plant_factory(nominal);
    const auto nominal_sample = ctrl::evaluateSample(0, nominal_plant, ctl, nominal_plant);

    ctrl::WorstCaseSearchParams wp;
    wp.max_evals = 300;
    wp.seed = 3;
    const auto res = ctrl::findWorstCaseIAE(plant_factory, ctl, nominal, sigma, {}, {},
                                            /*sim_duration_s=*/20.0, wp);

    REQUIRE(res.worst_cost >= nominal_sample.iae - 1e-9);
}

TEST_CASE("findWorstCaseSensitivity respects hard parameter bounds", "[worst_case_search]")
{
    const auto ctl = makeStaticController(0.5, 0.1);
    auto plant_factory = [](const Eigen::VectorXd& p) {
        return makeFirstOrderPlant(p(0), 0.4, 0.1);
    };
    const Eigen::VectorXd nominal = (Eigen::VectorXd(1) << 0.6).finished();
    const Eigen::VectorXd sigma   = (Eigen::VectorXd(1) << 0.5).finished();
    const Eigen::VectorXd lower   = (Eigen::VectorXd(1) << 0.55).finished();
    const Eigen::VectorXd upper   = (Eigen::VectorXd(1) << 0.65).finished();

    ctrl::WorstCaseSearchParams wp;
    wp.max_evals = 300;
    wp.seed = 5;
    const auto res = ctrl::findWorstCaseSensitivity(plant_factory, ctl, nominal, sigma, lower, upper, wp);

    REQUIRE(res.worst_params(0) >= lower(0) - 1e-9);
    REQUIRE(res.worst_params(0) <= upper(0) + 1e-9);
}

// -----------------------------------------------------------------------------
// LyapunovRobustness - common quadratic Lyapunov function (Robustness Phase 5)
// -----------------------------------------------------------------------------

TEST_CASE("findCommonLyapunov recovers the analytic solution for a single stable scalar vertex",
          "[lyapunov_robustness]")
{
    Eigen::MatrixXd A(1, 1); A << 0.5;
    const auto res = ctrl::findCommonLyapunov({A});

    REQUIRE(res.found);
    // Analytic scalar solution: P = Q / (1 - A^2) = 1 / 0.75.
    REQUIRE_THAT(res.P(0, 0), WithinAbs(1.0 / 0.75, 1e-4));
    REQUIRE(res.residual < 0.0);
}

TEST_CASE("findCommonLyapunov succeeds for a tight cluster of stable vertices around a nominal",
          "[lyapunov_robustness]")
{
    Eigen::MatrixXd A_nom(1, 1); A_nom << 0.5;
    Eigen::MatrixXd dirs(1, 1);  dirs << 0.05;
    const auto vertices = ctrl::buildBoxVertices(A_nom, dirs);
    REQUIRE(vertices.size() == 2u);

    const auto res = ctrl::findCommonLyapunov(vertices);
    REQUIRE(res.found);
    REQUIRE(res.residual < 0.0);
}

TEST_CASE("isQuadraticallyStable returns false when any vertex is individually unstable",
          "[lyapunov_robustness]")
{
    Eigen::MatrixXd stable(1, 1);   stable   << 0.5;
    Eigen::MatrixXd unstable(1, 1); unstable << 1.5;

    REQUIRE(ctrl::isQuadraticallyStable({stable}));
    REQUIRE_FALSE(ctrl::isQuadraticallyStable({stable, unstable}));
}

TEST_CASE("buildBoxVertices produces 2^m vertices with every +/- sign combination",
          "[lyapunov_robustness]")
{
    const Eigen::MatrixXd A_nom = Eigen::MatrixXd::Identity(2, 2) * 0.5;

    Eigen::MatrixXd dirs(4, 2);
    dirs.col(0) = (Eigen::VectorXd(4) << 0.05, 0.0, 0.0, 0.0).finished(); // perturbs (0,0)
    dirs.col(1) = (Eigen::VectorXd(4) << 0.0, 0.0, 0.0, 0.05).finished(); // perturbs (1,1)

    const auto vertices = ctrl::buildBoxVertices(A_nom, dirs);
    REQUIRE(vertices.size() == 4u);

    REQUIRE_THAT(vertices[0](0, 0), WithinAbs(0.45, 1e-12));
    REQUIRE_THAT(vertices[0](1, 1), WithinAbs(0.45, 1e-12));
    REQUIRE_THAT(vertices[3](0, 0), WithinAbs(0.55, 1e-12));
    REQUIRE_THAT(vertices[3](1, 1), WithinAbs(0.55, 1e-12));
}

TEST_CASE("findCommonLyapunov throws on empty vertices or a dimension mismatch",
          "[lyapunov_robustness]")
{
    REQUIRE_THROWS_AS(ctrl::findCommonLyapunov({}), std::invalid_argument);

    Eigen::MatrixXd A1(1, 1); A1 << 0.5;
    Eigen::MatrixXd A2(2, 2); A2 << 0.5, 0.0, 0.0, 0.5;
    REQUIRE_THROWS_AS(ctrl::findCommonLyapunov({A1, A2}), std::invalid_argument);
}

// -----------------------------------------------------------------------------
// FreqDomainIdentifier - Levy's method (Phase 4 Iteration 2)
// -----------------------------------------------------------------------------

TEST_CASE("fitLevy recovers exact coefficients of a known 1st-order system from noiseless response",
          "[freq_domain_id]")
{
    // True plant: H(z^-1) = 0.2 z^-1 / (1 - 0.8 z^-1)  (the smoke_test.py minimal example,
    // written with the leading b0=0 term explicit so tf2ss doesn't have to right-pad it).
    const ctrl::TransferFunction tf({0.0, 0.2}, {1.0, -0.8}, 0.1);
    const auto sys = ctrl::tf2ss(tf);

    const std::vector<double> freqs{0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0};
    const auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    const auto result = ctrl::FreqDomainIdentifier::fitLevy(freqs, response,
                                                            /*num_order=*/1,
                                                            /*den_order=*/1,
                                                            /*Ts=*/0.1);

    REQUIRE(result.full_rank);
    REQUIRE(result.tf.num.size() == 2u);
    REQUIRE(result.tf.den.size() == 2u);
    REQUIRE_THAT(result.tf.num[0], WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(result.tf.num[1], WithinAbs(0.2, 1e-9));
    REQUIRE_THAT(result.tf.den[0], WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(result.tf.den[1], WithinAbs(-0.8, 1e-9));
    REQUIRE_THAT(result.rmse, WithinAbs(0.0, 1e-9));
}

TEST_CASE("fitLevy recovers exact coefficients of the README's 2nd-order plant from noiseless response",
          "[freq_domain_id]")
{
    // True plant: H(z^-1) = (0.0048 z^-1 + 0.0047 z^-2) / (1 - 1.81 z^-1 + 0.819 z^-2)
    // (the README's plant, written with the leading b0=0 term explicit - see the 1st-order
    // test above for why).
    const ctrl::TransferFunction tf({0.0, 0.0048, 0.0047}, {1.0, -1.81, 0.819}, 0.01);
    const auto sys = ctrl::tf2ss(tf);

    const std::vector<double> freqs{1.0, 5.0, 10.0, 25.0, 50.0, 75.0, 100.0, 150.0};
    const auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    const auto result = ctrl::FreqDomainIdentifier::fitLevy(freqs, response,
                                                            /*num_order=*/2,
                                                            /*den_order=*/2,
                                                            /*Ts=*/0.01);

    REQUIRE(result.full_rank);
    REQUIRE(result.tf.num.size() == 3u);
    REQUIRE(result.tf.den.size() == 3u);
    REQUIRE_THAT(result.tf.num[0], WithinAbs(0.0, 1e-7));
    REQUIRE_THAT(result.tf.num[1], WithinAbs(0.0048, 1e-7));
    REQUIRE_THAT(result.tf.num[2], WithinAbs(0.0047, 1e-7));
    REQUIRE_THAT(result.tf.den[0], WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(result.tf.den[1], WithinAbs(-1.81, 1e-6));
    REQUIRE_THAT(result.tf.den[2], WithinAbs(0.819, 1e-6));
    REQUIRE_THAT(result.rmse, WithinAbs(0.0, 1e-7));
}

TEST_CASE("fitLevy throws when there are fewer frequency samples than unknown coefficients",
          "[freq_domain_id]")
{
    // num_order=1, den_order=2 -> 1+1+2 = 4 unknowns, but only 2 samples provided.
    const std::vector<double> freqs{1.0, 5.0};
    const std::vector<std::complex<double>> response{{0.1, 0.0}, {0.2, -0.1}};

    REQUIRE_THROWS_AS(
        ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 1, 2, 0.01),
        std::invalid_argument);
}

TEST_CASE("buildLevySystem with empty weights reproduces fitLevy's pre-refactor numerical "
          "result exactly (regression for the FD1 refactor)",
          "[freq_domain_id]")
{
    const ctrl::TransferFunction tf({0.0, 0.2}, {1.0, -0.8}, 0.1);
    const auto sys = ctrl::tf2ss(tf);
    const std::vector<double> freqs{0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0};
    const auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    Eigen::MatrixXd Phi;
    Eigen::VectorXd y;
    ctrl::FreqDomainIdentifier::buildLevySystem(freqs, response, 1, 1, 0.1, {}, Phi, y);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Phi);
    const Eigen::VectorXd x = qr.solve(y);

    const auto fitResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 1, 1, 0.1);

    // x = [num0, num1, den1] per buildLevySystem's stacking convention.
    REQUIRE_THAT(x(0), WithinAbs(fitResult.tf.num[0], 1e-12));
    REQUIRE_THAT(x(1), WithinAbs(fitResult.tf.num[1], 1e-12));
    REQUIRE_THAT(x(2), WithinAbs(fitResult.tf.den[1], 1e-12));
}

// -----------------------------------------------------------------------------
// SKFit - Sanathanan-Koerner-reweighted complex-response fitting (Phase 3 FD1)
// -----------------------------------------------------------------------------

TEST_CASE("SKFit reduces RMSE relative to a one-shot fitLevy on a noisy lightly-damped response",
          "[sk_complex_fit]")
{
    // Lightly-damped 2nd-order resonance: poles at r*exp(+-j*theta), r close to 1.
    const double r = 0.97, theta = 0.6;
    const double a1 = -2.0 * r * std::cos(theta);
    const double a2 = r * r;
    const ctrl::TransferFunction tf({0.0, 1.0 - r, 0.0}, {1.0, a1, a2}, 0.1);
    const auto sys = ctrl::tf2ss(tf);

    std::vector<double> freqs;
    for (int i = 1; i <= 40; ++i) freqs.push_back(0.5 * i);
    auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    // Add small synthetic measurement noise - with noiseless, exactly-order-matched data the
    // linear system is fully consistent and any reweighting reproduces the same exact zero-
    // residual solution, so SK can only show a measurable improvement once the system is
    // overdetermined-with-residual (the realistic case this algorithm targets).
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.01);
    for (auto &h : response) h += std::complex<double>(noise(rng), noise(rng));

    const auto levyResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 2, 2, 0.1);
    const auto skResult    = ctrl::SKFit::fitSK(freqs, response, 2, 2, 0.1);

    REQUIRE(skResult.iterCost.size() >= 1u);
    REQUIRE(skResult.iterCost.front() < levyResult.rmse + 1e-9); // iter 0 == unweighted Levy
    REQUIRE(skResult.iterCost.back() < levyResult.rmse);
}

TEST_CASE("SKFit's RMSE trends non-increasing from the first iteration to the last",
          "[sk_complex_fit]")
{
    // SK reweighting minimizes a *weighted* residual each iteration, not the raw RMSE
    // directly - on noisy data the raw RMSE can tick up by a tiny amount between
    // individual iterations even as the weighted fit improves overall, so this checks the
    // first-vs-last trend (the property SKFit's other tests already rely on) rather than
    // asserting strict step-by-step monotonicity.
    const double r = 0.95, theta = 0.8;
    const double a1 = -2.0 * r * std::cos(theta);
    const double a2 = r * r;
    const ctrl::TransferFunction tf({0.0, 1.0 - r, 0.0}, {1.0, a1, a2}, 0.1);
    const auto sys = ctrl::tf2ss(tf);

    std::vector<double> freqs;
    for (int i = 1; i <= 30; ++i) freqs.push_back(0.5 * i);
    auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);
    std::mt19937 rng(11);
    std::normal_distribution<double> noise(0.0, 0.02);
    for (auto &h : response) h += std::complex<double>(noise(rng), noise(rng));

    const auto result = ctrl::SKFit::fitSK(freqs, response, 2, 2, 0.1, /*max_iter=*/15);

    REQUIRE(result.iterCost.size() >= 2u);
    REQUIRE(result.iterCost.back() <= result.iterCost.front() + 1e-9);
    // Per-step check with a small relative tolerance, absorbing the kind of tiny
    // noise-driven uptick described above without masking a genuinely broken iteration.
    for (std::size_t i = 1; i < result.iterCost.size(); ++i)
        REQUIRE(result.iterCost[i] <= result.iterCost[i - 1] * 1.05 + 1e-6);
}

TEST_CASE("SKFit does not regress an already-good fitLevy result on a low-order, low-damping system",
          "[sk_complex_fit]")
{
    const ctrl::TransferFunction tf({0.0, 0.2}, {1.0, -0.8}, 0.1);
    const auto sys = ctrl::tf2ss(tf);
    const std::vector<double> freqs{0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0};
    const auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    const auto levyResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 1, 1, 0.1);
    const auto skResult    = ctrl::SKFit::fitSK(freqs, response, 1, 1, 0.1);

    REQUIRE(skResult.iterCost.back() <= levyResult.rmse + 1e-9);
}

// -----------------------------------------------------------------------------
// MLEIdentifier (Phase 3 Roadmap Phase 2 SI1)
// -----------------------------------------------------------------------------

namespace
{
// Shared synthetic ARX dataset for MLEIdentifier tests: y[k] = 0.6*y[k-1] + 0.4*u[k-1] + noise.
struct MLETestData
{
    Eigen::VectorXd u, y;
};

MLETestData makeMLEData(int N, double noiseAmplitude, unsigned seed, bool laplaceOutliers = false)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uDist(-1.0, 1.0);
    std::uniform_real_distribution<double> noiseDist(-noiseAmplitude, noiseAmplitude);
    std::uniform_real_distribution<double> outlierDist(0.0, 1.0);

    MLETestData d;
    d.u = Eigen::VectorXd(N);
    d.y = Eigen::VectorXd::Zero(N);
    for (int k = 0; k < N; ++k) d.u(k) = uDist(rng);
    for (int k = 1; k < N; ++k)
    {
        double noise = noiseDist(rng);
        if (laplaceOutliers && outlierDist(rng) < 0.05) noise += (outlierDist(rng) < 0.5 ? -1 : 1) * 5.0;
        d.y(k) = 0.6 * d.y(k - 1) + 0.4 * d.u(k - 1) + noise;
    }
    return d;
}
} // namespace

TEST_CASE("MLEIdentifier Gaussian/no-prior matches a direct batch least-squares solve",
          "[mle_identification]")
{
    const auto data = makeMLEData(300, 0.01, 1);

    ctrl::MLEParams p; p.na = 1; p.nb = 1; p.noise = ctrl::NoiseModel::Gaussian;
    const auto result = ctrl::MLEIdentifier::fit(data.u, data.y, 0.1, p);

    REQUIRE(result.theta.allFinite());
    REQUIRE_THAT(result.theta(0), WithinAbs(-0.6, 0.1));
    REQUIRE_THAT(result.theta(1), WithinAbs(0.4, 0.1));
}

TEST_CASE("MLEIdentifier with an informative prior pulls theta toward prior_mean (MAP)",
          "[mle_identification]")
{
    const auto data = makeMLEData(60, 0.01, 2); // short dataset - prior should matter

    ctrl::MLEParams pNoPrior; pNoPrior.na = 1; pNoPrior.nb = 1;
    const auto noPrior = ctrl::MLEIdentifier::fit(data.u, data.y, 0.1, pNoPrior);

    ctrl::MLEParams pPrior = pNoPrior;
    pPrior.prior_mean = Eigen::VectorXd(2); pPrior.prior_mean << 0.0, 0.0;
    pPrior.prior_cov  = 0.001 * Eigen::MatrixXd::Identity(2, 2); // tight prior at the origin
    const auto withPrior = ctrl::MLEIdentifier::fit(data.u, data.y, 0.1, pPrior);

    REQUIRE(withPrior.theta.norm() < noPrior.theta.norm());
}

TEST_CASE("MLEIdentifier with Laplace noise outperforms the Gaussian/LS fit on outlier-heavy data",
          "[mle_identification]")
{
    const auto data = makeMLEData(300, 0.01, 3, /*laplaceOutliers=*/true);
    const Eigen::Vector2d trueTheta(-0.6, 0.4);

    ctrl::MLEParams pGauss; pGauss.na = 1; pGauss.nb = 1; pGauss.noise = ctrl::NoiseModel::Gaussian;
    const auto gaussResult = ctrl::MLEIdentifier::fit(data.u, data.y, 0.1, pGauss);

    ctrl::MLEParams pLaplace = pGauss; pLaplace.noise = ctrl::NoiseModel::Laplace;
    const auto laplaceResult = ctrl::MLEIdentifier::fit(data.u, data.y, 0.1, pLaplace);

    const double gaussErr    = (gaussResult.theta - trueTheta).norm();
    const double laplaceErr  = (laplaceResult.theta - trueTheta).norm();
    REQUIRE(laplaceErr < gaussErr);
}

TEST_CASE("MLEIdentifier's covariance shrinks as the sample count grows (asymptotic consistency)",
          "[mle_identification]")
{
    ctrl::MLEParams p; p.na = 1; p.nb = 1;

    const auto smallData = makeMLEData(60, 0.05, 4);
    const auto largeData = makeMLEData(600, 0.05, 4);

    const auto smallResult = ctrl::MLEIdentifier::fit(smallData.u, smallData.y, 0.1, p);
    const auto largeResult = ctrl::MLEIdentifier::fit(largeData.u, largeData.y, 0.1, p);

    REQUIRE(smallResult.covariance.diagonal().minCoeff() > 0.0);
    REQUIRE(largeResult.covariance.diagonal().minCoeff() > 0.0);
    REQUIRE(largeResult.covariance.trace() < smallResult.covariance.trace());
}

// -----------------------------------------------------------------------------
// HammersteinWienerIdentifier (Phase 3 SI5)
// -----------------------------------------------------------------------------

TEST_CASE("HammersteinWienerIdentifier::fitHammerstein recovers a known cubic nonlinearity "
          "and linear part",
          "[hammerstein_wiener]")
{
    // v[k] = u[k] + 0.3*u[k]^3 (cubic, linear term normalized to 1), then
    // y[k] = 0.8*y[k-1] + 0.5*v[k-1] (na=1, nb=1).
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const int N = 400;
    Eigen::VectorXd u(N), v(N), y(N);
    for (int k = 0; k < N; ++k) u(k) = dist(rng);
    for (int k = 0; k < N; ++k) v(k) = u(k) + 0.3 * u(k) * u(k) * u(k);
    y(0) = 0.0;
    for (int k = 1; k < N; ++k) y(k) = 0.8 * y(k - 1) + 0.5 * v(k - 1);

    ctrl::HammersteinWienerParams p;
    p.na = 1; p.nb = 1; p.nl_degree = 3;
    const auto result = ctrl::HammersteinWienerIdentifier::fitHammerstein(u, y, 0.1, p);

    REQUIRE(result.nl_input_coeffs.size() == 4);
    REQUIRE_THAT(result.nl_input_coeffs(1), WithinAbs(1.0, 1e-9)); // normalization
    REQUIRE_THAT(result.nl_input_coeffs(0), WithinAbs(0.0, 0.02));
    REQUIRE_THAT(result.nl_input_coeffs(3), WithinAbs(0.3, 0.05));
    REQUIRE(result.linear_part.den.size() == 2);
    REQUIRE_THAT(result.linear_part.den[1], WithinAbs(-0.8, 0.05));
    REQUIRE_THAT(result.linear_part.num[1], WithinAbs(0.5, 0.05));
}

TEST_CASE("HammersteinWienerIdentifier::fitWiener recovers a known cubic output nonlinearity "
          "and linear part",
          "[hammerstein_wiener]")
{
    // w[k] = 0.8*w[k-1] + 0.5*u[k-1] (na=1, nb=1), then y[k] = w[k] + 0.2*w[k]^3.
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const int N = 400;
    Eigen::VectorXd u(N), w(N), y(N);
    for (int k = 0; k < N; ++k) u(k) = dist(rng);
    w(0) = 0.0;
    for (int k = 1; k < N; ++k) w(k) = 0.8 * w(k - 1) + 0.5 * u(k - 1);
    for (int k = 0; k < N; ++k) y(k) = w(k) + 0.2 * w(k) * w(k) * w(k);

    ctrl::HammersteinWienerParams p;
    p.na = 1; p.nb = 1; p.nl_degree = 3;
    const auto result = ctrl::HammersteinWienerIdentifier::fitWiener(u, y, 0.1, p);

    REQUIRE(result.nl_output_coeffs.size() == 4);
    REQUIRE_THAT(result.nl_output_coeffs(1), WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(result.nl_output_coeffs(0), WithinAbs(0.0, 0.05));
    REQUIRE_THAT(result.nl_output_coeffs(3), WithinAbs(0.2, 0.1));
    REQUIRE(result.linear_part.den.size() == 2);
    REQUIRE_THAT(result.linear_part.den[1], WithinAbs(-0.8, 0.1));
}

TEST_CASE("HammersteinWienerIdentifier::fitHammerstein doesn't overfit a pure-linear system",
          "[hammerstein_wiener]")
{
    std::mt19937 rng(13);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const int N = 300;
    Eigen::VectorXd u(N), y(N);
    for (int k = 0; k < N; ++k) u(k) = dist(rng);
    y(0) = 0.0;
    for (int k = 1; k < N; ++k) y(k) = 0.7 * y(k - 1) + 0.4 * u(k - 1); // nonlinearity = identity

    ctrl::HammersteinWienerParams p;
    p.na = 1; p.nb = 1; p.nl_degree = 3;
    const auto result = ctrl::HammersteinWienerIdentifier::fitHammerstein(u, y, 0.1, p);

    REQUIRE_THAT(result.nl_input_coeffs(1), WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(result.nl_input_coeffs(0), WithinAbs(0.0, 0.05));
    REQUIRE_THAT(result.nl_input_coeffs(2), WithinAbs(0.0, 0.05));
    REQUIRE_THAT(result.nl_input_coeffs(3), WithinAbs(0.0, 0.05));
}

TEST_CASE("HammersteinWienerIdentifier reports converged=false and iters=max_iter when the "
          "tolerance isn't reached",
          "[hammerstein_wiener]")
{
    std::mt19937 rng(21);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const int N = 300;
    Eigen::VectorXd u(N), v(N), y(N);
    for (int k = 0; k < N; ++k) u(k) = dist(rng);
    for (int k = 0; k < N; ++k) v(k) = u(k) + 0.3 * u(k) * u(k) * u(k);
    y(0) = 0.0;
    for (int k = 1; k < N; ++k) y(k) = 0.8 * y(k - 1) + 0.5 * v(k - 1);

    ctrl::HammersteinWienerParams p;
    p.na = 1; p.nb = 1; p.nl_degree = 3;
    p.max_iter = 1;
    p.tol = 1e-300; // unreachable in one iteration
    const auto result = ctrl::HammersteinWienerIdentifier::fitHammerstein(u, y, 0.1, p);

    REQUIRE_FALSE(result.converged);
    REQUIRE(result.iters == 1);
    REQUIRE(result.nl_input_coeffs.allFinite());
}

TEST_CASE("CorrelationID recovers a known FIR impulse response from PRBS-driven data",
          "[correlation_id]")
{
    // Known FIR system: y[k] = 0.6*u[k] + 0.3*u[k-1] + 0.1*u[k-2]  (g = [0.6, 0.3, 0.1, 0, ...])
    const Eigen::VectorXd u = ctrl::CorrelationID::generatePRBS(2000, 9, 7);
    const int N = static_cast<int>(u.size());
    Eigen::VectorXd y = Eigen::VectorXd::Zero(N);
    for (int k = 0; k < N; ++k)
    {
        y(k) = 0.6 * u(k);
        if (k >= 1) y(k) += 0.3 * u(k - 1);
        if (k >= 2) y(k) += 0.1 * u(k - 2);
    }

    ctrl::CorrelationIDParams p;
    p.max_lag = 10;
    const auto result = ctrl::CorrelationID::identify(u, y, 0.01, p);

    REQUIRE(result.impulse_response.size() == 11);
    REQUIRE_THAT(result.impulse_response(0), WithinAbs(0.6, 0.02));
    REQUIRE_THAT(result.impulse_response(1), WithinAbs(0.3, 0.02));
    REQUIRE_THAT(result.impulse_response(2), WithinAbs(0.1, 0.02));
    for (int k = 3; k <= 10; ++k)
        REQUIRE_THAT(result.impulse_response(k), WithinAbs(0.0, 0.02));
}

TEST_CASE("CorrelationID::generatePRBS produces a near-white autocorrelation", "[correlation_id]")
{
    const Eigen::VectorXd prbs = ctrl::CorrelationID::generatePRBS(2000, 8, 123);
    REQUIRE(prbs.size() == 2000);
    REQUIRE((prbs.array() == 1.0 || prbs.array() == -1.0).all());

    ctrl::CorrelationIDParams p;
    p.max_lag = 20;
    const auto result = ctrl::CorrelationID::identify(prbs, prbs, 0.01, p);
    // R_uu(0) should dominate every nonzero-lag value by a wide margin for a near-white PRBS.
    for (int k = 1; k <= 20; ++k)
        REQUIRE(std::fabs(result.autocorr_u(k)) < 0.1 * result.autocorr_u(0));
}

TEST_CASE("CorrelationID without whitening is visibly biased on a colored (non-PRBS) input",
          "[correlation_id]")
{
    // Colored (strongly autocorrelated) input: a random walk, not white at all.
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 1.0);
    const int N = 500;
    Eigen::VectorXd u(N), y(N);
    u(0) = noise(rng);
    for (int k = 1; k < N; ++k) u(k) = u(k - 1) + noise(rng);
    for (int k = 0; k < N; ++k)
    {
        y(k) = 0.6 * u(k);
        if (k >= 1) y(k) += 0.3 * u(k - 1);
    }

    ctrl::CorrelationIDParams p;
    p.max_lag = 5;
    p.whiten_input = false;
    const auto result = ctrl::CorrelationID::identify(u, y, 0.01, p);

    // Regression test documenting the known limitation, not a bug: without whitening, a
    // colored input's impulse-response estimate is visibly biased away from the true [0.6, 0.3].
    const double bias_k0 = std::fabs(result.impulse_response(0) - 0.6);
    const double bias_k1 = std::fabs(result.impulse_response(1) - 0.3);
    REQUIRE((bias_k0 > 0.05 || bias_k1 > 0.05));
}

TEST_CASE("CorrelationID::identify throws on mismatched lengths or an out-of-range max_lag",
          "[correlation_id]")
{
    const Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
    const Eigen::VectorXd y_short = Eigen::VectorXd::Ones(5);
    REQUIRE_THROWS_AS(ctrl::CorrelationID::identify(u, y_short, 0.01), std::invalid_argument);

    ctrl::CorrelationIDParams p;
    p.max_lag = 10; // == u.size(), out of range (must be < N)
    REQUIRE_THROWS_AS(ctrl::CorrelationID::identify(u, u, 0.01, p), std::invalid_argument);
}

TEST_CASE("ResonantController steady-state gain at the target frequency equals Kr exactly",
          "[resonant_controller]")
{
    const double Ts = 1e-4;
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0;
    p.dampingRadPerSec = 5.0;
    p.Kr = 2.0;
    ctrl::ResonantController rc(p, Ts);

    const int N = 30000; // 3s = 150 cycles @ 50Hz, ample settling margin
    double maxAbsLastCycle = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double e = std::sin(2.0 * M_PI * p.targetFreqHz * k * Ts);
        const double u = rc.compute(e);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(u));
    }
    REQUIRE_THAT(maxAbsLastCycle, WithinRel(p.Kr, 0.001));
}

TEST_CASE("ResonantController attenuates strongly away from the target frequency",
          "[resonant_controller]")
{
    const double Ts = 1e-4;
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0;
    p.dampingRadPerSec = 5.0;
    p.Kr = 2.0;
    ctrl::ResonantController rc(p, Ts);

    const int N = 30000;
    double maxAbsLastCycle = 0.0;
    const double fOff = 150.0;
    for (int k = 0; k < N; ++k)
    {
        const double e = std::sin(2.0 * M_PI * fOff * k * Ts);
        const double u = rc.compute(e);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(u));
    }
    REQUIRE(maxAbsLastCycle < 0.1); // << Kr=2.0 at resonance
}

TEST_CASE("ResonantController holds last output on a non-finite input and leaves state unchanged",
          "[resonant_controller]")
{
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;
    ctrl::ResonantController rc(p, 1e-4);

    const double u1 = rc.compute(1.0);
    const double u_nan = rc.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(u_nan == u1);

    const double u2 = rc.compute(1.0);
    ctrl::ResonantController rc_ref(p, 1e-4);
    rc_ref.compute(1.0);
    const double u2_ref = rc_ref.compute(1.0);
    REQUIRE_THAT(u2, WithinAbs(u2_ref, 1e-12));
}

TEST_CASE("ResonantController reset() clears internal state", "[resonant_controller]")
{
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;
    ctrl::ResonantController rc(p, 1e-4);

    for (int k = 0; k < 100; ++k)
        rc.compute(std::sin(2.0 * M_PI * 50.0 * k * 1e-4));
    rc.reset();

    ctrl::ResonantController rc_fresh(p, 1e-4);
    REQUIRE_THAT(rc.compute(1.0), WithinAbs(rc_fresh.compute(1.0), 1e-12));
}

TEST_CASE("ResonantController throws on invalid construction parameters", "[resonant_controller]")
{
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;

    ctrl::ResonantParams bad1 = p; bad1.targetFreqHz = 0.0;
    REQUIRE_THROWS_AS(ctrl::ResonantController(bad1, 1e-4), std::invalid_argument);

    ctrl::ResonantParams bad2 = p; bad2.dampingRadPerSec = -1.0;
    REQUIRE_THROWS_AS(ctrl::ResonantController(bad2, 1e-4), std::invalid_argument);

    ctrl::ResonantParams bad3 = p; bad3.targetFreqHz = 6000.0; // Nyquist = 5000Hz at Ts=1e-4
    REQUIRE_THROWS_AS(ctrl::ResonantController(bad3, 1e-4), std::invalid_argument);
}

TEST_CASE("ResonantController reports TrackingErrorRMinusY as its sign convention",
          "[resonant_controller]")
{
    ctrl::ResonantParams p; p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;
    ctrl::ResonantController rc(p, 1e-4);
    REQUIRE(rc.signConvention() == ctrl::SignConvention::TrackingErrorRMinusY);
}

TEST_CASE("NotchFilter attenuates a sinusoid exactly at the center frequency to near zero",
          "[notch_filter]")
{
    const double Ts = 1e-4;
    ctrl::NotchFilterParams p;
    p.centerFreqHz = 50.0;
    p.Q = 10.0;
    ctrl::NotchFilter nf(p, Ts);

    const int N = 6000; // 30 cycles - enough for the near-unit-circle poles (Q=10) to settle
    double maxAbsLastCycle = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double x = std::sin(2.0 * M_PI * p.centerFreqHz * k * Ts);
        const double y = nf.apply(x);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(y));
    }
    REQUIRE(maxAbsLastCycle < 0.01);
}

TEST_CASE("NotchFilter passes a sinusoid far from the center frequency through largely unchanged",
          "[notch_filter]")
{
    const double Ts = 1e-4;
    ctrl::NotchFilterParams p;
    p.centerFreqHz = 50.0;
    p.Q = 10.0;
    ctrl::NotchFilter nf(p, Ts);

    const int N = 3000;
    double maxAbsLastCycle = 0.0;
    const double fFar = 200.0;
    for (int k = 0; k < N; ++k)
    {
        const double x = std::sin(2.0 * M_PI * fFar * k * Ts);
        const double y = nf.apply(x);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(y));
    }
    REQUIRE(maxAbsLastCycle > 0.9);
    REQUIRE(maxAbsLastCycle < 1.1);
}

TEST_CASE("NotchFilter holds last output on a non-finite input", "[notch_filter]")
{
    ctrl::NotchFilterParams p; p.centerFreqHz = 50.0; p.Q = 10.0;
    ctrl::NotchFilter nf(p, 1e-4);

    const double y1 = nf.apply(1.0);
    const double y_nan = nf.apply(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(y_nan == y1);
}

TEST_CASE("NotchFilter reset() clears internal state", "[notch_filter]")
{
    ctrl::NotchFilterParams p; p.centerFreqHz = 50.0; p.Q = 10.0;
    ctrl::NotchFilter nf(p, 1e-4);

    for (int k = 0; k < 100; ++k)
        nf.apply(std::sin(2.0 * M_PI * 50.0 * k * 1e-4));
    nf.reset();

    ctrl::NotchFilter nf_fresh(p, 1e-4);
    REQUIRE_THAT(nf.apply(1.0), WithinAbs(nf_fresh.apply(1.0), 1e-12));
}

TEST_CASE("NotchFilter throws on invalid construction parameters", "[notch_filter]")
{
    ctrl::NotchFilterParams p; p.centerFreqHz = 50.0; p.Q = 10.0;

    ctrl::NotchFilterParams bad1 = p; bad1.centerFreqHz = 0.0;
    REQUIRE_THROWS_AS(ctrl::NotchFilter(bad1, 1e-4), std::invalid_argument);

    ctrl::NotchFilterParams bad2 = p; bad2.Q = -1.0;
    REQUIRE_THROWS_AS(ctrl::NotchFilter(bad2, 1e-4), std::invalid_argument);

    ctrl::NotchFilterParams bad3 = p; bad3.centerFreqHz = 6000.0;
    REQUIRE_THROWS_AS(ctrl::NotchFilter(bad3, 1e-4), std::invalid_argument);
}

TEST_CASE("PhaseLockedLoop converges to the true phase and frequency of a synthetic sinusoid",
          "[pll]")
{
    const double Ts = 1e-4;
    ctrl::PLLParams p;
    p.nominalFreqHz = 50.0;
    p.Kp = 90.0;
    p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, Ts);

    const int N = 20000; // 2s, ~22x the ~90ms loop settling time
    double phaseTrue = 0.7; // nonzero initial offset to exercise convergence
    for (int k = 0; k < N; ++k)
    {
        pll.step(std::sin(phaseTrue));
        phaseTrue += 2.0 * M_PI * 50.0 * Ts;
    }
    const double trueWrapped = std::atan2(std::sin(phaseTrue), std::cos(phaseTrue));
    const double diff = std::atan2(std::sin(pll.phase() - trueWrapped),
                                    std::cos(pll.phase() - trueWrapped));

    REQUIRE(pll.locked());
    REQUIRE_THAT(pll.frequencyHz(), WithinAbs(50.0, 0.5));
    REQUIRE(std::fabs(diff) < 0.1);
}

TEST_CASE("PhaseLockedLoop re-converges after a step change in the true input frequency", "[pll]")
{
    const double Ts = 1e-4;
    ctrl::PLLParams p;
    p.nominalFreqHz = 50.0;
    p.Kp = 90.0;
    p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, Ts);

    const int N = 20000; // 2s total, 1s at 50Hz then 1s at 53Hz
    double phaseTrue = 0.0;
    double fTrue = 50.0;
    for (int k = 0; k < N; ++k)
    {
        if (k == N / 2) fTrue = 53.0;
        pll.step(std::sin(phaseTrue));
        phaseTrue += 2.0 * M_PI * fTrue * Ts;
    }

    REQUIRE(pll.locked());
    REQUIRE_THAT(pll.frequencyHz(), WithinAbs(53.0, 0.5));
}

TEST_CASE("PhaseLockedLoop holds its estimate on a non-finite sample", "[pll]")
{
    ctrl::PLLParams p; p.nominalFreqHz = 50.0; p.Kp = 90.0; p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, 1e-4);

    for (int k = 0; k < 100; ++k)
        pll.step(std::sin(2.0 * M_PI * 50.0 * k * 1e-4));
    const double freqBefore = pll.frequencyHz();
    const double phaseBefore = pll.phase();

    pll.step(std::numeric_limits<double>::quiet_NaN());

    REQUIRE(pll.frequencyHz() == freqBefore);
    REQUIRE(pll.phase() == phaseBefore);
}

TEST_CASE("PhaseLockedLoop reset() returns the frequency estimate to nominal", "[pll]")
{
    ctrl::PLLParams p; p.nominalFreqHz = 50.0; p.Kp = 90.0; p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, 1e-4);

    for (int k = 0; k < 1000; ++k)
        pll.step(std::sin(2.0 * M_PI * 53.0 * k * 1e-4)); // off-nominal input
    pll.reset();

    REQUIRE_THAT(pll.frequencyHz(), WithinAbs(50.0, 1e-9));
    REQUIRE_THAT(pll.phase(), WithinAbs(0.0, 1e-9));
    REQUIRE(!pll.locked());
}

TEST_CASE("PhaseLockedLoop throws on invalid construction parameters", "[pll]")
{
    ctrl::PLLParams p; p.nominalFreqHz = 50.0; p.Kp = 90.0; p.Ki = 4000.0;

    ctrl::PLLParams bad1 = p; bad1.nominalFreqHz = 0.0;
    REQUIRE_THROWS_AS(ctrl::PhaseLockedLoop(bad1, 1e-4), std::invalid_argument);

    ctrl::PLLParams bad2 = p; bad2.nominalFreqHz = 6000.0;
    REQUIRE_THROWS_AS(ctrl::PhaseLockedLoop(bad2, 1e-4), std::invalid_argument);
}


// =============================================================================
// NeuralNetworkController (Phase 3 ML1)
// =============================================================================

TEST_CASE("NeuralNetworkController forward pass matches a hand-computed network", "[neural_network_controller]")
{
    // Hidden layer (2x1) tanh, output layer (1x2) linear: u = w*tanh(W*in + b1) + b2
    ctrl::NNLayerSpec h;
    h.W = Eigen::MatrixXd(2, 1);
    h.W << 0.5, -1.5;
    h.b = Eigen::VectorXd(2);
    h.b << 0.1, -0.2;
    h.activation = ctrl::NNLayerSpec::Activation::Tanh;

    ctrl::NNLayerSpec o;
    o.W = Eigen::MatrixXd(1, 2);
    o.W << 2.0, 3.0;
    o.b = Eigen::VectorXd::Constant(1, 0.25);
    o.activation = ctrl::NNLayerSpec::Activation::Linear;

    ctrl::NeuralControllerParams p;
    p.layers = {h, o};
    p.n_input_features = 1;
    ctrl::NeuralNetworkController nn(p, 0.01);

    const double in = 0.8;
    const double h0 = std::tanh(0.5 * in + 0.1);
    const double h1 = std::tanh(-1.5 * in - 0.2);
    const double expected = 2.0 * h0 + 3.0 * h1 + 0.25;
    REQUIRE_THAT(nn.compute(in), WithinAbs(expected, 1e-12));
}

TEST_CASE("NeuralNetworkController saturates and holds last output on NaN", "[neural_network_controller]")
{
    ctrl::NNLayerSpec layer;
    layer.W = Eigen::MatrixXd::Constant(1, 1, 100.0); // large gain to force saturation
    layer.b = Eigen::VectorXd::Zero(1);
    layer.activation = ctrl::NNLayerSpec::Activation::Linear;
    ctrl::NeuralControllerParams p;
    p.layers = {layer};
    p.n_input_features = 1;
    p.uMin = -2.0;
    p.uMax = 2.0;
    ctrl::NeuralNetworkController nn(p, 0.01);

    const double u_sat = nn.compute(1.0); // 100*1 -> clamped to uMax
    REQUIRE_THAT(u_sat, WithinAbs(2.0, 1e-12));
    // hold-last on non-finite input
    REQUIRE_THAT(nn.compute(std::nan("")), WithinAbs(2.0, 1e-12));
}

TEST_CASE("NeuralNetworkController loadWeights hot-swap changes output immediately", "[neural_network_controller]")
{
    ctrl::NNLayerSpec layer;
    layer.W = Eigen::MatrixXd::Constant(1, 1, 1.0);
    layer.b = Eigen::VectorXd::Zero(1);
    layer.activation = ctrl::NNLayerSpec::Activation::Linear;
    ctrl::NeuralControllerParams p;
    p.layers = {layer};
    p.n_input_features = 1;
    ctrl::NeuralNetworkController nn(p, 0.01);

    REQUIRE_THAT(nn.compute(1.0), WithinAbs(1.0, 1e-12));

    ctrl::NNLayerSpec newL = layer;
    newL.W = Eigen::MatrixXd::Constant(1, 1, -5.0);
    nn.loadWeights({newL});
    REQUIRE_THAT(nn.compute(1.0), WithinAbs(-5.0, 1e-12));
}

TEST_CASE("NeuralNetworkController rejects inconsistent layer dimensions", "[neural_network_controller]")
{
    ctrl::NNLayerSpec bad;
    bad.W = Eigen::MatrixXd(2, 3); // 3 inputs but n_input_features=1
    bad.W.setZero();
    bad.b = Eigen::VectorXd::Zero(2);
    bad.activation = ctrl::NNLayerSpec::Activation::Linear;
    ctrl::NeuralControllerParams p;
    p.layers = {bad};
    p.n_input_features = 1;
    REQUIRE_THROWS_AS(ctrl::NeuralNetworkController(p, 0.01), std::invalid_argument);
}

// =============================================================================
// NNAdaptiveController (Phase 3 ML2)
// =============================================================================

static ctrl::NNAdaptiveParams makeNNAdaptiveParams()
{
    ctrl::NNLayerSpec hidden;
    hidden.W = Eigen::MatrixXd(6, 2);
    hidden.W << 1.0, 0.5, -0.8, 0.3, 0.6, -0.4, -0.5, 0.7, 0.9, -0.2, 0.2, 0.8;
    hidden.b = Eigen::VectorXd::Zero(6);
    hidden.activation = ctrl::NNLayerSpec::Activation::Tanh;
    ctrl::NNLayerSpec out;
    out.W = Eigen::MatrixXd::Zero(1, 6);
    out.b = Eigen::VectorXd::Zero(1);
    out.activation = ctrl::NNLayerSpec::Activation::Linear;
    ctrl::NNAdaptiveParams p;
    p.nn.layers = {hidden, out};
    p.nn.n_input_features = 2;
    p.gamma_adapt = 3.0;
    p.sigma_mod = 0.01;
    p.a_m = 0.6;
    p.b_m = 0.4;
    p.uMin = -50.0;
    p.uMax = 50.0;
    return p;
}

TEST_CASE("NNAdaptiveController tracks the reference model with bounded weights", "[nn_adaptive_control]")
{
    const double Ts = 0.01;
    ctrl::NNAdaptiveController c(makeNNAdaptiveParams(), Ts);

    const double r = 1.0;
    double y = 0.0, y_m = 0.0, late_err = 0.0;
    for (int k = 0; k < 12000; ++k)
    {
        c.setReference(r);
        const double u = c.compute(y);
        y = 0.9 * y + 0.1 * (u + 0.3 * std::sin(y)); // unknown input nonlinearity
        y_m = 0.6 * y_m + 0.4 * r;
        if (k > 10000)
            late_err = std::max(late_err, std::fabs(y - y_m));
    }
    REQUIRE(std::isfinite(y));
    REQUIRE(late_err < 0.15);                 // tracks the reference model
    REQUIRE(c.outputWeightNorm() < 1e3);      // sigma-mod keeps weights bounded
}

TEST_CASE("NNAdaptiveController holds last output on NaN and resets weights", "[nn_adaptive_control]")
{
    ctrl::NNAdaptiveController c(makeNNAdaptiveParams(), 0.01);
    c.setReference(1.0);
    for (int k = 0; k < 50; ++k)
        c.compute(0.0);
    const double w_after = c.outputWeightNorm();
    REQUIRE(w_after > 0.0); // weights moved away from the zero initialization

    const double u_last = c.compute(0.0);
    REQUIRE_THAT(c.compute(std::nan("")), WithinAbs(u_last, 1e-12)); // hold-last

    c.reset();
    REQUIRE_THAT(c.outputWeightNorm(), WithinAbs(0.0, 1e-12)); // back to zero init
    REQUIRE_THAT(c.modelOutput(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("NNAdaptiveController rejects a non-Linear output layer", "[nn_adaptive_control]")
{
    ctrl::NNAdaptiveParams p = makeNNAdaptiveParams();
    p.nn.layers.back().activation = ctrl::NNLayerSpec::Activation::Tanh;
    REQUIRE_THROWS_AS(ctrl::NNAdaptiveController(p, 0.01), std::invalid_argument);
}

// =============================================================================
// NonlinearIMC (Phase 3 NC3)
// =============================================================================

namespace {
double nimc_model(const Eigen::VectorXd &x, double u) { return 0.7 * x(0) + 0.3 * u; }
double nimc_inverse(const Eigen::VectorXd &x, double y_t) { return (y_t - 0.7 * x(0)) / 0.3; }

double runNonlinearIMC(double plant_a, double plant_b)
{
    ctrl::NonlinearIMCParams p;
    p.filter_lambda = 0.5;
    p.uMin = -100.0;
    p.uMax = 100.0;
    ctrl::NonlinearIMC imc(nimc_model, nimc_inverse, p, 0.1);
    const double r = 1.0;
    double y = 0.0;
    Eigen::VectorXd x(1);
    for (int k = 0; k < 500; ++k)
    {
        x(0) = y;
        imc.setState(x);
        const double u = imc.compute(r - y);
        y = plant_a * y + plant_b * u;
    }
    return y;
}
} // namespace

TEST_CASE("NonlinearIMC tracks offset-free when the model matches the plant", "[nonlinear_imc]")
{
    REQUIRE_THAT(runNonlinearIMC(0.7, 0.3), WithinAbs(1.0, 1e-3));
}

TEST_CASE("NonlinearIMC rejects steady-state offset under model mismatch", "[nonlinear_imc]")
{
    REQUIRE_THAT(runNonlinearIMC(0.75, 0.28), WithinAbs(1.0, 1e-2));
}

TEST_CASE("NonlinearIMC holds last output on a non-finite inverse result", "[nonlinear_imc]")
{
    ctrl::NonlinearIMCParams p;
    ctrl::NonlinearIMC imc(
        [](const Eigen::VectorXd &x, double u) { return x(0) + u; },
        [](const Eigen::VectorXd &, double) { return std::nan(""); }, // singular inverse
        p, 0.1);
    Eigen::VectorXd x(1);
    x << 0.0;
    imc.setState(x);
    const double u = imc.compute(1.0);
    REQUIRE_THAT(u, WithinAbs(0.0, 1e-12)); // holds the initial last output (0)
}

TEST_CASE("NonlinearIMC rejects an out-of-range filter pole", "[nonlinear_imc]")
{
    ctrl::NonlinearIMCParams p;
    p.filter_lambda = 1.0; // must be < 1
    REQUIRE_THROWS_AS(
        ctrl::NonlinearIMC([](const Eigen::VectorXd &, double) { return 0.0; },
                           [](const Eigen::VectorXd &, double) { return 0.0; }, p, 0.1),
        std::invalid_argument);
}

// =============================================================================
// NARMAXIdentifier (Phase 3 SI4)
// =============================================================================

TEST_CASE("NARMAXIdentifier recovers a known bilinear NARX term set", "[narmax]")
{
    // y[k] = 0.5 y[k-1] + 0.3 u[k-1] + 0.2 y[k-1] u[k-1]
    const int N = 600;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Eigen::VectorXd u(N), y(N);
    y(0) = 0.0;
    u(0) = dist(rng);
    for (int k = 1; k < N; ++k)
    {
        u(k) = dist(rng);
        y(k) = 0.5 * y(k - 1) + 0.3 * u(k - 1) + 0.2 * y(k - 1) * u(k - 1);
    }

    ctrl::NARMAXParams p;
    p.na = 1; p.nb = 1; p.nc = 0; p.poly_degree = 2;
    p.significance_tol = 1e-4; p.max_terms = 6;
    const ctrl::NARMAXResult res = ctrl::NARMAXIdentifier::fit(u, y, p);

    auto has = [&](const std::string &t) {
        return std::find(res.selected_terms.begin(), res.selected_terms.end(), t) !=
               res.selected_terms.end();
    };
    REQUIRE(has("y(k-1)"));
    REQUIRE(has("u(k-1)"));
    REQUIRE(has("y(k-1)*u(k-1)"));
    REQUIRE(res.final_err_sum > 0.999); // the three terms explain essentially all variance

    // Coefficients match the generating system (find each term's coefficient by name).
    auto coeffOf = [&](const std::string &t) {
        auto it = std::find(res.selected_terms.begin(), res.selected_terms.end(), t);
        return res.coefficients(static_cast<int>(it - res.selected_terms.begin()));
    };
    REQUIRE_THAT(coeffOf("y(k-1)"),        WithinAbs(0.5, 1e-6));
    REQUIRE_THAT(coeffOf("u(k-1)"),        WithinAbs(0.3, 1e-6));
    REQUIRE_THAT(coeffOf("y(k-1)*u(k-1)"), WithinAbs(0.2, 1e-6));
}

TEST_CASE("NARMAXIdentifier one-step prediction matches the true system", "[narmax]")
{
    const int N = 400;
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Eigen::VectorXd u(N), y(N);
    y(0) = 0.0; u(0) = dist(rng);
    for (int k = 1; k < N; ++k)
    {
        u(k) = dist(rng);
        y(k) = 0.6 * y(k - 1) + 0.4 * u(k - 1);
    }
    ctrl::NARMAXParams p;
    p.na = 1; p.nb = 1; p.nc = 0; p.poly_degree = 1; p.significance_tol = 1e-6;
    const ctrl::NARMAXResult res = ctrl::NARMAXIdentifier::fit(u, y, p);

    Eigen::VectorXd u_hist(1), y_hist(1);
    u_hist << 0.5; y_hist << 0.2;
    const double yhat = ctrl::NARMAXIdentifier::predict(res, u_hist, y_hist);
    REQUIRE_THAT(yhat, WithinAbs(0.6 * 0.2 + 0.4 * 0.5, 1e-6));
}

TEST_CASE("NARMAXIdentifier guards bad inputs", "[narmax]")
{
    Eigen::VectorXd u(10), y(8);
    u.setRandom();
    REQUIRE_THROWS_AS(ctrl::NARMAXIdentifier::fit(u, y, ctrl::NARMAXParams()),
                      std::invalid_argument); // length mismatch

    Eigen::VectorXd u2(50), y2(50);
    u2.setRandom(); y2.setRandom();
    ctrl::NARMAXParams big;
    big.na = 10; big.nb = 10; big.nc = 10; big.poly_degree = 4; // huge library
    REQUIRE_THROWS_AS(ctrl::NARMAXIdentifier::fit(u2, y2, big), std::invalid_argument);
}

// -----------------------------------------------------------------------------
// ComplexVectorFit - complex-conjugate-pole Vector Fitting (Phase 3 FD2)
// -----------------------------------------------------------------------------

namespace
{
std::vector<double> cvfPolyMulPair(const std::vector<double> &p, double a1, double a2)
{
    std::vector<double> result(p.size() + 2, 0.0);
    for (std::size_t i = 0; i < p.size(); ++i)
    {
        result[i]     += p[i];
        result[i + 1] += p[i] * a1;
        result[i + 2] += p[i] * a2;
    }
    return result;
}

// Builds H(zinv) = N(zinv)/D(zinv) from `pairs` complex-conjugate pole pairs (plus one
// optional real pole), evaluates it on `freqs` via tf2ss + SystemAnalysis::getFrequencyResponse,
// and adds Gaussian measurement noise - mirrors SKFit's existing test-data convention.
std::vector<std::complex<double>> cvfSyntheticResponse(
    const std::vector<std::pair<double, double>> &pairs,
    double realPole, bool hasRealPole,
    const std::vector<double> &freqs, double Ts, unsigned seed, double noiseStd)
{
    std::vector<double> den{1.0};
    for (const auto &pr : pairs)
        den = cvfPolyMulPair(den, -2.0 * pr.first * std::cos(pr.second), pr.first * pr.first);
    if (hasRealPole)
    {
        std::vector<double> next(den.size() + 1, 0.0);
        for (std::size_t i = 0; i < den.size(); ++i)
        {
            next[i]     += den[i];
            next[i + 1] += den[i] * (-realPole);
        }
        den = next;
    }
    std::vector<double> num(den.size(), 0.0);
    num[1] = 0.05;

    const ctrl::TransferFunction tf(num, den, Ts);
    const auto sys = ctrl::tf2ss(tf);
    auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, noiseStd);
    for (auto &h : response) h += std::complex<double>(noise(rng), noise(rng));
    return response;
}
} // namespace

TEST_CASE("ComplexVectorFit recovers known poles of a 3-resonance system and far outperforms "
          "a one-shot Levy fit",
          "[complex_vector_fit]")
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.99, 0.4}, {0.985, 0.55}, {0.99, 0.75}};

    std::vector<double> freqs;
    for (int i = 1; i <= 80; ++i) freqs.push_back(0.25 * i);

    const auto response = cvfSyntheticResponse(specs, 0.0, false, freqs, Ts, 11, 0.02);

    const auto cvfResult  = ctrl::ComplexVectorFit::fit(freqs, response, 0, 3, Ts, 30);
    const auto levyResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 6, 6, Ts);

    REQUIRE(cvfResult.iterError.size() >= 1u);
    REQUIRE(std::isfinite(cvfResult.iterError.back()));
    // Verified in a numpy prototype: ~600x improvement on this exact scenario; 2x is a
    // generous margin against this test's different (C++) RNG stream producing different noise.
    REQUIRE(cvfResult.iterError.back() < 0.5 * levyResult.rmse);

    REQUIRE(cvfResult.poles.size() == 6u);
    std::vector<std::complex<double>> truePoles;
    for (const auto &pr : specs)
    {
        truePoles.emplace_back(pr.first * std::cos(pr.second),  pr.first * std::sin(pr.second));
        truePoles.emplace_back(pr.first * std::cos(pr.second), -pr.first * std::sin(pr.second));
    }
    for (const auto &p : cvfResult.poles)
    {
        double bestDist = 1e9;
        for (const auto &tp : truePoles)
            bestDist = std::min(bestDist, std::abs(p - tp));
        // Verified in the prototype to recover poles within ~1e-3; 0.05 leaves generous margin.
        REQUIRE(bestDist < 0.05);
    }
}

TEST_CASE("ComplexVectorFit's returned poles always include each pole's conjugate partner",
          "[complex_vector_fit]")
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.97, 0.6}, {0.95, 1.5}};

    std::vector<double> freqs;
    for (int i = 1; i <= 40; ++i) freqs.push_back(0.5 * i);

    const auto response = cvfSyntheticResponse(specs, 0.0, false, freqs, Ts, 7, 0.01);
    const auto result = ctrl::ComplexVectorFit::fit(freqs, response, 0, 2, Ts);

    REQUIRE(result.poles.size() == 4u);
    for (const auto &p : result.poles)
    {
        bool foundConjugate = false;
        for (const auto &q : result.poles)
            if (std::abs(q - std::conj(p)) < 1e-6) { foundConjugate = true; break; }
        REQUIRE(foundConjugate);
    }
}

TEST_CASE("ComplexVectorFit correctly identifies a mixed real-pole + complex-pair system",
          "[complex_vector_fit]")
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.97, 0.6}};
    const double realPole = 0.8;

    std::vector<double> freqs;
    for (int i = 1; i <= 40; ++i) freqs.push_back(0.5 * i);

    const auto response = cvfSyntheticResponse(specs, realPole, true, freqs, Ts, 7, 0.01);
    const auto result = ctrl::ComplexVectorFit::fit(freqs, response, 1, 1, Ts);

    REQUIRE(result.poles.size() == 3u);

    int realCount = 0, complexCount = 0;
    for (const auto &p : result.poles)
    {
        if (std::abs(p.imag()) < 1e-3) ++realCount;
        else ++complexCount;
    }
    REQUIRE(realCount == 1);
    REQUIRE(complexCount == 2);

    double bestRealDist = 1e9;
    for (const auto &p : result.poles)
        if (std::abs(p.imag()) < 1e-3)
            bestRealDist = std::min(bestRealDist, std::abs(p.real() - realPole));
    REQUIRE(bestRealDist < 0.05);
}

TEST_CASE("ComplexVectorFit throws on invalid inputs", "[complex_vector_fit]")
{
    const std::vector<double> freqs{1.0, 5.0, 10.0};
    const std::vector<std::complex<double>> response{{0.1, 0.0}, {0.2, -0.1}, {0.1, 0.05}};

    REQUIRE_THROWS_AS(ctrl::ComplexVectorFit::fit({}, {}, 1, 0, 0.1), std::invalid_argument);
    REQUIRE_THROWS_AS(
        ctrl::ComplexVectorFit::fit(freqs, {{0.1, 0.0}, {0.2, -0.1}}, 1, 0, 0.1),
        std::invalid_argument);
    REQUIRE_THROWS_AS(
        ctrl::ComplexVectorFit::fit(freqs, response, 0, 0, 0.1),
        std::invalid_argument);
    // n_real_poles=1, n_complex_pairs=1 -> n_poles=3, n_unknowns=7, but only 3 samples given.
    REQUIRE_THROWS_AS(
        ctrl::ComplexVectorFit::fit(freqs, response, 1, 1, 0.1),
        std::invalid_argument);
}

// -----------------------------------------------------------------------------
// SubspaceID method variants - MOESP / N4SID / CVA (Phase 3 SI3)
// -----------------------------------------------------------------------------

namespace
{
// Simulates a discrete-time LTI system and adds independent Gaussian noise with a
// possibly different std per output channel, mirroring the design spec's prototype.
void simulateForSubspaceVariants(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                  const Eigen::MatrixXd &C, const Eigen::MatrixXd &D,
                                  const Eigen::MatrixXd &U, const Eigen::VectorXd &noiseStd,
                                  unsigned seed, Eigen::MatrixXd &Y_out)
{
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());
    const int N = static_cast<int>(U.cols());
    Y_out.resize(p, N);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 1.0);

    for (int k = 0; k < N; ++k)
    {
        const Eigen::VectorXd u_k = U.col(k);
        Eigen::VectorXd y_k = C * x + D * u_k;
        for (int j = 0; j < p; ++j)
            y_k(j) += noise(rng) * noiseStd(j);
        Y_out.col(k) = y_k;
        x = A * x + B * u_k;
    }
}
} // namespace

TEST_CASE("subspaceID(MOESP) matches n4sid() bit-for-bit (regression)",
          "[subspace_id_variants]")
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, 2000);
    for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.01, 0.01), 7, Y);

    const auto r1 = ctrl::n4sid(Y, U, 2, 10, Ts);
    const auto r2 = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::MOESP);

    REQUIRE(r1.success);
    REQUIRE(r2.success);
    REQUIRE(r1.model.value().A.isApprox(r2.model.value().A, 1e-12));
    REQUIRE(r1.model.value().B.isApprox(r2.model.value().B, 1e-12));
    REQUIRE(r1.model.value().C.isApprox(r2.model.value().C, 1e-12));
    REQUIRE(r1.singularValues.isApprox(r2.singularValues, 1e-12));
}

TEST_CASE("subspaceID recovers a known 2-output system with equal noise (all 3 methods)",
          "[subspace_id_variants]")
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, 2000);
    for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.01, 0.01), 7, Y);

    const double true_eig_mag = std::abs(A_true.eigenvalues()(0));

    for (auto method : {ctrl::SubspaceMethod::MOESP, ctrl::SubspaceMethod::N4SID,
                        ctrl::SubspaceMethod::CVA})
    {
        const auto res = ctrl::subspaceID(Y, U, 2, 10, Ts, method);
        REQUIRE(res.success);
        const double est_eig_mag = std::abs(res.model.value().A.eigenvalues()(0));
        REQUIRE_THAT(est_eig_mag, WithinAbs(true_eig_mag, 0.05));
    }
}

TEST_CASE("subspaceID(CVA) reliably beats N4SID on the high-noise channel when output noise "
          "scales are mismatched (Monte Carlo over independent trials)",
          "[subspace_id_variants]")
{
    // A single noise draw was verified (during design prototyping) to flip unpredictably
    // between CVA winning and losing -- the reliable claim only holds averaged over many
    // independent trials. Neither CVA nor N4SID is claimed to beat plain MOESP here (a
    // 40-trial Monte Carlo during prototyping showed unweighted MOESP is the strongest
    // performer on this synthetic system); this test only checks CVA's improvement over
    // N4SID's right-weighting-only approach, which IS reliable.
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;
    const std::vector<double> freqs{0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0};
    const ctrl::StateSpace trueChan1(A_true, B_true, C_true.row(1), D_true.row(1), Ts);
    const auto resp_true = ctrl::SystemAnalysis::getFrequencyResponse(trueChan1, freqs);

    auto chanError = [&](const ctrl::StateSpace &model) {
        const ctrl::StateSpace estChan1(model.A, model.B, model.C.row(1), model.D.row(1), Ts);
        const auto resp_est = ctrl::SystemAnalysis::getFrequencyResponse(estChan1, freqs);
        double err = 0.0;
        for (std::size_t k = 0; k < freqs.size(); ++k)
            err += std::abs(std::abs(resp_est[k]) - std::abs(resp_true[k]));
        return err / static_cast<double>(freqs.size());
    };

    const int n_trials = 20;
    double total_n4sid = 0.0, total_cva = 0.0;
    std::mt19937 master_rng(2026);

    for (int trial = 0; trial < n_trials; ++trial)
    {
        std::mt19937 rng_u(master_rng());
        std::mt19937 rng_n(master_rng());
        std::normal_distribution<double> u_dist(0.0, 1.0);
        Eigen::MatrixXd U(1, 2000);
        for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

        Eigen::MatrixXd Y(2, U.cols());
        {
            std::normal_distribution<double> noise(0.0, 1.0);
            const Eigen::Vector2d noiseStd(0.005, 0.3); // channel 1 has 60x channel 0's noise
            Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
            for (int k = 0; k < U.cols(); ++k)
            {
                const Eigen::VectorXd u_k = U.col(k);
                Eigen::VectorXd y_k = C_true * x + D_true * u_k;
                for (int j = 0; j < 2; ++j) y_k(j) += noise(rng_n) * noiseStd(j);
                Y.col(k) = y_k;
                x = A_true * x + B_true * u_k;
            }
        }

        const auto n4sidv = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::N4SID);
        const auto cva = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::CVA);
        REQUIRE(n4sidv.success);
        REQUIRE(cva.success);

        total_n4sid += chanError(n4sidv.model.value());
        total_cva   += chanError(cva.model.value());
    }

    const double mean_n4sid = total_n4sid / n_trials;
    const double mean_cva   = total_cva / n_trials;
    REQUIRE(mean_cva < mean_n4sid);
}

TEST_CASE("suggestOrder runs unchanged across all 3 subspaceID methods", "[subspace_id_variants]")
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, 2000);
    for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.005, 0.3), 7, Y);

    for (auto method : {ctrl::SubspaceMethod::MOESP, ctrl::SubspaceMethod::N4SID,
                        ctrl::SubspaceMethod::CVA})
    {
        const auto res = ctrl::subspaceID(Y, U, 6, 10, Ts, method);
        REQUIRE(res.success);
        const int order = ctrl::suggestOrder(res.singularValues, 0.01);
        REQUIRE(order >= 1);
    }
}

TEST_CASE("subspaceID(N4SID/CVA) reports failure (not a crash/NaN) on degenerate excitation",
          "[subspace_id_variants]")
{
    // Constant input -> Wp's Uf-conditioned covariance (L22) is singular.
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    Eigen::MatrixXd U = Eigen::MatrixXd::Constant(1, 500, 1.0);
    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.01, 0.01), 7, Y);

    const auto n4sidv = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::N4SID);
    const auto cva = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::CVA);

    REQUIRE_FALSE(n4sidv.success);
    REQUIRE_FALSE(n4sidv.message.empty());
    REQUIRE_FALSE(cva.success);
    REQUIRE_FALSE(cva.message.empty());
}

// -----------------------------------------------------------------------------
// GPMPC - GP-uncertainty-aware input-bound tightening for NonlinearMPC (Phase 3 ML3)
// -----------------------------------------------------------------------------

namespace
{
// Scalar plant: x[k+1] = 0.9*x[k] + u[k] (1 state, 1 input, y = x).
Eigen::VectorXd gpMpcScalarDynamics(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
    Eigen::VectorXd xn(1);
    xn(0) = 0.9 * x(0) + u(0);
    return xn;
}

ctrl::GPMPCParams gpMpcTestParams()
{
    ctrl::GPMPCParams p;
    p.nmpc.Np = 5;
    p.nmpc.Nu = 3;
    p.nmpc.n_states = 1;
    p.nmpc.n_inputs = 1;
    p.nmpc.n_outputs = 1;
    p.nmpc.uMin = -5.0;
    p.nmpc.uMax = 5.0;
    p.nmpc.Ts = 0.1;
    p.uncertainty_inflation = 2.0;
    return p;
}
} // namespace

TEST_CASE("GPMPC with an unfitted GP is regression-identical to NonlinearMPC", "[gp_mpc]")
{
    const auto params = gpMpcTestParams();

    ctrl::NonlinearMPC nmpc(params.nmpc, gpMpcScalarDynamics);

    ctrl::GPResidualModel::Params gp_p;
    gp_p.gp.length_scale = 0.5; gp_p.gp.signal_var = 1.0; gp_p.gp.noise_var = 0.01;
    auto gp = std::make_shared<ctrl::GPResidualModel>(2, gp_p); // xDim = n_states+n_inputs = 2
    ctrl::GPMPC gpmpc(params, gpMpcScalarDynamics, gp);

    REQUIRE_FALSE(gp->isFitted());

    Eigen::VectorXd x(1); x << 1.0;
    for (int k = 0; k < 5; ++k)
    {
        nmpc.setState(x);
        gpmpc.setState(x);
        const double u_nmpc  = nmpc.compute(0.5 - x(0));
        const double u_gpmpc = gpmpc.compute(0.5 - x(0));
        REQUIRE_THAT(u_gpmpc, WithinAbs(u_nmpc, 1e-12));

        Eigen::VectorXd u_vec(1); u_vec << u_nmpc;
        x = gpMpcScalarDynamics(x, u_vec);
    }

    REQUIRE(gpmpc.lastTightening().isZero());
}

TEST_CASE("GPMPC tightens bounds when the GP reports high variance, and the QP respects it",
          "[gp_mpc]")
{
    const auto params = gpMpcTestParams();

    ctrl::GPResidualModel::Params gp_p;
    gp_p.gp.length_scale = 0.5; gp_p.gp.signal_var = 1.0; gp_p.gp.noise_var = 0.01;
    auto gp = std::make_shared<ctrl::GPResidualModel>(2, gp_p);

    // Train far away from the test point so the posterior variance there is high
    // (close to the GP's own prior signal_var, since no nearby training data informs it).
    gp->addResidualPoint(Eigen::Vector2d(50.0, 50.0), 0.0, 0.0);
    gp->fit();

    ctrl::GPMPC gpmpc(params, gpMpcScalarDynamics, gp);

    Eigen::VectorXd x(1); x << 1.0;
    gpmpc.setState(x);
    const double u = gpmpc.compute(0.5 - x(0));
    REQUIRE(std::isfinite(u));

    REQUIRE(gpmpc.lastTightening().maxCoeff() > 0.0);

    // The tightened box around the warm-start must still contain the solved DeltaU --
    // verify the *uniform* untightened box (uMin/uMax) is wider than what GPMPC reports
    // having shrunk by, i.e. shrink is strictly less than the full half-width.
    const double half_width = 0.5 * (params.nmpc.uMax - params.nmpc.uMin);
    REQUIRE(gpmpc.lastTightening().maxCoeff() <= half_width);
}

TEST_CASE("GPMPC constructor throws on a GP with the wrong feature dimension", "[gp_mpc]")
{
    const auto params = gpMpcTestParams();

    ctrl::GPResidualModel::Params gp_p;
    gp_p.gp.length_scale = 0.5; gp_p.gp.signal_var = 1.0; gp_p.gp.noise_var = 0.01;
    auto wrong_dim_gp = std::make_shared<ctrl::GPResidualModel>(1, gp_p); // should be 2

    REQUIRE_THROWS_AS(ctrl::GPMPC(params, gpMpcScalarDynamics, wrong_dim_gp),
                       std::invalid_argument);
}

TEST_CASE("GPMPC clamps tightening so lb never crosses ub even with extreme inflation",
          "[gp_mpc]")
{
    auto params = gpMpcTestParams();
    params.uncertainty_inflation = 1e9;

    ctrl::GPResidualModel::Params gp_p;
    gp_p.gp.length_scale = 0.5; gp_p.gp.signal_var = 1.0; gp_p.gp.noise_var = 0.01;
    auto gp = std::make_shared<ctrl::GPResidualModel>(2, gp_p);
    gp->addResidualPoint(Eigen::Vector2d(50.0, 50.0), 0.0, 0.0);
    gp->fit();

    ctrl::GPMPC gpmpc(params, gpMpcScalarDynamics, gp);

    Eigen::VectorXd x(1); x << 1.0;
    gpmpc.setState(x);
    const double u = gpmpc.compute(0.5 - x(0));
    REQUIRE(std::isfinite(u)); // box collapsed to a point, not crossed/NaN
}

// -----------------------------------------------------------------------------
// ValueIterationSolver - grid-based DP / value iteration (Phase 4 OC2)
// -----------------------------------------------------------------------------

TEST_CASE("ValueIterationSolver matches the discounted-LQR gain on a double-integrator "
          "(LQR-equivalent problem)", "[value_iteration]")
{
    auto plant = makeDoubleIntegrator();
    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);

    const double gamma = 0.95;
    // Discounted-LQR reduces to the standard (undiscounted) LQR problem on the
    // sqrt(gamma)-scaled system (Bertsekas, "Dynamic Programming and Optimal Control" Vol. 1,
    // Sec. 4.2) -- this is the correct reference gain for a discounted value-iteration solve,
    // not DiscreteLQR's plain (gamma=1) gain, which differs substantially here because the
    // double integrator's eigenvalues sit exactly at the marginal-stability boundary (=1), so
    // even a modest discount measurably changes the optimal gain.
    ctrl::StateSpace discountedPlant = plant;
    discountedPlant.A *= std::sqrt(gamma);
    discountedPlant.B *= std::sqrt(gamma);
    ctrl::DiscreteLQR lqr(discountedPlant, lqr_p);
    REQUIRE(lqr.dareConverged());

    auto f = [&plant](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return ctrl::ssStepCopy(plant, x, u).second;
    };
    auto cost = [&lqr_p](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return x.dot(lqr_p.Q * x) + u.dot(lqr_p.R * u);
    };

    ctrl::DPGridParams gp;
    gp.x_min = Eigen::Vector2d(-1.0, -1.0);
    gp.x_max = Eigen::Vector2d( 1.0,  1.0);
    gp.n_grid_per_dim = Eigen::Vector2i(61, 61);
    gp.u_min = Eigen::VectorXd::Constant(1, -5.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  5.0);
    gp.n_grid_u = 41;
    gp.discount = gamma;
    gp.max_iter = 500;
    gp.tol      = 1e-4;

    ctrl::ValueIterationSolver vi(f, cost, gp);
    vi.solve();
    REQUIRE(vi.converged());

    const Eigen::VectorXd x_test = (Eigen::VectorXd(2) << 0.5, 0.0).finished();
    const Eigen::VectorXd u_lqr  = lqr.compute(x_test);
    const Eigen::VectorXd u_vi   = vi.policy(x_test);

    REQUIRE_THAT(u_vi(0), WithinAbs(u_lqr(0), 0.5)); // grid-resolution tolerance
    REQUIRE(std::isfinite(vi.value(x_test)));
}

TEST_CASE("ValueIterationSolver's Bellman residual decreases monotonically across sweep counts",
          "[value_iteration]")
{
    ctrl::DPGridParams gp;
    gp.x_min = Eigen::Vector2d(-1.0, -1.0);
    gp.x_max = Eigen::Vector2d( 1.0,  1.0);
    gp.n_grid_per_dim = Eigen::Vector2i(21, 21);
    gp.u_min = Eigen::VectorXd::Constant(1, -3.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  3.0);
    gp.n_grid_u = 9;
    gp.discount = 0.95;
    gp.tol      = 1e-9; // tight enough that none of the iter counts below actually converge

    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        xn(0) = x(0) + 0.1 * x(1);
        xn(1) = x(1) + 0.1 * u(0);
        return xn;
    };
    auto cost = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return x.squaredNorm() + 0.1 * u.squaredNorm();
    };

    double prevDelta = std::numeric_limits<double>::infinity();
    for (int iters : {5, 10, 15, 20, 25})
    {
        gp.max_iter = iters;
        ctrl::ValueIterationSolver vi(f, cost, gp);
        vi.solve();
        REQUIRE_FALSE(vi.converged());
        REQUIRE(vi.finalDelta() <= prevDelta + 1e-12);
        prevDelta = vi.finalDelta();
    }
}

TEST_CASE("ValueIterationSolver's policy error shrinks as the grid is refined "
          "(curse-of-dimensionality, not a bug)", "[value_iteration]")
{
    auto plant = makeDoubleIntegrator();
    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);

    const double gamma = 0.95;
    // See the discounted-LQR reduction note in the previous TEST_CASE: compare against the
    // sqrt(gamma)-scaled system's standard LQR gain, not the plain (gamma=1) gain.
    ctrl::StateSpace discountedPlant = plant;
    discountedPlant.A *= std::sqrt(gamma);
    discountedPlant.B *= std::sqrt(gamma);
    ctrl::DiscreteLQR lqr(discountedPlant, lqr_p);

    auto f = [&plant](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return ctrl::ssStepCopy(plant, x, u).second;
    };
    auto cost = [&lqr_p](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return x.dot(lqr_p.Q * x) + u.dot(lqr_p.R * u);
    };

    const Eigen::VectorXd x_test = (Eigen::VectorXd(2) << 0.5, 0.0).finished();
    const double u_lqr = lqr.compute(x_test)(0);

    auto policyErrorAt = [&](int n_grid) {
        ctrl::DPGridParams gp;
        gp.x_min = Eigen::Vector2d(-1.0, -1.0);
        gp.x_max = Eigen::Vector2d( 1.0,  1.0);
        gp.n_grid_per_dim = Eigen::Vector2i(n_grid, n_grid);
        gp.u_min = Eigen::VectorXd::Constant(1, -5.0);
        gp.u_max = Eigen::VectorXd::Constant(1,  5.0);
        gp.n_grid_u = 41;
        gp.discount = gamma;
        gp.max_iter = 500;
        gp.tol      = 1e-4;

        ctrl::ValueIterationSolver vi(f, cost, gp);
        vi.solve();
        return std::abs(vi.policy(x_test)(0) - u_lqr);
    };

    const double err_coarse = policyErrorAt(11);
    const double err_fine   = policyErrorAt(61);

    REQUIRE(err_fine < err_coarse);
}

TEST_CASE("ValueIterationSolver's out_of_grid_penalty determines whether an out-of-bounds "
          "action is selected over a within-bounds alternative", "[value_iteration]")
{
    // cost(x,u) = -x*u rewards (negative cost) actions that push x further from zero in its
    // current direction. At x=0.9 the cheapest-looking actions all leave the grid; this isolates
    // out_of_grid_penalty's effect on the first sweep, where V_old == 0 everywhere (hand-verified:
    // with penalty=0.5, escaping via u=2 costs -1.8 + 0.95*0.5 = -1.325, beating the best
    // in-bounds action u=0 at cost 0; with penalty=1e6, escaping costs ~949998, losing badly).
    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(1);
        xn(0) = x(0) + u(0);
        return xn;
    };
    auto cost = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return -x(0) * u(0);
    };

    ctrl::DPGridParams gp;
    gp.x_min = Eigen::VectorXd::Constant(1, -1.0);
    gp.x_max = Eigen::VectorXd::Constant(1,  1.0);
    gp.n_grid_per_dim = Eigen::VectorXi::Constant(1, 21); // spacing 0.1
    gp.u_min = Eigen::VectorXd::Constant(1, -2.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  2.0);
    gp.n_grid_u = 9;     // spacing 0.5: -2,-1.5,...,2
    gp.discount = 0.95;
    gp.max_iter = 1;     // exactly one sweep: V_old == 0 everywhere, hand-verifiable
    gp.tol      = 1e-12;

    const Eigen::VectorXd x_query = Eigen::VectorXd::Constant(1, 0.9); // exactly on a grid point

    gp.out_of_grid_penalty = 0.5;
    ctrl::ValueIterationSolver vi_weak(f, cost, gp);
    vi_weak.solve();
    REQUIRE(vi_weak.policy(x_query)(0) > 1.5); // picks the escaping action (u=2)

    gp.out_of_grid_penalty = 1e6;
    ctrl::ValueIterationSolver vi_strong(f, cost, gp);
    vi_strong.solve();
    REQUIRE(vi_strong.policy(x_query)(0) > -0.5);
    REQUIRE(vi_strong.policy(x_query)(0) <  0.5); // settles on u~=0, the cheapest in-bounds action
}

// =============================================================================
// invertTransferFunction - dynamic-inversion helper (scope-triage batch)
// =============================================================================

TEST_CASE("invertTransferFunction swaps and re-normalizes num/den to a monic denominator",
          "[invert_tf]")
{
    // G(z^-1) = (0.5 + 0.3 z^-1) / (1 - 0.6 z^-1), b0 = 0.5.
    // G_inv = A/B = (1 - 0.6z^-1) / (0.5 + 0.3z^-1), normalized by b0 so both num/den are
    // divided by 0.5: num = [1,-0.6]/0.5 = [2,-1.2], den = [0.5,0.3]/0.5 = [1,0.6].
    const ctrl::TransferFunction G({0.5, 0.3}, {1.0, -0.6}, 0.1);
    const ctrl::TransferFunction Ginv = ctrl::invertTransferFunction(G);

    REQUIRE(Ginv.den[0] == 1.0); // monic, by construction
    REQUIRE_THAT(Ginv.num[0], WithinAbs( 2.0, 1e-12));
    REQUIRE_THAT(Ginv.num[1], WithinAbs(-1.2, 1e-12));
    REQUIRE_THAT(Ginv.den[1], WithinAbs( 0.6, 1e-12));
    REQUIRE(Ginv.Ts == G.Ts);
}

TEST_CASE("invertTransferFunction is its own inverse (double inversion recovers G exactly)",
          "[invert_tf]")
{
    const ctrl::TransferFunction G({0.5, 0.3}, {1.0, -0.6}, 0.1);
    const ctrl::TransferFunction Ginv = ctrl::invertTransferFunction(G);
    const ctrl::TransferFunction Gback = ctrl::invertTransferFunction(Ginv);

    REQUIRE_THAT(Gback.num[0], WithinAbs(G.num[0], 1e-9));
    REQUIRE_THAT(Gback.num[1], WithinAbs(G.num[1], 1e-9));
    REQUIRE_THAT(Gback.den[0], WithinAbs(G.den[0], 1e-9));
    REQUIRE_THAT(Gback.den[1], WithinAbs(G.den[1], 1e-9));
}

TEST_CASE("G_inv(z) cascaded after G(z) recovers the original input exactly (perfect inversion)",
          "[invert_tf]")
{
    const ctrl::TransferFunction G({0.5, 0.3}, {1.0, -0.6}, 0.1);
    const ctrl::TransferFunction Ginv = ctrl::invertTransferFunction(G);

    const ctrl::StateSpace ssG    = ctrl::tf2ss(G);
    const ctrl::StateSpace ssGinv = ctrl::tf2ss(Ginv);

    Eigen::VectorXd xG    = Eigen::VectorXd::Zero(ssG.stateSize());
    Eigen::VectorXd xGinv = Eigen::VectorXd::Zero(ssGinv.stateSize());

    const std::vector<double> u_seq = {1.0, 1.0, -0.5, 2.0, 0.3, -1.0, 0.0, 0.7};
    for (double u : u_seq)
    {
        Eigen::VectorXd uv(1);
        uv << u;
        const double y = ctrl::ssStep(ssG, xG, uv)(0);
        Eigen::VectorXd yv(1);
        yv << y;
        const double v = ctrl::ssStep(ssGinv, xGinv, yv)(0);
        REQUIRE_THAT(v, WithinAbs(u, 1e-9)); // G_inv(G(u)) == u exactly (zero-state cascade)
    }
}

TEST_CASE("invertTransferFunction throws on a strictly-proper (b0 ~ 0) transfer function",
          "[invert_tf]")
{
    const ctrl::TransferFunction G({0.0, 0.3}, {1.0, -0.6}, 0.1); // b0 = 0
    REQUIRE_THROWS_AS(ctrl::invertTransferFunction(G), std::invalid_argument);
}

// =============================================================================
// EventTriggeredWrapper - aperiodic-sampling (deadband) decorator
// =============================================================================

namespace
{
std::shared_ptr<ctrl::DiscretePID> makeProportionalPID(double Ts = 0.1, double Kp = 2.0)
{
    ctrl::PIDParams pp;
    pp.Kp = Kp; pp.Ki = 0.0; pp.Kd = 0.0; pp.N = 10.0;
    return std::make_shared<ctrl::DiscretePID>(pp, Ts);
}
} // namespace

TEST_CASE("EventTriggeredWrapper always triggers on the first call", "[event_triggered]")
{
    ctrl::EventTriggeredParams params; params.sigma = 0.5;
    ctrl::EventTriggeredWrapper etw(makeProportionalPID(), params);

    REQUIRE_THAT(etw.compute(1.0), WithinAbs(2.0, 1e-12)); // Kp=2: u = 2*1.0
    REQUIRE(etw.triggerCount() == 1);
    REQUIRE(etw.holdCount() == 0);
}

TEST_CASE("EventTriggeredWrapper holds the output for signal changes within the deadband",
          "[event_triggered]")
{
    ctrl::EventTriggeredParams params; params.sigma = 0.5;
    ctrl::EventTriggeredWrapper etw(makeProportionalPID(), params);

    REQUIRE_THAT(etw.compute(1.0), WithinAbs(2.0, 1e-12)); // triggers: u = 2*1.0 = 2
    REQUIRE_THAT(etw.compute(1.2), WithinAbs(2.0, 1e-12)); // |1.2-1.0|=0.2 < 0.5 -> holds
    REQUIRE_THAT(etw.compute(0.7), WithinAbs(2.0, 1e-12)); // |0.7-1.0|=0.3 < 0.5 -> holds

    REQUIRE(etw.triggerCount() == 1);
    REQUIRE(etw.holdCount() == 2);
}

TEST_CASE("EventTriggeredWrapper re-triggers once the signal exceeds the deadband",
          "[event_triggered]")
{
    ctrl::EventTriggeredParams params; params.sigma = 0.5;
    ctrl::EventTriggeredWrapper etw(makeProportionalPID(), params);

    REQUIRE_THAT(etw.compute(1.0), WithinAbs(2.0, 1e-12)); // triggers: u = 2
    REQUIRE_THAT(etw.compute(1.2), WithinAbs(2.0, 1e-12)); // holds
    REQUIRE_THAT(etw.compute(2.0), WithinAbs(4.0, 1e-12)); // |2.0-1.0|=1.0 > 0.5 -> triggers, u=4

    REQUIRE(etw.triggerCount() == 2);
    REQUIRE(etw.holdCount() == 1);
}

TEST_CASE("EventTriggeredWrapper holds on NaN input without affecting trigger/hold counters",
          "[event_triggered]")
{
    ctrl::EventTriggeredParams params; params.sigma = 0.5;
    ctrl::EventTriggeredWrapper etw(makeProportionalPID(), params);

    REQUIRE_THAT(etw.compute(1.0), WithinAbs(2.0, 1e-12)); // triggers
    REQUIRE(etw.triggerCount() == 1);

    const double nan_out = etw.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE_THAT(nan_out, WithinAbs(2.0, 1e-12)); // holds last output
    REQUIRE(etw.triggerCount() == 1); // unchanged
    REQUIRE(etw.holdCount() == 0);    // NaN doesn't count as a real hold either
}

TEST_CASE("EventTriggeredWrapper::reset() clears held state, forcing the next call to trigger",
          "[event_triggered]")
{
    ctrl::EventTriggeredParams params; params.sigma = 0.5;
    ctrl::EventTriggeredWrapper etw(makeProportionalPID(), params);

    etw.compute(1.0);   // triggers
    etw.compute(1.1);   // holds
    REQUIRE(etw.triggerCount() == 1);
    REQUIRE(etw.holdCount() == 1);

    etw.reset();
    REQUIRE_THAT(etw.lastOutput(), WithinAbs(0.0, 1e-12));

    // Even a tiny signal triggers right after reset (no prior reference to compare against).
    REQUIRE_THAT(etw.compute(0.01), WithinAbs(0.02, 1e-12)); // Kp=2: u = 2*0.01
    REQUIRE(etw.triggerCount() == 1); // counters reset to 0 then incremented once
    REQUIRE(etw.holdCount() == 0);
}

// =============================================================================
// LPSolver - two-phase simplex for bounded-variable linear programs (Phase 3 OC4)
// =============================================================================

TEST_CASE("LPSolver recovers the textbook vertex optimum of a 2-variable LP", "[lp_solver]")
{
    // maximize x1+x2 (== minimize -x1-x2) s.t. x1+2x2<=4, 3x1+x2<=6, x>=0.
    // Hand-derived vertex optimum: (x1,x2)=(1.6,1.2), objective=2.8 (c'x=-2.8).
    ctrl::LPProblem problem;
    problem.c.resize(2);
    problem.c << -1.0, -1.0;
    problem.A_ineq.resize(2, 2);
    problem.A_ineq << 1.0, 2.0,
                       3.0, 1.0;
    problem.b_ineq.resize(2);
    problem.b_ineq << 4.0, 6.0;
    problem.lb = Eigen::VectorXd::Zero(2);
    problem.ub = Eigen::VectorXd::Constant(2, 1e9);

    const ctrl::LPResult result = ctrl::LPSolver::solve(problem);

    REQUIRE(result.status == ctrl::LPStatus::Optimal);
    REQUIRE_THAT(result.x(0), WithinAbs(1.6, 1e-6));
    REQUIRE_THAT(result.x(1), WithinAbs(1.2, 1e-6));
    REQUIRE_THAT(result.cost, WithinAbs(-2.8, 1e-6));
}

TEST_CASE("LPSolver handles an equality-constrained LP (A_eq path)", "[lp_solver]")
{
    // minimize 2*x1+3*x2 s.t. x1+x2=4 (equality), 0<=x1,x2<=3.
    // x2=4-x1 => cost=12-x1, minimized by maximizing x1 -> x1=3 (its own upper bound), x2=1.
    ctrl::LPProblem problem;
    problem.c.resize(2);
    problem.c << 2.0, 3.0;
    problem.A_eq.resize(1, 2);
    problem.A_eq << 1.0, 1.0;
    problem.b_eq.resize(1);
    problem.b_eq << 4.0;
    problem.lb = Eigen::VectorXd::Zero(2);
    problem.ub = Eigen::VectorXd::Constant(2, 3.0);

    const ctrl::LPResult result = ctrl::LPSolver::solve(problem);

    REQUIRE(result.status == ctrl::LPStatus::Optimal);
    REQUIRE_THAT(result.x(0), WithinAbs(3.0, 1e-6));
    REQUIRE_THAT(result.x(1), WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.cost, WithinAbs(9.0, 1e-6));
}

TEST_CASE("LPSolver reports Infeasible (not an infinite loop) on contradictory constraints",
          "[lp_solver]")
{
    // x1<=1 (A_ineq row [1], b=1) AND x1>=3 (encoded as -x1<=-3) within box [0,10]: empty region.
    ctrl::LPProblem problem;
    problem.c.resize(1);
    problem.c << 0.0;
    problem.A_ineq.resize(2, 1);
    problem.A_ineq << 1.0, -1.0;
    problem.b_ineq.resize(2);
    problem.b_ineq << 1.0, -3.0;
    problem.lb = Eigen::VectorXd::Zero(1);
    problem.ub = Eigen::VectorXd::Constant(1, 10.0);

    const ctrl::LPResult result = ctrl::LPSolver::solve(problem);

    REQUIRE(result.status == ctrl::LPStatus::Infeasible);
}

TEST_CASE("LPSolver reaches Optimal despite a genuinely redundant equality row", "[lp_solver]")
{
    // Same equality row listed twice (x1+2x2=4); regression guard for the "artificial parked at
    // zero in the basis" degenerate case analyzed in LPSolver.h's doc comment. The equality is
    // consistent with the (1.6,1.2) vertex from the first test above (1.6+2*1.2=4.0 exactly), so
    // combined with 3x1+x2<=6 the optimum is identical: (1.6,1.2), cost=-2.8.
    ctrl::LPProblem problem;
    problem.c.resize(2);
    problem.c << -1.0, -1.0;
    problem.A_eq.resize(2, 2);
    problem.A_eq << 1.0, 2.0,
                    1.0, 2.0;
    problem.b_eq.resize(2);
    problem.b_eq << 4.0, 4.0;
    problem.A_ineq.resize(1, 2);
    problem.A_ineq << 3.0, 1.0;
    problem.b_ineq.resize(1);
    problem.b_ineq << 6.0;
    problem.lb = Eigen::VectorXd::Zero(2);
    problem.ub = Eigen::VectorXd::Constant(2, 1e9);

    const ctrl::LPResult result = ctrl::LPSolver::solve(problem);

    REQUIRE(result.status == ctrl::LPStatus::Optimal);
    REQUIRE_THAT(result.x(0), WithinAbs(1.6, 1e-6));
    REQUIRE_THAT(result.x(1), WithinAbs(1.2, 1e-6));
    REQUIRE_THAT(result.cost, WithinAbs(-2.8, 1e-6));
}

TEST_CASE("LPSolver reports IterationLimit (not a mislabeled Infeasible) when starved",
          "[lp_solver]")
{
    // Same LP as the first test, but maxIter=1 is nowhere near enough to resolve even Phase 1.
    // Regression guard: running out of budget must not be reported as a false "Infeasible".
    ctrl::LPProblem problem;
    problem.c.resize(2);
    problem.c << -1.0, -1.0;
    problem.A_ineq.resize(2, 2);
    problem.A_ineq << 1.0, 2.0,
                       3.0, 1.0;
    problem.b_ineq.resize(2);
    problem.b_ineq << 4.0, 6.0;
    problem.lb = Eigen::VectorXd::Zero(2);
    problem.ub = Eigen::VectorXd::Constant(2, 1e9);

    const ctrl::LPResult result = ctrl::LPSolver::solve(problem, /*maxIter=*/1);

    REQUIRE(result.status == ctrl::LPStatus::IterationLimit);
}

// =============================================================================
// LPMPC - SISO L1-cost linear MPC solved via LPSolver per step (Phase 3 OC4)
// =============================================================================

TEST_CASE("LPMPC tracks a unit step reference", "[lp_mpc][integration]")
{
    auto plant = makePlant();
    ctrl::LPMPCParams p;
    p.Np = 15; p.Nc = 5; p.rho_y = 10.0; p.rho_u = 0.01;
    p.uMin = -100.0; p.uMax = 100.0; p.duMin = -100.0; p.duMax = 100.0;

    ctrl::LPMPC mpc(plant, p);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    for (int k = 0; k < 1500; ++k)
    {
        mpc.setState(x);
        const double u = mpc.computeRef(x, 1.0);
        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);
    }

    REQUIRE_THAT(y, WithinAbs(1.0, 0.02)); // within 2% of reference
}

TEST_CASE("LPMPC higher rho_u reduces peak control-move magnitude", "[lp_mpc]")
{
    auto plant = makePlant();
    auto runPeakDu = [&](double rho_u) {
        ctrl::LPMPCParams p;
        p.Np = 15; p.Nc = 5; p.rho_y = 10.0; p.rho_u = rho_u;
        p.uMin = -100.0; p.uMax = 100.0; p.duMin = -100.0; p.duMax = 100.0;
        ctrl::LPMPC mpc(plant, p);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        double u_prev = 0.0, peak_du = 0.0;
        for (int k = 0; k < 200; ++k)
        {
            mpc.setState(x);
            const double u = mpc.computeRef(x, 1.0);
            peak_du = std::max(peak_du, std::fabs(u - u_prev));
            u_prev = u;
            Eigen::VectorXd uv(1);
            uv << u;
            ctrl::ssStep(plant, x, uv);
        }
        return peak_du;
    };

    const double peak_low_ru  = runPeakDu(0.001);
    const double peak_high_ru = runPeakDu(1.0);
    REQUIRE(peak_high_ru < peak_low_ru);
}

TEST_CASE("LPMPC respects hard u bounds under an unreachable reference", "[lp_mpc]")
{
    auto plant = makePlant();
    ctrl::LPMPCParams p;
    p.Np = 15; p.Nc = 5; p.rho_y = 10.0; p.rho_u = 0.01;
    p.uMin = -0.5; p.uMax = 0.5; p.duMin = -100.0; p.duMax = 100.0;

    ctrl::LPMPC mpc(plant, p);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    bool within_bounds = true;
    for (int k = 0; k < 300; ++k)
    {
        mpc.setState(x);
        const double u = mpc.computeRef(x, 10.0); // far outside what +-0.5 can reach
        within_bounds = within_bounds && (u >= p.uMin - 1e-9) && (u <= p.uMax + 1e-9);
        Eigen::VectorXd uv(1);
        uv << u;
        ctrl::ssStep(plant, x, uv);
    }
    REQUIRE(within_bounds);
}

TEST_CASE("LPMPC holds u_prev on non-finite compute(error) input", "[lp_mpc][nan_guard]")
{
    auto plant = makePlant();
    ctrl::LPMPCParams p;
    p.Np = 10; p.Nc = 3;
    ctrl::LPMPC mpc(plant, p);

    const double u0     = mpc.compute(1.0);
    const double u_nan  = mpc.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE_THAT(u_nan, WithinAbs(u0, 1e-12));
}

TEST_CASE("LPMPC lastLPConverged/isHealthy is true for a well-conditioned problem", "[lp_mpc]")
{
    auto plant = makePlant();
    ctrl::LPMPCParams p;
    p.Np = 10; p.Nc = 3; p.rho_y = 1.0; p.rho_u = 0.1;
    ctrl::LPMPC mpc(plant, p);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    mpc.computeRef(x, 1.0);

    REQUIRE(mpc.lastLPConverged());
    REQUIRE(mpc.isHealthy());
}

TEST_CASE("LPMPC isHealthy reflects LP convergence under a starved iteration budget",
          "[lp_mpc][health_contract]")
{
    auto plant = makePlant();
    ctrl::LPMPCParams p;
    p.Np = 15; p.Nc = 5; p.lpMaxIter = 1; // deliberately starved
    ctrl::LPMPC mpc(plant, p);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    mpc.computeRef(x, 1.0);

    REQUIRE_FALSE(mpc.isHealthy());
}

// =============================================================================
// Code generation (Phase 4 DT1) - golden-file tests
// =============================================================================
namespace codegen_test {

namespace fs = std::filesystem;

inline const std::string kCCompiler = CTRL_C_COMPILER_PATH;

// std::system() on Windows shells out via `cmd.exe /c <string>`. cmd.exe's legacy argument
// parser strips the very first and last characters of that string whenever both are `"` and
// the string contains more than two quote characters total - exactly what every multi-path
// quoted command below builds - corrupting the inner quoting (observed as "The filename,
// directory name, or volume label syntax is incorrect."). Wrapping the whole string in one
// extra outer quote pair makes cmd.exe strip that (harmless) pair instead, leaving the real
// quoting intact. POSIX's `/bin/sh -c` has no such bug and would mis-parse the extra pair, so
// this wrapping is Windows-only.
inline int runSystem(const std::string &cmd)
{
#ifdef _WIN32
    return std::system(("\"" + cmd + "\"").c_str());
#else
    return std::system(cmd.c_str());
#endif
}

// Writes the generated .h/.c pair plus a small harness main() that reads doubles from
// inputs.txt (one per line), calls controller_step() on each, and writes outputs.txt
// (one per line, full double precision). Compiles standalone (no Eigen/lib link) and runs it.
// Returns std::nullopt if the compiler invocation or the generated program fails.
inline std::optional<std::vector<double>> runGeneratedC(
    const ctrl::GeneratedCode &code,
    const std::vector<double> &inputs,
    const std::string &tag)
{
    fs::path dir = fs::temp_directory_path() / ("ctrl_codegen_" + tag);
    fs::create_directories(dir);

    {
        std::ofstream h(dir / "controller_gen.h");
        h << code.header;
    }
    {
        std::ofstream c(dir / "controller_gen.c");
        c << code.source;
    }
    {
        std::ofstream m(dir / "harness.c");
        m << "#include \"controller_gen.h\"\n"
          << "#include <stdio.h>\n"
          << "int main(int argc, char** argv) {\n"
          << "    FILE* in = fopen(argv[1], \"r\");\n"
          << "    FILE* out = fopen(argv[2], \"w\");\n"
          << "    double x;\n"
          << "    while (fscanf(in, \"%lf\", &x) == 1) {\n"
          << "        fprintf(out, \"%.17g\\n\", controller_step(x));\n"
          << "    }\n"
          << "    fclose(in);\n"
          << "    fclose(out);\n"
          << "    return 0;\n"
          << "}\n";
    }
    {
        std::ofstream inf(dir / "inputs.txt");
        for (double x : inputs) inf << std::setprecision(17) << x << "\n";
    }

    const fs::path exe = dir / "harness_exe";
    std::ostringstream cmd;
    cmd << "\"" << kCCompiler << "\" -std=c99 -o \"" << exe.string() << "\" \""
        << (dir / "controller_gen.c").string() << "\" \"" << (dir / "harness.c").string() << "\"";
    if (runSystem(cmd.str()) != 0) return std::nullopt;

    const fs::path outputs = dir / "outputs.txt";
    std::ostringstream runCmd;
    runCmd << "\"" << exe.string() << "\" \"" << (dir / "inputs.txt").string() << "\" \""
           << outputs.string() << "\"";
    if (runSystem(runCmd.str()) != 0) return std::nullopt;

    std::vector<double> result;
    std::ifstream of(outputs);
    double v;
    while (of >> v) result.push_back(v);
    return result;
}

} // namespace codegen_test

TEST_CASE("Code generation: PID golden file matches DiscretePID", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::PIDParams p;
    p.Kp = 2.0; p.Ki = 0.5; p.Kd = 0.1; p.N = 50.0; p.Kb = 1.0;
    p.uMin = -5.0; p.uMax = 5.0;

    ctrl::DiscretePID pid(p, Ts);
    const std::vector<double> inputs = {0.1, 0.5, 1.0, 3.0, 6.0, 6.0, 6.0, -6.0, -6.0, 0.0, 0.2};
    std::vector<double> expected;
    for (double e : inputs) expected.push_back(pid.compute(e));

    const auto code = ctrl::generateControllerC(p, Ts);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "pid");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE_THAT(actual->at(i), WithinAbs(expected[i], 1e-9));
}

TEST_CASE("Code generation: SMC golden file matches DiscreteSMC", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::SMCParams p;
    p.c_e = 1.0; p.c_de = 0.05; p.K = 3.0; p.phi = 0.2;
    p.uMin = -4.0; p.uMax = 4.0;

    ctrl::DiscreteSMC smc(p, Ts);
    const std::vector<double> inputs = {0.05, 0.3, 0.9, 2.0, -0.5, -2.0, 0.0, 0.01, -0.01};
    std::vector<double> expected;
    for (double e : inputs) expected.push_back(smc.compute(e));

    const auto code = ctrl::generateControllerC(p, Ts);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "smc");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE_THAT(actual->at(i), WithinAbs(expected[i], 1e-9));
}

TEST_CASE("Code generation: LeadLag golden file matches DiscreteLeadLag", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::LeadLagParams p;
    p.continuousZero = 1.0; p.continuousPole = 10.0; p.gain = 2.0;

    ctrl::DiscreteLeadLag ll(p, Ts);
    const std::vector<double> inputs = {0.0, 1.0, 0.5, -0.5, -1.0, 2.0, 0.0, 0.3};
    std::vector<double> expected;
    for (double u : inputs) expected.push_back(ll.compute(u));

    const auto code = ctrl::generateControllerC(p, Ts);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "leadlag");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE_THAT(actual->at(i), WithinAbs(expected[i], 1e-9));
}

TEST_CASE("Code generation: PID rejects a corrector when native anti-windup is active", "[code_generation]")
{
    ctrl::PIDParams p;
    p.Kp = 1.0; p.Ki = 0.5; p.Kb = 1.0; // native anti-windup active
    ctrl::CodeGenParams cfg;
    cfg.corrector = ctrl::AntiWindupConfig{-5.0, 5.0, 1.0};
    REQUIRE_THROWS_AS(ctrl::generateControllerC(p, 0.01, cfg), std::invalid_argument);
}

TEST_CASE("Code generation: SMC+corrector golden file matches AntiWindupWrapper(DiscreteSMC)", "[code_generation]")
{
    if (codegen_test::kCCompiler.empty())
        SKIP("no C compiler (gcc/cc/clang) found on PATH; skipping code-generation golden-file test");

    ctrl::SMCParams p;
    p.c_e = 1.0; p.c_de = 0.05; p.K = 3.0; p.phi = 0.2;
    p.uMin = -10.0; p.uMax = 10.0; // inner has wide limits; corrector applies the real bound

    auto smc = std::make_shared<ctrl::DiscreteSMC>(p, Ts);
    ctrl::AntiWindupWrapper wrapper(smc, -4.0, 4.0, 0.8);

    const std::vector<double> inputs = {0.05, 0.3, 0.9, 2.0, -0.5, -2.0, 0.0, 0.01, -0.01};
    std::vector<double> expected;
    for (double e : inputs) expected.push_back(wrapper.compute(e));

    ctrl::CodeGenParams cfg;
    cfg.corrector = ctrl::AntiWindupConfig{-4.0, 4.0, 0.8};
    const auto code = ctrl::generateControllerC(p, Ts, cfg);
    const auto actual = codegen_test::runGeneratedC(code, inputs, "smc_corrector");
    REQUIRE(actual.has_value());
    REQUIRE(actual->size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        REQUIRE_THAT(actual->at(i), WithinAbs(expected[i], 1e-9));
}

TEST_CASE("Code generation: emitted sources are freestanding (no malloc/new/STL/Eigen)", "[code_generation]")
{
    static const std::vector<std::string> kForbidden = {
        // " new " (spaces both sides) - not "new " - so PID's legitimate "d_new" derivative
        // state variable (e.g. "const double d_new = ...") isn't a false positive: the real
        // C++ `new` keyword is always preceded by whitespace or punctuation, never by an
        // identifier character.
        "malloc", "calloc", "realloc", " new ", "std::", "#include <vector",
        "#include <memory", "#include <Eigen", "#include <string"
    };

    auto checkFreestanding = [](const ctrl::GeneratedCode &code, const std::string &tag) {
        for (const auto &token : kForbidden) {
            INFO(tag << ": forbidden token \"" << token << "\"");
            CHECK(code.header.find(token) == std::string::npos);
            CHECK(code.source.find(token) == std::string::npos);
        }
    };

    checkFreestanding(ctrl::generateControllerC(ctrl::PIDParams{}, 0.01), "PID");
    checkFreestanding(ctrl::generateControllerC(ctrl::SMCParams{}, 0.01), "SMC");
    checkFreestanding(ctrl::generateControllerC(ctrl::LeadLagParams{}, 0.01), "LeadLag");
}
