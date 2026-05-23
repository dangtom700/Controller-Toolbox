// ============================================================
//  test_controllers.cpp  -  Comprehensive controller test suite
//
//  Validates all controllers in lib/ against the example plant:
//    G(s) = 1 / (s^2 + 1.5s + 1),  ZOH at Ts = 0.01 s
//
//  Covers: normal operation, boundary conditions, and unexpected
//  input scenarios (NaN propagation, saturation, throws, resets,
//  degenerate parameters, edge-case API misuse).
//
//  Build: part of the controller_tests CMake target.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <stdexcept>
#include <string>
#include <limits>
#include <vector>
#include <memory>

#include "test_framework.h"

// ---------------------------------------------------------------------------
// Shared example plant:  G(s) = 1/(s^2+1.5s+1),  ZOH Ts = 0.01 s
// ---------------------------------------------------------------------------
static constexpr double Ts = 0.01;

ctrl::StateSpace make_plant()
{
    ctrl::TransferFunction tf(
        {0.0, 4.9625e-5, 4.9125e-5},
        {1.0, -1.98511, 0.98522},
        Ts);
    return ctrl::tf2ss(tf);
}

// Run N steps of closed-loop with a given controller; return final output.
double closed_loop(ctrl::IController &ctrl_obj,
                   ctrl::StateSpace &plant,
                   double ref,
                   int N)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    for (int k = 0; k < N; ++k)
    {
        double u = ctrl_obj.compute(ref - y);
        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);
    }
    return y;
}

// ============================================================
//  1. PlantModel tests
// ============================================================
void test_plant_model()
{
    test::suite("PlantModel");

    // Valid transfer function construction
    test::no_throw([]
                   { ctrl::TransferFunction tf({1.0}, {1.0, 1.0}, 0.01); }, "Valid TF construction");

    // Non-monic denominator (den[0] == 0) must throw
    test::throws([]
                 { ctrl::TransferFunction tf({1.0}, {0.0, 1.0}, 0.01); }, "TF with zero leading denominator");

    // Empty denominator must throw
    test::throws([]
                 { ctrl::TransferFunction tf({1.0}, {}, 0.01); }, "TF with empty denominator");

    // tf2ss produces correct state dimension
    auto plant = make_plant();
    test::check(plant.stateSize() == 2, "Plant has 2 states");
    test::check(plant.inputSize() == 1, "Plant has 1 input");
    test::check(plant.outputSize() == 1, "Plant has 1 output");

    // DC gain approx = 1 for G(s)=1/(s^2+1.5s+1)
    auto Acl = Eigen::MatrixXd::Identity(2, 2) - plant.A;
    double dc = (plant.C * Acl.inverse() * plant.B + plant.D)(0, 0);
    test::check(std::abs(dc - 1.0) < 0.01, "DC gain approx 1.0");

    // ssStep: zero input, zero state -> zero output
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u(1);
    u << 0.0;
    auto y = ctrl::ssStep(plant, x, u);
    test::check(std::abs(y(0)) < 1e-12, "ssStep: zero input -> zero output");
}

// ============================================================
//  2. DiscretePID tests
// ============================================================
void test_pid()
{
    test::suite("DiscretePID");

    ctrl::PIDParams p;
    p.Kp = 2.0;
    p.Ki = 1.0;
    p.Kd = 0.1;
    p.N = 20.0;
    p.uMin = -10.0;
    p.uMax = 10.0;
    p.Kb = 1.0;

    ctrl::DiscretePID pid(p, Ts);

    // Zero error -> zero output (fresh controller)
    test::check(std::abs(pid.compute(0.0)) < 1e-12, "Zero error -> zero output");

    // Positive error -> positive output
    pid.reset();
    test::check(pid.compute(1.0) > 0.0, "Positive error -> positive output");

    // Saturation: large error should clamp to uMax
    pid.reset();
    double u_big = pid.compute(1e6);
    test::check(std::abs(u_big - p.uMax) < 1e-9, "Large positive error -> uMax");

    // Saturation: large negative error should clamp to uMin
    pid.reset();
    double u_neg = pid.compute(-1e6);
    test::check(std::abs(u_neg - p.uMin) < 1e-9, "Large negative error -> uMin");

    // Reset clears state: after reset, zero error gives zero output
    pid.compute(5.0); // build up integral
    pid.reset();
    test::check(std::abs(pid.compute(0.0)) < 1e-12, "reset() clears integral");

    // Closed-loop convergence to unit step reference
    {
        auto plant = make_plant();
        double y_final = closed_loop(pid, plant, 1.0, 2000);
        test::check(std::abs(y_final - 1.0) < 0.02, "PID closed-loop tracks unit step");
    }

    // NaN guard: NaN input must NOT corrupt state - controller holds last output
    {
        ctrl::DiscretePID pid2(p, Ts);
        pid2.compute(1.0); // prime with a valid step so u_prev_ != 0
        double u_nan = pid2.compute(std::numeric_limits<double>::quiet_NaN());
        test::check(std::isfinite(u_nan), "NaN input: output stays finite (safe hold)");
        // A second valid call must still work (integral must not be corrupted)
        double u_after = pid2.compute(0.0);
        test::check(std::isfinite(u_after), "NaN input: state not corrupted after recovery");
    }

    // Inf guard: Inf input must NOT corrupt state - controller holds last output
    {
        ctrl::DiscretePID pid3(p, Ts);
        double u_inf = pid3.compute(std::numeric_limits<double>::infinity());
        test::check(std::isfinite(u_inf), "Inf input: output stays finite (safe hold)");
    }

    // uMin == uMax -> every output equals that value
    {
        ctrl::PIDParams p2 = p;
        p2.uMin = p2.uMax = 3.0;
        ctrl::DiscretePID pid_flat(p2, Ts);
        test::check(std::abs(pid_flat.compute(1.0) - 3.0) < 1e-9,
                    "uMin == uMax: output is clamped constant");
    }

    // Pure P (Ki=Kd=0): output = Kp * error
    {
        ctrl::PIDParams pp;
        pp.Kp = 2.5;
        pp.Ki = 0.0;
        pp.Kd = 0.0;
        pp.uMin = -1e9;
        pp.uMax = 1e9;
        ctrl::DiscretePID pid_p(pp, Ts);
        test::check(std::abs(pid_p.compute(1.0) - 2.5) < 1e-9,
                    "Pure P: compute(1.0) == Kp");
    }

    // setParams hot-update doesn't crash
    {
        ctrl::PIDParams p3 = p;
        p3.Kp = 5.0;
        test::no_throw([&]
                       { pid.setParams(p3); }, "setParams hot-update");
    }
}

// ============================================================
//  3. DiscreteLQR tests
// ============================================================
void test_lqr()
{
    test::suite("DiscreteLQR");

    auto plant = make_plant();
    const int n = plant.stateSize();

    ctrl::LQRParams lqr_p;
    lqr_p.Q = Eigen::MatrixXd::Identity(n, n);
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1) * 0.04;

    // Normal construction (DARE must converge)
    ctrl::DiscreteLQR lqr(plant, lqr_p);
    test::check(lqr.gainMatrix().rows() == 1 && lqr.gainMatrix().cols() == n,
                "Gain matrix is 1 x n");

    // compute with zero state -> zero output
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd u = lqr.compute(x);
    test::check(std::abs(u(0)) < 1e-12, "LQR: zero state -> zero control");

    // compute with unit state -> nonzero output
    x(0) = 1.0;
    u = lqr.compute(x);
    test::check(u(0) != 0.0, "LQR: nonzero state -> nonzero control");

    // compute with x_ref equal to x -> zero error -> zero control
    u = lqr.compute(x, x);
    test::check(std::abs(u(0)) < 1e-12, "LQR: x == x_ref -> zero control");

    // Feedforward term shifts output
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd u_ff(1);
    u_ff << 2.0;
    u = lqr.compute(x0, Eigen::VectorXd(), u_ff);
    test::check(std::abs(u(0) - 2.0) < 1e-12, "LQR: feedforward offset applied");

    // Non-stabilizable plant: DARE cannot converge - constructor warns and reports non-convergence
    {
        Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
        A << 2.0, 0.0, 0.0, 0.5; // unstable mode at 2.0
        B << 0.0, 1.0;           // first state uncontrollable
        C << 1.0, 0.0;
        D << 0.0;
        ctrl::StateSpace unstable(A, B, C, D, Ts);
        ctrl::LQRParams lp;
        lp.Q = Eigen::MatrixXd::Identity(2, 2);
        lp.R = Eigen::MatrixXd::Identity(1, 1);
        // Constructor no longer throws - it warns and returns best available iterate
        ctrl::DiscreteLQR bad(unstable, lp);
        test::check(!bad.dareConverged(), "LQR: unstabilizable plant - dareConverged() is false");
        test::check(bad.gainMatrix().allFinite(), "LQR: unstabilizable - gain matrix still finite");
    }

    // LQRAdapter wraps LQR as IController
    {
        Eigen::VectorXd x_state = Eigen::VectorXd::Ones(n) * 0.5;
        ctrl::LQRAdapter adapter(lqr, [&]
                                 { return x_state; }, [&]
                                 { return Eigen::VectorXd::Zero(n); });
        double out = adapter.compute(0.0); // signal ignored
        test::check(std::isfinite(out), "LQRAdapter::compute returns finite value");
    }
}

// ============================================================
//  4. DiscreteMPC tests
// ============================================================
void test_mpc()
{
    test::suite("DiscreteMPC");

    auto plant = make_plant();

    ctrl::MPCParams mp;
    mp.Np = 20;
    mp.Nc = 5;
    mp.rho_y = 1.0;
    mp.rho_u = 0.1;
    mp.uMin = -5.0;
    mp.uMax = 5.0;

    ctrl::DiscreteMPC mpc(plant, mp);

    // Zero error -> near-zero output
    test::check(std::abs(mpc.compute(0.0)) < 1e-6, "MPC: zero error -> ~zero output");

    // Positive error -> positive control
    mpc.reset();
    test::check(mpc.compute(1.0) > 0.0, "MPC: positive error -> positive control");

    // Saturation: very large error should clamp to uMax
    mpc.reset();
    double u_big = mpc.compute(1e6);
    test::check(u_big <= mp.uMax + 1e-9, "MPC: large error clamped to uMax");

    // reset() clears state
    mpc.reset();
    test::check(std::abs(mpc.compute(0.0)) < 1e-6, "MPC: reset restores zero state");

    // Closed-loop tracking
    {
        mpc.reset();
        auto plant2 = make_plant();
        double y_final = closed_loop(mpc, plant2, 1.0, 3000);
        test::check(std::abs(y_final - 1.0) < 0.05, "MPC closed-loop tracks unit step");
    }

    // Nc > Np: setParams should clamp or the MPC should handle it gracefully
    {
        ctrl::MPCParams mp2 = mp;
        mp2.Nc = mp2.Np + 5; // invalid: Nc > Np
        test::no_throw([&]
                       {
            // Implementation clamps Nc = min(Nc, Np); confirm no crash
            ctrl::DiscreteMPC mpc2(plant, mp2);
            mpc2.compute(1.0); }, "MPC: Nc > Np handled without crash");
    }

    // MPCHorizonTuner recommendation
    {
        auto rec = ctrl::MPCHorizonTuner::recommend(plant, Ts);
        test::check(rec.Np >= 5, "MPCHorizonTuner: Np >= 5");
        test::check(rec.Nc >= 1 && rec.Nc <= rec.Np, "MPCHorizonTuner: 1 <= Nc <= Np");
    }
}

// ============================================================
//  5. DiscreteLeadLag tests
// ============================================================
void test_lead_lag()
{
    test::suite("DiscreteLeadLag");

    ctrl::LeadLagParams lp;
    lp.continuousZero = 1.0;
    lp.continuousPole = 10.0;
    lp.gain = 1.0;

    ctrl::DiscreteLeadLag ll(lp, Ts);

    // Unity input -> finite output
    test::check(std::isfinite(ll.compute(1.0)), "LeadLag: finite input -> finite output");

    // Zero input -> zero output
    ll.reset();
    test::check(std::abs(ll.compute(0.0)) < 1e-12, "LeadLag: zero input -> zero output");

    // phaseAt at the geometric mean of zero and pole should equal max phase
    double omega_max = std::sqrt(lp.continuousZero * lp.continuousPole);
    double phase = ll.phaseAt(omega_max);
    test::check(phase > 0.0, "LeadLag: phase lead is positive at omega_max");

    // Lag compensator (pole < zero) -> negative phase
    {
        ctrl::LeadLagParams lag;
        lag.continuousZero = 10.0;
        lag.continuousPole = 1.0;
        lag.gain = 1.0;
        ctrl::DiscreteLeadLag lag_ctrl(lag, Ts);
        test::check(lag_ctrl.phaseAt(std::sqrt(lag.continuousZero * lag.continuousPole)) < 0.0,
                    "LeadLag: lag compensator has negative phase");
    }

    // reset clears filter memory
    ll.compute(100.0);
    ll.reset();
    test::check(std::abs(ll.compute(0.0)) < 1e-12, "LeadLag: reset clears memory");

    // LoopShapingTuner: valid lead design
    {
        ctrl::LoopShapingTuner::Input in{10.0, 45.0, 0.5};
        ctrl::LeadLagParams tuned = ctrl::LoopShapingTuner::tuneImpl(in);
        test::check(tuned.continuousPole > tuned.continuousZero,
                    "LoopShaping: tuned lead has p > z");
        test::check(tuned.gain > 0.0, "LoopShaping: tuned gain > 0");
    }

    // LoopShapingTuner: phase_add_deg = 0 -> returns fallback (no crash)
    {
        ctrl::LoopShapingTuner::Input bad{10.0, 0.0, 0.5};
        test::no_throw([&]
                       { ctrl::LoopShapingTuner::tuneImpl(bad); }, "LoopShaping: phase_add=0 fallback, no crash");
    }
}

// ============================================================
//  6. DiscreteSMC tests
// ============================================================
void test_smc()
{
    test::suite("DiscreteSMC");

    ctrl::SMCParams sp;
    sp.c_e = 1.0;
    sp.c_de = 0.1;
    sp.K = 5.0;
    sp.phi = 0.5;
    sp.uMin = -10.0;
    sp.uMax = 10.0;

    ctrl::DiscreteSMC smc(sp, Ts);

    // Zero error -> zero surface -> zero control
    test::check(std::abs(smc.compute(0.0)) < 1e-12, "SMC: zero error -> zero output");

    // Large error: output must be within [uMin, uMax]
    smc.reset();
    double u_big = smc.compute(1e6);
    test::check(u_big >= sp.uMin - 1e-9 && u_big <= sp.uMax + 1e-9,
                "SMC: output within saturation limits");

    // Sliding surface sign matches error sign
    smc.reset();
    smc.compute(1.0); // positive error
    test::check(smc.slidingSurface() > 0.0, "SMC: positive error -> positive surface");

    // Closed-loop convergence
    {
        auto plant = make_plant();
        double y_final = closed_loop(smc, plant, 1.0, 3000);
        test::check(std::abs(y_final - 1.0) < 0.05, "SMC: closed-loop tracks unit step");
    }

    // reset clears error state
    smc.compute(10.0);
    smc.reset();
    test::check(std::abs(smc.compute(0.0)) < 1e-12, "SMC: reset clears error state");

    // Very small phi (near ideal SMC): still produces bounded output
    {
        ctrl::SMCParams sp2 = sp;
        sp2.phi = 1e-9;
        ctrl::DiscreteSMC smc2(sp2, Ts);
        double u = smc2.compute(0.3);
        test::check(std::isfinite(u), "SMC: near-ideal phi still finite output");
    }
}

// ============================================================
//  7. DiscreteADRC tests
// ============================================================
void test_adrc()
{
    test::suite("DiscreteADRC");

    ctrl::ADRCParams ap;
    ap.omega_o = 20.0;
    ap.omega_c = 5.0;
    ap.b0 = 1.0;
    ap.uMin = -20.0;
    ap.uMax = 20.0;

    ctrl::DiscreteADRC adrc(ap, Ts);

    // computeTracking with y=0, r=0 -> near-zero output
    test::check(std::abs(adrc.computeTracking(0.0, 0.0)) < 1e-12,
                "ADRC: zero y, zero ref -> zero output");

    // Nonzero reference -> nonzero output
    adrc.reset();
    test::check(adrc.computeTracking(0.0, 1.0) != 0.0,
                "ADRC: nonzero ref -> nonzero output");

    // ESO state initialized to zero
    test::check(adrc.esoState().norm() < 1e-12, "ADRC: ESO state zero after reset");

    // setReference + compute() interface
    adrc.reset();
    adrc.setReference(1.0);
    double u = adrc.compute(0.0); // y=0, r=1
    test::check(std::isfinite(u), "ADRC: setReference+compute is finite");

    // Closed-loop convergence
    {
        ctrl::DiscreteADRC adrc2(ap, Ts);
        auto plant = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        double y = 0.0;
        const double ref = 1.0;
        for (int k = 0; k < 3000; ++k)
        {
            double u2 = adrc2.computeTracking(y, ref);
            Eigen::VectorXd uv(1);
            uv << u2;
            y = ctrl::ssStep(plant, x, uv)(0);
        }
        test::check(std::abs(y - 1.0) < 0.05, "ADRC: closed-loop tracks unit step");
    }

    // Output saturation with very large reference
    adrc.reset();
    adrc.setReference(1e9);
    double u_sat = adrc.compute(0.0);
    test::check(u_sat <= ap.uMax + 1e-9, "ADRC: output clamped at uMax");
}

// ============================================================
//  8. ExtremumSeeker tests
// ============================================================
void test_esc()
{
    test::suite("ExtremumSeeker");

    ctrl::ExtremumSeekerParams ep;
    ep.perturbAmp = 0.05;
    ep.perturbFreq = 5.0;
    ep.lpfCutoff = 0.5;
    ep.hpfCutoff = 0.1;
    ep.integGain = 0.5;
    ep.seekMinimum = true;

    ctrl::ExtremumSeeker esc(ep, Ts);

    // Returns finite output on first call
    test::check(std::isfinite(esc.compute(0.5)), "ESC: first compute() is finite");

    // Seeking minimum of J(u) = (u-2)^2; ESC should drift theta toward 2
    {
        ctrl::ExtremumSeeker esc2(ep, Ts);
        // Run many steps on a static quadratic cost
        for (int k = 0; k < 5000; ++k)
        {
            double theta = esc2.currentEstimate();
            double u = esc2.compute((theta - 2.0) * (theta - 2.0));
            (void)u;
        }
        double theta_final = esc2.currentEstimate();
        test::check(std::abs(theta_final - 2.0) < 0.5,
                    "ESC: theta converges near minimum at 2.0");
    }

    // reset clears all filter states and integrator
    esc.compute(1.0);
    esc.reset();
    test::check(std::abs(esc.currentEstimate()) < 1e-12, "ESC: reset clears theta");

    // Zero perturbation amplitude -> no dither, theta stays at 0
    {
        ctrl::ExtremumSeekerParams ep2 = ep;
        ep2.perturbAmp = 0.0;
        ctrl::ExtremumSeeker esc3(ep2, Ts);
        for (int k = 0; k < 100; ++k)
            esc3.compute(1.0);
        test::check(std::abs(esc3.currentEstimate()) < 1e-6,
                    "ESC: zero perturbation -> theta stays at 0");
    }
}

// ============================================================
//  9. SmithPredictor tests
// ============================================================
void test_smith_predictor()
{
    test::suite("SmithPredictor");

    auto plant = make_plant();

    // Build an inner PID
    ctrl::PIDParams pp;
    pp.Kp = 2.0;
    pp.Ki = 1.0;
    pp.Kd = 0.05;
    pp.N = 20.0;
    pp.uMin = -10.0;
    pp.uMax = 10.0;
    auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // Delay model: the plant itself (no dead time in model)
    int delay_steps = 5;
    ctrl::SmithPredictor sp(inner, plant, delay_steps);

    // First compute is finite
    test::check(std::isfinite(sp.compute(1.0)), "SmithPredictor: first compute is finite");

    // Zero error -> near-zero output (inner PID is pure proportional if no integral built up)
    {
        ctrl::SmithPredictor sp2(std::make_shared<ctrl::DiscretePID>(pp, Ts), plant, delay_steps);
        test::check(std::abs(sp2.compute(0.0)) < 1e-12,
                    "SmithPredictor: zero error -> zero output");
    }

    // reset clears inner controller and buffers
    sp.compute(5.0);
    sp.reset();
    test::check(std::abs(sp.compute(0.0)) < 1e-12, "SmithPredictor: reset -> zero output");

    // delay_steps = 0: no prediction buffer, should work like a plain inner controller
    {
        ctrl::SmithPredictor sp0(std::make_shared<ctrl::DiscretePID>(pp, Ts), plant, 0);
        test::check(std::isfinite(sp0.compute(1.0)),
                    "SmithPredictor: delay=0 still works");
    }

    // innerController() accessor works
    test::no_throw([&]
                   { sp.innerController(); }, "SmithPredictor: innerController() accessible");
}

// ============================================================
//  10. KalmanFilter tests
// ============================================================
void test_kalman()
{
    test::suite("KalmanFilter");

    auto plant = make_plant();
    const int n = plant.stateSize();
    const int p = plant.outputSize();

    Eigen::MatrixXd Qn = Eigen::MatrixXd::Identity(n, n) * 1e-4;
    Eigen::MatrixXd Rn = Eigen::MatrixXd::Identity(p, p) * 1e-2;

    ctrl::KalmanFilter kf(plant, Qn, Rn);

    // Initial state estimate is zero
    test::check(kf.state().norm() < 1e-12, "KalmanFilter: initial state is zero");

    // predict + update: state evolves
    Eigen::VectorXd u(1);
    u << 1.0;
    kf.predict(u);
    Eigen::VectorXd y(1);
    y << 0.01;
    kf.update(y, u);
    test::check(kf.state().norm() > 0.0, "KalmanFilter: state nonzero after predict+update");

    // reset returns state to zero
    kf.reset();
    test::check(kf.state().norm() < 1e-12, "KalmanFilter: reset restores zero state");

    // covariance is positive (non-trivial)
    kf.predict(u);
    test::check(kf.covariance().trace() > 0.0, "KalmanFilter: covariance trace > 0");

    // step() combines predict+update in one call
    kf.reset();
    test::no_throw([&]
                   { kf.step(y, u); }, "KalmanFilter: step() no throw");

    // KalmanWeightTuner::isotropic produces correct dimensions
    {
        auto kp = ctrl::KalmanWeightTuner::isotropic(n, p, 0.01, 0.1);
        test::check(kp.Qf.rows() == n && kp.Qf.cols() == n,
                    "KalmanWeightTuner: Qf has correct size");
        test::check(kp.Rf.rows() == p && kp.Rf.cols() == p,
                    "KalmanWeightTuner: Rf has correct size");
    }
}

// ============================================================
//  11. DiscreteLQG tests
// ============================================================
void test_lqg()
{
    test::suite("DiscreteLQG");

    auto plant = make_plant();
    const int n = plant.stateSize();
    const int p = plant.outputSize();
    const int m = plant.inputSize();

    ctrl::LQRParams lqr_p;
    lqr_p.Q = Eigen::MatrixXd::Identity(n, n);
    lqr_p.R = Eigen::MatrixXd::Identity(m, m) * 0.1;

    Eigen::MatrixXd Qn = Eigen::MatrixXd::Identity(n, n) * 1e-4;
    Eigen::MatrixXd Rn = Eigen::MatrixXd::Identity(p, p) * 1e-2;

    ctrl::DiscreteLQG lqg(plant, lqr_p, Qn, Rn);

    // Full step interface
    Eigen::VectorXd y(1);
    y << 0.0;
    Eigen::VectorXd u_prev(1);
    u_prev << 0.0;
    Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(n);
    auto u_vec = lqg.step(y, u_prev, x_ref);
    test::check(std::isfinite(u_vec(0)), "LQG: step() returns finite control");

    // SISO compute interface
    lqg.reset();
    lqg.setReference(x_ref);
    lqg.setUPrev(u_prev);
    test::check(std::isfinite(lqg.compute(0.0)), "LQG: compute(y) returns finite");

    // State estimate accessible
    test::check(lqg.stateEstimate().size() == n,
                "LQG: stateEstimate() has correct size");

    // Closed-loop convergence
    {
        ctrl::DiscreteLQG lqg2(plant, lqr_p, Qn, Rn);
        auto plant2 = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
        double y_out = 0.0;
        Eigen::VectorXd u_p(1);
        u_p << 0.0;
        Eigen::VectorXd r = Eigen::VectorXd::Zero(n);
        // Track step in first state
        r(0) = 1.0;
        for (int k = 0; k < 3000; ++k)
        {
            Eigen::VectorXd ymeas(1);
            ymeas << y_out;
            auto u_lqg = lqg2.step(ymeas, u_p, r);
            u_p = u_lqg;
            Eigen::VectorXd uv(1);
            uv << u_lqg(0);
            y_out = ctrl::ssStep(plant2, x, uv)(0);
        }
        test::check(std::abs(y_out - 1.0) < 0.1, "LQG: closed-loop tracks step reference");
    }

    // reset
    lqg.reset();
    test::no_throw([&]
                   { lqg.compute(0.0); }, "LQG: compute after reset, no throw");
}

// ============================================================
//  12. ControllerStack tests
// ============================================================
void test_stack()
{
    test::suite("ControllerStack");

    // Supervisory stack with no controllers: compute returns 0
    {
        ctrl::ControllerStack empty_stack(ctrl::StackMode::Supervisory, Ts);
        test::check(std::abs(empty_stack.compute(1.0)) < 1e-12,
                    "Stack: no controllers -> zero output");
    }

    // Supervisory: single controller always active
    {
        ctrl::PIDParams pp;
        pp.Kp = 2.0;
        pp.Ki = 0.0;
        pp.Kd = 0.0;
        pp.uMin = -1e9;
        pp.uMax = 1e9;
        ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "PID");
        double u = stack.compute(1.0);
        test::check(std::abs(u - 2.0) < 1e-9, "Stack Supervisory: single PID u = Kp*e");
        test::check(stack.activeControllerName() == "PID",
                    "Stack Supervisory: active name is PID");
    }

    // Additive: sum of two P controllers
    {
        ctrl::PIDParams pp;
        pp.Kp = 1.0;
        pp.Ki = 0.0;
        pp.Kd = 0.0;
        pp.uMin = -1e9;
        pp.uMax = 1e9;
        ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "P1");
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "P2");
        double u = stack.compute(1.0);
        test::check(std::abs(u - 2.0) < 1e-9, "Stack Additive: sum of two P controllers");
    }

    // Weighted: 0.5 weight on each -> same as single for equal weights
    {
        ctrl::PIDParams pp;
        pp.Kp = 2.0;
        pp.Ki = 0.0;
        pp.Kd = 0.0;
        pp.uMin = -1e9;
        pp.uMax = 1e9;
        ctrl::ControllerStack stack(ctrl::StackMode::Weighted, Ts);
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "P1", 0.5);
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "P2", 0.5);
        double u = stack.compute(1.0);
        test::check(std::abs(u - 2.0) < 1e-9, "Stack Weighted: 50/50 blend");
    }

    // setActive disables a controller
    {
        ctrl::PIDParams pp;
        pp.Kp = 3.0;
        pp.Ki = 0.0;
        pp.Kd = 0.0;
        pp.uMin = -1e9;
        pp.uMax = 1e9;
        ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "P1");
        stack.setActive("P1", false);
        test::check(std::abs(stack.compute(1.0)) < 1e-12,
                    "Stack: disabled controller contributes 0");
    }

    // removeController then compute doesn't crash
    {
        ctrl::PIDParams pp;
        pp.Kp = 1.0;
        pp.Ki = 0.0;
        pp.Kd = 0.0;
        pp.uMin = -1e9;
        pp.uMax = 1e9;
        ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "P");
        stack.removeController("P");
        test::no_throw([&]
                       { stack.compute(1.0); }, "Stack: compute after removeController");
    }

    // Conditional Supervisory switching: SMC when |e|>0.5, PID otherwise
    {
        ctrl::PIDParams pp;
        pp.Kp = 2.0;
        pp.Ki = 0.0;
        pp.Kd = 0.0;
        pp.uMin = -1e9;
        pp.uMax = 1e9;
        ctrl::SMCParams sp;
        sp.c_e = 1.0;
        sp.c_de = 0.0;
        sp.K = 4.0;
        sp.phi = 1.0;
        sp.uMin = -1e9;
        sp.uMax = 1e9;
        ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
        stack.addController(std::make_shared<ctrl::DiscreteSMC>(sp, Ts), "SMC",
                            1.0, [](double e, double)
                            { return std::abs(e) > 0.5; });
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "PID");

        // Large error: SMC should be active
        stack.compute(1.0);
        test::check(stack.activeControllerName() == "SMC",
                    "Stack Supervisory: SMC active for large error");

        // Small error: PID should be active
        stack.compute(0.1);
        test::check(stack.activeControllerName() == "PID",
                    "Stack Supervisory: PID active for small error");
    }
}

// ============================================================
//  13. Tuner tests
// ============================================================
void test_tuners()
{
    test::suite("ControllerTuner");

    auto plant = make_plant();

    // StepResponseTuner::identify requires >= 4 samples
    test::throws([]
                 { ctrl::StepResponseTuner::identify({0.0, 0.1, 0.2}, {0.0, 0.1, 0.2}, 1.0); }, "StepResponseTuner: < 4 samples throws");

    // Mismatched time/output sizes
    test::throws([]
                 { ctrl::StepResponseTuner::identify({0.0, 0.1, 0.2, 0.3},
                                                     {0.0, 0.1},
                                                     1.0); }, "StepResponseTuner: mismatched sizes throws");

    // Output not reaching 63.2% throws
    test::throws([]
                 {
        // Flat output far below final value
        std::vector<double> t(100), y(100, 0.1);
        for (int i = 0; i < 100; ++i) t[i] = i * 0.01;
        ctrl::StepResponseTuner::identify(t, y, 1.0); }, "StepResponseTuner: output not reaching 63.2% throws");

    // RelayAutoTuner: computePIDParams before isDone throws
    {
        ctrl::RelayTunerConfig cfg;
        cfg.relayAmplitude = 1.0;
        cfg.cyclesRequired = 3;
        ctrl::RelayAutoTuner relay(cfg, Ts);
        test::throws([&]
                     { relay.computePIDParams(ctrl::PIDTuningRule::TyreusLuyben); }, "RelayAutoTuner: computePIDParams before isDone throws");
    }

    // ZieglerNicholsTuner: valid input
    {
        ctrl::ZieglerNicholsTuner::Input in{2.5, 1.0};
        ctrl::PIDParams p = ctrl::ZieglerNicholsTuner::tuneImpl(in);
        test::check(p.Kp > 0 && p.Ki > 0 && p.Kd > 0,
                    "ZN: positive gains for valid input");
    }

    // CohenCoonTuner: theta=0 -> throws
    test::throws([]
                 {
        ctrl::StepResponseTuner::FOPDTModel m{1.0, 2.0, 0.0};
        ctrl::CohenCoonTuner::tuneImpl(m, 0.01); }, "CohenCoon: theta=0 throws");

    // CohenCoonTuner: valid FOPDT model
    {
        ctrl::StepResponseTuner::FOPDTModel m{1.0, 2.0, 0.3};
        ctrl::PIDParams p = ctrl::CohenCoonTuner::tuneImpl(m, 0.01);
        test::check(p.Kp > 0, "CohenCoon: valid model gives Kp > 0");
    }

    // LQRWeightTuner::brysonMethod: diagonal weights
    {
        Eigen::VectorXd xmax(2);
        xmax << 1.0, 1.0;
        Eigen::VectorXd umax(1);
        umax << 5.0;
        ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
        test::check(lp.Q(0, 0) > 0 && lp.R(0, 0) > 0,
                    "Bryson: Q and R positive definite");
    }

    // IMC-tuned PID has all gains
    {
        // Collect open-loop step response
        std::vector<double> t_data(1500), y_data(1500);
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        Eigen::VectorXd uv(1);
        uv << 1.0;
        for (int k = 0; k < 1500; ++k)
        {
            y_data[k] = ctrl::ssStep(plant, x, uv)(0);
            t_data[k] = k * Ts;
        }
        auto fopdt = ctrl::StepResponseTuner::identify(t_data, y_data, 1.0);
        auto pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::IMC);
        test::check(pp.Kp > 0, "IMC: Kp > 0");
        test::check(pp.Ki > 0, "IMC: Ki > 0");
    }
}

// ============================================================
//  main
// ============================================================
int main()
{
    std::cout << "============================================================\n";
    std::cout << "  Controller Toolbox - Comprehensive Test Suite\n";
    std::cout << "  Plant: G(s) = 1/(s^2 + 1.5s + 1),  Ts = " << Ts << " s\n";
    std::cout << "============================================================\n";

    test_plant_model();
    test_pid();
    test_lqr();
    test_mpc();
    test_lead_lag();
    test_smc();
    test_adrc();
    test_esc();
    test_smith_predictor();
    test_kalman();
    test_lqg();
    test_stack();
    test_tuners();

    test::report();
    return (test::failed == 0) ? 0 : 1;
}
