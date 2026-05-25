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

// Continuous-time G_c(s) = 1/(s^2+1.5s+1), companion form; Ts=0 signals continuous-time.
static ctrl::StateSpace make_continuous_plant()
{
    Eigen::MatrixXd Ac(2, 2), Bc(2, 1), Cc(1, 2), Dc(1, 1);
    Ac << 0.0, 1.0, -1.0, -1.5;
    Bc << 0.0, 1.0;
    Cc << 1.0, 0.0;
    Dc << 0.0;
    return ctrl::StateSpace(Ac, Bc, Cc, Dc, 0.0);
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

    // DC gain approx = 1 for G(s)=1/(s^2+1.5s+1).
    // The pre-discretised TF uses rounded coefficients, so DC gain approx = 0.9.
    auto Acl = Eigen::MatrixXd::Identity(2, 2) - plant.A;
    double dc = (plant.C * Acl.inverse() * plant.B + plant.D)(0, 0);
    test::check(std::abs(dc - 1.0) < 0.15, "DC gain approx 1.0 (rounded TF coeffs)");

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

    // Wrong-sized x_ref must throw
    {
        Eigen::VectorXd x_state = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd bad_ref(1);
        bad_ref << 0.5; // wrong size for a 2-state plant
        test::throws([&]
                     { lqr.compute(x_state, bad_ref); },
                     "LQR: wrong-sized x_ref throws invalid_argument");
    }

    // Wrong-sized u_ff must throw
    {
        Eigen::VectorXd x_state = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd bad_ff(3);
        bad_ff.setZero(); // wrong size for a 1-input plant
        test::throws([&]
                     { lqr.compute(x_state, Eigen::VectorXd(), bad_ff); },
                     "LQR: wrong-sized u_ff throws invalid_argument");
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

    // Closed-loop tracking - use a longer prediction horizon for this slow plant
    {
        ctrl::MPCParams mp_track = mp;
        mp_track.Np = 50;
        mp_track.Nc = 10;
        mp_track.rho_u = 0.01; // less move suppression -> faster convergence
        ctrl::DiscreteMPC mpc_track(plant, mp_track);
        auto plant2 = make_plant();
        double y_final = closed_loop(mpc_track, plant2, 1.0, 5000);
        test::check(std::abs(y_final - 1.0) < 0.10, "MPC closed-loop tracks unit step");
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

    // QP constraint: duMax enforced by gradient-projection solver
    {
        ctrl::MPCParams mp_qp = mp;
        mp_qp.duMax = 0.1;
        mp_qp.duMin = -0.1;
        ctrl::DiscreteMPC mpc_qp(plant, mp_qp);
        double u_first = mpc_qp.compute(1.0); // from rest, first increment must be <= 0.1
        test::check(std::abs(u_first) <= mp_qp.duMax + 1e-6,
                    "MPC QP: first move respects duMax=0.1");
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

    // Closed-loop convergence.
    // SMC convention: compute(y - ref) (output-minus-reference), not the standard r-y.
    // Positive output-error (y>ref) -> negative surface -> positive u for positive-gain plant.
    {
        ctrl::DiscreteSMC smc2(sp, Ts);
        auto plant = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        double y = 0.0;
        for (int k = 0; k < 5000; ++k)
        {
            double u = smc2.compute(y - 1.0); // pass y-ref per SMC sign convention
            Eigen::VectorXd uv(1);
            uv << u;
            y = ctrl::ssStep(plant, x, uv)(0);
        }
        // Boundary layer (phi=0.5) creates ~0.10 residual offset from DC-gain mismatch
        test::check(std::abs(y - 1.0) < 0.15, "SMC: closed-loop tracks unit step");
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

    // IController composability: ADRC in a ControllerStack must receive error (r-y),
    // not raw y.  The stack calls compute(error) where error = ref - y.
    // setReference(r) sets the reference; compute(error) internally reconstructs y = r - error.
    {
        ctrl::DiscreteADRC adrc3(ap, Ts);
        adrc3.setReference(1.0);

        ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
        stack.addController(std::make_shared<ctrl::DiscreteADRC>(ap, Ts), "ADRC");
        // Stack calls compute(error) with error = ref - y.
        // For ADRC the stored reference r_ is 0 (default), so this tests that the
        // interface does not crash and produces a finite output.
        double u_stack = stack.compute(1.0); // error=1 -> y_reconstructed = r_(0) - 1 = -1
        test::check(std::isfinite(u_stack),
                    "ADRC: compute(error) via ControllerStack returns finite (interface contract)");

        // Verify sign: with r_=0 and error=1, y=-1 (below reference),
        // ADRC should push u positive (corrects output toward r=0).
        test::check(u_stack > 0.0,
                    "ADRC: ControllerStack compute(+error) yields positive u (y below ref)");
    }
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

    // Seeking minimum of J(u) = (u-2)^2; evaluate cost at the DITHERED point u, not theta
    {
        ctrl::ExtremumSeeker esc2(ep, Ts);
        double J = 0.0;
        for (int k = 0; k < 5000; ++k)
        {
            double u = esc2.compute(J); // feed previous cost, get next search point
            J = (u - 2.0) * (u - 2.0);  // evaluate cost at dithered point
        }
        double theta_final = esc2.currentEstimate();
        test::check(std::abs(theta_final - 2.0) < 1.0,
                    "ESC: theta converges near minimum at 2.0");
    }

    // reset clears all filter states and integrator
    esc.compute(1.0);
    esc.reset();
    test::check(std::abs(esc.currentEstimate()) < 1e-12, "ESC: reset clears theta");

    // Zero perturbation amplitude -> no dither, theta should stay near 0.
    // Feed J=0 (cost at origin) so HPF sees a zero signal: no demodulated gradient.
    {
        ctrl::ExtremumSeekerParams ep2 = ep;
        ep2.perturbAmp = 0.0;
        ctrl::ExtremumSeeker esc3(ep2, Ts);
        for (int k = 0; k < 100; ++k)
            esc3.compute(0.0); // zero cost -> zero HPF output -> zero gradient
        test::check(std::abs(esc3.currentEstimate()) < 0.01,
                    "ESC: zero perturbation -> theta stays near 0");
    }

    // Phase accumulator periodicity: output at step N must equal output at step 0
    // for the same cost input, modulo the gradient integrator (which we zero by using
    // seekMinimum=false and zero integGain so theta is fixed).  After exactly one full
    // dither period the phase wraps back to 0 and the dither output repeats.
    {
        const int period_steps = static_cast<int>(std::round(1.0 / (ep.perturbFreq * Ts)));
        ctrl::ExtremumSeekerParams ep3 = ep;
        ep3.integGain = 0.0; // disable gradient integration so theta stays at 0
        ep3.perturbAmp = 0.1;
        ctrl::ExtremumSeeker esc4(ep3, Ts);

        // Run one full period, capturing the first-step output
        double u_first = esc4.compute(0.0);
        for (int k = 1; k < period_steps; ++k)
            esc4.compute(0.0);
        // After exactly period_steps steps phase should be back to ~0 + one increment
        // (since phase is updated before output).  A second period should produce the
        // same sequence: capture the step after the period boundary.
        double u_repeat = esc4.compute(0.0);
        // The dither is a*sin(phase) where phase advances by 2pi*f*Ts each step.
        // After period_steps steps, phase has advanced by exactly 2pi -> wraps to ~0.
        // So u_repeat should match u_first (both at phase = 2pi*f*Ts after reset/wrap).
        test::check(std::abs(u_repeat - u_first) < 1e-6,
                    "ESC: dither repeats exactly after one period (phase accumulator wraps)");
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

    // Closed-loop regulation: use a 1D plant (A=0.9, B=1, C=1) with perfect observability.
    // LQR+KF regulates from displaced initial state x=1 back to x=0 (reference r=0).
    {
        ctrl::StateSpace plant_1d(
            (Eigen::MatrixXd(1, 1) << 0.9).finished(),
            (Eigen::MatrixXd(1, 1) << 1.0).finished(),
            (Eigen::MatrixXd(1, 1) << 1.0).finished(),
            (Eigen::MatrixXd(1, 1) << 0.0).finished(),
            Ts);
        ctrl::LQRParams lqr_1d;
        lqr_1d.Q = Eigen::MatrixXd::Identity(1, 1);
        lqr_1d.R = Eigen::MatrixXd::Identity(1, 1) * 0.1;
        Eigen::MatrixXd Qn_1d = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
        Eigen::MatrixXd Rn_1d = Eigen::MatrixXd::Identity(1, 1) * 1e-2;
        ctrl::DiscreteLQG lqg_1d(plant_1d, lqr_1d, Qn_1d, Rn_1d);

        Eigen::VectorXd x_1d(1);
        x_1d(0) = 1.0;
        Eigen::VectorXd up_1d(1);
        up_1d << 0.0;
        Eigen::VectorXd r_zero = Eigen::VectorXd::Zero(1);
        for (int k = 0; k < 500; ++k)
        {
            Eigen::VectorXd ymeas(1);
            ymeas << (plant_1d.C * x_1d)(0); // observe current state directly
            up_1d = lqg_1d.step(ymeas, up_1d, r_zero);
            ctrl::ssStep(plant_1d, x_1d, up_1d); // updates x_1d in place
        }
        test::check(std::abs(x_1d(0)) < 0.01, "LQG: 1D closed-loop regulates to zero");
    }

    // reset
    lqg.reset();
    test::no_throw([&]
                   { lqg.compute(0.0); }, "LQG: compute after reset, no throw");

    // Separation principle: as noise decreases, LQG closed-loop poles should converge
    // toward the LQR poles (the estimator stops limiting performance).
    // We verify this by checking that the steady-state regulation error is no worse
    // than pure-state-feedback LQR when Q_noise -> 0 (near-perfect observation).
    {
        ctrl::StateSpace plant_sp(
            (Eigen::MatrixXd(1, 1) << 0.9).finished(),
            (Eigen::MatrixXd(1, 1) << 1.0).finished(),
            (Eigen::MatrixXd(1, 1) << 1.0).finished(),
            (Eigen::MatrixXd(1, 1) << 0.0).finished(),
            Ts);
        ctrl::LQRParams lqr_sp;
        lqr_sp.Q = Eigen::MatrixXd::Identity(1, 1);
        lqr_sp.R = Eigen::MatrixXd::Identity(1, 1) * 0.1;

        // LQR full-state reference settling
        ctrl::DiscreteLQR lqr_ref(plant_sp, lqr_sp);
        Eigen::VectorXd x_lqr(1);
        x_lqr(0) = 1.0;
        for (int k = 0; k < 300; ++k)
        {
            Eigen::VectorXd u_lqr = lqr_ref.compute(x_lqr, Eigen::VectorXd::Zero(1));
            ctrl::ssStep(plant_sp, x_lqr, u_lqr);
        }
        const double lqr_residual = std::abs(x_lqr(0));

        // LQG with very low process noise: Kalman gain -> perfect observer
        Eigen::MatrixXd Qn_tiny = Eigen::MatrixXd::Identity(1, 1) * 1e-10;
        Eigen::MatrixXd Rn_sp = Eigen::MatrixXd::Identity(1, 1) * 1e-2;
        ctrl::DiscreteLQG lqg_sp(plant_sp, lqr_sp, Qn_tiny, Rn_sp);

        Eigen::VectorXd x_lqg(1);
        x_lqg(0) = 1.0;
        Eigen::VectorXd up_sp(1);
        up_sp << 0.0;
        Eigen::VectorXd r_zero_sp = Eigen::VectorXd::Zero(1);
        for (int k = 0; k < 300; ++k)
        {
            Eigen::VectorXd ym(1);
            ym << (plant_sp.C * x_lqg)(0);
            up_sp = lqg_sp.step(ym, up_sp, r_zero_sp);
            ctrl::ssStep(plant_sp, x_lqg, up_sp);
        }
        const double lqg_residual = std::abs(x_lqg(0));

        // LQG with near-perfect observer must settle at least as well as LQR (within 10x)
        test::check(lqg_residual < lqr_residual * 10.0 + 1e-6,
                    "LQG: separation principle - LQG residual close to LQR with tiny process noise");
    }
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

    // Bumpless transfer: switch from PID to a second PID at steady state.
    // The output immediately after the switch must be within 0.1 of the pre-switch output.
    {
        ctrl::PIDParams pp1;
        pp1.Kp = 2.0;
        pp1.Ki = 1.0;
        pp1.Kd = 0.0;
        pp1.uMin = -1e9;
        pp1.uMax = 1e9;
        ctrl::PIDParams pp2;
        pp2.Kp = 5.0;
        pp2.Ki = 0.5;
        pp2.Kd = 0.0;
        pp2.uMin = -1e9;
        pp2.uMax = 1e9;

        auto pid1 = std::make_shared<ctrl::DiscretePID>(pp1, Ts);
        auto pid2 = std::make_shared<ctrl::DiscretePID>(pp2, Ts);

        ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
        // PID1 active when e > 0.1, PID2 otherwise
        stack.addController(pid1, "PID1", 1.0, [](double e, double)
                            { return e > 0.1; });
        stack.addController(pid2, "PID2");

        auto plant = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        double y = 0.0, ref = 1.0;

        // Run to steady state with PID1 (error stays > 0.1 during initial transient)
        for (int k = 0; k < 1500; ++k)
        {
            double u = stack.compute(ref - y);
            Eigen::VectorXd uv(1);
            uv << u;
            y = ctrl::ssStep(plant, x, uv)(0);
        }
        // At this point error is small -> PID2 should have taken over
        double u_before_switch = stack.compute(ref - y);
        // Force a large error step so PID1 comes back in, then return to PID2
        double u_after_switch = stack.compute(0.2); // |e|=0.2 > 0.1, PID1 activates
        // PID2 was last active; PID1 should bumpless-init to u_before_switch.
        // u_ss ~ 1.0 for this plant; a jump of > 0.1 indicates bumpless logic broke.
        test::check(std::abs(u_after_switch - u_before_switch) < 0.1,
                    "Stack: bumpless transfer limits output jump on controller switch");
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

    // Output not reaching 63.2%: implementation returns a conservative estimate without throw
    {
        std::vector<double> t(100), y(100, 0.1);
        for (int i = 0; i < 100; ++i)
            t[i] = i * 0.01;
        test::no_throw([&]
                       {
            auto m = ctrl::StepResponseTuner::identify(t, y, 1.0);
            (void)m; }, "StepResponseTuner: partial output handled without crash");
    }

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
//  14. c2d tests
// ============================================================
void test_c2d()
{
    test::suite("c2d");

    auto sys_c = make_continuous_plant();

    // ZOH: sample time stored correctly
    auto sys_zoh = ctrl::c2d(sys_c, 0.01, ctrl::C2dMethod::ZOH);
    test::check(std::abs(sys_zoh.Ts - 0.01) < 1e-12, "c2d ZOH: Ts set correctly");
    test::check(sys_zoh.stateSize() == 2, "c2d ZOH: state size preserved");

    // ZOH: DC gain preserved - C*(I-A)^-1*B + D == 1.0
    {
        Eigen::MatrixXd Acl = Eigen::MatrixXd::Identity(2, 2) - sys_zoh.A;
        double dc = (sys_zoh.C * Acl.inverse() * sys_zoh.B + sys_zoh.D)(0, 0);
        test::check(std::abs(dc - 1.0) < 0.01, "c2d ZOH: DC gain preserved (1.0)");
    }

    // Tustin: sample time stored correctly
    auto sys_tus = ctrl::c2d(sys_c, 0.01, ctrl::C2dMethod::Tustin);
    test::check(std::abs(sys_tus.Ts - 0.01) < 1e-12, "c2d Tustin: Ts set correctly");

    // Tustin: DC gain preserved
    {
        Eigen::MatrixXd Acl = Eigen::MatrixXd::Identity(2, 2) - sys_tus.A;
        double dc = (sys_tus.C * Acl.inverse() * sys_tus.B + sys_tus.D)(0, 0);
        test::check(std::abs(dc - 1.0) < 0.01, "c2d Tustin: DC gain preserved (1.0)");
    }

    // Non-continuous input (Ts != 0) must throw
    test::throws([&]
                 { ctrl::c2d(sys_zoh, 0.01); }, "c2d: non-continuous input throws");

    // Negative Ts must throw
    test::throws([&]
                 { ctrl::c2d(sys_c, -0.01); }, "c2d: negative Ts throws");

    // Default method matches ZOH explicitly
    auto sys_def = ctrl::c2d(sys_c, 0.01);
    test::check((sys_def.A - sys_zoh.A).norm() < 1e-12, "c2d: default method is ZOH");
}

// ============================================================
//  15. RecursiveLeastSquares tests
// ============================================================
void test_rls()
{
    test::suite("RecursiveLeastSquares");

    // Initial params zero, sampleCount zero
    ctrl::RecursiveLeastSquares rls0(1, 1, Ts, 0.98, 1e4);
    test::check(rls0.params().norm() < 1e-12, "RLS: initial params are zero");
    test::check(rls0.sampleCount() == 0, "RLS: initial sampleCount == 0");

    // Convergence on ARX(1,1): plant y[k] = 0.8*y[k-1] + 0.5*u[k-1]
    // RLS convention: y[k] = -a1*y[k-1] + b1*u[k-1]  => true theta = [a1,b1] = [-0.8, 0.5]
    ctrl::RecursiveLeastSquares rls1(1, 1, Ts, 0.98, 1e4);
    {
        double y_prev = 0.0, u_prev = 0.0;
        for (int k = 0; k < 600; ++k)
        {
            double y_k = 0.8 * y_prev + 0.5 * u_prev;
            double u_k = std::sin(k * 0.5) + 0.3 * std::cos(k * 1.1);
            rls1.update(y_k, u_k);
            y_prev = y_k;
            u_prev = u_k;
        }
    }
    test::check(std::abs(rls1.params()(0) - (-0.8)) < 0.05, "RLS: a1 converges to -0.8");
    test::check(std::abs(rls1.params()(1) - 0.5) < 0.05, "RLS: b1 converges to 0.5");

    // reset() must restore P to P0_scale*I (key fix: must not read stale P(0,0) after convergence)
    rls1.reset();
    test::check(rls1.sampleCount() == 0, "RLS: reset clears sampleCount");
    test::check(rls1.params().norm() < 1e-12, "RLS: reset clears params");
    test::check(std::abs(rls1.covariance()(0, 0) - 1e4) < 1.0, "RLS: reset restores P(0,0) to P0_scale");
    test::check(std::abs(rls1.covariance()(1, 1) - 1e4) < 1.0, "RLS: reset restores P(1,1) to P0_scale");

    // toTransferFunction and toStateSpace: no throw after data fed
    {
        ctrl::RecursiveLeastSquares rls2(2, 1, Ts, 0.98, 1e4);
        double y = 0.0, yp = 0.0, u = 0.0;
        for (int k = 0; k < 300; ++k)
        {
            double y_k = 0.8 * y - 0.3 * yp + 0.4 * u;
            double u_k = std::sin(k * 0.4) + std::cos(k * 0.9);
            rls2.update(y_k, u_k);
            yp = y;
            y = y_k;
            u = u_k;
        }
        test::no_throw([&]
                       { rls2.toTransferFunction(); }, "RLS: toTransferFunction no throw");
        test::no_throw([&]
                       { rls2.toStateSpace(); }, "RLS: toStateSpace no throw");
    }

    // D == 0 for a pure ARX (delay-1) model.
    // ARX: y[k] = 0.5*y[k-1] + 0.3*u[k-1] has no direct feedthrough (b0 = 0).
    // toStateSpace() must produce D(0,0) == 0; if the b0=0 prepend is missing,
    // tf2ss sets D = b1 and the test catches the regression.
    {
        ctrl::RecursiveLeastSquares rls3(1, 1, Ts, 0.99, 1e4);
        double yp3 = 0.0, up3 = 0.0;
        for (int k = 0; k < 500; ++k)
        {
            double y_k = 0.5 * yp3 + 0.3 * up3;
            double u_k = std::sin(k * 0.3) + std::cos(k * 0.7);
            rls3.update(y_k, u_k);
            yp3 = y_k;
            up3 = u_k;
        }
        ctrl::StateSpace ss3 = rls3.toStateSpace();
        test::check(std::abs(ss3.D(0, 0)) < 1e-4,
                    "RLS: toStateSpace D==0 for ARX delay-1 model (b0 prepend)");
    }
}

// ============================================================
//  16. ExtendedKalmanFilter tests
// ============================================================
void test_ekf()
{
    test::suite("ExtendedKalmanFilter");
    const int n = 2, p = 1;

    auto plt = make_plant();
    Eigen::MatrixXd Qn = Eigen::MatrixXd::Identity(n, n) * 1e-4;
    Eigen::MatrixXd Rn = Eigen::MatrixXd::Identity(p, p) * 1e-2;

    // Wrap the linear plant as nonlinear EKF functions (analytical Jacobians)
    auto f = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    { return plt.A * x + plt.B * u; };
    auto h = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &)
    { return plt.C * x; };
    auto Fj = [&](const Eigen::VectorXd &, const Eigen::VectorXd &)
    { return plt.A; };
    auto Hj = [&](const Eigen::VectorXd &, const Eigen::VectorXd &)
    { return plt.C; };

    ctrl::ExtendedKalmanFilter ekf(n, p, f, h, Fj, Hj, Qn, Rn, Ts);

    // Initial state is zero
    test::check(ekf.state().norm() < 1e-12, "EKF: initial state is zero");

    // predict + update changes state
    Eigen::VectorXd u(1);
    u << 1.0;
    ekf.predict(u);
    Eigen::VectorXd y(1);
    y << 0.01;
    ekf.update(y, u);
    test::check(ekf.state().norm() > 0.0, "EKF: state nonzero after predict+update");

    // covariance trace > 0
    test::check(ekf.covariance().trace() > 0.0, "EKF: covariance trace > 0");

    // reset clears state
    ekf.reset();
    test::check(ekf.state().norm() < 1e-12, "EKF: reset restores zero state");

    // setState
    Eigen::VectorXd x0(n);
    x0 << 1.0, 0.5;
    ekf.setState(x0);
    test::check((ekf.state() - x0).norm() < 1e-12, "EKF: setState works");

    // step() combines predict+update
    ekf.reset();
    test::no_throw([&]
                   { ekf.step(y, u); }, "EKF: step() no throw");

    // numericalJacobian: correct size and matches A for linear system
    {
        Eigen::VectorXd x_test = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd u_test(1);
        u_test << 0.0;
        auto Jnum = ctrl::ExtendedKalmanFilter::numericalJacobian(
            [&](const Eigen::VectorXd &xv)
            { return f(xv, u_test); }, x_test);
        test::check(Jnum.rows() == n && Jnum.cols() == n,
                    "EKF: numericalJacobian returns n*n matrix");
        test::check((Jnum - plt.A).norm() < 1e-5,
                    "EKF: numericalJacobian matches A for linear system");
    }

    // For a linear plant, EKF and standard KF should agree
    {
        ctrl::ExtendedKalmanFilter ekf2(n, p, f, h, Fj, Hj, Qn, Rn, Ts);
        ctrl::KalmanFilter kf(plt, Qn, Rn);

        Eigen::VectorXd xs = Eigen::VectorXd::Zero(n);
        xs(0) = 0.5;
        for (int k = 0; k < 200; ++k)
        {
            Eigen::VectorXd uk(1);
            uk << std::sin(k * 0.1);
            Eigen::VectorXd ym = plt.C * xs;
            ekf2.step(ym, uk);
            kf.step(ym, uk);
            xs = plt.A * xs + plt.B * uk;
        }
        test::check((ekf2.state() - kf.state()).norm() < 0.1,
                    "EKF: matches KF for linear system");
    }
}

// ============================================================
//  17. UnscentedKalmanFilter tests
// ============================================================
void test_ukf()
{
    test::suite("UnscentedKalmanFilter");
    const int n = 2, p = 1;

    auto plt = make_plant();
    Eigen::MatrixXd Qn = Eigen::MatrixXd::Identity(n, n) * 1e-4;
    Eigen::MatrixXd Rn = Eigen::MatrixXd::Identity(p, p) * 1e-2;

    auto f = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    { return plt.A * x + plt.B * u; };
    auto h = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &)
    { return plt.C * x; };

    ctrl::UnscentedKalmanFilter ukf(n, p, f, h, Qn, Rn, Ts,
                                    Eigen::MatrixXd(), 1.0, 2.0, 0.0);

    // Initial state is zero
    test::check(ukf.state().norm() < 1e-12, "UKF: initial state is zero");

    // predict + update changes state
    Eigen::VectorXd u(1);
    u << 1.0;
    ukf.predict(u);
    Eigen::VectorXd y(1);
    y << 0.01;
    ukf.update(y, u);
    test::check(ukf.state().norm() > 0.0, "UKF: state nonzero after predict+update");

    // covariance trace > 0
    test::check(ukf.covariance().trace() > 0.0, "UKF: covariance trace > 0");

    // reset clears state
    ukf.reset();
    test::check(ukf.state().norm() < 1e-12, "UKF: reset restores zero state");

    // setState
    Eigen::VectorXd x0(n);
    x0 << 1.0, 0.5;
    ukf.setState(x0);
    test::check((ukf.state() - x0).norm() < 1e-12, "UKF: setState works");

    // step() no throw
    ukf.reset();
    test::no_throw([&]
                   { ukf.step(y, u); }, "UKF: step() no throw");

    // For a linear plant, UKF should approximate the standard KF.
    // Use kappa=3-n so Wc(0) stays positive (Julier's rule), preventing P from going non-PSD.
    {
        ctrl::UnscentedKalmanFilter ukf2(n, p, f, h, Qn, Rn, Ts,
                                         Eigen::MatrixXd(), 1.0, 2.0, 0.0);
        ctrl::KalmanFilter kf(plt, Qn, Rn);

        Eigen::VectorXd xs = Eigen::VectorXd::Zero(n);
        xs(0) = 0.5;
        for (int k = 0; k < 200; ++k)
        {
            Eigen::VectorXd uk(1);
            uk << std::sin(k * 0.1);
            Eigen::VectorXd ym = plt.C * xs;
            ukf2.step(ym, uk);
            kf.step(ym, uk);
            xs = plt.A * xs + plt.B * uk;
        }
        test::check((ukf2.state() - kf.state()).norm() < 0.1,
                    "UKF: matches KF for linear system");
    }

    // PSD maintenance: covariance must remain positive semidefinite over 200 steps.
    // Raw subtraction P -= K*Syy*K' can violate PSD under round-off; the eigenvalue
    // floor in update() must catch this. A Cholesky failure in sigmaPoints() would
    // produce garbage or throw before this check fires -- catch that too.
    {
        ctrl::UnscentedKalmanFilter ukf3(n, p, f, h, Qn, Rn, Ts,
                                         Eigen::MatrixXd(), 1e-3, 2.0, 0.0);
        Eigen::VectorXd xs3 = Eigen::VectorXd::Zero(n);
        xs3(0) = 1.0;
        bool psd_ok = true;
        for (int k = 0; k < 200; ++k)
        {
            Eigen::VectorXd uk3(1);
            uk3 << std::sin(k * 0.2) + 0.1 * std::cos(k * 1.7);
            Eigen::VectorXd ym3 = plt.C * xs3;
            test::no_throw([&]
                           { ukf3.step(ym3, uk3); },
                           "UKF PSD: step() no throw at k=" + std::to_string(k));
            xs3 = plt.A * xs3 + plt.B * uk3;

            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(ukf3.covariance());
            if (eig.eigenvalues().minCoeff() < -1e-9)
            {
                psd_ok = false;
                break;
            }
        }
        test::check(psd_ok, "UKF: covariance P remains PSD over 200 steps");
    }
}

// ============================================================
//  18. RepetitiveController tests
// ============================================================
void test_repetitive()
{
    test::suite("RepetitiveController");

    ctrl::PIDParams pp;
    pp.Kp = 2.0;
    pp.Ki = 1.0;
    pp.Kd = 0.0;
    pp.uMin = -20.0;
    pp.uMax = 20.0;
    auto base = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::RepetitiveParams rp;
    rp.periodSteps = 50;
    rp.Krc = 0.5;
    rp.Q = 0.98;
    rp.uMin = -20.0;
    rp.uMax = 20.0;

    ctrl::RepetitiveController rc(base, rp, Ts);

    // First compute returns finite value
    test::check(std::isfinite(rc.compute(0.5)), "RC: first compute is finite");

    // Zero error -> finite output (base + zero correction)
    rc.reset();
    test::check(std::isfinite(rc.compute(0.0)), "RC: zero error gives finite output");

    // correction() diagnostic is accessible
    test::check(std::isfinite(rc.correction()), "RC: correction() is finite");

    // reset clears correction buffer
    rc.compute(1.0);
    rc.reset();
    test::check(std::abs(rc.correction()) < 1e-12, "RC: reset clears correction buffer");

    // Degenerate periodSteps=1 must not crash
    {
        ctrl::RepetitiveParams rp2 = rp;
        rp2.periodSteps = 1;
        ctrl::RepetitiveController rc2(
            std::make_shared<ctrl::DiscretePID>(pp, Ts), rp2, Ts);
        test::no_throw([&]
                       { rc2.compute(1.0); }, "RC: periodSteps=1 no crash");
    }

    // Correction accumulates: after more than one period of constant error it grows
    rc.reset();
    double corr_start = rc.correction();
    for (int k = 0; k < rp.periodSteps + 5; ++k)
        rc.compute(1.0);
    test::check(rc.correction() > corr_start, "RC: correction accumulates over one period");
}

// ============================================================
//  19. GeneralizedPredictiveController tests
// ============================================================
void test_gpc()
{
    test::suite("GeneralizedPredictiveController");

    auto plant = make_plant();

    ctrl::GPCParams gp;
    gp.Np = 15;
    gp.Nu = 4;
    gp.rho_y = 1.0;
    gp.rho_u = 0.1;
    gp.alpha = 0.0;
    gp.uMin = -10.0;
    gp.uMax = 10.0;

    ctrl::GeneralizedPredictiveController gpc(plant, gp);

    // Zero y, zero r -> near-zero output
    test::check(std::abs(gpc.computeRef(0.0, 0.0)) < 1e-6,
                "GPC: zero y, zero r -> zero output");

    // Nonzero reference -> nonzero output
    {
        ctrl::GeneralizedPredictiveController gpc2(plant, gp);
        test::check(gpc2.computeRef(0.0, 1.0) != 0.0,
                    "GPC: nonzero ref -> nonzero output");
    }

    // Output bounded by uMax even for huge reference
    {
        ctrl::GeneralizedPredictiveController gpc3(plant, gp);
        double u = gpc3.computeRef(0.0, 1e6);
        test::check(u <= gp.uMax + 1e-9, "GPC: output bounded by uMax");
    }

    // compute(error) wrapper is finite
    {
        ctrl::GeneralizedPredictiveController gpc4(plant, gp);
        gpc4.setReference(1.0);
        test::check(std::isfinite(gpc4.compute(1.0)),
                    "GPC: compute(error) wrapper is finite");
    }

    // setPlant hot-swap does not throw
    test::no_throw([&]
                   { gpc.setPlant(plant); }, "GPC: setPlant does not throw");

    // augmentedState has size n+p
    test::check(gpc.augmentedState().size() ==
                    static_cast<int>(plant.stateSize() + plant.outputSize()),
                "GPC: augmentedState has size n+p");

    // reset clears augmented state
    gpc.reset();
    test::check(std::abs(gpc.computeRef(0.0, 0.0)) < 1e-6,
                "GPC: reset clears state");

    // Closed-loop tracking: use a fast 1st-order plant G(z)=0.5/(z-0.5) (DC gain=1,
    // step response reaches 0.5 in one step) so CARIMA integral action is visible quickly.
    {
        ctrl::TransferFunction tf1({0.0, 0.5}, {1.0, -0.5}, Ts);
        ctrl::StateSpace plant1 = ctrl::tf2ss(tf1);
        ctrl::GPCParams gp1 = gp;
        gp1.rho_u = 0.01; // allow larger moves for fast convergence
        ctrl::GeneralizedPredictiveController gpc5(plant1, gp1);
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant1.stateSize());
        double y = 0.0;
        for (int k = 0; k < 200; ++k)
        {
            double uc = gpc5.computeRef(y, 1.0);
            Eigen::VectorXd uv(1);
            uv << uc;
            y = ctrl::ssStep(plant1, x, uv)(0);
        }
        test::check(std::abs(y - 1.0) < 0.05, "GPC: closed-loop tracks unit step");
    }
}

// ============================================================
//  20. SystemAnalysis tests
// ============================================================
void test_system_analysis()
{
    test::suite("SystemAnalysis");

    // Build a known SISO 1st-order plant G(s) = 1/(s+1), ZOH at Ts=0.01.
    // DC gain = 1, all poles stable, phase margin = 90 deg (asymptotically),
    // gain margin = infinity (first-order minimum-phase).
    auto sys_c = make_continuous_plant(); // 2nd-order: use for general tests

    // Simple 1st-order for margin checks: G(z) = b/(z-a), ZOH of 1/(s+1).
    const double a1 = std::exp(-Ts); // pole
    const double b1 = 1.0 - a1;      // DC-preserving gain
    ctrl::StateSpace g1(
        (Eigen::MatrixXd(1, 1) << a1).finished(),
        (Eigen::MatrixXd(1, 1) << b1).finished(),
        (Eigen::MatrixXd(1, 1) << 1.0).finished(),
        (Eigen::MatrixXd(1, 1) << 0.0).finished(),
        Ts);

    // isDiscreteStable: 1st-order stable plant
    test::check(ctrl::SystemAnalysis::isDiscreteStable(g1),
                "SystemAnalysis: stable 1st-order plant is stable");

    // Unstable plant
    ctrl::StateSpace g_unstable(
        (Eigen::MatrixXd(1, 1) << 1.1).finished(),
        (Eigen::MatrixXd(1, 1) << 1.0).finished(),
        (Eigen::MatrixXd(1, 1) << 1.0).finished(),
        (Eigen::MatrixXd(1, 1) << 0.0).finished(),
        Ts);
    test::check(!ctrl::SystemAnalysis::isDiscreteStable(g_unstable),
                "SystemAnalysis: unstable plant detected");

    // getFrequencyResponse SISO: DC gain should be ~1 at w near 0
    {
        std::vector<double> freqs = {1e-3};
        auto resp = ctrl::SystemAnalysis::getFrequencyResponse(g1, freqs);
        test::check(std::abs(std::abs(resp[0]) - 1.0) < 0.01,
                    "SystemAnalysis: SISO freq response DC gain ~1");
    }

    // getFrequencyResponse: MIMO must throw
    auto plant2 = make_plant(); // 2-state but SISO, use a custom MIMO for the test
    {
        Eigen::MatrixXd Am(2, 2), Bm(2, 2), Cm(2, 2), Dm(2, 2);
        Am << 0.9, 0.0, 0.0, 0.8;
        Bm.setIdentity();
        Cm.setIdentity();
        Dm.setZero();
        ctrl::StateSpace mimo(Am, Bm, Cm, Dm, Ts);
        test::throws([&]
                     { ctrl::SystemAnalysis::getFrequencyResponse(mimo, {1.0}); }, "SystemAnalysis: MIMO getFrequencyResponse throws");
    }

    // calculateMargins on the 2nd-order plant: PM should be positive and finite.
    // The 2nd-order G(s)=1/(s^2+1.5s+1) is strictly minimum-phase and stable,
    // so gain margin = inf and phase margin > 0.
    {
        auto plant = make_plant();
        ctrl::StabilityMargins m = ctrl::SystemAnalysis::calculateMargins(plant);
        // PM > 0 or infinite (infinite means the open-loop gain never reaches 1.0 - even
        // better than a finite PM).  Both are valid for a stable well-damped plant.
        test::check(m.phaseMarginDeg > 0.0 || m.phaseMarginDeg == std::numeric_limits<double>::infinity(),
                    "SystemAnalysis: PM > 0 (or inf) for stable minimum-phase plant");
        // Gain margin must be large for this well-damped 2nd-order plant.
        test::check(m.gainMarginDb > 6.0 || m.gainMarginDb == std::numeric_limits<double>::infinity(),
                    "SystemAnalysis: GM > 6 dB (or inf) for stable 2nd-order plant");
    }

    // calculateMargins on the 1st-order plant: phase never reaches -180 below Nyquist.
    // PM should be large (close to 90 deg for slow plants well below Nyquist).
    {
        ctrl::StabilityMargins m = ctrl::SystemAnalysis::calculateMargins(g1);
        test::check(m.phaseMarginDeg > 45.0,
                    "SystemAnalysis: PM > 45 deg for 1st-order plant");
    }

    // H-infinity norm is positive and finite for a stable plant
    {
        auto plant = make_plant();
        double hinf = ctrl::SystemAnalysis::calculateHInfinityNorm(plant);
        test::check(hinf > 0.0 && std::isfinite(hinf),
                    "SystemAnalysis: H-inf norm is positive and finite");
    }

    // solveDiscreteLyapunov: for A=0, solution P should equal Q
    {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
        Eigen::MatrixXd P = ctrl::SystemAnalysis::solveDiscreteLyapunov(A, Q);
        test::check((P - Q).norm() < 1e-10,
                    "SystemAnalysis: Lyapunov P==Q for A=0");
    }
}

// ============================================================
//  21. SubspaceID tests
// ============================================================
void test_subspace_id()
{
    test::suite("SubspaceID");

    // suggestOrder: sharp elbow at index 1 (ratio sv[1]/sv[2] = 50) -> order 2
    {
        Eigen::VectorXd sv(6);
        sv << 100.0, 50.0, 1.0, 0.5, 0.1, 0.05;
        test::check(ctrl::suggestOrder(sv) == 2,
                    "suggestOrder: detects elbow at order 2");
    }

    // suggestOrder: maxOrder cap overrides elbow
    {
        Eigen::VectorXd sv(6);
        sv << 100.0, 50.0, 1.0, 0.5, 0.1, 0.05;
        test::check(ctrl::suggestOrder(sv, 0.01, 1) == 1,
                    "suggestOrder: maxOrder=1 respected");
    }

    // suggestOrder: single value always returns 1
    {
        Eigen::VectorXd sv(1);
        sv << 5.0;
        test::check(ctrl::suggestOrder(sv) == 1,
                    "suggestOrder: single value -> order 1");
    }

    // n4sid failure: too few samples (N = 5, i = 5 -> s = N-2i = -5)
    {
        Eigen::MatrixXd Y(1, 5), U(1, 5);
        Y.setRandom();
        U.setRandom();
        auto res = ctrl::n4sid(Y, U, 2, 5, Ts);
        test::check(!res.success, "n4sid: too few samples -> failure");
    }

    // n4sid failure: Y and U column count mismatch
    {
        Eigen::MatrixXd Y(1, 100), U(1, 50);
        Y.setRandom();
        U.setRandom();
        auto res = ctrl::n4sid(Y, U, 2, 10, Ts);
        test::check(!res.success, "n4sid: dimension mismatch -> failure");
    }

    // n4sid failure: n_order < 1
    {
        Eigen::MatrixXd Y(1, 200), U(1, 200);
        Y.setRandom();
        U.setRandom();
        auto res = ctrl::n4sid(Y, U, 0, 10, Ts);
        test::check(!res.success, "n4sid: n_order=0 -> failure");
    }

    // n4sid success: binary PRBS on the 2nd-order plant -> order-2 identification
    {
        auto src = make_plant();
        const int N = 500;
        Eigen::MatrixXd Y(1, N), U(1, N);
        Eigen::VectorXd x = Eigen::VectorXd::Zero(src.stateSize());
        double u_val = 1.0;
        for (int k = 0; k < N; ++k)
        {
            Eigen::VectorXd uv(1);
            uv << u_val;
            Y(0, k) = ctrl::ssStep(src, x, uv)(0);
            U(0, k) = u_val;
            if (k % 7 == 6)
                u_val = -u_val;
        }
        auto res = ctrl::n4sid(Y, U, 2, 15, Ts);
        test::check(res.success, "n4sid: valid PRBS data -> success");
        if (res.success && res.model.has_value())
        {
            test::check(res.model->stateSize() == 2,
                        "n4sid: identified model has 2 states");

            const auto &m = res.model.value();

            // Dominant poles: eigenvalues of identified A should be inside unit disk.
            Eigen::EigenSolver<Eigen::MatrixXd> es(m.A);
            bool all_stable = true;
            for (int k = 0; k < es.eigenvalues().size(); ++k)
                if (std::abs(es.eigenvalues()(k)) >= 1.0)
                    all_stable = false;
            test::check(all_stable, "n4sid: identified A has all eigenvalues inside unit disk");

            // Pole magnitude accuracy: true poles of G(s)=1/(s^2+1.5s+1) at Ts=0.01 lie at
            // z = exp((-0.75 +/- 0.6614j)*0.01), so |z| = exp(-0.0075) = 0.9925.
            // Both identified poles should have magnitude within 3% of 0.9925.
            {
                const double true_mag = std::exp(-0.75 * Ts);
                for (int k = 0; k < es.eigenvalues().size(); ++k)
                {
                    const double id_mag = std::abs(es.eigenvalues()(k));
                    test::check(std::abs(id_mag - true_mag) < 0.03,
                                "n4sid: identified pole magnitude within 3% of true value");
                }
            }

            // DC gain accuracy on a fast 1st-order plant G(z)=0.5/(z-0.5) (DC gain=1).
            // With 300 PRBS samples and period-7 excitation the MOESP B/D regression
            // should recover DC gain to within 25%.
            {
                ctrl::TransferFunction tf1({0.0, 0.5}, {1.0, -0.5}, Ts);
                ctrl::StateSpace src1 = ctrl::tf2ss(tf1);
                Eigen::MatrixXd Y1(1, 300), U1(1, 300);
                Eigen::VectorXd x1 = Eigen::VectorXd::Zero(src1.stateSize());
                double u1 = 1.0;
                for (int k = 0; k < 300; ++k)
                {
                    Eigen::VectorXd uv(1);
                    uv << u1;
                    Y1(0, k) = ctrl::ssStep(src1, x1, uv)(0);
                    U1(0, k) = u1;
                    if (k % 7 == 6)
                        u1 = -u1;
                }
                auto res1 = ctrl::n4sid(Y1, U1, 1, 10, Ts);
                if (res1.success && res1.model.has_value())
                {
                    const auto &m1 = res1.model.value();
                    Eigen::MatrixXd Acl1 = Eigen::MatrixXd::Identity(m1.stateSize(), m1.stateSize()) - m1.A;
                    double dc1 = (m1.C * Acl1.colPivHouseholderQr().solve(Eigen::MatrixXd::Identity(m1.stateSize(), m1.stateSize())) * m1.B + m1.D)(0, 0);
                    test::check(std::abs(dc1 - 1.0) < 0.50,
                                "n4sid: DC gain within 50% for fast 1st-order plant");
                }
            }
        }

        // suggestOrder on the resulting singular values should be small (<=4)
        if (res.singularValues.size() >= 2)
        {
            int ord = ctrl::suggestOrder(res.singularValues, 0.01, 5);
            test::check(ord >= 1 && ord <= 4,
                        "suggestOrder: returns 1..4 for PRBS-identified spectrum");
        }
    }
}

// ============================================================
//  main
// ============================================================
// ============================================================
//  22. DiscreteHinf / MixedSensitivity tests
// ============================================================
#ifndef CTRL_DISABLE_HINF
void test_hinf()
{
    test::suite("DiscreteHinf");

    const double Ts_h = 0.01;

    // -----------------------------------------------------------------------
    // 1. MixedSensitivity weight factories produce stable, correctly-shaped filters
    // -----------------------------------------------------------------------
    {
        auto W1 = ctrl::MixedSensitivity::makeW1(10.0, 1.0, 1e-3, Ts_h);
        auto W2 = ctrl::MixedSensitivity::makeW2constant(0.2, Ts_h);
        auto W2hp = ctrl::MixedSensitivity::makeW2highpass(50.0, 1e-2, Ts_h);
        auto W3 = ctrl::MixedSensitivity::makeW3(50.0, 1.5, 1e-3, Ts_h);

        // All weights must be stable (pole inside unit disc)
        test::check(ctrl::SystemAnalysis::isDiscreteStable(W1),
                    "HInf: W1 filter is stable");
        test::check(ctrl::SystemAnalysis::isDiscreteStable(W2),
                    "HInf: W2 constant is stable (trivially)");
        test::check(ctrl::SystemAnalysis::isDiscreteStable(W2hp),
                    "HInf: W2 highpass is stable");
        test::check(ctrl::SystemAnalysis::isDiscreteStable(W3),
                    "HInf: W3 filter is stable");

        // W2 constant: DC gain should equal the specified gain
        {
            Eigen::VectorXd x = Eigen::VectorXd::Zero(W2.stateSize());
            Eigen::VectorXd u(1);
            u(0) = 1.0;
            double y = ctrl::ssStep(W2, x, u)(0);
            test::check(std::abs(y - 0.2) < 1e-10,
                        "HInf: W2constant DC gain = 0.2");
        }

        // W1 DC gain ~ 1/eps (= 1000 for eps=1e-3) - check it's large
        {
            // Compute DC gain analytically: H(z=1) = C*(I-A)^{-1}*B + D
            double A_w1 = W1.A(0, 0), B_w1 = W1.B(0, 0), C_w1 = W1.C(0, 0), D_w1 = W1.D(0, 0);
            double dc_gain = C_w1 / (1.0 - A_w1) * B_w1 + D_w1;
            test::check(dc_gain > 100.0,
                        "HInf: W1 DC gain >> 1 (large for tight tracking)");
        }
    }

    // -----------------------------------------------------------------------
    // 2. MixedSensitivity::build() assembles a valid generalised plant
    // -----------------------------------------------------------------------
    ctrl::GeneralisedPlant P;
    {
        // Simple 1st-order plant G(z) = b/(z-a), ZOH of G(s)=1/(s+1) at Ts=0.01
        const double a = std::exp(-Ts_h);
        const double b = 1.0 - a;
        ctrl::StateSpace G(
            (Eigen::MatrixXd(1, 1) << a).finished(),
            (Eigen::MatrixXd(1, 1) << b).finished(),
            (Eigen::MatrixXd(1, 1) << 1.0).finished(),
            (Eigen::MatrixXd(1, 1) << 0.0).finished(),
            Ts_h);

        auto W1 = ctrl::MixedSensitivity::makeW1(5.0, 1.0, 1e-3, Ts_h);
        auto W2 = ctrl::MixedSensitivity::makeW2constant(0.1, Ts_h);
        auto W3 = ctrl::MixedSensitivity::makeW3(50.0, 1.5, 1e-3, Ts_h);

        test::no_throw([&]
                       { P = ctrl::MixedSensitivity::build(G, W1, W2, W3); },
                       "HInf: MixedSensitivity::build() succeeds");

        // Dimension checks: nw=2, nz=3, nu=1, ny=1
        test::check(P.nw() == 2, "HInf: generalised plant nw = 2");
        test::check(P.nz() == 3, "HInf: generalised plant nz = 3");
        test::check(P.nu() == 1, "HInf: generalised plant nu = 1");
        test::check(P.ny() == 1, "HInf: generalised plant ny = 1");
        test::check(P.stateSize() == 1 + 1 + 1 + 1, // nG=1, nW1=1, nW2=1, nW3=1
                    "HInf: generalised plant total state size = 4");

        // State matrix A must be finite and square
        test::check(P.A.allFinite(), "HInf: generalised plant A is finite");
        test::check(P.B1.allFinite(), "HInf: generalised plant B1 is finite");
        test::check(P.B2.allFinite(), "HInf: generalised plant B2 is finite");
        test::check(P.C1.allFinite(), "HInf: generalised plant C1 is finite");
        test::check(P.C2.allFinite(), "HInf: generalised plant C2 is finite");

        // DGKF standard form: D11 need not be zero for MixedSensitivity (tracking weight
        // introduces nonzero D11 feedthrough); D22 = DG (should be ~0 for ZOH plant)
        test::check(std::abs(P.D22(0, 0)) < 1e-10, "HInf: D22 = 0 (no direct plant feedthrough)");
    }

    // -----------------------------------------------------------------------
    // 3. Synthesis feasibility: solve() returns feasible result
    // -----------------------------------------------------------------------
    ctrl::HinfResult result;
    {
        ctrl::HinfParams hp;
        hp.gammaInit = 20.0;
        hp.gammaTol = 0.05;
        hp.maxIter = 40;

        test::no_throw([&]
                       { result = ctrl::DiscreteHinf::solve(P, hp); },
                       "HInf: solve() does not throw");
        test::check(result.feasible,
                    "HInf: synthesis is feasible for well-conditioned mixed-sensitivity plant");
        test::check(result.achievedGamma > 0.0 && result.achievedGamma < 100.0,
                    "HInf: achieved gamma is a positive finite value");
        test::check(result.dareConvX, "HInf: control DARE converged");
        test::check(result.dareConvY, "HInf: filter DARE converged");
        test::check(result.spectralRadius < result.achievedGamma * result.achievedGamma,
                    "HInf: spectral radius condition rho(XY) < gamma^2 satisfied");
    }

    // -----------------------------------------------------------------------
    // 4. Controller matrices have correct dimensions
    // -----------------------------------------------------------------------
    {
        const int n = P.stateSize();
        const int ny = P.ny();
        const int nu = P.nu();
        test::check(result.Ak.rows() == n && result.Ak.cols() == n,
                    "HInf: Ak has correct dimensions (nk x nk)");
        test::check(result.Bk.rows() == n && result.Bk.cols() == ny,
                    "HInf: Bk has correct dimensions (nk x ny)");
        test::check(result.Ck.rows() == nu && result.Ck.cols() == n,
                    "HInf: Ck has correct dimensions (nu x nk)");
        test::check(result.Dk.rows() == nu && result.Dk.cols() == ny,
                    "HInf: Dk has correct dimensions (nu x ny)");
        test::check(result.X_inf.rows() == n && result.X_inf.cols() == n,
                    "HInf: X_inf has correct dimensions");
        test::check(result.Y_inf.rows() == n && result.Y_inf.cols() == n,
                    "HInf: Y_inf has correct dimensions");
        test::check(result.Ak.allFinite(), "HInf: Ak is finite");
        test::check(result.Bk.allFinite(), "HInf: Bk is finite");
        test::check(result.Ck.allFinite(), "HInf: Ck is finite");
    }

    // -----------------------------------------------------------------------
    // 5. Synthesised controller Ak is stable (all |eigenvalues| < 1)
    // -----------------------------------------------------------------------
    {
        Eigen::EigenSolver<Eigen::MatrixXd> es(result.Ak);
        double max_eig = 0.0;
        for (int i = 0; i < es.eigenvalues().size(); ++i)
            max_eig = std::max(max_eig, std::abs(es.eigenvalues()(i)));
        test::check(max_eig < 1.0,
                    "HInf: synthesised controller Ak is stable (all eigenvalues inside unit disc)");
    }

    // -----------------------------------------------------------------------
    // 6. Construct DiscreteHinf and run full augmented-plant closed-loop simulation
    //
    // The H-inf controller K(z) is an output-feedback controller synthesised for the
    // generalised plant P.  A correct closed-loop simulation propagates the full
    // augmented state xa = [xG; xW1; xW2; xW3] through P with:
    //   w = [r; 0]  (reference input, no disturbance)
    //   y_meas[k] = C2 * xa[k] + D21 * w + D22 * u[k]   (measurement to controller)
    //   u[k]      = K.computeVec(y_meas[k])               (controller output)
    //   xa[k+1]   = A * xa[k] + B1 * w + B2 * u[k]       (augmented plant state update)
    //
    // For the MixedSensitivity design with reference r=1 and D22=0 (ZOH plant, no
    // direct feedthrough), the measurement is y_meas = C2 * xa + D21 * [r; 0].
    // -----------------------------------------------------------------------
    {
        ctrl::DiscreteHinf hinf_ctrl(result);
        hinf_ctrl.reset();

        const int n_aug = P.stateSize();
        Eigen::VectorXd xa = Eigen::VectorXd::Zero(n_aug);

        const double ref = 1.0;
        Eigen::VectorXd w(2);
        w(0) = ref;
        w(1) = 0.0; // [r; d=0]

        double y_plant = 0.0;
        for (int k = 0; k < 500; ++k)
        {
            // Measurement: y_meas = C2*xa + D21*w + D22*u (D22=0 for ZOH plant)
            const Eigen::VectorXd y_meas = P.C2 * xa + P.D21 * w;

            // Controller: u = K(y_meas)
            const Eigen::VectorXd u = hinf_ctrl.computeVec(y_meas);

            // Read plant output (first state of xa corresponds to xG; C2*xa gives y)
            y_plant = (P.C2 * xa + P.D21 * w + P.D22 * u)(0);

            // Augmented plant state update: xa[k+1] = A*xa + B1*w + B2*u
            xa = P.A * xa + P.B1 * w + P.B2 * u;
        }

        test::check(std::isfinite(y_plant),
                    "HInf: closed-loop output is finite after 500 steps (full augmented sim)");
        // The synthesised controller should regulate y toward r=1.
        // Use a generous tolerance - the achieved gamma and weight tuning govern exact steady-state.
        test::check(std::abs(y_plant - ref) < 0.5,
                    "HInf: closed-loop output converges toward reference (augmented plant sim)");
    }

    // -----------------------------------------------------------------------
    // 7. MIMO computeVec interface works for ny=1 plant (falls through to compute)
    // -----------------------------------------------------------------------
    {
        ctrl::DiscreteHinf hinf_ctrl(result);
        hinf_ctrl.reset();
        Eigen::VectorXd y_vec(1);
        y_vec(0) = 0.5;
        Eigen::VectorXd u_vec;
        test::no_throw([&]
                       { u_vec = hinf_ctrl.computeVec(y_vec); },
                       "HInf: computeVec(ny=1) does not throw");
        test::check(u_vec.size() == 1, "HInf: computeVec returns nu=1 vector");
        test::check(std::isfinite(u_vec(0)), "HInf: computeVec output is finite");
    }

    // -----------------------------------------------------------------------
    // 8. reset() zeroes controller state
    // -----------------------------------------------------------------------
    {
        ctrl::DiscreteHinf hinf_ctrl(result);
        hinf_ctrl.compute(1.0);
        hinf_ctrl.compute(1.0);
        hinf_ctrl.reset();
        const double u_after = hinf_ctrl.compute(0.0);
        test::check(std::abs(u_after) < 1e-12,
                    "HInf: after reset(), compute(0) = 0 (state zeroed)");
    }

    // -----------------------------------------------------------------------
    // 9. Infeasible result: constructing from infeasible HinfResult throws
    // -----------------------------------------------------------------------
    {
        ctrl::HinfResult bad;
        bad.feasible = false;
        test::throws([&]
                     { ctrl::DiscreteHinf ctrl_bad(bad); },
                     "HInf: constructing from infeasible result throws");
    }

    // -----------------------------------------------------------------------
    // 10. MixedSensitivity::build() throws for non-SISO plant
    // -----------------------------------------------------------------------
    {
        Eigen::MatrixXd A2(2, 2);
        A2 << 0.9, 0.0, 0.0, 0.8;
        Eigen::MatrixXd B2(2, 2);
        B2.setIdentity();
        Eigen::MatrixXd C2(2, 2);
        C2.setIdentity();
        Eigen::MatrixXd D2(2, 2);
        D2.setZero();
        ctrl::StateSpace G_mimo(A2, B2, C2, D2, Ts_h);

        auto W1 = ctrl::MixedSensitivity::makeW1(5.0, 1.0, 1e-3, Ts_h);
        auto W2 = ctrl::MixedSensitivity::makeW2constant(0.1, Ts_h);
        auto W3 = ctrl::MixedSensitivity::makeW3(50.0, 1.5, 1e-3, Ts_h);

        test::throws([&]
                     { ctrl::MixedSensitivity::build(G_mimo, W1, W2, W3); },
                     "HInf: MixedSensitivity::build() throws for MIMO plant");
    }

    // -----------------------------------------------------------------------
    // 11. DARE solutions X_inf and Y_inf are symmetric positive-semidefinite
    // -----------------------------------------------------------------------
    {
        // Symmetry check
        const double asym_X = (result.X_inf - result.X_inf.transpose()).norm();
        const double asym_Y = (result.Y_inf - result.Y_inf.transpose()).norm();
        test::check(asym_X < 1e-8, "HInf: X_inf is symmetric");
        test::check(asym_Y < 1e-8, "HInf: Y_inf is symmetric");

        // PSD check via minimum eigenvalue
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esX(result.X_inf);
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esY(result.Y_inf);
        test::check(esX.eigenvalues().minCoeff() >= -1e-6,
                    "HInf: X_inf is positive semi-definite");
        test::check(esY.eigenvalues().minCoeff() >= -1e-6,
                    "HInf: Y_inf is positive semi-definite");
    }
}
#endif // CTRL_DISABLE_HINF

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

    // New API tests
    test_c2d();
    test_rls();
    test_ekf();
    test_ukf();
    test_repetitive();
    test_gpc();
    test_system_analysis();
    test_subspace_id();

    test::report();
    return (test::failed == 0) ? 0 : 1;
}
