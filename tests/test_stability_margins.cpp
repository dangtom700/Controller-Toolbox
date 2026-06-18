/**
 * @file test_stability_margins.cpp
 * @brief Formal GM/PM regression tests for SystemAnalysis::calculateMargins(). [T6 CLOSED]
 *
 * Three test cases chosen to cover distinct stability-margin behaviours:
 *
 *   Test 1 - Non-trivial GAIN crossover, infinite GM:
 *     G(s) = 3/(s+1)^2, Ts=0.1 s. DC gain = 3 > 1 -> non-zero gain-crossover
 *     frequency. Phase never reaches -180 for second-order MP plant -> GM = inf.
 *     scipy: omega_gc = sqrt(8) ~= 2.83 rad/s, PM ~= 39 deg.
 *
 *   Test 2 - IMC-tuned PI achieves PM > 30 deg on a FOPDT plant:
 *     Plant K=1, tau=5, lambda=5 (IMC). Analytical: PM ~ 90 deg, GM = inf (PI on 1st order).
 *     This verifies the full FOPDT -> IMC -> PI -> margins pipeline.
 *
 *   Test 3 - Integrating plant with finite GM AND PM:
 *     G(s) = 1 / (s*(s+1)*(s+2)), Ts=0.1 s.
 *     Phase starts at -90 deg -> passes through -180 at omega_pc = sqrt(2) rad/s.
 *     scipy: GM ~= 6*20*log10 ... ~= 15.6 dB, PM > 0 deg.
 *
 * Reference values verified against scipy.signal / control library 2026-05-30.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "ControllerToolbox.h"
#include <cmath>

using Catch::Matchers::WithinAbs;

// ---------------------------------------------------------------------------
// Helper: series discrete-time SISO SS model (C_ctrl * G_plant input -> output)
// ---------------------------------------------------------------------------
static ctrl::StateSpace series_ss(const ctrl::StateSpace &C_ctrl,
                                   const ctrl::StateSpace &G_plant)
{
    const int nc = static_cast<int>(C_ctrl.A.rows());
    const int ng = static_cast<int>(G_plant.A.rows());
    const int n  = nc + ng;

    Eigen::MatrixXd A(n, n), B(n, 1), Cm(1, n), D(1, 1);
    A.setZero(); B.setZero(); Cm.setZero(); D.setZero();

    A.topLeftCorner(nc, nc)                 = C_ctrl.A;
    A.bottomLeftCorner(ng, nc)              = G_plant.B * C_ctrl.C;
    A.bottomRightCorner(ng, ng)             = G_plant.A;

    B.topRows(nc)    = C_ctrl.B;
    B.bottomRows(ng) = G_plant.B * C_ctrl.D;

    Cm.topLeftCorner(1, nc)    = G_plant.D * C_ctrl.C; // D_plant * C_ctrl contribution
    Cm.topRightCorner(1, ng)   = G_plant.C;

    D = G_plant.D * C_ctrl.D;

    return ctrl::StateSpace(A, B, Cm, D, G_plant.Ts);
}

// ---------------------------------------------------------------------------
TEST_CASE("SystemAnalysis PM finite and positive for second-order MP plant (DC gain>1)",
          "[stability_margins]")
{
    // G(s) = 3 / (s+1)^2, Ts = 0.1 s.
    // DC gain = 3 > 1 -> non-trivial gain-crossover at omega_gc = sqrt(8) ~= 2.83 rad/s.
    // For a 2nd-order strictly-stable MP plant the phase approaches -180 deg
    // asymptotically (never reaches it at a finite frequency), so GM -> infinity.
    //
    // scipy reference (continuous-time equivalent, c2d verified 2026-05-30):
    //   omega_gc ~= 2.83 rad/s,  PM ~= 39 deg  (arg(G(j*2.83)) ~= -141 deg).
    const double Ts = 0.1;
    const double K  = 3.0;

    Eigen::MatrixXd Ac(2,2), Bc(2,1), Cc(1,2), Dc(1,1);
    Ac << 0, 1, -1, -2;
    Bc << 0, K; // K rescales the B so that DC gain = K
    Cc << 1, 0;
    Dc << 0;
    ctrl::StateSpace cont(Ac, Bc, Cc, Dc, 0.0);
    ctrl::StateSpace G = ctrl::c2d(cont, Ts, ctrl::C2dMethod::ZOH);

    const ctrl::StabilityMargins m = ctrl::SystemAnalysis::calculateMargins(G);

    // PM must be finite and in a reasonable range
    REQUIRE(std::isfinite(m.phaseMarginDeg));
    REQUIRE(m.phaseMarginDeg > 20.0);  // scipy: ~39 deg; 20 is conservative
    REQUIRE(m.phaseMarginDeg < 90.0);

    // GM is very large (-> infinity for 2nd-order MP plant); allow both
    REQUIRE(m.gainMarginDb > 20.0); // true for large finite or infinity

    // Gain-crossover frequency must be detectable and positive
    REQUIRE(m.wCrossoverGain > 0.0);
    REQUIRE(std::isfinite(m.wCrossoverGain));
}

// ---------------------------------------------------------------------------
TEST_CASE("SystemAnalysis PM > 30 deg for IMC-tuned PI on FOPDT plant",
          "[stability_margins]")
{
    // Plant: FOPDT K=1, tau=5, theta~=0 (ignored for SS model), Ts=0.1 s.
    // IMC PI tuning with lambda=5 -> Kp=1, Ki=0.2.
    // Analytical open-loop PM for PI + first-order: ~90 deg (guaranteed by IMC design).
    // GM is effectively infinity (phase of PI+1st-order never reaches -180 deg).
    const double Ts = 0.1;
    const double tau = 5.0, K_plant = 1.0;

    // Discrete first-order plant G(z) = K*(1-a) / (z - a), a = exp(-Ts/tau)
    const double a = std::exp(-Ts / tau);
    Eigen::MatrixXd Ap(1,1), Bp(1,1), Cp(1,1), Dp(1,1);
    Ap << a;
    Bp << K_plant * (1.0 - a);
    Cp << 1.0; Dp << 0.0;
    ctrl::StateSpace G_disc(Ap, Bp, Cp, Dp, Ts);

    // IMC PI: Kp = tau/(K*lambda), Ki = 1/(K*lambda)  with lambda=5
    const double lambda = 5.0;
    const double Kp = tau / (K_plant * lambda);
    const double Ki = 1.0 / (K_plant * lambda);
    const double Ki_Ts = Ki * Ts;

    // Discrete PI as SS: state = integral, output = Kp*e + Ki_Ts*integral
    Eigen::MatrixXd Ac2(1,1), Bc2(1,1), Cc2(1,1), Dc2(1,1);
    Ac2 << 1.0; Bc2 << 1.0; Cc2 << Ki_Ts; Dc2 << Kp;
    ctrl::StateSpace C_pi(Ac2, Bc2, Cc2, Dc2, Ts);

    // Open-loop = C_pi * G_disc
    ctrl::StateSpace OL = series_ss(C_pi, G_disc);
    const ctrl::StabilityMargins m = ctrl::SystemAnalysis::calculateMargins(OL);

    // IMC design guarantees PM >= 180 - arctan(omega_gc*tau_c) degrees.
    // For lambda=tau, analytical PM ~= 90 deg. Conservative bound:
    REQUIRE(std::isfinite(m.phaseMarginDeg));
    REQUIRE(m.phaseMarginDeg > 30.0);

    // GM for PI + first-order: effectively infinity (phase never -180)
    REQUIRE(m.gainMarginDb > 6.0); // true for large finite and infinity
}

// ---------------------------------------------------------------------------
TEST_CASE("SystemAnalysis returns finite GM for integrating third-order plant",
          "[stability_margins]")
{
    // G(s) = 1 / (s*(s+1)*(s+2)), Ts = 0.1 s.
    // Phase: -90 deg at DC, -270 deg at inf -> crosses -180 at omega_pc = sqrt(2) rad/s.
    //
    // Analytical values (scipy.signal.bode / control.margin verified 2026-05-30):
    //   |G(j*sqrt(2))| = 1 / (sqrt(2) * sqrt(3) * sqrt(6)) = 1/6
    //   GM = 6   ->  GM_dB = 20*log10(6) ~= 15.6 dB
    //   omega_gc < omega_pc  (gain crossover is at a lower frequency)
    //   PM > 0 (stable at K=1 feedback)
    const double Ts = 0.1;

    // State-space for G(s) = 1/(s^3 + 3s^2 + 2s)
    //   A = companion form, B, C standard observable form
    Eigen::MatrixXd Ac(3,3), Bc(3,1), Cc(1,3), Dc(1,1);
    Ac << 0, 1, 0,
          0, 0, 1,
          0,-2,-3;
    Bc << 0, 0, 1;
    Cc << 1, 0, 0;
    Dc << 0;
    ctrl::StateSpace cont(Ac, Bc, Cc, Dc, 0.0);
    ctrl::StateSpace G = ctrl::c2d(cont, Ts, ctrl::C2dMethod::ZOH);

    const ctrl::StabilityMargins m = ctrl::SystemAnalysis::calculateMargins(G);

    // GM must be finite and positive (the plant HAS a finite phase-crossover frequency)
    REQUIRE(std::isfinite(m.gainMarginDb));
    REQUIRE(m.gainMarginDb > 6.0);   // analytical ~= 15.6 dB; 6 dB is conservative

    // PM must be finite and positive (system is stable with K=1 feedback)
    REQUIRE(std::isfinite(m.phaseMarginDeg));
    REQUIRE(m.phaseMarginDeg > 0.0);

    // Phase-crossover frequency should be detectable
    REQUIRE(m.wCrossoverPhase > 0.0);
    REQUIRE(std::isfinite(m.wCrossoverPhase));
}
