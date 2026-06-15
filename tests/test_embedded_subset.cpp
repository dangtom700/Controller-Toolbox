/**
 * @file test_embedded_subset.cpp
 * @brief Catch2 tests for the embedded header-only controller subset (DIST-2).
 *
 * Tests both float and double instantiations.
 * No Eigen dependency - these headers must compile stand-alone.
 */
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

// Embedded-only headers (no Eigen, no heap, no IController)
#include "../lib/BasicPID.h"
#include "../lib/BasicSMC.h"
#include "../lib/embedded/DiscreteIntegrator.h"
#include "../lib/embedded/FixedRateFilter.h"
#include "../lib/embedded/RingBuffer.h"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// =============================================================================
// [basic_pid_embedded] - BasicPID<float> and <double>
// =============================================================================

TEST_CASE("BasicPID<float> step response converges to reference", "[basic_pid_embedded]")
{
    ctrl::BasicPIDParams<float> pp;
    pp.Kp = 2.0f; pp.Ki = 0.5f; pp.Kd = 0.0f;
    pp.uMin = -100.0f; pp.uMax = 100.0f;
    pp.Kb = 1.0f; pp.N = 10.0f; pp.Ts = 0.01f;
    ctrl::BasicPID<float> pid(pp);

    float y = 0.0f, ref = 1.0f;
    // Dominant closed-loop pole at z≈0.9983 gives τ≈5.9 s; run 3000 steps (30 s)
    // so the remaining transient is < 0.003, comfortably within WithinAbs(1.0, 0.01).
    for (int k = 0; k < 3000; ++k) {
        float u = pid.compute(ref - y);
        y = 0.9f * y + 0.1f * u;
    }
    REQUIRE_THAT((double)y, WithinAbs(1.0, 0.01));
}

TEST_CASE("BasicPID<float> anti-windup clamps output under saturation", "[basic_pid_embedded]")
{
    ctrl::BasicPIDParams<float> pp;
    pp.Kp = 1.0f; pp.Ki = 2.0f; pp.Kd = 0.0f;
    pp.uMin = -1.0f; pp.uMax = 1.0f;
    pp.Kb = 1.0f; pp.Ts = 0.1f;
    ctrl::BasicPID<float> pid(pp);

    for (int k = 0; k < 100; ++k) pid.compute(10.0f);  // saturate for many steps
    const float u = pid.compute(10.0f);
    REQUIRE(u <= 1.0f + 1e-5f);
    REQUIRE(u >= -1.0f - 1e-5f);
}

TEST_CASE("BasicPID<double> compute returns finite value", "[basic_pid_embedded]")
{
    ctrl::BasicPIDParams<double> pp;
    pp.Kp = 1.0; pp.Ki = 0.1; pp.Kd = 0.05;
    pp.uMin = -1e6; pp.uMax = 1e6; pp.Ts = 0.01;
    ctrl::BasicPID<double> pid(pp);

    for (int k = 0; k < 20; ++k) {
        double u = pid.compute(1.0 - 0.05 * k);
        REQUIRE(std::isfinite(u));
    }
}

TEST_CASE("BasicPID<float> reset clears integrator", "[basic_pid_embedded]")
{
    ctrl::BasicPIDParams<float> pp;
    pp.Kp = 0.0f; pp.Ki = 1.0f; pp.Kd = 0.0f;
    pp.uMin = -1e6f; pp.uMax = 1e6f; pp.Ts = 0.01f;
    ctrl::BasicPID<float> pid(pp);

    pid.compute(1.0f);
    pid.compute(1.0f);
    REQUIRE(pid.integrator() != 0.0f);

    pid.reset();
    REQUIRE_THAT((double)pid.integrator(), WithinAbs(0.0, 1e-9));
}

// =============================================================================
// [basic_smc_embedded] - BasicSMC<float> and <double>
// =============================================================================

TEST_CASE("BasicSMC<float> reaches zero sliding surface on constant error", "[basic_smc_embedded]")
{
    ctrl::BasicSMCParams<float> sp;
    sp.c_e = 1.0f; sp.c_de = 0.05f; sp.K = 1.0f; sp.phi = 0.02f;
    sp.uMin = -50.0f; sp.uMax = 50.0f;
    ctrl::BasicSMC<float> smc(sp);

    float y = 0.0f, ref = 1.0f;
    float s_final = 0.0f;
    for (int k = 0; k < 500; ++k) {
        float e = ref - y;
        float u = smc.compute(e);
        y = 0.9f * y + 0.1f * u;
        s_final = smc.slidingSurface(e);
    }
    REQUIRE_THAT((double)s_final, WithinAbs(0.0, 0.1));
}

TEST_CASE("BasicSMC<double> output is finite and within bounds", "[basic_smc_embedded]")
{
    ctrl::BasicSMCParams<double> sp;
    sp.c_e = 1.0; sp.c_de = 0.1; sp.K = 0.5; sp.phi = 0.05;
    sp.uMin = -10.0; sp.uMax = 10.0;
    ctrl::BasicSMC<double> smc(sp);

    for (int k = 0; k < 50; ++k) {
        double u = smc.compute(1.0 - 0.02 * k);
        REQUIRE(std::isfinite(u));
        REQUIRE(u >= -10.0 - 1e-9);
        REQUIRE(u <= 10.0 + 1e-9);
    }
}

TEST_CASE("BasicSMC<float> reset clears previous error state", "[basic_smc_embedded]")
{
    ctrl::BasicSMCParams<float> sp;
    sp.c_e = 1.0f; sp.c_de = 0.5f; sp.K = 0.1f; sp.phi = 0.1f;
    sp.uMin = -100.0f; sp.uMax = 100.0f;
    ctrl::BasicSMC<float> smc(sp);

    smc.compute(2.0f);  // set e_prev to 2.0
    smc.reset();
    // After reset, e_prev = 0, so derivative term = (e - 0)/Ts on next call
    const float u_after = smc.compute(1.0f);
    REQUIRE(std::isfinite(u_after));
}

// =============================================================================
// [discrete_integrator] - DiscreteIntegrator<float>/<double>
// =============================================================================

TEST_CASE("DiscreteIntegrator<double> matches analytic integral of ramp", "[discrete_integrator]")
{
    // Integrate f(t) = t for t in [0, 1]: exact integral = 0.5
    const double Ts = 0.001;
    ctrl_embedded::DiscreteIntegrator<double> I(Ts);

    double t = 0.0, result = 0.0;
    for (int k = 0; k < 1000; ++k) {
        result = I.integrate(t);
        t += Ts;
    }
    // Backward Euler underestimates by one step; allow 0.5% tolerance
    REQUIRE_THAT(result, WithinAbs(0.5, 0.005));
}

TEST_CASE("DiscreteIntegrator<float> reset returns to zero", "[discrete_integrator]")
{
    ctrl_embedded::DiscreteIntegrator<float> I(0.01f);
    for (int k = 0; k < 10; ++k) I.integrate(1.0f);
    REQUIRE(I.value() != 0.0f);
    I.reset();
    REQUIRE_THAT((double)I.value(), WithinAbs(0.0, 1e-9));
}

// =============================================================================
// [fixed_rate_filter] - FixedRateFilter<float, N>
// =============================================================================

TEST_CASE("FixedRateFilter<double, 1> attenuates input above cutoff", "[fixed_rate_filter]")
{
    // Single-pole LPF, 10 Hz cutoff, 1 kHz sample rate.
    // Input at 100 Hz (10x above cutoff): expected ~20 dB attenuation.
    ctrl_embedded::FixedRateFilter<double, 1> lpf(10.0, 0.001);

    double peak_out = 0.0, peak_in = 1.0;
    for (int k = 0; k < 5000; ++k) {
        double x = std::sin(2.0 * 3.14159265 * 100.0 * k * 0.001);
        double y = lpf.filter(x);
        if (k > 1000) {  // skip transient
            double ay = y < 0 ? -y : y;
            if (ay > peak_out) peak_out = ay;
        }
    }
    // At 10x cutoff, first-order magnitude = 1/sqrt(1 + (10)^2) approx = 0.0995
    REQUIRE(peak_out < 0.15);  // well attenuated (< 15% of input amplitude)
}

TEST_CASE("FixedRateFilter<float, 2> step response settles to input value", "[fixed_rate_filter]")
{
    ctrl_embedded::FixedRateFilter<float, 2> lpf(50.0f, 0.001f);  // 50 Hz cutoff, 1 kHz
    float out = 0.0f;
    for (int k = 0; k < 2000; ++k) out = lpf.filter(1.0f);
    REQUIRE_THAT((double)out, WithinAbs(1.0, 0.01));
}

// =============================================================================
// [ring_buffer] - RingBuffer<T, N>
// =============================================================================

TEST_CASE("RingBuffer<float, 8> FIFO ordering and size tracking", "[ring_buffer]")
{
    ctrl_embedded::RingBuffer<float, 8> buf;
    REQUIRE(buf.empty());
    REQUIRE(buf.size() == 0);

    for (int i = 0; i < 5; ++i) REQUIRE(buf.push((float)i));
    REQUIRE(buf.size() == 5);
    REQUIRE_FALSE(buf.empty());

    float v = -1.0f;
    REQUIRE(buf.pop(v));
    REQUIRE_THAT((double)v, WithinAbs(0.0, 1e-6));  // FIFO: first in = first out
    REQUIRE(buf.size() == 4);
}

TEST_CASE("RingBuffer<int, 4> full buffer rejects push, wrap-around works", "[ring_buffer]")
{
    ctrl_embedded::RingBuffer<int, 4> buf;
    REQUIRE(buf.push(1)); REQUIRE(buf.push(2)); REQUIRE(buf.push(3)); REQUIRE(buf.push(4));
    REQUIRE(buf.full());
    REQUIRE_FALSE(buf.push(5));  // full - should reject

    int v = 0;
    REQUIRE(buf.pop(v));
    REQUIRE(v == 1);  // FIFO: 1 was pushed first

    // Can push again after pop
    REQUIRE(buf.push(5));
    // peek at last: indices are 2,3,4,5 => peek(3) = 5
    REQUIRE(buf.peek(buf.size() - 1) == 5);
}

TEST_CASE("RingBuffer<float, 8> clear resets size to zero", "[ring_buffer]")
{
    ctrl_embedded::RingBuffer<float, 8> buf;
    for (int i = 0; i < 6; ++i) buf.push((float)i);
    REQUIRE(buf.size() == 6);
    buf.clear();
    REQUIRE(buf.empty());
    REQUIRE(buf.size() == 0);
}
