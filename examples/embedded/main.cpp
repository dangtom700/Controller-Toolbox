/**
 * @file main.cpp
 * @brief Embedded subset demo - compiles without Eigen or dynamic allocation.
 *
 * Verify zero Eigen #include directives in this translation unit:
 *   python -c "import re,sys; lines=open('examples/embedded/main.cpp').readlines(); bad=[l for l in lines if re.search(r'^\s*#\s*include.*[Ee]igen',l)]; sys.exit(len(bad))"
 *   # exits 0 (success) if no Eigen includes found
 *
 * Build:
 *   cmake -S . -B build -DCTRL_BUILD_EMBEDDED_ONLY=ON
 *   cmake --build build --target ctrl_embedded
 *   # Or for a regular build that still includes full lib:
 *   g++ -std=c++17 -Ilib examples/embedded/main.cpp -o embedded_demo
 */
#include "lib/embedded/EmbeddedControllers.h"
#include <cstdio>
#include <cmath>

int main()
{
    // --- BasicPID<float> - no Eigen, stack allocated --------------------------
    ctrl::BasicPIDParams<float> pp;
    pp.Kp = 2.0f; pp.Ki = 0.5f; pp.Kd = 0.1f;
    pp.uMin = -100.0f; pp.uMax = 100.0f;
    pp.Kb = 1.0f; pp.N = 10.0f;
    ctrl::BasicPID<float> pid(pp, 0.01f);

    // Simulate first-order plant y[k+1] = 0.9*y[k] + 0.1*u[k], Ts=10ms
    float y = 0.0f, ref = 1.0f;
    for (int k = 0; k < 200; ++k) {
        float u = pid.compute(ref - y);
        y = 0.9f * y + 0.1f * u;
    }
    printf("[BasicPID<float>]  final y = %.4f  (ref = 1.0)\n", (double)y);

    // --- BasicSMC<float> - no Eigen -------------------------------------------
    ctrl::BasicSMCParams<float> sp;
    sp.c_e = 1.0f; sp.c_de = 0.1f; sp.eta = 0.5f; sp.phi = 0.05f;
    sp.uMin = -100.0f; sp.uMax = 100.0f;
    ctrl::BasicSMC<float> smc(sp, 0.01f);

    y = 0.0f;
    for (int k = 0; k < 200; ++k) {
        float u = smc.compute(ref - y);
        y = 0.9f * y + 0.1f * u;
    }
    printf("[BasicSMC<float>]  final y = %.4f  (ref = 1.0)\n", (double)y);

    // --- DiscreteIntegrator<float> --------------------------------------------
    ctrl_embedded::DiscreteIntegrator<float> integrator(0.01f);
    float integral = 0.0f;
    for (int k = 0; k < 100; ++k)
        integral = integrator.integrate(1.0f);  // integrate constant 1.0 for 1 second
    printf("[DiscreteIntegrator<float>]  integral of 1.0 over 1s = %.4f  (expected ~1.0)\n", (double)integral);

    // --- FixedRateFilter<float, 2> - second-order LPF -------------------------
    ctrl_embedded::FixedRateFilter<float, 2> lpf(10.0f, 0.001f);  // 10 Hz cutoff, 1 kHz sample
    float step_out = 0.0f;
    for (int k = 0; k < 1000; ++k)
        step_out = lpf.filter(1.0f);
    printf("[FixedRateFilter<float,2>]  step response after 1s = %.4f  (expected ~1.0)\n", (double)step_out);

    // --- RingBuffer<float, 8> - FIFO ------------------------------------------
    ctrl_embedded::RingBuffer<float, 8> buf;
    for (int i = 0; i < 5; ++i) buf.push((float)i);
    float v0 = 0.0f, v4 = 0.0f;
    buf.pop(v0);
    v4 = buf.peek(buf.size() - 1);
    printf("[RingBuffer<float,8>]  pop() = %.0f  peek(last) = %.0f  (expected 0, 4)\n", (double)v0, (double)v4);

    return 0;
}
