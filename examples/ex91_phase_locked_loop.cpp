/**
 * ex91_phase_locked_loop.cpp
 * Additional Controller Types: PhaseLockedLoop tracking a synthetic AC signal's phase/frequency.
 *
 * Feeds a synthetic sinusoid at 50Hz for the first second, then steps the true frequency to
 * 53Hz for the second second, and shows the PLL's frequency estimate re-converging.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 1e-4;
    ctrl::PLLParams p;
    p.nominalFreqHz = 50.0;
    p.Kp = 90.0;
    p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, Ts);

    const int N = 20000; // 2s total: 1s @ 50Hz, then a step to 53Hz for the 2nd second
    double phaseTrue = 0.7; // nonzero initial phase offset
    double fTrue = 50.0;
    for (int k = 0; k < N; ++k)
    {
        if (k == N / 2)
            fTrue = 53.0;
        pll.step(std::sin(phaseTrue));
        phaseTrue += 2.0 * M_PI * fTrue * Ts;

        if (k % 2000 == 0)
            std::cout << "t=" << k * Ts << "s  freq_est=" << pll.frequencyHz()
                      << "Hz  locked=" << pll.locked() << "\n";
    }

    std::cout << "Final frequency estimate: " << pll.frequencyHz() << " Hz (true 53.0 Hz)\n";

    const bool ok = pll.locked() && std::isfinite(pll.frequencyHz())
                   && std::fabs(pll.frequencyHz() - 53.0) < 0.5;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
