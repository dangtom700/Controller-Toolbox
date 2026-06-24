/**
 * ex90_notch_filter.cpp
 * Additional Controller Types: NotchFilter removing a known resonance from a synthetic signal.
 *
 * Feeds a synthetic sinusoid at the filter's center frequency (an unwanted resonance): the
 * notch attenuates it almost completely. A sinusoid well outside the notch band passes
 * through largely unchanged.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 1e-4;
    ctrl::NotchFilterParams p;
    p.centerFreqHz = 50.0;
    p.Q = 10.0;

    const int N = 6000; // 30 cycles of the 50Hz resonance - ample settling for Q=10

    ctrl::NotchFilter nf_at(p, Ts);
    double peakAtResonance = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double x = std::sin(2.0 * M_PI * p.centerFreqHz * k * Ts);
        const double y = nf_at.apply(x);
        if (k >= N - 200)
            peakAtResonance = std::max(peakAtResonance, std::fabs(y));
    }

    ctrl::NotchFilter nf_far(p, Ts);
    double peakFarFromResonance = 0.0;
    const double fFar = 200.0;
    const int N2 = 3000;
    for (int k = 0; k < N2; ++k)
    {
        const double x = std::sin(2.0 * M_PI * fFar * k * Ts);
        const double y = nf_far.apply(x);
        if (k >= N2 - 200)
            peakFarFromResonance = std::max(peakFarFromResonance, std::fabs(y));
    }

    std::cout << "Unwanted 50Hz resonance: amplitude 1.0 before, " << peakAtResonance << " after notch\n";
    std::cout << "Unrelated 200Hz content: amplitude 1.0 before, " << peakFarFromResonance << " after notch\n";

    const bool ok = std::isfinite(peakAtResonance) && std::isfinite(peakFarFromResonance)
                   && peakAtResonance < 0.01 && peakFarFromResonance > 0.9;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
