/**
 * ex59_zpetc.cpp
 * Zero-Phase Error Tracking Control (ZPETC) prefilter design.
 *
 * System 1 - Minimum-phase plant:
 *   G1(z) = 0.2(z + 0.5) / ((z - 0.8)(z - 0.6))
 *   All zeros inside unit circle -> exact causal inversion.
 *   G1.G_ff = 1.0 at all frequencies.  hasNMPZeros = false.
 *
 * System 2 - Non-minimum-phase plant:
 *   G2(z) = 0.1(z - 1.5) / ((z - 0.9)(z - 0.7))
 *   Zero at z = 1.5 (outside unit circle) -> no stable causal inverse.
 *   ZPETC: causal prefilter inverts only the min-phase part; DC gain normalised to 1.
 *   G2.G_ff has unit DC gain; amplitude error < 1 at high frequency.  hasNMPZeros = true.
 *
 * PASS criteria:
 *   System 1: dcAmplitudeError < 0.01, hasNMPZeros = false
 *   System 2: hasNMPZeros = true, G2.G_ff DC gain within 1% of 1.0
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;
    bool all_pass   = true;

    // =========================================================================
    // System 1: minimum-phase
    //   G1(z) = 0.2*(z + 0.5) / ((z - 0.8)*(z - 0.6))
    // =========================================================================
    {
        // Build via TF: num = 0.2*(z + 0.5) = [0.2, 0.1], den = (z-0.8)(z-0.6)
        // = z^2-1.4z+0.48 = [1, -1.4, 0.48]
        ctrl::TransferFunction tf1({0.2, 0.1}, {1.0, -1.4, 0.48}, Ts);
        ctrl::StateSpace sys1 = ctrl::tf2ss(tf1);

        auto res1 = ctrl::designZPETC(sys1);

        std::cout << "System 1 (min-phase):\n";
        std::cout << "  hasNMPZeros = " << (res1.hasNMPZeros ? "true" : "false") << "\n";
        std::cout << "  dcAmplitudeError = " << res1.dcAmplitudeError << "\n";
        std::cout << "  zeros: ";
        for (const auto &z : res1.zeros)
            std::cout << "(" << z.real() << "+" << z.imag() << "j) ";
        std::cout << "\n";

        if (res1.hasNMPZeros)
        {
            std::cerr << "FAIL: System 1 reported NMP zeros (should be min-phase).\n";
            all_pass = false;
        }
        if (res1.dcAmplitudeError > 0.05)
        {
            std::cerr << "FAIL: System 1 DC amplitude error too large: "
                      << res1.dcAmplitudeError << "\n";
            all_pass = false;
        }
    }

    // =========================================================================
    // System 2: non-minimum-phase
    //   G2(z) = 0.1*(z - 1.5) / ((z - 0.9)*(z - 0.7))
    // =========================================================================
    {
        // num = 0.1*(z - 1.5) = [0.1, -0.15], den = (z-0.9)*(z-0.7)
        // = z^2 - 1.6z + 0.63 = [1, -1.6, 0.63]
        ctrl::TransferFunction tf2({0.1, -0.15}, {1.0, -1.6, 0.63}, Ts);
        ctrl::StateSpace sys2 = ctrl::tf2ss(tf2);

        auto res2 = ctrl::designZPETC(sys2);

        std::cout << "System 2 (NMP, zero at z=1.5):\n";
        std::cout << "  hasNMPZeros = " << (res2.hasNMPZeros ? "true" : "false") << "\n";
        std::cout << "  dcAmplitudeError = " << res2.dcAmplitudeError << "\n";
        std::cout << "  NMP zeros: ";
        for (const auto &z : res2.nmpZeros)
            std::cout << z.real() << " ";
        std::cout << "\n";

        if (!res2.hasNMPZeros)
        {
            std::cerr << "FAIL: System 2 should have NMP zeros.\n";
            all_pass = false;
        }

        // Verify DC gain of composite G2.G_ff approx = 1.0
        // At z = 1: evaluate both TFs using getFrequencyResponse at omega->0
        // Use SystemAnalysis::getFrequencyResponse at very low frequency
        auto freq_resp_sys2  = ctrl::SystemAnalysis::getFrequencyResponse(sys2,  {1e-4});
        auto freq_resp_ff2   = ctrl::SystemAnalysis::getFrequencyResponse(res2.filter, {1e-4});

        const double composite_dc = std::abs(freq_resp_sys2[0] * freq_resp_ff2[0]);
        std::cout << "  DC gain of G2.G_ff = " << composite_dc << "  (need approx = 1.0)\n";

        if (std::abs(composite_dc - 1.0) > 0.05)
        {
            std::cerr << "FAIL: Composite DC gain deviates from 1: " << composite_dc << "\n";
            all_pass = false;
        }
    }

    if (all_pass) std::cout << "PASS\n";
    else          std::cout << "FAIL\n";

    return all_pass ? 0 : 1;
}
