#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <vector>
#include <complex>

namespace ctrl
{

    // Results from calculateMargins(). Infinity means the crossing was not found on
    // the frequency grid [1e-3, pi/Ts] rad/s - a stable system with very high margins
    // or a critically damped system may legitimately have no phase/gain crossover.
    struct StabilityMargins
    {
        double gainMarginDb;    // GM in dB: how much gain can increase before instability
        double phaseMarginDeg;  // PM in degrees: phase above -180 deg at the gain crossover
        double wCrossoverGain;  // Frequency where |G| = 1 (0 dB), i.e., the phase-margin freq
        double wCrossoverPhase; // Frequency where phase = -180 deg, i.e., the gain-margin freq
    };

    class SystemAnalysis
    {
    public:
        // Returns the poles (eigenvalues of A) of the state-space system.
        // For discrete-time systems, stability requires all |poles| < 1.
        static std::vector<std::complex<double>> getPoles(const StateSpace &sys);

        // Returns true iff all eigenvalues of A satisfy |lambda| < 1 (strict stability).
        // Marginally stable systems (|lambda| = 1) return false.
        static bool isDiscreteStable(const StateSpace &sys);

        // Solves the discrete Lyapunov equation:  A * P * A' - P + Q = 0
        // via Kronecker product vectorisation: (I - A\otimesA) * vec(P) = vec(Q).
        // Complexity: O(n^6) due to the n^2 x n^2 linear system - suitable for n <= 10.
        //   At n=15 this is ~170 million flops; at n=20 it is ~1.6 billion.
        //   For n > 10, prefer the Bartels-Stewart algorithm (Golub, Nash & Van Loan 1979),
        //   which reduces complexity to O(n^3) via a Schur decomposition. MATLAB's dlyap()
        //   uses Bartels-Stewart internally for this reason.
        //   The Kronecker approach is retained here for simplicity (single Eigen solve,
        //   no custom Schur loop), but be aware of the performance cliff for larger systems.
        // A must be strictly stable; an unstable A gives a singular (I - A\otimesA).
        // Ref: Bartels & Stewart "Solution of the Matrix Equation AX + XB = C" CACM (1972);
        //      Golub, Nash & Van Loan "A Hessenberg-Schur method for AX+XB=C" IEEE TAC (1979).
        static Eigen::MatrixXd solveDiscreteLyapunov(const Eigen::MatrixXd &A,
                                                     const Eigen::MatrixXd &Q);

        // Frequency response G(e^{j*w*Ts}) at each frequency in freqs [rad/s].
        // SISO only (1 input, 1 output); throws for MIMO plants.
        // Uses the direct formula: G(z) = C*(z*I - A)^{-1}*B + D  at z = e^{j*w*Ts}.
        static std::vector<std::complex<double>> getFrequencyResponse(const StateSpace &sys,
                                                                      const std::vector<double> &freqs);

        // Gain and phase margins for a SISO open-loop plant.
        // Algorithm: coarse logarithmic grid (200 pts) to bracket crossings, then
        // bisection (50 iterations) for accuracy. Phase is continuously unwrapped
        // across grid points, so the result is correct for higher-order and
        // non-minimum-phase plants where the phase can pass through -180 deg multiple times.
        // Returns the worst-case (smallest) margin when multiple crossings exist.
        static StabilityMargins calculateMargins(const StateSpace &sys);

        // Peak H-infinity norm: max over frequency of the largest singular value of G(e^{jwTs}).
        // For SISO plants this equals the peak magnitude. For MIMO it equals the peak
        // induced L2-gain (max singular value). Uses a coarse grid followed by golden-section
        // search for the peak - typical accuracy is better than 0.1% of the true peak.
        static double calculateHInfinityNorm(const StateSpace &sys);
    };

} // namespace ctrl
