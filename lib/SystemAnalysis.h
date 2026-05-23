#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <vector>
#include <complex>

namespace ctrl
{

    struct StabilityMargins
    {
        double gainMarginDb;    // Gain margin in decibels
        double phaseMarginDeg;  // Phase margin in degrees
        double wCrossoverGain;  // Frequency where phase = -180 deg (rad/s)
        double wCrossoverPhase; // Frequency where gain = 0 dB (rad/s)
    };

    class SystemAnalysis
    {
    public:
        // -----------------------------------------------------------------------
        // Stability & Root Locus Tools
        // -----------------------------------------------------------------------
        // Returns the poles (eigenvalues of A) of the state-space system.
        static std::vector<std::complex<double>> getPoles(const StateSpace &sys);

        // Checks if the discrete-time system is strictly stable (all |poles| < 1).
        static bool isDiscreteStable(const StateSpace &sys);

        // -----------------------------------------------------------------------
        // Discrete Lyapunov Solver
        // -----------------------------------------------------------------------
        // Solves the discrete Lyapunov equation: A * P * A^T - P + Q = 0
        // Requires A to be strictly stable.
        static Eigen::MatrixXd solveDiscreteLyapunov(const Eigen::MatrixXd &A,
                                                     const Eigen::MatrixXd &Q);

        // -----------------------------------------------------------------------
        // Frequency Domain & Margins (SISO)
        // -----------------------------------------------------------------------
        // Calculates the frequency response G(e^(j*w*Ts)) over a given array of frequencies (rad/s)
        static std::vector<std::complex<double>> getFrequencyResponse(const StateSpace &sys,
                                                                      const std::vector<double> &freqs);

        // Computes Gain and Phase Margins for a SISO system.
        // Uses a dense grid search over frequency to find crossovers.
        static StabilityMargins calculateMargins(const StateSpace &sys);

        // -----------------------------------------------------------------------
        // Robustness Metrics
        // -----------------------------------------------------------------------
        // Calculates the H-infinity norm (peak magnitude of the frequency response)
        // over a standard frequency grid. For MIMO systems, uses maximum singular value.
        static double calculateHInfinityNorm(const StateSpace &sys);
    };

} // namespace ctrl
