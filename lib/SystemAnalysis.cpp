#include "SystemAnalysis.h"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <stdexcept>

static constexpr double kPi = 3.14159265358979323846;

namespace ctrl
{

    std::vector<std::complex<double>> SystemAnalysis::getPoles(const StateSpace &sys)
    {
        Eigen::EigenSolver<Eigen::MatrixXd> es(sys.A);
        Eigen::VectorXcd eigs = es.eigenvalues();
        std::vector<std::complex<double>> poles(eigs.size());
        for (int i = 0; i < eigs.size(); ++i)
        {
            poles[i] = eigs[i];
        }
        return poles;
    }

    bool SystemAnalysis::isDiscreteStable(const StateSpace &sys)
    {
        auto poles = getPoles(sys);
        for (const auto &p : poles)
        {
            if (std::abs(p) >= 1.0)
            {
                return false;
            }
        }
        return true;
    }

    Eigen::MatrixXd SystemAnalysis::solveDiscreteLyapunov(const Eigen::MatrixXd &A,
                                                          const Eigen::MatrixXd &Q)
    {
        // Solves A * P * A^T - P + Q = 0 by vectorizing:
        // (I - A \otimes A) vec(P) = vec(Q)
        int n = A.rows();
        if (n != A.cols() || n != Q.rows() || n != Q.cols())
        {
            throw std::invalid_argument("solveDiscreteLyapunov: Matrix dimensions mismatch.");
        }

        // Kronecker product: A \otimes A
        Eigen::MatrixXd kronAA(n * n, n * n);
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                kronAA.block(i * n, j * n, n, n) = A(i, j) * A;
            }
        }

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n * n, n * n);
        Eigen::MatrixXd M = I - kronAA;

        // vec(Q)
        Eigen::VectorXd vecQ(n * n);
        for (int j = 0; j < n; ++j)
        {
            for (int i = 0; i < n; ++i)
            {
                vecQ(j * n + i) = Q(i, j);
            }
        }

        // Solve M * vec(P) = vec(Q)
        Eigen::VectorXd vecP = M.colPivHouseholderQr().solve(vecQ);

        // Reshape back to matrix P
        Eigen::MatrixXd P(n, n);
        for (int j = 0; j < n; ++j)
        {
            for (int i = 0; i < n; ++i)
            {
                P(i, j) = vecP(j * n + i);
            }
        }
        return P;
    }

    std::vector<std::complex<double>> SystemAnalysis::getFrequencyResponse(const StateSpace &sys,
                                                                           const std::vector<double> &freqs)
    {
        if (sys.C.rows() != 1 || sys.B.cols() != 1)
            throw std::invalid_argument(
                "getFrequencyResponse: system must be SISO (C rows=" +
                std::to_string(sys.C.rows()) + ", B cols=" +
                std::to_string(sys.B.cols()) + "). For MIMO use calculateHInfinityNorm().");

        std::vector<std::complex<double>> response(freqs.size());
        const int n = sys.A.rows();
        Eigen::MatrixXcd I = Eigen::MatrixXcd::Identity(n, n);

        const Eigen::MatrixXcd Ac = sys.A.cast<std::complex<double>>();
        const Eigen::MatrixXcd Bc = sys.B.cast<std::complex<double>>();
        const Eigen::MatrixXcd Cc = sys.C.cast<std::complex<double>>();
        const Eigen::MatrixXcd Dc = sys.D.cast<std::complex<double>>();

        for (size_t i = 0; i < freqs.size(); ++i)
        {
            const std::complex<double> z = std::polar(1.0, freqs[i] * sys.Ts);
            const Eigen::MatrixXcd G = Cc * (z * I - Ac).inverse() * Bc + Dc;
            response[i] = G(0, 0);
        }
        return response;
    }

    StabilityMargins SystemAnalysis::calculateMargins(const StateSpace &sys)
    {
        // Precompute complex matrices once for all frequency evaluations
        const int n = sys.A.rows();
        const Eigen::MatrixXcd I  = Eigen::MatrixXcd::Identity(n, n);
        const Eigen::MatrixXcd Ac = sys.A.cast<std::complex<double>>();
        const Eigen::MatrixXcd Bc = sys.B.cast<std::complex<double>>();
        const Eigen::MatrixXcd Cc = sys.C.cast<std::complex<double>>();
        const Eigen::MatrixXcd Dc = sys.D.cast<std::complex<double>>();

        // Evaluate G(e^{jwTs}) at a single frequency - shared by coarse scan and bisection
        auto evalAt = [&](double w) -> std::complex<double>
        {
            std::complex<double> z = std::polar(1.0, w * sys.Ts);
            Eigen::MatrixXcd G = Cc * (z * I - Ac).inverse() * Bc + Dc;
            return G(0, 0);
        };

        // Step 1: Coarse logarithmic grid to locate crossover brackets (~200 evals).
        // Phase is unwrapped continuously across consecutive grid points so that
        // sign-change detection for the -180° crossing works correctly for higher-order
        // plants and non-minimum-phase systems.
        const double w_min  = 1e-3;
        const double w_max  = kPi / sys.Ts;
        const int    coarse = 200;

        std::vector<double> fw(coarse), mag(coarse), pha(coarse);
        pha[0] = std::arg(evalAt(w_min * std::pow(w_max / w_min, 0.0))) * 180.0 / kPi;
        fw[0]  = w_min;
        mag[0] = std::abs(evalAt(fw[0]));
        for (int i = 1; i < coarse; ++i)
        {
            fw[i]  = w_min * std::pow(w_max / w_min, static_cast<double>(i) / (coarse - 1));
            auto g = evalAt(fw[i]);
            mag[i] = std::abs(g);
            // Unwrap: add the smallest possible ±360° shift to keep the phase continuous
            double raw  = std::arg(g) * 180.0 / kPi;
            double diff = raw - pha[i - 1];
            while (diff >  180.0) diff -= 360.0;
            while (diff < -180.0) diff += 360.0;
            pha[i] = pha[i - 1] + diff;
        }

        // Step 2: Bisection refinement over each bracket - converges in ~50 evals per crossing.
        // Bisection tracks the unwrapped phase by carrying the last known unwrapped value at
        // the left endpoint (p_lo) and shifting the midpoint evaluation by ±360° to match.
        const int bisect_iters = 50;

        StabilityMargins margins;
        margins.gainMarginDb    = std::numeric_limits<double>::infinity();
        margins.phaseMarginDeg  = std::numeric_limits<double>::infinity();
        margins.wCrossoverGain  = 0.0;
        margins.wCrossoverPhase = 0.0;

        // Helper: unwrap a raw arg() result relative to a reference unwrapped value.
        auto unwrapRelative = [](double raw_deg, double ref_deg) -> double {
            double diff = raw_deg - ref_deg;
            while (diff >  180.0) diff -= 360.0;
            while (diff < -180.0) diff += 360.0;
            return ref_deg + diff;
        };

        // Gain margin: locate every phase = -180^\circ crossing; keep worst-case (smallest GM)
        for (int i = 0; i + 1 < coarse; ++i)
        {
            if ((pha[i] - (-180.0)) * (pha[i + 1] - (-180.0)) >= 0.0) continue;

            double lo = fw[i], hi = fw[i + 1];
            double p_lo = pha[i];
            for (int k = 0; k < bisect_iters; ++k)
            {
                double mid   = std::sqrt(lo * hi); // geometric midpoint preserves log scale
                double p_mid = unwrapRelative(std::arg(evalAt(mid)) * 180.0 / kPi, p_lo);
                if ((p_lo - (-180.0)) * (p_mid - (-180.0)) < 0.0)
                    hi = mid;
                else { lo = mid; p_lo = p_mid; }
            }
            const double w_cross   = std::sqrt(lo * hi);
            const double mag_cross = std::abs(evalAt(w_cross));
            const double gm_cand   = (mag_cross >= 1e-300)
                                         ? -20.0 * std::log10(mag_cross)
                                         : std::numeric_limits<double>::infinity();
            if (gm_cand < margins.gainMarginDb)
            {
                margins.gainMarginDb   = gm_cand;
                margins.wCrossoverGain = w_cross;
            }
        }

        // Phase margin: locate every |G| = 1 crossing; keep worst-case (smallest PM)
        for (int i = 0; i + 1 < coarse; ++i)
        {
            if ((mag[i] - 1.0) * (mag[i + 1] - 1.0) >= 0.0) continue;

            double lo = fw[i], hi = fw[i + 1];
            double m_lo = mag[i];
            for (int k = 0; k < bisect_iters; ++k)
            {
                double mid   = std::sqrt(lo * hi);
                double m_mid = std::abs(evalAt(mid));
                if ((m_lo - 1.0) * (m_mid - 1.0) < 0.0)
                    hi = mid;
                else { lo = mid; m_lo = m_mid; }
            }
            // Find the unwrapped phase at the crossing by locating the bracket in the
            // coarse grid and unwrapping relative to the left grid neighbour.
            const double w_cross    = std::sqrt(lo * hi);
            const double phase_raw  = std::arg(evalAt(w_cross)) * 180.0 / kPi;
            const double phase_cross = unwrapRelative(phase_raw, pha[i]);
            const double pm_cand    = 180.0 + phase_cross;
            if (pm_cand < margins.phaseMarginDeg)
            {
                margins.phaseMarginDeg  = pm_cand;
                margins.wCrossoverPhase = w_cross;
            }
        }

        return margins;
    }

    double SystemAnalysis::calculateHInfinityNorm(const StateSpace &sys)
    {
        // Step 1: coarse logarithmic grid to locate the approximate peak frequency.
        const double w_min      = 1e-3;
        const double w_max      = kPi / sys.Ts;
        const int    num_points = 500;

        const int n = sys.A.rows();
        const Eigen::MatrixXcd I  = Eigen::MatrixXcd::Identity(n, n);
        const Eigen::MatrixXcd Ac = sys.A.cast<std::complex<double>>();
        const Eigen::MatrixXcd Bc = sys.B.cast<std::complex<double>>();
        const Eigen::MatrixXcd Cc = sys.C.cast<std::complex<double>>();
        const Eigen::MatrixXcd Dc = sys.D.cast<std::complex<double>>();

        // Evaluate max singular value at frequency w (log-domain: w in natural frequency)
        auto evalSV = [&](double w) -> double
        {
            const std::complex<double> z = std::polar(1.0, w * sys.Ts);
            const Eigen::MatrixXcd G = Cc * (z * I - Ac).inverse() * Bc + Dc;
            Eigen::JacobiSVD<Eigen::MatrixXcd> svd(G);
            return svd.singularValues()(0);
        };

        double peak_mag = 0.0;
        int    peak_i   = 0;
        std::vector<double> grid_w(num_points);

        for (int i = 0; i < num_points; ++i)
        {
            grid_w[i] = w_min * std::pow(w_max / w_min, static_cast<double>(i) / (num_points - 1));
            const double sv = evalSV(grid_w[i]);
            if (sv > peak_mag) { peak_mag = sv; peak_i = i; }
        }

        // Step 2: golden-section search in the log-frequency bracket around the grid peak.
        // This refines the grid lower bound to the true (or near-true) Hinf norm in O(50) evals.
        const double phi = (std::sqrt(5.0) - 1.0) * 0.5; // golden ratio reciprocal
        double a = (peak_i > 0)             ? grid_w[peak_i - 1] : w_min;
        double b = (peak_i < num_points - 1) ? grid_w[peak_i + 1] : w_max;

        // Use log-domain for better numerical behaviour across wide frequency ranges.
        double la = std::log(a), lb_w = std::log(b);
        double lc = lb_w - phi * (lb_w - la);
        double ld = la  + phi * (lb_w - la);
        double fc = evalSV(std::exp(lc));
        double fd = evalSV(std::exp(ld));

        const int gs_iters = 60; // typically reduces interval by phi^60 ~ 1e-13
        for (int k = 0; k < gs_iters; ++k)
        {
            if (fc < fd)
            {
                la = lc;
                lc = ld; fc = fd;
                ld = la + phi * (lb_w - la);
                fd = evalSV(std::exp(ld));
            }
            else
            {
                lb_w = ld;
                ld = lc; fd = fc;
                lc = lb_w - phi * (lb_w - la);
                fc = evalSV(std::exp(lc));
            }
        }
        const double refined = std::max(fc, fd);
        return std::max(peak_mag, refined); // take the best of grid and refined peak
    }

} // namespace ctrl
