#include "SystemAnalysis.h"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
        std::vector<std::complex<double>> response(freqs.size());
        int n = sys.A.rows();
        Eigen::MatrixXcd I = Eigen::MatrixXcd::Identity(n, n);

        // Convert to complex matrices
        Eigen::MatrixXcd Ac = sys.A.cast<std::complex<double>>();
        Eigen::MatrixXcd Bc = sys.B.cast<std::complex<double>>();
        Eigen::MatrixXcd Cc = sys.C.cast<std::complex<double>>();
        Eigen::MatrixXcd Dc = sys.D.cast<std::complex<double>>();

        for (size_t i = 0; i < freqs.size(); ++i)
        {
            // z = e^(j * w * Ts)
            std::complex<double> z = std::polar(1.0, freqs[i] * sys.Ts);

            // G(z) = C * (zI - A)^-1 * B + D
            Eigen::MatrixXcd resolvent = (z * I - Ac).inverse();
            Eigen::MatrixXcd G = Cc * resolvent * Bc + Dc;

            // For SISO
            if (G.rows() == 1 && G.cols() == 1)
            {
                response[i] = G(0, 0);
            }
            else
            {
                // For MIMO, just return the (0,0) element or handle differently
                response[i] = G(0, 0);
            }
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

        auto wrapPhase = [](double p) -> double
        {
            while (p > 0.0)   p -= 360.0;
            while (p < -360.0) p += 360.0;
            return p;
        };

        // Step 1: Coarse logarithmic grid to locate crossover brackets (~200 evals)
        const double w_min  = 1e-3;
        const double w_max  = M_PI / sys.Ts;
        const int    coarse = 200;

        std::vector<double> fw(coarse), mag(coarse), pha(coarse);
        for (int i = 0; i < coarse; ++i)
        {
            fw[i]  = w_min * std::pow(w_max / w_min, static_cast<double>(i) / (coarse - 1));
            auto g = evalAt(fw[i]);
            mag[i] = std::abs(g);
            pha[i] = wrapPhase(std::arg(g) * 180.0 / M_PI);
        }

        // Step 2: Bisection refinement over each bracket - converges in ~50 evals per crossing
        const int bisect_iters = 50;

        StabilityMargins margins;
        margins.gainMarginDb    = std::numeric_limits<double>::infinity();
        margins.phaseMarginDeg  = std::numeric_limits<double>::infinity();
        margins.wCrossoverGain  = 0.0;
        margins.wCrossoverPhase = 0.0;

        // Gain margin: locate every phase = -180^\circ crossing; keep worst-case (smallest GM)
        for (int i = 0; i + 1 < coarse; ++i)
        {
            if ((pha[i] - (-180.0)) * (pha[i + 1] - (-180.0)) >= 0.0) continue;

            double lo = fw[i], hi = fw[i + 1];
            double p_lo = pha[i];
            for (int k = 0; k < bisect_iters; ++k)
            {
                double mid  = std::sqrt(lo * hi); // geometric midpoint preserves log scale
                double p_mid = wrapPhase(std::arg(evalAt(mid)) * 180.0 / M_PI);
                if ((p_lo - (-180.0)) * (p_mid - (-180.0)) < 0.0)
                    hi = mid;
                else { lo = mid; p_lo = p_mid; }
            }
            const double w_cross  = std::sqrt(lo * hi);
            const double mag_cross = std::abs(evalAt(w_cross));
            const double gm_cand  = (mag_cross >= 1e-300)
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
                double mid  = std::sqrt(lo * hi);
                double m_mid = std::abs(evalAt(mid));
                if ((m_lo - 1.0) * (m_mid - 1.0) < 0.0)
                    hi = mid;
                else { lo = mid; m_lo = m_mid; }
            }
            const double w_cross    = std::sqrt(lo * hi);
            const double phase_cross = wrapPhase(std::arg(evalAt(w_cross)) * 180.0 / M_PI);
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
        // Grid-based search for peak singular value over frequency
        double w_min = 1e-3;
        double w_max = M_PI / sys.Ts;
        int num_points = 2000;

        int n = sys.A.rows();
        Eigen::MatrixXcd I = Eigen::MatrixXcd::Identity(n, n);
        Eigen::MatrixXcd Ac = sys.A.cast<std::complex<double>>();
        Eigen::MatrixXcd Bc = sys.B.cast<std::complex<double>>();
        Eigen::MatrixXcd Cc = sys.C.cast<std::complex<double>>();
        Eigen::MatrixXcd Dc = sys.D.cast<std::complex<double>>();

        double peak_mag = 0.0;

        for (int i = 0; i < num_points; ++i)
        {
            double w = w_min * std::pow(w_max / w_min, static_cast<double>(i) / (num_points - 1));
            std::complex<double> z = std::polar(1.0, w * sys.Ts);

            Eigen::MatrixXcd resolvent = (z * I - Ac).inverse();
            Eigen::MatrixXcd G = Cc * resolvent * Bc + Dc;

            // Max singular value is the induced 2-norm of the matrix
            Eigen::JacobiSVD<Eigen::MatrixXcd> svd(G);
            double max_sv = svd.singularValues()(0);

            if (max_sv > peak_mag)
            {
                peak_mag = max_sv;
            }
        }
        return peak_mag;
    }

} // namespace ctrl
