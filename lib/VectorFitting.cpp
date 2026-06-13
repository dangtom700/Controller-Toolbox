#include "VectorFitting.h"
#include "GapMetric.h" // freqResponseGrid for eval
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>

namespace ctrl
{

    // ============================================================================
    // initPoles: log-spaced real poles inside the unit disk, mapped from
    //            continuous-time: sigma_k = exp(log(omega_min) + k*step) -> p_k = exp(-sigma_k*Ts)
    // ============================================================================
    Eigen::VectorXd VectorFitting::initPoles(int n_poles, double omega_max, double Ts)
    {
        if (n_poles <= 0)
            return Eigen::VectorXd();
        const double omega_min = 1e-3; // avoid DC pole
        const double sig_lo = omega_min;
        const double sig_hi = std::min(omega_max * 0.8, M_PI / Ts * 0.9);

        Eigen::VectorXd poles(n_poles);
        if (n_poles == 1)
        {
            poles(0) = std::exp(-0.5 * (sig_lo + sig_hi) * Ts);
            return poles;
        }
        for (int k = 0; k < n_poles; ++k)
        {
            const double t = static_cast<double>(k) / (n_poles - 1);
            // log-spaced sigma
            const double sigma = std::exp(std::log(sig_lo) * (1.0 - t) + std::log(sig_hi) * t);
            poles(k) = std::exp(-sigma * Ts);
        }
        return poles;
    }

    // ============================================================================
    // enforcePoleStability: reflect any pole |p| >= 1 back to 1/p, and ensure p > 0.
    // ============================================================================
    void VectorFitting::enforcePoleStability(Eigen::VectorXd &poles)
    {
        for (int k = 0; k < poles.size(); ++k)
        {
            double p = poles(k);
            if (std::abs(p) >= 1.0)
                p = 1.0 / p; // reflect inside unit disk
            // keep poles real and positive (negative poles cause aliasing issues)
            if (p <= 0.0)
                p = 1e-6;
            poles(k) = p;
        }
    }

    // ============================================================================
    // buildSKSystem - construct the Sanathanan-Koerner divided LS system.
    //
    // Model: H(z) = N(z)/D(z)   where D is monic, n_poles-th order in z^{-1}.
    //
    // At iteration k: divide by D^{(k)}(z) to linearise:
    //   D^{(k)} * H approx = N  ->  Sigma_j c_j * phi_j(z) = (D^{(k)}(z) * H(z)) / D^{(k)}(z)
    //
    // Basis functions phi_j(z) = z^{-j} / D^{(k)}(z)  for j=0..n_poles  (numerator)
    //   and  psi_j(z) = -H(z) * z^{-j} / D^{(k)}(z)  for j=1..n_poles (denominator)
    //
    // Because H and phi/psi are all real at real frequencies, we work with real LS.
    // ============================================================================
    void VectorFitting::buildSKSystem(
        const std::vector<std::complex<double>> &z_grid,
        const std::vector<double> &mag,
        const Eigen::VectorXd &poles,
        int n_poles,
        Eigen::MatrixXd &A_out,
        Eigen::VectorXd &b_out)
    {
        const int N = static_cast<int>(z_grid.size());
        const int nc = n_poles + 1; // numerator coefficients c_0..c_{n_poles}
        const int nad = n_poles;    // denominator adjustments a_1..a_{n_poles}
        const int ncols = nc + nad;

        A_out.resize(N, ncols);
        b_out.resize(N);

        for (int i = 0; i < N; ++i)
        {
            const std::complex<double> z = z_grid[i];
            const double H_i = mag[i];

            // Evaluate D^{(k)}(z) = prod(z - p_k) / z^{n_poles}  (monic in z-form)
            // For the LS problem, we use partial-fractions:  1/D^{(k)} evaluates as
            // the polynomial  Pi(z - p_k) in the denominator.
            // Simpler: denominator in z^{-1} form: D(z^{-1}) = 1 + Sigma_k a_k * z^{-k}
            // We compute D^{(k)}(z_i) explicitly from current poles:
            //   D(z) = prod_{k=1..n} (z - p_k) / z^n
            //   On unit circle z=e^{jomega}: this is the product of (z-p_k).

            std::complex<double> Dk(1.0, 0.0);
            for (int k = 0; k < n_poles; ++k)
            {
                // pole p_k is real; cast to complex for multiplication
                Dk *= (z - std::complex<double>(poles(k), 0.0));
            }
            // normalise: Dk is z^{n_poles} * monic-D(z^{-1}) evaluated at z
            const double Dk_abs2 = std::norm(Dk);
            if (Dk_abs2 < 1e-30)
            {
                A_out.row(i).setZero();
                b_out(i) = 0.0;
                continue;
            }

            // Numerator columns: z^{j} / Dk for j = 0..n_poles (in z basis)
            // In practice: col j corresponds to z^{j} / prod(z - p_k).
            // We take the real part of z^j / Dk, weighted by |Dk|^{-1}.
            std::complex<double> z_pow(1.0, 0.0); // z^0
            for (int j = 0; j < nc; ++j)
            {
                // real part of (z^j / Dk)
                const std::complex<double> basis = z_pow / Dk;
                A_out(i, j) = basis.real();
                z_pow *= z;
            }

            // Denominator columns: -H(z) * z^{j+1} / Dk  for j=0..n_poles-1
            std::complex<double> z_pow2 = z; // z^1
            for (int j = 0; j < nad; ++j)
            {
                const std::complex<double> basis = -std::complex<double>(H_i, 0.0) * z_pow2 / Dk;
                A_out(i, nc + j) = basis.real();
                z_pow2 *= z;
            }

            b_out(i) = (std::complex<double>(H_i, 0.0) / Dk).real();
        }
    }

    // ============================================================================
    // buildStateSpace - companion form from real poles and residues.
    // H(z) = Sigma_k r_k / (z - p_k) + d_gain
    //       = C (zI - A)^{-1} B + d_gain
    // Diagonal A = diag(p), B = ones, C = r'.
    // ============================================================================
    StateSpace VectorFitting::buildStateSpace(
        const Eigen::VectorXd &poles,
        const Eigen::VectorXd &residues,
        double d_gain,
        double Ts)
    {
        const int n = poles.size();
        Eigen::MatrixXd A = poles.asDiagonal(); // n*n diagonal
        Eigen::MatrixXd B = Eigen::MatrixXd::Ones(n, 1);
        Eigen::MatrixXd C = residues.transpose(); // 1*n
        Eigen::MatrixXd D(1, 1);
        D(0, 0) = d_gain;
        return StateSpace(A, B, C, D, Ts);
    }

    // ============================================================================
    // evalMagnitude
    // ============================================================================
    double VectorFitting::evalMagnitude(const StateSpace &sys, double omega)
    {
        const double Ts = sys.Ts;
        const std::complex<double> z(std::cos(omega * Ts), std::sin(omega * Ts));
        const int n = sys.A.rows();
        // const int ny = sys.C.rows();
        // const int nu = sys.D.cols();

        Eigen::MatrixXcd I = Eigen::MatrixXcd::Identity(n, n);
        Eigen::MatrixXcd zI = z * I;

        Eigen::FullPivLU<Eigen::MatrixXcd> lu(zI - sys.A.cast<std::complex<double>>());
        if (!lu.isInvertible())
            return 0.0;

        const Eigen::MatrixXcd H = sys.C.cast<std::complex<double>>() *
                                       lu.solve(sys.B.cast<std::complex<double>>()) +
                                   sys.D.cast<std::complex<double>>();
        return H.norm(); // Frobenius norm = |scalar| for SISO
    }

    // ============================================================================
    // fitMagnitude - main entry point
    // ============================================================================
    StateSpace VectorFitting::fitMagnitude(
        const std::vector<double> &omega_grid,
        const std::vector<double> &magnitude,
        int n_poles,
        double Ts,
        VectorFittingResult &result,
        const VectorFittingParams &params)
    {
        const int N = static_cast<int>(omega_grid.size());
        if (N == 0 || magnitude.size() != static_cast<size_t>(N))
            throw std::invalid_argument("VectorFitting: omega_grid and magnitude must be non-empty and same size.");
        if (n_poles < 1)
            throw std::invalid_argument("VectorFitting: n_poles must be >= 1.");

        // Build z-grid on the unit circle
        std::vector<std::complex<double>> z_grid(N);
        for (int i = 0; i < N; ++i)
            z_grid[i] = std::complex<double>(std::cos(omega_grid[i] * Ts),
                                             std::sin(omega_grid[i] * Ts));

        // Sanitise magnitudes: replace zeros with floor
        std::vector<double> mag(N);
        for (int i = 0; i < N; ++i)
            mag[i] = std::max(std::abs(magnitude[i]), params.eps_mag);

        // Initialise poles
        const double omega_max = omega_grid.back();
        Eigen::VectorXd poles = initPoles(n_poles, omega_max, Ts);

        result.converged = false;
        result.iterations = 0;

        Eigen::VectorXd residues = Eigen::VectorXd::Ones(n_poles) * 0.1;
        double d_gain = mag[N / 2]; // rough DC initialisation

        for (int iter = 0; iter < params.max_iter; ++iter)
        {
            Eigen::MatrixXd A;
            Eigen::VectorXd b;
            buildSKSystem(z_grid, mag, poles, n_poles, A, b);

            // Solve LS: A * x = b
            Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

            // Extract numerator coefficients c_0..c_{n_poles} and denominator adjustment a_1..a_{n_poles}
            const int nc = n_poles + 1;
            const Eigen::VectorXd c_coeff = x.head(nc);      // numerator
            const Eigen::VectorXd a_coeff = x.tail(n_poles); // denominator a_1..a_n

            // Build denominator polynomial D(z) = z^n + a_1 z^{n-1} + ... + a_n
            // Companion matrix for root finding
            Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(n_poles, n_poles);
            for (int k = 0; k < n_poles - 1; ++k)
                companion(k + 1, k) = 1.0;
            for (int k = 0; k < n_poles; ++k)
                companion(0, k) = -a_coeff(n_poles - 1 - k);

            Eigen::EigenSolver<Eigen::MatrixXd> es(companion, false);
            Eigen::VectorXd new_poles(n_poles);
            const auto &evals = es.eigenvalues();
            for (int k = 0; k < n_poles; ++k)
            {
                // Take real part; imaginary part should be ~0 for real-valued signals
                new_poles(k) = evals(k).real();
            }
            enforcePoleStability(new_poles);

            // Convergence check
            const double max_disp = (new_poles - poles).cwiseAbs().maxCoeff();
            poles = new_poles;
            ++result.iterations;

            // After pole update, solve for residues with fixed poles (one LS)
            // H(z) = Sigma_k r_k / (z - p_k) + d
            // Build a new LS for residues only
            Eigen::MatrixXd Ar(N, n_poles + 1);
            Eigen::VectorXd br(N);
            for (int i = 0; i < N; ++i)
            {
                const std::complex<double> z = z_grid[i];
                for (int k = 0; k < n_poles; ++k)
                {
                    const std::complex<double> term = 1.0 / (z - std::complex<double>(poles(k), 0.0));
                    Ar(i, k) = term.real();
                }
                Ar(i, n_poles) = 1.0; // constant term d
                br(i) = mag[i];
            }
            Eigen::VectorXd rd = Ar.colPivHouseholderQr().solve(br);
            residues = rd.head(n_poles);
            d_gain = rd(n_poles);

            if (max_disp < params.tol)
            {
                result.converged = true;
                break;
            }
        }

        // Compute RMS relative error
        {
            const StateSpace sys = buildStateSpace(poles, residues, d_gain, Ts);
            double rms = 0.0;
            for (int i = 0; i < N; ++i)
            {
                const double fit = evalMagnitude(sys, omega_grid[i]);
                const double rel = (fit - mag[i]) / mag[i];
                rms += rel * rel;
            }
            result.rms_error = std::sqrt(rms / N);
        }

        return buildStateSpace(poles, residues, d_gain, Ts);
    }

} // namespace ctrl
