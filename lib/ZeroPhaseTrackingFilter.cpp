#include "ZeroPhaseTrackingFilter.h"
#include "SystemAnalysis.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace ctrl
{

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Multiply polynomial (coefficients in descending z powers) by (z - root).
static std::vector<std::complex<double>> polyMulRoot(
    const std::vector<std::complex<double>> &p,
    std::complex<double> root)
{
    // p = [p_0, p_1, ..., p_n]  representing p_0*z^n + p_1*z^{n-1} + ... + p_n
    // result = z*p - root*p  ->  degree n+1
    std::vector<std::complex<double>> result(p.size() + 1, 0.0);
    for (std::size_t i = 0; i < p.size(); ++i)
    {
        result[i]     += p[i];           // z * p[i] shifted up
        result[i + 1] -= root * p[i];   // -root * p[i]
    }
    return result;
}

// Build monic polynomial from its roots: [1, c_{n-1}, ..., c_0] in descending z powers.
// Returns the real part (imaginary cancels for conjugate-pair root sets).
static Eigen::VectorXd rootsToPoly(const std::vector<std::complex<double>> &roots)
{
    std::vector<std::complex<double>> poly = {std::complex<double>(1.0, 0.0)};
    for (const auto &r : roots)
        poly = polyMulRoot(poly, r);

    Eigen::VectorXd real_poly(static_cast<int>(poly.size()));
    for (int i = 0; i < static_cast<int>(poly.size()); ++i)
        real_poly(i) = poly[static_cast<std::size_t>(i)].real();
    return real_poly;
}

// Evaluate SISO transfer function G(z) = C*(zI-A)^{-1}*B + D at complex z.
static std::complex<double> evalTF(const StateSpace &sys, std::complex<double> z)
{
    const int n = sys.stateSize();
    const Eigen::MatrixXcd zIA = z * Eigen::MatrixXcd::Identity(n, n)
                                   - sys.A.cast<std::complex<double>>();
    const Eigen::MatrixXcd B_c = sys.B.cast<std::complex<double>>();  // n * m
    const Eigen::MatrixXcd C_c = sys.C.cast<std::complex<double>>();  // p * n  (keep as matrix)
    const Eigen::MatrixXcd D_c = sys.D.cast<std::complex<double>>();  // p * m
    // G(z) = C*(zI-A)^{-1}*B + D  -> (p*n) * (n*n)^{-1} * (n*m) + (p*m) = (p*m)
    const Eigen::MatrixXcd G = C_c * zIA.colPivHouseholderQr().solve(B_c) + D_c;
    return G(0, 0);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::vector<std::complex<double>> transmissionZeros(const StateSpace &sys)
{
    if (sys.inputSize() != 1 || sys.outputSize() != 1)
        throw std::invalid_argument("transmissionZeros: only SISO systems are supported.");

    const int n = sys.stateSize();
    // System matrix M = [[A, B], [C, D]] and E = [[I, 0], [0, 0]]
    // Transmission zeros = finite eigenvalues of the pencil M*v = lambda*E*v
    // i.e., solve det([[A-lambdaI, B],[C, D]]) = 0

    const int total = n + 1;
    Eigen::MatrixXd M(total, total), E = Eigen::MatrixXd::Zero(total, total);

    M.topLeftCorner(n, n) = sys.A;
    M.topRightCorner(n, 1) = sys.B;
    M.bottomLeftCorner(1, n) = sys.C;
    M(n, n) = sys.D(0, 0);

    E.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n);
    // E(n,n) = 0 (already) - makes the pencil detect zeros at finite eigenvalues

    Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges(M, E);

    std::vector<std::complex<double>> zeros;
    zeros.reserve(n);
    for (int i = 0; i < total; ++i)
    {
        const double beta = std::abs(ges.betas()(i));
        if (beta > 1e-10)
            zeros.push_back(ges.alphas()(i) / ges.betas()(i));
        // beta approx = 0 -> infinite eigenvalue -> not a finite transmission zero
    }
    return zeros;
}

ZPETCResult designZPETC(const StateSpace &plant)
{
    if (plant.inputSize() != 1 || plant.outputSize() != 1)
        throw std::invalid_argument("designZPETC: only SISO plants are supported.");

    const int n = plant.stateSize();

    // --- Transmission zeros and partition ---
    const auto all_zeros = transmissionZeros(plant);

    std::vector<std::complex<double>> min_phase_zeros, nmp_zeros;
    const double nmp_threshold = 1.0 - 1e-8;
    for (const auto &z : all_zeros)
    {
        if (std::abs(z) < nmp_threshold)
            min_phase_zeros.push_back(z);
        else
            nmp_zeros.push_back(z);
    }

    const bool has_nmp = !nmp_zeros.empty();
    const int m_plus = static_cast<int>(min_phase_zeros.size()); // degree of B+(z)

    // --- Numerator gain: first non-zero Markov parameter of the plant ---
    // For G(z) = B(z)/A(z), b_gain = leading coefficient of B(z) in z-polynomial form.
    // = D (for bi-proper plant) or first non-zero C*A^k*B (for strictly proper).
    double b_gain = plant.D(0, 0);
    if (std::abs(b_gain) < 1e-10)
    {
        Eigen::VectorXd Ak_B = plant.B;
        for (int k = 0; k < n; ++k)
        {
            b_gain = (plant.C * Ak_B)(0, 0);
            if (std::abs(b_gain) > 1e-10) break;
            Ak_B = plant.A * Ak_B;
        }
    }
    if (std::abs(b_gain) < 1e-10)
        b_gain = 1.0; // fallback for all-pole plant

    // --- DC normalisation: B^-(1) = Pi (1 - z_i) for NMP zeros ---
    std::complex<double> B_minus_dc(1.0, 0.0);
    for (const auto &z : nmp_zeros)
        B_minus_dc *= (1.0 - z);
    // B-(1) is real and positive for real conjugate-pair zeros
    const double dc_norm = (std::abs(B_minus_dc) < 1e-12) ? 1.0 : B_minus_dc.real();

    // --- Denominator polynomial: B+(z) * z^{n - m_plus} ---
    // Start with B+(z) from min-phase zeros
    Eigen::VectorXd B_plus_coeffs = rootsToPoly(min_phase_zeros);
    // B_plus_coeffs has length m_plus + 1 (degree m_plus)

    // Pad with n - m_plus trailing zeros to represent multiplication by z^{n-m_plus}
    const int n_pad = n - m_plus;
    Eigen::VectorXd den_coeffs(B_plus_coeffs.size() + n_pad);
    den_coeffs.head(static_cast<int>(B_plus_coeffs.size())) = B_plus_coeffs;
    den_coeffs.tail(n_pad).setZero();
    // den_coeffs now represents B+(z) * z^{n-m_plus}, degree = n

    // --- Numerator polynomial: K * A(z) ---
    // A(z) = characteristic polynomial of plant.A = Pi(z - lambda_i)
    const auto plant_poles_complex = plant.A.eigenvalues();
    std::vector<std::complex<double>> poles_vec(plant_poles_complex.data(),
                                                 plant_poles_complex.data() + n);
    Eigen::VectorXd A_coeffs = rootsToPoly(poles_vec); // [1, a_1, ..., a_n], degree n

    // K normalises both the plant gain and the NMP DC contribution
    const double K = 1.0 / (b_gain * dc_norm);
    Eigen::VectorXd num_coeffs = K * A_coeffs; // degree n

    // Both num and den have degree n -> bi-proper filter (D != 0 via tf2ss)

    // --- Build TransferFunction struct (TF stores coefficients in descending z powers,
    //     denominator must be monic with den[0] = 1) ---
    // den_coeffs[0] = B_plus_coeffs[0] (leading coefficient of B+(z))
    // Normalise denominator to be monic:
    const double den_lead = den_coeffs(0);
    if (std::abs(den_lead) < 1e-12)
    {
        // All zeros are NMP - trivial min-phase inverse = constant gain
        // Build a simple gain filter: G_ff(z) = K
        StateSpace gain_filter(
            Eigen::MatrixXd::Zero(1, 1),
            Eigen::MatrixXd::Zero(1, 1),
            Eigen::MatrixXd::Zero(1, 1),
            Eigen::MatrixXd::Constant(1, 1, K),
            plant.Ts);
        return {gain_filter, 0.0, has_nmp, all_zeros, nmp_zeros};
    }

    Eigen::VectorXd den_norm = den_coeffs / den_lead;
    Eigen::VectorXd num_norm = num_coeffs / den_lead; // scale num proportionally

    // Convert Eigen vectors to std::vector for TransferFunction constructor
    const int poly_len = static_cast<int>(num_norm.size());
    std::vector<double> num_vec(num_norm.data(), num_norm.data() + poly_len);
    std::vector<double> den_vec(den_norm.data(), den_norm.data() + poly_len);

    TransferFunction ff_tf(num_vec, den_vec, plant.Ts);
    StateSpace filter = tf2ss(ff_tf);

    // --- Compute DC amplitude error ---
    // Evaluate |G(e^{jomega}) * G_ff(e^{jomega}) - 1| at 50 frequencies
    double max_amp_err = 0.0;
    const int N_freq = 50;
    for (int i = 0; i < N_freq; ++i)
    {
        const double omega = (i + 1.0) * M_PI / (N_freq + 1.0) / plant.Ts;
        const std::complex<double> z = std::exp(std::complex<double>(0.0, omega * plant.Ts));
        const std::complex<double> G_val    = evalTF(plant, z);
        const std::complex<double> Gff_val  = evalTF(filter, z);
        max_amp_err = std::max(max_amp_err, std::abs(std::abs(G_val * Gff_val) - 1.0));
    }

    return {filter, max_amp_err, has_nmp, all_zeros, nmp_zeros};
}

} // namespace ctrl
