#include "MuAnalysis.h"
#include "SystemAnalysis.h"
#include "GapMetric.h"
#include <Eigen/SVD>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <functional>

namespace ctrl {

int UncertaintyStructure::totalInputs() const
{
    int n = 0;
    for (const auto& b : blocks) n += b.r_in;
    return n;
}

int UncertaintyStructure::totalOutputs() const
{
    int n = 0;
    for (const auto& b : blocks) n += b.r_out;
    return n;
}

namespace {

double sigmaMax(const Eigen::MatrixXcd& M)
{
    Eigen::JacobiSVD<Eigen::MatrixXcd> svd(M);
    return svd.singularValues()(0);
}

// Build the two diagonal scaling matrices D_L (totalOutputs x totalOutputs) and
// D_R (totalInputs x totalInputs) from one positive scalar d_i per block.
void buildScaling(const UncertaintyStructure& struc, const Eigen::VectorXd& d,
                   Eigen::MatrixXcd& D_L, Eigen::MatrixXcd& D_R)
{
    const int P = struc.totalOutputs();
    const int Q = struc.totalInputs();
    D_L = Eigen::MatrixXcd::Zero(P, P);
    D_R = Eigen::MatrixXcd::Zero(Q, Q);

    int row = 0, col = 0;
    for (std::size_t i = 0; i < struc.blocks.size(); ++i) {
        const auto& b = struc.blocks[i];
        const std::complex<double> di(d(static_cast<int>(i)), 0.0);
        for (int k = 0; k < b.r_out; ++k) D_L(row + k, row + k) = di;
        for (int k = 0; k < b.r_in;  ++k) D_R(col + k, col + k) = di;
        row += b.r_out;
        col += b.r_in;
    }
}

// Build the G-scaling matrix (P x P, real-valued stored as complex) for Packard & Doyle
// (1993) G-scaling: g_i placed on every diagonal row belonging to RealScalar block i, zero
// elsewhere (including ComplexFull/ComplexScalar block rows, which have no G contribution).
void buildGScaling(const UncertaintyStructure& struc, const Eigen::VectorXd& g,
                   Eigen::MatrixXcd& G)
{
    const int P = struc.totalOutputs();
    G = Eigen::MatrixXcd::Zero(P, P);

    int row = 0;
    for (std::size_t i = 0; i < struc.blocks.size(); ++i) {
        const auto& b = struc.blocks[i];
        if (b.type == UncertaintyBlock::Type::RealScalar) {
            const std::complex<double> gi(g(static_cast<int>(i)), 0.0);
            for (int k = 0; k < b.r_out; ++k) G(row + k, row + k) = gi;
        }
        row += b.r_out;
    }
}

// sigma_max((I+G^2)^{-1/2} * (D_L*M*D_R^-1 - jG)) - the Packard & Doyle (1993) G-scaled
// upper bound. G is diagonal (one scalar g_i per RealScalar block, zero on Complex* block
// rows), so (I+G^2)^{-1/2} is also diagonal and reduces to a per-row scale factor - no matrix
// square root needed. With g == 0 (no RealScalar blocks, or not yet swept) this reduces
// exactly to the original D-only upper bound: G == 0 and every scale factor == 1.
double scaledSigmaMax(const Eigen::MatrixXcd& M, const UncertaintyStructure& struc,
                       const Eigen::VectorXd& d, const Eigen::VectorXd& g)
{
    Eigen::MatrixXcd D_L, D_R, G;
    buildScaling(struc, d, D_L, D_R);
    buildGScaling(struc, g, G);

    Eigen::MatrixXcd N = D_L * M * D_R.inverse() - std::complex<double>(0.0, 1.0) * G;

    // Scale each row by 1/sqrt(1+g_i^2) directly (avoids mixing real VectorXd::asDiagonal()
    // with a complex matrix, which Eigen's operator* does not allow without an explicit cast).
    int row = 0;
    for (std::size_t i = 0; i < struc.blocks.size(); ++i) {
        const auto& b = struc.blocks[i];
        if (b.type == UncertaintyBlock::Type::RealScalar) {
            const double gi = g(static_cast<int>(i));
            const double s  = 1.0 / std::sqrt(1.0 + gi * gi);
            for (int k = 0; k < b.r_out; ++k) N.row(row + k) *= s;
        }
        row += b.r_out;
    }
    return sigmaMax(N);
}

// Golden-section minimisation of a unimodal scalar function over [lo, hi].
double goldenSectionMinimize(const std::function<double(double)>& f,
                              double lo, double hi, int iters)
{
    const double gr = 0.6180339887498949; // (sqrt(5)-1)/2
    double a = lo, b = hi;
    double c = b - gr * (b - a);
    double d = a + gr * (b - a);
    double fc = f(c), fd = f(d);
    for (int i = 0; i < iters; ++i) {
        if (fc < fd) {
            b = d; d = c; fd = fc;
            c = b - gr * (b - a);
            fc = f(c);
        } else {
            a = c; c = d; fc = fd;
            d = a + gr * (b - a);
            fd = f(d);
        }
    }
    return 0.5 * (a + b);
}

MuBound computeMuOneFreq(const Eigen::MatrixXcd& M, const UncertaintyStructure& struc,
                          bool compute_lower)
{
    const int P = struc.totalOutputs();
    const int Q = struc.totalInputs();

    if (M.rows() != P || M.cols() != Q)
        throw std::invalid_argument(
            "computeMu: M dimensions do not match the uncertainty structure "
            "(M is " + std::to_string(M.rows()) + "x" + std::to_string(M.cols()) +
            ", structure expects " + std::to_string(P) + "x" + std::to_string(Q) + ").");
    if (P != Q)
        throw std::invalid_argument(
            "computeMu: the M-Delta loop requires a square interconnection "
            "(totalOutputs() must equal totalInputs()).");

    bool has_real_scalar = false;
    for (const auto& b : struc.blocks) {
        if (b.type == UncertaintyBlock::Type::RealScalar)
            has_real_scalar = true;
        if (b.type != UncertaintyBlock::Type::ComplexFull && b.r_out != b.r_in)
            throw std::invalid_argument(
                "computeMu: scalar (repeated) uncertainty blocks must have r_out == r_in.");
    }

    const int F = static_cast<int>(struc.blocks.size());
    const double sigma0 = sigmaMax(M);

    MuBound result;
    result.upper = sigma0;
    result.lower = 0.0;

    // D-scaling alone has no effect for a single block (a uniform global D cancels in
    // D*M*D^-1), but G-scaling does NOT have that gauge freedom - a lone RealScalar block
    // still benefits from searching g, so the sweep runs whenever F > 1 (D matters) or any
    // block is RealScalar (G matters), not just F > 1.
    if (F > 1 || has_real_scalar) {
        // Coordinate-descent D/G-scaling: minimise
        //   sigma_max((I+G^2)^-1/2 * (D*M*D^-1 - jG))
        // over one positive scalar d_i per block (gauge-fixed d_{F-1} = 1, since a uniform
        // global scaling leaves D*M*D^-1 unchanged) and one real scalar g_i per RealScalar
        // block (Packard & Doyle 1993 G-scaling; g_i == 0 for Complex* blocks, where this
        // reduces exactly to the original D-only upper bound). Exact for ComplexFull blocks
        // and RealScalar blocks of size 1; a valid-but-possibly-loose upper bound for
        // ComplexScalar/RealScalar blocks of size > 1 (see header).
        Eigen::VectorXd d = Eigen::VectorXd::Ones(F);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(F);
        constexpr double LOG_RANGE       = 4.0;  // search d in [e^-4, e^4] per coordinate
        constexpr double G_THETA_RANGE   = 1.56; // search g = tan(theta), |g| up to ~tan(1.56)~92.6
        constexpr int    SWEEPS          = 5;
        double best = sigma0;

        for (int sweep = 0; sweep < SWEEPS; ++sweep) {
            for (int i = 0; i < F - 1; ++i) {
                auto cost = [&](double log_di) {
                    Eigen::VectorXd dt = d;
                    dt(i) = std::exp(log_di);
                    return scaledSigmaMax(M, struc, dt, g);
                };
                const double log_d0  = std::log(d(i));
                const double log_opt = goldenSectionMinimize(
                    cost, log_d0 - LOG_RANGE, log_d0 + LOG_RANGE, 40);
                d(i) = std::exp(log_opt);
                const double s = scaledSigmaMax(M, struc, d, g);
                if (s < best) best = s;
            }
            for (int i = 0; i < F; ++i) {
                if (struc.blocks[static_cast<std::size_t>(i)].type
                    != UncertaintyBlock::Type::RealScalar)
                    continue;
                // Fixed window [-G_THETA_RANGE, G_THETA_RANGE] (not centered on the current
                // g(i)) so theta always stays within tan()'s (-pi/2, pi/2) domain - golden
                // section explores the whole given interval, so a fixed bracket is fine.
                auto cost = [&](double theta) {
                    Eigen::VectorXd gt = g;
                    gt(i) = std::tan(theta);
                    return scaledSigmaMax(M, struc, d, gt);
                };
                const double theta_opt = goldenSectionMinimize(
                    cost, -G_THETA_RANGE, G_THETA_RANGE, 40);
                g(i) = std::tan(theta_opt);
                const double s = scaledSigmaMax(M, struc, d, g);
                if (s < best) best = s;
            }
        }
        result.upper = std::min(sigma0, best);
    }

    if (compute_lower) {
        result.lower = M.eigenvalues().cwiseAbs().maxCoeff();
    }

    return result;
}

std::vector<double> logFreqGrid(double Ts, int N, double omega_min)
{
    const double omega_max = 3.14159265358979323846 / Ts;
    const double log_min   = std::log(omega_min);
    const double log_max   = std::log(omega_max);
    std::vector<double> grid(N);
    for (int i = 0; i < N; ++i)
        grid[i] = std::exp(log_min + (log_max - log_min) * i / (N - 1));
    return grid;
}

} // anonymous namespace

std::vector<MuBound> computeMu(const std::vector<Eigen::MatrixXcd>& M_freq,
                                const UncertaintyStructure& struc,
                                bool compute_lower_bound)
{
    std::vector<MuBound> result;
    result.reserve(M_freq.size());
    for (const auto& M : M_freq)
        result.push_back(computeMuOneFreq(M, struc, compute_lower_bound));
    return result;
}

PeakMuResult peakMu(const StateSpace& G, const StateSpace& K,
                     const UncertaintyStructure& struc,
                     double sigma_rel, int freq_points, double omega_min)
{
    if (freq_points < 2) freq_points = 2;

    const GangOfFour g4   = SystemAnalysis::gangOfFour(G, K);
    const auto       grid = logFreqGrid(G.Ts, freq_points, omega_min);
    const auto       T_resp = freqResponseGrid(g4.T, grid);

    std::vector<Eigen::MatrixXcd> M_freq;
    M_freq.reserve(T_resp.size());
    for (const auto& T : T_resp)
        M_freq.push_back(sigma_rel * T);

    const std::vector<MuBound> curve = computeMu(M_freq, struc, false);

    PeakMuResult res;
    res.mu_curve         = curve;
    res.peak             = MuBound{0.0, 0.0};
    res.peak_omega_rad_s  = grid.empty() ? 0.0 : grid[0];
    for (std::size_t i = 0; i < curve.size(); ++i) {
        if (curve[i].upper > res.peak.upper) {
            res.peak             = curve[i];
            res.peak_omega_rad_s = grid[i];
        }
    }
    return res;
}

double robustStabilityRadius(const StateSpace& G, const StateSpace& K,
                              const UncertaintyStructure& struc,
                              double sigma_max, int bisect_iters)
{
    auto peakAt = [&](double s) {
        return peakMu(G, K, struc, s).peak.upper;
    };

    if (peakAt(sigma_max) < 1.0) return sigma_max;

    double lo = 0.0, hi = sigma_max;
    for (int i = 0; i < bisect_iters; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (peakAt(mid) < 1.0) lo = mid; else hi = mid;
    }
    return lo;
}

} // namespace ctrl
