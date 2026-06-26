#include "GapMetric.h"
#include "PlantModel.h"
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ctrl {

namespace {

// Log-spaced frequency grid from omega_min to pi/Ts.
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

std::vector<Eigen::MatrixXcd> freqResponseGrid(const StateSpace& sys,
                                                const std::vector<double>& omega_grid)
{
    const int n  = sys.stateSize();
    const double Ts = sys.Ts;
    (void)sys.inputSize();   // referenced via B dimensions
    (void)sys.outputSize();  // referenced via C dimensions

    Eigen::MatrixXcd Ac = sys.A.cast<std::complex<double>>();
    Eigen::MatrixXcd Bc = sys.B.cast<std::complex<double>>();
    Eigen::MatrixXcd Cc = sys.C.cast<std::complex<double>>();
    Eigen::MatrixXcd Dc = sys.D.cast<std::complex<double>>();
    Eigen::MatrixXcd I  = Eigen::MatrixXcd::Identity(n, n);

    std::vector<Eigen::MatrixXcd> result;
    result.reserve(omega_grid.size());

    // n == 0 means a purely static (zero-state) map: C*(zI-A)^-1*B is vacuously zero (C has
    // no columns), so H(z) == D at every frequency. Skip straight to that, since
    // FullPivLU::compute() unconditionally reduces the matrix via cwiseAbs().colwise().sum()
    // .maxCoeff() for its internal rcond() support, and that reduction asserts on the 0-column
    // result when decomposing a 0x0 matrix.
    if (n == 0) {
        result.assign(omega_grid.size(), Dc);
        return result;
    }

    for (double omega : omega_grid) {
        std::complex<double> z = std::exp(std::complex<double>(0.0, omega * Ts));
        // (z*I - A) is well-conditioned away from poles; fullPivLu handles near-singular.
        Eigen::MatrixXcd zIA = z * I - Ac;
        Eigen::MatrixXcd H   = Cc * zIA.fullPivLu().solve(Bc) + Dc;
        result.push_back(std::move(H));
    }
    return result;
}

double chordalDist(std::complex<double> p1, std::complex<double> p2)
{
    double num = std::abs(p1 - p2);
    double den = std::sqrt((1.0 + std::norm(p1)) * (1.0 + std::norm(p2)));
    return (den < 1e-15) ? 0.0 : num / den;
}

double subspaceDist(const Eigen::MatrixXcd& H1, const Eigen::MatrixXcd& H2)
{
    // Form normalised graphs G_i = [H_i; I_{m}] of shape (p+m) x m, then extract
    // orthonormal column bases U_i via thin SVD.  The subspace chordal distance is:
    //   d = sqrt(1 - sigma_min(U1^H * U2)^2)
    // This reduces to chordalDist() for SISO (p=m=1) -- see header for proof.
    const int p = static_cast<int>(H1.rows());
    const int m = static_cast<int>(H1.cols());

    Eigen::MatrixXcd G1(p + m, m), G2(p + m, m);
    G1.topRows(p)    = H1;
    G1.bottomRows(m) = Eigen::MatrixXcd::Identity(m, m);
    G2.topRows(p)    = H2;
    G2.bottomRows(m) = Eigen::MatrixXcd::Identity(m, m);

    // Thin SVD: U_i is (p+m) x m with orthonormal columns.
    Eigen::JacobiSVD<Eigen::MatrixXcd> svd1(G1, Eigen::ComputeThinU);
    Eigen::JacobiSVD<Eigen::MatrixXcd> svd2(G2, Eigen::ComputeThinU);

    // Smallest singular value of the m x m inner product matrix U1^H * U2.
    Eigen::JacobiSVD<Eigen::MatrixXcd> svd_cross(
        svd1.matrixU().adjoint() * svd2.matrixU());
    const double sigma_min = svd_cross.singularValues().minCoeff();
    return std::sqrt(std::max(0.0, 1.0 - sigma_min * sigma_min));
}

double nuGap(const StateSpace& P1, const StateSpace& P2,
             int freq_points, double omega_min)
{
    if (P1.inputSize()  != P2.inputSize() ||
        P1.outputSize() != P2.outputSize())
        throw std::invalid_argument(
            "nuGap: both systems must have the same input/output dimensions.");

    if (std::abs(P1.Ts - P2.Ts) > 1e-10 * P1.Ts)
        throw std::invalid_argument(
            "nuGap: both systems must have the same sample time Ts.");

    if (freq_points < 2) freq_points = 2;

    const bool siso = (P1.inputSize() == 1 && P1.outputSize() == 1);
    auto grid = logFreqGrid(P1.Ts, freq_points, omega_min);
    auto H1   = freqResponseGrid(P1, grid);
    auto H2   = freqResponseGrid(P2, grid);

    double gap = 0.0;
    for (int i = 0; i < freq_points; ++i) {
        // SISO: use the exact chordal formula (no SVD overhead).
        // MIMO: subspace chordal distance via normalised-graph SVD (reduces to
        //       chordal for p=m=1 -- see subspaceDist() derivation in header).
        const double d = siso
            ? chordalDist(H1[i](0, 0), H2[i](0, 0))
            : subspaceDist(H1[i], H2[i]);
        if (d > gap) gap = d;
    }
    return gap;
}

double nuGap(const TransferFunction& P1, const TransferFunction& P2,
             int freq_points, double omega_min)
{
    return nuGap(tf2ss(P1), tf2ss(P2), freq_points, omega_min);
}

Eigen::MatrixXd nuGapMatrix(const std::vector<StateSpace>& models, int freq_points)
{
    const int N = static_cast<int>(models.size());
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(N, N);
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            double d  = nuGap(models[i], models[j], freq_points);
            G(i, j)   = d;
            G(j, i)   = d;
        }
    }
    return G;
}

} // namespace ctrl
