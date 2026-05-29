#include "GapMetric.h"
#include "PlantModel.h"
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

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

double nuGap(const StateSpace& P1, const StateSpace& P2,
             int freq_points, double omega_min)
{
    if (P1.inputSize()  != 1 || P1.outputSize() != 1 ||
        P2.inputSize()  != 1 || P2.outputSize() != 1)
        throw std::invalid_argument(
            "nuGap: only SISO systems accepted (inputSize=1, outputSize=1).");

    if (std::abs(P1.Ts - P2.Ts) > 1e-10 * P1.Ts)
        throw std::invalid_argument(
            "nuGap: both systems must have the same sample time Ts.");

    if (freq_points < 2) freq_points = 2;

    auto grid = logFreqGrid(P1.Ts, freq_points, omega_min);
    auto H1   = freqResponseGrid(P1, grid);
    auto H2   = freqResponseGrid(P2, grid);

    double gap = 0.0;
    for (int i = 0; i < freq_points; ++i) {
        double d = chordalDist(H1[i](0, 0), H2[i](0, 0));
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
