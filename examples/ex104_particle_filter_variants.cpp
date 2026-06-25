/**
 * @file ex104_particle_filter_variants.cpp
 * @brief Phase 3 Roadmap Phase 2 (EF3): Bootstrap vs Auxiliary vs Rao-Blackwellized PF.
 *
 * A 2-state tracking problem with a nonlinear angle-like state (theta) and a linear-Gaussian
 * velocity (v), additively coupled in the measurement (y = sin(theta) + v) - the standard
 * "mixed linear/nonlinear" structure Rao-Blackwellization targets. Compares all three variants'
 * RMSE on the velocity (linear) substate at a low, shared particle count.
 */

#include "ControllerToolbox.h"
#include <iostream>
#include <random>

namespace
{
double rmseOf(const std::vector<double> &est, const std::vector<double> &truth)
{
    double sse = 0.0;
    for (size_t i = 0; i < est.size(); ++i)
    {
        const double e = est[i] - truth[i];
        sse += e * e;
    }
    return std::sqrt(sse / static_cast<double>(est.size()));
}
} // namespace

int main()
{
    const double Ts = 0.1;
    const int N = 150;
    const int nParticles = 40; // deliberately low, to favour Rao-Blackwellization

    auto f = [Ts](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        xn(0) = x(0) + x(1) * Ts + u(0);
        xn(1) = 0.0; // overwritten by the embedded KF in RB mode; irrelevant in Bootstrap/Auxiliary
        return xn;
    };
    auto h = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        Eigen::VectorXd y(1); y(0) = std::sin(x(0)) + x(1); return y;
    };

    Eigen::MatrixXd A_lin(1, 1); A_lin << 1.0;
    Eigen::MatrixXd B_lin(1, 1); B_lin << 0.0;
    Eigen::MatrixXd C_lin(1, 1); C_lin << 1.0;
    Eigen::MatrixXd Q_lin(1, 1); Q_lin << 0.001;
    Eigen::MatrixXd R_lin(1, 1); R_lin << 0.05;

    // True trajectory: theta evolves nonlinearly, v is a slowly-varying linear-Gaussian process.
    std::mt19937 plantRng(11);
    std::normal_distribution<double> procNoise(0.0, 0.01);
    std::normal_distribution<double> measNoise(0.0, std::sqrt(0.05));
    std::vector<double> vTrue(N), yMeas(N);
    double theta = 0.0, v = 0.5;
    for (int k = 0; k < N; ++k)
    {
        v += procNoise(plantRng);
        theta += v * Ts;
        vTrue[k] = v;
        yMeas[k] = std::sin(theta) + v + measNoise(plantRng);
    }

    auto runVariant = [&](ctrl::PFVariant variant) {
        ctrl::ParticleFilterParamsV2 p;
        p.n_particles = nParticles;
        p.Q = Eigen::MatrixXd::Identity(2, 2) * 0.001;
        p.R = Eigen::MatrixXd::Constant(1, 1, 0.05);
        p.seed = 21u;
        p.variant = variant;
        p.linear_state_indices = {1};

        ctrl::ParticleFilterV2 pf(p, 2, 1, f, h, A_lin, B_lin, C_lin, Q_lin, R_lin);
        pf.initialise(Eigen::VectorXd::Zero(2));

        std::vector<double> vEst(N);
        const Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);
        for (int k = 0; k < N; ++k)
        {
            Eigen::VectorXd y(1); y(0) = yMeas[k];
            pf.step(y, u0);
            vEst[k] = pf.state()(1);
        }
        return rmseOf(vEst, vTrue);
    };

    const double rmseBootstrap = runVariant(ctrl::PFVariant::Bootstrap);
    const double rmseAuxiliary = runVariant(ctrl::PFVariant::Auxiliary);
    const double rmseRB = runVariant(ctrl::PFVariant::RaoBlackwellized);

    std::cout << "Velocity RMSE @ N=" << nParticles << " particles:\n";
    std::cout << "  Bootstrap:        " << rmseBootstrap << "\n";
    std::cout << "  Auxiliary:        " << rmseAuxiliary << "\n";
    std::cout << "  RaoBlackwellized: " << rmseRB << "\n";

    const bool ok = std::isfinite(rmseBootstrap) && std::isfinite(rmseAuxiliary) &&
                    std::isfinite(rmseRB) && rmseRB < rmseBootstrap;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
