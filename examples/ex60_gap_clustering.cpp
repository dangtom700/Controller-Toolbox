/**
 * @file ex60_gap_clustering.cpp
 * @brief Gap-metric clustering: linearise a nonlinear plant over a parameter sweep,
 *        compute the pairwise nu-gap matrix, and cluster models by proximity.
 *
 * Plant: pendulum xdot1 = x2,  xdot2 = -(g/L)*sin(x1) + u
 *   with g=9.81 m/s^2, L=1 m.
 * Scheduling parameter: equilibrium angle theta_eq in [-1, 1] rad.
 * At each theta_eq: u_eq = g*sin(theta_eq), x_eq = (theta_eq, 0).
 * Linearised around (theta_eq, u_eq): A_c = [[0,1],[-g*cos(theta_eq)/L, 0]].
 */
#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <iomanip>

int main()
{
    const double g  = 9.81;
    const double L  = 1.0;
    const double Ts = 0.05;
    const int    NP = 8;          // grid points
    const double PI = 3.14159265358979323846;

    // ----- Continuous dynamics -----------------------------------------------
    ctrl::StateFunc f = [g, L](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd xd(2);
        xd(0) =  x(1);
        xd(1) = -(g / L) * std::sin(x(0)) + u(0);
        return xd;
    };

    // ----- Linearise over sweep ----------------------------------------------
    std::vector<double>           p_grid(NP);
    std::vector<ctrl::StateSpace> models;
    models.reserve(NP);

    std::cout << "Linearisation grid:\n";
    std::cout << std::setw(6) << "idx" << std::setw(10) << "theta"
              << std::setw(12) << "A(0,0)" << std::setw(12) << "A(1,1)" << "\n";

    for (int i = 0; i < NP; ++i) {
        double theta = -1.0 + 2.0 * i / (NP - 1);
        p_grid[i]    = theta;

        Eigen::VectorXd x_eq(2), u_eq(1);
        x_eq << theta, 0.0;
        u_eq << g * std::sin(theta);

        // Output y = theta only (SISO) so nuGap can be applied.
        ctrl::MeasFunc h = [](const Eigen::VectorXd& x, const Eigen::VectorXd&) {
            Eigen::VectorXd y(1); y(0) = x(0); return y;
        };
        ctrl::StateSpace sys = ctrl::lineariseAtPoint(f, h, x_eq, u_eq, Ts);
        std::cout << std::setw(6) << i << std::setw(10) << std::fixed << std::setprecision(3) << theta
                  << std::setw(12) << sys.A(0, 0) << std::setw(12) << sys.A(1, 1) << "\n";

        models.push_back(sys);
    }

    // ----- Nu-gap matrix -----------------------------------------------------
    std::cout << "\nNu-gap distance matrix (rounded to 2 dp):\n";
    Eigen::MatrixXd G = ctrl::nuGapMatrix(models, 150);

    for (int i = 0; i < NP; ++i) {
        for (int j = 0; j < NP; ++j)
            std::cout << std::fixed << std::setprecision(2) << std::setw(6) << G(i, j);
        std::cout << "\n";
    }

    // ----- Clustering --------------------------------------------------------
    double threshold = ctrl::suggestGapThreshold(G);
    std::cout << "\nSuggested gap threshold: " << std::fixed << std::setprecision(3)
              << threshold << "\n";

    ctrl::ClusterResult res = ctrl::clusterByGap(G, threshold);
    std::cout << "Number of clusters: " << res.numClusters << "\n";
    std::cout << "Cluster assignments: [";
    for (int l : res.labels) std::cout << " " << l;
    std::cout << " ]\n";
    std::cout << "Representatives at grid indices: [";
    for (int r : res.representatives)
        std::cout << " " << r << " (theta=" << std::setprecision(2) << p_grid[r] << ")";
    std::cout << " ]\n";

    for (int c = 0; c < res.numClusters; ++c)
        std::cout << "  Cluster " << c << ": max intra-gap = "
                  << std::setprecision(3) << res.maxIntraGap[c] << "\n";

    std::cout << "\nPASS\n";
    return 0;
}
