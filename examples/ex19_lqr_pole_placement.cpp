// ============================================================
//  ex19_lqr_pole_placement.cpp
//  Uses the LQRWeightTuner's polePlacementHint to find Q and R
//  that push closed-loop poles toward desired locations.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <complex>

int main()
{
    const double Ts = 0.05;

    // Plant: 1/(s^2 + 1) oscillator -> discretised
    Eigen::Matrix2d Ac; Ac << 0.0, 1.0, -1.0, 0.0;
    Eigen::Vector2d Bc; Bc << 0.0, 1.0;
    Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity() + Ts * Ac;
    Eigen::Vector2d Bd = Ts * Bc;
    ctrl::StateSpace plant(Ad, Bd, Eigen::RowVector2d(1,0), Eigen::MatrixXd::Zero(1,1), Ts);

    // Desired continuous poles: -2 +/- 2j
    // Discrete poles: z = exp(s * Ts)
    std::complex<double> s1(-2.0, 2.0);
    std::complex<double> s2(-2.0, -2.0);
    
    std::vector<std::complex<double>> targetPoles = {
        std::exp(s1 * Ts),
        std::exp(s2 * Ts)
    };

    std::cout << "Target discrete poles:\n" 
              << "  p1 = " << targetPoles[0] << "\n"
              << "  p2 = " << targetPoles[1] << "\n\n";

    // Tune
    ctrl::LQRParams p = ctrl::LQRWeightTuner::polePlacementHint(plant, targetPoles);

    std::cout << "Identified Weights via Hint Method:\n";
    std::cout << "Q =\n" << p.Q << "\n";
    std::cout << "R =\n" << p.R << "\n\n";

    ctrl::DiscreteLQR lqr(plant, p);
    std::cout << "LQR Gain Matrix K = " << lqr.gainMatrix() << "\n\n";

    // Simulation
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::Vector2d x_ref; x_ref << 1.0, 0.0;

    std::cout << "  k    t[s]    x(0)    x(1)      u\n";
    std::cout << std::string(45, '-') << "\n";

    for (int k = 0; k <= 100; ++k) {
        double u = lqr.compute(x, x_ref)(0);
        Eigen::VectorXd uv(1); uv << u;
        ctrl::ssStep(plant, x, uv);

        if (k % 10 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(8) << x(0)
                      << std::setw(8) << x(1)
                      << std::setw(8) << u << "\n";
    }

    return 0;
}
