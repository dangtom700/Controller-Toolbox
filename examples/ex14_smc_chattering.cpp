// ============================================================
//  ex14_smc_chattering.cpp
//  Demonstrates Sliding Mode Control (SMC) chattering reduction
//  using the boundary layer parameter (phi).
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    // Plant
    ctrl::TransferFunction tf({0.0, 4.9e-5, 4.9e-5}, {1.0, -1.98, 0.98}, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    ctrl::SMCParams p;
    p.c_e = 1.0; p.c_de = 0.5; p.K = 10.0;
    p.uMin = -15.0; p.uMax = 15.0;

    // Controller 1: Pure sign function (tiny phi) -> Chattering
    p.phi = 1e-6;
    ctrl::DiscreteSMC smc_chat(p, Ts);

    // Controller 2: Smooth boundary layer -> No Chattering
    p.phi = 0.5;
    ctrl::DiscreteSMC smc_smooth(p, Ts);

    std::cout << "=== SMC Chattering Comparison ===\n";
    std::cout << "  k    t[s]   u(phi=1e-6)   u(phi=0.5)\n";
    std::cout << std::string(45, '-') << "\n";

    double y1 = 0.0, y2 = 0.0;
    Eigen::VectorXd x1 = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd x2 = Eigen::VectorXd::Zero(plant.stateSize());
    const double ref = 1.0;

    for (int k = 0; k <= 100; ++k) {
        double u1 = smc_chat.compute(ref - y1);
        double u2 = smc_smooth.compute(ref - y2);

        Eigen::VectorXd uv1(1); uv1 << u1;
        Eigen::VectorXd uv2(1); uv2 << u2;
        
        y1 = ctrl::ssStep(plant, x1, uv1)(0);
        y2 = ctrl::ssStep(plant, x2, uv2)(0);

        // Near steady state, observe the control input
        if (k > 80 && k % 2 == 0) {
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(4)
                      << std::setw(8) << k * Ts
                      << std::setw(12) << u1
                      << std::setw(14) << u2 << "\n";
        }
    }
    return 0;
}
