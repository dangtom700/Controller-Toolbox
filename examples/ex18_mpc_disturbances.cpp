// ============================================================
//  ex18_mpc_disturbances.cpp
//  Demonstrates MPC rejecting step disturbances acting on the input.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.05;

    // Plant
    ctrl::TransferFunction tf({0.0, 0.0487}, {1.0, -0.9512}, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    ctrl::MPCParams mp;
    mp.Np = 40;
    mp.Nc = 10;
    mp.rho_y = 1.0;
    mp.rho_u = 0.1;
    mp.uMin = -5.0; mp.uMax = 5.0;
    
    ctrl::DiscreteMPC mpc(plant, mp);

    std::cout << "=== MPC Disturbance Rejection ===\n";
    std::cout << "  k    t[s]      y       u     Dist\n";
    std::cout << std::string(45, '-') << "\n";

    double y = 0.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    const double ref = 1.0;

    for (int k = 0; k <= 200; ++k) {
        double dist = (k * Ts >= 5.0) ? -2.0 : 0.0; // Input disturbance at t=5s
        
        double e = ref - y;
        double u = mpc.compute(e);

        Eigen::VectorXd uv(1); uv << (u + dist);
        y = ctrl::ssStep(plant, x, uv)(0);

        if (k % 20 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(9) << y
                      << std::setw(8) << u
                      << std::setw(8) << dist << "\n";
    }

    return 0;
}
