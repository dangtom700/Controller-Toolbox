// ============================================================
//  ex12_weighted_stack.cpp
//  ControllerStack in Weighted mode:
//    Demonstrates smooth blending between two controllers.
//    At start, Controller A (aggressive) has weight 1.0, B has 0.0.
//    We linearly crossfade to A=0.0, B=1.0 over the simulation.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    // Plant: 1st order
    ctrl::TransferFunction tf({0.0, 0.01}, {1.0, -0.99}, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    // Controller A: Aggressive Proportional
    ctrl::PIDParams pA; pA.Kp = 10.0; pA.Ki = 0.0; pA.Kd = 0.0;
    auto ctrlA = std::make_shared<ctrl::DiscretePID>(pA, Ts);

    // Controller B: Gentle Proportional
    ctrl::PIDParams pB; pB.Kp = 1.0; pB.Ki = 0.0; pB.Kd = 0.0;
    auto ctrlB = std::make_shared<ctrl::DiscretePID>(pB, Ts);

    // Build Weighted Stack
    ctrl::ControllerStack stack(ctrl::StackMode::Weighted, Ts);
    stack.addController(ctrlA, "Aggressive", 1.0);
    stack.addController(ctrlB, "Gentle", 0.0);

    // Simulation
    double y = 0.0, ref = 1.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());

    std::cout << "=== Weighted Stack Blending ===\n";
    std::cout << "  k    t[s]      y       u   wA(Aggr) wB(Gentle)\n";
    std::cout << std::string(55, '-') << "\n";

    const int N = 200;
    for (int k = 0; k <= N; ++k) {
        // Crossfade weights
        double wA = 1.0 - static_cast<double>(k) / N;
        double wB = 1.0 - wA;
        
        stack.setWeight("Aggressive", wA);
        stack.setWeight("Gentle", wB);

        double u = stack.compute(ref - y);
        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);

        if (k % 20 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(8) << y
                      << std::setw(8) << u
                      << std::setw(9) << wA
                      << std::setw(9) << wB << "\n";
    }
    return 0;
}
