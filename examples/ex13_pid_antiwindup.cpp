// ============================================================
//  ex13_pid_antiwindup.cpp
//  Demonstrates Integral Anti-windup (back-calculation) in PID.
//  Compares a heavily saturated process with and without Kb.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.05;

    // Plant: 1/(s+1)
    ctrl::TransferFunction tf({0.0, 0.0487}, {1.0, -0.9512}, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    // Common PID parameters (high integral to force windup)
    ctrl::PIDParams p;
    p.Kp = 2.0; p.Ki = 5.0; p.Kd = 0.0;
    p.uMin = -1.0; p.uMax = 1.0; // Strict saturation

    // Controller 1: No anti-windup (Kb = 0)
    p.Kb = 0.0;
    ctrl::DiscretePID pid_no_aw(p, Ts);

    // Controller 2: With anti-windup (Kb > 0)
    p.Kb = 2.0; // Typical is ~sqrt(Ki/Kd) or simply 1/Ti
    ctrl::DiscretePID pid_aw(p, Ts);

    std::cout << "=== PID Anti-Windup Comparison ===\n";
    std::cout << "  k    t[s]   y(No AW)   u(No AW) |   y(With AW)  u(With AW)\n";
    std::cout << std::string(64, '-') << "\n";

    double y1 = 0.0, y2 = 0.0;
    Eigen::VectorXd x1 = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd x2 = Eigen::VectorXd::Zero(plant.stateSize());
    
    // Large step to cause saturation
    const double ref = 5.0; 

    for (int k = 0; k <= 100; ++k) {
        double u1 = pid_no_aw.compute(ref - y1);
        double u2 = pid_aw.compute(ref - y2);

        Eigen::VectorXd uv1(1); uv1 << u1;
        Eigen::VectorXd uv2(1); uv2 << u2;
        
        y1 = ctrl::ssStep(plant, x1, uv1)(0);
        y2 = ctrl::ssStep(plant, x2, uv2)(0);

        if (k % 10 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(10) << y1 << std::setw(10) << u1 << " | "
                      << std::setw(10) << y2 << std::setw(10) << u2 << "\n";
    }
    
    // Drop reference to 0.0 to observe un-windup behavior
    std::cout << "\n--- Reference drops to 0.0 ---\n";
    for (int k = 101; k <= 200; ++k) {
        double u1 = pid_no_aw.compute(0.0 - y1);
        double u2 = pid_aw.compute(0.0 - y2);

        Eigen::VectorXd uv1(1); uv1 << u1;
        Eigen::VectorXd uv2(1); uv2 << u2;
        
        y1 = ctrl::ssStep(plant, x1, uv1)(0);
        y2 = ctrl::ssStep(plant, x2, uv2)(0);

        if (k % 10 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(10) << y1 << std::setw(10) << u1 << " | "
                      << std::setw(10) << y2 << std::setw(10) << u2 << "\n";
    }

    return 0;
}
