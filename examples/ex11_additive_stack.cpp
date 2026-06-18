// ============================================================
//  ex11_additive_stack.cpp
//  ControllerStack in Additive mode:
//    Demonstrates parallel composition of controllers.
//    We use a base PID controller for steady-state tracking 
//    and a Lead-Lag compensator in parallel to provide 
//    additional transient kick during fast changes.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    // ---- Plant: G(s) = 1/(s^2 + 2s + 1) ----
    ctrl::TransferFunction tf(
        { 0.0, 4.983e-5, 4.967e-5 },
        { 1.0, -1.980,  0.9802   }, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    // ---- 1. Base PID Controller ----
    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 1.0; pp.Kd = 0.0; // PI only
    pp.uMin = -10.0; pp.uMax = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // ---- 2. Lead-Lag Compensator (for transient) ----
    ctrl::LeadLagParams llp;
    llp.continuousZero = 2.0;
    llp.continuousPole = 20.0;
    llp.gain = 0.5;
    auto lead = std::make_shared<ctrl::DiscreteLeadLag>(llp, Ts);

    // ---- Build Additive Stack ----
    ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
    
    // Additive mode simply sums outputs of all eligible controllers.
    // Weights act as constant scaling factors on each controller's output.
    stack.addController(pid, "Base_PI", 1.0);
    stack.addController(lead, "Transient_Lead", 1.0);

    // ---- Simulation ----
    double y = 0.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    const double ref = 1.0;

    std::cout << "=== Additive Stack Simulation ===\n";
    std::cout << std::setw(6) << "k" << std::setw(8) << "t[s]"
              << std::setw(10) << "y" << std::setw(10) << "u_total" << "\n";
    std::cout << std::string(36, '-') << "\n";

    for (int k = 0; k <= 300; ++k) {
        double e = ref - y;
        double u = stack.compute(e); // u = pid(e) + lead(e)

        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);

        if (k % 30 == 0)
            std::cout << std::setw(6) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(10) << y
                      << std::setw(10) << u << "\n";
    }
    return 0;
}
