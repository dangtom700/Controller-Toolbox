// ============================================================
//  ex06_lead_lag.cpp
//  Phase-lead compensator for a slow first-order plant.
//  Compares step response: proportional only vs P + Lead.
//
//  Design goal: add ~45^\circ phase lead at crossover to improve
//  phase margin and reduce rise time.
//
//  MATLAB equivalent:
//    G  = tf([1],[1 0.2]);
//    Gd = c2d(G, 0.05, 'zoh');
//    C  = c2d(tf([1 2],[1 20]), 0.05, 'tustin');  % lead compensator
//    rlocus(Gd);  bode(C*Gd);
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.05;

    // ---- Plant G(s) = 1/(s+0.2) -> ZOH ----
    // G(z^-¹) approx = (0.00995) / (1 - 0.99z^-¹)
    ctrl::TransferFunction tf({ 0.0, 0.00995 }, { 1.0, -0.990 }, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    // ---- Lead compensator C(s) = K*(s+z_c)/(s+p_c) ----
    // z_c=2, p_c=20, K=1 -> adds ~45^\circ at ~6 rad/s crossover
    ctrl::LeadLagParams llp;
    llp.continuousZero = 2.0;
    llp.continuousPole = 20.0;
    llp.gain           = 1.0;

    ctrl::DiscreteLeadLag lead(llp, Ts);

    // Phase contributed at several frequencies
    for (double w : {1.0, 2.0, 5.0, 10.0, 20.0})
        std::cout << "Lead phase at " << std::setw(4) << w
                  << " rad/s: " << std::fixed << std::setprecision(1)
                  << lead.phaseAt(w) * 180.0 / M_PI << " deg\n";
    std::cout << "\n";

    const double Kp = 5.0; // proportional gain in both cases

    // ---- Run 1: proportional only ----
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        double y = 0.0;
        std::cout << "=== Proportional only (Kp=" << Kp << ") ===\n";
        std::cout << std::setw(8) << "t[s]" << std::setw(10) << "y" << "\n";

        for (int k = 0; k <= 300; ++k) {
            double e  = 1.0 - y;
            double u  = Kp * e;
            Eigen::VectorXd uv(1); uv << u;
            y = ctrl::ssStep(plant, x, uv)(0);
            if (k % 30 == 0)
                std::cout << std::fixed << std::setprecision(3)
                          << std::setw(8) << k * Ts
                          << std::setw(10) << y << "\n";
        }
    }

    // ---- Run 2: Lead + proportional ----
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        lead.reset();
        double y = 0.0;
        std::cout << "\n=== Lead compensator + Kp=" << Kp << " ===\n";
        std::cout << std::setw(8) << "t[s]" << std::setw(10) << "y" << "\n";

        for (int k = 0; k <= 300; ++k) {
            double e       = 1.0 - y;
            double u_lead  = lead.compute(e);   // error -> lead filter -> plant
            double u       = Kp * u_lead;
            Eigen::VectorXd uv(1); uv << u;
            y = ctrl::ssStep(plant, x, uv)(0);
            if (k % 30 == 0)
                std::cout << std::fixed << std::setprecision(3)
                          << std::setw(8) << k * Ts
                          << std::setw(10) << y << "\n";
        }
    }
    return 0;
}
