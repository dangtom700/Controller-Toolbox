// ============================================================
//  ex16_adrc_time_varying.cpp
//  Demonstrates Active Disturbance Rejection Control (ADRC)
//  compensating for a sudden change in plant parameters.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    // Nominal Plant: 1/(s^2 + 2s + 1)
    ctrl::TransferFunction tf_nom({0.0, 4.98e-5, 4.97e-5}, {1.0, -1.98, 0.98}, Ts);
    ctrl::StateSpace plant_nom = ctrl::tf2ss(tf_nom);

    // Degraded Plant (Gain drops by 50%, poles shift): 0.5/(s^2 + s + 0.5)
    ctrl::TransferFunction tf_deg({0.0, 2.49e-5, 2.48e-5}, {1.0, -1.99, 0.99}, Ts);
    ctrl::StateSpace plant_deg = ctrl::tf2ss(tf_deg);

    // ADRC Design (Tuned on nominal plant assumptions)
    ctrl::ADRCParams ap;
    ap.omega_c = 4.0;
    ap.omega_o = 20.0;
    ap.b0      = 1.0; // Nominal high-frequency gain estimate
    ap.uMin    = -10.0;
    ap.uMax    =  10.0;

    ctrl::DiscreteADRC adrc(ap, Ts);
    const double ref = 1.0;

    std::cout << "=== ADRC on Time-Varying Plant ===\n";
    std::cout << "  k    t[s]   Plant_Type      y      u     ESO_z3(DistEst)\n";
    std::cout << std::string(62, '-') << "\n";

    double y = 0.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant_nom.stateSize());

    for (int k = 0; k <= 600; ++k) {
        adrc.setReference(ref);
        double u = adrc.compute(y);
        Eigen::VectorXd uv(1); uv << u;

        // Switch to degraded plant at t = 3.0s
        bool isDegraded = (k * Ts >= 3.0);
        if (isDegraded) {
            y = ctrl::ssStep(plant_deg, x, uv)(0);
        } else {
            y = ctrl::ssStep(plant_nom, x, uv)(0);
        }

        if (k % 50 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << (isDegraded ? "   Degraded " : "   Nominal  ")
                      << std::setw(8) << y
                      << std::setw(8) << u
                      << std::setw(12) << adrc.esoState()(2) << "\n";
    }

    return 0;
}
