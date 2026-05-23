// ============================================================
//  ex09_adrc.cpp
//  Active Disturbance Rejection Control on a 2nd-order plant.
//  A step disturbance is injected at t=3s to test rejection.
//
//  MATLAB equivalent:
//    % ADRC / LADRC: implement ESO manually or use MATLAB Central
//    % file "LADRC_2DOF_example.m" by Gao (2003)
//    % ESO bandwidth: omega_o = 20 rad/s
//    % Controller BW: omega_c = 5 rad/s
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    // ---- Plant: G(s) = 1/(s^2+2s+1) (actual dynamics, unknown to ADRC) ----
    Eigen::Matrix2d Ac; Ac <<  0.0, 1.0,
                              -1.0, -2.0;
    Eigen::Vector2d Bc; Bc << 0.0, 1.0;
    Eigen::RowVector2d Cc; Cc << 1.0, 0.0;
    Eigen::MatrixXd Dc(1,1); Dc << 0.0;
    const Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity() + Ts * Ac;
    const Eigen::Vector2d Bd = Ts * Bc;
    ctrl::StateSpace plant(Ad, Bd, Cc, Dc, Ts);

    // ---- ADRC tuning ----
    ctrl::ADRCParams ap;
    ap.omega_o = 20.0; // ESO bandwidth: observes disturbance quickly
    ap.omega_c = 5.0;  // controller bandwidth
    ap.b0      = 1.0;  // approximate input gain (ADRC is robust to this estimate)
    ap.uMin    = -20.0;
    ap.uMax    =  20.0;

    ctrl::DiscreteADRC adrc(ap, Ts);

    const double ref = 1.0;

    std::cout << "k     t[s]   y      z1(est)  z3(dist)  u    disturbance\n";
    std::cout << std::string(55, '-') << "\n";

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;

    for (int k = 0; k <= 800; ++k) {
        const double t         = k * Ts;
        const double disturbance = (t >= 3.0) ? 2.0 : 0.0; // step disturbance at 3s

        adrc.setReference(ref);
        double u = adrc.compute(y);

        // Plant simulation with external disturbance added to input
        Eigen::VectorXd uv(1); uv << (u + disturbance);
        y = ctrl::ssStep(plant, x, uv)(0);

        const auto& z = adrc.esoState();

        if (k % 80 == 0)
            std::cout << std::setw(4)  << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(7)  << t
                      << std::setw(7)  << y
                      << std::setw(9)  << z(0)
                      << std::setw(10) << z(2)
                      << std::setw(7)  << u
                      << std::setw(13) << disturbance << "\n";
    }

    std::cout << "\nNote: z3 (ESO state 3) tracks the disturbance estimate.\n";
    return 0;
}
