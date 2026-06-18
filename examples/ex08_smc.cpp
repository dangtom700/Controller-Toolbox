// ============================================================
//  ex08_smc.cpp
//  Sliding Mode Controller on a 2nd-order plant with
//  +/-30 % parameter uncertainty. Compares SMC vs PID.
//
//  Nominal plant: G(s) = 1/(s^2+2s+1)
//  Perturbed:     G'(s) = 1.3/(s^2+1.4s+0.7)   (gain +30%, poles shifted)
//
//  MATLAB equivalent:
//    % Sliding mode is in Simulink Variable Structure Control library
//    % or: custom implementation with sign() / sat() function
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    // ---- Nominal plant (for controller design) ----
    ctrl::TransferFunction tf_nom(
        { 0.0, 4.975e-5, 4.925e-5 },
        { 1.0, -1.9800,   0.9802  }, Ts);
    ctrl::StateSpace nom = ctrl::tf2ss(tf_nom);

    // ---- Perturbed plant (for simulation) - +/-30% uncertainty ----
    ctrl::TransferFunction tf_pert(
        { 0.0, 6.45e-5, 6.35e-5 },
        { 1.0, -1.9860, 0.9862  }, Ts);
    ctrl::StateSpace pert = ctrl::tf2ss(tf_pert);

    // ---- SMC tuning ----
    ctrl::SMCParams sp;
    sp.c_e  = 1.0;
    sp.c_de = 0.8;
    sp.K    = 6.0;
    sp.phi  = 0.4;   // boundary layer - avoids chattering
    sp.uMin = -10.0;
    sp.uMax =  10.0;

    ctrl::DiscreteSMC smc(sp, Ts);

    // ---- PID (relay-tuned on nominal) ----
    ctrl::RelayTunerConfig rc; rc.relayAmplitude=0.5; rc.cyclesRequired=4;
    ctrl::RelayAutoTuner tuner(rc, Ts);
    Eigen::VectorXd xt = Eigen::VectorXd::Zero(nom.stateSize());
    double yt = 0.0;
    while (!tuner.isDone()) {
        Eigen::VectorXd uv(1); uv << tuner.step(yt);
        yt = ctrl::ssStep(nom, xt, uv)(0);
    }
    ctrl::PIDParams pp = tuner.computePIDParams(ctrl::PIDTuningRule::TyreusLuyben);
    pp.uMin = -10.0; pp.uMax = 10.0;
    ctrl::DiscretePID pid(pp, Ts);

    const double ref = 1.0;

    // ---- SMC on perturbed plant ----
    std::cout << "=== SMC on perturbed plant ===\n";
    std::cout << std::setw(8) << "t[s]" << std::setw(10) << "y"
              << std::setw(10) << "s(surf)" << std::setw(8) << "u" << "\n";
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(pert.stateSize());
        double y = 0.0;
        for (int k = 0; k <= 500; ++k) {
            double u = smc.compute(ref - y);
            Eigen::VectorXd uv(1); uv << u;
            y = ctrl::ssStep(pert, x, uv)(0);
            if (k % 50 == 0)
                std::cout << std::fixed << std::setprecision(3)
                          << std::setw(8) << k * Ts
                          << std::setw(10) << y
                          << std::setw(10) << smc.slidingSurface()
                          << std::setw(8) << u << "\n";
        }
    }

    // ---- PID on perturbed plant ----
    std::cout << "\n=== PID on perturbed plant ===\n";
    std::cout << std::setw(8) << "t[s]" << std::setw(10) << "y"
              << std::setw(8) << "u" << "\n";
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(pert.stateSize());
        double y = 0.0;
        for (int k = 0; k <= 500; ++k) {
            double u = pid.compute(ref - y);
            Eigen::VectorXd uv(1); uv << u;
            y = ctrl::ssStep(pert, x, uv)(0);
            if (k % 50 == 0)
                std::cout << std::fixed << std::setprecision(3)
                          << std::setw(8) << k * Ts
                          << std::setw(10) << y
                          << std::setw(8) << u << "\n";
        }
    }
    return 0;
}
