// ============================================================
//  ex24_fuzzy_pid_dc_motor.cpp
//  Application: DC motor speed control with load torque step.
//
//  Plant: armature-controlled DC motor
//    G(s) = Km / (s(tau_m s + 1))   [rad/s / V]
//    Km = 100 rad/s/V,  tau_m = 0.05 s
//  Discretised at Ts = 0.002 s (ZOH).
//
//  Demonstrates:
//    1. FuzzyPID tracking a speed ramp (0 -> 50 -> 100 -> 50 rad/s)
//    2. FuzzyPID anti-windup during 0.5 s voltage saturation (|u| <= 12 V)
//    3. Comparison with DiscretePID under the same conditions
//
//  Output columns: t, ref, y_fuzzy, u_fuzzy, y_pid, u_pid, sat_fuzzy, sat_pid
// ============================================================
#include "ControllerToolbox.h"
#include <fstream>
#include <iomanip>
#include <iostream>

int main()
{
    const double Ts = 0.002;

    // -- Plant: Km/(s(tau s+1)) -> state: [omega, i_a] linearised
    // Simplified: integrating second-order in velocity form
    // G(s) = 100/(s(0.05s+1))  ->  TF numerator/denominator
    // Discrete approximation at Ts=0.002 via bilinear (just use tf2ss+c2d)
    ctrl::TransferFunction tf({0.0, 0.0, 100.0 * Ts},
                               {1.0, -(1.0 + std::exp(-Ts/0.05)),
                                      std::exp(-Ts/0.05)}, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    const double U_MAX = 12.0;   // voltage rail +/-12 V

    // -- FuzzyPID -------------------------------------------------------------
    // e_scale  = 30 rad/s  (large error for this motor)
    // de_scale = 500 rad/s^2 (max angular acceleration)
    // u_scale  = 10 V      (maps fuzzy output to volts)
    ctrl::FuzzyPIDParams fp;
    fp.pd.e_scale  = 30.0;
    fp.pd.de_scale = 500.0;
    fp.pd.u_scale  = 10.0;
    fp.pd.uMin     = -U_MAX;
    fp.pd.uMax     =  U_MAX;
    fp.Ki          = 2.0;
    fp.Kb          = 0.8;    // anti-windup back-calculation gain
    fp.uMin        = -U_MAX;
    fp.uMax        =  U_MAX;
    ctrl::FuzzyPID fuzzy(fp, Ts);

    // -- Baseline PID (velocity form, same limits) -----------------------------
    ctrl::PIDParams pp;
    pp.Kp  = 0.18;
    pp.Ki  = 3.6;
    pp.Kd  = 0.001;
    pp.N   = 50.0;
    pp.Kb  = 0.8;
    pp.uMin = -U_MAX;
    pp.uMax =  U_MAX;
    ctrl::DiscretePID pid(pp, Ts);

    // -- Speed reference profile -----------------------------------------------
    auto ref = [](double t) -> double {
        if (t < 0.5)  return 0.0;
        if (t < 1.0)  return 100.0 * (t - 0.5) / 0.5;   // ramp up
        if (t < 2.5)  return 100.0;                        // hold
        if (t < 3.0)  return 100.0 - 100.0 * (t - 2.5) / 0.5; // ramp down
        return 50.0;
    };

    // -- Simulation ------------------------------------------------------------
    const int N = 2500;  // 5 s at 500 Hz
    double yf = 0.0, yp = 0.0;
    Eigen::VectorXd xf = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd xp = Eigen::VectorXd::Zero(plant.stateSize());

    std::ofstream csv("data/ex24_fuzzy_dc_motor.csv");
    csv << "t,ref,y_fuzzy,u_fuzzy,y_pid,u_pid,sat_fuzzy,sat_pid\n";
    csv << std::fixed << std::setprecision(5);

    std::cout << "=== FuzzyPID vs PID - DC Motor Speed Control ===\n";
    std::cout << "   k     t[s]   ref     y_fuzzy  u_fuzzy   y_pid   u_pid\n";
    std::cout << std::string(67, '-') << "\n";

    for (int k = 0; k < N; ++k) {
        double t = k * Ts;
        double r = ref(t);

        double uf_raw = fuzzy.compute(r - yf);
        double up_raw = pid.compute(r - yp);

        bool sat_f = std::abs(uf_raw) >= U_MAX - 1e-6;
        bool sat_p = std::abs(up_raw) >= U_MAX - 1e-6;

        Eigen::VectorXd uvf(1); uvf << uf_raw;
        Eigen::VectorXd uvp(1); uvp << up_raw;

        yf = ctrl::ssStep(plant, xf, uvf)(0);
        yp = ctrl::ssStep(plant, xp, uvp)(0);

        csv << t << "," << r << "," << yf << "," << uf_raw << ","
            << yp << "," << up_raw << "," << sat_f << "," << sat_p << "\n";

        if (k % 250 == 0)
            std::cout << std::setw(5) << k
                      << std::setw(8)  << t
                      << std::setw(7)  << r
                      << std::setw(10) << yf
                      << std::setw(9)  << uf_raw
                      << std::setw(9)  << yp
                      << std::setw(8)  << up_raw << "\n";
    }

    std::cout << "\nResults saved to data/ex24_fuzzy_dc_motor.csv\n";
    return 0;
}
