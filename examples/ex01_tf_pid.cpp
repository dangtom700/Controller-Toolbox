// ============================================================
//  ex01_tf_pid.cpp
//  Plant defined as a discrete Transfer Function.
//  Tuning: open-loop step response -> StepResponseTuner (FOPDT fit) -> IMC-PID.
//  Closed-loop unit-step response.
//
//  MATLAB equivalent:
//    G   = tf([1],[1 1.5 1]);
//    Gd  = c2d(G, 0.01, 'zoh');
//    data = step(Gd, 15);                  % open-loop step
//    sys  = procest(iddata(data, ones(size(data)), 0.01), 'P2');  % FOPDT fit
//    C    = pidtune(Gd, 'PID');
//    step(feedback(C*Gd, 1));
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <vector>

int main()
{
    // ---- Plant: G(s) = 1/(s^2+1.5s+1) -> ZOH at Ts=0.01s ----
    const double Ts = 0.01;
    ctrl::TransferFunction plant_tf(
        {0.0, 4.9625e-5, 4.9125e-5}, // num: [b0, b1, b2]
        {1.0, -1.98511, 0.98522},    // den: monic [1, a1, a2]
        Ts);

    ctrl::StateSpace plant = ctrl::tf2ss(plant_tf);

    std::cout << "Plant (controllable canonical form):\n"
              << "A =\n"
              << plant.A << "\nB =\n"
              << plant.B
              << "\nC =\n"
              << plant.C << "\n"
              << "DC gain approx = " << (plant.C * (Eigen::MatrixXd::Identity(2, 2) - plant.A).inverse() * plant.B + plant.D)(0, 0) << "\n\n";

    // ---- Open-loop step response (collect 1500 samples = 15s) ----
    const int N_STEP = 1500;
    const double u_step = 1.0;

    std::vector<double> t_data(N_STEP), y_data(N_STEP);
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        Eigen::VectorXd uv(1);
        uv << u_step;
        for (int k = 0; k < N_STEP; ++k)
        {
            y_data[k] = ctrl::ssStep(plant, x, uv)(0);
            t_data[k] = k * Ts;
        }
    }

    // ---- Identify FOPDT model from step data ----
    auto fopdt = ctrl::StepResponseTuner::identify(t_data, y_data, u_step);
    std::cout << "FOPDT fit:  K=" << fopdt.K
              << "  tau=" << fopdt.tau << "s"
              << "  theta=" << fopdt.theta << "s\n";

    // ---- PID via IMC tuning (lambda = 0.5*tau) ----
    ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
        fopdt, Ts, ctrl::PIDTuningRule::IMC);
    pp.uMin = -5.0;
    pp.uMax = 5.0;
    pp.N = 20.0;
    std::cout << "IMC-PID:  Kp=" << std::fixed << std::setprecision(3) << pp.Kp
              << "  Ki=" << pp.Ki << "  Kd=" << pp.Kd << "\n\n";

    ctrl::DiscretePID pid(pp, Ts);

    // ---- Closed-loop unit-step simulation ----
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    const double ref = 1.0;

    std::cout << std::setw(6) << "k" << std::setw(8) << "t[s]"
              << std::setw(10) << "y" << std::setw(10) << "error"
              << std::setw(10) << "u" << "\n";
    std::cout << std::string(44, '-') << "\n";

    for (int k = 0; k <= 1500; ++k)
    {
        const double e = ref - y;
        const double u = pid.compute(e);
        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);

        if (k % 150 == 0)
            std::cout << std::setw(6) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(10) << y
                      << std::setw(10) << e
                      << std::setw(10) << u << "\n";
    }
    return 0;
}
