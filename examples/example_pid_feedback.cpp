// ============================================================
//  example_pid_feedback.cpp
//  Demonstrates: define plant -> relay-tune PID -> closed-loop simulation
//
//  Equivalent MATLAB/Simulink workflow:
//    G = tf([1], [1 1.5 1]);           % second-order plant
//    Gd = c2d(G, 0.01, 'zoh');         % discretise with ZOH
//    C = pidtune(Gd, 'PID');           % auto-tune
//    sim('feedback_loop_model.slx');   % simulate
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    // -------------------------------------------------------
    // 1. Plant: continuous G(s) = 1/(s^2 + 1.5s + 1)
    //    Discretised with ZOH at Ts = 0.01 s (MATLAB c2d result):
    //    G(z^-¹) = (4.99e-5 + 4.95e-5.z^-¹) / (1 - 1.985.z^-¹ + 0.985.z^-^2)
    // -------------------------------------------------------
    const double Ts = 0.01; // 10 ms sample time

    ctrl::TransferFunction plant_tf(
        { 0.0,       4.99e-5,  4.95e-5 }, // num: b0, b1, b2
        { 1.0,      -1.98511,  0.98522 }, // den: 1, a1, a2  (monic)
        Ts);

    ctrl::StateSpace plant_ss = ctrl::tf2ss(plant_tf);
    Eigen::VectorXd  x        = Eigen::VectorXd::Zero(plant_ss.stateSize());

    // -------------------------------------------------------
    // 2. Relay auto-tuner - identifies ultimate gain / period
    //    (run open-loop relay test on the plant in simulation)
    // -------------------------------------------------------
    ctrl::RelayTunerConfig relay_cfg;
    relay_cfg.relayAmplitude = 0.5;
    relay_cfg.cyclesRequired = 4;

    ctrl::RelayAutoTuner tuner(relay_cfg, Ts);
    Eigen::VectorXd      x_tune = Eigen::VectorXd::Zero(plant_ss.stateSize());

    double y_tune = 0.0;
    while (!tuner.isDone()) {
        double u_relay = tuner.step(y_tune);
        Eigen::VectorXd uv(1); uv << u_relay;
        y_tune = ctrl::ssStep(plant_ss, x_tune, uv)(0);
    }

    std::cout << "Relay test -> Ku = " << std::fixed << std::setprecision(4)
              << tuner.ultimateGain() << ", Tu = " << tuner.ultimatePeriod() << " s\n";

    // -------------------------------------------------------
    // 3. PID parameters via Tyreus-Luyben rule
    // -------------------------------------------------------
    ctrl::PIDParams pid_p = tuner.computePIDParams(ctrl::PIDTuningRule::TyreusLuyben);
    pid_p.uMin = -20.0;
    pid_p.uMax =  20.0;
    pid_p.N    = 20.0;

    std::cout << "PID: Kp=" << pid_p.Kp
              << " Ki="     << pid_p.Ki
              << " Kd="     << pid_p.Kd << "\n";

    // -------------------------------------------------------
    // 4. Build a ControllerStack (Supervisory, single PID entry).
    //    Adding MPC or LQR later only requires addController() - loop unchanged.
    // -------------------------------------------------------
    ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
    stack.addController(std::make_shared<ctrl::DiscretePID>(pid_p, Ts), "PID_primary");

    // -------------------------------------------------------
    // 5. Closed-loop unit-step simulation  (N = 600 steps = 6 s)
    // -------------------------------------------------------
    const double ref   = 1.0;
    const int    N_SIM = 600;
    double       y     = 0.0;

    std::cout << "\nStep, t[s], y,      e,       u\n";
    std::cout << std::string(45, '-') << "\n";

    for (int k = 0; k < N_SIM; ++k) {
        double error = ref - y;
        double u     = stack.compute(error);

        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(plant_ss, x, uv)(0);

        if (k % 50 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(4)
                      << "  " << k * Ts
                      << "  " << y
                      << "  " << error
                      << "  " << u << "\n";
    }

    return 0;
}
