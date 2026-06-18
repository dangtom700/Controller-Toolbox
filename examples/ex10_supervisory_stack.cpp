// ============================================================
//  ex10_supervisory_stack.cpp
//  ControllerStack in Supervisory mode:
//    - Large error  (|e| > 0.3): SMC handles the fast, nonlinear regime
//    - Small error  (|e| <= 0.3): LQR takes over for near-optimal regulation
//
//  The stack evaluates activation conditions in insertion order and
//  selects the first eligible controller.
//
//  MATLAB equivalent:
//    % Simulink Switch block + two controllers
//    % or: Gain-Scheduled Control (MATLAB gainSched toolbox)
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    // ---- 2nd-order plant ----
    ctrl::TransferFunction tf(
        { 0.0, 4.975e-5, 4.925e-5 },
        { 1.0, -1.9800,  0.9802   }, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    // ---- SMC (active when |error| > 0.3) ----
    ctrl::SMCParams sp;
    sp.c_e = 1.0; sp.c_de = 0.5; sp.K = 8.0; sp.phi = 0.3;
    sp.uMin = -10.0; sp.uMax = 10.0;

    // ---- LQR via adapter (active when |error| <= 0.3) ----
    Eigen::Vector2d xmax_v; xmax_v << 1.0, 1.0;
    Eigen::VectorXd umax_v(1); umax_v << 10.0;
    ctrl::LQRParams lqr_p = ctrl::LQRWeightTuner::brysonMethod(xmax_v, umax_v);
    ctrl::DiscreteLQR lqr(plant, lqr_p);

    // State provider: closure captures a pointer-to-current-state
    Eigen::VectorXd x_sim = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd x_ref_v(2); x_ref_v << 1.0, 0.0; // position=1, velocity=0

    auto stateGetter = [&]() -> Eigen::VectorXd { return x_sim; };
    auto refGetter   = [&]() -> Eigen::VectorXd { return x_ref_v; };

    // ---- Build supervisory stack ----
    ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);

    stack.addController(
        std::make_shared<ctrl::DiscreteSMC>(sp, Ts),
        "SMC",
        1.0,
        [](double error, double /*u*/) { return std::abs(error) > 0.3; });

    stack.addController(
        std::make_shared<ctrl::LQRAdapter>(lqr, stateGetter, refGetter),
        "LQR",
        1.0,
        nullptr); // always eligible (LQR is the fallback)

    // ---- Simulation ----
    double y = 0.0, ref = 1.0;

    std::cout << "  k    t[s]    y      error    u     active\n";
    std::cout << std::string(50, '-') << "\n";

    for (int k = 0; k <= 600; ++k) {
        const double e = ref - y;
        const double u = stack.compute(e);

        Eigen::VectorXd uv(1); uv << u;
        Eigen::VectorXd yv = ctrl::ssStep(plant, x_sim, uv);
        y = yv(0);

        if (k % 60 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(7)  << k * Ts
                      << std::setw(8)  << y
                      << std::setw(9)  << e
                      << std::setw(7)  << u
                      << "  " << stack.activeControllerName() << "\n";
    }
    return 0;
}
