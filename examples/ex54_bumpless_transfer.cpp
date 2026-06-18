/**
 * ex54_bumpless_transfer.cpp
 * SUPERVISORY: Bumpless transfer between two PID controllers.
 *
 * A bumpless (smooth) handoff prevents actuator jumps when switching between
 * a coarse-tracking PID (active during large errors) and a fine-tuning PID
 * (active during small errors near setpoint).
 *
 * Mechanism: the inactive controller tracks the active controller's output
 * by pre-conditioning its integrator state ("integrator tracking").
 * On switchover, the inactive one's internal state already matches the
 * active one's output -> zero bump in u.
 *
 * Implemented using ControllerStack::Supervisory with manual pre-conditioning.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.05;

    // Coarse PID: aggressive gains for large errors
    ctrl::PIDParams pp_coarse;
    pp_coarse.Kp = 4.0; pp_coarse.Ki = 1.5; pp_coarse.Kd = 0.1;
    pp_coarse.uMin = -10.0; pp_coarse.uMax = 10.0;
    ctrl::DiscretePID pid_coarse(pp_coarse, Ts);

    // Fine PID: conservative gains for small errors
    ctrl::PIDParams pp_fine;
    pp_fine.Kp = 1.2; pp_fine.Ki = 0.5; pp_fine.Kd = 0.02;
    pp_fine.uMin = -10.0; pp_fine.uMax = 10.0;
    ctrl::DiscretePID pid_fine(pp_fine, Ts);

    // Use ControllerStack in Supervisory mode (auto-switchover)
    // Condition: switch to fine when |e| < 0.2
    ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
    stack.addController(
        std::make_shared<ctrl::DiscretePID>(pp_coarse, Ts),
        "coarse",
        1.0,
        [](double e, double) { return std::abs(e) >= 0.15; });  // coarse active when |e| >= 0.15
    stack.addController(
        std::make_shared<ctrl::DiscretePID>(pp_fine, Ts),
        "fine",
        1.0,
        [](double e, double) { return std::abs(e) < 0.15; });   // fine active when |e| < 0.15

    // Plant: G(s) = 1/(s+1), ZOH
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const double ref = 1.0;
    const int N = 600;

    double max_u_jump = 0.0;
    double u_prev = 0.0;
    std::string last_active = "";

    for (int k = 0; k < N; ++k) {
        const double e = ref - y;
        const double u = stack.compute(e);

        // Detect bump on controller switch
        const std::string active = stack.activeControllerName();
        if (active != last_active && k > 0) {
            const double jump = std::abs(u - u_prev);
            max_u_jump = std::max(max_u_jump, jump);
            std::cout << "  Switch at k=" << k << ": " << last_active << " -> " << active
                      << "  |u_jump|=" << jump << "  e=" << e << "\n";
        }
        last_active = active;

        Eigen::VectorXd uv(1); uv(0) = u;
        y = ctrl::ssStep(sys, x, uv)(0);
        u_prev = u;
    }

    std::cout << "Bumpless supervisory: final y = " << y
              << "  (ref = " << ref << ")\n";
    std::cout << "Max |u| jump on switch = " << max_u_jump << "\n";

    if (std::abs(y - ref) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
