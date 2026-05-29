/**
 * ex37_smith_predictor_fractional.cpp
 * Smith predictor with fractional dead time (padeDelayFilter wired).
 *
 * Plants with non-integer dead-time multiples of Ts use the Pade approximation
 * for the sub-sample remainder. Verifies that the controller reaches the
 * reference after sufficient steps.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    // Plant: G(s) = exp(-1.7s) / (s+1), theta=1.7 s, Ts=0.5 s
    // Integer delay: d=3 steps (1.5s), fractional: 0.2s remainder
    const double Ts    = 0.5;
    const double theta = 1.7;

    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace G0 = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    // Inner PI controller
    ctrl::PIDParams pp;
    pp.Kp   = 1.5;
    pp.Ki   = 0.4;
    pp.Kd   = 0.0;
    pp.uMin = -10.0;
    pp.uMax =  10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // Smith predictor with fractional dead-time ctor (theta, Ts)
    ctrl::SmithPredictor sp(pid, G0, theta, Ts);

    // Simulate: include the actual integer delay in the plant
    const int d_int = static_cast<int>(theta / Ts);  // = 3
    std::vector<double> u_buf(d_int + 2, 0.0);

    Eigen::VectorXd x_plant = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const double ref = 1.0;
    const int N = 300;
    double final_y = 0.0;

    for (int k = 0; k < N; ++k) {
        const double u_sp = sp.compute(ref - y);

        // Apply delayed input to plant
        const double u_delayed = u_buf[d_int];
        for (int i = d_int; i > 0; --i) u_buf[i] = u_buf[i-1];
        u_buf[0] = u_sp;

        Eigen::VectorXd uv(1); uv(0) = u_delayed;
        y = ctrl::ssStep(G0, x_plant, uv)(0);
        final_y = y;
    }

    std::cout << "Smith predictor (fractional theta=" << theta
              << " s) final y = " << final_y << "  (ref = " << ref << ")\n";

    if (std::abs(final_y - ref) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
