/**
 * ex48_fuzzy_chattering_smc.cpp
 * ADDITIVE: FuzzyPD correction to reduce SMC chattering.
 *
 * SMC primary controller provides robust sliding-mode tracking.
 * FuzzyPD adds a smooth correction signal that reduces chattering near the
 * sliding surface, improving actuator life without sacrificing robustness.
 *
 * u_total = u_smc + alpha * u_fuzzy_correction
 *
 * The fuzzy corrector is tuned to be active near s=0 (sliding surface),
 * providing a smooth boundary-layer correction rather than the hard saturation.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;

    // SMC (primary)
    ctrl::SMCParams smc_p;
    smc_p.c_e  = 1.0;           // error weight in surface
    smc_p.c_de = 5.0 * Ts;      // c_de = lambda * Ts (lambda=5 [1/s])
    smc_p.phi  = 0.1;            // boundary layer
    smc_p.uMin = -10.0;
    smc_p.uMax =  10.0;
    ctrl::DiscreteSMC smc_ctrl(smc_p, Ts);

    // FuzzyPD correction (reduce chattering near sliding surface)
    ctrl::FuzzyPDParams fpdp;
    fpdp.e_scale  = 0.5;   // sensitive to small errors (near sliding surface)
    fpdp.de_scale = 0.05;
    fpdp.u_scale  = 0.8;   // small correction magnitude
    ctrl::FuzzyPD fuzzy_corr(fpdp, Ts);

    // Plant: G(s) = 1/(s+1), ZOH
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0, prev_e = 0.0;
    const double ref = 1.0;
    const int N = 600;
    double total_u_sq = 0.0;

    for (int k = 0; k < N; ++k) {
        const double e  = ref - y;  // SMC uses compute(y-ref) sign convention
        const double de = (e - prev_e) / Ts;

        // SMC: primary control (compute(y-ref))
        const double u_smc  = smc_ctrl.compute(y - ref);
        // FuzzyPD: smooth correction on error and derivative
        const double u_fuzzy = fuzzy_corr.compute(e);

        // Combined: SMC dominates, fuzzy smooths near s=0
        const double u_total = u_smc + 0.3 * u_fuzzy;

        Eigen::VectorXd uv(1); uv(0) = u_total;
        y = ctrl::ssStep(sys, x, uv)(0);

        total_u_sq += u_total * u_total * Ts;
        prev_e = e;
    }

    std::cout << "ADDITIVE FuzzyPD+SMC: final y = " << y
              << "  (ref = " << ref << ")\n";
    std::cout << "Control energy: " << total_u_sq << "\n";

    if (std::abs(y - ref) < 0.05 && std::isfinite(total_u_sq))
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
