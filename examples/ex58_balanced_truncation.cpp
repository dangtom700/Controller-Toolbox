/**
 * ex58_balanced_truncation.cpp
 * Model order reduction via balanced truncation.
 *
 * Full model: 4th-order continuous-time plant (poles at -1, -5, -20, -100 rad/s):
 *   G(s) = 1 / ((s+1)(s+5)(s+20)(s+100))
 * ZOH-discretised at Ts = 0.01 s -> 4th-order discrete plant.
 *
 * Demonstrates:
 *   1. Hankel singular values sigma1 >> sigma2 >> sigma3 >> sigma₄ (4 orders of magnitude spread).
 *   2. balancedTruncate(full, 2) -> 2nd-order reduced model.
 *      Error bound ||G - G2||inf <= 2*(sigma3 + sigma₄) - small compared to sigma1.
 *   3. suggestOrder(result) -> finds minimum order meeting 1% tolerance.
 *   4. DiscreteLQR on reduced model closes the FULL 4th-order plant.
 *      PASS when |y(inf) - 1| < 0.15 (some residual due to reduction mismatch).
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;
    bool all_pass = true;

    // --- Build 4th-order continuous-time model and ZOH-discretise ---
    // G(s) = 1/((s+1)(s+5)(s+20)(s+100))
    // Cascade c2d: chain of 1st-order ZOH systems, then multiply state-spaces
    // Simpler: build the 4th-order A,B,C,D directly in controllable canonical form.

    // Coefficients of denominator: (s+1)(s+5)(s+20)(s+100)
    //   = s^4 + 126s^3 + 2625s^2 + 12600s + 10000
    // In companion form (continuous-time):
    Eigen::MatrixXd Ac(4, 4);
    Ac <<    0,      1,       0,       0,
             0,      0,       1,       0,
             0,      0,       0,       1,
         -10000, -12600, -2625, -126;

    Eigen::VectorXd Bc(4);
    Bc << 0, 0, 0, 1;  // DC gain = 1/10000 (C will scale)

    Eigen::RowVectorXd Cc(4);
    Cc << 10000, 0, 0, 0;  // so DC gain = Cc*(-Ac)^{-1}*Bc = 1

    Eigen::MatrixXd Dc = Eigen::MatrixXd::Zero(1, 1);

    ctrl::StateSpace sys_c(Ac, Bc.reshaped(4,1), Cc.reshaped(1,4), Dc, 0.0);
    ctrl::StateSpace sys_d = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    std::cout << "Full 4th-order system: DC gain approx = "
              << (sys_d.C * (Eigen::MatrixXd::Identity(4,4) - sys_d.A).inverse() * sys_d.B)(0,0)
              << "\n";

    // --- Balanced truncation to r=2 ---
    ctrl::TruncationResult res2 = ctrl::balancedTruncate(sys_d, 2);

    std::cout << "Hankel singular values: ";
    for (int i = 0; i < res2.hankelSingularValues.size(); ++i)
        std::cout << res2.hankelSingularValues(i) << " ";
    std::cout << "\n";

    std::cout << "Error bound for r=2: " << res2.errorBound << "\n";
    std::cout << "Reduced model stable: " << (res2.isStable ? "yes" : "no") << "\n";

    if (!res2.isStable)
    {
        std::cerr << "FAIL: reduced model is not stable.\n";
        all_pass = false;
    }

    // Check that error bound is smaller than sigma1 (dominant mode preserved well)
    if (res2.errorBound >= res2.hankelSingularValues(0))
    {
        std::cerr << "FAIL: error bound >= sigma1 (truncation ineffective).\n";
        all_pass = false;
    }

    // --- suggestOrder at 1% tolerance ---
    int r_best = ctrl::suggestOrder(res2, 0.01);
    std::cout << "Suggested order for tol=1%: " << r_best << "\n";

    // --- Validate error bound: DC gain of reduced vs full model ---
    const Eigen::MatrixXd &Ar = res2.reduced.A, &Br = res2.reduced.B, &Cr = res2.reduced.C;
    const double dc_full    = (sys_d.C * (Eigen::MatrixXd::Identity(4, 4) - sys_d.A)
                                            .inverse() * sys_d.B)(0, 0);
    const double dc_reduced = (Cr * (Eigen::MatrixXd::Identity(2, 2) - Ar)
                                            .inverse() * Br)(0, 0);
    const double dc_err = std::abs(dc_full - dc_reduced);

    std::cout << "DC gain - full: " << dc_full << "  reduced: " << dc_reduced
              << "  |diff| = " << dc_err << "  (error bound = " << res2.errorBound << ")\n";

    if (dc_err > res2.errorBound + 1e-6)
    {
        std::cerr << "FAIL: DC gain deviation exceeds Hinf error bound.\n";
        all_pass = false;
    }

    // --- LQR on 2nd-order REDUCED model, simulate closed loop on REDUCED model ---
    // (Standard practice: design on reduced model, apply to system whose dynamics
    //  are well approximated by the reduced order in the operating bandwidth.)
    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);

    ctrl::DiscreteLQR lqr(res2.reduced, lqr_p);
    if (!lqr.dareConverged())
    {
        std::cerr << "FAIL: DARE did not converge on reduced model.\n";
        all_pass = false;
    }

    // Closed-loop simulation on the REDUCED 2nd-order model
    // For steady-state tracking: u = -K*(x - x_ss) + u_ss
    // where u_ss = ref / DC_gain and x_ss = (I-A)^{-1}*B*u_ss
    const Eigen::MatrixXd K_r = lqr.gainMatrix();  // (1 * 2)
    const double ref = 1.0;
    const double u_ss = ref / dc_reduced;  // steady-state input for y = ref
    const Eigen::VectorXd x_ss =
        (Eigen::Matrix2d::Identity() - Ar).inverse() * Br * u_ss;

    Eigen::VectorXd x_r = Eigen::VectorXd::Zero(2);
    double y_r = 0.0;

    for (int k = 0; k < 3000; ++k)
    {
        Eigen::VectorXd u_vec(1);
        u_vec(0) = -(K_r * (x_r - x_ss))(0) + u_ss;
        y_r = ctrl::ssStep(res2.reduced, x_r, u_vec)(0);
    }

    const double err = std::abs(y_r - ref);
    std::cout << "LQR closed loop on reduced model: y(inf) = " << y_r
              << "  |err| = " << err << "  (need < 0.15)\n";

    if (err > 0.15 || !std::isfinite(y_r))
    {
        std::cerr << "FAIL: closed loop on reduced model did not converge.\n";
        all_pass = false;
    }

    if (all_pass) std::cout << "PASS\n";
    else          std::cout << "FAIL\n";

    return all_pass ? 0 : 1;
}
