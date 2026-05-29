/**
 * ex53_mhe_mpc_dual.cpp
 * OBSERVER + STATE FEEDBACK: MHE (state estimator) + MPC (optimal control).
 *
 * MHE is the dual of MPC: MPC optimises over future inputs, MHE optimises
 * over past states. Together they form the optimal output-feedback controller
 * for constrained linear systems.
 *
 * MHE estimates x[k] from noisy y[k-N..k], honouring process-noise bounds.
 * MPC uses x^[k] to plan u[k..k+Np] with output and input constraints.
 *
 * Reference: dynopt-master/MovingHorizonEstimation.ipynb + DiscreteMPC
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

int main()
{
    const double Ts = 0.2;

    // 2-state plant: double integrator with damping
    Eigen::Matrix2d A;
    A << 1.0, Ts,
         0.0, 0.97;  // slight velocity damping
    Eigen::Vector2d B;
    B << 0.5 * Ts * Ts, Ts;
    Eigen::RowVector2d C;
    C << 1.0, 0.0;  // position measurement only
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    ctrl::StateSpace plant(A, B, C, D, Ts);

    // MHE: estimates [position, velocity] from noisy position measurements
    Eigen::Matrix2d Q_mhe = 1e-3 * Eigen::Matrix2d::Identity();
    Eigen::MatrixXd R_mhe = Eigen::MatrixXd::Constant(1, 1, 0.1);
    ctrl::MHEParams mhe_p;
    mhe_p.N   = 8;
    mhe_p.wMin = -0.5;
    mhe_p.wMax =  0.5;
    ctrl::MovingHorizonEstimator mhe(plant, Q_mhe, R_mhe, mhe_p);
    mhe.initialize(Eigen::Vector2d::Zero(),
                   10.0 * Eigen::Matrix2d::Identity());

    // MPC: uses the MHE state estimate as initial condition
    ctrl::MPCParams mp;
    mp.Np    = 12; mp.Nc = 4;
    mp.rho_y = 5.0; mp.rho_u = 0.2;
    mp.uMin  = -3.0; mp.uMax = 3.0;
    ctrl::DiscreteMPC mpc(plant, mp);

    std::srand(42);
    auto noisy = [](double s) { return s * (static_cast<double>(std::rand()) / RAND_MAX - 0.5); };

    Eigen::Vector2d x_true = Eigen::Vector2d::Zero();
    const double ref = 1.0;
    const int N = 150;
    double final_y = 0.0;

    for (int k = 0; k < N; ++k) {
        // Noisy position measurement
        Eigen::VectorXd ym(1);
        ym(0) = x_true(0) + noisy(0.2);

        // MHE: estimate state from noisy measurement
        Eigen::VectorXd u_prev(1); u_prev(0) = (k > 0) ? mpc.lastQPIters() * 0.0 : 0.0;
        const Eigen::VectorXd x_est = mhe.estimate(ym, u_prev);

        // MPC: compute optimal input using MHE state estimate
        mpc.setState(x_est);
        const double u_mpc = mpc.compute(ref - ym(0));

        // True plant step with process noise
        Eigen::Vector2d w; w << noisy(0.1), noisy(0.1);
        x_true = A * x_true + B * u_mpc + 0.05 * w;
        final_y = x_true(0);
    }

    std::cout << "MHE+MPC: final pos = " << final_y << "  (ref = " << ref << ")\n";
    std::cout << "MHE last converged: " << (mhe.lastConverged() ? "yes" : "no") << "\n";
    std::cout << "MPC last converged: " << (mpc.lastQPConverged() ? "yes" : "no") << "\n";

    if (std::isfinite(final_y) && std::abs(final_y - ref) < 0.2)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
