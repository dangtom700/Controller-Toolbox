// ============================================================
//  ex31_subspace_id.cpp
//  Subspace system identification (N4SID / MOESP).
//  Identifies a second-order discrete-time system from I/O data.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>
#include <numbers>

#if !defined(CTRL_HAS_SUBSPACE)
int main() { std::puts("Skipped: CTRL_HAS_SUBSPACE not enabled."); return 0; }
#else

int main()
{
    // True second-order plant: poles at z = 0.85 +/- 0.1j
    // State-space (controllable canonical form):
    //   A = [[1.7, -0.7250], [1, 0]], B = [[1], [0]], C = [[0.1, 0.05]]
    Eigen::Matrix2d A_true;
    A_true << 1.70, -0.725,
              1.00,  0.000;
    Eigen::Vector2d B_true; B_true << 1.0, 0.0;
    Eigen::RowVector2d C_true; C_true << 0.1, 0.05;
    Eigen::MatrixXd D_true(1, 1); D_true << 0.0;
    const double Ts = 0.01;
    ctrl::StateSpace plant(A_true, B_true, C_true, D_true, Ts);

    // Generate PRBS input excitation + collect noisy I/O data
    const int N = 1000;
    std::mt19937 rng(99);
    std::bernoulli_distribution prbs(0.5);
    std::normal_distribution<double> w_noise(0.0, 0.02);

    Eigen::MatrixXd Y_data(1, N), U_data(1, N);
    Eigen::VectorXd x(2); x << 0.0, 0.0;
    Eigen::VectorXd u_vec(1);

    for (int k = 0; k < N; ++k) {
        const double u = prbs(rng) ? 1.0 : -1.0;
        u_vec << u;
        U_data(0, k) = u;
        const Eigen::VectorXd y = ctrl::ssStep(plant, x, u_vec);
        Y_data(0, k) = y(0) + w_noise(rng); // noisy measurement
    }

    std::cout << "=== SubspaceID (N4SID / MOESP) ===\n"
              << "  True plant: 2nd-order, Ts=" << Ts << "s\n"
              << "  Data: N=" << N << " samples, PRBS input, output noise sigma=0.02\n\n";

    // Run n4sid identification
    const auto result = ctrl::n4sid(Y_data, U_data, 2, 8, Ts);

    if (!result.success) {
        std::cerr << "Identification failed: " << result.message << "\n";
        return 1;
    }

    // Print singular values (useful for order selection)
    std::cout << "Singular values of oblique projection:\n  ";
    for (int i = 0; i < std::min(6, (int)result.singularValues.size()); ++i)
        std::cout << std::fixed << std::setprecision(4) << result.singularValues(i) << "  ";
    std::cout << "\n\n";

    // Identified model
    const ctrl::StateSpace& model = result.model.value();
    std::cout << "Identified A:\n" << model.A << "\n";
    std::cout << "Identified C: " << model.C << "\n\n";

    // Eigenvalue comparison (similarity-invariant)
    Eigen::EigenSolver<Eigen::Matrix2d> eig_true(A_true);
    Eigen::EigenSolver<Eigen::Matrix2d> eig_id(model.A);

    std::cout << "True eigenvalues:       ";
    for (int i = 0; i < 2; ++i)
        std::cout << "|z| = " << std::abs(eig_true.eigenvalues()(i)) << "  ";
    std::cout << "\nIdentified eigenvalues: ";
    for (int i = 0; i < 2; ++i)
        std::cout << "|z| = " << std::abs(eig_id.eigenvalues()(i)) << "  ";

    // Suggested order heuristic
    const int order = ctrl::suggestOrder(result.singularValues, 0.01);
    std::cout << "\n\nSuggested model order: " << order << " (true order = 2)\n";

    // DC gain comparison (uncertainty is expected for subspace ID)
    const double dc_true = (C_true * (Eigen::Matrix2d::Identity() - A_true).inverse() * B_true)(0,0);
    const double dc_id   = (model.C * (Eigen::Matrix2d::Identity() - model.A).inverse() * model.B)(0,0);
    std::cout << "DC gain - true: " << std::fixed << std::setprecision(4) << dc_true
              << "  identified: " << dc_id
              << " (similarity-transform uncertainty is expected)\n";

    return 0;
}
#endif
