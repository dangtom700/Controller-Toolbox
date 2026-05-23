// ============================================================
//  ex07_lqg_kalman.cpp
//  LQG = LQR + Kalman filter on a 2nd-order plant with
//  process noise and noisy output measurements.
//
//  Compares:
//    (A) Full-state LQR   - ideal, no noise
//    (B) LQG output-feedback - realistic, noisy y
//
//  MATLAB equivalent:
//    sys = ss(A,B,C,D,Ts);
//    K  = dlqr(A, B, Q, R);
//    Kf = kalman(sys, Qn, Rn);
//    lqgsys = lqgreg(Kf, K);
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <random>

int main()
{
    const double Ts = 0.01;

    // ---- Plant: 2nd-order system (discretised with Euler) ----
    // Continuous: ẍ + 0.4ẋ + 4x = u  (underdamped, omegan=2, ζ=0.1)
    Eigen::Matrix2d Ac; Ac << 0.0, 1.0, -4.0, -0.4;
    Eigen::Vector2d Bc; Bc << 0.0, 1.0;
    Eigen::RowVector2d Cc; Cc << 1.0, 0.0; // only position measured
    Eigen::MatrixXd Dc(1, 1); Dc << 0.0;

    const Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity() + Ts * Ac;
    const Eigen::Vector2d Bd = Ts * Bc;
    ctrl::StateSpace plant(Ad, Bd, Cc, Dc, Ts);

    // ---- LQR weights (Bryson) ----
    Eigen::Vector2d xmax; xmax << 1.0, 2.0;  // position, velocity
    Eigen::VectorXd umax(1); umax << 5.0;
    ctrl::LQRParams lqr_p = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);

    // ---- Noise covariances ----
    Eigen::Matrix2d Qn = 0.001 * Eigen::Matrix2d::Identity(); // process noise
    Eigen::MatrixXd Rn(1, 1); Rn << 0.01;                     // measurement noise sigma=0.1

    // ---- Construct LQG ----
    ctrl::DiscreteLQG lqg(plant, lqr_p, Qn, Rn);
    ctrl::DiscreteLQR lqr_ideal(plant, lqr_p); // ideal full-state reference

    // ---- Noise generator ----
    std::mt19937 rng(42);
    std::normal_distribution<double> proc_noise(0.0, 0.03);
    std::normal_distribution<double> meas_noise(0.0, 0.10);

    // ---- Reference state ----
    Eigen::Vector2d x_ref; x_ref << 1.0, 0.0;

    std::cout << "  k    t[s]  y_true  y_noisy  x_est(pos)  u_lqg  u_ideal\n";
    std::cout << std::string(60, '-') << "\n";

    Eigen::Vector2d x_true  = Eigen::Vector2d::Zero();
    Eigen::Vector2d x_noisy = Eigen::Vector2d::Zero(); // ideal state copy
    Eigen::VectorXd u_lqg(1);  u_lqg.setZero();
    Eigen::VectorXd u_ideal(1); u_ideal.setZero();

    for (int k = 0; k <= 600; ++k) {
        // True simulation
        double y_true  = plant.C.row(0) * x_true;
        double y_noisy = y_true + meas_noise(rng);

        // LQG (uses noisy output)
        Eigen::VectorXd yv(1); yv << y_noisy;
        Eigen::VectorXd x_ref_v = x_ref;
        lqg.setReference(x_ref_v);
        lqg.setUPrev(u_lqg);
        u_lqg(0) = lqg.compute(y_noisy);
        u_lqg(0) = std::max(-5.0, std::min(5.0, u_lqg(0)));

        // Ideal LQR (uses true state)
        u_ideal  = lqr_ideal.compute(x_true, x_ref);
        u_ideal(0) = std::max(-5.0, std::min(5.0, u_ideal(0)));

        if (k % 60 == 0)
            std::cout << std::setw(4)  << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(6)  << k * Ts
                      << std::setw(8)  << y_true
                      << std::setw(9)  << y_noisy
                      << std::setw(12) << lqg.stateEstimate()(0)
                      << std::setw(7)  << u_lqg(0)
                      << std::setw(8)  << u_ideal(0) << "\n";

        // Advance true plant with LQG control + process noise
        Eigen::VectorXd noise(2); noise << proc_noise(rng), proc_noise(rng);
        x_true = Ad * x_true + Bd * u_lqg(0) + Ts * noise;
    }
    return 0;
}
