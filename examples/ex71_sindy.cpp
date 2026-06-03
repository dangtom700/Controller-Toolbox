/**
 * @file ex71_sindy.cpp
 * @brief SINDy - Sparse Identification of Nonlinear Dynamics.
 *
 * Demonstrates:
 *   1. Collecting snapshot data from a known nonlinear system (Van der Pol oscillator).
 *   2. Fitting a sparse SINDy model using degree-2 polynomial library and STLS.
 *   3. Comparing the SINDy model prediction against the true dynamics.
 *   4. Using the identified StateFunc inside an ExtendedKalmanFilter.
 *
 * True system (continuous):
 *   dx0/dt = x1
 *   dx1/dt = mu*(1-x0^2)*x1 - x0 + u   (forced Van der Pol, mu=0.5)
 */

#include "ControllerToolbox.h"
#include "SINDy.h"
#include <cmath>
#include <cstdio>

// Euler integration of Van der Pol
static Eigen::Vector2d vanDerPol(const Eigen::Vector2d& x, double u, double dt,
                                  double mu = 0.5)
{
    Eigen::Vector2d xdot;
    xdot(0) = x(1);
    xdot(1) = mu * (1.0 - x(0)*x(0)) * x(1) - x(0) + u;
    return x + dt * xdot;
}

static Eigen::Vector2d vanDerPolDot(const Eigen::Vector2d& x, double u, double mu = 0.5)
{
    Eigen::Vector2d d;
    d(0) = x(1);
    d(1) = mu * (1.0 - x(0)*x(0)) * x(1) - x(0) + u;
    return d;
}

int main()
{
    std::printf("=== ex71: SINDy on Van der Pol oscillator ===\n\n");

    // ------------------------------------------------------------------
    // 1. Collect training snapshots from simulation
    // ------------------------------------------------------------------
    ctrl::SINDy::Params sp;
    sp.n_state    = 2;
    sp.n_input    = 1;
    sp.library    = ctrl::SINDyLibrary::PolyDeg2;
    sp.threshold  = 0.05;
    sp.stls_iter  = 10;
    ctrl::SINDy sindy(sp);

    constexpr double dt = 0.01;
    constexpr int    N  = 2000;

    Eigen::Vector2d x;
    x << 1.0, 0.5;

    for (int k = 0; k < N; ++k) {
        // Pseudo-random forcing input
        double u = 0.5 * std::sin(0.3 * k * dt) + 0.2 * ((k % 7 == 0) ? 1.0 : -0.5);

        Eigen::VectorXd x_dev(2), u_dev(1);
        x_dev = x;
        u_dev(0) = u;

        // True derivative (continuous, known for this demo)
        Eigen::VectorXd xdot = vanDerPolDot(x, u);

        sindy.addSnapshot(x_dev, u_dev, xdot);
        x = vanDerPol(x, u, dt);
    }

    std::printf("Training data: %d snapshots, %d library terms\n",
                sindy.snapshotCount(), sindy.nTerms());

    // ------------------------------------------------------------------
    // 2. Fit sparse model
    // ------------------------------------------------------------------
    ctrl::SINDyModel model = sindy.fit();

    std::printf("Fitted model: %d terms, sparsity = %.2f%%\n",
                model.nTerms(),
                model.sparsity() * 100.0);
    std::printf("Non-zero coefficient fraction: %.2f%%\n",
                (1.0 - model.sparsity()) * 100.0);

    // ------------------------------------------------------------------
    // 3. Validation: compare prediction vs truth on test trajectory
    // ------------------------------------------------------------------
    std::printf("\n  Step | True xdot0 | SINDy xdot0 | True xdot1 | SINDy xdot1\n");
    std::printf("  -----|------------|-------------|------------|-----------\n");

    x << 0.5, -0.3;
    double err_rms = 0.0;
    int n_val = 20;
    for (int k = 0; k < n_val; ++k) {
        double u = 0.3 * std::sin(0.1 * k);
        Eigen::VectorXd x_v(2); x_v = x;
        Eigen::VectorXd u_v(1); u_v(0) = u;

        Eigen::VectorXd true_dot  = vanDerPolDot(x, u);
        Eigen::VectorXd sindy_dot = model.predict(x_v, u_v);

        err_rms += (true_dot - sindy_dot).squaredNorm();

        if (k < 6)
            std::printf("  %4d | %10.4f | %11.4f | %10.4f | %10.4f\n", k,
                        true_dot(0), sindy_dot(0), true_dot(1), sindy_dot(1));

        x = vanDerPol(x, u, dt);
    }
    err_rms = std::sqrt(err_rms / (n_val * 2));
    std::printf("  Validation RMS error: %.4f\n\n", err_rms);

    // ------------------------------------------------------------------
    // 4. Use SINDy StateFunc in an EKF
    // ------------------------------------------------------------------
    auto sf = model.stateFunc();

    // Jacobian via numerical differentiation (provided by LinearisationHelper)
    auto Fjac = [&sf](const Eigen::VectorXd& xv, const Eigen::VectorXd& uv) -> Eigen::MatrixXd {
        return ctrl::jacobianX(sf, xv, uv);
    };
    auto Hjac = [](const Eigen::VectorXd&, const Eigen::VectorXd&) -> Eigen::MatrixXd {
        Eigen::MatrixXd H(1, 2);
        H << 1.0, 0.0;   // observe position only
        return H;
    };
    auto h_fn = [](const Eigen::VectorXd& xv, const Eigen::VectorXd&) -> Eigen::VectorXd {
        return xv.head(1);
    };

    Eigen::Matrix2d Q_n = 1e-4 * Eigen::Matrix2d::Identity();
    Eigen::Matrix<double,1,1> R_n; R_n << 1e-3;

    ctrl::ExtendedKalmanFilter ekf(2, 1, sf, h_fn, Fjac, Hjac, Q_n, R_n, dt);

    // Run EKF on test trajectory
    Eigen::Vector2d x_true; x_true << 1.0, 0.0;
    Eigen::VectorXd x0(2); x0 << 0.9, 0.1;
    ekf.setState(x0);

    double pos_err = 0.0;
    for (int k = 0; k < 50; ++k) {
        double u = 0.2 * std::sin(0.05 * k);
        x_true = vanDerPol(x_true, u, dt);

        Eigen::Matrix<double,1,1> y_meas; y_meas << x_true(0) + 0.01 * ((k % 3) - 1);
        Eigen::Matrix<double,1,1> u_vec; u_vec << u;
        ekf.step(y_meas, u_vec);

        pos_err += std::abs(ekf.state()(0) - x_true(0));
    }
    pos_err /= 50;
    std::printf("  EKF with SINDy model - mean position error: %.4f\n", pos_err);
    std::printf("  (< 0.10 expected for a well-identified model)\n");

    return 0;
}
