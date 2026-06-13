/**
 * @file ex80_grey_box_estimator.cpp
 * @brief E1/E2/E3: GreyBoxEstimator, RecursiveGreyBoxEstimator, GPResidualModel.
 *
 * Demonstrates three Phase-2 model estimation algorithms on a second-order
 * spring-mass-damper system:
 *
 *   Plant:  x1dot = x2
 *           x2dot = -(k/m)*x1 - (c/m)*x2 + (1/m)*u
 *   y     = x1
 *
 *   True parameters:  m=1 kg, k=4 N/m, c=0.8 N.s/m  => [k/m=4, c/m=0.8, 1/m=1]
 *
 * E1 - GreyBoxEstimator:
 *   Batch LM fit from 80-step offline trajectory.  Starts from [3, 0.5, 0.8].
 *
 * E2 - RecursiveGreyBoxEstimator:
 *   Online augmented-UKF tracking while the system runs in real time.
 *   Tracks a single unknown parameter (k/m) starting from 3.0 toward 4.0.
 *
 * E3 - GPResidualModel:
 *   After E1, the nominal model has small prediction errors.  GPResidualModel
 *   learns the residuals and demonstrates reduced prediction error.
 *
 * Expected output:
 *   [E1] params converged: k/m ~ 4.0  c/m ~ 0.8  1/m ~ 1.0
 *   [E2] param estimate after 100 steps finite and approaching 4.0
 *   [E3] GP correction reduces max error vs. nominal model
 *   [PASS] All checks passed.
 */

#include <ControllerToolbox.h>
#include <cmath>
#include <iomanip>
#include <iostream>

// ---------------------------------------------------------------------------
// True plant dynamics: 2-state SMD
// ---------------------------------------------------------------------------
static Eigen::VectorXd smd_true(const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& u)
{
    const double km = 4.0, cm = 0.8, inv_m = 1.0;
    Eigen::VectorXd xd(2);
    xd(0) = x(1);
    xd(1) = -km*x(0) - cm*x(1) + inv_m*u(0);
    return xd;
}

// RK4 helper for the true plant
static Eigen::VectorXd rk4_true(const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& u, double Ts)
{
    auto k1 = smd_true(x, u);
    auto k2 = smd_true(x + 0.5*Ts*k1, u);
    auto k3 = smd_true(x + 0.5*Ts*k2, u);
    auto k4 = smd_true(x + Ts*k3, u);
    return x + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);
}

int main()
{
    const double Ts = 0.02;
    const int    N  = 80;

    // -----------------------------------------------------------------------
    // Generate offline data with a step input + sinusoid
    // -----------------------------------------------------------------------
    Eigen::MatrixXd U(1, N), Y(1, N);
    Eigen::VectorXd xs = Eigen::VectorXd::Zero(2);
    for (int k = 0; k < N; ++k) {
        double t  = k * Ts;
        U(0, k)   = (t < 0.5) ? 1.0 : 0.5 * std::sin(4.0*t);
        Y(0, k)   = xs(0);
        xs        = rk4_true(xs, U.col(k), Ts);
    }

    // -----------------------------------------------------------------------
    // E1 - GreyBoxEstimator: batch LM fit
    // -----------------------------------------------------------------------
    std::cout << "\n=== E1  GreyBoxEstimator ===\n";

    ctrl::GreyBoxEstimator::OdeFn f_ode =
        [](const Eigen::VectorXd& x, const Eigen::VectorXd& u,
           const Eigen::VectorXd& p) {
            Eigen::VectorXd xd(2);
            xd(0) = x(1);
            xd(1) = -p(0)*x(0) - p(1)*x(1) + p(2)*u(0);
            return xd;
        };

    ctrl::GreyBoxEstimator::MeasFn h_meas =
        [](const Eigen::VectorXd& x, const Eigen::VectorXd&) {
            return Eigen::VectorXd(x.head(1));
        };

    ctrl::GreyBoxEstimator::Params gbe_par;
    gbe_par.p0    = (Eigen::VectorXd(3) << 3.0, 0.5, 0.8).finished();
    gbe_par.lower = (Eigen::VectorXd(3) << 0.1, 0.01, 0.1).finished();
    gbe_par.upper = (Eigen::VectorXd(3) << 20.0, 5.0, 10.0).finished();
    gbe_par.Ts        = Ts;
    gbe_par.rk4_steps = 2;
    gbe_par.max_iter  = 80;

    ctrl::GreyBoxEstimator gbe(f_ode, h_meas, gbe_par);
    auto res = gbe.fit(Eigen::VectorXd::Zero(2), U, Y);

    auto& p_hat = gbe.params();
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  converged:  " << (res.converged ? "yes" : "no") << "\n";
    std::cout << "  iterations: " << res.iterations << "   cost: " << res.cost << "\n";
    std::cout << "  k/m  = " << p_hat(0) << "  (true 4.0)\n";
    std::cout << "  c/m  = " << p_hat(1) << "  (true 0.8)\n";
    std::cout << "  1/m  = " << p_hat(2) << "  (true 1.0)\n";

    bool e1_ok = gbe.isFitted() &&
                 std::abs(p_hat(0) - 4.0) < 0.5 &&
                 std::abs(p_hat(1) - 0.8) < 0.3 &&
                 std::abs(p_hat(2) - 1.0) < 0.3;
    std::cout << "  [E1] " << (e1_ok ? "PASS" : "FAIL") << "\n";

    // -----------------------------------------------------------------------
    // E2 - RecursiveGreyBoxEstimator: online UKF tracking (1 unknown: k/m)
    // -----------------------------------------------------------------------
    std::cout << "\n=== E2  RecursiveGreyBoxEstimator ===\n";

    ctrl::RecursiveGreyBoxEstimator::OdeFn rf =
        [](const Eigen::VectorXd& x, const Eigen::VectorXd& u,
           const Eigen::VectorXd& p) {
            Eigen::VectorXd xd(2);
            xd(0) = x(1);
            xd(1) = -p(0)*x(0) - 0.8*x(1) + u(0);
            return xd;
        };

    ctrl::RecursiveGreyBoxEstimator::MeasFn rh =
        [](const Eigen::VectorXd& x, const Eigen::VectorXd&) {
            return Eigen::VectorXd(x.head(1));
        };

    ctrl::RecursiveGreyBoxEstimator::Params rge_par;
    rge_par.p0      = Eigen::VectorXd::Constant(1, 3.0);
    rge_par.Q_state = Eigen::MatrixXd::Identity(2, 2) * 1e-4;
    rge_par.Q_param = Eigen::MatrixXd::Identity(1, 1) * 1e-4;
    rge_par.R_meas  = Eigen::MatrixXd::Identity(1, 1) * 1e-3;
    rge_par.Ts      = Ts;
    rge_par.alpha   = 0.3;
    rge_par.beta    = 2.0;
    rge_par.kappa   = 0.0;

    ctrl::RecursiveGreyBoxEstimator rge(rf, rh, 2, 1, rge_par);
    rge.initialize(Eigen::VectorXd::Zero(2), Eigen::MatrixXd::Identity(2, 2) * 0.1);

    xs = Eigen::VectorXd::Zero(2);
    for (int k = 0; k < 100; ++k) {
        Eigen::VectorXd u_k(1);
        u_k(0) = (k < 50) ? 1.0 : -0.5;
        xs = rk4_true(xs, u_k, Ts);
        Eigen::VectorXd y_k(1);
        y_k(0) = xs(0) + 0.005 * (k % 5 - 2);  // small measurement noise
        rge.step(y_k, u_k);
    }

    double p_rge = rge.paramEstimate()(0);
    std::cout << "  k/m estimate = " << p_rge
              << "  (started 3.0, true 4.0)\n";
    std::cout << "  state finite: " << (rge.stateEstimate().allFinite() ? "yes" : "no") << "\n";

    bool e2_ok = rge.isInitialized() &&
                 rge.stateEstimate().allFinite() &&
                 rge.paramEstimate().allFinite() &&
                 p_rge > 3.0;   // converging toward 4.0
    std::cout << "  [E2] " << (e2_ok ? "PASS" : "FAIL") << "\n";

    // -----------------------------------------------------------------------
    // E3 - GPResidualModel: learn residuals from E1 nominal model
    // -----------------------------------------------------------------------
    std::cout << "\n=== E3  GPResidualModel ===\n";

    ctrl::GPResidualModel::Params grm_par;
    grm_par.gp.length_scale = 0.2;
    grm_par.gp.signal_var   = 0.01;
    grm_par.gp.noise_var    = 1e-4;
    grm_par.gp.n_max        = N;

    ctrl::GPResidualModel grm(1, grm_par);

    // Predict with nominal (E1) model and record residuals
    auto Y_nom = gbe.predict(Eigen::VectorXd::Zero(2), U);

    Eigen::VectorXd x_feat_prev(1);
    x_feat_prev(0) = Y(0, 0);
    for (int k = 1; k < N; ++k) {
        Eigen::VectorXd xf(1); xf(0) = Y(0, k-1);
        grm.addResidualPoint(xf, Y(0, k), Y_nom(0, k));
    }
    grm.fit();

    // Compare max error: nominal vs. GP-corrected on a test window
    double max_err_nom = 0.0, max_err_gp = 0.0;
    for (int k = 1; k < N; ++k) {
        double err_nom = std::abs(Y(0, k) - Y_nom(0, k));
        Eigen::VectorXd xf(1); xf(0) = Y(0, k-1);
        auto pred = grm.predictWithUncertainty(xf, Y_nom(0, k));
        double err_gp = std::abs(Y(0, k) - pred.mean_total);
        max_err_nom = std::max(max_err_nom, err_nom);
        max_err_gp  = std::max(max_err_gp, err_gp);
    }

    std::cout << "  max error nominal:      " << max_err_nom << "\n";
    std::cout << "  max error GP-corrected: " << max_err_gp  << "\n";
    std::cout << "  fitted: " << (grm.isFitted() ? "yes" : "no")
              << "  size: " << grm.size() << "\n";

    bool e3_ok = grm.isFitted() && grm.size() == N - 1 &&
                 max_err_gp <= max_err_nom + 1e-6;
    std::cout << "  [E3] " << (e3_ok ? "PASS" : "FAIL") << "\n";

    // -----------------------------------------------------------------------
    // Final verdict
    // -----------------------------------------------------------------------
    if (e1_ok && e2_ok && e3_ok) {
        std::cout << "\n[PASS] All checks passed.\n";
        return 0;
    } else {
        std::cout << "\n[FAIL] One or more checks failed.\n";
        return 1;
    }
}
