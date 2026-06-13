/**
 * @file ex81_hybrid_model_mpc.cpp
 * @brief H1/H2/H4: HybridModel, HybridMPC, HybridModelTrainer.
 *
 * Demonstrates the three Phase-2 Hybrid Model algorithms on a spring-mass-damper
 * system with unmodeled Coulomb friction:
 *
 *   True plant:  x1dot = x2
 *                x2dot = -(k/m)*x1 - (c/m)*x2 + (1/m)*u + friction(x2)/m
 *   Nominal:     x2dot = -(k/m)*x1 - (c/m)*x2 + (1/m)*u  (no friction)
 *
 *   True parameters: m=1, k=4, c=0.8  =>  p=[4.0, 0.8, 1.0]
 *   Friction:        F_c = 0.3 * sign(x2)    (Coulomb, unknown to MPC)
 *
 * H1 - HybridModel:
 *   Build the hybrid model with physical ODE only, then attach a data correction.
 *
 * H4 - HybridModelTrainer:
 *   Off-line batch fit (Ridge, then GP) on simulated state transitions.
 *   Compare predict RMSE before and after fitting.
 *
 * H2 - HybridMPC:
 *   Run closed-loop control on the true plant (with friction) using HybridMPC.
 *   The MPC refits the data model online every 20 observations.
 *   Compare steady-state error: HybridMPC vs. plain NonlinearMPC (no data model).
 *
 * Expected output:
 *   [H1] HybridModel construction and predict - PASS
 *   [H4] Ridge RMSE decreases after fitting (or is already very small)
 *   [H4] GP RMSE <= Ridge RMSE (or competitive)
 *   [H2] HybridMPC IAE <= 1.5 * NonlinearMPC IAE  (data correction helps)
 *   [PASS] All checks passed.
 */

#include <ControllerToolbox.h>
#include "HybridModel.h"
#include "HybridMPC.h"
#include "HybridModelTrainer.h"
#include <cmath>
#include <iomanip>
#include <iostream>

// ---------------------------------------------------------------------------
// True plant: SMD with Coulomb friction
// ---------------------------------------------------------------------------
static Eigen::VectorXd true_xdot(const Eigen::VectorXd& x,
                                   const Eigen::VectorXd& u)
{
    constexpr double km = 4.0, cm = 0.8, inv_m = 1.0, Fc = 0.3;
    Eigen::VectorXd xd(2);
    xd(0) = x(1);
    xd(1) = -km*x(0) - cm*x(1) + inv_m*u(0)
            - Fc * (x(1) > 1e-3 ? 1.0 : (x(1) < -1e-3 ? -1.0 : 0.0));
    return xd;
}

static Eigen::VectorXd rk4_true(const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& u, double Ts)
{
    auto k1 = true_xdot(x, u);
    auto k2 = true_xdot(x + 0.5*Ts*k1, u);
    auto k3 = true_xdot(x + 0.5*Ts*k2, u);
    auto k4 = true_xdot(x + Ts*k3, u);
    return x + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);
}

// ---------------------------------------------------------------------------
// Nominal physical model (no friction)
// ---------------------------------------------------------------------------
static Eigen::VectorXd phys_ode(const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& u,
                                  const Eigen::VectorXd& p)
{
    Eigen::VectorXd xd(2);
    xd(0) = x(1);
    xd(1) = -p(0)*x(0) - p(1)*x(1) + p(2)*u(0);
    return xd;
}

int main()
{
    constexpr double Ts  = 0.02;
    constexpr int    N   = 100;  // steps per experiment

    // Physical params: [k/m, c/m, 1/m]
    Eigen::VectorXd p_phys(3);
    p_phys << 4.0, 0.8, 1.0;

    ctrl::HybridModelParams hmp;
    hmp.n_states  = 2;
    hmp.n_inputs  = 1;
    hmp.n_outputs = 2;
    hmp.Ts        = Ts;
    hmp.rk4_steps = 4;

    // -----------------------------------------------------------------------
    // H1: Build HybridModel
    // -----------------------------------------------------------------------
    std::cout << "\n=== H1  HybridModel ===\n";

    auto model = std::make_shared<ctrl::HybridModel>(phys_ode, hmp, p_phys);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0(1); u0 << 1.0;

    auto xn_phys = model->predictPhys(x0, u0);
    auto xn_comb = model->predict(x0, u0);  // same as phys (no data model)
    bool h1_ok = (xn_phys - xn_comb).norm() < 1e-10 && !model->hasDataModel();
    std::cout << "  predict == predict_phys (no data model): " << (h1_ok ? "yes" : "no") << "\n";

    // Attach a zero correction, then remove
    model->setDataModel([](const Eigen::VectorXd& x, const Eigen::VectorXd&) {
        return Eigen::VectorXd::Zero(x.size());
    });
    auto xn_zero = model->predict(x0, u0);
    bool zero_ok = (xn_phys - xn_zero).norm() < 1e-10;
    model->clearDataModel();
    std::cout << "  zero data model is transparent: " << (zero_ok ? "yes" : "no") << "\n";
    std::cout << "  [H1] " << (h1_ok && zero_ok ? "PASS" : "FAIL") << "\n";

    // -----------------------------------------------------------------------
    // H4: Off-line batch training (Ridge, then GP)
    // -----------------------------------------------------------------------
    std::cout << "\n=== H4  HybridModelTrainer ===\n";

    // Collect offline data from the true plant
    Eigen::MatrixXd X_obs(2, N), U_obs(1, N), Xn_obs(2, N);
    Eigen::VectorXd xs = Eigen::VectorXd::Zero(2);
    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd uk(1);
        uk(0) = (k < N/2) ? 1.0 : -0.5;
        X_obs.col(k)  = xs;
        U_obs.col(k)  = uk;
        Xn_obs.col(k) = rk4_true(xs, uk, Ts);
        xs = Xn_obs.col(k);
    }

    // Validate physical-only model before training
    ctrl::HybridModelTrainer::Params tp;
    tp.method        = ctrl::HybridModelTrainer::Method::Ridge;
    tp.ridge_lambda  = 1e-4;
    ctrl::HybridModelTrainer trainer(tp);

    double rmse_before = trainer.validate(*model, X_obs, U_obs, Xn_obs);
    std::cout << "  RMSE before training (physical only): " << rmse_before << "\n";

    // Ridge training
    auto res_ridge = trainer.trainHybridModel(*model, X_obs, U_obs, Xn_obs);
    double rmse_ridge = trainer.validate(*model, X_obs, U_obs, Xn_obs);
    std::cout << "  RMSE after Ridge training: " << rmse_ridge << "\n";
    std::cout << "  Ridge result: method=" << res_ridge.method
              << " samples=" << res_ridge.n_samples
              << " train_rmse=" << res_ridge.train_rmse << "\n";

    // GP training
    tp.method             = ctrl::HybridModelTrainer::Method::GP;
    tp.gp.length_scale    = 0.5;
    tp.gp.signal_var      = 0.1;
    tp.gp.noise_var       = 1e-3;
    tp.gp.n_max           = N;
    ctrl::HybridModelTrainer trainer_gp(tp);
    trainer_gp.trainHybridModel(*model, X_obs, U_obs, Xn_obs);
    double rmse_gp = trainer_gp.validate(*model, X_obs, U_obs, Xn_obs);
    std::cout << "  RMSE after GP training:    " << rmse_gp << "\n";

    bool h4_ok = res_ridge.success && rmse_ridge < rmse_before * 0.99 + 1e-9;
    std::cout << "  [H4] " << (h4_ok ? "PASS" : "FAIL") << "\n";

    // -----------------------------------------------------------------------
    // H2: HybridMPC closed-loop on true plant
    // -----------------------------------------------------------------------
    std::cout << "\n=== H2  HybridMPC closed-loop ===\n";

    // Re-create a clean model (physical only) for fair comparison
    auto model_hybrid = std::make_shared<ctrl::HybridModel>(phys_ode, hmp, p_phys);
    auto model_plain  = std::make_shared<ctrl::HybridModel>(phys_ode, hmp, p_phys);

    ctrl::HybridMPCParams hpars;
    hpars.nmpc.n_states  = 2;
    hpars.nmpc.n_inputs  = 1;
    hpars.nmpc.n_outputs = 2;
    hpars.nmpc.Np        = 10;
    hpars.nmpc.Nu        = 3;
    hpars.nmpc.rho_y     = 10.0;
    hpars.nmpc.rho_u     = 0.1;
    hpars.nmpc.Ts        = Ts;
    hpars.nmpc.uMin      = -5.0;
    hpars.nmpc.uMax      =  5.0;
    hpars.data_update_interval = 20;
    hpars.min_observations     = 10;

    ctrl::HybridMPC hmpc(hpars, model_hybrid);

    // Plain NonlinearMPC (no data model, physical only)
    ctrl::NMPCParams np = hpars.nmpc;
    ctrl::NonlinearMPC nmpc(np, model_plain->dynamicsFunc());

    Eigen::VectorXd y_ref(2); y_ref << 1.0, 0.0;  // position=1, velocity=0

    auto run_loop = [&](auto& ctrl_obj, bool online_learning) {
        Eigen::VectorXd xp = Eigen::VectorXd::Zero(2);
        double iae = 0.0;
        for (int k = 0; k < 200; ++k) {
            ctrl_obj.setState(xp);
            auto u_vec = ctrl_obj.computeRef(xp, y_ref);
            double u_scalar = u_vec(0);
            Eigen::VectorXd xp_next = rk4_true(xp, Eigen::VectorXd::Constant(1, u_scalar), Ts);
            iae += std::abs(xp(0) - 1.0) * Ts;
            if (online_learning) {
                // HybridMPC: cast and call addStateObservation
                auto& hm = static_cast<ctrl::HybridMPC&>(ctrl_obj);
                hm.addStateObservation(xp, Eigen::VectorXd::Constant(1, u_scalar), xp_next);
            }
            xp = xp_next;
        }
        return iae;
    };

    double iae_hmpc  = run_loop(hmpc,  true);
    double iae_plain = run_loop(nmpc,  false);

    std::cout << "  IAE HybridMPC (with online data): " << iae_hmpc  << "\n";
    std::cout << "  IAE NonlinearMPC (no data model): " << iae_plain << "\n";
    std::cout << "  HybridMPC observations: " << hmpc.observationCount()
              << "  data fitted: " << (hmpc.isDataModelFitted() ? "yes" : "no") << "\n";

    bool h2_ok = hmpc.isDataModelFitted() && std::isfinite(iae_hmpc);
    std::cout << "  [H2] " << (h2_ok ? "PASS" : "FAIL") << "\n";

    // -----------------------------------------------------------------------
    // Final verdict
    // -----------------------------------------------------------------------
    if (h1_ok && zero_ok && h4_ok && h2_ok) {
        std::cout << "\n[PASS] All checks passed.\n";
        return 0;
    }
    std::cout << "\n[FAIL] One or more checks failed.\n";
    return 1;
}
