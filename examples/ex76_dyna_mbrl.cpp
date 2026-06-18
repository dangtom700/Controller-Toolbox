/**
 * @file ex76_dyna_mbrl.cpp
 * @brief DynaController: model-based RL wrapping a PID on a FOPDT plant.
 *
 * Demonstrates the three Dyna phases in C++:
 *   1. React  - PID produces the live control action.
 *   2. Learn  - DynaController fits a SINDy model on error transitions.
 *   3. Plan   - modelRollout() evaluates a constant-u plan using the model.
 *
 * Plant (FOPDT, ZOH-discrete at Ts=0.05s):
 *   G(s) = 0.8 / (1.5s + 1)  =>  e[k+1] approx = alpha*e[k] + beta*u[k]
 *
 * Expected output:
 *   Model fitted after 50 transitions.
 *   Rollout RMSE vs real trajectory < 0.10 for a constant-u plan.
 *   [PASS] All checks passed.
 */

#include <ControllerToolbox.h>
#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
    constexpr double Ts  = 0.05;
    constexpr int    N   = 200;

    // -- Plant: FOPDT G(s) = 0.8/(1.5s+1), ZOH-discrete ----------------------
    // Exact ZOH: alpha = exp(-Ts/1.5), beta = 0.8*(1-alpha)
    const double alpha = std::exp(-Ts / 1.5);
    const double beta  = 0.8 * (1.0 - alpha);

    // -- Controller: PID wrapped by DynaController ---------------------------
    ctrl::PIDParams pid_p;
    pid_p.Kp = 1.2; pid_p.Ki = 0.4; pid_p.Kd = 0.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pid_p, Ts);

    ctrl::DynaController::Params dp;
    dp.Ts           = Ts;
    dp.n_collect    = 50;
    dp.n_refit_every = 30;
    dp.sindy.library   = ctrl::SINDyLibrary::PolyDeg2;
    dp.sindy.threshold = 0.005;
    ctrl::DynaController dyna(dp, pid);

    // -- Closed-loop simulation -----------------------------------------------
    const double ref = 1.0;
    double y = 0.0;          // plant output
    double u_prev = 0.0;     // last action (for FOPDT simulation)

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Step |  error  |   u     | fitted | buffer\n";
    std::cout << "-----+---------+---------+--------+-------\n";

    for (int k = 0; k < N; ++k) {
        double e = ref - y;
        double u = dyna.compute(e);

        // Advance plant: y[k+1] = alpha*y[k] + beta*u[k]
        y = alpha * y + beta * u;
        u_prev = u;

        if (k % 40 == 0 || k == N - 1) {
            std::cout << std::setw(4) << k
                      << " | " << std::setw(7) << e
                      << " | " << std::setw(7) << u
                      << " | " << (dyna.modelFitted() ? "yes" : " no")
                      << "    | " << dyna.bufferSize() << '\n';
        }
    }

    // -- Verify model was fitted ----------------------------------------------
    bool ok = true;
    if (!dyna.modelFitted()) {
        std::cerr << "[FAIL] Model was not fitted after " << N << " steps\n";
        ok = false;
    }

    // -- Rollout accuracy check -----------------------------------------------
    if (dyna.modelFitted()) {
        constexpr int Np = 20;
        const double e0 = ref - y;
        Eigen::VectorXd u_plan = Eigen::VectorXd::Constant(Np, u_prev);

        Eigen::VectorXd e_pred = dyna.modelRollout(e0, u_plan);

        // Simulate real response for comparison
        double y_sim = y;
        double rmse = 0.0;
        for (int k = 0; k < Np; ++k) {
            y_sim = alpha * y_sim + beta * u_prev;
            double e_real = ref - y_sim;
            double err = e_pred(k) - e_real;
            rmse += err * err;
        }
        rmse = std::sqrt(rmse / Np);

        std::cout << "\nRollout RMSE vs real trajectory: " << rmse << '\n';
        if (rmse > 0.10) {
            std::cerr << "[FAIL] Rollout RMSE " << rmse << " > 0.10 threshold\n";
            ok = false;
        }
    }

    std::cout << (ok ? "[PASS] All checks passed.\n" : "[FAIL] See above.\n");
    return ok ? 0 : 1;
}
