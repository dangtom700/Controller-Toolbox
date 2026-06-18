// ============================================================
//  ex28_gpc_adaptive.cpp
//  Adaptive GPC: GeneralizedPredictiveController + RecursiveLeastSquares.
//  Plant gain shifts mid-run; RLS re-identifies and GPC adapts.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main()
{
    const double Ts   = 0.5;
    const double r_sp = 10.0; // setpoint [L/min]

    // Build a first-order discrete plant: G(s) = K/(10s+1), ZOH @ Ts
    // Initial gain K=1.0; switches to K=1.6 at t=100s
    auto make_plant = [&](double K) {
        const double a = std::exp(-Ts / 10.0);
        Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
        A << a; B << K*(1-a); C << 1.0; D << 0.0;
        return ctrl::StateSpace(A, B, C, D, Ts);
    };

    ctrl::StateSpace plant_nominal = make_plant(1.0);

    // GPC: CARIMA velocity-form predictor, alpha=0.1 for soft reference trajectory
    ctrl::GPCParams gp;
    gp.Np = 20; gp.Nu = 6;
    gp.rho_y = 5.0; gp.rho_u = 0.5;
    gp.alpha = 0.1;
    gp.uMin = 0.0; gp.uMax = 25.0;
    ctrl::GeneralizedPredictiveController gpc(plant_nominal, gp);

    // RLS online identification: na=1, nb=1, forgetting factor 0.98
    ctrl::RecursiveLeastSquares rls(1, 1, 0.98, Ts);

    Eigen::VectorXd x(1); x << 0.0;
    double u = 0.0;
    const int RETUNE_EVERY = 50;

    std::cout << "=== Adaptive GPC with RLS Re-identification ===\n"
              << "  Plant gain K=1.0 switches to K=1.6 at t=100s\n"
              << "  RLS re-identifies every " << RETUNE_EVERY << " steps\n\n"
              << std::setw(6) << "t[s]"
              << std::setw(10) << "y[L/min]"
              << std::setw(10) << "u[%]"
              << std::setw(12) << "K_identified"
              << std::setw(8) << "QP_ok\n"
              << std::string(50, '-') << "\n";

    const int N = static_cast<int>(250.0 / Ts);

    for (int k = 0; k < N; ++k) {
        const double t = k * Ts;
        const ctrl::StateSpace& pl = (t >= 100.0) ? make_plant(1.6) : plant_nominal;

        // Step plant
        Eigen::VectorXd u_vec(1); u_vec << u;
        const Eigen::VectorXd y_vec = ctrl::ssStep(pl, x, u_vec);
        const double y = y_vec(0);

        // Update RLS
        rls.update(y, u);

        // Periodically hot-swap GPC plant model
        if (k > 20 && k % RETUNE_EVERY == 0) {
            ctrl::StateSpace ss_id = rls.toStateSpace();
            gpc.setPlant(ss_id);   // hot-swap; ss_id is well-formed when RLS has enough data
        }

        // GPC compute
        u = std::clamp(gpc.computeRef(y, r_sp), gp.uMin, gp.uMax);

        // Identified gain estimate: K_id = b1 / (1 + a1)
        const Eigen::VectorXd theta = rls.params();
        const double a1 = theta(0), b1 = theta(1);
        const double K_id = (std::abs(1.0 + a1) > 0.01) ? b1 / (1.0 + a1) : 0.0;

        if (k % 50 == 0)
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(6) << t
                      << std::setw(10) << y
                      << std::setw(10) << u
                      << std::setw(12) << std::setprecision(3) << K_id
                      << std::setw(8) << (gpc.lastQPConverged() ? "Yes" : "No") << "\n";
    }

    std::cout << "\nFinal: y=" << x(0) << " L/min  setpoint=" << r_sp << " L/min\n";
    return 0;
}
