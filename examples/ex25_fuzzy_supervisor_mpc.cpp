// ============================================================
//  ex25_fuzzy_supervisor_mpc.cpp
//  Application: pH neutralisation reactor — nonlinear plant with
//  FuzzySupervisor driving adaptive MPC re-linearisation.
//
//  Plant: Hammerstein model (static nonlinearity + linear dynamics)
//    Linear:  G(s) = 1 / (30s + 1)   [pH / valve_cmd]
//    NL gain: k_pH(pH) = 5.0 - 3.0*tanh(2*(pH - 7))
//    Effective:  G_eff = k_pH(y) * G(s)
//
//  The nonlinear gain changes by factor ~3× across the operating range
//  (pH 4–10), invalidating a single linear MPC model.
//
//  Demonstrates:
//    - Baseline MPC (fixed linearisation at pH 7)
//    - FuzzySupervisor detecting plant-model mismatch -> triggers relinearize
//    - Relinearised MPC adapts gain at current operating point
//
//  Output columns: t,ref,y_mpc_fixed,u_mpc_fixed,y_mpc_adapt,u_mpc_adapt,
//                  relinearize_signal,relinearize_event,k_eff
// ============================================================
#include "ControllerToolbox.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>

// Hammerstein plant: linear dynamics wrapped with static NL output gain.
// The "true" plant gain seen by the controller depends on current pH.
static double phGain(double pH)
{
    return 5.0 - 3.0 * std::tanh(2.0 * (pH - 7.0));
}

// Build a discretised first-order plant scaled by gain k.
static ctrl::StateSpace buildPlantSS(double k, double Ts)
{
    // G(s) = k / (30s + 1)  ->  c2d ZOH
    double a = std::exp(-Ts / 30.0);
    double b = k * (1.0 - a);
    Eigen::MatrixXd A(1,1); A << a;
    Eigen::MatrixXd B(1,1); B << b;
    Eigen::MatrixXd C(1,1); C << 1;
    Eigen::MatrixXd D(1,1); D << 0;
    return ctrl::StateSpace(A, B, C, D, Ts);
}

int main()
{
    const double Ts  = 2.0;    // 2-second sample
    const double pH0 = 7.0;    // nominal operating point

    // ── Reference trajectory ─────────────────────────────────────────────────
    // Steps through pH 7 -> 9 -> 5 -> 8 to excite the nonlinearity
    auto refProfile = [](double t) -> double {
        if (t <  60.0) return 7.0;
        if (t < 180.0) return 9.0;
        if (t < 300.0) return 5.0;
        return 8.0;
    };

    // ── Fixed MPC (linearised at pH 7, never updated) ────────────────────────
    ctrl::MPCParams mp;
    mp.Np    = 20;
    mp.Nc    = 5;
    mp.rho_y = 10.0;
    mp.rho_u = 0.01;
    mp.uMin  = 0.0;   // valve: 0–100%
    mp.uMax  = 100.0;
    mp.duMin = -10.0;
    mp.duMax =  10.0;

    double k_nom = phGain(pH0);
    ctrl::DiscreteMPC mpc_fixed(buildPlantSS(k_nom, Ts), mp);
    ctrl::DiscreteMPC mpc_adapt(buildPlantSS(k_nom, Ts), mp);

    // ── FuzzySupervisor ───────────────────────────────────────────────────────
    // Fires when |e| > 0.8 pH units AND growing -> relinearise
    ctrl::SupervisorParams sp;
    sp.e_threshold     = 0.8;    // 0.8 pH unit error = "Large"
    sp.trend_threshold = 0.05;   // 0.05 pH/s divergence = "Increasing"
    sp.signal_threshold = 0.5;
    sp.cooldown_steps  = 15;     // wait 30 s before re-triggering
    ctrl::FuzzySupervisor supervisor(sp, Ts);

    // ── Simulation ────────────────────────────────────────────────────────────
    const int N = 240;   // 480 s total
    double yf = 7.0, ya = 7.0;
    Eigen::VectorXd xf = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd xa = Eigen::VectorXd::Zero(1);
    // Warm-start: steady state of y=7 with no input is handled by gain offset;
    // for this demo we start at equilibrium (x=7/k_nom·(1-a)/... -> initial y=7
    // via a precomputed IC).
    xf(0) = 7.0;  // approximate: output ~ state for unity C
    xa(0) = 7.0;

    int relinearize_count = 0;

    std::ofstream csv("data/ex25_fuzzy_supervisor_mpc.csv");
    csv << "t,ref,y_mpc_fixed,u_mpc_fixed,y_mpc_adapt,u_mpc_adapt,"
        << "relinearize_signal,relinearize_event,k_eff\n";
    csv << std::fixed << std::setprecision(4);

    std::cout << "=== FuzzySupervisor + Adaptive MPC — pH Neutralisation ===\n";
    std::cout << "  k    t[s]  ref  y_fix  u_fix  y_adp  u_adp  sig   relins\n";
    std::cout << std::string(72, '-') << "\n";

    for (int k = 0; k < N; ++k) {
        double t = k * Ts;
        double r = refProfile(t);

        // ── Fixed MPC ────────────────────────────────────────────────────────
        double ef = r - yf;
        double uf = mpc_fixed.compute(ef);
        // True plant response: NL gain applied to linear output
        double k_true_f = phGain(yf);
        double y_lin_f  = ctrl::ssStep(buildPlantSS(k_true_f, Ts), xf,
                                        Eigen::VectorXd::Constant(1, uf))(0);
        // Offset to keep around operating point pH (add bias so y represents pH)
        yf = y_lin_f;

        // ── Adaptive MPC via FuzzySupervisor ─────────────────────────────────
        double ea  = r - ya;
        double ua  = mpc_adapt.compute(ea);

        ctrl::SupervisorDecision dec = supervisor.update(std::abs(ea));
        bool did_relin = false;

        if (dec.relinearize) {
            // Re-linearise the MPC model at the current operating pH
            double k_op = phGain(ya);
            mpc_adapt.setPlant(buildPlantSS(k_op, Ts));
            ++relinearize_count;
            did_relin = true;
        }

        double k_true_a = phGain(ya);
        double y_lin_a  = ctrl::ssStep(buildPlantSS(k_true_a, Ts), xa,
                                        Eigen::VectorXd::Constant(1, ua))(0);
        ya = y_lin_a;

        csv << t << "," << r << ","
            << yf << "," << uf << ","
            << ya << "," << ua << ","
            << dec.relinearize_signal << ","
            << (did_relin ? 1 : 0) << ","
            << k_true_a << "\n";

        if (k % 20 == 0)
            std::cout << std::setw(4) << k
                      << std::setw(6)  << t
                      << std::setw(5)  << r
                      << std::setw(7)  << yf
                      << std::setw(7)  << uf
                      << std::setw(7)  << ya
                      << std::setw(7)  << ua
                      << std::setw(6)  << dec.relinearize_signal
                      << std::setw(4)  << relinearize_count << "\n";
    }

    std::cout << "\nTotal re-linearisations: " << relinearize_count
              << "\nResults saved to data/ex25_fuzzy_supervisor_mpc.csv\n";
    return 0;
}
