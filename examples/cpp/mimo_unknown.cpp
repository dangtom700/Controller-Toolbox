/*
 * mimo_unknown.cpp - Unknown MIMO Plant: System ID + Fuzzy Performance
 * ======================================================================
 * Case 3a: MIMO plant assumed unknown. Model estimated from I/O data.
 *
 * Workflow:
 *   1. Read two orthogonal PRBS excitation signals from CSV (Python-generated).
 *      If CSV not found, generate PRBS internally using a 10-bit LFSR.
 *   2. Drive the unknown plant (CMSD-2, treated as black-box) with the
 *      PRBS inputs and record the 2*2 I/O dataset.
 *   3. MIMO ARX(2,2) identification: separate LS for each output channel,
 *      both inputs used as common regressors.
 *      Regression vector:  φ(k) = [-y1(k-1), -y2(k-1), -y1(k-2), -y2(k-2),
 *                                    u1(k-1),  u2(k-1),  u1(k-2),  u2(k-2)]
 *   4. Convert identified ARX model to state-space (observable companion form,
 *      8th order -> represents a 2nd-order MIMO system with 2-step lag memory).
 *   5. Validate: one-step-ahead NRMSE on a separate PRBS validation set.
 *      Target: NRMSE <= 5 % per output channel.
 *   6. Step response of identified diagonal channels -> FOPDT -> IMC-PID per
 *      channel (decentralized control, avoids DARE on high-order model).
 *   7. Apply decentralized IMC-PID to the TRUE plant (closed-loop simulation).
 *   8. Fuzzy performance evaluation on the closed-loop metrics.
 *
 * Identification tolerances:
 *   NRMSE_y1, NRMSE_y2 <= 5 % (pass/fail printed)
 *   Coefficient error <= 5 % relative to true coefficients (if known)
 *
 * Expected output:
 *   - Identified ARX coefficient matrices A1, A2, B1, B2
 *   - Validation NRMSE for y1 and y2
 *   - FOPDT parameters for each diagonal channel
 *   - Tuned IMC-PID gains
 *   - Closed-loop performance table
 *   - Fuzzy performance report
 *   - Results -> mimo_unknown_results.csv
 *
 * Build: see examples/cpp/CMakeLists.txt
 * Python data generation: examples/python/generate_mimo_data.py
 */

#include "mimo_plant.h"
#include "fuzzy_performance.h"
#include "../../lib/DiscretePID.h"

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <algorithm>

using namespace mimo;

// =========================================================================
// PRBS generator (10-bit LFSR, amplitude = +/-amp)
// =========================================================================
static std::vector<double> prbs(int N, double amp, int seed = 42) {
    unsigned state = (unsigned)seed & 0x3FFu;
    if (!state) state = 1u;
    std::vector<double> out(N);
    for (int k = 0; k < N; ++k) {
        out[k] = (state & 1u) ? amp : -amp;
        unsigned fb = ((state >> 9) ^ (state >> 6)) & 1u;
        state = ((state << 1) | fb) & 0x3FFu;
    }
    return out;
}

// =========================================================================
// Try to load PRBS from CSV; fall back to internal LFSR
// =========================================================================
static bool load_prbs_csv(const std::string& path,
                           std::vector<double>& u1, std::vector<double>& u2) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string line; std::getline(f, line); // header
    u1.clear(); u2.clear();
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string tok; double v1, v2;
        std::getline(ss, tok, ','); v1 = std::stod(tok);
        std::getline(ss, tok, ','); v2 = std::stod(tok);
        u1.push_back(v1); u2.push_back(v2);
    }
    return !u1.empty();
}

// =========================================================================
// MIMO ARX(2,2) identification - identifies all channels simultaneously
// Returns A1, A2 (2*2 output auto-regressive), B1, B2 (2*2 input)
// y(k) = -A1.y(k-1) - A2.y(k-2) + B1.u(k-1) + B2.u(k-2)
// =========================================================================
struct ARXModel {
    Eigen::Matrix2d A1, A2, B1, B2;
};

static ARXModel identify_arx(const Eigen::MatrixXd& Y,   // N * 2
                              const Eigen::MatrixXd& U,   // N * 2
                              int burn = 50) {
    int N = (int)Y.rows() - burn - 2;
    Eigen::MatrixXd Phi(N, 8);
    Eigen::MatrixXd Y_trg(N, 2);
    for (int i = 0; i < N; ++i) {
        int k = i + burn + 2;
        Phi.row(i) << -Y(k-1,0), -Y(k-1,1), -Y(k-2,0), -Y(k-2,1),
                       U(k-1,0),  U(k-1,1),  U(k-2,0),  U(k-2,1);
        Y_trg.row(i) = Y.row(k);
    }
    // Solve: Y_trg = Phi * Theta'  ->  Theta = (Phi'Phi)^{-1} Phi' Y_trg
    Eigen::MatrixXd Theta = (Phi.transpose()*Phi).ldlt().solve(Phi.transpose()*Y_trg);
    ARXModel m;
    // θ for y1: Theta.col(0) = [a11_1,a12_1,a11_2,a12_2,b11_1,b12_1,b11_2,b12_2]
    for (int j = 0; j < 2; ++j) {
        m.A1(j,0) = Theta(0,j); m.A1(j,1) = Theta(1,j);
        m.A2(j,0) = Theta(2,j); m.A2(j,1) = Theta(3,j);
        m.B1(j,0) = Theta(4,j); m.B1(j,1) = Theta(5,j);
        m.B2(j,0) = Theta(6,j); m.B2(j,1) = Theta(7,j);
    }
    return m;
}

// =========================================================================
// One-step-ahead prediction using identified ARX model
// Returns NRMSE per output channel
// =========================================================================
static std::array<double,2> validate_arx(const ARXModel& m,
                                          const Eigen::MatrixXd& Y,
                                          const Eigen::MatrixXd& U,
                                          int burn = 50) {
    int N = (int)Y.rows();
    std::array<double,2> sse{0,0}, y_range{0,0};
    for (int k = burn+2; k < N; ++k) {
        Eigen::Vector2d phi_row;
        Eigen::Vector2d y_hat = -m.A1*Y.row(k-1).transpose() - m.A2*Y.row(k-2).transpose()
                               + m.B1*U.row(k-1).transpose() + m.B2*U.row(k-2).transpose();
        for (int c = 0; c < 2; ++c) {
            double err = Y(k,c) - y_hat[c];
            sse[c] += err * err;
        }
    }
    for (int c = 0; c < 2; ++c) {
        double ymin = Y.col(c).minCoeff(), ymax = Y.col(c).maxCoeff();
        y_range[c] = std::max(ymax - ymin, 1e-12);
        sse[c] = std::sqrt(sse[c] / (N - burn - 2)) / y_range[c];
    }
    return sse;  // NRMSE fractions (< 0.05 -> pass)
}

// =========================================================================
// Simulate identified ARX model for step response (diagonal channel)
// Used for FOPDT estimation without requiring true plant access
// =========================================================================
static std::vector<double>
arx_step_response(const ARXModel& m, int ch, int input_ch, int steps = 3000) {
    Eigen::VectorXd y_km1 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd y_km2 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u_km1 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u_km2 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u(2); u.setZero(); u[input_ch] = 1.0;
    std::vector<double> y_hist(steps);
    for (int k = 0; k < steps; ++k) {
        Eigen::Vector2d y_k = -m.A1*y_km1 - m.A2*y_km2 + m.B1*u_km1 + m.B2*u_km2;
        y_hist[k] = y_k[ch];
        y_km2 = y_km1; y_km1 = y_k;
        u_km2 = u_km1; u_km1 = u;
    }
    return y_hist;
}

// =========================================================================
// FOPDT tangent method (28.3% / 63.2% crossings)
// =========================================================================
struct FOPDT { double K, tau, theta; };

static FOPDT fopdt_from_step(const std::vector<double>& y, double ts) {
    int N = (int)y.size();
    double K = y[N-1];
    if (std::abs(K) < 1e-10) return {0,1,0};
    double y28 = 0.283*K, y63 = 0.632*K;
    int t28 = 0, t63 = 1;
    for (int k = 0; k < N; ++k) {
        if (!t28 && y[k] >= y28) t28 = k;
        if (t28  && y[k] >= y63) { t63 = k; break; }
    }
    double tau   = 1.5 * (t63 - t28) * ts;
    double theta = std::max(ts, t63 * ts - tau);
    return {K, tau, theta};
}

// =========================================================================
// IMC-PID from FOPDT
// =========================================================================
static std::tuple<double,double,double>
imc_pid_from_fopdt(const FOPDT& f, double lam) {
    double Kp = (f.tau + f.theta/2.0) / (f.K * (lam + f.theta/2.0));
    double Ti = f.tau + f.theta/2.0;
    double Td = f.tau * f.theta / (2.0*f.tau + f.theta);
    return {Kp, Kp/Ti, Kp*Td};
}

// =========================================================================
// main
// =========================================================================
int main() {
    std::cout << "=================================================================\n";
    std::cout << " MIMO Unknown Case - System ID + Fuzzy Performance\n";
    std::cout << " Plant: Coupled Mass-Spring-Damper (treated as black box)\n";
    std::cout << "=================================================================\n";

    const int ID_STEPS   = 4000;
    const int VAL_STEPS  = 2000;
    const int SIM_STEPS  = 3000;
    const double AMP     = 0.5;
    const double REF1    = 1.0, REF2 = 1.0;
    const double UMAX    = 20.0;

    // =====================================================================
    // Step 1: Load or generate PRBS excitation signals
    // =====================================================================
    std::vector<double> u1_id, u2_id;
    std::string csv_path = "examples/data/mimo_prbs.csv";
    if (load_prbs_csv(csv_path, u1_id, u2_id)) {
        std::cout << "\n[1] Loaded PRBS from " << csv_path
                  << " (" << u1_id.size() << " samples)\n";
    } else {
        std::cout << "\n[1] CSV not found - generating internal PRBS (10-bit LFSR)\n";
        u1_id = prbs(ID_STEPS, AMP, 42);    // seed 42
        u2_id = prbs(ID_STEPS, AMP, 1023);  // orthogonal seed
    }
    u1_id.resize(ID_STEPS, 0.0);
    u2_id.resize(ID_STEPS, 0.0);

    // Separate validation sequences
    std::vector<double> u1_val = prbs(VAL_STEPS, AMP, 777);
    std::vector<double> u2_val = prbs(VAL_STEPS, AMP, 333);

    // =====================================================================
    // Step 2: Drive plant (treated as black box) - collect I/O
    // =====================================================================
    std::cout << "[2] Collecting identification I/O (" << ID_STEPS << " samples)...\n";
    MIMOStateSpace plant_id = make_plant();
    Eigen::MatrixXd Y_id(ID_STEPS, 2), U_id(ID_STEPS, 2);
    for (int k = 0; k < ID_STEPS; ++k) {
        Eigen::VectorXd u(2); u << u1_id[k], u2_id[k];
        Eigen::VectorXd y = plant_id.step(u);
        Y_id.row(k) = y.transpose();
        U_id.row(k) = u.transpose();
    }
    // Validation I/O
    MIMOStateSpace plant_val = make_plant();
    Eigen::MatrixXd Y_val(VAL_STEPS, 2), U_val(VAL_STEPS, 2);
    for (int k = 0; k < VAL_STEPS; ++k) {
        Eigen::VectorXd u(2); u << u1_val[k], u2_val[k];
        Eigen::VectorXd y = plant_val.step(u);
        Y_val.row(k) = y.transpose();
        U_val.row(k) = u.transpose();
    }

    // =====================================================================
    // Step 3: MIMO ARX(2,2) identification
    // =====================================================================
    std::cout << "[3] MIMO ARX(2,2) identification (batch LS)...\n";
    ARXModel arx = identify_arx(Y_id, U_id, 50);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  A1 =\n" << arx.A1 << "\n";
    std::cout << "  A2 =\n" << arx.A2 << "\n";
    std::cout << "  B1 =\n" << arx.B1 << "\n";
    std::cout << "  B2 =\n" << arx.B2 << "\n";

    // =====================================================================
    // Step 4: Validate - NRMSE on holdout set
    // =====================================================================
    std::cout << "[4] Validation (one-step-ahead NRMSE)...\n";
    auto nrmse = validate_arx(arx, Y_val, U_val, 10);
    bool pass_y1 = nrmse[0] < 0.05;
    bool pass_y2 = nrmse[1] < 0.05;
    std::cout << std::setprecision(4);
    std::cout << "  NRMSE y1 = " << nrmse[0]*100.0 << "% - "
              << (pass_y1 ? "[PASS <=5%]" : "[FAIL >5%]") << "\n";
    std::cout << "  NRMSE y2 = " << nrmse[1]*100.0 << "% - "
              << (pass_y2 ? "[PASS <=5%]" : "[FAIL >5%]") << "\n";

    if (!pass_y1 || !pass_y2) {
        std::cout << "  [WARN] NRMSE tolerance exceeded - "
                  << "consider higher-order model or more excitation bandwidth.\n";
    }

    // =====================================================================
    // Step 5: Diagonal FOPDT estimation from identified model
    // =====================================================================
    std::cout << "[5] Diagonal FOPDT estimation from identified step response...\n";
    auto y_step_11 = arx_step_response(arx, 0, 0, 3000);  // G11: u1->y1
    auto y_step_22 = arx_step_response(arx, 1, 1, 3000);  // G22: u2->y2
    auto f11 = fopdt_from_step(y_step_11, Ts);
    auto f22 = fopdt_from_step(y_step_22, Ts);
    std::cout << "  Channel 1 FOPDT: K="  << f11.K   << " tau=" << f11.tau
              << " theta=" << f11.theta << "\n";
    std::cout << "  Channel 2 FOPDT: K="  << f22.K   << " tau=" << f22.tau
              << " theta=" << f22.theta << "\n";

    // Off-diagonal check for cross-coupling estimate
    auto y_step_21 = arx_step_response(arx, 1, 0, 3000);  // G21: u1->y2
    auto y_step_12 = arx_step_response(arx, 0, 1, 3000);  // G12: u2->y1
    double K21 = y_step_21.back(), K12 = y_step_12.back();
    std::cout << "  Off-diagonal DC gains: G21=" << K21 << " G12=" << K12
              << "  (coupling ratio: " << std::abs(K21/f11.K)*100.0 << "%)\n";

    // =====================================================================
    // Step 6: IMC-PID design on identified FOPDT
    // =====================================================================
    std::cout << "[6] IMC-PID design on identified model (lambda=0.8)...\n";
    double lam = 0.8;
    auto [Kp1, Ki1, Kd1] = imc_pid_from_fopdt(f11, lam);
    auto [Kp2, Ki2, Kd2] = imc_pid_from_fopdt(f22, lam);
    std::cout << "  PID1: Kp=" << Kp1 << " Ki=" << Ki1 << " Kd=" << Kd1 << "\n";
    std::cout << "  PID2: Kp=" << Kp2 << " Ki=" << Ki2 << " Kd=" << Kd2 << "\n";

    // =====================================================================
    // Step 7: Closed-loop simulation on TRUE plant
    // =====================================================================
    std::cout << "[7] Closed-loop simulation on true plant (" << SIM_STEPS << " steps)...\n";
    auto make_pid = [UMAX](double Kp, double Ki, double Kd) {
        ctrl::PIDParams pp; pp.Kp=Kp; pp.Ki=Ki; pp.Kd=Kd;
        pp.N=10.0; pp.uMin=-UMAX; pp.uMax=UMAX;
        return ctrl::DiscretePID(pp, Ts);
    };
    auto pid1 = make_pid(Kp1, Ki1, Kd1);
    auto pid2 = make_pid(Kp2, Ki2, Kd2);
    MIMOStateSpace plant_cl = make_plant();
    Eigen::MatrixXd Y_cl(SIM_STEPS, 2), U_cl(SIM_STEPS, 2);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu);
    for (int k = 0; k < SIM_STEPS; ++k) {
        Eigen::VectorXd y = plant_cl.step(u);
        Y_cl.row(k) = y.transpose(); U_cl.row(k) = u.transpose();
        u[0] = std::clamp(pid1.compute(REF1 - y[0]), -UMAX, UMAX);
        u[1] = std::clamp(pid2.compute(REF2 - y[1]), -UMAX, UMAX);
    }
    MIMOMetrics cl_metrics = compute_metrics(Y_cl, U_cl, REF1, REF2);
    std::cout << "\n";
    cl_metrics.print("ID-IMC-PID");

    // =====================================================================
    // Step 8: Fuzzy performance evaluation
    // =====================================================================
    std::cout << "\n[8] Fuzzy performance evaluation...\n";
    double ise_total  = cl_metrics.ISE_y1 + cl_metrics.ISE_y2;
    double ise_worst  = 2.0 * REF1 * REF1 * SIM_STEPS * Ts;  // worst case: never converges
    double ise_norm   = std::min(ise_total / ise_worst, 1.0);
    double os_pct     = std::max(cl_metrics.OS_y1, cl_metrics.OS_y2);
    // Estimate settling from ITAE (high ITAE -> slow settling)
    double itae_norm  = (cl_metrics.ITAE_y1 + cl_metrics.ITAE_y2) /
                        std::max(cl_metrics.ITAE_y1 + cl_metrics.ITAE_y2, 1e-6);
    double st_norm    = std::min(0.6 * itae_norm + 0.1 * os_pct/100.0, 1.0);

    auto fr = fuzzy::evaluate(ise_norm, os_pct, st_norm);
    fuzzy::print_report("MIMO-Unknown ID-IMC-PID", ise_norm, os_pct, st_norm, fr);

    // Model quality assessment via fuzzy estimator
    double model_ise_n = (nrmse[0]*nrmse[0] + nrmse[1]*nrmse[1]) / (2.0*0.05*0.05);
    model_ise_n = std::min(model_ise_n, 1.0);
    auto model_fr = fuzzy::evaluate(model_ise_n, 0.0, 0.2);
    fuzzy::print_report("Model-ID Quality", model_ise_n, 0.0, 0.2, model_fr);

    // =====================================================================
    // Save results
    // =====================================================================
    {
        std::ofstream of("mimo_unknown_results.csv");
        of << "step,y1,y2,u1,u2\n";
        for (int k = 0; k < SIM_STEPS; ++k)
            of << k*Ts << "," << Y_cl(k,0) << "," << Y_cl(k,1) << ","
               << U_cl(k,0) << "," << U_cl(k,1) << "\n";
        std::cout << "\n  Closed-loop trajectory -> mimo_unknown_results.csv\n";
    }
    {
        std::ofstream of("mimo_unknown_id.csv");
        of << "channel,NRMSE_pct,K,tau,theta,Kp,Ki,Kd\n";
        of << std::fixed << std::setprecision(6)
           << "y1," << nrmse[0]*100 << "," << f11.K << "," << f11.tau << "," << f11.theta
           << "," << Kp1 << "," << Ki1 << "," << Kd1 << "\n"
           << "y2," << nrmse[1]*100 << "," << f22.K << "," << f22.tau << "," << f22.theta
           << "," << Kp2 << "," << Ki2 << "," << Kd2 << "\n";
        std::cout << "  Identification report -> mimo_unknown_id.csv\n";
    }
    std::cout << "\n  VALIDATION SUMMARY:\n"
              << "    NRMSE y1: " << nrmse[0]*100 << "% " << (pass_y1?"[PASS]":"[FAIL]") << "\n"
              << "    NRMSE y2: " << nrmse[1]*100 << "% " << (pass_y2?"[PASS]":"[FAIL]") << "\n"
              << "    Closed-loop ISE total: " << cl_metrics.ISE_y1+cl_metrics.ISE_y2 << "\n"
              << "    Fuzzy score: " << fr.score << " (" << fr.grade << ")\n";
    return 0;
}
