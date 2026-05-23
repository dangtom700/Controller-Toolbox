/*
 * mimo_known.cpp - MIMO Known-Plant: Full Suite of Tuning Methods
 * ================================================================
 * Case 1: Strong cross-coupling MIMO plant (Coupled Mass-Spring-Damper, 2-mass)
 *
 * Plant: G(s) 2*2, states [x1,ẋ1,x2,ẋ2]', inputs [F1,F2]', outputs [x1,x2]'
 * Cross-coupling RGA approx = [1.22, -0.22; -0.22, 1.22] (non-trivial pairing effect)
 *
 * Controllers evaluated (all tuned for identical reference [1, 1]'):
 *   A. Decentralized PID    - two independent loops; IMC tuned on diagonal channels
 *   B. Decentralized PID    - Nelder-Mead optimized composite ISE cost
 *   C. Full-state LQR       - Bryson's Rule (xmax=[1,1,1,1], umax=[10,10])
 *   D. Full-state LQR       - Nelder-Mead weight optimization
 *   E. LQG (LQR + Kalman)  - Bryson + isotropic process/measurement noise
 *   F. MIMO MPC             - Np=20, Nc=10, Q=I, R=0.01*I
 *   G. MIMO MPC             - Nelder-Mead horizon/weight optimization
 *
 * Cost function for Nelder-Mead (J):
 *   J = ISE_y1 + ISE_y2 + 0.1*(ITAE_y1 + ITAE_y2) + 0.01*(E_u1 + E_u2)
 *
 * Expected output:
 *   - System equations and RGA printed to stdout
 *   - Per-method table: ISE_y1, ISE_y2, ITAE_y1, ITAE_y2, E_u1, E_u2, OS[%], J
 *   - Pareto-optimal method identified (lowest J)
 *   - All results saved to examples/data/mimo_known_results.csv
 *
 * Build: see examples/cpp/CMakeLists.txt
 * Run:   ./mimo_known
 */

#include "mimo_plant.h"
#include "fuzzy_performance.h"
#include "../../lib/DiscretePID.h"

#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <numeric>

using namespace mimo;
using namespace ctrl;

// =========================================================================
// Simulation helpers
// =========================================================================
static constexpr int   SIM_STEPS = 3000;   // 30 s at Ts=0.01
static constexpr double REF1 = 1.0, REF2 = 1.0;
static constexpr double DIST_STEP = 2000;  // disturbance at k=2000
static constexpr double DIST_MAG  = 0.2;

// Run simulation; returns [Y(N*2), U(N*2)]
static std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
run_sim(std::function<Eigen::VectorXd(const Eigen::VectorXd& /*y*/,
                                      const Eigen::VectorXd& /*u_prev*/)> ctrl,
        bool apply_disturbance = true) {
    MIMOStateSpace plant = make_plant();
    Eigen::MatrixXd Y(SIM_STEPS, 2), U(SIM_STEPS, 2);
    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(Nu);

    for (int k = 0; k < SIM_STEPS; ++k) {
        Eigen::VectorXd y = plant.step(u_prev);
        Y.row(k) = y.transpose();
        U.row(k) = u_prev.transpose();
        Eigen::VectorXd u_new = ctrl(y, u_prev);
        // Saturate control input [-20, 20]
        u_new = u_new.cwiseMax(-20.0).cwiseMin(20.0);
        // Add output disturbance at k=DIST_STEP
        Eigen::VectorXd dist = Eigen::VectorXd::Zero(Ny);
        if (apply_disturbance && k >= DIST_STEP) dist.fill(DIST_MAG);
        u_prev = u_new;
        // Note: dist enters as plant input perturbation
        if (apply_disturbance && k >= DIST_STEP) plant.x += plant.B * dist * Ts;
    }
    return {Y, U};
}

// =========================================================================
// Nelder-Mead minimiser (identical parameters to lib/TunerSuite.cpp)
// =========================================================================
static std::vector<double>
nelder_mead(std::function<double(const std::vector<double>&)> f,
            std::vector<double> x0,
            const std::vector<std::pair<double,double>>& bounds,
            int maxEvals = 400, double tol = 1e-6) {
    const double alpha = 1.0, gamma = 2.0, rho = 0.5, sigma = 0.5;
    int n = (int)x0.size();
    auto clamp = [&](std::vector<double> v) {
        for (int i = 0; i < n; ++i)
            v[i] = std::clamp(v[i], bounds[i].first, bounds[i].second);
        return v;
    };
    std::vector<std::vector<double>> simp = { clamp(x0) };
    for (int i = 0; i < n; ++i) {
        auto v = x0; v[i] *= 1.05;
        simp.push_back(clamp(v));
    }
    std::vector<double> costs;
    for (auto& s : simp) costs.push_back(f(s));
    int evals = n + 1;

    while (evals < maxEvals) {
        // Sort
        std::vector<int> idx(n + 1); std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&](int a, int b){ return costs[a] < costs[b]; });
        std::vector<std::vector<double>> s2; std::vector<double> c2;
        for (int i : idx) { s2.push_back(simp[i]); c2.push_back(costs[i]); }
        simp = s2; costs = c2;
        // Convergence
        double diam = 0;
        for (int i = 1; i <= n; ++i)
            for (int j = 0; j < n; ++j)
                diam = std::max(diam, std::abs(simp[i][j] - simp[0][j]));
        if (diam < tol) break;
        // Centroid of all but worst
        std::vector<double> cen(n, 0.0);
        for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) cen[j] += simp[i][j] / n;
        // Reflect
        std::vector<double> xr(n); for (int j = 0; j < n; ++j) xr[j] = cen[j] + alpha*(cen[j] - simp[n][j]);
        xr = clamp(xr); double fr = f(xr); ++evals;
        if (fr < costs[0]) {
            std::vector<double> xe(n); for (int j = 0; j < n; ++j) xe[j] = cen[j] + gamma*(xr[j] - cen[j]);
            xe = clamp(xe); double fe = f(xe); ++evals;
            simp[n] = (fe < fr) ? xe : xr; costs[n] = (fe < fr) ? fe : fr;
        } else if (fr < costs[n-1]) {
            simp[n] = xr; costs[n] = fr;
        } else {
            std::vector<double> xc(n); for (int j = 0; j < n; ++j) xc[j] = cen[j] + rho*(simp[n][j] - cen[j]);
            xc = clamp(xc); double fc = f(xc); ++evals;
            if (fc < costs[n]) { simp[n] = xc; costs[n] = fc; }
            else {
                for (int i = 1; i <= n; ++i) {
                    for (int j = 0; j < n; ++j) simp[i][j] = simp[0][j] + sigma*(simp[i][j] - simp[0][j]);
                    simp[i] = clamp(simp[i]); costs[i] = f(simp[i]); ++evals;
                }
            }
        }
    }
    return simp[0];
}

// =========================================================================
// A. Decentralized PID - IMC-tuned on diagonal FOPDT channels
// =========================================================================
// Identify diagonal channels via step response, fit FOPDT, apply IMC rule.
static MIMOMetrics run_decentralized_pid_imc() {
    // Channel 1: step F1=1, F2=0, observe y1
    // Channel 2: step F2=1, F1=0, observe y2
    auto identify_channel = [](int input_ch) {
        MIMOStateSpace p = make_plant();
        Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu);
        u[input_ch] = 1.0;
        const int ID_STEPS = 2000;
        double y_prev = 0.0, y_final = 0.0;
        std::vector<double> y_hist;
        for (int k = 0; k < ID_STEPS; ++k) {
            Eigen::VectorXd yv = p.step(u);
            double y = yv[input_ch];   // diagonal output
            y_hist.push_back(y);
            y_final = y;
        }
        double K   = y_final;
        double y28 = 0.283 * K, y63 = 0.632 * K;
        int    t28 = 0, t63 = 0;
        for (int k = 0; k < ID_STEPS; ++k) {
            if (!t28 && y_hist[k] >= y28) t28 = k;
            if (!t63 && y_hist[k] >= y63) { t63 = k; break; }
        }
        double tau   = 1.5 * (t63 - t28) * Ts;
        double theta = std::max(Ts, t63 * Ts - tau);
        return std::make_tuple(K, tau, theta);
    };

    auto imc_pid = [](double K, double tau, double theta, double lam)
        -> std::tuple<double,double,double> {
        double Kp = (tau + theta / 2.0) / (K * (lam + theta / 2.0));
        double Ti = tau + theta / 2.0;
        double Td = tau * theta / (2.0 * tau + theta);
        return { Kp, Kp / Ti, Kp * Td };
    };

    auto [K1, tau1, theta1] = identify_channel(0);
    auto [K2, tau2, theta2] = identify_channel(1);
    auto [Kp1, Ki1, Kd1]   = imc_pid(K1, tau1, theta1, 1.0);
    auto [Kp2, Ki2, Kd2]   = imc_pid(K2, tau2, theta2, 1.0);

    auto make_pid20 = [](double Kp, double Ki, double Kd) {
        PIDParams pp; pp.Kp=Kp; pp.Ki=Ki; pp.Kd=Kd;
        pp.N=10.0; pp.uMin=-20.0; pp.uMax=20.0;
        return DiscretePID(pp, Ts);
    };
    auto pid1 = make_pid20(Kp1, Ki1, Kd1);
    auto pid2 = make_pid20(Kp2, Ki2, Kd2);

    auto ctrl = [&](const Eigen::VectorXd& y, const Eigen::VectorXd&) {
        Eigen::VectorXd u(Nu);
        u[0] = pid1.compute(REF1 - y[0]);
        u[1] = pid2.compute(REF2 - y[1]);
        return u;
    };

    auto [Y, U] = run_sim(ctrl);
    std::cout << "  IMC-PID:  Kp1=" << std::setprecision(4) << Kp1
              << " Ki1=" << Ki1 << " Kd1=" << Kd1
              << " | Kp2=" << Kp2 << " Ki2=" << Ki2 << " Kd2=" << Kd2 << "\n";
    return compute_metrics(Y, U, REF1, REF2);
}

// =========================================================================
// B. Decentralized PID - Nelder-Mead optimised
// =========================================================================
static MIMOMetrics run_decentralized_pid_opt() {
    auto cost = [](const std::vector<double>& p) -> double {
        double Kp1=p[0], Ki1=p[1], Kd1=p[2], Kp2=p[3], Ki2=p[4], Kd2=p[5];
        if (Kp1 <= 0 || Ki1 < 0 || Kp2 <= 0 || Ki2 < 0) return 1e6;
        auto make_pid_opt = [](double Kp, double Ki, double Kd) {
            PIDParams pp; pp.Kp=Kp; pp.Ki=Ki; pp.Kd=Kd;
            pp.N=10.0; pp.uMin=-20.0; pp.uMax=20.0;
            return DiscretePID(pp, Ts);
        };
        DiscretePID pid1 = make_pid_opt(Kp1, Ki1, Kd1);
        DiscretePID pid2 = make_pid_opt(Kp2, Ki2, Kd2);
        MIMOStateSpace plant = make_plant();
        Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu);
        double J = 0.0;
        for (int k = 0; k < SIM_STEPS; ++k) {
            Eigen::VectorXd y = plant.step(u);
            double t = k * Ts;
            double e1 = REF1 - y[0], e2 = REF2 - y[1];
            J += (e1*e1 + e2*e2) * Ts + 0.1 * t * (std::abs(e1)+std::abs(e2)) * Ts
               + 0.01 * (u[0]*u[0] + u[1]*u[1]) * Ts;
            u[0] = std::clamp(pid1.compute(e1), -20.0, 20.0);
            u[1] = std::clamp(pid2.compute(e2), -20.0, 20.0);
        }
        return J;
    };

    std::vector<double> x0 = {5.0, 0.5, 0.2, 5.0, 0.5, 0.2};
    std::vector<std::pair<double,double>> bounds = {
        {0.01,30},{0,20},{0,5},{0.01,30},{0,20},{0,5}};
    auto xopt = nelder_mead(cost, x0, bounds, 600);

    auto make_pid_final = [](double Kp, double Ki, double Kd) {
        PIDParams pp; pp.Kp=Kp; pp.Ki=Ki; pp.Kd=Kd;
        pp.N=10.0; pp.uMin=-20.0; pp.uMax=20.0;
        return DiscretePID(pp, Ts);
    };
    auto p1 = make_pid_final(xopt[0],xopt[1],xopt[2]);
    auto p2 = make_pid_final(xopt[3],xopt[4],xopt[5]);
    auto ctrl = [&](const Eigen::VectorXd& y, const Eigen::VectorXd&) {
        Eigen::VectorXd u(Nu);
        u[0] = p1.compute(REF1 - y[0]);
        u[1] = p2.compute(REF2 - y[1]);
        return u;
    };
    auto [Y, U] = run_sim(ctrl);
    std::cout << "  NM-PID:   Kp1=" << std::setprecision(4) << xopt[0]
              << " Ki1=" << xopt[1] << " Kd1=" << xopt[2]
              << " | Kp2=" << xopt[3] << " Ki2=" << xopt[4] << " Kd2=" << xopt[5] << "\n";
    return compute_metrics(Y, U, REF1, REF2);
}

// =========================================================================
// C. Full-state LQR - Bryson's Rule
// =========================================================================
static MIMOMetrics run_lqr_bryson() {
    MIMOStateSpace ref = make_plant();
    Eigen::VectorXd xmax(Nx); xmax << 1.0, 2.0, 1.0, 2.0;
    double umax = 10.0;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Nx, Nx);
    for (int i = 0; i < Nx; ++i) Q(i,i) = 1.0 / (xmax[i]*xmax[i]);
    Eigen::MatrixXd R = (1.0/(umax*umax)) * Eigen::MatrixXd::Identity(Nu, Nu);

    Eigen::MatrixXd K = lqr_gain(ref.A, ref.B, Q, R);
    Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(Nx);
    // Compute steady-state input u_ss = -K*x_ss + u_ff
    // For setpoint tracking: find x_ss such that x_ss = A*x_ss + B*u_ss, y_ss = C*x_ss = r
    // u_ss = -(R+B'PB)^{-1}B'P(A-I)*x_ss  (complex; use DC feed-forward instead)
    // Simplified: precompute feedforward gain Nbar
    Eigen::MatrixXd A_cl = ref.A - ref.B * K;
    Eigen::MatrixXd Nbar = -(ref.C * (A_cl - Eigen::MatrixXd::Identity(Nx,Nx)).inverse() * ref.B).inverse();

    Eigen::VectorXd r(Ny); r << REF1, REF2;
    auto ctrl = [&](const Eigen::VectorXd& y, const Eigen::VectorXd&) {
        MIMOStateSpace& p = ref;  // share state (we use actual state from plant)
        Eigen::VectorXd u = -K * p.x + Nbar * r;
        return u;
    };
    // For full-state feedback, we need plant.x directly
    MIMOStateSpace plant = make_plant();
    Eigen::MatrixXd Y(SIM_STEPS, 2), U(SIM_STEPS, 2);
    for (int k = 0; k < SIM_STEPS; ++k) {
        Eigen::VectorXd u = (-K * plant.x + Nbar * r).cwiseMax(-20.0).cwiseMin(20.0);
        U.row(k) = u.transpose();
        Eigen::VectorXd yv = plant.step(u);
        Y.row(k) = yv.transpose();
    }
    std::cout << "  LQR-Bryson: K = " << K.row(0).head(4).format(
                Eigen::IOFormat(4, 0, ", ", " ", "[", "]")) << "\n";
    return compute_metrics(Y, U, REF1, REF2);
}

// =========================================================================
// D. Full-state LQR - Nelder-Mead weight optimisation
// =========================================================================
static MIMOMetrics run_lqr_opt() {
    // Optimise 2 diagonal Q multipliers and 1 R multiplier (log-scale)
    auto cost = [](const std::vector<double>& p) {
        double qx = p[0], qv = p[1], rr = p[2];
        if (qx <= 0 || qv <= 0 || rr <= 0) return 1e6;
        Eigen::MatrixXd Q(Nx,Nx); Q.setZero();
        Q(0,0) = Q(2,2) = qx; Q(1,1) = Q(3,3) = qv;
        Eigen::MatrixXd R = rr * Eigen::MatrixXd::Identity(Nu,Nu);
        MIMOStateSpace plant = make_plant();
        Eigen::MatrixXd K;
        try { K = lqr_gain(plant.A, plant.B, Q, R); }
        catch (...) { return 1e6; }
        Eigen::MatrixXd A_cl = plant.A - plant.B * K;
        // Simple DC feedforward
        Eigen::MatrixXd tmp = plant.C * (-(A_cl - Eigen::MatrixXd::Identity(Nx,Nx))).inverse() * plant.B;
        if (std::abs(tmp.determinant()) < 1e-12) return 1e6;
        Eigen::MatrixXd Nbar = tmp.inverse();
        Eigen::VectorXd r(Ny); r << REF1, REF2;
        double J = 0.0;
        for (int k = 0; k < SIM_STEPS; ++k) {
            Eigen::VectorXd u = (-K * plant.x + Nbar * r).cwiseMax(-20.0).cwiseMin(20.0);
            Eigen::VectorXd yv = plant.step(u);
            double t = k * Ts;
            double e1 = REF1-yv[0], e2 = REF2-yv[1];
            J += (e1*e1+e2*e2)*Ts + 0.1*t*(std::abs(e1)+std::abs(e2))*Ts
               + 0.01*(u[0]*u[0]+u[1]*u[1])*Ts;
        }
        return J;
    };
    std::vector<double> x0 = {1.0, 0.25, 0.01};
    std::vector<std::pair<double,double>> bounds = {{0.001,100},{0.001,100},{0.0001,10}};
    auto xopt = nelder_mead(cost, x0, bounds, 400);

    Eigen::MatrixXd Q(Nx,Nx); Q.setZero();
    Q(0,0)=Q(2,2)=xopt[0]; Q(1,1)=Q(3,3)=xopt[1];
    Eigen::MatrixXd R = xopt[2] * Eigen::MatrixXd::Identity(Nu,Nu);
    MIMOStateSpace plant = make_plant();
    Eigen::MatrixXd K = lqr_gain(plant.A, plant.B, Q, R);
    Eigen::MatrixXd A_cl = plant.A - plant.B * K;
    Eigen::MatrixXd Nbar = (plant.C*(-(A_cl-Eigen::MatrixXd::Identity(Nx,Nx))).inverse()*plant.B).inverse();
    Eigen::VectorXd r(Ny); r << REF1, REF2;
    Eigen::MatrixXd Y(SIM_STEPS,2), U(SIM_STEPS,2);
    for (int k = 0; k < SIM_STEPS; ++k) {
        Eigen::VectorXd u = (-K*plant.x+Nbar*r).cwiseMax(-20.0).cwiseMin(20.0);
        U.row(k)=u.transpose(); Y.row(k)=plant.step(u).transpose();
    }
    std::cout << "  NM-LQR:   qx=" << std::setprecision(4) << xopt[0]
              << " qv=" << xopt[1] << " r=" << xopt[2] << "\n";
    return compute_metrics(Y, U, REF1, REF2);
}

// =========================================================================
// E. LQG - Bryson LQR + isotropic Kalman
// =========================================================================
static MIMOMetrics run_lqg() {
    MIMOStateSpace ref = make_plant();
    Eigen::VectorXd xmax(Nx); xmax << 1.0, 2.0, 1.0, 2.0;
    Eigen::MatrixXd Q_lqr = Eigen::MatrixXd::Zero(Nx,Nx);
    for (int i = 0; i < Nx; ++i) Q_lqr(i,i) = 1.0 / (xmax[i]*xmax[i]);
    Eigen::MatrixXd R_lqr = 0.01 * Eigen::MatrixXd::Identity(Nu, Nu);
    Eigen::MatrixXd K = lqr_gain(ref.A, ref.B, Q_lqr, R_lqr);

    // Kalman: Q_kf = process noise, R_kf = measurement noise
    Eigen::MatrixXd Q_kf = 1e-4 * Eigen::MatrixXd::Identity(Nx, Nx);
    Eigen::MatrixXd R_kf = 1e-3 * Eigen::MatrixXd::Identity(Ny, Ny);

    // Steady-state Kalman gain L via DARE on (A', C', Q_kf, R_kf)
    Eigen::MatrixXd P_kf = dare(ref.A.transpose(), ref.C.transpose(), Q_kf, R_kf);
    Eigen::MatrixXd S_kf = R_kf + ref.C * P_kf * ref.C.transpose();
    Eigen::MatrixXd L    = P_kf * ref.C.transpose() * S_kf.inverse();

    Eigen::MatrixXd A_cl = ref.A - ref.B * K;
    Eigen::MatrixXd Nbar = (ref.C*(-(A_cl-Eigen::MatrixXd::Identity(Nx,Nx))).inverse()*ref.B).inverse();
    Eigen::VectorXd r(Ny); r << REF1, REF2;

    MIMOStateSpace plant = make_plant();
    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(Nx);
    Eigen::MatrixXd Y(SIM_STEPS,2), U(SIM_STEPS,2);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu);

    for (int k = 0; k < SIM_STEPS; ++k) {
        Eigen::VectorXd y = plant.step(u);
        Y.row(k) = y.transpose(); U.row(k) = u.transpose();
        // KF update
        x_hat = ref.A * x_hat + ref.B * u + L * (y - ref.C * x_hat);
        u = (-K * x_hat + Nbar * r).cwiseMax(-20.0).cwiseMin(20.0);
    }
    std::cout << "  LQG:      Q_lqr=diag(Bryson), R_lqr=0.01*I, Q_kf=1e-4*I, R_kf=1e-3*I\n";
    return compute_metrics(Y, U, REF1, REF2);
}

// =========================================================================
// F. MIMO MPC - condensed QP (unconstrained closed-form)
// =========================================================================
static MIMOMetrics run_mpc(int Np = 20, int Nc = 10, double Qw = 1.0, double Rw = 0.01) {
    MIMOStateSpace ref = make_plant();
    Eigen::MatrixXd Phi  (Np*Ny, Nx);    Phi.setZero();
    Eigen::MatrixXd Theta(Np*Ny, Nc*Nu); Theta.setZero();
    Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(Nx, Nx);
    for (int i = 0; i < Np; ++i) {
        Ak = ref.A * Ak;
        Phi.block(i*Ny, 0, Ny, Nx) = ref.C * Ak;
        for (int j = 0; j < std::min(i+1, Nc); ++j) {
            Eigen::MatrixXd Akj = Eigen::MatrixXd::Identity(Nx, Nx);
            for (int s = 0; s < i-j; ++s) Akj = ref.A * Akj;
            Theta.block(i*Ny, j*Nu, Ny, Nu) = ref.C * Akj * ref.B;
        }
    }
    Eigen::MatrixXd Q_bar = Qw * Eigen::MatrixXd::Identity(Np*Ny, Np*Ny);
    Eigen::MatrixXd R_bar = Rw * Eigen::MatrixXd::Identity(Nc*Nu, Nc*Nu);
    Eigen::MatrixXd H     = Theta.transpose()*Q_bar*Theta + R_bar;
    Eigen::MatrixXd F     = H.ldlt().solve(Theta.transpose()*Q_bar*Phi);
    Eigen::MatrixXd G     = H.ldlt().solve(Theta.transpose()*Q_bar);
    Eigen::VectorXd r_bar = Eigen::VectorXd::Ones(Np*Ny);
    r_bar.head(Np) *= REF1;
    r_bar.tail(Np) *= REF2;

    MIMOStateSpace plant = make_plant();
    Eigen::MatrixXd Y(SIM_STEPS,2), U(SIM_STEPS,2);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu);
    for (int k = 0; k < SIM_STEPS; ++k) {
        Eigen::VectorXd y = plant.step(u); Y.row(k) = y.transpose(); U.row(k) = u.transpose();
        Eigen::VectorXd u_seq = -F*plant.x + G*r_bar;
        u = u_seq.head(Nu).cwiseMax(-20.0).cwiseMin(20.0);
    }
    return compute_metrics(Y, U, REF1, REF2);
}

// =========================================================================
// G. MIMO MPC - Nelder-Mead optimised Q/R weights
// =========================================================================
static MIMOMetrics run_mpc_opt() {
    auto cost = [](const std::vector<double>& p) {
        double Qw = p[0], Rw = p[1];
        if (Qw <= 0 || Rw <= 0) return 1e6;
        MIMOStateSpace ref = make_plant();
        int Np = 20, Nc = 10;
        Eigen::MatrixXd Phi(Np*Ny,Nx), Theta(Np*Ny,Nc*Nu);
        Phi.setZero(); Theta.setZero();
        Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(Nx,Nx);
        for (int i = 0; i < Np; ++i) {
            Ak = ref.A*Ak; Phi.block(i*Ny,0,Ny,Nx) = ref.C*Ak;
            for (int j = 0; j < std::min(i+1,Nc); ++j) {
                Eigen::MatrixXd Akj = Eigen::MatrixXd::Identity(Nx,Nx);
                for (int s = 0; s < i-j; ++s) Akj = ref.A*Akj;
                Theta.block(i*Ny,j*Nu,Ny,Nu) = ref.C*Akj*ref.B;
            }
        }
        Eigen::MatrixXd H  = Qw*(Theta.transpose()*Theta) + Rw*Eigen::MatrixXd::Identity(Nc*Nu,Nc*Nu);
        Eigen::MatrixXd F  = H.ldlt().solve(Qw*Theta.transpose()*Phi);
        Eigen::MatrixXd G  = H.ldlt().solve(Qw*Theta.transpose());
        Eigen::VectorXd rb = Eigen::VectorXd::Ones(Np*Ny);
        rb.head(Np) *= REF1; rb.tail(Np) *= REF2;
        MIMOStateSpace plant = make_plant();
        Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu);
        double J = 0.0;
        for (int k = 0; k < SIM_STEPS; ++k) {
            Eigen::VectorXd y = plant.step(u);
            double t=k*Ts, e1=REF1-y[0], e2=REF2-y[1];
            J += (e1*e1+e2*e2)*Ts + 0.1*t*(std::abs(e1)+std::abs(e2))*Ts + 0.01*(u[0]*u[0]+u[1]*u[1])*Ts;
            u = (-F*plant.x+G*rb).head(Nu).cwiseMax(-20.0).cwiseMin(20.0);
        }
        return J;
    };
    auto xopt = nelder_mead(cost, {1.0, 0.01}, {{0.001,100},{0.0001,10}}, 300);
    std::cout << "  NM-MPC:   Qw=" << std::setprecision(4) << xopt[0] << " Rw=" << xopt[1] << "\n";
    return run_mpc(20, 10, xopt[0], xopt[1]);
}

// =========================================================================
// Save results to CSV
// =========================================================================
static void save_csv(const std::string& path,
                     const std::vector<std::string>& labels,
                     const std::vector<MIMOMetrics>& results) {
    std::ofstream f(path);
    f << "method,ISE_y1,ISE_y2,ITAE_y1,ITAE_y2,E_u1,E_u2,OS_y1,OS_y2,J_total\n";
    for (size_t i = 0; i < labels.size(); ++i) {
        const auto& m = results[i];
        f << std::fixed << std::setprecision(6)
          << labels[i] << ","
          << m.ISE_y1  << "," << m.ISE_y2  << ","
          << m.ITAE_y1 << "," << m.ITAE_y2 << ","
          << m.E_u1    << "," << m.E_u2    << ","
          << m.OS_y1   << "," << m.OS_y2   << ","
          << m.total_cost() << "\n";
    }
    std::cout << "\n  Results saved -> " << path << "\n";
}

// =========================================================================
// main
// =========================================================================
int main() {
    std::cout << "=================================================================\n";
    std::cout << " MIMO Known Case - All Tuning Methods\n";
    std::cout << " Coupled Mass-Spring-Damper (2-mass, 2-input, 2-output)\n";
    std::cout << "=================================================================\n";

    MIMOStateSpace plant = make_plant();
    print_plant_info(plant);

    std::cout << "\n--- Optimised parameter values ---\n";
    std::vector<std::string>  labels;
    std::vector<MIMOMetrics>  results;

    auto run = [&](const std::string& lbl, auto fn) {
        std::cout << "\n[" << lbl << "]\n";
        auto m = fn();
        m.print(lbl);
        labels.push_back(lbl);
        results.push_back(m);

        // Fuzzy performance estimate
        double ise_worst = (labels.size()==1) ? m.total_cost() * 2.0 : results[0].total_cost()*2.0;
        double ise_norm  = std::min((m.ISE_y1+m.ISE_y2) / ise_worst, 1.0);
        double os_pct    = std::max(m.OS_y1, m.OS_y2);
        double st_norm   = 0.5;   // approximate (full settling analysis omitted for brevity)
        auto fr = fuzzy::evaluate(ise_norm, os_pct, st_norm);
        fuzzy::print_report(lbl, ise_norm, os_pct, st_norm, fr);
    };

    run("IMC-PID",    run_decentralized_pid_imc);
    run("NM-PID",     run_decentralized_pid_opt);
    run("LQR-Bryson", run_lqr_bryson);
    run("NM-LQR",     run_lqr_opt);
    run("LQG",        run_lqg);
    run("MPC",        [](){ return run_mpc(); });
    run("NM-MPC",     run_mpc_opt);

    // Summary table
    std::cout << "\n=== Performance Comparison Table ===\n";
    std::cout << std::setw(18) << "Method"
              << " | ISE1     ISE2     | ITAE1    ITAE2    | Eu1      Eu2      | J_total\n";
    std::cout << std::string(100, '-') << "\n";
    size_t best_idx = 0;
    for (size_t i = 0; i < results.size(); ++i) {
        results[i].print(labels[i]);
        if (results[i].total_cost() < results[best_idx].total_cost()) best_idx = i;
    }
    std::cout << "\n  * Best method (minimum J): " << labels[best_idx]
              << "  J=" << std::setprecision(4) << results[best_idx].total_cost() << "\n";

    // Cost breakdown analysis
    std::cout << "\n=== Cost Breakdown (J = ISE + 0.1*ITAE + 0.01*Energy) ===\n";
    for (size_t i = 0; i < results.size(); ++i) {
        double ise_part    = results[i].ISE_y1 + results[i].ISE_y2;
        double itae_part   = 0.1 * (results[i].ITAE_y1 + results[i].ITAE_y2);
        double energy_part = 0.01 * (results[i].E_u1 + results[i].E_u2);
        std::cout << std::setw(12) << labels[i]
                  << " : ISE=" << std::setw(8) << std::setprecision(4) << ise_part
                  << " ITAE=" << std::setw(8) << itae_part
                  << " Energy=" << std::setw(8) << energy_part << "\n";
    }

    // Save CSV
    std::string out_path = "mimo_known_results.csv";
    save_csv(out_path, labels, results);

    return 0;
}
