/*
 * mimo_plant.h - Coupled 2-Mass-Spring-Damper MIMO Plant
 * =======================================================
 * Physical system: Two masses on a frictionless surface, each attached to a
 * wall by a spring and damper, and coupled together by a coupling spring and
 * damper. Forces F1 and F2 are the actuator inputs; outputs are positions x1
 * and x2.
 *
 * Parameters:
 *   m1 = m2 = 1 kg
 *   k1 = k2 = 4 N/m   (wall springs)
 *   kc = 3 N/m         (coupling spring - gives RGA approx = [1.22, -0.22; -0.22, 1.22])
 *   c1 = c2 = 0.5 Ns/m (wall dampers)
 *   cc = 0.5 Ns/m      (coupling damper)
 *
 * Continuous-time equations of motion:
 *   m1*ẍ1 = -(k1+kc)*x1 - (c1+cc)*ẋ1 + kc*x2 + cc*ẋ2 + F1
 *   m2*ẍ2 =  kc*x1 + cc*ẋ1 - (k2+kc)*x2 - (c2+cc)*ẋ2 + F2
 *
 * State: q = [x1, ẋ1, x2, ẋ2]'   Input: u = [F1, F2]'   Output: y = [x1, x2]'
 *
 *          [  0,    1,    0,    0 ]              [ 0, 0 ]
 * A_ct  =  [ -7,   -1,    3,   0.5]   B_ct  =  [ 1, 0 ]
 *          [  0,    0,    0,    1 ]             [ 0, 0 ]
 *          [  3,   0.5,  -7,  -1  ]             [ 0, 1 ]
 *
 * C = [1, 0, 0, 0;  0, 0, 1, 0]     D = 0
 *
 * Continuous eigenvalues: -0.25 +/- 1.98j (slow), -0.75 +/- 3.07j (fast)
 * DC gain: G(0) = C*(-A_ct)^{-1}*B_ct approx = [0.175, 0.075; 0.075, 0.175]
 *
 * ZOH discretization at Ts = 0.01 s via matrix exponential of the augmented
 * system [A, B; 0, 0] * Ts (see zoh() below).
 */

#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace mimo {

constexpr double Ts = 0.01;   // sampling period [s]
constexpr int    Nx = 4;      // state dimension
constexpr int    Nu = 2;      // input dimension
constexpr int    Ny = 2;      // output dimension

// -------------------------------------------------------------------------
// Matrix exponential via 20-term Taylor series.
// Convergence is assured when ||M||_F << 1 (here ||A_ct||*Ts approx = 0.1).
// -------------------------------------------------------------------------
inline Eigen::MatrixXd matexp(const Eigen::MatrixXd& M, int order = 20) {
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(M.rows(), M.cols());
    Eigen::MatrixXd T = R;
    for (int k = 1; k <= order; ++k) {
        T = T * M / double(k);
        R += T;
        if (T.norm() < 1e-15 * R.norm()) break;
    }
    return R;
}

// -------------------------------------------------------------------------
// Zero-order hold discretization: Ad, Bd <- (Ac, Bc, Ts)
// Uses the augmented matrix exponential:
//   exp([Ac, Bc; 0, 0] * Ts) = [Ad, Bd; 0, I]
// -------------------------------------------------------------------------
inline void zoh(const Eigen::MatrixXd& Ac, const Eigen::MatrixXd& Bc, double ts,
                Eigen::MatrixXd& Ad, Eigen::MatrixXd& Bd) {
    int n = Ac.rows(), m = Bc.cols();
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n + m, n + m);
    M.topLeftCorner(n, n)  = Ac * ts;
    M.topRightCorner(n, m) = Bc * ts;
    Eigen::MatrixXd eM = matexp(M);
    Ad = eM.topLeftCorner(n, n);
    Bd = eM.topRightCorner(n, m);
}

// -------------------------------------------------------------------------
// MIMO state-space struct (multiple inputs, multiple outputs)
// Convention: output FIRST, state update SECOND (matches lib/StateSpace.h)
// -------------------------------------------------------------------------
struct MIMOStateSpace {
    Eigen::MatrixXd A, B, C, D;
    Eigen::VectorXd x;

    MIMOStateSpace(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_,
                   const Eigen::MatrixXd& C_, const Eigen::MatrixXd& D_)
        : A(A_), B(B_), C(C_), D(D_), x(Eigen::VectorXd::Zero(A_.rows())) {}

    // Output-before-update to match SISO ssStep convention
    Eigen::VectorXd step(const Eigen::VectorXd& u) {
        Eigen::VectorXd y = C * x + D * u;
        x = A * x + B * u;
        return y;
    }

    void reset() { x.setZero(); }
};

// -------------------------------------------------------------------------
// Build the canonical CMSD-2 plant (discrete, Ts = 0.01 s)
// -------------------------------------------------------------------------
inline MIMOStateSpace make_plant() {
    Eigen::MatrixXd Ac(Nx, Nx), Bc(Nx, Nu);
    Ac << 0,    1,    0,    0,
         -7,   -1,    3,    0.5,
          0,    0,    0,    1,
          3,    0.5, -7,   -1;

    Bc << 0, 0,
          1, 0,
          0, 0,
          0, 1;

    Eigen::MatrixXd C(Ny, Nx), D(Ny, Nu);
    C << 1, 0, 0, 0,
         0, 0, 1, 0;
    D.setZero();

    Eigen::MatrixXd Ad, Bd;
    zoh(Ac, Bc, Ts, Ad, Bd);
    return MIMOStateSpace(Ad, Bd, C, D);
}

// -------------------------------------------------------------------------
// Print DC gain matrix and Relative Gain Array (RGA) for pairing analysis
// -------------------------------------------------------------------------
inline void print_plant_info(const MIMOStateSpace& plant) {
    // DC gain: G(0) = C*(I-A)^{-1}*B  (discrete-time formula)
    Eigen::MatrixXd IminA = Eigen::MatrixXd::Identity(Nx, Nx) - plant.A;
    Eigen::MatrixXd Gdc   = plant.C * IminA.lu().solve(plant.B);

    // RGA = G_dc .* (G_dc^{-T})  element-wise
    Eigen::MatrixXd Gdc_inv = Gdc.inverse();
    Eigen::MatrixXd RGA     = Gdc.array() * Gdc_inv.transpose().array();

    std::cout << "\n=== MIMO Plant: Coupled Mass-Spring-Damper (2-mass) ===\n";
    std::cout << "  State: [x1, ẋ1, x2, ẋ2]'   Input: [F1, F2]'   Output: [x1, x2]'\n";
    std::cout << "  Ts = " << Ts << " s   (ZOH discretization)\n\n";
    std::cout << "  DC Gain G(1) =\n" << std::fixed << std::setprecision(5) << Gdc << "\n\n";
    std::cout << "  Relative Gain Array (RGA) =\n" << RGA << "\n";
    std::cout << "  RGA note: diagonal > 1 -> coupling exists; "
              << "pair (u1->y1) and (u2->y2) is recommended.\n\n";
}

// -------------------------------------------------------------------------
// DARE via value iteration (supports MIMO: B is n*m, R is m*m)
// Solves: P = Q + A'PA - A'PB(R + B'PB)^{-1}B'PA
// -------------------------------------------------------------------------
inline Eigen::MatrixXd dare(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                            int maxIter = 2000, double tol = 1e-12) {
    Eigen::MatrixXd P = Q;
    for (int i = 0; i < maxIter; ++i) {
        Eigen::MatrixXd S    = R + B.transpose() * P * B;
        Eigen::MatrixXd K    = S.ldlt().solve(B.transpose() * P * A);
        Eigen::MatrixXd Pnew = Q + A.transpose() * P * A - A.transpose() * P * B * K;
        double delta = (Pnew - P).norm();
        P = Pnew;
        if (delta < tol * (1.0 + P.norm())) break;
    }
    return P;
}

// -------------------------------------------------------------------------
// LQR gain from solved DARE: K = (R + B'PB)^{-1} B'PA
// -------------------------------------------------------------------------
inline Eigen::MatrixXd lqr_gain(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    Eigen::MatrixXd P = dare(A, B, Q, R);
    Eigen::MatrixXd S = R + B.transpose() * P * B;
    return S.ldlt().solve(B.transpose() * P * A);
}

// -------------------------------------------------------------------------
// Performance metrics for a MIMO simulation run
// -------------------------------------------------------------------------
struct MIMOMetrics {
    double ISE_y1, ISE_y2, ITAE_y1, ITAE_y2;
    double E_u1, E_u2;   // control energy \intu^2dt
    double OS_y1, OS_y2; // overshoot [%]

    double total_cost(double w_ise  = 1.0,
                      double w_itae = 0.1,
                      double w_energy = 0.01) const {
        return w_ise * (ISE_y1 + ISE_y2)
             + w_itae * (ITAE_y1 + ITAE_y2)
             + w_energy * (E_u1 + E_u2);
    }

    void print(const std::string& label) const {
        std::cout << std::setw(18) << label
                  << " | ISE1=" << std::setw(8) << std::fixed << std::setprecision(5) << ISE_y1
                  << " ISE2=" << std::setw(8) << ISE_y2
                  << " | ITAE1=" << std::setw(8) << ITAE_y1
                  << " ITAE2=" << std::setw(8) << ITAE_y2
                  << " | Eu1=" << std::setw(8) << E_u1
                  << " Eu2=" << std::setw(8) << E_u2
                  << " | OS1=" << std::setw(6) << std::setprecision(1) << OS_y1
                  << "% OS2=" << std::setw(6) << OS_y2 << "%"
                  << " | J=" << std::setw(9) << std::setprecision(4) << total_cost()
                  << "\n";
    }
};

// -------------------------------------------------------------------------
// Compute performance metrics from y1, y2, u1, u2 trajectories
// ref1, ref2 = reference setpoints (assumed constant step)
// -------------------------------------------------------------------------
inline MIMOMetrics compute_metrics(const Eigen::MatrixXd& Y,  // STEPS * 2
                                   const Eigen::MatrixXd& U,  // STEPS * 2
                                   double ref1, double ref2) {
    int N = (int)Y.rows();
    MIMOMetrics m{};
    for (int k = 0; k < N; ++k) {
        double t  = k * Ts;
        double e1 = ref1 - Y(k, 0);
        double e2 = ref2 - Y(k, 1);
        m.ISE_y1  += e1 * e1 * Ts;
        m.ISE_y2  += e2 * e2 * Ts;
        m.ITAE_y1 += t * std::abs(e1) * Ts;
        m.ITAE_y2 += t * std::abs(e2) * Ts;
        m.E_u1    += U(k, 0) * U(k, 0) * Ts;
        m.E_u2    += U(k, 1) * U(k, 1) * Ts;
    }
    double y1max = Y.col(0).maxCoeff();
    double y2max = Y.col(1).maxCoeff();
    m.OS_y1 = std::max(0.0, (y1max - ref1) / std::abs(ref1) * 100.0);
    m.OS_y2 = std::max(0.0, (y2max - ref2) / std::abs(ref2) * 100.0);
    return m;
}

} // namespace mimo
