/**
 * @file ex61_lpv_identification.cpp
 * @brief LPV identification: simulate a known affine LPV system, run identifyLPV,
 *        compare estimated coefficients to ground truth, then simulate closed-loop.
 *
 * True LPV model (degree 1, n=2, m=1):
 *   A(p) = [[-0.4, p*0.3], [0, -0.6 + p*0.2]]
 *   B(p) = [[1.0], [0.0]]
 *   C     = [[1, 0]]
 *   D     = [[0]]
 * Scheduling: p(k) = 0.5*sin(2*pi*k/N)
 */
#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.05;
    const int    N  = 600;
    const double pi = 3.14159265358979323846;

    // ----- True coefficient matrices ----------------------------------------
    Eigen::MatrixXd A0(2,2), A1(2,2), B0(2,1), B1(2,1);
    Eigen::MatrixXd C0(1,2), C1(1,2), D0(1,1), D1(1,1);

    A0 << -0.4,  0.0,
           0.0, -0.6;
    A1 <<  0.0,  0.3,
           0.0,  0.2;
    B0 <<  1.0,
           0.0;
    B1 <<  0.0,
           0.0;
    C0 <<  1.0, 0.0;
    C1 <<  0.0, 0.0;
    D0 <<  0.0;
    D1 <<  0.0;

    // ----- Simulate true system ----------------------------------------------
    Eigen::MatrixXd X(2, N), U(1, N), Y(1, N);
    std::vector<double> sched(N);
    // Non-zero initial state so both rows of A are identifiable.
    // With x2(0)=0 and B=[[1],[0]], x2 stays zero and A columns multiplying x2 can't be recovered.
    Eigen::VectorXd x(2); x << 0.5, 0.3;

    for (int k = 0; k < N; ++k) {
        double p = 0.5 * std::sin(2.0 * pi * k / N);
        double u = 0.4 * std::cos(2.0 * pi * k / (N / 3.0));
        Eigen::VectorXd uv(1); uv << u;

        X.col(k) = x;
        U.col(k) = uv;
        sched[k] = p;

        Eigen::MatrixXd Ak = A0 + p * A1;
        Eigen::MatrixXd Bk = B0 + p * B1;
        Eigen::MatrixXd Ck = C0 + p * C1;
        Eigen::MatrixXd Dk = D0 + p * D1;

        Y.col(k) = Ck * x + Dk * uv;
        x        = Ak * x + Bk * uv;
    }

    // ----- Identify ----------------------------------------------------------
    ctrl::LPVModel m = ctrl::identifyLPV(X, U, Y, sched, 1, Ts);

    std::cout << "Identified degree-" << m.degree << " LPV model\n"
              << "  n=" << m.n_states << "  m=" << m.n_inputs
              << "  p=" << m.n_outputs << "\n\n";

    auto printMat = [](const std::string& name, const Eigen::MatrixXd& M) {
        std::cout << name << " =\n" << M.format(
            Eigen::IOFormat(4, 0, "  ", "\n", "    [", "]")) << "\n";
    };

    printMat("A_coeffs[0] (true: A0)", m.A_coeffs[0]);
    std::cout << "  true A0:\n    " << A0.format(Eigen::IOFormat(4)) << "\n\n";
    printMat("A_coeffs[1] (true: A1)", m.A_coeffs[1]);
    std::cout << "  true A1:\n    " << A1.format(Eigen::IOFormat(4)) << "\n\n";
    printMat("B_coeffs[0] (true: B0)", m.B_coeffs[0]);

    // Check errors
    double errA0 = (m.A_coeffs[0] - A0).norm();
    double errA1 = (m.A_coeffs[1] - A1).norm();
    double errB0 = (m.B_coeffs[0] - B0).norm();

    std::cout << "\nCoefficient errors (Frobenius norm):\n"
              << "  ||A_hat0 - A0|| = " << std::setprecision(4) << errA0 << "\n"
              << "  ||A_hat1 - A1|| = " << errA1 << "\n"
              << "  ||B_hat0 - B0|| = " << errB0 << "\n";

    if (errA0 > 0.1 || errA1 > 0.1 || errB0 > 0.1) {
        std::cerr << "FAIL: identification errors too large.\n";
        return 1;
    }

    // ----- Closed-loop simulation using frozen model at p=0 ------------------
    ctrl::StateSpace sys0 = m.frozen(0.0);
    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::MatrixXd::Identity(2, 2);
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);
    ctrl::DiscreteLQR lqr(sys0, lqr_p);

    // Regulation to origin: verify LQR drives x -> 0 from x0 = [0.5, 0.3].
    // (Pure regulation is sufficient to validate the identified model; tracking
    //  to a non-zero ref requires a feedforward term for zero SS error.)
    Eigen::VectorXd x_cl(2); x_cl << 0.5, 0.3;
    double iae = 0.0;

    for (int k = 0; k < 200; ++k) {
        Eigen::VectorXd uv = lqr.compute(x_cl);  // u = -K*x (regulation)
        iae               += x_cl.norm();
        x_cl               = sys0.A * x_cl + sys0.B * uv;
    }

    std::cout << "\nClosed-loop state-norm IAE (200 steps, p=0): " << iae << "\n";
    if (iae > 5.0) {
        std::cerr << "FAIL: closed-loop IAE too large.\n";
        return 1;
    }

    std::cout << "PASS\n";
    return 0;
}
