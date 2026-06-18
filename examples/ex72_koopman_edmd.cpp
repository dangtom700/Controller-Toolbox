/**
 * @file ex72_koopman_edmd.cpp
 * @brief Koopman/EDMD - lift nonlinear system to a linear state-space.
 *
 * True plant: Van der Pol oscillator (mu=0.3, forced).
 * 1. Collect offline I/O data.
 * 2. Fit a PolyDeg2 Koopman model -> get a ctrl::StateSpace.
 * 3. Design DiscreteLQR on the projected linear model.
 * 4. Close the loop and measure tracking performance.
 */

#include "ControllerToolbox.h"
#include "KoopmanEDMD.h"
#include <cmath>
#include <cstdio>

// Van der Pol (Euler)
static Eigen::Vector2d vdp(const Eigen::Vector2d& x, double u, double dt,
                            double mu = 0.3)
{
    Eigen::Vector2d d;
    d(0) = x(1);
    d(1) = mu * (1.0 - x(0)*x(0)) * x(1) - x(0) + u;
    return x + dt * d;
}

int main()
{
    std::printf("=== ex72: Koopman/EDMD on Van der Pol ===\n\n");

    constexpr double Ts = 0.05;

    // 1. Data collection
    ctrl::KoopmanEDMD::Params kp;
    kp.n_state  = 2;
    kp.n_input  = 1;
    kp.dict     = ctrl::KoopmanDict::PolyDeg2;
    kp.tikhonov = 1e-4;
    ctrl::KoopmanEDMD edmd(kp);

    Eigen::Vector2d x; x << 0.5, -0.3;
    for (int k = 0; k < 2000; ++k) {
        double u = 0.5 * std::sin(0.2 * k * Ts) + 0.3 * ((k % 5 == 0) ? 1.0 : -0.5);
        Eigen::VectorXd xv = x, uv(1); uv(0) = u;
        Eigen::Vector2d xn = vdp(x, u, Ts);
        edmd.addSnapshot(xv, uv, xn.cast<double>());
        x = xn;
    }
    std::printf("Snapshots: %d,  lifted dim: %d\n", edmd.snapshotCount(), edmd.nLifted());

    // 2. Fit projected model  (n_state x n_state state-space)
    ctrl::StateSpace ss_proj = edmd.fitProjected();
    std::printf("Projected SS: A size %ldx%ld, B size %ldx%ld\n",
                (long)ss_proj.A.rows(), (long)ss_proj.A.cols(),
                (long)ss_proj.B.rows(), (long)ss_proj.B.cols());

    // 3. LQR on the linear model
    ctrl::LQRParams lp;
    lp.Q = Eigen::Vector2d(10.0, 1.0).asDiagonal();
    lp.R = Eigen::Matrix<double,1,1>::Identity();
    ctrl::DiscreteLQR lqr(ss_proj, lp);
    std::printf("LQR gain: [%.4f  %.4f]\n",
                lqr.gainMatrix()(0, 0), lqr.gainMatrix()(0, 1));

    // 4. Closed-loop tracking: regulate to origin from x0=[1, 0]
    x << 1.0, 0.0;
    double iae = 0.0;
    for (int k = 0; k < 200; ++k) {
        Eigen::VectorXd xv = x.cast<double>();
        Eigen::VectorXd u_lqr = lqr.compute(xv, Eigen::VectorXd::Zero(2));
        double u = std::clamp(u_lqr(0), -2.0, 2.0);
        iae += x.norm() * Ts;
        x = vdp(x, u, Ts);
    }
    std::printf("Regulation IAE: %.4f (lower = better tracking)\n", iae);
    std::printf("Final state: [%.4f  %.4f]\n", x(0), x(1));

    return 0;
}
