/**
 * @file ex113_gp_mpc.cpp
 * @brief Phase 3 (ML3): GP-uncertainty-aware tightening of NonlinearMPC's input bounds.
 *
 * Compares plain NonlinearMPC against GPMPC on the same scalar plant: with an unfitted GP,
 * GPMPC behaves identically (regression); with a GP trained far from the operating point
 * (high posterior variance there), GPMPC visibly tightens its input bounds.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xn(1);
        xn(0) = 0.9 * x(0) + u(0);
        return xn;
    };

    ctrl::GPMPCParams params;
    params.nmpc.Np = 5; params.nmpc.Nu = 3;
    params.nmpc.n_states = 1; params.nmpc.n_inputs = 1; params.nmpc.n_outputs = 1;
    params.nmpc.uMin = -5.0; params.nmpc.uMax = 5.0; params.nmpc.Ts = 0.1;

    ctrl::GPResidualModel::Params gp_p;
    gp_p.gp.length_scale = 0.5; gp_p.gp.signal_var = 1.0; gp_p.gp.noise_var = 0.01;

    std::cout << "=== GPMPC: unfitted GP (regression check) ===\n";
    {
        auto gp = std::make_shared<ctrl::GPResidualModel>(2, gp_p);
        ctrl::NonlinearMPC nmpc(params.nmpc, f);
        ctrl::GPMPC gpmpc(params, f, gp);

        Eigen::VectorXd x(1); x << 1.0;
        bool ok = true;
        for (int k = 0; k < 5; ++k)
        {
            nmpc.setState(x);
            gpmpc.setState(x);
            const double u_nmpc  = nmpc.compute(0.5 - x(0));
            const double u_gpmpc = gpmpc.compute(0.5 - x(0));
            ok = ok && (std::abs(u_nmpc - u_gpmpc) < 1e-9);
            Eigen::VectorXd u_vec(1); u_vec << u_nmpc;
            x = f(x, u_vec);
        }
        std::cout << "NonlinearMPC == GPMPC(unfitted): " << (ok ? "yes" : "NO (bug)") << "\n";
        std::cout << "max tightening: " << gpmpc.lastTightening().maxCoeff() << " (expect 0)\n\n";
        if (!ok) return 1;
    }

    std::cout << "=== GPMPC: GP trained far from x=1.0 (high local variance) ===\n";
    {
        auto gp = std::make_shared<ctrl::GPResidualModel>(2, gp_p);
        gp->addResidualPoint(Eigen::Vector2d(50.0, 50.0), 0.0, 0.0);
        gp->fit();
        ctrl::GPMPC gpmpc(params, f, gp);

        Eigen::VectorXd x(1); x << 1.0;
        gpmpc.setState(x);
        const double u = gpmpc.compute(0.5 - x(0));
        std::cout << "u = " << u << "  max tightening = " << gpmpc.lastTightening().maxCoeff() << "\n";

        const bool ok = std::isfinite(u) && gpmpc.lastTightening().maxCoeff() > 0.0;
        std::cout << "\n" << (ok ? "PASS" : "FAIL") << "\n";
        return ok ? 0 : 1;
    }
}
