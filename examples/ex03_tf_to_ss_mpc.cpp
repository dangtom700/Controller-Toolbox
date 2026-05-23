// ============================================================
//  ex03_tf_to_ss_mpc.cpp
//  3rd-order plant defined as a Transfer Function,
//  converted to State Space via tf2ss(), then controlled with MPC.
//  MPCHorizonTuner recommends prediction / control horizons.
//
//  MATLAB equivalent:
//    G_discrete = tf([0 0 0 0.006], [1 -2.4 1.91 -0.504], 0.05, 'Variable', 'z^-1');
//    [A,B,C,D] = ssdata(G_discrete);
//    mpcobj = mpc(ss(A,B,C,D,0.05), 0.05, 10, 3);
//    sim(mpcobj, 10);
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    // ---- TF: G(z^-1) = 0.006 z^-3 / (1 - 2.4z^-1 + 1.91z^-2 - 0.504z^-3) ----
    const double Ts = 0.05;
    ctrl::TransferFunction plant_tf(
        {0.0, 0.0, 0.0, 0.006},       // numerator
        {1.0, -2.4, 1.91, -0.504},    // denominator (monic)
        Ts);

    // ---- TF -> SS conversion (controllable canonical form) ----
    ctrl::StateSpace plant = ctrl::tf2ss(plant_tf);

    std::cout << "3rd-order plant (tf2ss):\n"
              << "A =\n"
              << plant.A << "\nB =\n"
              << plant.B
              << "\nC =\n"
              << plant.C << "\n\n";

    // ---- MPC horizon recommendation ----
    auto rec = ctrl::MPCHorizonTuner::recommend(plant, Ts);
    std::cout << "MPC recommendation:  Np=" << rec.Np
              << "  Nc=" << rec.Nc
              << "  t_settle approx =" << std::fixed << std::setprecision(2)
              << rec.estimatedSettlingTime << "s\n\n";

    ctrl::MPCParams mp;
    mp.Np = rec.Np;
    mp.Nc = rec.Nc;
    mp.rho_y = 1.0;
    mp.rho_u = 0.05;
    mp.uMin = -5.0;
    mp.uMax = 5.0;

    ctrl::DiscreteMPC mpc(plant, mp);

    // ---- Closed-loop simulation ----
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    const double ref = 1.0;
    const int N = static_cast<int>(10.0 / Ts);

    std::cout << std::setw(6) << "k" << std::setw(8) << "t[s]"
              << std::setw(10) << "y" << std::setw(10) << "error"
              << std::setw(10) << "u" << "\n";
    std::cout << std::string(44, '-') << "\n";

    for (int k = 0; k <= N; ++k)
    {
        const double e = ref - y;
        const double u = mpc.compute(e);

        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);

        if (k % (N / 10) == 0)
            std::cout << std::setw(6) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(10) << y
                      << std::setw(10) << e
                      << std::setw(10) << u << "\n";
    }
    return 0;
}
