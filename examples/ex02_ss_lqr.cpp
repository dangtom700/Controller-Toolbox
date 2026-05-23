// ============================================================
//  ex02_ss_lqr.cpp
//  Plant defined directly as a State-Space model (DC motor).
//  LQR designed with Bryson's method, full state feedback.
//
//  DC motor (continuous):
//    J.domega/dt = -b.omega + Km.i
//    L.di/dt = -Km.omega - R.i + V
//    y = omega  (angular velocity)
//
//  Discretised with Euler forward at Ts = 0.001 s.
//
//  MATLAB equivalent:
//    A = [-b/J  Km/J; -Km/L  -R/L];  B = [0; 1/L];
//    C = [1 0];  D = 0;
//    sys = c2d(ss(A,B,C,D), 0.001, 'zoh');
//    Q = diag([1/100^2, 1/5^2]);  R = 1/12^2;
//    K = dlqr(sys.A, sys.B, Q, R);
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    // ---- DC motor parameters ----
    const double J = 0.001;  // rotor inertia  [kg.m^2]
    const double b = 0.1;    // viscous friction [N.m.s/rad]
    const double Km = 0.01;  // motor / back-EMF constant
    const double R = 1.0;    // armature resistance [Ω]
    const double L = 0.5;    // armature inductance [H]
    const double Ts = 0.001; // sample time [s]

    // ---- Continuous A, B; discretise with Euler forward ----
    Eigen::Matrix2d Ac;
    Ac << -b / J, Km / J,
        -Km / L, -R / L;
    Eigen::Vector2d Bc;
    Bc << 0.0, 1.0 / L;
    Eigen::RowVector2d Cc;
    Cc << 1.0, 0.0;
    Eigen::MatrixXd Dc(1, 1);
    Dc << 0.0;

    const Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity() + Ts * Ac;
    const Eigen::Vector2d Bd = Ts * Bc;

    ctrl::StateSpace plant(Ad, Bd, Cc, Dc, Ts);
    std::cout << "Motor state-space (discrete, Ts=" << Ts << "):\n"
              << "A =\n"
              << plant.A << "\nB =\n"
              << plant.B
              << "\nC =\n"
              << plant.C << "\n\n";

    // ---- Bryson LQR weights ----
    // Max acceptable velocity = 100 rad/s, max current = 5 A, max voltage = 12 V
    Eigen::Vector2d xmax;
    xmax << 100.0, 5.0;
    Eigen::VectorXd umax(1);
    umax << 12.0;
    ctrl::LQRParams lqr_p = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);

    std::cout << "LQR Q =\n"
              << lqr_p.Q << "\nLQR R =\n"
              << lqr_p.R << "\n\n";

    ctrl::DiscreteLQR lqr(plant, lqr_p);
    std::cout << "Gain K = " << lqr.gainMatrix() << "\n\n";

    // ---- Reference state: omega_ref = 50 rad/s, i_ref = 0 ----
    Eigen::Vector2d x_ref;
    x_ref << 50.0, 0.0;

    // ---- Simulation ----
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());

    std::cout << std::setw(6) << "k" << std::setw(8) << "t[s]"
              << std::setw(12) << "omega[rad/s]" << std::setw(10) << "i[A]"
              << std::setw(10) << "V[V]" << "\n";
    std::cout << std::string(46, '-') << "\n";

    for (int k = 0; k <= 1000; ++k)
    {
        Eigen::VectorXd u = lqr.compute(x, x_ref);
        // Saturate voltage to +/-12 V
        u(0) = std::max(-12.0, std::min(12.0, u(0)));

        ctrl::ssStep(plant, x, u); // x updated in-place to x[k+1]

        if (k % 100 == 0)
            std::cout << std::setw(6) << k
                      << std::fixed << std::setprecision(4)
                      << std::setw(8) << k * Ts
                      << std::setw(12) << x(0)
                      << std::setw(10) << x(1)
                      << std::setw(10) << u(0) << "\n";
    }
    return 0;
}
