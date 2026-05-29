/**
 * ex52_dob_pi.cpp
 * OBSERVER + STATE FEEDBACK: Disturbance Observer (DOB) + PI controller.
 *
 * A Disturbance Observer inverts the nominal plant model to estimate the
 * total disturbance (external + model error) acting on the system.
 * The PI controller provides setpoint tracking; the DOB feedforwards the
 * estimated disturbance to cancel it before the PI sees it.
 *
 * DOB structure:
 *   d_hat = Q(z) * [y - G_nom(z) * u]   (inverse filter method, stable Q)
 *   u_total = u_PI - d_hat / G_nom_DC    (feedforward cancellation)
 *
 * For simplicity: Q(z) is a low-pass filter with cutoff omega_q [rad/s].
 * G_nom_DC = DC gain of the nominal plant (G_nom(0)).
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.05;

    // Nominal plant: G_nom(s) = 1/(s+1), ZOH-discretised
    ctrl::StateSpace sys_nom_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys_nom = ctrl::c2d(sys_nom_c, Ts, ctrl::C2dMethod::ZOH);

    // True plant: G_true(s) = 1.5/(s+0.8) (mismatched gain + pole)
    const double K_true = 1.5, tau_true = 1.0 / 0.8;
    const double a_true = std::exp(-Ts / tau_true);

    // PI controller (primary)
    ctrl::PIDParams pp;
    pp.Kp = 1.5; pp.Ki = 0.8; pp.Kd = 0.0;
    pp.uMin = -10.0; pp.uMax = 10.0;
    ctrl::DiscretePID pi(pp, Ts);

    // DOB: Q-filter as 1st-order low-pass, omega_q = 5 rad/s (stable, causal)
    // Q(z) via Tustin: Q(s) = omega_q / (s + omega_q)
    const double omega_q = 5.0;
    const double q_a     = std::exp(-omega_q * Ts);  // ZOH pole
    const double q_b     = 1.0 - q_a;               // ZOH gain

    // Nominal plant inverse approximation: G_nom_DC = 1 (DC gain of G_nom(s)=1/(s+1))
    const double G_nom_DC = 1.0;

    // DOB state: internal state of Q filter applied to [y - G_nom*u]
    double x_q = 0.0;  // Q-filter state
    double x_nom = 0.0; // nominal plant state (simulate G_nom(z)*u)

    double y_true = 0.0;  // true plant output
    double u_prev = 0.0;
    const double ref = 1.0;
    const double step_disturbance = 0.5;  // output disturbance at k=100
    const int N = 600;
    double final_y = 0.0;

    for (int k = 0; k < N; ++k) {
        const double d = (k >= 100) ? step_disturbance : 0.0;

        // PI controller output
        const double u_pi = pi.compute(ref - y_true);

        // Simulate nominal plant G_nom(z) * u_pi
        const double y_nom = sys_nom.C(0,0) * x_nom + sys_nom.D(0,0) * u_pi;
        x_nom = sys_nom.A(0,0) * x_nom + sys_nom.B(0,0) * u_pi;

        // DOB: disturbance estimate via Q-filter on (y - y_nom)
        const double innov = y_true - y_nom;
        const double d_hat = q_b * innov + q_a * x_q; // Q * innov (1st-order LP)
        // Actually: x_q stores the Q-filtered value
        // Q-filter: d_hat_next = q_a*d_hat_prev + q_b*innov
        x_q = q_a * x_q + q_b * innov;
        const double d_hat_used = x_q / G_nom_DC;

        // DOB feedforward cancellation
        const double u_total = u_pi - d_hat_used;

        // True plant step: y+ = a_true*y + (1-a_true)*K_true*u + d
        y_true = a_true * y_true + (1.0 - a_true) * K_true * u_total + d;
        final_y = y_true;
        u_prev = u_total;
    }

    std::cout << "DOB+PI: final y = " << final_y << "  (ref = " << ref << ")\n";

    if (std::abs(final_y - ref) < 0.1 && std::isfinite(final_y))
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
