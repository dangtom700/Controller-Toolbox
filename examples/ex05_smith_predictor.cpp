// ============================================================
//  ex05_smith_predictor.cpp
//  Plant with 8-step pure dead-time.
//  Compares: plain PID  vs  Smith Predictor + PID.
//
//  MATLAB equivalent:
//    G_nodelay = tf([1],[1 0.5]);
//    d_steps   = 8;   Ts = 0.1;
//    Gd = c2d(G_nodelay, Ts, 'zoh') * tf(1,1,Ts,'InputDelay',d_steps-1);
//    % Design PID on delay-free model, then compare with/without Smith
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <deque>

// Simulate SISO plant with integer dead-time using a shift register
struct DelayedPlant {
    ctrl::StateSpace      model;  // delay-free dynamics
    int                   d;
    Eigen::VectorXd       x;
    std::deque<double>    ubuf; // ring buffer of past inputs

    DelayedPlant(ctrl::StateSpace m, int delay)
        : model(m), d(delay), x(Eigen::VectorXd::Zero(m.stateSize()))
    { ubuf.assign(d, 0.0); }

    double step(double u) {
        // Apply the d-step-delayed input to the plant
        double u_delayed = ubuf.front();
        ubuf.pop_front(); ubuf.push_back(u);

        Eigen::VectorXd uv(1); uv << u_delayed;
        return ctrl::ssStep(model, x, uv)(0);
    }

    void reset() { x.setZero(); ubuf.assign(d, 0.0); }
};

int main()
{
    const double Ts = 0.1;
    const int    d  = 8;   // 8-sample = 0.8 s dead-time

    // Delay-free plant G(s) = 1/(s+0.5) -> ZOH at Ts=0.1:
    // G(z) = 0.3935 / (z - 0.9512)  ->  num={0,0.3935}, den={1,-0.9512}
    ctrl::TransferFunction tf_nodelay({ 0.0, 0.3935 }, { 1.0, -0.9512 }, Ts);
    ctrl::StateSpace       ss_nodelay = ctrl::tf2ss(tf_nodelay);

    // Fixed PID tuned on the delay-free model (Ziegler-Nichols from relay test)
    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 0.8; pp.Kd = 0.5;
    pp.N = 10.0; pp.uMin = -10.0; pp.uMax = 10.0;

    // ---- Run 1: plain PID ----
    {
        DelayedPlant   plant(ss_nodelay, d);
        ctrl::DiscretePID pid(pp, Ts);
        double y = 0.0, ref = 1.0;

        std::cout << "=== Plain PID with dead-time ===\n";
        std::cout << std::setw(6)  << "k" << std::setw(8)  << "t[s]"
                  << std::setw(10) << "y" << std::setw(8)   << "u" << "\n";

        for (int k = 0; k <= 200; ++k) {
            double u = pid.compute(ref - y);
            y = plant.step(u);
            if (k % 20 == 0)
                std::cout << std::setw(6) << k
                          << std::fixed << std::setprecision(3)
                          << std::setw(8)  << k * Ts
                          << std::setw(10) << y
                          << std::setw(8)  << u << "\n";
        }
    }

    // ---- Run 2: Smith Predictor + same PID ----
    {
        DelayedPlant plant(ss_nodelay, d);
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::SmithPredictor sp(inner, ss_nodelay, d);
        double y = 0.0, ref = 1.0;

        std::cout << "\n=== Smith Predictor + PID ===\n";
        std::cout << std::setw(6)  << "k" << std::setw(8)  << "t[s]"
                  << std::setw(10) << "y" << std::setw(8)   << "u" << "\n";

        for (int k = 0; k <= 200; ++k) {
            double u = sp.compute(ref - y);
            y = plant.step(u);
            if (k % 20 == 0)
                std::cout << std::setw(6) << k
                          << std::fixed << std::setprecision(3)
                          << std::setw(8)  << k * Ts
                          << std::setw(10) << y
                          << std::setw(8)  << u << "\n";
        }
    }
    return 0;
}
