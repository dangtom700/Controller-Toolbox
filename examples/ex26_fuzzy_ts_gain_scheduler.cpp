// ============================================================
//  ex26_fuzzy_ts_gain_scheduler.cpp
//  Application: Inverted pendulum balancing - Takagi-Sugeno (TS)
//  fuzzy gain scheduler that blends two linear PID controllers
//  tuned at different operating points.
//
//  Plant: linearised inverted pendulum on a cart
//    theta_ddot = (g/l)*sin(theta) - (b/ml^2)*theta_dot + (1/ml^2)*F
//  Linearised at two points:
//    OP1: theta = 0    (upright)   -> unstable pole pair
//    OP2: theta = 0.3  (tilted)    -> different gain requirement
//
//  TS scheme:
//    IF theta is Near_Zero   THEN use PID_1 (tight, upright gains)
//    IF theta is Large       THEN use PID_2 (aggressive recovery gains)
//    Output = w1*u_pid1 + w2*u_pid2   (weighted average, w1+w2=1)
//    Weights derived from Gaussian MFs on |theta|.
//
//  Demonstrates:
//    - Hand-built TS FuzzySystem (not using the FuzzyPD preset)
//    - Smooth gain blending vs hard switch vs single PID
//    - Recovery from large initial tilt (theta0 = 0.4 rad)
//
//  Output columns: t, theta, theta_dot, u_ts, u_pid1, w1, w2
// ============================================================
#include "ControllerToolbox.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>

// -- True nonlinear pendulum (Euler integration) -------------------------------
struct Pendulum {
    double g  = 9.81;
    double l  = 0.5;    // m
    double m  = 0.1;    // kg
    double b  = 0.05;   // friction Ns/m
    double theta     = 0.0;
    double theta_dot = 0.0;

    void step(double F, double dt) {
        double alpha = (g/l)*std::sin(theta)
                     - (b/(m*l*l))*theta_dot
                     + (1.0/(m*l*l))*F;
        theta_dot += alpha * dt;
        theta     += theta_dot * dt;
    }
};

int main()
{
    const double Ts = 0.005;   // 5 ms control loop

    // -- TS FuzzySystem - 1 input (|theta|), 2 singleton outputs (w1, w2) -----
    // We build TWO separate TS systems (one per weight) for clarity.
    // Near_Zero: Gaussian centred at 0, sigma=0.15 rad
    // Large:     Gaussian centred at 0.35 rad, sigma=0.15 rad
    // The system directly returns the normalised weight.

    auto buildWeightSys = [](double centre) -> ctrl::FuzzySystem {
        ctrl::FuzzySystem sys;
        sys.params.inference = ctrl::InferenceMethod::TakagiSugeno;
        sys.params.defuzz    = ctrl::DefuzzMethod::WeightedAverage;
        sys.params.uMin      = 0.0;
        sys.params.uMax      = 1.0;

        ctrl::LinguisticVariable vtheta;
        vtheta.name = "abs_theta"; vtheta.lo = 0.0; vtheta.hi = 0.5;
        vtheta.terms.push_back({"Near_Zero", ctrl::mfGaussian(0.00, 0.15)});
        vtheta.terms.push_back({"Large",     ctrl::mfGaussian(0.35, 0.15)});
        sys.addInput(vtheta);

        ctrl::LinguisticVariable vout;
        vout.name = "weight"; vout.lo = 0.0; vout.hi = 1.0;
        // Singletons: Near_Zero activates -> output 1.0; Large -> output 0.0
        // (and vice versa for the second weight system)
        if (centre < 0.1) {
            // Weight for PID_1 (upright): high when Near_Zero fires
            vout.terms.push_back(ctrl::ltSingleton("w1_hi", 1.0));
            vout.terms.push_back(ctrl::ltSingleton("w1_lo", 0.0));
        } else {
            // Weight for PID_2 (tilted): high when Large fires
            vout.terms.push_back(ctrl::ltSingleton("w2_lo", 0.0));
            vout.terms.push_back(ctrl::ltSingleton("w2_hi", 1.0));
        }
        sys.addOutput(vout);

        // Rule 1: IF Near_Zero THEN term[0]
        ctrl::Rule r1; r1.antecedents.push_back({0, 0}); r1.consequent_term_idx = 0; sys.addRule(r1);
        // Rule 2: IF Large THEN term[1]
        ctrl::Rule r2; r2.antecedents.push_back({0, 1}); r2.consequent_term_idx = 1; sys.addRule(r2);

        return sys;
    };

    ctrl::FuzzySystem sys_w1 = buildWeightSys(0.0);   // weight for PID_1
    ctrl::FuzzySystem sys_w2 = buildWeightSys(0.35);  // weight for PID_2

    // -- PID_1: upright-regime (tuned via pole placement, tau_c = 0.08 s) -----
    ctrl::PIDParams p1;
    p1.Kp = 18.0; p1.Ki = 5.0; p1.Kd = 2.5;
    p1.N  = 80.0; p1.Kb = 1.0;
    p1.uMin = -15.0; p1.uMax = 15.0;
    ctrl::DiscretePID pid1(p1, Ts);

    // -- PID_2: large-angle recovery (aggressive proportional) ----------------
    ctrl::PIDParams p2;
    p2.Kp = 40.0; p2.Ki = 2.0; p2.Kd = 5.0;
    p2.N  = 80.0; p2.Kb = 1.0;
    p2.uMin = -15.0; p2.uMax = 15.0;
    ctrl::DiscretePID pid2(p2, Ts);

    // -- Single PID baseline (pid1 gains, no scheduling) ---------------------
    ctrl::DiscretePID pid_baseline(p1, Ts);

    // -- Pendulum instances ---------------------------------------------------
    const double theta0 = 0.4;   // rad - large initial tilt
    Pendulum pend_ts, pend_base;
    pend_ts.theta = pend_base.theta = theta0;

    std::ofstream csv(std::string(PROJECT_DATA_DIR) + "/ex26_fuzzy_ts_pendulum.csv");
    csv << "t,theta_ts,theta_dot_ts,u_ts,theta_base,u_base,w1,w2\n";
    csv << std::fixed << std::setprecision(5);

    std::cout << "=== TS Fuzzy Gain Scheduler - Inverted Pendulum ===\n";
    std::cout << "   k     t[s]  theta_ts   u_ts    w1     theta_base  u_base\n";
    std::cout << std::string(70, '-') << "\n";

    const int N = 2000;   // 10 s
    const double ref = 0.0;  // upright

    // Weight threshold below which a PID's integral is frozen to prevent
    // windup while the controller is effectively inactive.
    static constexpr double W_FREEZE = 0.05;
    double w1_prev = 1.0, w2_prev = 0.0;   // track previous weights for bumpless init

    for (int k = 0; k < N; ++k) {
        double t = k * Ts;

        // -- TS-scheduled controller -------------------------------------------
        double abs_th = std::abs(pend_ts.theta);
        double w1 = sys_w1.evaluate({std::clamp(abs_th, 0.0, 0.5)});
        double w2 = sys_w2.evaluate({std::clamp(abs_th, 0.0, 0.5)});
        // Normalise
        double wsum = w1 + w2 + 1e-12;
        w1 /= wsum; w2 /= wsum;

        double e_ts = ref - pend_ts.theta;

        // Freeze the integral of a low-weight PID to prevent windup while it is
        // inactive.  When its weight recovers above the threshold, call bumplessInit
        // so its first compute() returns close to the current blended output rather
        // than injecting a pre-wound integral spike.
        double u_ts_prev = std::clamp(w1_prev * pid1.lastOutput() +
                                      w2_prev * pid2.lastOutput(), -15.0, 15.0);

        if (w1 <= W_FREEZE) {
            // pid1 is dormant: keep it synchronised without advancing its integral
            pid1.bumplessInit(u_ts_prev, e_ts);
        }
        if (w2 <= W_FREEZE) {
            pid2.bumplessInit(u_ts_prev, e_ts);
        }

        double u1 = (w1 > W_FREEZE) ? pid1.compute(e_ts) : pid1.lastOutput();
        double u2 = (w2 > W_FREEZE) ? pid2.compute(e_ts) : pid2.lastOutput();

        double u_ts = std::clamp(w1 * u1 + w2 * u2, -15.0, 15.0);
        w1_prev = w1; w2_prev = w2;

        pend_ts.step(u_ts, Ts);

        // -- Baseline single PID -----------------------------------------------
        double u_base = pid_baseline.compute(ref - pend_base.theta);
        u_base = std::clamp(u_base, -15.0, 15.0);
        pend_base.step(u_base, Ts);

        csv << t << ","
            << pend_ts.theta   << "," << pend_ts.theta_dot << "," << u_ts   << ","
            << pend_base.theta << "," << u_base << ","
            << w1 << "," << w2 << "\n";

        if (k % 200 == 0)
            std::cout << std::setw(5) << k
                      << std::setw(8)  << t
                      << std::setw(9)  << pend_ts.theta
                      << std::setw(8)  << u_ts
                      << std::setw(7)  << w1
                      << std::setw(13) << pend_base.theta
                      << std::setw(9)  << u_base << "\n";
    }

    std::cout << "\nResults saved to " << PROJECT_DATA_DIR << "/ex26_fuzzy_ts_pendulum.csv\n";
    return 0;
}
