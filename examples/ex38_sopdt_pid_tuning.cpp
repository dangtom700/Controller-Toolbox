/**
 * ex38_sopdt_pid_tuning.cpp
 * Compare IMC-SOPDT vs ZN tuning on a SOPDT plant (IAE benchmark).
 *
 * Identifies a SOPDT model from step data, tunes PID with both methods,
 * and compares closed-loop IAE.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <numeric>

int main()
{
    // True SOPDT: K=1.8, tau1=3.5, tau2=1.0, theta=1.0
    const double K_t    = 1.8;
    const double tau1_t = 3.5;
    const double tau2_t = 1.0;
    const double th_t   = 1.0;
    const double step_m = 1.0;
    const double Ts_id  = 0.2;

    const int N_id = 150;
    std::vector<double> t_id(N_id), y_id(N_id);
    for (int i = 0; i < N_id; ++i) {
        t_id[i] = i * Ts_id;
        const double dt = t_id[i] - th_t;
        y_id[i] = 0.0;
        if (dt > 0.0) {
            y_id[i] = K_t * step_m
                    * (1.0 - (tau1_t * std::exp(-dt / tau1_t)
                             - tau2_t * std::exp(-dt / tau2_t))
                             / (tau1_t - tau2_t));
        }
    }

    ctrl::SOPDTIdentifier id(t_id, y_id, step_m, 0.0);
    const ctrl::SOPDTModel m = id.identify(ctrl::SOPDTMethod::Optimization);

    // IMC-PID tuning (lambda_c = 2*theta)
    const double Ts = 0.1;
    const auto pp_imc = ctrl::SOPDTIdentifier::imcTuning(m, 2.0 * m.theta, Ts);

    // ZN tuning approximation: use FOPDT-equivalent with tauEq = tau1+tau2
    const double tauEq = m.tau1 + m.tau2;
    // ZN tuning for FOPDT: Ku ~ pi/(2*K*tau_eq), Tu ~ 4*theta
    const double Ku = 3.14159265 / (2.0 * m.K * tauEq);
    const double Tu = 4.0 * m.theta;
    ctrl::PIDParams pp_zn;
    pp_zn.Kp   = 0.6 * Ku;
    pp_zn.Ki   = pp_zn.Kp / (0.5 * Tu);
    pp_zn.Kd   = pp_zn.Kp * 0.125 * Tu;
    pp_zn.uMin = -20.0;
    pp_zn.uMax =  20.0;

    // Simulate on first-order approximation (tauEq)
    const double ref = 1.0;
    const int N_sim = 2000;

    auto simulate = [&](const ctrl::PIDParams &pp) -> double {
        ctrl::DiscretePID pid(pp, Ts);
        double y = 0.0, iae = 0.0;
        for (int k = 0; k < N_sim; ++k) {
            const double e = ref - y;
            const double u = pid.compute(e);
            const double a = std::exp(-Ts / tauEq);
            y = a * y + (1.0 - a) * m.K * u;
            iae += std::abs(e) * Ts;
        }
        return iae;
    };

    const double iae_imc = simulate(pp_imc);
    const double iae_zn  = simulate(pp_zn);

    std::cout << "Identified: K=" << m.K << " tau1=" << m.tau1
              << " tau2=" << m.tau2 << " theta=" << m.theta << "\n";
    std::cout << "IMC-PID IAE = " << iae_imc << "\n";
    std::cout << "ZN-PID  IAE = " << iae_zn  << "\n";
    std::cout << "IMC improvement over ZN: "
              << 100.0 * (iae_zn - iae_imc) / iae_zn << " %\n";

    if (std::isfinite(iae_imc) && iae_imc < 50.0)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
