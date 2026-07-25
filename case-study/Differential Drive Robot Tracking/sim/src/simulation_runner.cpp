#include "simulation_runner.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace differentialdriverobottracking {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kSettleBand = 0.05;   // paper: "after 12.7 s the error reduces to below 0.05 m"

/// Inner PI wheel-velocity loop - the paper's "Velocity control and actuation layer":
///   tau_R = (1/r)*[(Kp_v*e_v + Ki_v*int e_v)*R + (Kp_w*e_w + Ki_w*int e_w)*R]
///   tau_L = same with the angular term subtracted
/// Conditional anti-windup: integration is frozen while either wheel is saturated.
struct WheelPI {
    double int_v = 0.0;
    double int_w = 0.0;

    void reset() { int_v = 0.0; int_w = 0.0; }

    void step(const PlantParams& p, double v_cmd, double w_cmd, double v_meas, double w_meas,
              double& tau_R, double& tau_L) {
        const double e_v = v_cmd - v_meas;
        const double e_w = w_cmd - w_meas;

        const double trial_v = int_v + e_v * p.Ts_plant;
        const double trial_w = int_w + e_w * p.Ts_plant;

        const double lin = (p.Kp_v * e_v + p.Ki_v * trial_v) * p.R_half_axle;
        const double ang = (p.Kp_w * e_w + p.Ki_w * trial_w) * p.R_half_axle;

        const double inv_r = (p.r_wheel > 1e-9) ? 1.0 / p.r_wheel : 0.0;
        double tR = inv_r * (lin + ang);
        double tL = inv_r * (lin - ang);

        const bool saturated = (std::abs(tR) > p.tau_max) || (std::abs(tL) > p.tau_max);
        if (!saturated) {           // conditional anti-windup: hold the integrators when clipped
            int_v = trial_v;
            int_w = trial_w;
        }

        tau_R = std::clamp(tR, -p.tau_max, p.tau_max);
        tau_L = std::clamp(tL, -p.tau_max, p.tau_max);
    }
};

}  // namespace

Scenario Scenario::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    Scenario s;
    s.id          = j.value("id",          std::string("scenario"));
    s.description = j.value("description", std::string(""));
    s.trajectory  = j.value("trajectory",  s.trajectory);
    s.a           = j.value("a",           s.a);
    s.T_sim       = j.value("T_sim",       s.T_sim);
    s.time_scale  = j.value("time_scale",  s.time_scale);

    s.start_on_path = j.value("start_on_path", s.start_on_path);
    s.x0            = j.value("x0",     s.x0);
    s.y0            = j.value("y0",     s.y0);
    s.theta0        = j.value("theta0", s.theta0);

    s.encoder_noise_std = j.value("encoder_noise_std", s.encoder_noise_std);
    s.torque_dist_amp   = j.value("torque_dist_amp",   s.torque_dist_amp);
    s.torque_dist_freq  = j.value("torque_dist_freq",  s.torque_dist_freq);
    s.tau_max_override  = j.value("tau_max_override",  s.tau_max_override);
    s.mass_mismatch     = j.value("mass_mismatch",     s.mass_mismatch);
    s.seed              = j.value("seed",              s.seed);
    return s;
}

PlantParams effectivePlantParams(const PlantParams& nominal, const Scenario& scen) {
    PlantParams p = nominal;
    if (scen.tau_max_override > 0.0) p.tau_max = scen.tau_max_override;
    if (scen.mass_mismatch > 0.0)    p.M_total *= scen.mass_mismatch;
    return p;
}

RunMetrics runSimulation(const PlantParams& plant_p,
                         const Scenario&    scen,
                         ControllerBase&    ctrl,
                         const std::string& log_dir) {
    RunMetrics m;
    m.Ks_final = std::numeric_limits<double>::quiet_NaN();

    const PlantParams sim_p = effectivePlantParams(plant_p, scen);
    Plant plant(sim_p);
    Trajectory traj(scen.pathType(), scen.a, scen.time_scale, plant_p.v_max, plant_p.w_max);

    // Initial pose: exactly on the path, or the scenario's explicit offset start.
    const RefPoint r0 = traj.at(0.0);
    if (scen.start_on_path) plant.resetPose(r0.x, r0.y, r0.theta);
    else                    plant.resetPose(scen.x0, scen.y0, scen.theta0);

    ctrl.reset();
    WheelPI inner;
    std::mt19937 rng(scen.seed);
    std::normal_distribution<double> noise(0.0, 1.0);

    std::ofstream out;
    const bool logging = !log_dir.empty();
    if (logging) {
        out.open(log_dir + "/run_" + scen.id + "_" + ctrl.name() + ".csv");
        // Pose columns are x_pos/y_pos/theta_pos, NOT x/y/theta: tools/metrics.py auto-picks
        // its output column as the first of ['y', 'y1', ...] and its reference as the first of
        // ['ref', ..., 'x_ref', ...]. Naming the planar position 'y' would pair robot Y against
        // reference X and produce a meaningless SISO loop (mu_analysis reported peak_T = 0).
        // The trailing ref/y/u triple exposes the heading loop - a genuine SISO pair - to the
        // generic tooling, while iae_cumulative remains what extract_final_iae reads.
        out << "t,x_ref,y_ref,theta_ref,x_pos,y_pos,theta_pos,e1,e2,e3,e_pos,v_cmd,w_cmd,"
               "tau_R,tau_L,d_hat,alpha,Ks,V_lyap,iae_cumulative,ref,y,u\n";
        out << std::fixed << std::setprecision(6);
    }

    const int    n_fast     = static_cast<int>(std::lround(scen.T_sim / plant_p.Tf));
    const int    sub_steps  = plant_p.plantSubSteps();
    const int    slow_div   = plant_p.slowDivider();
    const double Tf         = plant_p.Tf;

    double iae = 0.0;
    double e1_prev = 0.0, e2_prev = 0.0, e3_prev = 0.0;
    bool   first = true;
    bool   settled = false;
    double settle_candidate = -1.0;
    double torque_accum = 0.0;
    int    torque_samples = 0;

    for (int k = 0; k < n_fast; ++k) {
        const double t  = k * Tf;
        const RefPoint r = traj.at(t);

        // -- Measurement (optionally noisy) ---------------------------------
        double xm = plant.X(), ym = plant.Y(), thm = plant.theta();
        if (scen.encoder_noise_std > 0.0) {
            xm  += scen.encoder_noise_std * noise(rng);
            ym  += scen.encoder_noise_std * noise(rng);
            thm += scen.encoder_noise_std * noise(rng);
        }

        // -- Body-frame errors: [e1,e2,e3] = Rz(theta)^T * (q_r - q) --------
        const double dx = r.x - xm, dy = r.y - ym;
        const double c = std::cos(thm), s = std::sin(thm);
        BodyError e;
        e.e1 =  c * dx + s * dy;
        e.e2 = -s * dx + c * dy;
        e.e3 = wrapAngle(r.theta - thm);
        if (first) { e1_prev = e.e1; e2_prev = e.e2; e3_prev = e.e3; first = false; }
        e.de1 = (e.e1 - e1_prev) / Tf;
        e.de2 = (e.e2 - e2_prev) / Tf;
        e.de3 = wrapAngle(e.e3 - e3_prev) / Tf;
        e1_prev = e.e1; e2_prev = e.e2; e3_prev = e.e3;

        // -- Outer kinematic controller (fast loop, Tf) ----------------------
        const Eigen::Vector2d u = ctrl.compute(e, r.v, r.w);
        const double v_cmd = std::clamp(u(0), -plant_p.v_max, plant_p.v_max);
        const double w_cmd = std::clamp(u(1), -plant_p.w_max, plant_p.w_max);

        // -- Slow adaptation loop (Ts_slow = 5 * Tf) -------------------------
        if (k > 0 && (k % slow_div) == 0) ctrl.slowTick();

        // -- Inner PI torque loop + plant, sub-stepped at Ts_plant -----------
        double tau_R = 0.0, tau_L = 0.0;
        for (int i = 0; i < sub_steps; ++i) {
            inner.step(sim_p, v_cmd, w_cmd, plant.v(), plant.w(), tau_R, tau_L);

            double dR = tau_R, dL = tau_L;
            if (scen.torque_dist_amp > 0.0) {
                const double td = t + i * sim_p.Ts_plant;
                const double d  = scen.torque_dist_amp *
                                  std::sin(2.0 * kPi * scen.torque_dist_freq * td);
                dR += d;
                dL -= d;   // differential disturbance: also perturbs heading
            }
            plant.step(dR, dL);
        }

        // -- Metrics (paper's Table 2 definitions) ---------------------------
        const double ex = r.x - plant.X();
        const double ey = r.y - plant.Y();
        const double e_pos = std::hypot(ex, ey);

        m.ise  += (ex * ex + ey * ey) * Tf;
        m.iae  += (std::abs(ex) + std::abs(ey)) * Tf;
        m.itae += t * (std::abs(ex) + std::abs(ey)) * Tf;
        iae    += e_pos * Tf;
        m.max_err = std::max(m.max_err, e_pos);

        const double tau_peak = std::max(std::abs(tau_R), std::abs(tau_L));
        torque_accum += tau_peak;
        ++torque_samples;
        m.max_torque = std::max(m.max_torque, tau_peak);

        // Settling: first time the error enters the band and never leaves again.
        if (e_pos < kSettleBand) {
            if (!settled) { settle_candidate = t; settled = true; }
        } else {
            settled = false;
            settle_candidate = -1.0;
        }

        if (logging) {
            const CtrlTelemetry tel = ctrl.telemetry();
            out << t << ',' << r.x << ',' << r.y << ',' << r.theta << ','
                << plant.X() << ',' << plant.Y() << ',' << plant.theta() << ','
                << e.e1 << ',' << e.e2 << ',' << e.e3 << ',' << e_pos << ','
                << v_cmd << ',' << w_cmd << ',' << tau_R << ',' << tau_L << ','
                << tel.d_hat << ',' << tel.alpha << ',' << tel.Ks << ',' << tel.V << ','
                << iae << ','
                // Generic-tooling SISO triple: the heading loop (ref, y, u).
                << r.theta << ',' << plant.theta() << ',' << w_cmd << '\n';
        }

        m.final_err = e_pos;
    }

    m.settle_time = settle_candidate;
    m.mean_torque = (torque_samples > 0) ? torque_accum / torque_samples : 0.0;
    m.Ks_final    = ctrl.telemetry().Ks;

    if (logging) out.flush();
    return m;
}

}  // namespace differentialdriverobottracking
