#include "simulation_runner.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <stdexcept>

namespace bouyancydrivenairshipinverticalplan {

namespace {
constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
}

Scenario Scenario::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    Scenario s;
    s.id              = j.value("id", s.id);
    s.description     = j.value("description", s.description);
    s.T_sim           = j.value("T_sim", s.T_sim);
    s.theta0_deg      = j.value("theta0_deg", s.theta0_deg);
    s.rp1_ref         = j.value("rp1_ref", s.rp1_ref);
    s.v1_0            = j.value("v1_0", s.v1_0);
    s.v3_0            = j.value("v3_0", s.v3_0);
    s.m0_initial      = j.value("m0_initial", s.m0_initial);
    s.ref_type        = j.value("ref_type", s.ref_type);
    s.theta_ref_deg   = j.value("theta_ref_deg", s.theta_ref_deg);
    s.step_time       = j.value("step_time", s.step_time);
    s.theta_ref_a_deg = j.value("theta_ref_a_deg", s.theta_ref_a_deg);
    s.theta_ref_b_deg = j.value("theta_ref_b_deg", s.theta_ref_b_deg);
    s.half_period     = j.value("half_period", s.half_period);
    s.m0_switch_kg    = j.value("m0_switch_kg", s.m0_switch_kg);
    s.dist_time       = j.value("dist_time", s.dist_time);
    s.dist_q_impulse  = j.value("dist_q_impulse", s.dist_q_impulse);
    return s;
}

double Scenario::thetaRefAt(double t) const {
    if (ref_type == "sawtooth") {
        const double phase = std::fmod(t, 2.0 * half_period);
        return (phase < half_period) ? theta_ref_a_deg * DEG2RAD : theta_ref_b_deg * DEG2RAD;
    }
    return (t >= step_time) ? theta_ref_deg * DEG2RAD : theta0_deg * DEG2RAD;
}

double Scenario::m0At(double t) const {
    if (ref_type == "sawtooth") {
        const double phase = std::fmod(t, 2.0 * half_period);
        return (phase < half_period) ? m0_switch_kg : -m0_switch_kg;
    }
    return m0_initial;
}

double runSimulation(const PlantParams&    plant_p,
                     const Scenario&       scen,
                     ControllerBase&       nc,
                     const std::string&    log_dir) {
    Plant plant(plant_p);
    plant.reset(scen.theta0_deg * DEG2RAD, scen.rp1_ref, scen.v1_0, scen.v3_0);
    nc.reset();

    const std::string csv_path = log_dir + "/run_" + scen.id + "_" + nc.name() + ".csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) throw std::runtime_error("cannot open log: " + csv_path);
    csv << "t,theta_ref,theta,rp1,v1,v3,u,m0,error,iae_cumulative\n";

    const int N = static_cast<int>(std::lround(scen.T_sim / plant_p.Ts));
    double iae = 0.0;
    bool dist_applied = false;

    auto t_wall_start = std::chrono::steady_clock::now();

    for (int k = 0; k < N; ++k) {
        const double t         = k * plant_p.Ts;
        const double theta_ref = scen.thetaRefAt(t);
        const double m0        = scen.m0At(t);

        if (!dist_applied && scen.dist_time >= 0.0 && t >= scen.dist_time) {
            State x = plant.state();
            x(Q) += scen.dist_q_impulse;
            plant.setState(x);
            dist_applied = true;
        }

        const State& x  = plant.state();
        const double err = theta_ref - x(THETA);
        iae += std::abs(err) * plant_p.Ts;

        const double u = nc.compute(x, theta_ref, scen.rp1_ref, m0);

        csv << std::fixed << std::setprecision(6)
            << t << ',' << theta_ref << ',' << x(THETA) << ',' << x(RP1) << ','
            << x(V1) << ',' << x(V3) << ',' << u << ',' << m0 << ','
            << err << ',' << iae << '\n';

        plant.step(u, m0);
    }

    csv.close();

    auto t_wall_end = std::chrono::steady_clock::now();
    const double wall_ms = std::chrono::duration<double, std::milli>(t_wall_end - t_wall_start).count();

    const double theta_final_deg = plant.state()(THETA) / DEG2RAD;
    std::cout << std::fixed << std::setprecision(4)
              << "[" << scen.id << " | " << nc.name() << "]"
              << "  IAE=" << iae
              << "  theta_final=" << theta_final_deg << " deg"
              << "  wall=" << std::setprecision(1) << wall_ms << " ms"
              << "  -> " << csv_path << '\n';

    return iae;
}

}  // namespace bouyancydrivenairshipinverticalplan
