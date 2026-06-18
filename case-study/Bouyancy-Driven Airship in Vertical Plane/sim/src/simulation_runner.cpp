#include "simulation_runner.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <stdexcept>

namespace bouyancydrivenairshipinverticalplan {

Scenario Scenario::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    Scenario s;
    s.id        = j.value("id", std::string("scenario"));
    s.T_sim     = j.value("T_sim", s.T_sim);
    s.ref_init  = j.value("ref_init", s.ref_init);
    s.ref_final = j.value("ref_final", s.ref_final);
    s.step_time = j.value("step_time", s.step_time);
    return s;
}

double runSimulation(const PlantParams& plant_p,
                     const Scenario& scen,
                     const NamedController& nc,
                     const std::string& log_dir) {
    Plant plant(plant_p);
    plant.reset(Eigen::VectorXd::Zero(plant.stateSize()));
    if (nc.ctrl) nc.ctrl->reset();

    const std::string csv = log_dir + "/run_" + scen.id + "_" + nc.name + ".csv";
    std::ofstream out(csv);
    out << "t,ref,y,u,iae_cumulative\n";

    const int N = static_cast<int>(scen.T_sim / plant_p.Ts);
    double iae = 0.0;
    for (int k = 0; k < N; ++k) {
        const double t   = k * plant_p.Ts;
        const double ref = (t >= scen.step_time) ? scen.ref_final : scen.ref_init;
        const double y   = plant.output();
        const double e   = ref - y;
        const double u   = nc.ctrl ? nc.ctrl->compute(e) : 0.0;  // TODO: sign convention
        plant.step(u);
        iae += std::abs(e) * plant_p.Ts;
        out << t << ',' << ref << ',' << y << ',' << u << ',' << iae << '\n';
    }
    return iae;
}

}  // namespace bouyancydrivenairshipinverticalplan
