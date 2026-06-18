#include "bouyancy_driven_airship_in_vertical_plan_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace bouyancydrivenairshipinverticalplan {

PlantParams PlantParams::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    PlantParams p;
    p.Ts      = j.value("Ts", p.Ts);
    p.param_a = j.value("param_a", p.param_a);
    p.param_b = j.value("param_b", p.param_b);
    p.u_max   = j.value("u_max", p.u_max);
    p.u_min   = j.value("u_min", p.u_min);
    return p;
}

Plant::Plant(const PlantParams& p) : p_(p) {
    x_ = Eigen::VectorXd::Zero(1);       // TODO: real state dimension
}

void Plant::reset(const Eigen::VectorXd& x0) { x_ = x0; }

void Plant::step(double u) {
    if (u > p_.u_max) u = p_.u_max;
    if (u < p_.u_min) u = p_.u_min;
    // TODO: real dynamics. Placeholder first-order: x' = -a*x + b*u
    const double dx = -p_.param_a * x_(0) + p_.param_b * u;
    x_(0) += p_.Ts * dx;
}

double Plant::output() const { return x_(0); }

}  // namespace bouyancydrivenairshipinverticalplan
