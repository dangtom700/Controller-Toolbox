#pragma once
// bouyancy_driven_airship_in_vertical_plan_plant.h - plant model for Bouyancy-Driven Airship in Vertical Plane (TEMPLATE)
#include <Eigen/Dense>
#include <string>

namespace bouyancydrivenairshipinverticalplan {

// Plant parameters loaded from config/plant_params.json (nlohmann/json).
struct PlantParams {
    double Ts     = 0.01;
    double param_a = 1.0;
    double param_b = 1.0;
    double u_max  =  1.0;
    double u_min  = -1.0;
    static PlantParams fromJson(const std::string& path);
};

// Discrete-time plant: x[k+1] = f(x[k], u[k]); y = h(x).
class Plant {
public:
    explicit Plant(const PlantParams& p);
    void reset(const Eigen::VectorXd& x0);
    void step(double u);                 // advance one Ts (RK4/Euler internally)
    double output() const;               // measured output y
    const Eigen::VectorXd& state() const { return x_; }
    int stateSize() const { return static_cast<int>(x_.size()); }
private:
    PlantParams p_;
    Eigen::VectorXd x_;
};

}  // namespace bouyancydrivenairshipinverticalplan
