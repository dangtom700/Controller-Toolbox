#pragma once
#include <Eigen/Dense>
#include <string>

// Bell-Astrom (1987) nonlinear boiler-turbine plant.
//
// States:  x = [x1, x2, x3]
//   x1 = drum pressure        [kg/cm^2]
//   x2 = electric power       [MW]
//   x3 = fluid density in drum [kg/cm^3]
//
// Inputs:  u = [u1, u2, u3]  (all in [0, 1])
//   u1 = fuel flow valve
//   u2 = steam control valve
//   u3 = feedwater valve
//
// Outputs: y = [y1, y2, y3]
//   y1 = drum pressure  (= x1)
//   y2 = electric power (= x2)
//   y3 = drum water level deviation [m]  = 0.05*(0.13073*x3 + 100*acs + qe/9 - 67.975)

namespace boiler {

struct OperatingPoint {
    std::string label;
    double x1, x2, x3;   // state at equilibrium
    double u1, u2, u3;   // control at equilibrium
    double y1, y2, y3;   // output at equilibrium  (y1=x1, y2=x2)
};

// Three standard operating points from Bell-Astrom (1987)
extern const OperatingPoint op_A;   // Low Load
extern const OperatingPoint op_B;   // Medium Load
extern const OperatingPoint op_C;   // High Load

// Returns the operating point by label "A", "B", or "C".
const OperatingPoint& getOperatingPoint(const std::string& label);

// Compute y3 from plant state and inputs (used for initialisation).
double computeY3(double x1, double x2, double x3, double u1, double u2, double u3);

class BoilerTurbine {
public:
    static constexpr double Ts = 1.0;   // sample time [s]

    double u1, u2, u3;   // current valve positions
    double x1, x2, x3;   // current states
    double y1, y2, y3;   // current outputs
    double du1, du2, du3; // last applied valve increments

    BoilerTurbine();

    // Initialise at an operating point.
    void initAt(const OperatingPoint& op);

    // Set state directly (for scenario initial conditions).
    void setState(double x1_, double x2_, double x3_);

    // Set control inputs directly (before constrain + update).
    void setControls(double u1_, double u2_, double u3_);

    // Clamp valves to [0, 1].
    void constrainValve();

    // Enforce per-step valve rate limits and update du1/du2/du3.
    void constrainValveRate();

    // Advance nonlinear ODE one Ts step and recompute outputs.
    void update();

    // Return current outputs as vector [y1, y2, y3].
    Eigen::Vector3d measureOutputs() const;

    // Return current controls as vector [u1, u2, u3].
    Eigen::Vector3d controls() const;

    // Return current state as vector [x1, x2, x3].
    Eigen::Vector3d state() const;

private:
    double u1_prev_, u2_prev_, u3_prev_;

    void recomputeOutputs();
};

} // namespace boiler
