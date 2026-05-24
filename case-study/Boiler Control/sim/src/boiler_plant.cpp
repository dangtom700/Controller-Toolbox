#include "boiler_plant.h"
#include <cmath>
#include <stdexcept>

namespace boiler {

// -- Operating points ---------------------------------------------------------

double computeY3(double x1, double x2, double x3, double u1, double u2, double u3)
{
    (void)x2;
    double acs = ((1.0 - 0.001538 * x3) * 0.8 * x1 - 25.6) /
                 (x3 * (1.0394 - 0.0012304 * x1));
    double qe  = (0.854 * u2 - 0.147) * x1 + 45.59 * u1 - 2.514 * u3 - 2.096;
    return 0.05 * (0.13073 * x3 + 100.0 * acs + qe / 9.0 - 67.975);
}

const OperatingPoint op_A = {
    "Low Load",
    75.6, 15.3, 508.97,
    0.11926, 0.38063, 0.12262,
    75.6, 15.3, computeY3(75.6, 15.3, 508.97, 0.11926, 0.38063, 0.12262)
};

const OperatingPoint op_B = {
    "Medium Load",
    97.2, 50.5, 469.51,
    0.27049, 0.62082, 0.33979,
    97.2, 50.5, computeY3(97.2, 50.5, 469.51, 0.27049, 0.62082, 0.33979)
};

const OperatingPoint op_C = {
    "High Load",
    140.0, 128.0, 323.68,
    0.59589, 0.89447, 0.78829,
    140.0, 128.0, computeY3(140.0, 128.0, 323.68, 0.59589, 0.89447, 0.78829)
};

const OperatingPoint& getOperatingPoint(const std::string& label)
{
    if (label == "A") return op_A;
    if (label == "B") return op_B;
    if (label == "C") return op_C;
    throw std::invalid_argument("Unknown operating point label: " + label);
}

// -- BoilerTurbine ------------------------------------------------------------

BoilerTurbine::BoilerTurbine()
    : u1(0.5), u2(0.5), u3(0.5)
    , x1(100.0), x2(50.0), x3(20.0)
    , y1(0.0), y2(0.0), y3(0.0)
    , du1(0.0), du2(0.0), du3(0.0)
    , u1_prev_(0.5), u2_prev_(0.5), u3_prev_(0.5)
{
    recomputeOutputs();
}

void BoilerTurbine::initAt(const OperatingPoint& op)
{
    x1 = op.x1; x2 = op.x2; x3 = op.x3;
    u1 = op.u1; u2 = op.u2; u3 = op.u3;
    u1_prev_ = op.u1; u2_prev_ = op.u2; u3_prev_ = op.u3;
    du1 = du2 = du3 = 0.0;
    recomputeOutputs();
}

void BoilerTurbine::setState(double x1_, double x2_, double x3_)
{
    x1 = x1_; x2 = x2_; x3 = x3_;
    recomputeOutputs();
}

void BoilerTurbine::setControls(double u1_, double u2_, double u3_)
{
    u1 = u1_; u2 = u2_; u3 = u3_;
}

void BoilerTurbine::constrainValve()
{
    u1 = std::clamp(u1, 0.0, 1.0);
    u2 = std::clamp(u2, 0.0, 1.0);
    u3 = std::clamp(u3, 0.0, 1.0);
}

void BoilerTurbine::constrainValveRate()
{
    du1 = std::clamp(u1 - u1_prev_, -0.007, 0.007);
    du2 = std::clamp(u2 - u2_prev_, -0.020, 0.020);
    du3 = std::clamp(u3 - u3_prev_, -0.050, 0.050);
    u1 = u1_prev_ + du1;
    u2 = u2_prev_ + du2;
    u3 = u3_prev_ + du3;
    u1_prev_ = u1;
    u2_prev_ = u2;
    u3_prev_ = u3;
}

void BoilerTurbine::update()
{
    double x1_98 = std::pow(x1, 9.0 / 8.0);

    double dx1 = -0.0018 * u2 * x1_98 + 0.9 * u1 - 0.15 * u3;
    double dx2 = (0.073 * u2 - 0.016) * x1_98 - 0.1 * x2;
    double dx3 = (141.0 * u3 - (1.1 * u2 - 0.19) * x1) / 85.0;

    x1 += Ts * dx1;
    x2 += Ts * dx2;
    x3 += Ts * dx3;

    recomputeOutputs();
}

Eigen::Vector3d BoilerTurbine::measureOutputs() const
{
    return Eigen::Vector3d(y1, y2, y3);
}

Eigen::Vector3d BoilerTurbine::controls() const
{
    return Eigen::Vector3d(u1, u2, u3);
}

Eigen::Vector3d BoilerTurbine::state() const
{
    return Eigen::Vector3d(x1, x2, x3);
}

void BoilerTurbine::recomputeOutputs()
{
    y1 = x1;
    y2 = x2;
    y3 = computeY3(x1, x2, x3, u1, u2, u3);
}

} // namespace boiler
