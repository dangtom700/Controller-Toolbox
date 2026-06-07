#pragma once
#include <Eigen/Dense>
#include <string>

// Non-Inverting Buck-Boost Converter averaged plant model.
//
// Reference: Almasi et al. (2017) ISA Transactions 67, 515-527.
//
// 2 states: x = [i_L, v_C]   (inductor current [A], cap. voltage [V])
//
// Buck mode (V_ref < V_in):
//   L * di_L/dt = d*V_in - v_C
//   C * dv_C/dt = i_L - v_C/R
//
// Boost mode (V_ref > V_in):
//   L * di_L/dt = V_in - (1-d)*v_C
//   C * dv_C/dt = (1-d)*i_L - v_C/R
//
// Integration: classical RK4 at Ts = 1/f_s = 20 us.

namespace conv {

struct PlantParams {
    double L          = 50e-6;   // inductance [H]
    double C          = 1.8e-3;  // capacitance [F]
    double R          = 2.0;     // load resistance [Ohm]
    double V_in_nom   = 10.0;    // nominal input voltage [V]
    double f_s        = 50000.0; // switching frequency [Hz]
    double Ts         = 20e-6;   // sample time [s]
    double d_min      = 0.0;
    double d_max      = 1.0;
    double hysteresis = 0.1;     // mode-switch hysteresis band [V]
    double duration   = 0.060;   // default scenario duration [s]

    static PlantParams fromJson(const std::string& path);
};

enum class ConvMode { BUCK, BOOST };

class BuckBoostPlant {
public:
    explicit BuckBoostPlant(const PlantParams& p);

    // Advance one Ts step.
    void step(double d, ConvMode mode, double V_in);

    const Eigen::Vector2d& state() const { return x_; }
    double i_L() const { return x_(0); }
    double v_C() const { return x_(1); }

    void reset();
    const PlantParams& params() const { return p_; }

private:
    PlantParams      p_;
    Eigen::Vector2d  x_;

    Eigen::Vector2d f(const Eigen::Vector2d& x, double d,
                      ConvMode mode, double V_in) const;
};

} // namespace conv
