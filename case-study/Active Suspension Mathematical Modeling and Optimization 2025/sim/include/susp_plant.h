#pragma once
#include <Eigen/Dense>
#include <string>

// Quarter-car active suspension plant.
//
// Source: Abdulwahab et al. (2025) "Mathematical modeling and optimization of
//         the active suspension", Alexandria Engineering Journal.
//
// 2-DOF quarter-car model; 4 states:
//   x = [z_s, dz_s, z_u, dz_u]
//
// Equations of motion:
//   m_s * ddz_s = -k_s*(z_s-z_u) - c_s*(dz_s-dz_u) + F_act
//   m_u * ddz_u =  k_s*(z_s-z_u) + c_s*(dz_s-dz_u) - k_t*(z_u-z_r) - F_act
//
// Integration: RK4 at fixed Ts.

namespace susp {

struct PlantParams {
    double m_s    = 240.0;    // sprung mass [kg]
    double m_u    = 36.0;     // unsprung mass [kg]
    double k_s    = 16000.0;  // suspension spring stiffness [N/m]
    double c_s    = 980.0;    // suspension damper coefficient [N.s/m]
    double k_t    = 160000.0; // tyre stiffness [N/m]
    double F_max  = 2000.0;   // actuator force saturation [N]
    double Ts     = 0.005;    // controller/integrator sample time [s]
    double duration = 5.0;    // default scenario duration [s]

    // Derived (filled by init())
    double omega_body  = 0.0; // body natural frequency [rad/s]
    double omega_wheel = 0.0; // wheel hop natural frequency [rad/s]
    double zeta_body   = 0.0; // body damping ratio [-]

    void init();
    static PlantParams fromJson(const std::string& path);
};

// 4-state quarter-car plant.  Integrates with RK4 at Ts.
class QuarterCarPlant {
public:
    explicit QuarterCarPlant(const PlantParams& p);

    // Advance one Ts step.
    //   F_act: actuator force [N] (positive = extends suspension)
    //   z_r  : road surface height [m]
    void step(double F_act, double z_r);

    // Full 4-state vector [z_s, dz_s, z_u, dz_u]
    const Eigen::Vector4d& state() const { return x_; }

    double z_s()  const { return x_(0); } // sprung mass displacement [m]
    double dz_s() const { return x_(1); } // sprung mass velocity [m/s]
    double z_u()  const { return x_(2); } // unsprung mass displacement [m]
    double dz_u() const { return x_(3); } // unsprung mass velocity [m/s]

    // Body vertical acceleration [m/s^2] given current inputs
    double bodyAccel(double F_act, double z_r) const;

    // Suspension deflection z_s - z_u [m]
    double suspDeflection() const { return x_(0) - x_(2); }

    // Dynamic tyre deflection z_u - z_r [m]
    double tyreDeflection(double z_r) const { return x_(2) - z_r; }

    void reset();
    const PlantParams& params() const { return p_; }

private:
    PlantParams      p_;
    Eigen::Vector4d  x_;

    Eigen::Vector4d f(const Eigen::Vector4d& x, double F_act, double z_r) const;
};

} // namespace susp
