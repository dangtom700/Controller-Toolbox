#pragma once
#include <Eigen/Dense>
#include <ControllerToolbox.h>

// Separate Meter-In / Meter-Out (SMISMO) hydraulic actuator plant model.
//
// 9-state nonlinear ODE (for simulation):
//   x[0] = piston position       [m]
//   x[1] = piston velocity       [m/s]
//   x[2] = piston-side pressure  [Pa]
//   x[3] = rod-side pressure     [Pa]
//   x[4] = supply pressure       [Pa]
//   x[5] = meter-in spool pos    [-1, 1]
//   x[6] = meter-in spool vel    [/s]
//   x[7] = meter-out spool pos   [-1, 1]
//   x[8] = meter-out spool vel   [/s]
//
// Inputs: u[0] = meter-in cmd (u1), u[1] = meter-out cmd (u2), both in [-1, 1].
// Outputs (measure()): [x, v, p1, p2].
//
// For controller design a 4-state SISO reduced model is provided via linearise4():
//   states [x, v, p1, p2], input u_cmd (coupled u1=u2), supply pressure held constant.

namespace smismo {

struct SmismoParams {
    // Cylinder geometry
    double A1     = 3.117e-3;   // piston-side area [m^2]  (bore D=63 mm)
    double A2     = 1.526e-3;   // rod-side area    [m^2]  (rod  D=45 mm)
    double L      = 0.30;       // stroke           [m]

    // Load
    double M      = 15.0;       // piston+load mass [kg]
    double Bv     = 500.0;      // viscous damping  [N.s/m]
    double F_ext  = 0.0;        // external load    [N]  (+ opposes extension)

    // Hydraulics
    double beta_e = 300.0e6;    // effective bulk modulus [Pa] (accounting for hose/air compliance)
    double rho    = 870.0;      // fluid density          [kg/m^3]
    double Pt     = 0.0;        // tank pressure          [Pa] (gauge)

    // Volumes
    double V10    = 0.8e-3;     // min piston-side dead vol [m^3]
    double V20    = 0.5e-3;     // min rod-side dead vol    [m^3]
    double Vs     = 1.5e-3;     // supply line volume       [m^3]

    // Pump & relief valve
    double Qp     = 2.5e-4;     // pump ideal flow  [m^3/s]  (15 L/min)
    double Pr     = 14.5e6;     // relief cracking  [Pa]    (145 bar)
    double Kr     = 5.0e-10;    // relief flow gain [m^3/(s.Pa)]

    // Proportional valves (both identical)
    double wn_v   = 251.3;      // natural frequency [rad/s]  (40 Hz)
    double zeta_v = 0.70;       // damping ratio
    double Cd     = 0.70;       // discharge coefficient
    double Amax   = 2.252e-6;   // max orifice area per spool [m^2]
};

// Static equilibrium: piston at mid-stroke, valves closed.
// Force balance: A1*p1_eq = A2*p2_eq (zero load).
struct SmismoOperatingPoint {
    double x_eq   = 0.15;       // position   [m]
    double v_eq   = 0.0;        // velocity   [m/s]
    double p1_eq  = 1.0e6;      // piston-side [Pa]  (10 bar)
    double p2_eq  = 2.044e6;    // rod-side    [Pa]  (A1/A2 * p1)
    double ps_eq  = 15.0e6;     // supply      [Pa]  (150 bar)
    double xv1_eq = 0.0;
    double xv2_eq = 0.0;
    double u1_eq  = 0.0;
    double u2_eq  = 0.0;
};

class SmismoPlant {
public:
    static constexpr int Nx = 9;
    static constexpr int Nu = 2;
    static constexpr int Ny = 4;   // [x, v, p1, p2]

    explicit SmismoPlant(const SmismoParams& params = SmismoParams{},
                         const SmismoOperatingPoint& op = SmismoOperatingPoint{});

    // Continuous-time ODE
    Eigen::Matrix<double,Nx,1> f(const Eigen::Matrix<double,Nx,1>& x,
                                  const Eigen::Matrix<double,Nu,1>& u) const;

    // RK4 integration step
    void step(double dt, const Eigen::Matrix<double,Nu,1>& u);

    // Output: [x, v, p1, p2]
    Eigen::Vector4d measure() const;

    // Reset state to initial conditions at position x0_pos
    void reset(double x0_pos = -1.0);   // -1 -> use op.x_eq

    // 4-state SISO reduced model for controller design (ZOH at Ts)
    // States: [x, v, p1, p2], single input u_cmd (coupled valves), output: full state
    ctrl::StateSpace linearise4(double Ts) const;

    // 2-state reduced model [x, v] for LQR/LQG/TubeMPC design (ZOH at Ts).
    // Lumps hydraulic force into effective acceleration gain b0 [m/s^2/unit].
    // Output: full state y = [x, v] with C = I.
    // This model is fully controllable and avoids DARE stabilizability warnings.
    ctrl::StateSpace linearise2(double Ts) const;

    // Estimated hydraulic acceleration gain b0 [m/s^2/unit] at the operating point.
    double accelGain() const;

    void setLoad(double F_ext) { p_.F_ext = F_ext; }

    const Eigen::Matrix<double,Nx,1>& state() const { return x_; }
    void setState(const Eigen::Matrix<double,Nx,1>& s) { x_ = s; }
    const SmismoParams& params() const { return p_; }
    const SmismoOperatingPoint& op() const { return op_; }

private:
    SmismoParams p_;
    SmismoOperatingPoint op_;
    Eigen::Matrix<double,Nx,1> x_;

    double V1(double xpos) const { return p_.V10 + p_.A1 * xpos; }
    double V2(double xpos) const { return p_.V20 + p_.A2 * (p_.L - xpos); }

    // Orifice flow: q = Cd * Amax * |opening| * sqrt(2*max(dP,0)/rho)
    double orifice(double opening, double dP) const;
};

} // namespace smismo
