#pragma once
#include <Eigen/Dense>
#include <string>

// Separate Meter-In Separate Meter-Out (SMISMO) hydraulic actuator plant.
//
// References:
//   Chen, Wang, Wang, Zhao, Shen (2018), Control Engineering Practice 72, 138-150
//     (Eqs. 2-7, Table 1 parameters, identified Stribeck friction).
//   Liu, Xu, Yang, Zeng (2009), IEEE/ASME AIM, 227-232
//     (rated-flow valve normalisation Eq. 14, dual-loop controller structure Fig. 10).
//
// 8 states: x = [x_L, v_L, P_1, P_2, xv_1, dxv_1, xv_2, dxv_2]
//   x_L  = load position [m] (0..stroke, extension positive, lifts hanging load)
//   v_L  = load velocity [m/s]
//   P_1  = cap (cylinder-end) chamber pressure [Pa]
//   P_2  = rod-end chamber pressure [Pa]
//   xv_i = normalised PDCV spool position [-1, 1] (2nd-order dynamics from u_i)
//
// Inputs: u_1, u_2 in [-10, 10] V (PDCV1 cap side, PDCV2 rod side).
//   xv_i >= 0 connects chamber i to supply P_s; xv_i < 0 connects to tank P_r.
//
// Load dynamics (Chen Eq. 2):  m*dv = P_1*A_1 - P_2*A_2 - m*g - F_f(v) - F_ext
// Continuity   (Chen Eq. 3):   (V_i(x)/beta_e)*dP_i = Q_i -/+ A_i*v
// Orifice      (Chen Eq. 4-5 via Liu Eq. 14): Q_i = xv_i*Q_nom_i*sqrt(DP_i/DP_nom)
//
// Integration: RK4 at Ts = 1 ms with N_SUBSTEPS inner steps.

namespace smismo {

struct PlantParams {
    // Cylinder (Chen Table 1)
    double A1       = 4.91e-4;   // cap-side piston area [m^2]
    double A2       = 2.90e-4;   // rod-side annulus area [m^2]
    double V10      = 1.0e-3;    // cap chamber dead volume [m^3]
    double V20      = 1.0e-3;    // rod chamber dead volume [m^3]
    double stroke   = 0.5;       // piston stroke [m]
    double m        = 50.0;      // load mass [kg]
    double g        = 9.8;       // gravity [m/s^2] (hanging load: F = m*g)

    // Fluid
    double beta_e   = 890.0e6;   // effective bulk modulus [Pa]
    double rho      = 870.0;     // oil density [kg/m^3]
    double C_le     = 5.0e-13;   // cross-port laminar leakage [m^3/(s.Pa)] (DQ residual)

    // Supply
    double P_s      = 6.0e6;     // supply pressure [Pa] (60 bar)
    double P_r      = 0.0;       // tank pressure [Pa]

    // Valves: rated flow at rated pressure drop (Liu Eq. 14 normalisation).
    // PDCV2 rated flow scaled by k_v2/k_v1 = 0.0065/0.0097 (Chen Sec. 2, Eq. 7).
    double Q_nom1   = 6.667e-4;  // PDCV1 rated flow [m^3/s] (40 L/min)
    double Q_nom2   = 4.467e-4;  // PDCV2 rated flow [m^3/s] (~26.8 L/min)
    double DP_nom   = 3.5e6;     // rated pressure differential [Pa]

    // Valve spool dynamics (Chen Eq. 7)
    double w_v1     = 86.2;      // PDCV1 natural frequency [rad/s]
    double xi_v1    = 0.70;      // PDCV1 damping ratio
    double w_v2     = 91.4;      // PDCV2 natural frequency [rad/s]
    double xi_v2    = 0.68;      // PDCV2 damping ratio
    double u_max    = 10.0;      // valve command range [V] (xv = u/u_max at DC)

    // Friction (Chen Sec. 5.1 identified Stribeck) - hard-coded in frictionForce()
    double v_eps    = 5.0e-3;    // static-band regularisation [m/s]

    // Backpressure objective
    double P_bd     = 2.0e6;     // desired off-side backpressure [Pa] (20 bar)

    // Pressure clamps
    double P_min    = 0.0;       // vapor limit [Pa]
    double P_max    = 5.0e7;     // burst clamp [Pa]

    // Simulation
    double Ts       = 1.0e-3;    // controller sample time [s]
    int    n_sub    = 4;         // RK4 substeps per Ts
    double duration = 8.0;       // default duration [s]

    static PlantParams fromJson(const std::string& path);
};

struct ScenarioConfig {
    std::string id;
    std::string description;
    double duration_s = 8.0;

    // Initial conditions
    double x_init  = 0.05;       // [m]
    double P1_init = 2.5e6;      // [Pa]
    double P2_init = 2.0e6;      // [Pa]

    // Reference profile: "step" | "hold" | "paper_sine"
    //   step:       x_ref = x_ref_0, then x_ref_1 from t_step
    //   hold:       x_ref = x_ref_0
    //   paper_sine: x_ref = 0.25 + 0.25*sin(pi*t/2 - pi/2)   (Chen Sec. 5)
    std::string ref_type = "step";
    double x_ref_0 = 0.05;
    double x_ref_1 = 0.25;
    double t_step  = 0.5;

    // External load force [N] (positive resists extension): F_0, then F_1 from t_F_step
    double F_0      = 0.0;
    double F_1      = 0.0;
    double t_F_step = -1.0;

    double xRefAt(double t) const;
    double fExtAt(double t) const;

    static ScenarioConfig fromJson(const std::string& path);
};

class SmismoPlant {
public:
    using State = Eigen::Matrix<double, 8, 1>;

    explicit SmismoPlant(const PlantParams& p);

    // Advance one controller period Ts (RK4 with n_sub substeps).
    // Also accumulates supply energy E += P_s * Q_s * dt.
    void step(double u1, double u2, double F_ext);

    const State& state() const { return x_; }
    double x_L() const { return x_(0); }
    double v_L() const { return x_(1); }
    double P1()  const { return x_(2); }
    double P2()  const { return x_(3); }
    double xv1() const { return x_(4); }
    double xv2() const { return x_(6); }

    // Supply flow currently drawn [m^3/s] and cumulative hydraulic energy [J]
    double Q_supply() const { return Q_s_last_; }
    double energy()   const { return energy_J_; }

    // Dynamic supply pressure (DOBEnergyCtrl adjusts this each step).
    // Other controllers leave it unchanged (defaults to p_.P_s at construction).
    void   setSupplyPressure(double P_s_pa) { P_s_dyn_ = P_s_pa; }
    double supplyPressure()  const { return P_s_dyn_; }

    // Signed orifice flows at the current state [m^3/s]
    double Q1() const;
    double Q2() const;

    // Identified Stribeck friction force [N] (regularised at |v| < v_eps)
    double frictionForce(double v) const;

    void reset(double x_init, double P1_init, double P2_init);
    const PlantParams& params() const { return p_; }

private:
    PlantParams p_;
    State       x_;
    double      Q_s_last_ = 0.0;
    double      energy_J_ = 0.0;
    double      P_s_dyn_  = 0.0;  // set to p_.P_s in constructor; adjustable via setSupplyPressure()

    // Signed, regularised orifice characteristic: dp/sqrt(|dp|+eps) ~ sgn(dp)*sqrt(|dp|)
    double flowSqrt(double dp) const;
    double orificeFlow(double xv, double P_chamber, double Q_nom) const;

    State deriv(const State& x, double u1, double u2, double F_ext) const;
};

} // namespace smismo
