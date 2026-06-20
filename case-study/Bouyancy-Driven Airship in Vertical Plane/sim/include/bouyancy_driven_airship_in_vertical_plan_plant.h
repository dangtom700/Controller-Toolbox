#pragma once
// bouyancy_driven_airship_in_vertical_plan_plant.h - "liberated center point" airship plant
// (Wu, Moog, Marquez-Martinez, Hu (2013), Aerospace Science and Technology 26, 138-152,
// Eq. (35), Sec. 4.3). See HANDOFF_PROMPT.md / README.md for the full derivation.
#include <Eigen/Dense>
#include <string>

namespace bouyancydrivenairshipinverticalplan {

// State: x = [theta, q, rp1, w, v1, v3]
//   theta [rad]  pitch angle
//   q     [rad/s] pitch rate (= d theta/dt)
//   rp1   [m]    moving-mass position along the body x-axis
//   w     [m/s]  = d rp1/dt
//   v1    [m/s]  body-frame forward velocity (ballistic)
//   v3    [m/s]  body-frame heave velocity (ballistic)
using State = Eigen::Matrix<double, 6, 1>;
enum StateIndex { THETA = 0, Q = 1, RP1 = 2, W = 3, V1 = 4, V3 = 5 };

// Plant parameters loaded from config/plant_params.json (nlohmann/json).
// Table 1 of the paper plus assumed engineering limits (see README "Model simplifications").
struct PlantParams {
    double m_bar = 30.0;     // moving (slider) mass [kg]
    double ms    = 269.0;    // hull mass [kg]
    double J     = 8000.0;   // pitch moment of inertia [kg m^2]
    double rp3   = 2.0;      // slider lever arm above/below hull centerline [m]
    double g     = 9.81;     // gravity [m/s^2]

    double u_max    =  400.0; // actuator force limit [N] (assumed; paper's own u peaks ~150-220 N)
    double u_min    = -400.0;
    double rp1_max  =  1.5;   // slider track limit [m] (assumed)
    double rp1_min  = -1.5;

    double Ts       = 0.05;   // sample time [s] (assumed; slow mechanical plant, see README)
    double duration  = 60.0;  // default scenario duration [s]

    static PlantParams fromJson(const std::string& path);
};

// rho1 (fixed-center pitch accel) / sigma1 (fixed-center slider accel) - paper's Eq. (24),
// Sec. 4.2. Shared closed form used both by the full liberated-center model (35) below and
// by the 4-state fixed-center design model used to linearize LQR/MPC (see controllers.cpp).
struct RhoSigma { double rho1; double sigma1; };
RhoSigma rhoSigma(const PlantParams& p, double theta, double q, double rp1, double w, double u);

// Full 6-state liberated-center ODE (paper's Eq. 35): xdot = f(x, u, m0).
// m0 [kg] is the net-lift parameter (paper's "ballistic" imbalance term); piecewise-constant
// per scenario (see simulation_runner.h for the sawtooth bang-bang extension, Sec. 4.6).
State ode(const PlantParams& p, const State& x, double u, double m0);

// Fixed-center 4-state subsystem (paper's Eq. 24, Sec. 4.2): z = [theta, q, rp1, w],
// zdot = [q, rho1, w, sigma1]. Used as the LQR/MPC design model (no v1/v3 - see README
// "Trim point").
Eigen::Vector4d fixedCenterOde(const PlantParams& p, const Eigen::Vector4d& z, double u);

// Equilibrium actuator force at (theta_ref, rp1_ref): the fixed-center subsystem's own
// theta_ddot=0 condition at q=w=0 (README "Trim point").
double trimInput(const PlantParams& p, double theta_ref, double rp1_ref);

// Discrete-time plant: x[k+1] = RK4(f, x[k], u[k], m0[k]); y = theta.
class Plant {
public:
    explicit Plant(const PlantParams& p);

    // v1_0/v3_0 default to 0; rp1_0 should match the scenario's rp1_ref (see README).
    void reset(double theta0, double rp1_0, double v1_0 = 0.0, double v3_0 = 0.0);

    // Advance one Ts via RK4. u is clamped to [u_min, u_max]; rp1 is clamped post-step to
    // [rp1_min, rp1_max] as a defensive mechanical-stop guard (see Plant::step in the .cpp).
    void step(double u, double m0);

    // Directly overwrite the full state - used by the simulation runner to inject the s04
    // transient pitch-rate disturbance (not part of the plant's own dynamics).
    void setState(const State& x) { x_ = x; }

    double output() const { return x_(THETA); }
    const State& state() const { return x_; }
    const PlantParams& params() const { return p_; }

private:
    PlantParams p_;
    State x_;
};

}  // namespace bouyancydrivenairshipinverticalplan
