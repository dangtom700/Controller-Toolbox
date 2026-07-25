#pragma once
// differential_drive_robot_tracking_plant.h - 5-state nonlinear differential-drive mobile
// robot (DDMR) with a non-negligible centre-of-mass offset, per Xu, Maghsoudniazi &
// Maghsoudniazi, Sci. Rep. 16:11961 (2026), doi:10.1038/s41598-026-39667-1, Sec. "Kinematic
// and dynamic model".
//
//   state x = [X, Y, theta, omega_R, omega_L]
//
//   v     = r*(omega_R + omega_L)/2                    body-frame linear velocity  [m/s]
//   omega = r*(omega_R - omega_L)/(2R)                 body-frame angular velocity [rad/s]
//
//   X'     = v*cos(theta) - d*omega*sin(theta)         (paper's kinematic matrix; d = COM offset)
//   Y'     = v*sin(theta) + d*omega*cos(theta)
//   theta' = omega
//   [omega_R', omega_L']^T = Minv * ([tau_R, tau_L]^T - Kf*[omega_R, omega_L]^T)
//
//   M = [[A, B], [B, A]]
//   A = M_t*r^2/4 + (I_A + M_t*d^2)*r^2/(4R^2) + I_0
//   B = M_t*r^2/4 - (I_A + M_t*d^2)*r^2/(4R^2)
//
// M is constant, so Minv is factored ONCE in the constructor and never recomputed - step()
// is allocation-free and does no 2x2 solve.
#include <Eigen/Dense>
#include <string>

namespace differentialdriverobottracking {

// State indices into the 5-vector.
enum StateIdx { SX = 0, SY = 1, STHETA = 2, SWR = 3, SWL = 4 };
inline constexpr int N_STATES = 5;

// Plant parameters loaded from config/plant_params.json (nlohmann/json).
//
// NOTE: the paper never tabulates the robot's physical constants - it only names the Pioneer
// platform. These are documented Pioneer 3-DX values (see README "Plant parameters"), except
// Kf which is CALIBRATED so mean wheel torque lands in the paper's reported 5.9-9.7 N.m band.
struct PlantParams {
    // -- Physical ------------------------------------------------------------
    double M_total     = 9.0;     ///< Total system mass M [kg].
    double r_wheel     = 0.0975;  ///< Wheel radius r [m].
    double R_half_axle = 0.165;   ///< Half the wheelbase R [m].
    double I_A         = 0.16;    ///< Body inertia about the COM [kg m^2].
    double I_0         = 0.005;   ///< Wheel inertia [kg m^2].
    double Kf          = 0.25;    ///< Viscous wheel friction [N m s/rad].
    double d_com       = 0.05;    ///< Wheel-axis-centre to COM distance d [m].

    // -- Actuation limits ----------------------------------------------------
    // v_max must exceed the fastest reference (the a = 3 m circle demands 3.0 m/s) or no
    // controller can ever catch up; the physical ceiling is v_ss = r*tau_max/Kf = 5.85 m/s.
    double tau_max     = 15.0;    ///< Per-wheel torque saturation [N m] (paper Fig. 22).
    double v_max       = 5.0;     ///< Linear velocity command clamp [m/s].
    double w_max       = 6.0;     ///< Angular velocity command clamp [rad/s].

    // -- Timing (three nested rates; see README "Multi-rate structure") -------
    double Ts_plant    = 0.005;   ///< RK4 step and inner PI-loop period [s].
    double Tf          = 0.03;    ///< Fast (outer kinematic) control period [s] - paper's 30 ms.
    double Ts_slow     = 0.15;    ///< Slow adaptation period [s] - paper's 150 ms; eps = Tf/Ts_slow = 0.2.

    // -- Inner PI wheel-velocity loop (paper's "Velocity control and actuation layer") --
    double Kp_v        = 30.0;    ///< Linear-velocity proportional gain.
    double Ki_v        = 60.0;    ///< Linear-velocity integral gain.
    double Kp_w        = 6.0;     ///< Angular-velocity proportional gain.
    double Ki_w        = 12.0;    ///< Angular-velocity integral gain.

    static PlantParams fromJson(const std::string& path);

    /// Fast-ticks per slow-tick, >= 1. eps = Tf/Ts_slow is the paper's time-scale ratio.
    int slowDivider() const;
    /// Plant sub-steps per fast control tick, >= 1.
    int plantSubSteps() const;
};

// Discrete-time plant: RK4 integration of the 5-state continuous model over Ts_plant.
class Plant {
public:
    explicit Plant(const PlantParams& p);

    void reset(const Eigen::VectorXd& x0);
    /// Reset to a pose with both wheels at rest.
    void resetPose(double X, double Y, double theta);

    /// Advance one Ts_plant with the given (already saturated) wheel torques.
    void step(double tau_R, double tau_L);

    const Eigen::VectorXd& state() const { return x_; }
    int    stateSize() const { return N_STATES; }

    double X()      const { return x_(SX); }
    double Y()      const { return x_(SY); }
    double theta()  const { return x_(STHETA); }
    double omegaR() const { return x_(SWR); }
    double omegaL() const { return x_(SWL); }

    /// Body-frame linear velocity v [m/s].
    double v() const;
    /// Body-frame angular velocity omega [rad/s].
    double w() const;

    /// Scalar output kept for interface symmetry with the other studies (X position).
    double output() const { return x_(SX); }

    const PlantParams& params() const { return p_; }
    /// true when the 2x2 inertia matrix M was invertible at construction (A^2 != B^2).
    bool isHealthy() const { return healthy_; }

private:
    void derivative(const Eigen::VectorXd& x, double tau_R, double tau_L,
                    Eigen::VectorXd& dx) const;

    PlantParams     p_;
    Eigen::Matrix2d Minv_;
    bool            healthy_ = true;
    Eigen::VectorXd x_;

    // Pre-allocated RK4 workspace - step() must not allocate.
    mutable Eigen::VectorXd k1_, k2_, k3_, k4_, xt_;
};

/// Wrap an angle to (-pi, pi].
double wrapAngle(double a);

}  // namespace differentialdriverobottracking
