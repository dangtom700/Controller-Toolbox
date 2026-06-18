#pragma once
#include <Eigen/Dense>
#include <string>

// 6-DOF Stewart platform vessel motion simulator - rod/geometry plant model.
//
// Reference: Ma et al. (2025) Ocean Engineering 334, 121595.
//
// Scope: this header holds ONLY rod/geometry physics (closed-form inverse
// kinematics + velocity Jacobian + per-rod 2nd-order actuator dynamics).
// Wave/sea-state reference generation lives in cfd_input_model.h, kept
// architecturally separate (see README "Implementation Notes").
//
// 12 states: x = [L1,dL1, L2,dL2, L3,dL3, L4,dL4, L5,dL5, L6,dL6]   [m, m/s]
// Input:     u = [u1..u6]   per-rod actuator force [N]
// Disturbance: F_load[6]    per-rod quasi-static load force [N] (from CFDInputModel/runner)
//
// Per-rod dynamics:
//   ddL_i = (u_i - k_spring*(L_i - l0_i) - b_damp*dL_i - F_load_i) / m_rod
//
// Integration: classical RK4 at Ts (no sub-stepping; Ts=5ms is already fast
// relative to the rod time constant sqrt(m_rod/k_spring) ~ 27 ms).

namespace stewart {

static constexpr int N_RODS = 6;
using Vec6  = Eigen::Matrix<double, 6, 1>;
using Mat6  = Eigen::Matrix<double, 6, 6>;
using Vec12 = Eigen::Matrix<double, 12, 1>;

static constexpr double PI       = 3.14159265358979323846;
static constexpr double DEG2RAD  = PI / 180.0;
static constexpr double RAD2DEG  = 180.0 / PI;
static constexpr double GRAVITY  = 9.81; // [m/s^2]

// -- Workspace limits (paper Table 1) ----------------------------------------
struct WorkspaceLimits {
    double surge_max = 0.366; // [m]
    double sway_max  = 0.422; // [m]
    double heave_max = 0.182; // [m] half-range about z0_mid (621-985mm -> +/-182mm)
    double roll_max_deg  = 27.0;
    double pitch_max_deg = 23.0;
    double yaw_max_deg   = 30.0;
};

struct PlantParams {
    // Geometry (paper does not specify exact hinge angles - documented assumption,
    // see README "Implementation Notes" decision #1: 3 hinge pairs per platform,
    // spaced 120 deg apart, each pair split by a half-angle).
    //
    // platform_phase_deg=0 (platform hinge pattern NOT rotated relative to the
    // base pattern) gives every leg the SAME neutral length l0_i regardless of
    // delta_base_deg vs delta_platform_deg, because l0_i depends only on
    // cos(beta_i-alpha_i) and that angle takes the same magnitude (+/-(db-dp))
    // for the "lo"/"hi" leg of every pair - balanced load sharing for free.
    //
    // BUT equal half-angles (delta_base_deg == delta_platform_deg) make every
    // leg a pure rotation of every other leg, which is a genuine kinematic
    // (architecture) singularity of this construction for a pure-vertical
    // load at the home pose: J^T becomes numerically singular and the required
    // per-rod load-support force blows up to absurd values (verified
    // empirically - det(J^T) ~ 1e-49 for every (delta, phase) tried with equal
    // half-angles). A sizeable gap between the two half-angles (~40 deg) avoids
    // this while phase=0 keeps the load perfectly balanced across all 6 legs.
    double Ra = 0.500;               // base hinge distribution radius [m]
    double Rb = 0.350;                // platform hinge distribution radius [m]
    double delta_base_deg     = 5.0;  // base hinge pair half-angle [deg]
    double delta_platform_deg = 45.0; // platform hinge pair half-angle [deg]
    double platform_phase_deg = 0.0;  // platform pattern rotation relative to base [deg]
    double z0_mid = 0.803;            // home/neutral platform height [m] ((0.621+0.985)/2)

    // Rod actuator dynamics (decision #2: u is a direct force in Newtons)
    double m_rod     = 15.0;   // moving rod mass [kg]
    double k_spring  = 2.0e4;  // parasitic spring stiffness [N/m]
    double b_damp    = 800.0;  // viscous damping [N.s/m]
    double F_rod_max = 6000.0; // actuator force limit [N]

    // Load (decision #3)
    double F_eq        = 12000.0; // shipboard equipment load [N]
    double m_platform  = 500.0;   // moving platform mass [kg]

    // Communication delay (decision #5)
    int comm_delay_samples = 1;

    double Ts       = 0.005; // sample time [s]
    double duration = 60.0;  // fallback scenario duration [s]

    WorkspaceLimits workspace;

    static PlantParams fromJson(const std::string& path);
};

struct PoseRef {
    Eigen::Vector3d P{0.0, 0.0, 0.0};   // [x,y,z] translation [m]
    Eigen::Vector3d rpy{0.0, 0.0, 0.0}; // [phi,theta,psi] [rad]
};

// ZYX roll-pitch-yaw rotation matrix (Response 2's convention).
Eigen::Matrix3d rpyRotation(double phi, double theta, double psi);

// Closed-form inverse kinematics + velocity Jacobian (paper Eq. 1/2 + virtual-work
// Jacobian rows [n_i^T, (r_i x n_i)^T]). Geometry only - no wave logic.
class StewartGeometry {
public:
    explicit StewartGeometry(const PlantParams& p);

    // Computes L_cmd(6,1) [m] and Jacobian J(6,6) (rows = [n_i^T, (r_i x n_i)^T])
    // for the given platform pose.
    void ikAndJacobian(const PoseRef& pose, Vec6& L_cmd, Mat6& J) const;

    const Vec6& l0() const { return l0_; }

private:
    Eigen::Matrix<double, N_RODS, 3> A_; // base hinges, base frame
    Eigen::Matrix<double, N_RODS, 3> B_; // platform hinges, platform-local frame
    Vec6 l0_;                            // neutral rod lengths at home pose
};

// Per-rod 2nd-order actuator dynamics. No wave/sea-state code lives here.
class StewartPlant {
public:
    explicit StewartPlant(const PlantParams& p);

    void reset();

    // Advances one Ts step via RK4. u and F_load are 6-vectors of force [N];
    // u is clamped to +/-F_rod_max before integrating.
    void step(const Vec6& u, const Vec6& F_load);

    Vec6 length() const;   // L_i [m]
    Vec6 velocity() const; // dL_i [m/s]

    const PlantParams&     params()   const { return p_; }
    const StewartGeometry& geometry() const { return geom_; }

private:
    PlantParams     p_;
    StewartGeometry geom_;
    Vec12           x_; // [L1,dL1,...,L6,dL6]

    Vec12 dynamics(const Vec12& x, const Vec6& u, const Vec6& F_load) const;
};

} // namespace stewart
