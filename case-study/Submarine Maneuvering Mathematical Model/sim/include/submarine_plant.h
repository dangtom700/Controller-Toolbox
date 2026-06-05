#pragma once
#include <cmath>
#include <string>

// MARIN BB2 submarine maneuvering model (model scale: L=3.826 m, U=1.4 m/s).
// Source: Lee & Ahn, "A study on a physical based manoeuvring mathematical
//         model for submarines", Ocean Engineering 311 (2024) 118839.
// Hydrodynamic coefficients re-dimensionalised from Table 11 (Taylor series,
// combined horizontal + vertical 6-DOF form).
// Euler integration at Ts provided by the simulation loop.

namespace sub {

// ---------------------------------------------------------------------------
// Plant parameters
// ---------------------------------------------------------------------------

struct PlantParams {
    // Geometry & mass  (model scale 1:18.348, L=3.826 m)
    double L   = 3.826;   // ship length [m]
    double m   = 737.0;   // displacement mass [kg]  (nabla=0.719 m^3, rho=1025 kg/m^3)
    double Iyy = 300.0;   // pitch moment of inertia [kg.m^2]
    double Izz = 300.0;   // yaw moment of inertia [kg.m^2]
    double U   = 1.4;     // design forward speed [m/s]
    double g   = 9.81;    // gravitational acceleration [m/s^2]
    double BG  = 0.100;   // VCB - VCG [m] (positive = metacentric stability)

    // Added mass - potential-flow cylinder estimate, correction factor C_a=0.8
    double m22 = 1000.0;  // sway added mass [kg]
    double m33 = 1000.0;  // heave added mass [kg]
    double m55 = 1250.0;  // pitch added inertia [kg.m^2]
    double m66 = 1250.0;  // yaw added inertia [kg.m^2]

    // Effective inertia (mass + added mass)
    double A22() const { return m + m22; }   // effective sway mass [kg]
    double A33() const { return m + m33; }   // effective heave mass [kg]
    double A55() const { return Iyy + m55; } // effective pitch inertia [kg.m^2]
    double A66() const { return Izz + m66; } // effective yaw inertia [kg.m^2]

    // Horizontal plane hydrodynamic derivatives (dimensional)
    // Re-dimensionalised from Table 11: Y_coeff = Y'*0.5*rho*L^n*U^p per convention
    double Yv  = -586.0;  // [N/(m/s)]       Y'_v  = -0.0557
    double Yr  = -346.0;  // [N/(rad/s)]     Y'_r  = -0.0086  (hydrodynamic only)
    double Yvv = -734.0;  // [N/(m/s)^2]      Y'_v|v| = -0.0978
    double Yrr = -209.0;  // [N/(rad/s)^2]    Y'_r|r| = -0.0019
    double Nv  = -696.0;  // [N.m/(m/s)]     N'_v  = -0.0173
    double Nr  = -895.0;  // [N.m/(rad/s)]   N'_r  = -0.0058  (sign -> yaw damping)
    double Nvv =  391.0;  // [N.m/(m/s)^2]    N'_v|v| = +0.0136
    double Nrr =  378.0;  // [N.m/(rad/s)^2]  N'_r|r| = +0.0009

    // Rudder effectiveness - positive deltar produces a starboard turn (ψ increasing)
    double Ydr = -200.0;  // [N/rad]          lateral force from rudder
    double Ndr = 1014.0;  // [N.m/rad]        yaw moment from rudder

    // Vertical plane hydrodynamic derivatives (dimensional, from Table 11)
    double Zw  = -290.0;  // [N/(m/s)]        Z'_w  = -0.02761
    double Zq  =  453.0;  // [N/(rad/s)]      Z'_q  = +0.01125  (hydrodynamic)
    double Zww = -405.0;  // [N/(m/s)^2]       Z'_w|w| = -0.05398
    double Mw  =  350.0;  // [N.m/(m/s)]      M'_w  = +0.00871
    double Mq  = -877.0;  // [N.m/(rad/s)]    M'_q  = -0.00569  (sign -> pitch damping)
    double Mqq = -93.0;   // [N.m/(rad/s)^2]   M'_q|q| = -0.00022

    // Stern plane effectiveness - positive deltas produces nose-down pitch (theta increases)
    double Zds = -200.0;  // [N/rad]           vertical force from stern plane
    double Mds =  400.0;  // [N.m/rad]         pitch moment from stern plane

    // External disturbance (set per scenario)
    double dist_Y = 0.0;  // constant lateral sway force [N] (e.g. ocean current)
};

// ---------------------------------------------------------------------------
// Plant state
// ---------------------------------------------------------------------------

struct SubState {
    double v   = 0.0;  // sway velocity [m/s]
    double r   = 0.0;  // yaw rate [rad/s]
    double psi = 0.0;  // heading angle [rad]  (positive = starboard = clockwise from above)
    double w   = 0.0;  // heave velocity [m/s]
    double q   = 0.0;  // pitch rate [rad/s]
    double tht = 0.0;  // pitch angle [rad]    (positive = nose down)
    double z   = 0.0;  // depth [m]            (positive down)
    double x   = 0.0;  // surge position [m]
    double y   = 0.0;  // sway position [m]
};

// ---------------------------------------------------------------------------
// Step function - Euler integration
// ---------------------------------------------------------------------------

// Advance one time step dt.
// delta_r : rudder angle  [rad], positive -> starboard turn (clamped +/-0.611 rad)
// delta_s : stern-plane angle [rad], positive -> nose down  (clamped +/-0.611 rad)
SubState step(const SubState& s, double delta_r, double delta_s,
              const PlantParams& p, double dt);

} // namespace sub
