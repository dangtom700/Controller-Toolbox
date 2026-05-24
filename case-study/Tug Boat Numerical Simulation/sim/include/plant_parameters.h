#pragma once
#include <Eigen/Dense>
#include <array>
#include <string>

// Physical parameters for the Li et al. (2026) barge-tugboat system.
// All values are loaded from config/plant_params.json.
// Matrix assembly follows paper Eq. (20)-(21).

namespace tug {

static constexpr int NUM_TUGS = 4;

struct TugStation {
    double x;          // surge offset from barge centre (m)
    double y;          // sway  offset from barge centre (m)
    double alpha;      // nominal push-thruster bearing (rad)
};

struct PlantParameters {
    // -- Barge generalised mass (rigid-body + added mass, 3x3) --------------
    Eigen::Matrix3d M_b;
    Eigen::Matrix3d D_b;   // linear damping

    // -- Per-tug scalars ----------------------------------------------------
    double m_tug;                       // displacement (kg)
    Eigen::Vector3d added_tug;          // [Xu_dot, Yv_dot, Nr_dot] (kg)
    Eigen::Vector3d damp_tug;           // [Xu, Yv, Nr] (N.s/m or N.m.s/rad)
    double T_min;                       // minimum thrust per tug (N)
    double T_max;                       // maximum thrust per tug (N)
    double dT_max;                      // rate limit per tug per step (N)

    std::array<TugStation, NUM_TUGS> stations;

    // -- Reconstructed system matrices -------------------------------------
    Eigen::Matrix3d M_re;   // M_b + sum M_tilde_i
    Eigen::Matrix3d M_re_inv;
    Eigen::Matrix3d D_re;   // D_b + sum D_tilde_i

    // -- Environment -------------------------------------------------------
    double rho_air;
    double rho_water;
    // Barge above-waterline areas
    double frontal_area;    // m^2
    double lateral_area;    // m^2
    double Cwx_max, Cwy_max, Cwpsi_max;
    // Current drag coefficients
    double Cu, Cv, Cpsi;
    double length;          // m
    double beam;            // m
    double draft;           // m

    // -- Wave (JONSWAP 2-component) -----------------------------------------
    double Hs;
    double Tp;
    double gamma_j;
    double drift_x, drift_y, drift_psi;

    // -- Simulation ---------------------------------------------------------
    double dt;
    double duration;

    // Load all parameters from a JSON file.  Assembles M_re and D_re internally.
    static PlantParameters fromJson(const std::string& path);

private:
    void assemble();   // builds M_re, D_re from barge + tug contributions
};

} // namespace tug
