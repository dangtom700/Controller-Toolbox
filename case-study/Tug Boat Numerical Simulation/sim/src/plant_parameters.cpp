#include "plant_parameters.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <stdexcept>

using json = nlohmann::json;
using namespace Eigen;

namespace tug {

PlantParameters PlantParameters::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open: " + path);
    json cfg = json::parse(f);

    PlantParameters p{};

    // -- Barge ----------------------------------------------------------------
    auto& b   = cfg["barge"];
    auto& bam = b["added_mass"];
    auto& bld = b["linear_damping"];
    auto& bwc = b["wind_coeff"];
    auto& bcc = b["current_coeff"];

    double m_b = b["displacement_kg"].get<double>();
    p.length   = b["length_m"].get<double>();
    p.beam     = b["beam_m"].get<double>();
    p.draft    = b["draft_m"].get<double>();

    double L = p.length, B = p.beam;
    double I_zz = (m_b / 12.0) * (L * L + B * B);

    // M_b = diag(m - Xu_dot, m - Yv_dot, Izz - Nr_dot)
    double Xud = bam["Xudot"].get<double>();
    double Yvd = bam["Yvdot"].get<double>();
    double Nrd = bam["Nrdot"].get<double>();
    p.M_b = Matrix3d::Zero();
    p.M_b(0,0) = m_b - Xud;
    p.M_b(1,1) = m_b - Yvd;
    p.M_b(2,2) = I_zz - Nrd;

    // D_b = diag(Xu, Yv, Nr)  (positive values, applied as -D*nu in EOM)
    p.D_b = Matrix3d::Zero();
    p.D_b(0,0) = bld["Xu"].get<double>();
    p.D_b(1,1) = bld["Yv"].get<double>();
    p.D_b(2,2) = bld["Nr"].get<double>();

    p.frontal_area  = bwc["frontal_area_m2"].get<double>();
    p.lateral_area  = bwc["lateral_area_m2"].get<double>();
    p.Cwx_max       = bwc["Cwx_max"].get<double>();
    p.Cwy_max       = bwc["Cwy_max"].get<double>();
    p.Cwpsi_max     = bwc["Cwpsi_max"].get<double>();

    p.Cu   = bcc["Cu"].get<double>();
    p.Cv   = bcc["Cv"].get<double>();
    p.Cpsi = bcc["Cpsi"].get<double>();

    // -- Tugs -----------------------------------------------------------------
    auto& t   = cfg["tugs"];
    auto& tam = t["added_mass"];
    auto& tld = t["linear_damping"];

    p.m_tug = t["displacement_kg"].get<double>();
    p.added_tug = Vector3d(
        tam["Xudot"].get<double>(),
        tam["Yvdot"].get<double>(),
        tam["Nrdot"].get<double>());
    p.damp_tug = Vector3d(
        tld["Xu"].get<double>(),
        tld["Yv"].get<double>(),
        tld["Nr"].get<double>());

    auto limits = t["thrust_limits_kN"].get<std::vector<double>>();
    p.T_min  = limits[0] * 1e3;   // kN -> N
    p.T_max  = limits[1] * 1e3;
    p.dT_max = t["thrust_rate_limit_kN_s"].get<double>() * 1e3 * p.dt; // N per step

    for (int i = 0; i < NUM_TUGS; ++i) {
        auto& s = t["stations"][i];
        p.stations[i] = TugStation{
            s["x"].get<double>(),
            s["y"].get<double>(),
            s["alpha_deg"].get<double>() * M_PI / 180.0
        };
    }

    // -- Fluid -----------------------------------------------------------------
    p.rho_air   = cfg["fluid"]["rho_air"].get<double>();
    p.rho_water = cfg["fluid"]["rho_water"].get<double>();

    // -- Wave -----------------------------------------------------------------
    auto& w   = cfg["wave"];
    p.Hs        = w["Hs_m"].get<double>();
    p.Tp        = w["Tp_s"].get<double>();
    p.gamma_j   = w["gamma"].get<double>();
    p.drift_x   = w["drift_coeff_x"].get<double>();
    p.drift_y   = w["drift_coeff_y"].get<double>();
    p.drift_psi = w["drift_coeff_psi"].get<double>();

    // -- Simulation ------------------------------------------------------------
    p.dt       = cfg["simulation"]["dt_s"].get<double>();
    p.duration = cfg["simulation"]["duration_s"].get<double>();

    // rate limit needs dt, re-compute now that dt is known
    p.dT_max = t["thrust_rate_limit_kN_s"].get<double>() * 1e3 * p.dt;

    p.assemble();
    return p;
}

void PlantParameters::assemble()
{
    // Reconstruct M_re and D_re by projecting each tug onto the barge frame.
    // For tug i at station (xi, yi) with thrust angle alpha_i, the lever-arm
    // vector entering the tug-to-barge projection is (Li et al. Eq. 20):
    //
    //   J_i = [cos(a), sin(a), -yi*cos(a) + xi*sin(a)]^T
    //
    // The tug adds (m_tug + Xudot_t)*J_i*J_i^T to M_re in the surge direction,
    // and similarly for sway/yaw diagonal terms projected via J.
    // Here we use the simplified additive diagonal projection consistent with the
    // GDScript prototype and the Python reference module.

    M_re = M_b;
    D_re = D_b;

    for (int i = 0; i < NUM_TUGS; ++i) {
        double xi = stations[i].x;
        double yi = stations[i].y;
        double a  = stations[i].alpha;
        double ca = std::cos(a);
        double sa = std::sin(a);
        double li = -yi * ca + xi * sa;   // moment arm

        // J = [ca, sa, li]^T  (direction vector of tug thrust mapped to barge DOFs)
        Vector3d J(ca, sa, li);

        // Tug effective masses: surge m_tug+Xu_dot acts along ca direction,
        // sway m_tug+Yv_dot acts along sa, yaw I_tug+Nr_dot acts along li.
        // Diagonal of M_tilde_i = J .* [ms_u, ms_v, ms_r] component-wise outer product
        double ms_u = m_tug + added_tug(0);
        double ms_v = m_tug + added_tug(1);
        double I_tug = m_tug * (xi*xi + yi*yi) / 6.0;   // rough yaw MOI
        double ms_r = I_tug + added_tug(2);

        Matrix3d M_tilde;
        M_tilde(0,0) = ms_u * ca * ca;
        M_tilde(1,1) = ms_v * sa * sa;
        M_tilde(2,2) = ms_r * li * li;
        M_tilde(0,1) = M_tilde(1,0) = ms_u * ca * sa;
        M_tilde(0,2) = M_tilde(2,0) = ms_u * ca * li;
        M_tilde(1,2) = M_tilde(2,1) = ms_v * sa * li;

        Matrix3d D_tilde;
        D_tilde(0,0) = damp_tug(0) * ca * ca;
        D_tilde(1,1) = damp_tug(1) * sa * sa;
        D_tilde(2,2) = damp_tug(2) * li * li;
        D_tilde(0,1) = D_tilde(1,0) = damp_tug(0) * ca * sa;
        D_tilde(0,2) = D_tilde(2,0) = damp_tug(0) * ca * li;
        D_tilde(1,2) = D_tilde(2,1) = damp_tug(1) * sa * li;

        M_re += M_tilde;
        D_re += D_tilde;
    }

    M_re_inv = M_re.inverse();
}

} // namespace tug
