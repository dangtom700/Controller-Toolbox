#include "stewart_plant.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <cmath>

namespace stewart {

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// PlantParams::fromJson
// ---------------------------------------------------------------------------
PlantParams PlantParams::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open plant JSON: " + path);
    json j; f >> j;

    PlantParams p;
    if (j.contains("Ra"))                  p.Ra                  = j["Ra"].get<double>();
    if (j.contains("Rb"))                  p.Rb                  = j["Rb"].get<double>();
    if (j.contains("delta_base_deg"))      p.delta_base_deg      = j["delta_base_deg"].get<double>();
    if (j.contains("delta_platform_deg"))  p.delta_platform_deg  = j["delta_platform_deg"].get<double>();
    if (j.contains("platform_phase_deg"))  p.platform_phase_deg  = j["platform_phase_deg"].get<double>();
    if (j.contains("z0_mid"))              p.z0_mid              = j["z0_mid"].get<double>();
    if (j.contains("m_rod"))               p.m_rod               = j["m_rod"].get<double>();
    if (j.contains("k_spring"))            p.k_spring            = j["k_spring"].get<double>();
    if (j.contains("b_damp"))              p.b_damp              = j["b_damp"].get<double>();
    if (j.contains("F_rod_max"))           p.F_rod_max           = j["F_rod_max"].get<double>();
    if (j.contains("F_eq"))                p.F_eq                = j["F_eq"].get<double>();
    if (j.contains("m_platform"))          p.m_platform          = j["m_platform"].get<double>();
    if (j.contains("comm_delay_samples"))  p.comm_delay_samples  = j["comm_delay_samples"].get<int>();
    if (j.contains("Ts"))                  p.Ts                  = j["Ts"].get<double>();
    if (j.contains("duration"))            p.duration            = j["duration"].get<double>();

    if (j.contains("workspace")) {
        const auto& w = j["workspace"];
        if (w.contains("surge_max"))     p.workspace.surge_max     = w["surge_max"].get<double>();
        if (w.contains("sway_max"))      p.workspace.sway_max      = w["sway_max"].get<double>();
        if (w.contains("heave_max"))     p.workspace.heave_max     = w["heave_max"].get<double>();
        if (w.contains("roll_max_deg"))  p.workspace.roll_max_deg  = w["roll_max_deg"].get<double>();
        if (w.contains("pitch_max_deg")) p.workspace.pitch_max_deg = w["pitch_max_deg"].get<double>();
        if (w.contains("yaw_max_deg"))   p.workspace.yaw_max_deg   = w["yaw_max_deg"].get<double>();
    }
    return p;
}

// ---------------------------------------------------------------------------
// rpyRotation - ZYX roll-pitch-yaw
// ---------------------------------------------------------------------------
Eigen::Matrix3d rpyRotation(double phi, double theta, double psi)
{
    const double cphi = std::cos(phi),   sphi = std::sin(phi);
    const double cth  = std::cos(theta), sth  = std::sin(theta);
    const double cpsi = std::cos(psi),   spsi = std::sin(psi);

    Eigen::Matrix3d R;
    R(0,0) = cpsi*cth;
    R(0,1) = cpsi*sth*sphi - spsi*cphi;
    R(0,2) = cpsi*sth*cphi + spsi*sphi;
    R(1,0) = spsi*cth;
    R(1,1) = spsi*sth*sphi + cpsi*cphi;
    R(1,2) = spsi*sth*cphi - cpsi*sphi;
    R(2,0) = -sth;
    R(2,1) = cth*sphi;
    R(2,2) = cth*cphi;
    return R;
}

// ---------------------------------------------------------------------------
// StewartGeometry
// ---------------------------------------------------------------------------
StewartGeometry::StewartGeometry(const PlantParams& p)
{
    const double db = p.delta_base_deg * DEG2RAD;
    const double dp = p.delta_platform_deg * DEG2RAD;
    const double phase = p.platform_phase_deg * DEG2RAD;

    for (int k = 0; k < 3; ++k) {
        const double base_center = (2.0 * PI / 3.0) * k;
        const double plat_center = phase + (2.0 * PI / 3.0) * k;

        const double alpha_lo = base_center - db;
        const double alpha_hi = base_center + db;
        const double beta_lo  = plat_center - dp;
        const double beta_hi  = plat_center + dp;

        const int i_lo = 2 * k;
        const int i_hi = 2 * k + 1;

        A_(i_lo, 0) = p.Ra * std::cos(alpha_lo); A_(i_lo, 1) = p.Ra * std::sin(alpha_lo); A_(i_lo, 2) = 0.0;
        A_(i_hi, 0) = p.Ra * std::cos(alpha_hi); A_(i_hi, 1) = p.Ra * std::sin(alpha_hi); A_(i_hi, 2) = 0.0;

        B_(i_lo, 0) = p.Rb * std::cos(beta_lo); B_(i_lo, 1) = p.Rb * std::sin(beta_lo); B_(i_lo, 2) = 0.0;
        B_(i_hi, 0) = p.Rb * std::cos(beta_hi); B_(i_hi, 1) = p.Rb * std::sin(beta_hi); B_(i_hi, 2) = 0.0;
    }

    // Neutral length at the home pose (P=[0,0,z0_mid], R=I).
    const Eigen::Vector3d P_home(0.0, 0.0, p.z0_mid);
    for (int i = 0; i < N_RODS; ++i) {
        const Eigen::Vector3d l_vec = P_home + B_.row(i).transpose() - A_.row(i).transpose();
        l0_(i) = l_vec.norm();
    }
}

void StewartGeometry::ikAndJacobian(const PoseRef& pose, Vec6& L_cmd, Mat6& J) const
{
    const Eigen::Matrix3d R = rpyRotation(pose.rpy(0), pose.rpy(1), pose.rpy(2));

    for (int i = 0; i < N_RODS; ++i) {
        const Eigen::Vector3d r_i = R * B_.row(i).transpose();           // platform hinge, base frame
        const Eigen::Vector3d l_vec = pose.P + r_i - A_.row(i).transpose();
        const double L_i = l_vec.norm();
        L_cmd(i) = L_i;

        const Eigen::Vector3d n_i = (L_i > 1e-9) ? (l_vec / L_i) : Eigen::Vector3d(0.0, 0.0, 1.0);
        const Eigen::Vector3d moment = r_i.cross(n_i);

        J(i, 0) = n_i(0); J(i, 1) = n_i(1); J(i, 2) = n_i(2);
        J(i, 3) = moment(0); J(i, 4) = moment(1); J(i, 5) = moment(2);
    }
}

// ---------------------------------------------------------------------------
// StewartPlant
// ---------------------------------------------------------------------------
StewartPlant::StewartPlant(const PlantParams& p)
    : p_(p), geom_(p)
{
    reset();
}

void StewartPlant::reset()
{
    const Vec6& l0 = geom_.l0();
    for (int i = 0; i < N_RODS; ++i) {
        x_(2*i)     = l0(i);
        x_(2*i + 1) = 0.0;
    }
}

Vec12 StewartPlant::dynamics(const Vec12& x, const Vec6& u, const Vec6& F_load) const
{
    Vec12 dx;
    const Vec6& l0 = geom_.l0();
    for (int i = 0; i < N_RODS; ++i) {
        const double L_i  = x(2*i);
        const double dL_i = x(2*i + 1);
        const double ddL_i = (u(i) - p_.k_spring * (L_i - l0(i)) - p_.b_damp * dL_i - F_load(i)) / p_.m_rod;
        dx(2*i)     = dL_i;
        dx(2*i + 1) = ddL_i;
    }
    return dx;
}

void StewartPlant::step(const Vec6& u_in, const Vec6& F_load)
{
    Vec6 u = u_in;
    for (int i = 0; i < N_RODS; ++i)
        u(i) = std::clamp(u(i), -p_.F_rod_max, p_.F_rod_max);

    const double h = p_.Ts;
    const Vec12 k1 = dynamics(x_,                u, F_load);
    const Vec12 k2 = dynamics(x_ + 0.5*h*k1, u, F_load);
    const Vec12 k3 = dynamics(x_ + 0.5*h*k2, u, F_load);
    const Vec12 k4 = dynamics(x_ + h*k3,     u, F_load);
    x_ += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

Vec6 StewartPlant::length() const
{
    Vec6 L;
    for (int i = 0; i < N_RODS; ++i) L(i) = x_(2*i);
    return L;
}

Vec6 StewartPlant::velocity() const
{
    Vec6 dL;
    for (int i = 0; i < N_RODS; ++i) dL(i) = x_(2*i + 1);
    return dL;
}

} // namespace stewart
