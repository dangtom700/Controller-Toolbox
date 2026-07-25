#include "differential_drive_robot_tracking_plant.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace differentialdriverobottracking {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double wrapAngle(double a) {
    if (!std::isfinite(a)) return 0.0;
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0.0) a += 2.0 * kPi;
    return a - kPi;
}

PlantParams PlantParams::fromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j;
    PlantParams p;
    p.M_total     = j.value("M_total",     p.M_total);
    p.r_wheel     = j.value("r_wheel",     p.r_wheel);
    p.R_half_axle = j.value("R_half_axle", p.R_half_axle);
    p.I_A         = j.value("I_A",         p.I_A);
    p.I_0         = j.value("I_0",         p.I_0);
    p.Kf          = j.value("Kf",          p.Kf);
    p.d_com       = j.value("d_com",       p.d_com);

    p.tau_max     = j.value("tau_max",     p.tau_max);
    p.v_max       = j.value("v_max",       p.v_max);
    p.w_max       = j.value("w_max",       p.w_max);

    p.Ts_plant    = j.value("Ts_plant",    p.Ts_plant);
    p.Tf          = j.value("Tf",          p.Tf);
    p.Ts_slow     = j.value("Ts_slow",     p.Ts_slow);

    p.Kp_v        = j.value("Kp_v",        p.Kp_v);
    p.Ki_v        = j.value("Ki_v",        p.Ki_v);
    p.Kp_w        = j.value("Kp_w",        p.Kp_w);
    p.Ki_w        = j.value("Ki_w",        p.Ki_w);
    return p;
}

int PlantParams::slowDivider() const {
    if (!(Tf > 0.0) || !(Ts_slow > 0.0)) return 1;
    return std::max(1, static_cast<int>(std::lround(Ts_slow / Tf)));
}

int PlantParams::plantSubSteps() const {
    if (!(Tf > 0.0) || !(Ts_plant > 0.0)) return 1;
    return std::max(1, static_cast<int>(std::lround(Tf / Ts_plant)));
}

Plant::Plant(const PlantParams& p) : p_(p) {
    // Symmetric inertia matrix M = [[A,B],[B,A]] (paper Sec. "Kinematic and dynamic model").
    const double r2  = p_.r_wheel * p_.r_wheel;
    const double R2  = p_.R_half_axle * p_.R_half_axle;
    const double com = (R2 > 1e-12) ? (p_.I_A + p_.M_total * p_.d_com * p_.d_com) * r2 / (4.0 * R2)
                                    : 0.0;
    const double A = p_.M_total * r2 / 4.0 + com + p_.I_0;
    const double B = p_.M_total * r2 / 4.0 - com;

    // det(M) = A^2 - B^2. Guard it: a singular M means the wheel accelerations are undefined.
    const double det = A * A - B * B;
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        healthy_ = false;
        Minv_.setZero();
    } else {
        Minv_ << A / det, -B / det,
                -B / det,  A / det;
    }

    x_  = Eigen::VectorXd::Zero(N_STATES);
    k1_ = Eigen::VectorXd::Zero(N_STATES);
    k2_ = Eigen::VectorXd::Zero(N_STATES);
    k3_ = Eigen::VectorXd::Zero(N_STATES);
    k4_ = Eigen::VectorXd::Zero(N_STATES);
    xt_ = Eigen::VectorXd::Zero(N_STATES);
}

void Plant::reset(const Eigen::VectorXd& x0) {
    if (x0.size() == N_STATES) x_ = x0;
    else                       x_.setZero();
}

void Plant::resetPose(double X, double Y, double theta) {
    x_.setZero();
    x_(SX)     = X;
    x_(SY)     = Y;
    x_(STHETA) = theta;
}

double Plant::v() const {
    return p_.r_wheel * (x_(SWR) + x_(SWL)) / 2.0;
}

double Plant::w() const {
    if (p_.R_half_axle < 1e-9) return 0.0;
    return p_.r_wheel * (x_(SWR) - x_(SWL)) / (2.0 * p_.R_half_axle);
}

void Plant::derivative(const Eigen::VectorXd& x, double tau_R, double tau_L,
                       Eigen::VectorXd& dx) const {
    const double th = x(STHETA);
    const double vv = p_.r_wheel * (x(SWR) + x(SWL)) / 2.0;
    const double ww = (p_.R_half_axle > 1e-9)
                        ? p_.r_wheel * (x(SWR) - x(SWL)) / (2.0 * p_.R_half_axle)
                        : 0.0;

    const double c = std::cos(th), s = std::sin(th);
    dx(SX)     = vv * c - p_.d_com * ww * s;
    dx(SY)     = vv * s + p_.d_com * ww * c;
    dx(STHETA) = ww;

    // [wR', wL'] = Minv * (tau - Kf*[wR, wL])
    const double fR = tau_R - p_.Kf * x(SWR);
    const double fL = tau_L - p_.Kf * x(SWL);
    dx(SWR) = Minv_(0, 0) * fR + Minv_(0, 1) * fL;
    dx(SWL) = Minv_(1, 0) * fR + Minv_(1, 1) * fL;
}

void Plant::step(double tau_R, double tau_L) {
    // NaN guard: a non-finite torque command freezes the state rather than poisoning it.
    if (!std::isfinite(tau_R) || !std::isfinite(tau_L) || !healthy_) return;

    tau_R = std::clamp(tau_R, -p_.tau_max, p_.tau_max);
    tau_L = std::clamp(tau_L, -p_.tau_max, p_.tau_max);

    const double h = p_.Ts_plant;

    derivative(x_, tau_R, tau_L, k1_);
    xt_ = x_ + 0.5 * h * k1_;
    derivative(xt_, tau_R, tau_L, k2_);
    xt_ = x_ + 0.5 * h * k2_;
    derivative(xt_, tau_R, tau_L, k3_);
    xt_ = x_ + h * k3_;
    derivative(xt_, tau_R, tau_L, k4_);

    x_ += (h / 6.0) * (k1_ + 2.0 * k2_ + 2.0 * k3_ + k4_);
    x_(STHETA) = wrapAngle(x_(STHETA));

    if (!x_.allFinite()) x_.setZero();
}

}  // namespace differentialdriverobottracking
