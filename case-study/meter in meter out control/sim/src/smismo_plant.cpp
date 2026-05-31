#include "smismo_plant.h"
#include <LinearisationHelper.h>
#include <cmath>
#include <algorithm>

namespace smismo {

SmismoPlant::SmismoPlant(const SmismoParams& p, const SmismoOperatingPoint& op)
    : p_(p), op_(op)
{
    reset();
}

void SmismoPlant::reset(double x0_pos)
{
    double xp = (x0_pos < 0.0) ? op_.x_eq : x0_pos;
    x_.setZero();
    x_[0] = xp;
    x_[1] = op_.v_eq;
    x_[2] = op_.p1_eq;
    x_[3] = op_.p2_eq;
    x_[4] = op_.ps_eq;
    x_[5] = op_.xv1_eq;
    x_[6] = 0.0;
    x_[7] = op_.xv2_eq;
    x_[8] = 0.0;
}

double SmismoPlant::orifice(double opening, double dP) const
{
    return p_.Cd * p_.Amax * opening * std::sqrt(2.0 * std::max(dP, 0.0) / p_.rho);
}

Eigen::Matrix<double,9,1> SmismoPlant::f(const Eigen::Matrix<double,9,1>& x,
                                           const Eigen::Matrix<double,2,1>& u) const
{
    const double xpos = x[0];
    const double v    = x[1];
    const double p1   = x[2];
    const double p2   = x[3];
    const double ps   = x[4];
    const double xv1  = x[5];
    const double vv1  = x[6];
    const double xv2  = x[7];
    const double vv2  = x[8];

    // Meter-in valve (3-way): xv1>0 -> supply->p1, xv1<0 -> p1->tank
    double qmi = (xv1 >= 0.0)
        ? orifice(xv1, ps - p1)
        : -orifice(-xv1, p1 - p_.Pt);

    // Meter-out valve (3-way): xv2>0 -> p2->tank, xv2<0 -> supply->p2
    double qmo = (xv2 >= 0.0)
        ? orifice(xv2, p2 - p_.Pt)
        : -orifice(-xv2, ps - p2);

    // Supply demand terms
    double q_from_supply = (xv1 >= 0.0 ? orifice(xv1, ps - p1) : 0.0)
                         + (xv2 <  0.0 ? orifice(-xv2, ps - p2) : 0.0);

    // Relief valve
    double qr = p_.Kr * std::max(ps - p_.Pr, 0.0);

    // Volumes
    double V1n = V1(xpos);
    double V2n = V2(xpos);

    Eigen::Matrix<double,9,1> dx;
    dx[0] = v;
    dx[1] = (p_.A1 * p1 - p_.A2 * p2 - p_.Bv * v - p_.F_ext) / p_.M;
    dx[2] = (p_.beta_e / V1n) * (qmi - p_.A1 * v);
    dx[3] = (p_.beta_e / V2n) * (p_.A2 * v - qmo);
    dx[4] = (p_.beta_e / p_.Vs) * (p_.Qp - qr - q_from_supply);
    dx[5] = vv1;
    dx[6] = p_.wn_v * p_.wn_v * (u[0] - xv1) - 2.0 * p_.zeta_v * p_.wn_v * vv1;
    dx[7] = vv2;
    dx[8] = p_.wn_v * p_.wn_v * (u[1] - xv2) - 2.0 * p_.zeta_v * p_.wn_v * vv2;
    return dx;
}

void SmismoPlant::step(double dt, const Eigen::Matrix<double,2,1>& u)
{
    auto k1 = f(x_,               u);
    auto k2 = f(x_ + 0.5*dt*k1,  u);
    auto k3 = f(x_ + 0.5*dt*k2,  u);
    auto k4 = f(x_ + dt    *k3,  u);
    x_ += (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    // Physical limits
    x_[2] = std::max(x_[2], 0.0);
    x_[3] = std::max(x_[3], 0.0);
    x_[4] = std::max(x_[4], 0.0);
    x_[5] = std::clamp(x_[5], -1.0, 1.0);
    x_[7] = std::clamp(x_[7], -1.0, 1.0);
    if (x_[0] <= 0.0)          { x_[0] = 0.0;    x_[1] = std::max(x_[1], 0.0); }
    if (x_[0] >= p_.L)         { x_[0] = p_.L;   x_[1] = std::min(x_[1], 0.0); }
}

Eigen::Vector4d SmismoPlant::measure() const
{
    return Eigen::Vector4d(x_[0], x_[1], x_[2], x_[3]);
}

ctrl::StateSpace SmismoPlant::linearise4(double Ts) const
{
    const SmismoParams& p = p_;
    const SmismoOperatingPoint& op = op_;

    // 4-state SISO: supply pressure constant, valve instantaneous (xv = u_cmd)
    auto f4 = [&p, &op](const Eigen::VectorXd& x4,
                          const Eigen::VectorXd& u1) -> Eigen::VectorXd {
        const double xpos = x4(0), v = x4(1), p1 = x4(2), p2 = x4(3);
        const double u = u1(0);
        const double ps = op.ps_eq;

        const double xv_p = std::max( u, 0.0);
        const double xv_n = std::max(-u, 0.0);

        auto orif = [&](double a, double dP) {
            return p.Cd * p.Amax * a * std::sqrt(2.0 * std::max(dP, 0.0) / p.rho);
        };

        double qmi = orif(xv_p, ps - p1) - orif(xv_n, p1 - p.Pt);
        double qmo = orif(xv_p, p2 - p.Pt) - orif(xv_n, ps - p2);

        const double V1n = p.V10 + p.A1 * xpos;
        const double V2n = p.V20 + p.A2 * (p.L - xpos);

        // Internal cylinder leakage (physically realistic; ensures A has no
        // eigenvalues exactly on the imaginary axis for LQR/TubeMPC design).
        constexpr double Cl_eff = 1e-12;   // m^3/(s.Pa)

        Eigen::VectorXd dx(4);
        dx(0) = v;
        dx(1) = (p.A1*p1 - p.A2*p2 - p.Bv*v - p.F_ext) / p.M;
        dx(2) = (p.beta_e / V1n) * (qmi - p.A1 * v - Cl_eff * p1);
        dx(3) = (p.beta_e / V2n) * (p.A2 * v - qmo - Cl_eff * p2);
        return dx;
    };

    auto h4 = [](const Eigen::VectorXd& x4, const Eigen::VectorXd&) {
        return x4;   // full-state output
    };

    Eigen::VectorXd x0(4), u0(1);
    x0 << op.x_eq, op.v_eq, op.p1_eq, op.p2_eq;
    u0 << op.u1_eq;

    return ctrl::lineariseAtPoint(f4, h4, x0, u0, Ts);
}

double SmismoPlant::accelGain() const
{
    // Effective b0 for the velocity-servo 2-state model  dv/dt = b0*u - (Bv/M)*v.
    // At quasi-static equilibrium the terminal velocity is v_ss = b0 / (Bv/M) = Kv * u,
    // so  b0 = (Bv/M) * Kv  where  Kv = quasi-static velocity gain [m/s / unit].
    //
    // Kv is the average of the meter-in slope (extension) linearised at the op-point.
    // Central-difference gives the symmetric average at u=0.
    const double Kv = p_.Cd * p_.Amax *
        (std::sqrt(2.0*(op_.ps_eq - op_.p1_eq) / p_.rho) +
         std::sqrt(2.0*(op_.p1_eq - p_.Pt)     / p_.rho)) / 2.0 / p_.A1;
    return (p_.Bv / p_.M) * Kv;
}

ctrl::StateSpace SmismoPlant::linearise2(double Ts) const
{
    const double b0 = accelGain();
    const double a  = p_.Bv / p_.M;   // viscous damping coefficient

    // Continuous-time 2-state model: dx = v, dv = b0*u - a*v
    auto f2 = [b0, a](const Eigen::VectorXd& x2, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd dx(2);
        dx(0) = x2(1);
        dx(1) = b0 * u(0) - a * x2(1);
        return dx;
    };

    Eigen::VectorXd x0(2), u0(1);
    x0 << op_.x_eq, op_.v_eq;
    u0 << op_.u1_eq;
    return ctrl::lineariseAtPoint(f2, x0, u0, Ts);
}

} // namespace smismo
