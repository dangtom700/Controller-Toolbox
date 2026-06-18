#include "linearizer.h"
#include <cmath>
#include <stdexcept>

namespace boiler {

ctrl::StateSpace linearize(const OperatingPoint& op, double Ts)
{
    const double x1 = op.x1;
    const double x3 = op.x3;
    const double u2 = op.u2;

    const double x1_18 = std::pow(x1, 1.0 / 8.0);
    const double x1_98 = std::pow(x1, 9.0 / 8.0);

    // -- Continuous Ac (df/dx) ------------------------------------------------
    Eigen::Matrix3d Ac = Eigen::Matrix3d::Zero();
    Ac(0, 0) = -0.0018 * u2 * (9.0 / 8.0) * x1_18;
    Ac(1, 0) = (0.073 * u2 - 0.016) * (9.0 / 8.0) * x1_18;
    Ac(1, 1) = -0.1;
    Ac(2, 0) = -(1.1 * u2 - 0.19) / 85.0;

    // -- Continuous Bc (df/du) ------------------------------------------------
    Eigen::Matrix3d Bc = Eigen::Matrix3d::Zero();
    Bc(0, 0) = 0.9;
    Bc(0, 1) = -0.0018 * x1_98;
    Bc(0, 2) = -0.15;
    Bc(1, 1) = 0.073 * x1_98;
    Bc(2, 1) = -1.1 * x1 / 85.0;
    Bc(2, 2) = 141.0 / 85.0;

    // -- Continuous Cc (dy/dx) - analytical Jacobian of y3(x,u) --------------
    // acs = numer / denom
    //   numer = (1 - 0.001538*x3)*0.8*x1 - 25.6
    //   denom = x3*(1.0394 - 0.0012304*x1)
    const double numer  = (1.0 - 0.001538 * x3) * 0.8 * x1 - 25.6;
    const double denom  = x3 * (1.0394 - 0.0012304 * x1);
    const double denom2 = denom * denom;

    const double dn_dx1 = 0.8 * (1.0 - 0.001538 * x3);
    const double dn_dx3 = -0.001538 * 0.8 * x1;
    const double dd_dx1 = -0.0012304 * x3;
    const double dd_dx3 =  1.0394 - 0.0012304 * x1;

    const double dacs_dx1 = (dn_dx1 * denom - numer * dd_dx1) / denom2;
    const double dacs_dx3 = (dn_dx3 * denom - numer * dd_dx3) / denom2;

    // y3 = 0.05*(0.13073*x3 + 100*acs + qe/9 - 67.975)
    // qe = (0.854*u2 - 0.147)*x1 + 45.59*u1 - 2.514*u3 - 2.096
    Eigen::Matrix3d Cc = Eigen::Matrix3d::Zero();
    Cc(0, 0) = 1.0;
    Cc(1, 1) = 1.0;
    Cc(2, 0) = 0.05 * (100.0 * dacs_dx1 + (0.854 * u2 - 0.147) / 9.0);
    Cc(2, 2) = 0.05 * (0.13073 + 100.0 * dacs_dx3);

    // -- Continuous Dc (dy/du) ------------------------------------------------
    Eigen::Matrix3d Dc = Eigen::Matrix3d::Zero();
    Dc(2, 0) = 0.05 * 45.59 / 9.0;
    Dc(2, 1) = 0.05 * 0.854 * x1 / 9.0;
    Dc(2, 2) = -0.05 * 2.514 / 9.0;

    // -- Discretise via ZOH ---------------------------------------------------
    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);   // Ts=0 -> continuous
    return ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);
}

Eigen::Matrix3d computeNbar(const ctrl::StateSpace& ss)
{
    // Nbar = -(Cd . (Ad - I)^{-1} . Bd)^{-1}
    Eigen::Matrix3d AmI = ss.A - Eigen::Matrix3d::Identity();
    Eigen::FullPivLU<Eigen::Matrix3d> lu(AmI);
    if (!lu.isInvertible())
        return Eigen::Matrix3d::Identity();

    Eigen::Matrix3d M = ss.C * lu.inverse() * ss.B;
    Eigen::FullPivLU<Eigen::Matrix3d> lu2(M);
    if (!lu2.isInvertible())
        return Eigen::Matrix3d::Identity();

    return -lu2.inverse();
}

ctrl::StateSpace diagonalChannel(const ctrl::StateSpace& ss, int axis)
{
    // Extract SISO (axis, axis) channel from the MIMO 3x3 system.
    // State is the full 3-state; input is u[axis]; output is y[axis].
    Eigen::MatrixXd B_col = ss.B.col(axis);
    Eigen::MatrixXd C_row = ss.C.row(axis);
    Eigen::MatrixXd D_elem(1, 1);
    D_elem(0, 0) = ss.D(axis, axis);

    return ctrl::StateSpace(ss.A, B_col, C_row, D_elem, ss.Ts);
}

} // namespace boiler
