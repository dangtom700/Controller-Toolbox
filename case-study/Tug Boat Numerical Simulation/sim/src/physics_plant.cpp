#include "physics_plant.h"
#include <cmath>

using namespace Eigen;

namespace tug {

PhysicsPlant::PhysicsPlant(const PlantParameters& p) : pp_(p)
{
    X_.setZero();
}

void PhysicsPlant::reset(const Matrix<double,6,1>& X0)
{
    X_ = X0;
}

Matrix3d PhysicsPlant::R(double psi)
{
    double cp = std::cos(psi), sp = std::sin(psi);
    Matrix3d Rot;
    Rot << cp, -sp, 0,
           sp,  cp, 0,
            0,   0, 1;
    return Rot;
}

Matrix3d PhysicsPlant::C_rb(const Vector3d& nu) const
{
    // Barge rigid-body Coriolis (skew-symmetric):
    //   C(nu) = [ 0        0        -(m-Yvd)*v ]
    //           [ 0        0         (m-Xud)*u ]
    //           [ (m-Yvd)*v  -(m-Xud)*u   0    ]
    double mXud = pp_.M_b(0,0);  // m - Xu_dot
    double mYvd = pp_.M_b(1,1);  // m - Yv_dot
    double u = nu(0), v = nu(1);

    Matrix3d C = Matrix3d::Zero();
    C(0,2) = -mYvd * v;
    C(1,2) =  mXud * u;
    C(2,0) =  mYvd * v;
    C(2,1) = -mXud * u;
    return C;
}

Matrix<double,6,1> PhysicsPlant::f(const Matrix<double,6,1>& X,
                                    const Vector3d& tau_total) const
{
    Vector3d eta = X.head<3>();
    Vector3d nu  = X.tail<3>();
    double psi   = eta(2);

    // Kinematics: eta_dot = R(psi) * nu
    Vector3d eta_dot = R(psi) * nu;

    // Coriolis + damping terms (applied as -C*nu - D*nu)
    Vector3d C_nu = C_rb(nu) * nu;
    Vector3d D_nu = pp_.D_re * nu;

    // nu_dot = M_re_inv * (tau_total - C*nu - D*nu)
    Vector3d nu_dot = pp_.M_re_inv * (tau_total - C_nu - D_nu);

    Matrix<double,6,1> Xdot;
    Xdot.head<3>() = eta_dot;
    Xdot.tail<3>() = nu_dot;
    return Xdot;
}

void PhysicsPlant::step(const Vector3d& tau_main, const Vector3d& tau_env)
{
    const double dt = pp_.dt;
    const Vector3d tau = tau_main + tau_env;

    // Classical RK4
    auto k1 = f(X_,              tau);
    auto k2 = f(X_ + (dt/2)*k1, tau);
    auto k3 = f(X_ + (dt/2)*k2, tau);
    auto k4 = f(X_ + dt*k3,     tau);

    X_ += (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    // Clamp any Inf/NaN that a diverging controller can produce to keep the
    // simulation alive. NaN/Inf in psi causes the while-loop wrap to hang.
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(X_(i)))
            X_(i) = 0.0;
    }

    // Wrap heading to (-pi, pi] in O(1) - safe for all finite values.
    X_(2) = std::remainder(X_(2), 2.0 * M_PI);
}

} // namespace tug
