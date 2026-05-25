#include "DiscreteADRC.h"
#include <algorithm>
#include <cassert>
#include <iostream>

namespace ctrl
{

    DiscreteADRC::DiscreteADRC(const ADRCParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime), r_(0.0), r_was_set_(false)
    {
        // Backward-Euler ESO is A-stable: stable for all omega_o * Ts > 0.
        // No stability-limit check needed (unlike the previous Forward-Euler formulation).
        updateGains();
        reset();
    }

    void DiscreteADRC::updateGains()
    {
        beta1_ = 3.0 * p_.omega_o;
        beta2_ = 3.0 * p_.omega_o * p_.omega_o;
        beta3_ = p_.omega_o * p_.omega_o * p_.omega_o;
    }

    void DiscreteADRC::setParams(const ADRCParams &p)
    {
        p_ = p;
        updateGains();
    }

    // ESO backward-Euler update (A-stable for all omega_o * Ts > 0).
    //
    // Continuous ESO in matrix form:  dz/dt = Ae z + Be_eps * eps + Be_u * u
    //   Ae  = [[0,1,0],[0,0,1],[0,0,0]]   (nilpotent, Ae^3 = 0)
    //   Be_eps = [beta1, beta2, beta3]'
    //   Be_u   = [0, b0, 0]'
    //   eps = y - z1  (observer error, computed from the current z1)
    //
    // Backward Euler: (I - Ts.Ae) z_new = z + Ts.(Be_eps.eps + Be_u.u)
    //
    // Since Ae is strictly lower-shift, (I - Ts.Ae) is unit upper-triangular and
    // its inverse is available analytically:
    //   (I - Ts.Ae)^{-1} = [[1, Ts, Ts^2],[0, 1, Ts],[0, 0, 1]]
    //
    // eps uses the OLD z1 (semi-implicit); this gives unconditional A-stability at
    // the cost of one step of phase lag in the observer error - negligible for
    // omega_o * Ts << 1 and far better than the forward-Euler instability risk.
    //
    // PD + disturbance cancellation (unchanged from continuous parameterisation):
    //   u0 = omega_c^2.(r - z1) - 2.omega_c.z2
    //   u  = (u0 - z3) / b0
    double DiscreteADRC::computeTracking(double y, double r)
    {
        if (!std::isfinite(y) || !std::isfinite(r))
            return u_prev_;

        // Observer error (uses current z1 - semi-implicit)
        const double eps = y - z_(0);

        // RHS of (I - Ts.Ae) z_new = z + Ts.(Be_eps.eps + Be_u.u)
        const double rhs1 = z_(0) + Ts_ * beta1_ * eps;
        const double rhs2 = z_(1) + Ts_ * (beta2_ * eps + p_.b0 * u_prev_);
        const double rhs3 = z_(2) + Ts_ * beta3_ * eps;

        // Apply (I - Ts.Ae)^{-1} via back-substitution (upper triangular, unit diagonal)
        const double z3n = rhs3;
        const double z2n = rhs2 + Ts_ * z3n;
        const double z1n = rhs1 + Ts_ * z2n;  // = rhs1 + Ts*rhs2 + Ts^2*rhs3
        z_ << z1n, z2n, z3n;

        // PD setpoint control + disturbance cancellation
        const double u0 = p_.omega_c * p_.omega_c * (r - z_(0)) - 2.0 * p_.omega_c * z_(1);
        double u = (u0 - z_(2)) / p_.b0;
        u = std::max(p_.uMin, std::min(p_.uMax, u));

        u_prev_ = u;
        return u;
    }

    // IController contract: signal = error = r - y.
    // Recover y from the stored reference and the error so computeTracking() receives
    // the raw plant output it expects.  This keeps the ESO semantics intact while making
    // compute() composable inside ControllerStack without manual sign management.
    double DiscreteADRC::compute(double error)
    {
        // Guard: if setReference() was never called, r_ == 0 and the controller will
        // silently drive y to 0 instead of the intended setpoint.
        assert(r_was_set_ &&
               "DiscreteADRC::compute() called without setReference(). "
               "Call setReference(r) once per cycle before compute(error), "
               "or use computeTracking(y, r) directly.");
        return computeTracking(r_ - error, r_);
    }

    void DiscreteADRC::reset()
    {
        z_.setZero();
        u_prev_ = 0.0;
        r_ = 0.0;
        r_was_set_ = false;
    }

} // namespace ctrl
