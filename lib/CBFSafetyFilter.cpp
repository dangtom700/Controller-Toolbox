#include "CBFSafetyFilter.h"
#include <algorithm>
#include <cmath>

namespace ctrl {

CBFSafetyFilter::CBFSafetyFilter(std::shared_ptr<IController> nominal,
                                   ScalarFn h, ScalarFn dh_dx,
                                   ScalarFn f0, ScalarFn g,
                                   const Params& params, double Ts)
    : nominal_(std::move(nominal))
    , h_(std::move(h)), dh_dx_(std::move(dh_dx))
    , f0_(std::move(f0)), g_(std::move(g))
    , p_(params), Ts_(Ts)
{
    if (!nominal_)
        throw std::invalid_argument("CBFSafetyFilter: nominal controller must not be null");
}

double CBFSafetyFilter::compute(double error)
{
    // Step 1: nominal controller output
    u_nom_   = nominal_->compute(error);

    // Step 2: evaluate CBF condition
    h_val_      = h_(x_);
    const double dh  = dh_dx_(x_);
    const double f0v = f0_(x_);
    const double gv  = g_(x_);

    // CBF linear constraint (continuous): dh.(f0 + g.u) >= -alpha.(h + slack)
    // => (dh.g).u >= -dh.f0 - alpha.(h + slack)
    // Note: Euler discretisation: dh.(x_{k+1}-x_k)/Ts >= -alpha.h(x_k)
    //   => dh.(f0 + g.u) >= -alpha.h  (continuous form, acceptable for small Ts)

    const double A_cbf = dh * gv;
    const double b_cbf = -dh * f0v - p_.alpha * (h_val_ + p_.slack);

    // Step 3: 1D QP with analytical solution
    // min (u - u_nom)^2  s.t.  A_cbf * u >= b_cbf, u in [uMin, uMax]
    double u_safe = u_nom_;
    cbf_active_   = false;

    if (std::abs(A_cbf) > 1e-12) {
        // Minimum u satisfying CBF: u_min_cbf = b_cbf / A_cbf  (if A_cbf > 0)
        //                           u_max_cbf = b_cbf / A_cbf  (if A_cbf < 0)
        const double u_cbf = b_cbf / A_cbf;

        if (A_cbf > 0.0 && u_nom_ < u_cbf) {
            u_safe     = u_cbf;
            cbf_active_ = true;
        } else if (A_cbf < 0.0 && u_nom_ > u_cbf) {
            u_safe     = u_cbf;
            cbf_active_ = true;
        }
    }

    // Hard box constraint
    u_safe = std::clamp(u_safe, p_.uMin, p_.uMax);
    notifyObserver(u_safe, x_);
    return u_safe;
}

void CBFSafetyFilter::reset()
{
    nominal_->reset();
    x_          = 0.0;
    cbf_active_ = false;
    h_val_      = 0.0;
    u_nom_      = 0.0;
}

} // namespace ctrl
