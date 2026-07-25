#include "CascadeController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

CascadeController::CascadeController(std::shared_ptr<IController> outer,
                                     std::shared_ptr<IController> inner,
                                     const CascadeParams &params,
                                     double Ts)
    : outer_(std::move(outer)), inner_(std::move(inner)), p_(params), Ts_(Ts)
{
    if (!outer_ || !inner_)
        throw std::invalid_argument("CascadeController: outer and inner controllers must not be null");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("CascadeController: Ts must be > 0");
    if (p_.outerDecimation < 1)
        throw std::invalid_argument("CascadeController: outerDecimation must be >= 1");
    if (p_.spMin >= p_.spMax)
        throw std::invalid_argument("CascadeController: spMin must be < spMax");

    // Cache the inner convention once: querying it per step would cost a virtual call
    // in the hot loop for a value that cannot change after construction.
    inner_flip_ = (inner_->signConvention() == SignConvention::TrackingErrorYMinusR);
}

double CascadeController::compute(double outer_error)
{
    if (!std::isfinite(outer_error))
        return u_prev_; // hold last command; neither loop advances

    // -- Outer loop (optionally decimated for a multi-rate cascade) -----------
    double sp_raw = sp_prev_;
    if ((tick_ % p_.outerDecimation) == 0)
        sp_raw = outer_->compute(outer_error);
    ++tick_;

    // -- Rate limit, then hard clamp -----------------------------------------
    // Rate first so that a clamped setpoint can still slew back at the allowed rate.
    const double dmax = p_.spRateMax * Ts_;
    double sp = sp_raw;
    if (dmax >= 0.0)
        sp = std::clamp(sp, sp_prev_ - dmax, sp_prev_ + dmax);
    sp = std::clamp(sp, p_.spMin, p_.spMax);

    sp_clamped_ = (sp != sp_raw);

    // Setpoint anti-windup: tell the outer loop what was actually delivered, so its
    // integrator settles on the achievable setpoint instead of winding up past it.
    if (p_.antiWindup && sp_clamped_)
        outer_->bumplessInit(sp, outer_error);

    sp_prev_ = sp;

    // -- Inner loop ----------------------------------------------------------
    const double e_inner = inner_flip_ ? (y_inner_ - sp) : (sp - y_inner_);
    const double u = inner_->compute(e_inner);

    if (std::isfinite(u))
        u_prev_ = u;

    notify_buf_(0) = sp;
    notifyObserverState("inner_setpoint", notify_buf_);
    notifyObserver(u_prev_, outer_error);
    return u_prev_;
}

void CascadeController::bumplessInit(double u_target, double /*error*/)
{
    // The cascade's own argument is the OUTER error; the inner loop needs the inner
    // error, which is reconstructed from the last applied setpoint and measurement.
    const double e_inner = inner_flip_ ? (y_inner_ - sp_prev_) : (sp_prev_ - y_inner_);
    inner_->bumplessInit(u_target, e_inner);
    u_prev_ = u_target;
}

void CascadeController::reset()
{
    outer_->reset();
    inner_->reset();
    y_inner_ = 0.0;
    sp_prev_ = 0.0;
    u_prev_ = 0.0;
    sp_clamped_ = false;
    tick_ = 0;
    notifyObserverReset();
}

} // namespace ctrl
