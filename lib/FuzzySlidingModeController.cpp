#include "FuzzySlidingModeController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

void FuzzySlidingModeController::validate(const FuzzySMCParams &p, double Ts)
{
    if (Ts <= 0.0)
        throw std::invalid_argument("FuzzySlidingModeController: Ts must be > 0");
    if (p.fuzzy.u_scale <= 0.0)
        throw std::invalid_argument("FuzzySlidingModeController: fuzzy.u_scale must be > 0");
    if (p.phiMin <= 0.0)
        throw std::invalid_argument("FuzzySlidingModeController: phiMin must be > 0");
    if (p.Kmin >= p.Kmax)
        throw std::invalid_argument("FuzzySlidingModeController: Kmin must be < Kmax");
    if (p.phiMin >= p.phiMax)
        throw std::invalid_argument("FuzzySlidingModeController: phiMin must be < phiMax");
    // Spans <= -1 would let the multiplier reach zero or go negative, flipping the
    // sign of the reaching law rather than merely softening it.
    if (p.gainSpan <= -1.0 || p.phiSpan <= -1.0)
        throw std::invalid_argument("FuzzySlidingModeController: gainSpan and phiSpan must be > -1");
}

FuzzySlidingModeController::FuzzySlidingModeController(const FuzzySMCParams &params, double sampleTime)
    : p_(params), Ts_(sampleTime), smc_(params.smc, sampleTime), fuzzy_(params.fuzzy, sampleTime)
{
    validate(p_, Ts_);
    K_ = p_.smc.K;
    phi_ = p_.smc.phi;
}

double FuzzySlidingModeController::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev_; // hold last output; surface, fuzzy state and SMC all stall

    // The surface must be known BEFORE the reaching law runs, so it is formed here
    // rather than read back from smc_.slidingSurface() (which holds s[k-1]). The
    // formula and the error history match DiscreteSMC's exactly, so the two stay in
    // step - smc_.slidingSurface() equals s_ after this call.
    s_ = p_.smc.c_e * error + p_.smc.c_de * (error - e_prev_);
    e_prev_ = error;

    // Mamdani inference on (s, s_dot); magnitude only - the reaching law owns the sign.
    const double fz = fuzzy_.compute(s_);
    m_ = std::isfinite(fz) ? std::min(1.0, std::abs(fz) / p_.fuzzy.u_scale) : 0.0;

    K_ = std::clamp(p_.smc.K * (1.0 + p_.gainSpan * m_), p_.Kmin, p_.Kmax);
    phi_ = std::clamp(p_.smc.phi * (1.0 + p_.phiSpan * m_), p_.phiMin, p_.phiMax);

    SMCParams mod = p_.smc;
    mod.K = K_;
    mod.phi = phi_;
    smc_.setParams(mod);

    const double u = smc_.compute(error);
    if (std::isfinite(u))
        u_prev_ = u;

    notify_buf_(0) = s_;
    notify_buf_(1) = K_;
    notify_buf_(2) = phi_;
    notifyObserverState("fsmc", notify_buf_);
    notifyObserver(u_prev_, error);
    return u_prev_;
}

void FuzzySlidingModeController::setParams(const FuzzySMCParams &p)
{
    validate(p, Ts_);
    p_ = p;
    fuzzy_.setParams(p_.fuzzy);
    smc_.setParams(p_.smc);
    K_ = p_.smc.K;
    phi_ = p_.smc.phi;
}

void FuzzySlidingModeController::reset()
{
    smc_.reset();
    fuzzy_.reset();
    e_prev_ = 0.0;
    s_ = 0.0;
    m_ = 0.0;
    K_ = p_.smc.K;
    phi_ = p_.smc.phi;
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
