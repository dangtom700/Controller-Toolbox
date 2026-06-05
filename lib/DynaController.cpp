#include "DynaController.h"
#include <stdexcept>
#include <cmath>

namespace ctrl {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static SINDy::Params fixSindyParams(SINDy::Params sp)
{
    // DynaController always uses a 1D error state and 1D SISO control input.
    sp.n_state = 1;
    sp.n_input = 1;
    return sp;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

DynaController::DynaController(const Params& p, std::shared_ptr<IController> inner)
    : p_(p)
    , inner_(std::move(inner))
    , sindy_(fixSindyParams(p_.sindy))
{
    if (!inner_)
        throw std::invalid_argument("DynaController: inner controller must not be null");
}

// ---------------------------------------------------------------------------
// IController
// ---------------------------------------------------------------------------

double DynaController::compute(double error)
{
    if (!std::isfinite(error)) return u_prev_;
    double u = inner_->compute(error);

    if (has_prev_) {
        addTransition(e_prev_, u_prev_, error);

        const bool should_fit =
            (!model_fitted_ && n_total_ >= p_.n_collect) ||
            (model_fitted_  && steps_since_fit_ >= p_.n_refit_every);

        if (should_fit)
            fitModel();
    }

    e_prev_   = error;
    u_prev_   = u;
    has_prev_ = true;

    notifyObserver(u, error);
    return u;
}

void DynaController::reset()
{
    inner_->reset();
    e_prev_   = 0.0;
    u_prev_   = 0.0;
    has_prev_ = false;
    notifyObserverReset();
}

// ---------------------------------------------------------------------------
// Data collection
// ---------------------------------------------------------------------------

void DynaController::addTransition(double e, double u, double e_next)
{
    Eigen::VectorXd e_v(1);  e_v(0)  = e;
    Eigen::VectorXd en_v(1); en_v(0) = e_next;
    Eigen::VectorXd u_v(1);  u_v(0)  = u;

    // addSnapshotFD(x_k, x_k1, u_k, Ts) - computes xdot = (x_k1 - x_k) / Ts
    sindy_.addSnapshotFD(e_v, en_v, u_v, p_.Ts);

    ++n_total_;
    ++steps_since_fit_;
}

bool DynaController::fitModel()
{
    if (n_total_ < p_.n_collect)
        return false;

    model_        = sindy_.fit();
    model_fitted_ = true;
    steps_since_fit_ = 0;
    return true;
}

// ---------------------------------------------------------------------------
// Model-based rollout (used by Python for policy improvement)
// ---------------------------------------------------------------------------

Eigen::VectorXd DynaController::modelRollout(double                  e0,
                                               const Eigen::VectorXd&  u_sequence) const
{
    if (!model_fitted_)
        throw std::logic_error("DynaController::modelRollout: model not fitted - "
                                "call compute() for at least n_collect steps first");

    const int n_steps = static_cast<int>(u_sequence.size());
    Eigen::VectorXd e_traj(n_steps);

    Eigen::VectorXd e(1);  e(0) = e0;
    Eigen::VectorXd u(1);

    for (int k = 0; k < n_steps; ++k) {
        u(0) = u_sequence(k);
        Eigen::VectorXd edot = model_.predict(e, u);
        e = e + p_.Ts * edot;          // Euler forward step
        e_traj(k) = e(0);
    }
    return e_traj;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

const SINDyModel& DynaController::model() const
{
    if (!model_fitted_)
        throw std::logic_error("DynaController::model: not fitted yet");
    return model_;
}

} // namespace ctrl
