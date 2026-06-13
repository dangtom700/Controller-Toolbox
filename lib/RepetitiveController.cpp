#include "RepetitiveController.h"
#include <algorithm>
#include <stdexcept>

namespace ctrl
{

    RepetitiveController::RepetitiveController(std::shared_ptr<IController> inner,
                                               const RepetitiveParams &params,
                                               double Ts)
        : inner_(std::move(inner)), p_(params), Ts_(Ts), buf_idx_(0), v_now_(0.0)
    {
        if (p_.periodSteps < 1)
            throw std::invalid_argument("[RepetitiveController] periodSteps must be >= 1");
        v_buf_.assign(p_.periodSteps, 0.0);
    }

    double RepetitiveController::compute(double error)
    {
        if (!std::isfinite(error)) return u_prev_;
        const int N = p_.periodSteps;

        // v_buf_ is a circular buffer of length N holding one full period of corrections.
        // buf_idx_ always points to the slot that holds v[k-N] (the value from exactly
        // one period ago). Read before write so v_prev = v[k-N] correctly.
        const double v_prev = v_buf_[buf_idx_];

        // Learning law: v[k] = Q*v[k-N] + Krc*e[k]
        // Q < 1: exponential forgetting - previous period's correction decays;
        //        adds robustness when the disturbance period is slightly uncertain.
        // Q = 1: perfect memory - converges to exact cancellation of any periodic disturbance
        //        with period N*Ts, but is sensitive to model error.
        v_now_ = p_.Q * v_prev + p_.Krc * error;

        // Overwrite the same slot: v_buf_[buf_idx_] becomes v[k] for use N steps later.
        v_buf_[buf_idx_] = v_now_;
        buf_idx_ = (buf_idx_ + 1) % N;

        // Base controller output (stabilises the loop independently of the repetitive term)
        const double u_base = inner_->compute(error);

        u_prev_ = std::max(p_.uMin, std::min(p_.uMax, u_base + v_now_));
        return u_prev_;
    }

    void RepetitiveController::reset()
    {
        inner_->reset();
        std::fill(v_buf_.begin(), v_buf_.end(), 0.0);
        buf_idx_ = 0;
        v_now_ = 0.0;
    }

    void RepetitiveController::setParams(const RepetitiveParams &p)
    {
        if (p.periodSteps != p_.periodSteps)
        {
            v_buf_.assign(p.periodSteps, 0.0);
            buf_idx_ = 0;
        }
        p_ = p;
    }

} // namespace ctrl
