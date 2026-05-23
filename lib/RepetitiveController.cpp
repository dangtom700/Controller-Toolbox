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
        const int N = p_.periodSteps;

        // Read the stored correction from one period ago (read before write)
        const double v_prev = v_buf_[buf_idx_];

        // Learning update: v[k] = Q.v[k-N] + Krc.e[k]
        v_now_ = p_.Q * v_prev + p_.Krc * error;

        // Write updated correction back into the same slot (replaces k-N with k)
        v_buf_[buf_idx_] = v_now_;
        buf_idx_ = (buf_idx_ + 1) % N;

        // Base controller output
        const double u_base = inner_->compute(error);

        // Combined output, clamped
        return std::max(p_.uMin, std::min(p_.uMax, u_base + v_now_));
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
