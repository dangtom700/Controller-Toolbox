#include "GPMPC.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

namespace
{
NonlinearMPC::DiscreteDynamics validateAndForward(const GPMPCParams &params,
                                                   NonlinearMPC::DiscreteDynamics f_d,
                                                   const std::shared_ptr<GPResidualModel> &gp)
{
    if (!gp)
        throw std::invalid_argument("GPMPC: gp must not be null.");
    const int expected = params.nmpc.n_states + params.nmpc.n_inputs;
    if (gp->xDim() != expected)
        throw std::invalid_argument(
            "GPMPC: gp->xDim() (" + std::to_string(gp->xDim()) +
            ") must equal n_states + n_inputs (" + std::to_string(expected) + ").");
    return f_d;
}
} // namespace

GPMPC::GPMPC(const GPMPCParams &params, NonlinearMPC::DiscreteDynamics f_d,
             std::shared_ptr<GPResidualModel> gp)
    : NonlinearMPC(params.nmpc, validateAndForward(params, f_d, gp)),
      gp_params_(params),
      gp_(std::move(gp))
{
    shrink_ = Eigen::VectorXd::Zero(params.nmpc.n_inputs * params.nmpc.Nu);
}

void GPMPC::tightenStepBounds()
{
    const int m  = static_cast<int>(U_warm_.rows());
    const int Nu = static_cast<int>(U_warm_.cols());

    for (int k = 0; k < Nu; ++k)
    {
        // Feature convention mirrors HybridMPC::addStateObservation/refitDataModel
        // exactly (lib/HybridMPC.cpp:26-27,71-72): feat << x, u.
        Eigen::VectorXd feat(x_traj_[k].size() + m);
        feat << x_traj_[k], U_warm_.col(k);

        const auto pred = gp_->predictWithUncertainty(feat, 0.0); // model_pred unused for variance
        const double shrink_k = gp_params_.uncertainty_inflation
                               * std::sqrt(std::max(pred.variance, 0.0));

        for (int j = 0; j < m; ++j)
        {
            const int idx = k * m + j;
            const double mid = 0.5 * (lb_qp_(idx) + ub_qp_(idx));
            lb_qp_(idx) = std::min(lb_qp_(idx) + shrink_k, mid);
            ub_qp_(idx) = std::max(ub_qp_(idx) - shrink_k, mid);
            shrink_(idx) = shrink_k;
        }
    }
}

} // namespace ctrl
