#include "HybridMPC.h"
#include <cmath>
#include <stdexcept>

namespace ctrl {

HybridMPC::HybridMPC(const HybridMPCParams& params, std::shared_ptr<HybridModel> model)
    : NonlinearMPC(params.nmpc,
                   // Lambda captures model by value (shared_ptr copy) so the HybridModel
                   // stays alive as long as the NonlinearMPC base does. Any setDataModel()
                   // call on *model is immediately visible here.
                   [m = model](const Eigen::VectorXd& x, const Eigen::VectorXd& u)
                       -> Eigen::VectorXd { return m->predict(x, u); })
    , hp_(params)
    , model_(std::move(model))
    , max_buffer_(std::max(params.min_observations * 2, 1))
{}

void HybridMPC::addStateObservation(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u,
                                     const Eigen::VectorXd& x_next_true)
{
    const Eigen::VectorXd x_next_phys = model_->predictPhys(x, u);
    const Eigen::VectorXd residual    = x_next_true - x_next_phys;

    Eigen::VectorXd feat(x.size() + u.size());
    feat << x, u;

    feat_data_.push_back(feat);
    resid_data_.push_back(residual);
    ++obs_since_refit_;

    // FIFO eviction: keep at most max_buffer_ observations to bound memory
    if (static_cast<int>(feat_data_.size()) > max_buffer_) {
        feat_data_.erase(feat_data_.begin());
        resid_data_.erase(resid_data_.begin());
    }

    const int interval = hp_.data_update_interval;
    if (interval > 0
        && obs_since_refit_ >= interval
        && static_cast<int>(feat_data_.size()) >= hp_.min_observations)
    {
        refitDataModel();
    }
}

bool HybridMPC::refitDataModel()
{
    const int N = static_cast<int>(feat_data_.size());
    if (N < hp_.min_observations) return false;

    const int n_feat = static_cast<int>(feat_data_[0].size());
    const int n_out  = static_cast<int>(resid_data_[0].size());

    // Build design matrix X (N x n_feat) and target Y (N x n_out)
    Eigen::MatrixXd X(N, n_feat), Y(N, n_out);
    for (int k = 0; k < N; ++k) {
        X.row(k) = feat_data_[k].transpose();
        Y.row(k) = resid_data_[k].transpose();
    }

    // Ridge regression: W = (X'X + lambda*I)^{-1} X'Y  (n_feat x n_out)
    Eigen::MatrixXd XtX = X.transpose() * X;
    XtX.diagonal().array() += hp_.ridge_lambda;
    const Eigen::MatrixXd W = XtX.ldlt().solve(X.transpose() * Y);

    // Install fitted linear model as the data component
    model_->setDataModel(
        [W](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
            Eigen::VectorXd feat(x.size() + u.size());
            feat << x, u;
            return W.transpose() * feat;  // n_out x 1
        });

    data_fitted_     = true;
    obs_since_refit_ = 0;
    return true;
}

} // namespace ctrl
