#include "GPResidualModel.h"
#include <stdexcept>

namespace ctrl {

GPResidualModel::GPResidualModel(int x_dim, const Params& par)
    : gp_(x_dim, par.gp), par_(par)
{
}

void GPResidualModel::addResidualPoint(const Eigen::VectorXd& x_feat,
                                        double y_true,
                                        double y_model)
{
    gp_.addPoint(x_feat, y_true - y_model);
}

void GPResidualModel::residualFit(const Eigen::MatrixXd& X_feat,
                                   const Eigen::VectorXd& Y_true,
                                   ModelFn model_fn)
{
    const int N = static_cast<int>(X_feat.cols());
    if (N != static_cast<int>(Y_true.size()))
        throw std::invalid_argument(
            "GPResidualModel::residualFit: X_feat.cols() must equal Y_true.size()");

    gp_.reset();
    for (int k = 0; k < N; ++k)
        gp_.addPoint(X_feat.col(k), Y_true(k) - model_fn(X_feat.col(k)));
    gp_.fit();
}

void GPResidualModel::fit()
{
    gp_.fit();
}

GPResidualModel::Prediction
GPResidualModel::predictWithUncertainty(const Eigen::VectorXd& x_feat,
                                         double model_pred) const
{
    if (!gp_.isFitted())
        return {model_pred, 0.0, 0.0};

    const auto [mu, var] = gp_.predict(x_feat);
    return {model_pred + mu, mu, var};
}

void GPResidualModel::reset()
{
    gp_.reset();
}

} // namespace ctrl
