#include "HybridModelTrainer.h"
#include <cmath>
#include <stdexcept>

namespace ctrl {

HybridModelTrainer::HybridModelTrainer(const Params& params)
    : p_(params)
{}

// =============================================================================
// Main training entry point
// =============================================================================

HybridModelTrainer::Result
HybridModelTrainer::trainHybridModel(HybridModel&           model,
                                      const Eigen::MatrixXd& X_obs,
                                      const Eigen::MatrixXd& U_obs,
                                      const Eigen::MatrixXd& X_next_obs) const
{
    const int N       = static_cast<int>(X_obs.cols());
    const int n_state = static_cast<int>(X_obs.rows());
    const int n_in    = static_cast<int>(U_obs.rows());
    const int n_feat  = n_state + n_in;

    if (N < 1 || U_obs.cols() != N || X_next_obs.cols() != N)
        throw std::invalid_argument("HybridModelTrainer: inconsistent column counts");

    // Build feature matrix and residual matrix
    Eigen::MatrixXd Feat(n_feat, N);
    Eigen::MatrixXd Resid(n_state, N);
    for (int k = 0; k < N; ++k) {
        Feat.col(k) << X_obs.col(k), U_obs.col(k);
        Resid.col(k) = X_next_obs.col(k) - model.predictPhys(X_obs.col(k), U_obs.col(k));
    }

    HybridModel::DataFunc f_data;
    std::string method_name;

    switch (p_.method) {
    case Method::Ridge:
        f_data      = trainRidge(Feat, Resid);
        method_name = "Ridge";
        break;
    case Method::GP:
        f_data      = trainGP(Feat, Resid);
        method_name = "GP";
        break;
    case Method::ESN:
        f_data      = trainESN(Feat, Resid);
        method_name = "ESN";
        break;
    }

    model.setDataModel(std::move(f_data));

    // Compute training RMSE
    double mse = 0.0;
    for (int k = 0; k < N; ++k) {
        const Eigen::VectorXd pred = model.predict(X_obs.col(k), U_obs.col(k));
        mse += (pred - X_next_obs.col(k)).squaredNorm();
    }
    const double rmse = std::sqrt(mse / static_cast<double>(N));

    return Result{true, rmse, method_name, N};
}

// =============================================================================
// Validation
// =============================================================================

double HybridModelTrainer::validate(const HybridModel&     model,
                                     const Eigen::MatrixXd& X_obs,
                                     const Eigen::MatrixXd& U_obs,
                                     const Eigen::MatrixXd& X_next_obs) const
{
    const int N = static_cast<int>(X_obs.cols());
    if (N < 1) return 0.0;

    double mse = 0.0;
    for (int k = 0; k < N; ++k) {
        const Eigen::VectorXd pred = model.predict(X_obs.col(k), U_obs.col(k));
        mse += (pred - X_next_obs.col(k)).squaredNorm();
    }
    return std::sqrt(mse / static_cast<double>(N));
}

// =============================================================================
// Private: Ridge regression
// =============================================================================

HybridModel::DataFunc
HybridModelTrainer::trainRidge(const Eigen::MatrixXd& Feat,
                                const Eigen::MatrixXd& Resid) const
{
    // Feat:  n_feat x N  (each column = one feature vector [x;u])
    // Resid: n_out  x N
    // We want W (n_feat x n_out) such that W' * feat approx = resid
    // via min ||Feat' W - Resid'||_F  + lambda * ||W||_F
    // Normal equations: (Feat Feat' + lambda*I) W = Feat Resid'
    const int n_feat = static_cast<int>(Feat.rows());
    Eigen::MatrixXd FtF = Feat * Feat.transpose();
    FtF.diagonal().array() += p_.ridge_lambda;
    const Eigen::MatrixXd W = FtF.ldlt().solve(Feat * Resid.transpose());  // n_feat x n_out

    return [W](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd feat(x.size() + u.size());
        feat << x, u;
        return W.transpose() * feat;
    };
}

// =============================================================================
// Private: Gaussian Process (one GP per output dimension)
// =============================================================================

HybridModel::DataFunc
HybridModelTrainer::trainGP(const Eigen::MatrixXd& Feat,
                              const Eigen::MatrixXd& Resid) const
{
    const int N      = static_cast<int>(Feat.cols());
    const int n_out  = static_cast<int>(Resid.rows());
    const int x_dim  = static_cast<int>(Feat.rows());

    // Train one GP per state dimension (independent GPs)
    auto gps = std::make_shared<std::vector<GaussianProcess>>();
    gps->reserve(n_out);

    for (int d = 0; d < n_out; ++d) {
        GaussianProcess gp(x_dim, p_.gp);
        for (int k = 0; k < N; ++k)
            gp.addPoint(Feat.col(k), Resid(d, k));
        gp.fit();
        gps->push_back(std::move(gp));
    }

    return [gps](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd feat(x.size() + u.size());
        feat << x, u;
        const int n = static_cast<int>(gps->size());
        Eigen::VectorXd delta(n);
        for (int d = 0; d < n; ++d)
            delta(d) = (*gps)[d].predict(feat).mean;
        return delta;
    };
}

// =============================================================================
// Private: Echo State Network
// =============================================================================

HybridModel::DataFunc
HybridModelTrainer::trainESN(const Eigen::MatrixXd& Feat,
                               const Eigen::MatrixXd& Resid) const
{
    const int N      = static_cast<int>(Feat.cols());
    const int n_in   = static_cast<int>(Feat.rows());
    const int n_out  = static_cast<int>(Resid.rows());

    EchoStateNetwork::Params ep = p_.esn;
    ep.n_in  = n_in;
    ep.n_out = n_out;

    auto esn = std::make_shared<EchoStateNetwork>(ep);
    for (int k = 0; k < N; ++k) {
        esn->stepReservoir(Feat.col(k));
        esn->addTrainingTarget(Resid.col(k));
    }
    esn->fitReadout();

    // Zero-state evaluation: reset reservoir before every prediction so
    // f_data is a stateless mapping (required for MPC multi-step rollout).
    return [esn, n_in](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd feat(n_in);
        feat << x, u;
        esn->reset();
        return esn->predict(feat);
    };
}

} // namespace ctrl
