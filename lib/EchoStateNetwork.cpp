#include "EchoStateNetwork.h"
#include <cmath>
#include <stdexcept>

namespace ctrl {

// ---------------------------------------------------------------------------
// Weight initialisation
// ---------------------------------------------------------------------------

void EchoStateNetwork::initWeights()
{
    std::mt19937 rng(p_.seed);
    std::uniform_real_distribution<double> u_dist(-1.0, 1.0);
    std::bernoulli_distribution sparse_mask(1.0 - p_.sparsity);

    // Sparse random reservoir W_res
    W_res_ = Eigen::MatrixXd::Zero(p_.n_res, p_.n_res);
    for (int i = 0; i < p_.n_res; ++i)
        for (int j = 0; j < p_.n_res; ++j)
            if (sparse_mask(rng))
                W_res_(i, j) = u_dist(rng);

    // Scale to target spectral radius
    Eigen::EigenSolver<Eigen::MatrixXd> es(W_res_, false);
    double rho = 0.0;
    for (int i = 0; i < p_.n_res; ++i)
        rho = std::max(rho, std::abs(es.eigenvalues()(i)));
    if (rho > 1e-12)
        W_res_ *= p_.spectral_radius / rho;

    // Dense random input weights
    W_in_ = Eigen::MatrixXd::Zero(p_.n_res, p_.n_in);
    for (int i = 0; i < p_.n_res; ++i)
        for (int j = 0; j < p_.n_in; ++j)
            W_in_(i, j) = p_.input_scaling * u_dist(rng);

    // Readout weights initialised to zero; set by fitReadout()
    W_out_ = Eigen::MatrixXd::Zero(p_.n_out, p_.n_res + p_.n_in);
}

EchoStateNetwork::EchoStateNetwork(const Params& p) : p_(p)
{
    if (p_.n_res <= 0 || p_.n_in <= 0 || p_.n_out <= 0)
        throw std::invalid_argument("EchoStateNetwork: n_res, n_in, n_out must be positive");
    if (p_.spectral_radius >= 1.0)
        throw std::invalid_argument("EchoStateNetwork: spectral_radius must be < 1");

    r_.setZero(p_.n_res);
    initWeights();
}

// ---------------------------------------------------------------------------
// Reservoir dynamics
// ---------------------------------------------------------------------------

Eigen::VectorXd EchoStateNetwork::extendedState(const Eigen::VectorXd& u) const
{
    Eigen::VectorXd z(p_.n_res + p_.n_in);
    z.head(p_.n_res) = r_;
    z.tail(p_.n_in)  = u;
    return z;
}

void EchoStateNetwork::stepReservoir(const Eigen::VectorXd& u)
{
    // Leaky integration: r[k+1] = (1-alpha)*r[k] + alpha*tanh(W_res*r[k] + W_in*u[k])
    const Eigen::VectorXd r_new = (W_res_ * r_ + W_in_ * u).array().tanh();
    r_ = (1.0 - p_.alpha) * r_ + p_.alpha * r_new;
}

void EchoStateNetwork::addTrainingTarget(const Eigen::VectorXd& y_target)
{
    if (washout_cnt_++ < p_.washout) return;   // discard reservoir warm-up

    // We need u but don't have it here; store from last step
    // This is called right after stepReservoir(u), so r_ is updated.
    // We record the reservoir state but need u for the extended state.
    // For simplicity, store r_ directly (readout = W_out * [r; u] will use current r)
    states_ .push_back(r_);
    targets_.push_back(y_target);
}

// ---------------------------------------------------------------------------
// Ridge regression readout
// ---------------------------------------------------------------------------

void EchoStateNetwork::fitReadout()
{
    if (states_.empty())
        throw std::runtime_error("EchoStateNetwork::fitReadout(): no training data");

    const int M = static_cast<int>(states_.size());
    const int D = p_.n_res;   // state dimension (simplified: readout from r_ only)

    Eigen::MatrixXd S(M, D);
    Eigen::MatrixXd T(M, p_.n_out);
    for (int k = 0; k < M; ++k) {
        S.row(k) = states_ [k].transpose();
        T.row(k) = targets_[k].transpose();
    }

    // Ridge: W_out' = (S'S + ridge*I)^{-1} S' T
    Eigen::MatrixXd A = S.transpose() * S
                       + p_.ridge * Eigen::MatrixXd::Identity(D, D);
    Eigen::MatrixXd rhs = S.transpose() * T;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    Eigen::MatrixXd W_out_T = ldlt.solve(rhs);  // D * n_out

    // Store as n_out * D (resize W_out_ without the input part for simplified readout)
    W_out_.resize(p_.n_out, D);
    W_out_ = W_out_T.transpose();

    fitted_ = true;
}

// ---------------------------------------------------------------------------
// Prediction
// ---------------------------------------------------------------------------

Eigen::VectorXd EchoStateNetwork::predict() const
{
    if (!fitted_)
        throw std::runtime_error("EchoStateNetwork::predict(): call fitReadout() first");
    return W_out_ * r_;
}

Eigen::VectorXd EchoStateNetwork::predict(const Eigen::VectorXd& u)
{
    stepReservoir(u);
    return predict();
}

StateFunc EchoStateNetwork::reservoirStateFunc() const
{
    EchoStateNetwork self = *this;
    return [self](const Eigen::VectorXd& r, const Eigen::VectorXd& u) mutable -> Eigen::VectorXd {
        self.r_ = r;
        const Eigen::VectorXd r_new = (self.W_res_ * r + self.W_in_ * u).array().tanh();
        return (1.0 - self.p_.alpha) * r + self.p_.alpha * r_new;
    };
}

void EchoStateNetwork::reset()
{
    r_.setZero();
    states_.clear();
    targets_.clear();
    washout_cnt_ = 0;
    // NOTE: W_out_ and fitted_ are intentionally NOT cleared - reset() only
    // resets the reservoir state for inference; readout weights persist.
}

} // namespace ctrl
