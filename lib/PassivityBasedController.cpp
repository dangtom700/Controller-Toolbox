#include "PassivityBasedController.h"
#include <algorithm>
#include <stdexcept>

namespace ctrl {

PassivityBasedController::PassivityBasedController(MassMatrixFn M, PotentialGradFn dV,
                                                     CoriolisFn C, const PBCParams &params,
                                                     double Ts)
    : M_(std::move(M)), dV_(std::move(dV)), C_(std::move(C)), params_(params), Ts_(Ts)
{
}

Eigen::VectorXd PassivityBasedController::computeVec(const Eigen::VectorXd &state)
{
    const int total = static_cast<int>(state.size());
    if (total % 2 != 0)
        throw std::invalid_argument(
            "PassivityBasedController::computeVec: state must be [q; qdot], an even-length vector.");
    const int n = total / 2;

    if (n_ < 0)
    {
        n_ = n;
        if (q_d_.size() == 0) q_d_ = Eigen::VectorXd::Zero(n_);
        u_prev_ = Eigen::VectorXd::Zero(n_);
    }
    else if (n != n_)
    {
        throw std::invalid_argument(
            "PassivityBasedController::computeVec: state size changed between calls.");
    }

    if (!state.allFinite())
        return u_prev_;

    const Eigen::VectorXd q    = state.head(n_);
    const Eigen::VectorXd qdot = state.tail(n_);

    const Eigen::MatrixXd Mq = M_(q);
    const Eigen::MatrixXd Cq = C_(q, qdot);
    const Eigen::VectorXd dVq = dV_(q);

    if (!Mq.allFinite() || !Cq.allFinite() || !dVq.allFinite())
        return u_prev_; // caller-supplied physics callback hit a singular/boundary configuration

    const Eigen::VectorXd qErr = q - q_d_;
    Eigen::VectorXd u = dVq - params_.Kp * qErr - params_.Kd * qdot;
    u = u.cwiseMax(params_.uMin).cwiseMin(params_.uMax);

    storageEnergy_ = 0.5 * qdot.dot(Mq * qdot) + 0.5 * qErr.dot(params_.Kp * qErr);

    u_prev_ = u;
    notifyObserverVec(u, state);
    return u;
}

double PassivityBasedController::compute(double /*signal*/)
{
    throw std::logic_error(
        "PassivityBasedController is MIMO-only ([q;qdot] together) - call computeVec().");
}

void PassivityBasedController::reset()
{
    if (n_ >= 0)
        u_prev_ = Eigen::VectorXd::Zero(n_);
    storageEnergy_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
