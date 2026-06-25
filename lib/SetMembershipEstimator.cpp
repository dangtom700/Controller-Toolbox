#include "SetMembershipEstimator.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ctrl
{

SetMembershipEstimator::SetMembershipEstimator(const StateSpace &plant,
                                                 const SetMembershipParams &params,
                                                 const Eigen::VectorXd &x0_center,
                                                 const Eigen::MatrixXd &E0_shape)
    : A_(plant.A), B_(plant.B), C_(plant.C), Ts_(plant.Ts), params_(params),
      x0_(x0_center), P0_(E0_shape), c_(x0_center), P_(E0_shape)
{
    const int n = static_cast<int>(A_.rows());
    if (x0_center.size() != n)
        throw std::invalid_argument("SetMembershipEstimator: x0_center size must match plant state size");
    if (E0_shape.rows() != n || E0_shape.cols() != n)
        throw std::invalid_argument("SetMembershipEstimator: E0_shape must be n x n");
    if (params_.w_bound <= 0.0 || params_.v_bound <= 0.0)
        throw std::invalid_argument("SetMembershipEstimator: w_bound and v_bound must be > 0");
}

void SetMembershipEstimator::predict(const Eigen::VectorXd &u)
{
    const int n = static_cast<int>(A_.rows());
    const Eigen::MatrixXd APA = A_ * P_ * A_.transpose();
    const Eigen::MatrixXd Qw = params_.w_bound * params_.w_bound * Eigen::MatrixXd::Identity(n, n);

    const double a = APA.trace();
    const double b = Qw.trace();
    const double pOpt = (b > 1e-300) ? std::sqrt(a / b) : 1.0;

    c_ = A_ * c_ + B_ * u;
    P_ = (1.0 + 1.0 / pOpt) * APA + (1.0 + pOpt) * Qw;
    P_ = 0.5 * (P_ + P_.transpose()); // suppress numerical asymmetry drift
}

void SetMembershipEstimator::update(const Eigen::VectorXd &y)
{
    const int n = static_cast<int>(A_.rows());
    const int p = static_cast<int>(C_.rows());
    const Eigen::MatrixXd Rv = params_.v_bound * params_.v_bound * Eigen::MatrixXd::Identity(p, p);

    const Eigen::MatrixXd Pinv = P_.inverse();
    const Eigen::MatrixXd RvInv = Rv.inverse();
    const Eigen::MatrixXd CtRvInvC = C_.transpose() * RvInv * C_;
    const Eigen::VectorXd PinvC = Pinv * c_;
    const Eigen::VectorXd CtRvInvY = C_.transpose() * RvInv * y;
    const double cPc = c_.dot(PinvC);
    const double yRy = y.dot(RvInv * y);

    double bestTrace = std::numeric_limits<double>::infinity();
    Eigen::VectorXd bestCenter;
    Eigen::MatrixXd bestShape;
    bool found = false;

    const int gridPoints = 99;
    for (int g = 1; g <= gridPoints; ++g)
    {
        const double lambda = static_cast<double>(g) / (gridPoints + 1); // (0,1) exclusive
        const Eigen::MatrixXd M = (1.0 - lambda) * Pinv + lambda * CtRvInvC;
        const Eigen::VectorXd bvec = (1.0 - lambda) * PinvC + lambda * CtRvInvY;
        const double k = (1.0 - lambda) * (cPc - 1.0) + lambda * (yRy - 1.0);

        Eigen::FullPivLU<Eigen::MatrixXd> luM(M);
        if (!luM.isInvertible()) continue;
        const Eigen::VectorXd MinvB = luM.solve(bvec);
        const double scale = bvec.dot(MinvB) - k;
        if (scale <= 0.0) continue; // not a valid (nonempty-interior) ellipsoid at this lambda

        const Eigen::MatrixXd MinvScaled = luM.solve(Eigen::MatrixXd::Identity(n, n)) * scale;
        const double tr = MinvScaled.trace();
        if (tr < bestTrace)
        {
            bestTrace = tr;
            bestCenter = MinvB;
            bestShape = MinvScaled;
            found = true;
        }
    }

    if (!found)
    {
        consistent_ = false;
        return; // keep the pre-update (predicted) ellipsoid - do not corrupt state
    }

    consistent_ = true;
    c_ = bestCenter;
    P_ = 0.5 * (bestShape + bestShape.transpose());
}

void SetMembershipEstimator::reset()
{
    c_ = x0_;
    P_ = P0_;
    consistent_ = true;
}

} // namespace ctrl
