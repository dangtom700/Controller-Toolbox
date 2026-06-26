#include "ValueIterationSolver.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace ctrl
{

ValueIterationSolver::ValueIterationSolver(DynamicsFn f, StageCost cost, const DPGridParams &params)
    : f_(std::move(f)), cost_(std::move(cost)), params_(params)
{
    n_states_ = static_cast<int>(params_.x_min.size());
    n_inputs_ = static_cast<int>(params_.u_min.size());

    if (n_states_ == 0 || params_.x_max.size() != n_states_ || params_.n_grid_per_dim.size() != n_states_)
        throw std::invalid_argument(
            "ValueIterationSolver: x_min/x_max/n_grid_per_dim must be non-empty and equal size.");
    if (n_inputs_ == 0 || params_.u_max.size() != n_inputs_)
        throw std::invalid_argument("ValueIterationSolver: u_min/u_max must be non-empty and equal size.");
    for (int d = 0; d < n_states_; ++d)
        if (params_.n_grid_per_dim(d) < 2)
            throw std::invalid_argument("ValueIterationSolver: every n_grid_per_dim entry must be >= 2.");
    if (params_.n_grid_u < 2)
        throw std::invalid_argument("ValueIterationSolver: n_grid_u must be >= 2.");
    if (!(params_.x_max.array() > params_.x_min.array()).all())
        throw std::invalid_argument("ValueIterationSolver: every x_max entry must exceed the matching x_min entry.");
    if (!(params_.u_max.array() > params_.u_min.array()).all())
        throw std::invalid_argument("ValueIterationSolver: every u_max entry must exceed the matching u_min entry.");

    n_state_grid_  = totalGridPoints(params_.n_grid_per_dim);
    n_action_grid_ = 1;
    for (int d = 0; d < n_inputs_; ++d) n_action_grid_ *= params_.n_grid_u;

    constexpr long long kGridWarnThreshold = 20'000'000LL;
    if (n_state_grid_ * n_action_grid_ > kGridWarnThreshold)
    {
        std::fprintf(stderr,
            "[ValueIterationSolver] large grid: %lld state points x %lld action points "
            "(warn threshold %lld) - solve() may be slow. Consider coarsening "
            "n_grid_per_dim/n_grid_u.\n",
            n_state_grid_, n_action_grid_, kGridWarnThreshold);
    }

    V_.setZero(n_state_grid_);
    policy_.assign(static_cast<size_t>(n_state_grid_), Eigen::VectorXd::Zero(n_inputs_));
}

long long ValueIterationSolver::totalGridPoints(const Eigen::VectorXi &counts)
{
    long long total = 1;
    for (int d = 0; d < counts.size(); ++d) total *= counts(d);
    return total;
}

Eigen::VectorXi ValueIterationSolver::linearToMultiIndex(long long linear, const Eigen::VectorXi &counts)
{
    Eigen::VectorXi idx(counts.size());
    for (int d = 0; d < counts.size(); ++d)
    {
        idx(d) = static_cast<int>(linear % counts(d));
        linear /= counts(d);
    }
    return idx;
}

long long ValueIterationSolver::multiIndexToLinear(const Eigen::VectorXi &multiIndex, const Eigen::VectorXi &counts)
{
    long long linear = 0;
    long long stride = 1;
    for (int d = 0; d < counts.size(); ++d)
    {
        linear += static_cast<long long>(multiIndex(d)) * stride;
        stride *= counts(d);
    }
    return linear;
}

Eigen::VectorXd ValueIterationSolver::stateAt(const Eigen::VectorXi &multiIndex) const
{
    Eigen::VectorXd x(n_states_);
    for (int d = 0; d < n_states_; ++d)
    {
        const double t = static_cast<double>(multiIndex(d)) / (params_.n_grid_per_dim(d) - 1);
        x(d) = params_.x_min(d) + t * (params_.x_max(d) - params_.x_min(d));
    }
    return x;
}

Eigen::VectorXd ValueIterationSolver::actionAt(const Eigen::VectorXi &multiIndex) const
{
    Eigen::VectorXd u(n_inputs_);
    for (int d = 0; d < n_inputs_; ++d)
    {
        const double t = static_cast<double>(multiIndex(d)) / (params_.n_grid_u - 1);
        u(d) = params_.u_min(d) + t * (params_.u_max(d) - params_.u_min(d));
    }
    return u;
}

Eigen::VectorXd ValueIterationSolver::clampToGrid(const Eigen::VectorXd &x) const
{
    return x.cwiseMax(params_.x_min).cwiseMin(params_.x_max);
}

std::vector<std::pair<long long, double>>
ValueIterationSolver::interpolationCorners(const Eigen::VectorXd &xClamped) const
{
    Eigen::VectorXi i0(n_states_);
    Eigen::VectorXd frac(n_states_);
    for (int d = 0; d < n_states_; ++d)
    {
        const int n        = params_.n_grid_per_dim(d);
        const double span  = params_.x_max(d) - params_.x_min(d);
        const double t     = (xClamped(d) - params_.x_min(d)) / span * (n - 1);
        const int i0d      = std::clamp(static_cast<int>(std::floor(t)), 0, n - 2);
        i0(d)   = i0d;
        frac(d) = t - i0d;
    }

    const long long nCorners = 1LL << n_states_;
    std::vector<std::pair<long long, double>> corners;
    corners.reserve(static_cast<size_t>(nCorners));
    for (long long c = 0; c < nCorners; ++c)
    {
        Eigen::VectorXi corner = i0;
        double weight = 1.0;
        for (int d = 0; d < n_states_; ++d)
        {
            if ((c >> d) & 1) { corner(d) += 1; weight *= frac(d); }
            else                weight *= (1.0 - frac(d));
        }
        corners.emplace_back(multiIndexToLinear(corner, params_.n_grid_per_dim), weight);
    }
    return corners;
}

double ValueIterationSolver::interpolateValue(const Eigen::VectorXd &xClamped) const
{
    double result = 0.0;
    for (const auto &[linear, weight] : interpolationCorners(xClamped))
        result += weight * V_(linear);
    return result;
}

Eigen::VectorXd ValueIterationSolver::interpolatePolicy(const Eigen::VectorXd &xClamped) const
{
    Eigen::VectorXd result = Eigen::VectorXd::Zero(n_inputs_);
    for (const auto &[linear, weight] : interpolationCorners(xClamped))
        result += weight * policy_[static_cast<size_t>(linear)];
    return result;
}

void ValueIterationSolver::solve()
{
    V_.setZero(n_state_grid_);
    policy_.assign(static_cast<size_t>(n_state_grid_), Eigen::VectorXd::Zero(n_inputs_));

    Eigen::VectorXd V_new(n_state_grid_);
    const Eigen::VectorXi actionCounts = Eigen::VectorXi::Constant(n_inputs_, params_.n_grid_u);

    converged_   = false;
    iterations_  = 0;
    final_delta_ = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < params_.max_iter; ++iter)
    {
        double maxDelta = 0.0;

        for (long long i = 0; i < n_state_grid_; ++i)
        {
            const Eigen::VectorXd x_i = stateAt(linearToMultiIndex(i, params_.n_grid_per_dim));

            double bestCost = std::numeric_limits<double>::infinity();
            Eigen::VectorXd bestAction = Eigen::VectorXd::Zero(n_inputs_);

            for (long long j = 0; j < n_action_grid_; ++j)
            {
                const Eigen::VectorXd u_j    = actionAt(linearToMultiIndex(j, actionCounts));
                const Eigen::VectorXd x_next = f_(x_i, u_j);

                const bool inBounds = (x_next.array() >= params_.x_min.array()).all() &&
                                       (x_next.array() <= params_.x_max.array()).all();
                const double v_next = inBounds ? interpolateValue(x_next) : params_.out_of_grid_penalty;
                const double candidate = cost_(x_i, u_j) + params_.discount * v_next;

                if (candidate < bestCost)
                {
                    bestCost   = candidate;
                    bestAction = u_j;
                }
            }

            V_new(i) = bestCost;
            policy_[static_cast<size_t>(i)] = bestAction;
            maxDelta = std::max(maxDelta, std::abs(bestCost - V_(i)));
        }

        V_.swap(V_new);
        iterations_  = iter + 1;
        final_delta_ = maxDelta;

        if (maxDelta < params_.tol)
        {
            converged_ = true;
            break;
        }
    }
}

Eigen::VectorXd ValueIterationSolver::policy(const Eigen::VectorXd &x) const
{
    return interpolatePolicy(clampToGrid(x));
}

double ValueIterationSolver::value(const Eigen::VectorXd &x) const
{
    return interpolateValue(clampToGrid(x));
}

} // namespace ctrl
