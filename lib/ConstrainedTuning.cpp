#include "ConstrainedTuning.h"

namespace ctrl
{

TunerResult tuneConstrained(
    std::function<TunerResult(const AutoTuner::CostFn &, const Eigen::VectorXd &)> optimizerRun,
    const AutoTuner::CostFn &objective, const ConstrainedTuneParams &params,
    const Eigen::VectorXd &x0)
{
    double mu = params.penalty_init;
    Eigen::VectorXd x = x0;
    int totalEvals = 0, totalGens = 0;
    TunerResult innerResult;

    for (int outer = 0; outer < params.outer_iters; ++outer)
    {
        const AutoTuner::CostFn penalizedCost = [&](const Eigen::VectorXd &theta) {
            double cost = objective(theta);
            if (params.constraints)
            {
                const Eigen::VectorXd g = params.constraints(theta);
                cost += mu * g.cwiseMax(0.0).squaredNorm();
            }
            return cost;
        };

        innerResult = optimizerRun(penalizedCost, x);
        x = innerResult.params;
        totalEvals += innerResult.nEvals;
        totalGens += innerResult.nGens;
        mu *= params.penalty_growth;
    }

    bool feasible = true;
    if (params.constraints)
        feasible = params.constraints(x).maxCoeff() <= params.feasTol;

    TunerResult result;
    result.params = x;
    result.cost = objective(x);
    result.nEvals = totalEvals;
    result.nGens = totalGens;
    result.converged = innerResult.converged && feasible;
    return result;
}

} // namespace ctrl
