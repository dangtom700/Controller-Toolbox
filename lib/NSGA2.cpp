#include "NSGA2.h"
#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace ctrl
{

namespace
{

bool dominates(const Eigen::VectorXd &a, const Eigen::VectorXd &b)
{
    bool anyStrictlyLess = false;
    for (int i = 0; i < a.size(); ++i)
    {
        if (a(i) > b(i)) return false;
        if (a(i) < b(i)) anyStrictlyLess = true;
    }
    return anyStrictlyLess;
}

// Fast non-dominated sort (Deb et al. 2002): returns fronts as lists of indices into objs,
// and fills rank[i] = front index containing i.
std::vector<std::vector<int>> fastNonDominatedSort(const std::vector<Eigen::VectorXd> &objs,
                                                    std::vector<int> &rank)
{
    const int n = static_cast<int>(objs.size());
    rank.assign(n, -1);
    std::vector<int> dominationCount(n, 0);
    std::vector<std::vector<int>> dominatedSet(n);
    std::vector<std::vector<int>> fronts;

    std::vector<int> front0;
    for (int p = 0; p < n; ++p)
    {
        for (int q = 0; q < n; ++q)
        {
            if (p == q) continue;
            if (dominates(objs[p], objs[q])) dominatedSet[p].push_back(q);
            else if (dominates(objs[q], objs[p])) ++dominationCount[p];
        }
        if (dominationCount[p] == 0)
        {
            rank[p] = 0;
            front0.push_back(p);
        }
    }
    fronts.push_back(front0);

    int f = 0;
    while (!fronts[f].empty())
    {
        std::vector<int> next;
        for (int p : fronts[f])
            for (int q : dominatedSet[p])
            {
                if (--dominationCount[q] == 0)
                {
                    rank[q] = f + 1;
                    next.push_back(q);
                }
            }
        fronts.push_back(next);
        ++f;
    }
    if (!fronts.empty() && fronts.back().empty()) fronts.pop_back();
    return fronts;
}

// Crowding distance within one front, aligned with frontIdx's order.
std::vector<double> crowdingDistance(const std::vector<int> &frontIdx,
                                      const std::vector<Eigen::VectorXd> &objs, int nObj)
{
    const int m = static_cast<int>(frontIdx.size());
    std::vector<double> dist(m, 0.0);
    if (m == 0) return dist;

    for (int k = 0; k < nObj; ++k)
    {
        std::vector<int> order(m);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return objs[frontIdx[a]](k) < objs[frontIdx[b]](k);
        });

        const double minV = objs[frontIdx[order.front()]](k);
        const double maxV = objs[frontIdx[order.back()]](k);
        const double range = maxV - minV;

        dist[order.front()] = std::numeric_limits<double>::infinity();
        dist[order.back()] = std::numeric_limits<double>::infinity();
        for (int idx = 1; idx < m - 1; ++idx)
        {
            if (range > 1e-300)
                dist[order[idx]] += (objs[frontIdx[order[idx + 1]]](k) -
                                      objs[frontIdx[order[idx - 1]]](k)) / range;
        }
    }
    return dist;
}

} // namespace

NSGA2::NSGA2(const NSGA2Params &p) : p_(p), rng_(p.seed)
{
    if (p_.population < 4)
        throw std::invalid_argument("NSGA2: population must be >= 4");
    if (p_.lower.size() != p_.n_dim || p_.upper.size() != p_.n_dim)
        throw std::invalid_argument("NSGA2: bounds size must equal n_dim");
}

ParetoResult NSGA2::optimize(const MultiCostFn &cost)
{
    const int N = p_.population;
    const int n = p_.n_dim;

    std::vector<Eigen::VectorXd> pop(N, Eigen::VectorXd(n));
    std::vector<Eigen::VectorXd> objs(N);
    int nEvals = 0;
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            std::uniform_real_distribution<double> ud(p_.lower(j), p_.upper(j));
            pop[i](j) = ud(rng_);
        }
        objs[i] = cost(pop[i]);
        ++nEvals;
    }

    std::vector<int> rank;
    auto fronts = fastNonDominatedSort(objs, rank);
    std::vector<double> crowd(N, 0.0);
    for (auto &fr : fronts)
    {
        const auto d = crowdingDistance(fr, objs, p_.n_objectives);
        for (size_t k = 0; k < fr.size(); ++k) crowd[fr[k]] = d[k];
    }

    std::uniform_int_distribution<int> uidN(0, N - 1);
    std::uniform_real_distribution<double> ud01(0.0, 1.0);

    auto better = [&](int a, int b) {
        if (rank[a] != rank[b]) return rank[a] < rank[b];
        return crowd[a] > crowd[b];
    };
    auto tournament = [&]() {
        const int a = uidN(rng_), b = uidN(rng_);
        return better(a, b) ? a : b;
    };

    int gen = 0;
    for (; gen < p_.max_gen; ++gen)
    {
        std::vector<Eigen::VectorXd> offspring(N, Eigen::VectorXd(n));
        std::vector<Eigen::VectorXd> offObjs(N);
        for (int child = 0; child < N; ++child)
        {
            const int p1 = tournament(), p2 = tournament();
            Eigen::VectorXd kid(n);
            if (ud01(rng_) < p_.crossover)
            {
                for (int j = 0; j < n; ++j)
                {
                    const double lo = std::min(pop[p1](j), pop[p2](j));
                    const double hi = std::max(pop[p1](j), pop[p2](j));
                    const double range = hi - lo;
                    std::uniform_real_distribution<double> blx(lo - p_.alpha * range,
                                                                hi + p_.alpha * range);
                    kid(j) = blx(rng_);
                }
            }
            else
            {
                kid = (ud01(rng_) < 0.5) ? pop[p1] : pop[p2];
            }
            for (int j = 0; j < n; ++j)
            {
                if (ud01(rng_) < p_.mutation)
                {
                    const double sigma = 0.1 * (p_.upper(j) - p_.lower(j));
                    std::normal_distribution<double> nd(0.0, sigma);
                    kid(j) += nd(rng_);
                }
                kid(j) = std::max(p_.lower(j), std::min(p_.upper(j), kid(j)));
            }
            offspring[child] = kid;
            offObjs[child] = cost(kid);
            ++nEvals;
        }

        std::vector<Eigen::VectorXd> combinedPop = pop;
        std::vector<Eigen::VectorXd> combinedObjs = objs;
        combinedPop.insert(combinedPop.end(), offspring.begin(), offspring.end());
        combinedObjs.insert(combinedObjs.end(), offObjs.begin(), offObjs.end());

        std::vector<int> combinedRank;
        auto combinedFronts = fastNonDominatedSort(combinedObjs, combinedRank);

        std::vector<Eigen::VectorXd> nextPop;
        std::vector<Eigen::VectorXd> nextObjs;
        nextPop.reserve(N);
        nextObjs.reserve(N);

        for (auto &fr : combinedFronts)
        {
            if (static_cast<int>(nextPop.size() + fr.size()) <= N)
            {
                for (int idx : fr)
                {
                    nextPop.push_back(combinedPop[idx]);
                    nextObjs.push_back(combinedObjs[idx]);
                }
            }
            else
            {
                const auto d = crowdingDistance(fr, combinedObjs, p_.n_objectives);
                std::vector<int> order(fr.size());
                std::iota(order.begin(), order.end(), 0);
                std::sort(order.begin(), order.end(),
                          [&](int a, int b) { return d[a] > d[b]; });
                const int remaining = N - static_cast<int>(nextPop.size());
                for (int k = 0; k < remaining; ++k)
                {
                    nextPop.push_back(combinedPop[fr[order[k]]]);
                    nextObjs.push_back(combinedObjs[fr[order[k]]]);
                }
            }
            if (static_cast<int>(nextPop.size()) >= N) break;
        }

        pop = std::move(nextPop);
        objs = std::move(nextObjs);

        fronts = fastNonDominatedSort(objs, rank);
        for (auto &fr : fronts)
        {
            const auto d = crowdingDistance(fr, objs, p_.n_objectives);
            for (size_t k = 0; k < fr.size(); ++k) crowd[fr[k]] = d[k];
        }
    }

    ParetoResult result;
    const auto &front0 = fronts.front();
    result.front_params.resize(static_cast<int>(front0.size()), n);
    result.front_objectives.resize(static_cast<int>(front0.size()), p_.n_objectives);
    for (size_t i = 0; i < front0.size(); ++i)
    {
        result.front_params.row(static_cast<int>(i)) = pop[front0[i]];
        result.front_objectives.row(static_cast<int>(i)) = objs[front0[i]];
    }
    result.nGens = gen;
    result.nEvals = nEvals;
    return result;
}

} // namespace ctrl
