#pragma once
#include "GapMetric.h"
#include "PlantModel.h"
#include <vector>
#include <map>
#include <numeric>
#include <algorithm>
#include <functional>
#include <limits>
#include <Eigen/Dense>

/**
 * @file LinearModelCluster.h
 * @brief Single-linkage agglomerative clustering of linear models by nu-gap.
 *
 * Groups a collection of SISO discrete-time linear models into clusters such
 * that all models within a cluster are within `threshold` nu-gap distance of
 * each other. A representative model (the one with minimal average in-cluster
 * gap) is selected for each cluster.
 *
 * **Algorithm:** Single-linkage agglomerative clustering via Union-Find.
 *   Models i and j are merged into the same cluster if nuGap(i,j) < threshold.
 *   Single-linkage is appropriate here because the nu-gap threshold has a direct
 *   robust-stability interpretation: any controller that robustly stabilises the
 *   representative also stabilises every model in the cluster (Vinnicombe 2001).
 *
 * @par Typical workflow
 * @code
 *   // Compute pairwise gap matrix
 *   Eigen::MatrixXd G = ctrl::nuGapMatrix(models);
 *
 *   // Suggest an appropriate threshold
 *   double t = ctrl::suggestGapThreshold(G);
 *
 *   // Cluster
 *   ctrl::ClusterResult res = ctrl::clusterByGap(G, t);
 *   std::cout << res.numClusters << " clusters\n";
 *   for (int c : res.representatives)
 *       std::cout << "  rep at grid index " << c << "\n";
 * @endcode
 */

namespace ctrl {

/**
 * @brief Result of gap-metric clustering.
 */
struct ClusterResult {
    std::vector<int>    labels;           ///< Cluster label (0..k-1) for each input model.
    std::vector<int>    representatives;  ///< Index in original list of each cluster's representative.
    std::vector<double> maxIntraGap;      ///< Maximum nu-gap between any two models in each cluster.
    int                 numClusters;      ///< Total number of clusters k.
    double              threshold;        ///< Threshold used.
};

/**
 * @brief Cluster N models using a pre-computed nu-gap distance matrix.
 *
 * Uses single-linkage: models i and j are in the same cluster whenever
 * gapMatrix(i,j) < threshold. The representative of each cluster minimises
 * the average gap to other cluster members.
 *
 * @param gapMatrix  N*N symmetric distance matrix (diagonal = 0).
 * @param threshold  Maximum intra-cluster gap (default 0.5).
 * @return           ClusterResult with labels, representatives, and diagnostics.
 */
inline ClusterResult clusterByGap(const Eigen::MatrixXd& gapMatrix,
                                  double threshold = 0.5)
{
    const int N = static_cast<int>(gapMatrix.rows());

    // ---- Union-Find --------------------------------------------------------
    std::vector<int> parent(N);
    std::iota(parent.begin(), parent.end(), 0);

    std::function<int(int)> find = [&](int x) -> int {
        return (parent[x] == x) ? x : (parent[x] = find(parent[x]));
    };
    auto unite = [&](int a, int b) { parent[find(a)] = find(b); };

    // Merge all pairs with gap < threshold (single-linkage)
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j)
            if (gapMatrix(i, j) < threshold)
                unite(i, j);

    // ---- Assign contiguous cluster labels -----------------------------------
    std::map<int, int> rootToLabel;
    std::vector<int> labels(N);
    int k = 0;
    for (int i = 0; i < N; ++i) {
        int root = find(i);
        if (rootToLabel.find(root) == rootToLabel.end())
            rootToLabel[root] = k++;
        labels[i] = rootToLabel[root];
    }

    // ---- Select representative: minimise average in-cluster gap -------------
    std::vector<int>    reps(k, -1);
    std::vector<double> bestScore(k, std::numeric_limits<double>::max());

    for (int i = 0; i < N; ++i) {
        int li = labels[i];
        double score = 0.0;
        int    cnt   = 0;
        for (int j = 0; j < N; ++j) {
            if (labels[j] == li) { score += gapMatrix(i, j); ++cnt; }
        }
        if (cnt > 1) score /= static_cast<double>(cnt - 1);
        if (score < bestScore[li]) { bestScore[li] = score; reps[li] = i; }
    }
    // Fallback for singleton clusters
    for (int c = 0; c < k; ++c) {
        if (reps[c] == -1) {
            for (int i = 0; i < N; ++i) {
                if (labels[i] == c) { reps[c] = i; break; }
            }
        }
    }

    // ---- Max intra-cluster gap ----------------------------------------------
    std::vector<double> maxGap(k, 0.0);
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j)
            if (labels[i] == labels[j])
                maxGap[labels[i]] = std::max(maxGap[labels[i]], gapMatrix(i, j));

    ClusterResult res;
    res.labels        = labels;
    res.representatives = reps;
    res.maxIntraGap   = maxGap;
    res.numClusters   = k;
    res.threshold     = threshold;
    return res;
}

/**
 * @brief Convenience overload: compute gap matrix then cluster.
 *
 * @param models      SISO discrete-time models with the same Ts.
 * @param threshold   Nu-gap threshold (default 0.5).
 * @param freq_points Frequency resolution for gap computation (default 200).
 */
inline ClusterResult clusterByGap(const std::vector<StateSpace>& models,
                                  double threshold  = 0.5,
                                  int    freq_points = 200)
{
    return clusterByGap(nuGapMatrix(models, freq_points), threshold);
}

/**
 * @brief Suggest a gap threshold by finding the "knee" of the cluster-count curve.
 *
 * Sweeps threshold from lo to hi, records the cluster count at each value,
 * and returns the threshold where the count first becomes stable (changes
 * by at most 1 for three consecutive steps).
 *
 * @param gapMatrix  Pre-computed gap matrix.
 * @param lo         Lower threshold bound (default 0.1).
 * @param hi         Upper threshold bound (default 0.9).
 * @param steps      Number of sweep points (default 20).
 * @return           Suggested threshold value.
 */
inline double suggestGapThreshold(const Eigen::MatrixXd& gapMatrix,
                                  double lo    = 0.1,
                                  double hi    = 0.9,
                                  int    steps = 20)
{
    if (steps < 3) steps = 3;
    std::vector<double> thresholds(steps);
    std::vector<int>    counts(steps);
    for (int i = 0; i < steps; ++i) {
        thresholds[i] = lo + (hi - lo) * i / (steps - 1);
        counts[i]     = clusterByGap(gapMatrix, thresholds[i]).numClusters;
    }
    // Find first plateau: |count[i] - count[i-1]| <= 1 for two consecutive steps
    for (int i = 2; i < steps; ++i) {
        if (std::abs(counts[i] - counts[i - 1]) <= 1 &&
            std::abs(counts[i - 1] - counts[i - 2]) <= 1)
            return thresholds[i - 1];
    }
    return thresholds[steps / 2]; // fallback: midpoint
}

} // namespace ctrl
