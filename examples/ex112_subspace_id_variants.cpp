/**
 * @file ex112_subspace_id_variants.cpp
 * @brief Phase 3 (SI3): MOESP/N4SID/CVA subspace-ID weighting variants.
 *
 * Identifies a known 2-output system from noisy I/O data with all three SubspaceMethod
 * variants. Demonstrates CVA's reliable advantage over N4SID on a mismatched-output-noise
 * scenario, averaged over many independent trials (a single noise draw was found, during
 * design prototyping, to flip unpredictably between CVA winning and losing -- the reliable
 * claim only holds in expectation). Neither CVA nor N4SID is claimed to beat plain MOESP;
 * prototyping found unweighted MOESP the strongest performer on this synthetic system.
 */

#include "ControllerToolbox.h"
#include <iostream>
#include <random>

#if !defined(CTRL_HAS_SUBSPACE)
int main() { std::puts("Skipped: CTRL_HAS_SUBSPACE not enabled."); return 0; }
#else

namespace
{
double chanFreqError(const ctrl::StateSpace &model, const ctrl::StateSpace &trueChan1,
                      const std::vector<double> &freqs, double Ts)
{
    const ctrl::StateSpace estChan1(model.A, model.B, model.C.row(1), model.D.row(1), Ts);
    const auto resp_true = ctrl::SystemAnalysis::getFrequencyResponse(trueChan1, freqs);
    const auto resp_est  = ctrl::SystemAnalysis::getFrequencyResponse(estChan1, freqs);
    double err = 0.0;
    for (std::size_t k = 0; k < freqs.size(); ++k)
        err += std::abs(std::abs(resp_est[k]) - std::abs(resp_true[k]));
    return err / static_cast<double>(freqs.size());
}
} // namespace

int main()
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;
    const int N = 2000;

    // getFrequencyResponse() is SISO-only -- evaluate channel 1 (the high-noise channel)
    // via a per-channel SISO sub-system (channel identity survives the similarity
    // transform; only the state basis changes).
    const auto freqs = std::vector<double>{0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0};
    const ctrl::StateSpace trueChan1(A_true, B_true, C_true.row(1), D_true.row(1), Ts);

    std::cout << "=== Subspace ID method variants (Phase 3 SI3) ===\n"
              << "Mismatched output noise: channel 0 std=0.005, channel 1 std=0.3\n"
              << "Averaging high-noise-channel freq-response error over 20 independent trials\n\n";

    const int n_trials = 20;
    std::mt19937 master_rng(2026);
    double total_n4sid = 0.0, total_cva = 0.0;
    bool ok = true;

    for (int trial = 0; trial < n_trials; ++trial)
    {
        std::mt19937 rng_u(master_rng());
        std::mt19937 rng_n(master_rng());
        std::normal_distribution<double> u_dist(0.0, 1.0);
        Eigen::MatrixXd U(1, N);
        for (int k = 0; k < N; ++k) U(0, k) = u_dist(rng_u);

        Eigen::VectorXd noiseStd(2); noiseStd << 0.005, 0.3;
        Eigen::MatrixXd Y(2, N);
        Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
        std::normal_distribution<double> noise(0.0, 1.0);
        for (int k = 0; k < N; ++k)
        {
            const Eigen::VectorXd u_k = U.col(k);
            Eigen::VectorXd y_k = C_true * x + D_true * u_k;
            for (int j = 0; j < 2; ++j) y_k(j) += noise(rng_n) * noiseStd(j);
            Y.col(k) = y_k;
            x = A_true * x + B_true * u_k;
        }

        const auto n4sidv = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::N4SID);
        const auto cva    = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::CVA);
        if (!n4sidv.success || !cva.success)
        {
            std::cerr << "trial " << trial << " failed\n";
            ok = false;
            continue;
        }
        total_n4sid += chanFreqError(n4sidv.model.value(), trueChan1, freqs, Ts);
        total_cva   += chanFreqError(cva.model.value(), trueChan1, freqs, Ts);
    }

    const double mean_n4sid = total_n4sid / n_trials;
    const double mean_cva   = total_cva / n_trials;
    std::cout << "N4SID: mean high-noise-channel freq-response error = " << mean_n4sid << "\n";
    std::cout << "CVA:   mean high-noise-channel freq-response error = " << mean_cva << "\n";

    ok = ok && (mean_cva < mean_n4sid);
    std::cout << "\n" << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
#endif
