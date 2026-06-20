/**
 * @file ex87_lyapunov_robust.cpp
 * @brief Robustness Phase 5: common quadratic Lyapunov function for polytopic uncertainty.
 *
 * Searches for a single P > 0 such that V(x) = x'Px is a Lyapunov function for every
 * vertex of a polytopic uncertain state matrix A(t) in conv{A_1, ..., A_L} simultaneously.
 *
 *   (a) Single stable scalar vertex A=0.5  -> recovers the analytic Lyapunov solution.
 *   (b) Tight cluster of vertices (+/-10% box around A=0.5) -> common P found.
 *   (c) Cluster including a clearly unstable vertex (A=1.5) -> no common P.
 *
 * Expected output:
 *   [A] single vertex: found, P matches the analytic value
 *   [B] tight cluster: found
 *   [C] unstable vertex included: not found
 *   [PASS] All checks passed.
 */

#include <ControllerToolbox.h>
#include <iomanip>
#include <iostream>

int main()
{
    std::cout << "\n=== Robustness Phase 5: common quadratic Lyapunov function ===\n";
    std::cout << std::fixed << std::setprecision(6);

    // ----- (a) Single stable scalar vertex --------------------------------
    Eigen::MatrixXd A_single(1, 1); A_single << 0.5;
    const auto res_single = ctrl::findCommonLyapunov({A_single});
    const double analytic_P = 1.0 / 0.75; // Q / (1 - A^2), Q = I

    std::cout << "  [A] single vertex A=0.5: found=" << res_single.found
              << "  P=" << res_single.P(0, 0)
              << "  (analytic=" << analytic_P << ")\n";

    // ----- (b) Tight cluster of vertices around the same nominal -----------
    Eigen::MatrixXd dirs(1, 1); dirs << 0.05; // +/-10% box around A=0.5
    const auto vertices_tight = ctrl::buildBoxVertices(A_single, dirs);
    const auto res_tight = ctrl::findCommonLyapunov(vertices_tight);

    std::cout << "  [B] tight cluster (" << vertices_tight.size() << " vertices): found="
              << res_tight.found << "  residual=" << res_tight.residual << "\n";

    // ----- (c) Cluster including a clearly unstable vertex ------------------
    Eigen::MatrixXd A_unstable(1, 1); A_unstable << 1.5;
    const bool stable_q   = ctrl::isQuadraticallyStable({A_single});
    const bool unstable_q = ctrl::isQuadraticallyStable({A_single, A_unstable});

    std::cout << "  [C] {0.5}: quadratically stable=" << stable_q
              << "   {0.5, 1.5}: quadratically stable=" << unstable_q << "\n";

    // ----- Checks ------------------------------------------------------------
    bool a_ok = res_single.found && std::abs(res_single.P(0, 0) - analytic_P) < 1e-4;
    bool b_ok = res_tight.found && res_tight.residual < 0.0;
    bool c_ok = stable_q && !unstable_q;

    std::cout << "  [A] " << (a_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [B] " << (b_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [C] " << (c_ok ? "PASS" : "FAIL") << "\n";

    if (a_ok && b_ok && c_ok)
    {
        std::cout << "\n[PASS] All checks passed.\n";
        return 0;
    }
    std::cout << "\n[FAIL] One or more checks failed.\n";
    return 1;
}
