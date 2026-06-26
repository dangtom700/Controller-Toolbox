/**
 * @file ex111_narmax.cpp
 * @brief Phase 3 (SI4): polynomial NARMAX identification via orthogonal forward regression.
 *
 * Generates a known bilinear NARX system
 *   y[k] = 0.5 y[k-1] + 0.3 u[k-1] + 0.2 y[k-1] u[k-1]
 * driven by white noise, fits it on the first 500 samples, and checks that (a) the three true
 * terms are selected and (b) one-step-ahead prediction on the held-out tail is accurate.
 */

#include "ControllerToolbox.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

int main()
{
    const int N = 600;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Eigen::VectorXd u(N), y(N);
    y(0) = 0.0;
    u(0) = dist(rng);
    for (int k = 1; k < N; ++k)
    {
        u(k) = dist(rng);
        y(k) = 0.5 * y(k - 1) + 0.3 * u(k - 1) + 0.2 * y(k - 1) * u(k - 1);
    }

    const int Nfit = 500;
    ctrl::NARMAXParams p;
    p.na = 1;
    p.nb = 1;
    p.nc = 0;
    p.poly_degree = 2;
    p.significance_tol = 1e-4;
    p.max_terms = 6;

    const ctrl::NARMAXResult res =
        ctrl::NARMAXIdentifier::fit(u.head(Nfit), y.head(Nfit), p);

    std::printf("Selected %zu terms, cumulative ERR = %.5f:\n", res.selected_terms.size(),
                res.final_err_sum);
    for (std::size_t i = 0; i < res.selected_terms.size(); ++i)
        std::printf("  %-16s coeff=%.4f\n", res.selected_terms[i].c_str(), res.coefficients(i));

    auto hasTerm = [&](const std::string &t) {
        return std::find(res.selected_terms.begin(), res.selected_terms.end(), t) !=
               res.selected_terms.end();
    };
    const bool terms_ok = hasTerm("y(k-1)") && hasTerm("u(k-1)") && hasTerm("y(k-1)*u(k-1)");

    // One-step-ahead prediction on the held-out tail.
    double sse = 0.0;
    int    cnt = 0;
    for (int k = Nfit; k < N; ++k)
    {
        Eigen::VectorXd u_hist(1), y_hist(1);
        u_hist << u(k - 1);
        y_hist << y(k - 1);
        const double yhat = ctrl::NARMAXIdentifier::predict(res, u_hist, y_hist);
        sse += (yhat - y(k)) * (yhat - y(k));
        ++cnt;
    }
    const double rmse = std::sqrt(sse / cnt);
    std::printf("Held-out one-step RMSE = %.3e\n", rmse);

    const bool ok = terms_ok && res.final_err_sum > 0.999 && rmse < 1e-6;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
