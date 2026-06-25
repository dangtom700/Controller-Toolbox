#pragma once
#include "PlantModel.h"
#include "Features.h"
#include <Eigen/Dense>

/**
 * @file HammersteinWienerIdentifier.h
 * @brief Hammerstein/Wiener structured nonlinear system identification (Phase 3 SI5).
 *
 * Hammerstein (static input nonlinearity -> linear dynamics) and Wiener (linear dynamics ->
 * static output nonlinearity) model identification via alternating linear/nonlinear least
 * squares. Fills a gap `docs/algorithm_backlog.md` flags directly ("no current equivalent"),
 * despite this structure being extremely common (valve/actuator deadzone or saturation,
 * sensor saturation).
 *
 * **Deviation from the roadmap's "reuse RecursiveLeastSquares" claim:**
 * `RecursiveLeastSquares::update()` is strictly sample-by-sample/online - there is no batch
 * entry point. The linear sub-step here uses a direct batch ARX least-squares solve (build
 * the regressor matrix once, solve via QR) instead of looping the recursive filter.
 *
 * **Scale-ambiguity normalization:** Hammerstein/Wiener separation has a fundamental scale
 * ambiguity between the static nonlinearity's coefficients and the linear part's gain.
 * Resolved by fixing the polynomial's linear-term coefficient (`nl_input_coeffs[1]` /
 * `nl_output_coeffs[1]`) to 1.0 after every outer iteration, absorbing the corresponding
 * scale factor into the linear part's gain instead.
 *
 * @see docs/superpowers/specs/2026-06-24-hammerstein-wiener-design.md
 */

namespace ctrl {

/** @brief Parameters for HammersteinWienerIdentifier. */
struct HammersteinWienerParams
{
    int    na = 1, nb = 1;     ///< Linear ARX orders.
    int    nl_degree = 3;      ///< Polynomial degree for the static nonlinearity.
    int    max_iter   = 20;
    double tol         = 1e-6;
};

/** @brief Result of HammersteinWienerIdentifier::fitHammerstein/fitWiener. */
struct HammersteinWienerResult
{
    Eigen::VectorXd  nl_input_coeffs;   ///< Hammerstein static map [c0..c_d], c1 fixed = 1.0.
    Eigen::VectorXd  nl_output_coeffs;  ///< Wiener static map [d0..d_d], d1 fixed = 1.0
                                          ///< (empty for fitHammerstein's result).
    TransferFunction linear_part;
    bool             converged = false;
    int              iters     = 0;
};

/**
 * @brief Hammerstein/Wiener structured nonlinear identification.
 */
class HammersteinWienerIdentifier
{
public:
    /**
     * @brief Fit a Hammerstein model: v[k] = poly(u[k]), then linear ARX(v -> y).
     * @throws std::invalid_argument If `u`/`y` differ in length, or there are too few
     *         samples for the requested `na`/`nb`/`nl_degree`.
     */
    static HammersteinWienerResult fitHammerstein(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                                    double Ts, const HammersteinWienerParams &params = {});

    /**
     * @brief Fit a Wiener model: linear ARX(u -> w), then y[k] = poly(w[k]).
     * @throws std::invalid_argument Same conditions as fitHammerstein.
     */
    static HammersteinWienerResult fitWiener(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                               double Ts, const HammersteinWienerParams &params = {});
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(hammerstein_wiener)
