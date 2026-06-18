#pragma once
#include "PlantModel.h"
#include <vector>
#include <Eigen/Dense>

/**
 * @file LPVSystemID.h
 * @brief Linear Parameter-Varying system identification via polynomial regression.
 *
 * Identifies a discrete-time LPV model:
 * @code
 *   x[k+1] = A(p[k]) x[k] + B(p[k]) u[k]
 *   y[k]   = C(p[k]) x[k] + D(p[k]) u[k]
 * @endcode
 *
 * where each matrix has polynomial dependence on the scalar scheduling parameter p:
 * @code
 *   A(p) = A0 + p*A1 + p^2*A2 + ... + p^d*Ad   (degree d)
 * @endcode
 *
 * **Regression formulation** (state equation, degree d, n states, m inputs, N steps):
 *
 * Stack row-regressors phi_k = [x^T, p*x^T, ..., p^d*x^T, u^T, p*u^T, ..., p^d*u^T]
 * (1 * (n+m)*(d+1) wide) and solve the overdetermined system:
 * @code
 *   Phi [(N-1) x (n+m)(d+1)] * Theta_AB [(n+m)(d+1) x n] = Z_x [(N-1) x n]
 * @endcode
 * via colPivHouseholderQr() (handles rank deficiency when p is nearly constant).
 *
 * @par Typical workflow (state sequence known)
 * @code
 *   // X (n*N), U (m*N), Y (p_out*N) are data matrices, sched has N values.
 *   ctrl::LPVModel model = ctrl::identifyLPV(X, U, Y, sched, 1, Ts);
 *   ctrl::StateSpace frozen = model.frozen(0.5);   // linearise at p = 0.5
 * @endcode
 *
 * @par Typical workflow (I/O data only)
 * @code
 *   // n4sid estimates the state sequence internally at nominal p.
 *   ctrl::LPVModel model = ctrl::identifyLPVFromIO(U, Y, sched, n, 1, Ts);
 * @endcode
 *
 * @par Reference
 * Toth, R. (2010). *Modeling and Identification of Linear Parameter-Varying Systems*.
 * Springer. Ch. 5 (regression-based ID).
 */

namespace ctrl {

/**
 * @brief Identified LPV model with polynomial parameter dependence.
 */
struct LPVModel {
    std::vector<Eigen::MatrixXd> A_coeffs; ///< A(p) = sum_{i=0}^d p^i * A_coeffs[i].
    std::vector<Eigen::MatrixXd> B_coeffs; ///< B(p) = sum_{i=0}^d p^i * B_coeffs[i].
    std::vector<Eigen::MatrixXd> C_coeffs; ///< C(p) = sum_{i=0}^d p^i * C_coeffs[i].
    std::vector<Eigen::MatrixXd> D_coeffs; ///< D(p) = sum_{i=0}^d p^i * D_coeffs[i].
    int    degree;    ///< Polynomial degree d.
    int    n_states;  ///< State dimension n.
    int    n_inputs;  ///< Input dimension m.
    int    n_outputs; ///< Output dimension p_out.
    double Ts;        ///< Sample time [s].

    /** @brief Evaluate A(p) at scheduling parameter value p. */
    Eigen::MatrixXd evalA(double p) const {
        Eigen::MatrixXd M = A_coeffs[0];
        double pk = p;
        for (int i = 1; i <= degree; ++i, pk *= p) M += pk * A_coeffs[i];
        return M;
    }
    /** @brief Evaluate B(p) at scheduling parameter value p. */
    Eigen::MatrixXd evalB(double p) const {
        Eigen::MatrixXd M = B_coeffs[0];
        double pk = p;
        for (int i = 1; i <= degree; ++i, pk *= p) M += pk * B_coeffs[i];
        return M;
    }
    /** @brief Evaluate C(p) at scheduling parameter value p. */
    Eigen::MatrixXd evalC(double p) const {
        Eigen::MatrixXd M = C_coeffs[0];
        double pk = p;
        for (int i = 1; i <= degree; ++i, pk *= p) M += pk * C_coeffs[i];
        return M;
    }
    /** @brief Evaluate D(p) at scheduling parameter value p. */
    Eigen::MatrixXd evalD(double p) const {
        Eigen::MatrixXd M = D_coeffs[0];
        double pk = p;
        for (int i = 1; i <= degree; ++i, pk *= p) M += pk * D_coeffs[i];
        return M;
    }

    /**
     * @brief Return the frozen (time-invariant) discrete-time StateSpace at p.
     *
     * Equivalent to a single-operating-point linearisation. Can be passed
     * directly to DiscreteLQR, DiscreteMPC, etc.
     */
    StateSpace frozen(double p) const {
        return StateSpace(evalA(p), evalB(p), evalC(p), evalD(p), Ts);
    }
};

/**
 * @brief Identify an LPV model given state, input, output, and scheduling data.
 *
 * Requires the state sequence to be pre-estimated (e.g., via n4sid or a Kalman
 * filter). All data must be synchronised: X(:,k) = x[k], etc.
 *
 * @param X       State sequence matrix, n x N (column k = x[k]).
 * @param U       Input sequence matrix, m x N.
 * @param Y       Output sequence matrix, p_out x N.
 * @param sched   Scheduling parameter sequence, length >= N.
 * @param degree  Polynomial degree d (>= 0, default 1 = affine LPV).
 * @param Ts      Sample time [s].
 * @return        LPVModel with estimated polynomial coefficients.
 * @throws std::invalid_argument On dimension mismatch or insufficient data.
 */
LPVModel identifyLPV(const Eigen::MatrixXd&   X,
                     const Eigen::MatrixXd&   U,
                     const Eigen::MatrixXd&   Y,
                     const std::vector<double>& sched,
                     int    degree = 1,
                     double Ts     = 0.01);

/**
 * @brief Identify an LPV model from input-output data only.
 *
 * Calls n4sid() internally to obtain a state sequence, then calls identifyLPV().
 * The n4sid call uses all the data and returns a linear model valid near the
 * mean value of p; this is then used only for state-sequence estimation.
 *
 * @param U         Input matrix, m x N.
 * @param Y         Output matrix, p_out x N.
 * @param sched     Scheduling parameter sequence, length >= N.
 * @param n_states  Desired model order.
 * @param degree    Polynomial degree (default 1).
 * @param Ts        Sample time.
 * @param n4sid_i   N4SID block-rows parameter (default 10; must be >= n_states).
 * @return          LPVModel.
 */
LPVModel identifyLPVFromIO(const Eigen::MatrixXd&   U,
                           const Eigen::MatrixXd&   Y,
                           const std::vector<double>& sched,
                           int    n_states,
                           int    degree   = 1,
                           double Ts       = 0.01,
                           int    n4sid_i  = 10);

} // namespace ctrl
