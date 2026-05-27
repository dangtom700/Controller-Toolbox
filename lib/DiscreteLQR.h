#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>
#include <functional>
#include <stdexcept>

/**
 * @file DiscreteLQR.h
 * @brief Discrete-time Linear Quadratic Regulator (LQR).
 *
 * Solves the Discrete Algebraic Riccati Equation (DARE) offline via value iteration, then
 * applies the optimal state-feedback law online.
 *
 * **DARE (value iteration):**
 * @code
 *   P∞ = AᵀP∞A − (AᵀP∞B)(R + BᵀP∞B)⁻¹(BᵀP∞A) + Q
 *   K* = (R + BᵀP∞B)⁻¹ BᵀP∞A
 * @endcode
 *
 * **Online control law:**
 * @code
 *   u[k] = −K*·(x[k] − x_ref) + u_ff
 * @endcode
 *
 * For output feedback, reconstruct the full state with an observer (e.g., KalmanFilter)
 * before calling compute().
 *
 * @see Anderson & Moore, "Optimal Control" (1990).
 * @see MATLAB dlqr(), Simulink Optimal LQR Controller block.
 */

namespace ctrl
{

/**
 * @brief Result returned by the DARE solver.
 *
 * Carries the solution matrix, convergence flag, and iteration count so callers can decide
 * whether to accept an approximate result.
 */
struct DareResult
{
    Eigen::MatrixXd P;         ///< Best available Riccati solution (converged or last iterate).
    bool            converged; ///< @c true if value iteration reached the tolerance.
    int             iterations;///< Actual number of iterations performed.
};

/**
 * @brief LQR weighting matrices.
 */
struct LQRParams
{
    Eigen::MatrixXd Q; ///< State cost (n × n, positive semi-definite). Increase Q_ii to tighten tracking of state i.
    Eigen::MatrixXd R; ///< Control cost (m × m, positive definite). Increase R_jj to penalise actuator j.
};

/**
 * @brief Discrete-time Linear Quadratic Regulator.
 *
 * Stateless at runtime — no internal memory between compute() calls. Pass the actual
 * plant state (from a sensor or state observer) to compute() at every step to benefit
 * from closed-loop disturbance rejection.
 */
class DiscreteLQR
{
public:
    /**
     * @brief Construct, run PBH stabilisability/detectability checks, and solve the DARE.
     *
     * Prints a warning to stderr if DARE does not converge; uses the best available iterate.
     *
     * @param plant  Discrete-time plant model (A, B, C, D, Ts).
     * @param params LQR weighting matrices Q and R.
     *
     * @note PBH rank test: stabilisability and detectability are checked using
     *       Eigen's fullPivLu().rank() with an automatic threshold scaled by
     *       max_singular_value × n × ε. For plants with eigenvalues very close to the
     *       unit circle (e.g., 0.9999 or 1.0001), this may produce a false failure.
     *       If suspected, verify manually: rank([A − λI, B]) == n for all |λ| ≥ 1.
     */
    DiscreteLQR(const StateSpace &plant, const LQRParams &params);

    /**
     * @brief Compute u[k] = −K*·(x − x_ref) + u_ff.
     *
     * @p x_ref and @p u_ff default to zero when empty (no-argument versions).
     *
     * @param x     Current state vector x[k] (n × 1).
     * @param x_ref Reference state x_ref[k] (n × 1). Pass empty for regulation to origin.
     * @param u_ff  Feed-forward term u_ff[k] (m × 1). Pass empty for zero feed-forward.
     * @return Control action u[k] (m × 1).
     *
     * @note If the returned u[k] is clamped by an external actuator, always pass the
     *       *actual* measured state to compute() rather than integrating with the
     *       unsaturated u[k]. The observer's measurement update implicitly corrects for
     *       saturation. For explicit anti-windup in state-feedback loops, see
     *       Åström & Wittenmark §9.3.
     */
    Eigen::VectorXd compute(const Eigen::VectorXd &x,
                            const Eigen::VectorXd &x_ref = Eigen::VectorXd(),
                            const Eigen::VectorXd &u_ff  = Eigen::VectorXd()) const;

    /** @brief Optimal feedback gain matrix K* (m × n). */
    const Eigen::MatrixXd &gainMatrix()       const { return K_; }

    /** @brief DARE stabilising solution P∞ (n × n). */
    const Eigen::MatrixXd &riccatiSolution()  const { return P_; }

    /** @brief @c true if the DARE converged to the requested tolerance. */
    bool   dareConverged()   const { return dare_converged_; }

    /** @brief Number of value-iteration steps taken by the DARE solver. */
    int    dareIterations()  const { return dare_iterations_; }

    /** @brief Sample time Ts [s]. */
    double sampleTime()      const { return Ts_; }

private:
    Eigen::MatrixXd K_; ///< Optimal feedback gain (m × n).
    Eigen::MatrixXd P_; ///< DARE stabilising solution (n × n).
    double Ts_;
    int n_, m_;
    bool dare_converged_;
    int  dare_iterations_;

    /** @brief Value-iteration DARE solver — never throws; convergence indicated in result. */
    static DareResult solveDARE(const Eigen::MatrixXd &A,
                                const Eigen::MatrixXd &B,
                                const Eigen::MatrixXd &Q,
                                const Eigen::MatrixXd &R);
};

/**
 * @brief Adapter that wraps DiscreteLQR as an IController for use in ControllerStack.
 *
 * State and reference are supplied via std::function callbacks, decoupling the LQR from
 * specific sensor or trajectory objects. For SISO plants, the scalar IController::compute()
 * interface is used; the adapter extracts the first element of the LQR control vector.
 */
class LQRAdapter : public IController
{
public:
    /**
     * @brief Construct the adapter with state and optional reference providers.
     * @param lqr           The underlying DiscreteLQR solver.
     * @param stateProvider Callable returning the current state vector x[k] (n × 1).
     * @param refProvider   Callable returning the reference state x_ref[k] (n × 1);
     *                      may return an empty vector for regulation to origin.
     */
    LQRAdapter(DiscreteLQR &lqr,
               std::function<Eigen::VectorXd()> stateProvider,
               std::function<Eigen::VectorXd()> refProvider = {})
        : lqr_(lqr), stateFn_(std::move(stateProvider)), refFn_(std::move(refProvider))
    {
    }

    /**
     * @brief Compute u[k] — @p signal is ignored; state and reference come from callbacks.
     * @param signal Unused (inherited interface).
     * @return First element of the LQR control vector (SISO extraction).
     */
    double compute(double /*signal*/) override
    {
        Eigen::VectorXd x_ref;
        if (refFn_)
            x_ref = refFn_();
        return lqr_.compute(stateFn_(), x_ref)(0);
    }

    /** @brief No-op — DiscreteLQR is stateless at runtime. */
    void reset() override {}

    /** @brief Sample time inherited from the underlying LQR. */
    double sampleTime() const override { return lqr_.sampleTime(); }

    /**
     * @brief Health reflects DARE convergence.
     *
     * Returns @c false if DARE did not converge at construction; the gain matrix K* is an
     * approximate iterate that may not be stabilising. ControllerStack will skip this entry
     * and fall back to the next eligible controller.
     *
     * @return @c true if DARE converged; @c false otherwise.
     */
    bool isHealthy() const override { return lqr_.dareConverged(); }

private:
    DiscreteLQR &lqr_;
    std::function<Eigen::VectorXd()> stateFn_;
    std::function<Eigen::VectorXd()> refFn_;
};

} // namespace ctrl
