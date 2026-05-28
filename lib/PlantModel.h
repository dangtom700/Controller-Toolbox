#pragma once
#include <vector>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>

/**
 * @file PlantModel.h
 * @brief Discrete-time plant model representations and utilities.
 *
 * Provides TransferFunction and StateSpace data structures, plus conversion and simulation
 * utilities (tf2ss, ss2tf, ssStep, c2d, minreal) that parallel MATLAB's control toolbox.
 *
 * @see MATLAB tf(), ss(), tf2ss(), c2d() documentation.
 */

namespace ctrl
{

/**
 * @brief Discrete-time transfer function in z^-^1 polynomial form.
 *
 * @code
 *   H(z^-^1) = (b0 + b1.z^-^1 + ... + bm.z^-^m)
 *             -----------------------------
 *             (1  + a1.z^-^1 + ... + an.z^-^n)
 * @endcode
 *
 * Equivalent MATLAB: `G = tf(num, den, Ts, 'Variable', 'z^-1')`
 */
struct TransferFunction
{
    std::vector<double> num; ///< Numerator coefficients [b0, b1, ..., bm].
    std::vector<double> den; ///< Monic denominator [1, a1, ..., an]; den[0] must equal 1.
    double Ts;               ///< Sample time [s].

    /**
     * @brief Construct and validate a transfer function.
     * @param numerator   Numerator coefficient vector [b0, b1, ..., bm].
     * @param denominator Monic denominator vector [1, a1, ..., an].
     * @param sampleTime  Sample period Ts [s].
     * @throws std::invalid_argument If denominator is empty or den[0] != 1.
     */
    TransferFunction(std::vector<double> numerator,
                     std::vector<double> denominator,
                     double sampleTime)
        : num(std::move(numerator)), den(std::move(denominator)), Ts(sampleTime)
    {
        if (den.empty() || std::abs(den[0] - 1.0) > 1e-9)
            throw std::invalid_argument("TransferFunction: denominator must be monic (den[0] = 1).");
    }

    /** @brief Denominator order n. */
    int order() const { return static_cast<int>(den.size()) - 1; }
};

/**
 * @brief Discrete-time state-space model.
 *
 * @code
 *   x[k+1] = A.x[k] + B.u[k]
 *   y[k]   = C.x[k] + D.u[k]
 * @endcode
 *
 * Equivalent MATLAB: `sys = ss(A, B, C, D, Ts)`
 */
struct StateSpace
{
    Eigen::MatrixXd A; ///< State-transition matrix (n * n).
    Eigen::MatrixXd B; ///< Input matrix (n * m).
    Eigen::MatrixXd C; ///< Output matrix (p * n).
    Eigen::MatrixXd D; ///< Feedthrough matrix (p * m).
    double Ts;         ///< Sample time [s]; 0.0 indicates continuous time (used by c2d()).

    /**
     * @brief Construct and validate a state-space model.
     * @param a          State-transition matrix (n * n).
     * @param b          Input matrix (n * m).
     * @param c          Output matrix (p * n).
     * @param d          Feedthrough matrix (p * m).
     * @param sampleTime Sample period Ts [s]; pass 0.0 for continuous-time models.
     * @throws std::invalid_argument If matrix dimensions are inconsistent.
     */
    StateSpace(Eigen::MatrixXd a,
               Eigen::MatrixXd b,
               Eigen::MatrixXd c,
               Eigen::MatrixXd d,
               double sampleTime)
        : A(std::move(a)), B(std::move(b)), C(std::move(c)), D(std::move(d)), Ts(sampleTime)
    {
        validate();
    }

    int stateSize()  const { return static_cast<int>(A.rows()); } ///< Number of states n.
    int inputSize()  const { return static_cast<int>(B.cols()); } ///< Number of inputs m.
    int outputSize() const { return static_cast<int>(C.rows()); } ///< Number of outputs p.

    /**
     * @brief Validate that all matrix dimensions are mutually consistent.
     *
     * Called automatically by the constructor. Call manually after modifying matrices in-place.
     * @throws std::invalid_argument On any dimension mismatch.
     */
    void validate() const;
};

/**
 * @brief Result returned by Discrete Algebraic Riccati Equation (DARE) solvers.
 *
 * Shared between DiscreteLQR (value-iteration doubling) and DiscreteHinf (symplectic
 * pencil Schur method). Using a common type allows diagnostics code to handle both
 * without knowing which solver produced the result.
 */
struct DareResult
{
    Eigen::MatrixXd P;          ///< Best available Riccati solution (converged or last iterate).
    bool            converged;  ///< @c true if the solver reached its convergence criterion.
    int             iterations; ///< Number of solver iterations performed.
};

/**
 * @brief Convert a SISO discrete transfer function to controllable canonical state-space form.
 *
 * Equivalent MATLAB: `[A,B,C,D] = tf2ss(num, den)` applied to the z^-^1 polynomial.
 *
 * @param tf Source transfer function (SISO, z^-^1 form).
 * @return Equivalent StateSpace model in controllable canonical form.
 * @see Ogata, "Modern Control Engineering"; MATLAB tf2ss documentation.
 */
StateSpace tf2ss(const TransferFunction &tf);

/**
 * @brief Simulate one step of a state-space model with an in-place state update.
 *
 * Execution order:
 * 1. Compute y[k] = C.x[k] + D.u[k]
 * 2. Advance x[k+1] = A.x[k] + B.u[k]  (x updated in-place)
 *
 * @p x accepts both fixed-size (Vector2d) and dynamic (VectorXd) vectors via Eigen::Ref.
 *
 * @param sys Plant model.
 * @param x   State vector x[k] (modified in-place to x[k+1]).
 * @param u   Input vector u[k].
 * @return Output vector y[k].
 */
Eigen::VectorXd ssStep(const StateSpace &sys,
                       Eigen::Ref<Eigen::VectorXd> x,
                       const Eigen::VectorXd &u);

/**
 * @brief Non-mutating variant of ssStep - returns {y[k], x[k+1]}.
 *
 * Identical semantics to ssStep() but takes x by value and returns the
 * updated state alongside the output, so the caller's state is never
 * modified in-place.  Preferred for Python bindings where Eigen::Ref
 * in-place mutation is not directly supported.
 *
 * @param sys Plant model.
 * @param x   State vector x[k] (not modified).
 * @param u   Input vector u[k].
 * @return {y[k], x[k+1]}.
 */
std::pair<Eigen::VectorXd, Eigen::VectorXd> ssStepCopy(
    const StateSpace &sys,
    const Eigen::VectorXd &x,
    const Eigen::VectorXd &u);

/**
 * @brief Discretisation method selector for c2d().
 */
enum class C2dMethod
{
    ZOH,             ///< Zero-order hold - exact for piecewise-constant inputs.
    Tustin,          ///< Bilinear (Tustin) transform: s = (2/Ts).(z-1)/(z+1).
    TustinPrewarped  ///< Bilinear transform prewarped at a specified frequency [rad/s].
};

/**
 * @brief Discretise a continuous-time state-space model.
 *
 * The input StateSpace must have Ts == 0.0 to indicate continuous time.
 *
 * **ZOH** (zero-order hold) - uses the Van Loan (1978) matrix-exponential embedding:
 * @code
 *   M = expm([Ac  Bc; 0  0] . Ts)  ->  Ad = M[:n,:n],  Bd = M[:n,n:]
 * @endcode
 *
 * **Tustin** - bilinear transform s = (2/Ts).(z-1)/(z+1):
 * @code
 *   Ad = (I - alpha.Ac)^-^1.(I + alpha.Ac),  Bd = Ts.(I - alpha.Ac)^-^1.Bc,  alpha = Ts/2
 * @endcode
 *
 * **TustinPrewarped** - bilinear transform with frequency prewarping at @p prewarp_freq [rad/s],
 * so that the discrete frequency response matches the continuous response exactly at that
 * frequency. Degenerates to standard Tustin when prewarp_freq = 0.
 *
 * Cd and Dd are unchanged for all methods.
 * A stability warning is printed to stderr if any eigenvalue of Ad lies outside the unit disk.
 *
 * Equivalent MATLAB: `c2d(sys_c, Ts, 'zoh')` / `c2d(sys_c, Ts, 'tustin')`
 *
 * @param sys_c       Continuous-time model (Ts must be 0.0).
 * @param Ts          Desired sample period [s].
 * @param method      Discretisation method (default: ZOH).
 * @param prewarp_freq Prewarping frequency [rad/s] (only used for TustinPrewarped).
 * @return Equivalent discrete-time StateSpace model.
 */
StateSpace c2d(const StateSpace &sys_c, double Ts,
               C2dMethod method = C2dMethod::ZOH,
               double prewarp_freq = 0.0);

/**
 * @brief Convert a SISO discrete state-space model to transfer function form.
 *
 * Computes the denominator from the characteristic polynomial of A (Faddeev-LeVerrier) and
 * the numerator from Markov parameters h[0] = D, h[k] = C.Aᵏ^-^1.B (k >= 1).
 *
 * The returned TransferFunction uses the z^-^1 polynomial form with a monic denominator
 * (den[0] = 1). Coefficient order: den = {1, a1, ..., an} in z^-^1 powers.
 *
 * Equivalent MATLAB: `tf(sys)` where sys is a discrete ss object.
 *
 * @param sys Source SISO discrete state-space model.
 * @return Equivalent TransferFunction in z^-^1 form.
 * @throws std::invalid_argument If the system is not SISO.
 */
TransferFunction ss2tf(const StateSpace &sys);

/**
 * @brief Compute the minimal realisation of a state-space model.
 *
 * Removes uncontrollable and unobservable states using balanced truncation (Ho-Kalman).
 * States with Hankel singular values below @p tol are discarded.
 *
 * @param sys Source state-space model.
 * @param tol Hankel singular value threshold (default 1e-6).
 * @return Minimal-order StateSpace model.
 */
StateSpace minreal(const StateSpace &sys, double tol = 1e-6);

} // namespace ctrl
