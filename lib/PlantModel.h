#pragma once
#include <vector>
#include <stdexcept>
#include <string>
#include <functional>
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

// =============================================================================
// DAESystem - Index-1 semi-explicit Differential-Algebraic Equation
//
//   x1' = f(x1, x2, u)    differential states (integrated by Euler/RK4)
//   0   = g(x1, x2, u)    algebraic constraints (solved by Newton at each step)
//   y   = h(x1, x2, u)    outputs
//
// Index-1 requirement: dg/dx2 must be invertible at the operating point.
// Index >= 2 (Pantelides / BLT algorithm) is out of scope.
// =============================================================================

/**
 * @brief Index-1 semi-explicit Differential-Algebraic Equation system.
 *
 * Represents plants of the form:
 * @code
 *   x1' = f(x1, x2, u)     (n_diff differential states)
 *   0   = g(x1, x2, u)     (n_alg  algebraic states)
 *   y   = h(x1, x2, u)     (output map)
 * @endcode
 *
 * Use @c dae2ode() to obtain a discrete-time step function, @c consistentInit()
 * to find consistent initial conditions, and the @c c2d() overload to obtain a
 * linearised discrete-time StateSpace for controller design (MPC, LQR, LQG).
 *
 * @note Only SISO (scalar u) is supported at the DAE level. MIMO extensions
 *   can be layered on top by the user.
 */
struct DAESystem
{
    /** @brief x1' = f(x1, x2, u) - differential equations. */
    using DiffFunc   = std::function<Eigen::VectorXd(const Eigen::VectorXd &x1,
                                                      const Eigen::VectorXd &x2,
                                                      double u)>;
    /** @brief 0 = g(x1, x2, u) - algebraic constraints (n_alg equations). */
    using AlgFunc    = std::function<Eigen::VectorXd(const Eigen::VectorXd &x1,
                                                      const Eigen::VectorXd &x2,
                                                      double u)>;
    /** @brief y = h(x1, x2, u) - output map. */
    using OutputFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &x1,
                                                      const Eigen::VectorXd &x2,
                                                      double u)>;

    DiffFunc   f;          ///< Differential equations.
    AlgFunc    g;          ///< Algebraic constraints.
    OutputFunc h;          ///< Output map.
    int        n_diff = 0; ///< Number of differential states (x1 dimension).
    int        n_alg  = 0; ///< Number of algebraic states  (x2 dimension).
    double     Ts     = 0.0; ///< Sample time [s] used by dae2ode() for Euler stepping.
};

/**
 * @brief Find consistent algebraic states: solve g(x1_init, x2, u0) = 0 for x2.
 *
 * Runs Newton-Raphson on the algebraic constraint starting from @p x2_guess.
 * The numerical Jacobian dg/dx2 is computed via central differences.
 * Returns the last iterate (not necessarily converged) when the Jacobian is
 * singular or max_iter is reached - callers should check residual if needed.
 *
 * @param dae      DAESystem whose AlgFunc g defines the constraint.
 * @param x1_init  Fixed differential states (not modified).
 * @param u0       Fixed input scalar.
 * @param x2_guess Initial guess for x2.
 * @param max_iter Maximum Newton iterations (default 20).
 * @param tol      Convergence tolerance on ||g|| (default 1e-9).
 * @return         x2 satisfying ||g(x1_init, x2, u0)|| < tol (or best iterate).
 */
Eigen::VectorXd consistentInit(const DAESystem     &dae,
                                const Eigen::VectorXd &x1_init,
                                double               u0,
                                const Eigen::VectorXd &x2_guess,
                                int                  max_iter = 20,
                                double               tol      = 1e-9);

/**
 * @brief Convert a DAESystem to a discrete-time step function.
 *
 * Returns @c F(x_aug, u) = x_aug_next where x_aug = [x1; x2].
 * At each call the function:
 * 1. Projects x2 onto the constraint manifold (Newton solve on g).
 * 2. Integrates x1 forward by one Euler step using dae.Ts.
 * 3. Solves for x2_next from g(x1_next, x2_next, u) = 0.
 *
 * The resulting step function is directly usable in simulation loops and as the
 * state prediction function for @c NonlinearMPC / @c CEMController.
 *
 * @param dae            DAESystem (must have Ts > 0 for meaningful stepping).
 * @param newton_max_iter Newton iteration limit (default 20).
 * @param newton_tol     Newton convergence tolerance (default 1e-9).
 * @return               Step function @c x_aug_next = F(x_aug, u).
 */
std::function<Eigen::VectorXd(const Eigen::VectorXd &, double)>
dae2ode(const DAESystem &dae,
        int    newton_max_iter = 20,
        double newton_tol      = 1e-9);

/**
 * @brief Linearise a DAESystem at (x1_op, x2_op, u_op), eliminate x2, and discretise.
 *
 * Implements index-1 algebraic elimination:
 * @code
 *   A_red = A11 - A12 * G2^{-1} * G1
 *   B_red = B1  - A12 * G2^{-1} * B2
 * @endcode
 * where A11=df/dx1, A12=df/dx2, G1=dg/dx1, G2=dg/dx2, B1=df/du, B2=dg/du.
 *
 * Then applies the existing @c c2d(StateSpace, Ts, method) to the reduced
 * continuous-time model (A_red, B_red, C_red, D_red).
 *
 * @throws std::runtime_error if G2 is singular ("DAE index > 1 at operating point").
 *
 * @param dae      Source DAESystem.
 * @param x1_op    Operating-point differential states.
 * @param x2_op    Operating-point algebraic states (must satisfy g approx = 0).
 * @param u_op     Operating-point input.
 * @param Ts       Desired sample period [s].
 * @param method   Discretisation method (default: ZOH).
 * @return         Discrete-time StateSpace of dimension n_diff.
 */
StateSpace c2d(const DAESystem     &dae,
               const Eigen::VectorXd &x1_op,
               const Eigen::VectorXd &x2_op,
               double               u_op,
               double               Ts,
               C2dMethod            method = C2dMethod::ZOH);

} // namespace ctrl

// Self-registration - included by ControllerToolbox.h umbrella.
#include "ControllerRegistry.h"
CTRL_REGISTER_FEATURE(dae_system)
