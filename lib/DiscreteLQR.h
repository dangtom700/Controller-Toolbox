#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>
#include <functional>
#include <stdexcept>

// Discrete-time Linear Quadratic Regulator (LQR).
//
// Offline: solves the Discrete Algebraic Riccati Equation (DARE) via value iteration.
//   Pinf = A'PinfA - (A'PinfB)(R + B'PinfB)^-^1(B'PinfA) + Q
//   K*  = (R + B'PinfB)^-^1 B'PinfA
//
// Online: u[k] = -K*.(x[k] - x_ref) + u_ff
//
// For output feedback, first reconstruct the state with an observer (e.g., Kalman filter)
// before calling compute().
//
// Ref: Anderson & Moore "Optimal Control" (1990);
//      MATLAB dlqr(), Simulink Optimal LQR Controller block.
namespace ctrl
{

    // Result returned by the DARE solver - carries the solution, convergence flag, and
    // iteration count so callers can decide whether to accept an approximate result.
    struct DareResult
    {
        Eigen::MatrixXd P;          // Best available Riccati solution (converged or last iterate)
        bool            converged;  // true when value iteration reached the tolerance
        int             iterations; // actual number of iterations performed
    };

    // Tuning weights.
    struct LQRParams
    {
        Eigen::MatrixXd Q; // State cost  (n*n, positive semi-definite)
                           //   -> increase Q_ii to tighten tracking of state i
        Eigen::MatrixXd R; // Control cost (m*m, positive definite)
                           //   -> increase R_jj to penalise actuator j
    };

    class DiscreteLQR
    {
    public:
        // Construct and solve DARE for the given plant model and weighting matrices.
        // Warns to stderr if DARE does not converge; uses the best available iterate.
        //
        // PBH rank test note: stabilisability and detectability are checked at construction
        // using Eigen's fullPivLu().rank(), which applies an automatic rank threshold scaled
        // by max_singular_value * n * epsilon. For plants with eigenvalues very close to the
        // unit circle (e.g., poles at 0.9999 or 1.0001), this threshold may misdeclassify the
        // system as non-stabilisable. If you suspect a false PBH failure, tighten the plant
        // model or manually verify the PBH condition: rank([A - lambda*I, B]) == n for all
        // |lambda| >= 1.
        DiscreteLQR(const StateSpace &plant, const LQRParams &params);

        // Compute u[k] = -K*(x - x_ref) + u_ff.
        // x_ref and u_ff default to zero when empty.
        //
        // Note: DiscreteLQR is stateless at runtime (no internal memory between compute() calls).
        // If the returned u[k] is clamped by an external actuator before it reaches the plant,
        // the state x[k+1] will evolve under the saturated input. For closed-loop correctness,
        // always pass the ACTUAL plant state x[k] (from a sensor or state observer) to compute()
        // rather than integrating the plant model with the unsaturated u[k]. This is the standard
        // closed-loop correction mechanism: the observer implicitly corrects for saturation via
        // the measurement update. For explicit anti-windup in state-feedback loops, see the
        // discussion in Astrom & Wittenmark §9.3.
        Eigen::VectorXd compute(const Eigen::VectorXd &x,
                                const Eigen::VectorXd &x_ref = Eigen::VectorXd(),
                                const Eigen::VectorXd &u_ff = Eigen::VectorXd()) const;

        const Eigen::MatrixXd &gainMatrix() const { return K_; }
        const Eigen::MatrixXd &riccatiSolution() const { return P_; }
        bool   dareConverged() const { return dare_converged_; }
        int    dareIterations() const { return dare_iterations_; }
        double sampleTime() const { return Ts_; }

    private:
        Eigen::MatrixXd K_; // optimal feedback gain (m*n)
        Eigen::MatrixXd P_; // DARE stabilising solution (n*n)
        double Ts_;
        int n_, m_;
        bool dare_converged_;
        int  dare_iterations_;

        // Value-iteration DARE solver - returns DareResult (never throws).
        static DareResult solveDARE(const Eigen::MatrixXd &A,
                                    const Eigen::MatrixXd &B,
                                    const Eigen::MatrixXd &Q,
                                    const Eigen::MatrixXd &R);
    };

    // ---------------------------------------------------------------------------
    // LQRAdapter - wraps DiscreteLQR as an IController for use in ControllerStack.
    // Callers supply state and reference via std::function callbacks.
    // For SISO plants the scalar IController::compute() interface is used; the
    // adapter extracts the first element of the LQR control vector.
    // ---------------------------------------------------------------------------
    class LQRAdapter : public IController
    {
    public:
        // stateProvider()  -> current state vector x[k]  (n*1)
        // refProvider()    -> reference state x_ref[k]   (n*1), may return empty for zero ref
        LQRAdapter(DiscreteLQR &lqr,
                   std::function<Eigen::VectorXd()> stateProvider,
                   std::function<Eigen::VectorXd()> refProvider = {})
            : lqr_(lqr), stateFn_(std::move(stateProvider)), refFn_(std::move(refProvider))
        {
        }

        // signal is ignored; state and reference come from the registered callbacks.
        double compute(double /*signal*/) override
        {
            Eigen::VectorXd x_ref;
            if (refFn_)
                x_ref = refFn_();
            return lqr_.compute(stateFn_(), x_ref)(0);
        }

        void reset() override {} // LQR is stateless at runtime
        double sampleTime() const override { return lqr_.sampleTime(); }

        // IController health interface - reflects DARE convergence from construction.
        // If DARE did not converge the gain matrix K_ is an approximate iterate; the
        // controller still runs but may not be stabilising.  ControllerStack will skip
        // this entry and fall back to the next eligible entry.
        bool isHealthy() const override { return lqr_.dareConverged(); }

    private:
        DiscreteLQR &lqr_;
        std::function<Eigen::VectorXd()> stateFn_;
        std::function<Eigen::VectorXd()> refFn_;
    };

} // namespace ctrl
