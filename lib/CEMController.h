#pragma once
#include "IController.h"
#include "PlantModel.h"          // StateSpace
#include "LinearisationHelper.h" // StateFunc
#include <Eigen/Dense>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>

namespace ctrl {

/**
 * @brief Cross-Entropy Method (CEM) Model Predictive Controller.
 *
 * Derivative-free stochastic MPC: samples N candidate action sequences from
 * a Gaussian distribution, simulates each under a plant model, keeps the top
 * elite_frac * N samples (elite set), and refits the Gaussian.  Returns the
 * first action of the best sequence.
 *
 * **Algorithm (one control step):**
 * @code
 *   mu = mu_prev,  sigma = sigma_init                    // warm-start with previous solution
 *   for iter in 1..n_iter:
 *       Sample N action sequences of length Np from N(mu, sigma^2)
 *       Clip to [u_min, u_max]
 *       Roll out each sequence under f(x, u) for Np steps
 *       Evaluate cost J = Sigma ||y - y_ref||_Q^2 + ||u||_R^2
 *       Select top-K elite sequences (K = elite_frac * N)
 *       mu, sigma = mean, std of elite set
 *   return mu[0]   // first action of best sequence
 * @endcode
 *
 * This is equivalent to a population-based MPPI approximation and works for
 * non-convex cost functions where FISTA-based MPC (NonlinearMPC) may find poor
 * local optima.
 *
 * **Usage:**
 * @code
 *   ctrl::CEMController::Params p;
 *   p.Np = 20; p.N_samples = 100; p.n_iter = 5;
 *   p.Q = 1.0; p.R = 0.1; p.uMin = -1.0; p.uMax = 1.0;
 *   ctrl::CEMController cem(p, model_func, C_matrix, Ts);
 *
 *   cem.setState(x_current);
 *   cem.setReference(r_ref);
 *   double u = cem.compute(r - y);   // or computeRef(x, r)
 * @endcode
 *
 * @see Rubinstein, R.Y. & Kroese, D.P. (2004). The Cross-Entropy Method.
 *      Springer.  Also used in DeepMind's MPPI and CEM-based visual planning.
 */
class CEMController : public IController {
public:
    struct Params {
        int    Np         = 20;    ///< Prediction horizon
        int    N_samples  = 100;   ///< Number of candidate sequences per iteration
        int    n_iter     = 5;     ///< CEM refinement iterations
        double elite_frac = 0.1;   ///< Fraction of samples kept as elite set
        double sigma_init = 0.5;   ///< Initial standard deviation of the sampling distribution
        double sigma_min  = 0.01;  ///< Minimum std (prevents collapse)
        double Q          = 1.0;   ///< Output tracking weight (SISO: scalar Q)
        double R          = 0.1;   ///< Input effort weight
        double uMin       = -1e9;  ///< Input lower bound
        double uMax       =  1e9;  ///< Input upper bound
        unsigned int seed = 42;    ///< Random seed
    };

    /**
     * @brief Construct CEM-MPC with a discrete-time plant model.
     * @param p      Design parameters.
     * @param f      Discrete-time state transition: x_{k+1} = f(x_k, u_k).
     * @param C      Output matrix: y_k = C . x_k.
     * @param Ts     Sample time [s].
     */
    CEMController(const Params&          p,
                  StateFunc              f,
                  const Eigen::MatrixXd& C,
                  double                 Ts);

    /**
     * @brief Set the current plant state (called before compute()).
     */
    void setState(const Eigen::VectorXd& x) { x_ = x; }

    /**
     * @brief Set the reference trajectory (constant over the horizon).
     */
    void setReference(const Eigen::VectorXd& r_ref) { r_ = r_ref; }

    /**
     * @brief Compute optimal first action via CEM.
     * @param x_cur   Current state.
     * @param y_ref   Reference output (length = C.rows()).
     * @return Optimal first action u_0.
     */
    Eigen::VectorXd computeRef(const Eigen::VectorXd& x_cur,
                                 const Eigen::VectorXd& y_ref);

    // IController interface (SISO convenience, for 1-input 1-output systems)
    double compute(double error) override;
    void   reset()               override;
    double sampleTime() const    override { return Ts_; }
    std::string name()  const    override { return "CEM-MPC"; }

    /** @brief Last achieved cost (best elite sample cost). */
    double lastCost() const noexcept { return last_cost_; }

private:
    Params p_;
    StateFunc       f_;
    Eigen::MatrixXd C_;
    double          Ts_;

    Eigen::VectorXd x_;          ///< Current state
    Eigen::VectorXd r_;          ///< Reference (p-dimensional)
    Eigen::VectorXd mu_;         ///< Warm-started mean action sequence (Np)
    double          last_cost_ = 0.0;

    std::mt19937 rng_;

    // Pre-allocated workspaces for computeRef() - avoids per-call heap allocation
    std::vector<Eigen::VectorXd> samples_;  ///< N_samples x Np candidate sequences
    std::vector<double>          costs_;    ///< N_samples rollout costs
    Eigen::VectorXd              new_mu_;   ///< Elite mean update workspace (Np)
    Eigen::VectorXd              u_k_;      ///< Single-step control vector (size 1)

    // Mutable rollout workspaces for rolloutCost() - avoids per-rollout heap alloc
    mutable Eigen::VectorXd x_roll_;    ///< State trajectory workspace (n)
    mutable Eigen::VectorXd e_roll_;    ///< Output error workspace (p)
    mutable Eigen::VectorXd u_k_roll_;  ///< Single-step control workspace for rollout (size 1)

    // Cost of a single action sequence under f, starting from x0
    double rolloutCost(const Eigen::VectorXd& x0,
                       const Eigen::VectorXd& u_seq,
                       const Eigen::VectorXd& y_ref) const;
};

} // namespace ctrl
