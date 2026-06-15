#pragma once
#include <Eigen/Dense>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

namespace ctrl {

/**
 * @brief Gaussian Process Regression (GPR) with squared-exponential kernel.
 *
 * Rasmussen & Williams, *Gaussian Processes for Machine Learning*, MIT Press 2006.
 *
 * Exact inference via Cholesky factorisation.  Supports a **fixed-budget** mode
 * that evicts the oldest training point when the budget N_max is reached, keeping
 * memory and computation bounded for online use.
 *
 * **Kernel:**
 * @code
 *   k(x, x') = sigma_f^2 . exp(-||x - x'||^2 / (2 . ℓ^2)) + sigma_n^2 . delta(x, x')
 * @endcode
 *
 * **Usage:**
 * @code
 *   ctrl::GaussianProcess::Params p;
 *   p.length_scale = 1.0; p.signal_var = 1.0; p.noise_var = 0.01;
 *   p.n_max = 100;   // fixed budget
 *   ctrl::GaussianProcess gp(1, p);   // 1-D input
 *
 *   gp.addPoint(x, y);   // add training data one at a time
 *   gp.fit();             // factorize covariance matrix
 *
 *   auto [mu, var] = gp.predict(x_test);
 * @endcode
 *
 * @note For use as an MPC plant model, wrap in a `StateFunc` that returns
 *       the GP mean prediction, and use the variance for constraint tightening.
 */
class GaussianProcess {
public:
    struct Params {
        double length_scale = 1.0;    ///< RBF length scale ℓ
        double signal_var   = 1.0;    ///< Signal variance sigma_f^2
        double noise_var    = 1e-4;   ///< Observation noise variance sigma_n^2
        int    n_max        = 200;    ///< Fixed budget (oldest points evicted; <=0 = unlimited)
    };

    struct Prediction {
        double mean;        ///< Posterior mean mu*(x)
        double variance;    ///< Posterior variance sigma^2*(x)  (>= 0)
    };

    /**
     * @brief Construct GP with given input dimension and parameters.
     * @param x_dim Input dimension (e.g., 1 for SISO plant).
     * @param p     Kernel and budget parameters (length-scale, noise variance, max points).
     */
    GaussianProcess(int x_dim, const Params& p);

    /**
     * @brief Add a training point (x, y) to the dataset.
     * If n_max > 0 and the budget is reached, the oldest point is removed.
     */
    void addPoint(const Eigen::VectorXd& x, double y);

    /**
     * @brief (Re-)compute the Cholesky factorisation of the training covariance.
     * Must be called after adding new data points and before calling predict().
     */
    void fit();

    /**
     * @brief Predict the posterior distribution at x_test.
     * @throws std::runtime_error if fit() has not been called yet.
     */
    Prediction predict(const Eigen::VectorXd& x_test) const;

    /** @brief Number of training points currently stored. */
    int  size()    const noexcept { return static_cast<int>(X_.size()); }
    bool isFitted() const noexcept { return fitted_; }

    void reset();

private:
    int    x_dim_;
    Params p_;
    bool   fitted_ = false;

    // Training data stored as deques for O(1) push_back and pop_front (fixed-budget eviction)
    std::deque<Eigen::VectorXd>  X_;   // inputs
    std::deque<double>           Y_;   // outputs

    // LDLT factorisation of training covariance K + sigma_n^2 I  (rebuilt in fit())
    Eigen::LDLT<Eigen::MatrixXd> K_ldlt_;
    Eigen::VectorXd              alpha_;   // K^{-1} y

    double kernel(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const;
};

} // namespace ctrl
