#pragma once
#include "PlantModel.h"          // StateSpace, c2d
#include "LinearisationHelper.h" // StateFunc
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ctrl {

/**
 * @brief SINDy - Sparse Identification of Nonlinear Dynamics.
 *
 * Brunton, Proctor & Kutz (2016).  Builds a library Theta(x) of candidate
 * right-hand-side terms (constant, linear, quadratic monomials, and optional
 * trig functions), then solves a sparse regression problem:
 *
 * @code
 *   Ẋ approx = Theta(X) . Ξ
 * @endcode
 *
 * where Ξ is the sparse coefficient matrix found by sequentially-thresholded
 * least squares (STLS, equivalent to a greedy LASSO) or ordinary least squares.
 *
 * **Result:** a `SINDyModel` object whose `predict(x_dev, u_dev)` method returns
 * the state derivative, compatible with `NonlinearMPC`'s `StateFunc` and with
 * `ExtendedKalmanFilter`'s process function.
 *
 * **Usage:**
 * @code
 *   ctrl::SINDy::Params p;
 *   p.n_state = 2; p.n_input = 1;
 *   p.poly_degree = 2;        // constant + linear + quadratic
 *   p.threshold = 0.01;       // sparsity threshold
 *   ctrl::SINDy sindy(p);
 *
 *   // Collect snapshot data from the real plant
 *   sindy.addSnapshot(x_dev, u_dev, xdot_dev);   // or use addSnapshotFD for finite-difference
 *
 *   // Fit sparse model
 *   ctrl::SINDyModel model = sindy.fit();
 *
 *   // Use as StateFunc in NonlinearMPC / EKF
 *   ctrl::NonlinearMPC nmpc(nmpc_params, model.stateFunc(), C);
 * @endcode
 *
 * @see Brunton, S.L., Proctor, J.L. & Kutz, J.N. (2016).
 *      Discovering governing equations from data by sparse identification of nonlinear
 *      dynamical systems. PNAS 113(15), 3932-3937.
 */

// ---------------------------------------------------------------------------
// SINDy library term types
// ---------------------------------------------------------------------------

/** @brief Types of basis functions included in the SINDy library. */
enum class SINDyLibrary {
    PolyDeg1  = 1,   ///< 1 + x + u
    PolyDeg2  = 2,   ///< PolyDeg1 + x^2 + x*u + u^2
    PolyDeg3  = 3,   ///< PolyDeg2 + cubic terms
    PolyTrig  = 10,  ///< PolyDeg2 + sin(x_i) + cos(x_i) for each state
};

// Forward declaration so SINDyModel can hold a cached SINDy helper
class SINDy;

// ---------------------------------------------------------------------------
// SINDyModel - the fitted sparse model
// ---------------------------------------------------------------------------

/**
 * @brief Fitted SINDy model: xdot = Theta(x,u) . Ξ.
 *
 * Returned by SINDy::fit().  Acts as an opaque black-box; use stateFunc()
 * to get a `StateFunc` lambda for use with NonlinearMPC / EKF / UKF.
 */
class SINDyModel {
public:
    SINDyModel() = default;

    // helper_cache_ is a lazy-init perf cache, not model state - copy only real data
    SINDyModel(const SINDyModel& o)
        : n_state_(o.n_state_), n_input_(o.n_input_)
        , lib_type_(o.lib_type_), xi_(o.xi_) {}

    SINDyModel& operator=(const SINDyModel& o) {
        if (this != &o) {
            n_state_ = o.n_state_; n_input_ = o.n_input_;
            lib_type_ = o.lib_type_; xi_ = o.xi_;
            helper_cache_.reset();
        }
        return *this;
    }

    SINDyModel(SINDyModel&&)            = default;
    SINDyModel& operator=(SINDyModel&&) = default;

    /**
     * @brief Predict the state derivative at (x_dev, u_dev).
     * @param x_dev  State deviation vector (length n_state).
     * @param u_dev  Input deviation vector (length n_input).
     * @return xdot  (length n_state).
     */
    Eigen::VectorXd predict(const Eigen::VectorXd& x_dev,
                             const Eigen::VectorXd& u_dev) const;

    /** @brief Return a StateFunc lambda wrapping predict(). */
    StateFunc stateFunc() const;

    /** @brief Number of states. */
    int nState() const noexcept { return n_state_; }
    /** @brief Number of inputs. */
    int nInput() const noexcept { return n_input_; }
    /** @brief Number of library terms. */
    int nTerms() const noexcept { return static_cast<int>(xi_.rows()); }
    /** @brief Non-zero fraction of the coefficient matrix (sparsity metric). */
    double sparsity() const;

    /** @brief Coefficient matrix Ξ, size (n_terms * n_state). */
    const Eigen::MatrixXd& coefficients() const noexcept { return xi_; }

private:
    friend class SINDy;

    int             n_state_ = 0;
    int             n_input_ = 0;
    SINDyLibrary    lib_type_ = SINDyLibrary::PolyDeg2;
    Eigen::MatrixXd xi_;   ///< n_terms * n_state coefficient matrix

    // Cached library helper (avoids constructing SINDy each predict() call)
    mutable std::unique_ptr<SINDy> helper_cache_;
    mutable Eigen::VectorXd        row_work_;    ///< workspace for libraryRow output
};

// ---------------------------------------------------------------------------
// SINDy - data collector and fitter
// ---------------------------------------------------------------------------

class SINDy {
public:
    struct Params {
        int          n_state     = 1;                   ///< State dimension
        int          n_input     = 1;                   ///< Input dimension
        SINDyLibrary library     = SINDyLibrary::PolyDeg2; ///< Basis function set
        double       threshold   = 0.01;                ///< STLS sparsity threshold
        int          stls_iter   = 10;                  ///< STLS iterations
        bool         use_ols     = false;               ///< If true, use plain OLS (no sparsification)
    };

    explicit SINDy(const Params& p);

    /**
     * @brief Add a single snapshot: (state, input) -> state derivative.
     * @param x_dev    State deviation at time k.
     * @param u_dev    Input deviation at time k.
     * @param xdot_dev State derivative (measured or finite-differenced).
     */
    void addSnapshot(const Eigen::VectorXd& x_dev,
                     const Eigen::VectorXd& u_dev,
                     const Eigen::VectorXd& xdot_dev);

    /**
     * @brief Add a snapshot using forward finite difference of consecutive states.
     * @param x_k   State deviation at time k.
     * @param x_k1  State deviation at time k+1.
     * @param u_k   Input deviation at time k.
     * @param Ts    Sample time.
     */
    void addSnapshotFD(const Eigen::VectorXd& x_k,
                       const Eigen::VectorXd& x_k1,
                       const Eigen::VectorXd& u_k,
                       double Ts);

    /**
     * @brief Fit the sparse model to all accumulated snapshots.
     * @return Fitted SINDyModel.
     * @throws std::runtime_error if no snapshots have been added.
     */
    SINDyModel fit() const;

    /** @brief Number of snapshots accumulated so far. */
    int snapshotCount() const noexcept;

    /** @brief Pre-reserve snapshot storage for N time steps. */
    void reserveSnapshots(int N);

    /** @brief Build the library row Theta(x, u) for a single snapshot. */
    Eigen::VectorXd libraryRow(const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u) const;

    /** @brief Number of library terms for the chosen library type. */
    int nTerms() const noexcept { return n_terms_; }

private:
    Params p_;
    int n_terms_;

    // Accumulated data matrices (grown dynamically)
    std::vector<Eigen::VectorXd> theta_rows_;  // library rows, each length n_terms
    std::vector<Eigen::VectorXd> xdot_rows_;   // target derivatives, each length n_state

    int computeNTerms() const;

    // Sequentially-thresholded least squares (STLS / sparse regression)
    Eigen::MatrixXd stls(const Eigen::MatrixXd& Theta,
                          const Eigen::MatrixXd& Xdot) const;

    // Workspace for stls() inner loop - avoids per-iteration heap allocation
    mutable std::vector<int>     stls_active_;
    mutable Eigen::MatrixXd      stls_Theta_a_;
};

} // namespace ctrl
