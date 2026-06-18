#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ctrl {

/**
 * @brief Koopman / Extended Dynamic Mode Decomposition (EDMD).
 *
 * Williams, Kevrekidis & Rowley (2015) / Proctor et al. (2016, DMDc).
 * Lifts nonlinear I/O data into a high-dimensional linear space via a
 * dictionary of observable functions ψ(x), then estimates a linear operator K:
 *
 * @code
 *   z[k] = ψ(x[k], u[k])     (lifting map)
 *   z[k+1] approx = K . z[k]        (linear dynamics in lifted space)
 * @endcode
 *
 * The result is projected back onto the original state coordinates to give a
 * `ctrl::StateSpace` that can be used directly with `DiscreteMPC`, `DiscreteLQR`,
 * `DiscreteLQG`, and `DiscreteHinf`.
 *
 * **Dictionary options:**
 *  - `PolyDeg1` : [1; x; u]  (recovers a linear model, equivalent to DMDc)
 *  - `PolyDeg2` : [1; x; u; x^2; xu; u^2]  (quadratic lifting)
 *  - `RBF`      : Gaussian radial-basis functions centred on training data
 *
 * **Usage:**
 * @code
 *   ctrl::KoopmanEDMD::Params p;
 *   p.n_state = 2; p.n_input = 1;
 *   p.dict = ctrl::KoopmanEDMD::Dict::PolyDeg2;
 *   ctrl::KoopmanEDMD edmd(p);
 *
 *   for (int k = 0; k < N; ++k)
 *       edmd.addSnapshot(x_k, u_k, x_k1);   // x[k+1] known (simulation or measurement)
 *
 *   ctrl::StateSpace ss_lifted = edmd.fit();   // returns a ctrl::StateSpace in the lifted space
 *   ctrl::StateSpace ss_proj   = edmd.fitProjected();  // projected back to original n_state
 *
 *   ctrl::DiscreteMPC mpc(ss_proj, mpc_params);
 * @endcode
 *
 * @see Williams, M.O., Kevrekidis, I.G. & Rowley, C.W. (2015). A data-driven
 *      approximation of the Koopman operator. JNLS 25, 1307-1346.
 * @see Proctor, J.L., Brunton, S.L. & Kutz, J.N. (2016). Dynamic mode
 *      decomposition with control. SIAM 6(1), 388-413.
 */

enum class KoopmanDict {
    PolyDeg1 = 1,  ///< [1; x; u]  - linear (DMDc)
    PolyDeg2 = 2,  ///< [1; x; u; x^2; xu; u^2]
    RBF      = 10, ///< Gaussian RBF centred on random centres from the training set
};

class KoopmanEDMD {
public:
    using Dict = KoopmanDict;

    struct Params {
        int    n_state    = 1;            ///< State dimension
        int    n_input    = 1;            ///< Input dimension
        Dict   dict       = Dict::PolyDeg2;  ///< Dictionary type
        int    rbf_n_cent = 20;           ///< RBF centres (used only for Dict::RBF)
        double rbf_width  = 1.0;         ///< RBF Gaussian width sigma
        double tikhonov   = 1e-6;        ///< Regularisation on the EDMD least-squares
    };

    explicit KoopmanEDMD(const Params& p);

    /**
     * @brief Add a snapshot: (state at k, input at k) -> state at k+1.
     */
    void addSnapshot(const Eigen::VectorXd& x_k,
                     const Eigen::VectorXd& u_k,
                     const Eigen::VectorXd& x_k1);

    /**
     * @brief Fit the Koopman operator from accumulated snapshots.
     * @return StateSpace in the full lifted space (nz states, n_input inputs,
     *         C maps lifted state back to original n_state).
     * @throws std::runtime_error if no snapshots added or EDMD solve fails.
     */
    StateSpace fit();

    /**
     * @brief Fit and project back to original n_state coordinates.
     * The resulting `ctrl::StateSpace` has n_state states, matching the
     * original plant dimension - ready for LQR/MPC/LQG.
     */
    StateSpace fitProjected();

    /** @brief Number of lifted dimensions for the current dictionary. */
    int nLifted() const noexcept { return n_lifted_; }
    /** @brief Number of snapshots accumulated. */
    int snapshotCount() const noexcept;

    /** @brief Pre-reserve snapshot storage for N time steps. */
    void reserveSnapshots(int N);

    /**
     * @brief Lift a state-input pair to the full observable vector ψ(x, u).
     */
    Eigen::VectorXd lift(const Eigen::VectorXd& x,
                          const Eigen::VectorXd& u) const;

private:
    Params p_;
    int n_lifted_;   ///< Dimension of the lifted state space

    // Accumulated snapshot matrices
    std::vector<Eigen::VectorXd> psi_k_;   // lifted ψ(x[k], u[k])
    std::vector<Eigen::VectorXd> x_k1_;    // x[k+1] (for projection)
    std::vector<Eigen::VectorXd> psi_k1_;  // lifted ψ(x[k+1], 0) (Koopman target)

    // RBF centres (n_state * n_centres), set from first batch of snapshots
    Eigen::MatrixXd rbf_centres_;
    bool centres_set_ = false;

    // Workspace vectors for liftPoly (avoids per-call allocation)
    mutable Eigen::VectorXd lift_z_;
    mutable Eigen::VectorXd lift_psi_;

    void maybeSetCentres(const Eigen::VectorXd& x);
    int computeNLifted() const;

    // Poly ψ(x, u) - [1; x; u; degree-2 cross-terms]
    Eigen::VectorXd liftPoly(const Eigen::VectorXd& x,
                              const Eigen::VectorXd& u) const;
    // RBF ψ(x) - Gaussian basis
    Eigen::VectorXd liftRBF(const Eigen::VectorXd& x,
                              const Eigen::VectorXd& u) const;
};

} // namespace ctrl
