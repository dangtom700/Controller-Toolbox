#pragma once
#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>

namespace ctrl {

/**
 * @brief Iterative Learning Controller (ILC).
 *
 * Learns a feedforward correction that eliminates repeating errors across trials
 * of a fixed-duration task.  Two update laws are provided:
 *
 * **P-type (simple, no model required):**
 * @code
 *   u_{j+1}[k] = u_j[k] + L_p * e_j[k]
 * @endcode
 *
 * **D-type (derivative of error):**
 * @code
 *   u_{j+1}[k] = u_j[k] + L_p * e_j[k] + L_d * (e_j[k] - e_j[k-1]) / Ts
 * @endcode
 *
 * Both are filtered through a Q-filter (zero-phase FIR low-pass) to suppress
 * trial-to-trial noise amplification.
 *
 * **Norm-optimal ILC** (optional, requires Markov-parameter matrix G):
 * @code
 *   u_{j+1} = Q_e * u_j + L_opt * e_j
 *   L_opt = (G' R G + Q_u)^{-1} G' R
 *   Q_e   = I - L_opt * G
 * @endcode
 *
 * **Usage:**
 * @code
 *   ctrl::ILC::Params p;
 *   p.N = 200;  p.mode = ctrl::ILC::Mode::PType;
 *   p.Lp = 0.5;  p.Ts = 0.01;
 *   ctrl::ILC ilc(p);
 *
 *   // Each trial:
 *   for (int k = 0; k < p.N; ++k) {
 *       double u = ilc.feedforward(k) + feedback_pid.compute(e[k]);
 *       applyInput(u);
 *       ilc.recordError(k, r[k] - y[k]);
 *   }
 *   ilc.updateFeedforward();   // compute u_{j+1} from stored e_j
 * @endcode
 *
 * @see Bristow, D.A., Tharayil, M. & Alleyne, A.G. (2006).
 *      A survey of iterative learning control. IEEE CSM 26(3), 96-114.
 */
class ILC {
public:
    enum class Mode {
        PType,       ///< u_{j+1}[k] = u_j[k] + Lp * e_j[k]  (no model needed)
        DType,       ///< PType + derivative term  Ld * de/dt
        NormOptimal  ///< Minimum-norm update requiring Markov matrix G
    };

    struct Params {
        int    N        = 100;    ///< Trial length (number of time steps per trial)
        double Ts       = 0.01;   ///< Sample time [s]
        Mode   mode     = Mode::PType;

        // P/D-type gains
        double Lp       = 0.5;    ///< Proportional ILC gain in (0, 1)
        double Ld       = 0.0;    ///< Derivative ILC gain (0 = P-type only)

        // Q-filter: scalar forgetting factor applied to old feedforward each trial.
        // Q_filter in (0, 1): smaller = more stable but slower learning.
        double Q_filter = 0.95;

        // Norm-optimal parameters (used only when mode == NormOptimal)
        double rho_u    = 1.0;    ///< Input effort weight in the norm-optimal update
        double rho_e    = 1.0;    ///< Tracking error weight

        // Output saturation (applied after update)
        double uMin     = -1e9;
        double uMax     =  1e9;
    };

    /**
     * @brief Construct P-type or D-type ILC with given parameters.
     * @throws std::invalid_argument if N <= 0.
     */
    explicit ILC(const Params& p);

    /**
     * @brief Construct norm-optimal ILC from plant Markov matrix.
     * @param p       Parameters (mode should be NormOptimal; Ts/N must be consistent).
     * @param G       Np * Np lower-triangular Markov (step-response convolution) matrix.
     *                G[i,j] = plant impulse response at step i from input at step j.
     *                Typically G = Toeplitz of the step response differences.
     * @throws std::invalid_argument if G is not square or its size != N.
     */
    ILC(const Params& p, const Eigen::MatrixXd& G);

    // ---- Trial interface ---------------------------------------------------

    /**
     * @brief Return the stored feedforward input for time step k within a trial.
     * @param k  Step index in [0, N-1].
     */
    double feedforward(int k) const;

    /**
     * @brief Record the tracking error at step k of the current trial.
     * @param k      Step index.
     * @param error  Tracking error e[k] = r[k] - y[k].
     */
    void recordError(int k, double error);

    /**
     * @brief Compute and store the updated feedforward for the next trial.
     *
     * Must be called after every step of the trial (k = 0 ... N-1) has been
     * recorded via recordError().  The new feedforward is immediately available
     * via feedforward(k) for the next trial.
     */
    void updateFeedforward();

    // ---- Accessors --------------------------------------------------------

    /** @brief Current trial index (incremented by each updateFeedforward call). */
    int trialIndex()       const noexcept { return trial_; }
    /** @brief Trial length N. */
    int trialLength()      const noexcept { return p_.N; }
    /** @brief RMS tracking error of the last completed trial. */
    double lastRMSError()  const noexcept { return rms_last_; }

    /**
     * @brief Reset feedforward to zero and restart from trial 0.
     * Useful when the reference trajectory changes.
     */
    void reset();

    std::string name() const { return "ILC"; }

private:
    Params p_;

    Eigen::VectorXd u_ff_;    ///< Feedforward sequence for the current trial (length N)
    Eigen::VectorXd e_curr_;  ///< Error record for the current trial
    Eigen::VectorXd e_prev_;  ///< Error from the previous trial (for D-type)

    // Norm-optimal precomputed matrices
    bool            has_G_   = false;
    Eigen::MatrixXd L_opt_;   ///< Optimal learning matrix (N x N)
    Eigen::MatrixXd Q_e_;     ///< Forgetting matrix (N x N)

    int    trial_    = 0;
    double rms_last_ = 0.0;

    void buildNormOptimal(const Eigen::MatrixXd& G);
};

} // namespace ctrl
