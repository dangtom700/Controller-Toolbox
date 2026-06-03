#pragma once
#include "IController.h"
#include <Eigen/Dense>
#include <stdexcept>

namespace ctrl {

/**
 * @brief Data-Enabled Predictive Controller (DeePC).
 *
 * Implements the behavioural systems approach of Coulson, Lygeros & Dorfler (2019).
 * Rather than identifying a parametric model, a pair of block-Hankel matrices built
 * from a **persistently-exciting** offline I/O dataset implicitly represents every
 * reachable trajectory of the unknown LTI system (Willems' fundamental lemma).
 *
 * **QP solved at each step (SISO):**
 * @code
 *   min_{g, u}   (rho_y/2)||H_yf.g - y_ref||^2 + (rho_u/2)||u||^2
 *              + (lambda_g/2)||g||^2 + (lambda_eq/2)||H_up.g - u_ini||^2
 *   s.t.         H_uf.g = u,    u_min <= u <= u_max
 * @endcode
 *
 * Solved via ADMM with three closed-form updates per iteration:
 *   - g-update  : LDLT solve with a **constant** Hessian (factorised once at construction)
 *   - u-update  : element-wise box projection of the unconstrained minimiser
 *   - lambda-update  : dual ascent on the coupling constraint H_uf.g = u
 *
 * **Data requirements (Willems PE condition):**
 * Offline input must be persistently exciting of order >= T_ini + Np + n_plant.
 * A PRBS of length N >= 2.(T_ini + Np) + n_plant is typically sufficient for SISO.
 *
 * **Usage:**
 * @code
 *   // 1. Collect offline data (PRBS on the real or simulated plant)
 *   Eigen::VectorXd u_off(300), y_off(300);
 *   // ... fill u_off, y_off ...
 *
 *   // 2. Construct controller
 *   ctrl::DeePC::Params p;
 *   p.T_ini = 5; p.Np = 20; p.lambda_g = 1.0; p.lambda_eq = 1e5;
 *   p.rho_y = 1.0; p.rho_u = 0.1; p.uMin = -1.0; p.uMax = 1.0;
 *   ctrl::DeePC deepc(u_off, y_off, p, Ts);
 *
 *   // 3. Closed-loop (absolute I/O)
 *   double u = deepc.computeIO(y_measured, r_setpoint);
 *   plant_step(u);
 *
 *   // -- or via IController::compute(error) --
 *   deepc.setReference(r);
 *   double u = deepc.compute(r - y);
 * @endcode
 *
 * @note Warm-up: returns 0 for the first T_ini steps while the I/O buffer fills.
 *
 * @see Coulson, J., Lygeros, J. & Dorfler, F. (2019). Data-Enabled Predictive Control.
 *      European Control Conference.
 */
class DeePC : public IController {
public:
    /**
     * @brief DeePC design parameters.
     */
    struct Params {
        int    T_ini     = 5;     ///< Initialisation (past) window length.  Should be >= plant order.
        int    Np        = 20;    ///< Prediction horizon (future window).
        double rho_y     = 1.0;   ///< Output tracking weight (Q = rho_y . I_{Np}).
        double rho_u     = 0.1;   ///< Input effort weight   (R = rho_u . I_{Np}).
        double lambda_g  = 1.0;   ///< Tikhonov regularisation on g (prevents over-fitting to noisy data).
        double lambda_eq = 1e5;   ///< Penalty for violating the past-input consistency H_up.g = u_ini.
                                  ///<  WARNING: if lambda_eq >> rho_y and the I/O buffers are all-zero
                                  ///<  (cold start), the tracking objective is numerically negligible and
                                  ///<  DeePC outputs u≈0.  Use lambda_eq ≤ 10×rho_y for cold-start scenarios.
        double uMin      = -1e9;  ///< Input lower bound.
        double uMax      =  1e9;  ///< Input upper bound.
        double rho_admm  = 1.0;   ///< ADMM coupling penalty rho.  Larger -> faster consensus, more oscillation.
        int    admm_iter = 100;   ///< ADMM iterations per control step.
    };

    /**
     * @brief Construct DeePC and pre-factor the ADMM Hessian.
     *
     * @param u_data  Offline input sequence (length N).  Must be persistently exciting.
     * @param y_data  Corresponding output sequence (length N).
     * @param params  Design parameters.
     * @param Ts      Sample time [s].
     * @throws std::invalid_argument If data is too short or parameters are non-positive.
     * @throws std::runtime_error    If the Hessian is not positive-definite (non-PE data).
     */
    DeePC(const Eigen::VectorXd& u_data,
          const Eigen::VectorXd& y_data,
          const Params&          params,
          double                 Ts);

    /**
     * @brief Compute control action from absolute plant output and setpoint.
     *
     * Updates the internal I/O buffer, runs the ADMM solve, and returns the first
     * element of the optimal future input sequence.
     *
     * @param y_meas  Absolute plant output at the current step.
     * @param r       Setpoint for the current and future steps.
     * @return Absolute control input to apply.
     */
    double computeIO(double y_meas, double r);

    // ---- IController interface -----------------------------------------------

    /**
     * @brief IController::compute compatibility.
     *
     * Convention: `compute(error)` where `error = r - y_plant`.
     * Call `setReference(r)` once per step **before** `compute(r-y)`.
     */
    double compute(double error) override;

    /** @brief Store the reference for the next `compute(error)` call. */
    void setReference(double r) noexcept { r_ = r; }

    void        reset()               override;
    double      sampleTime()    const override { return Ts_; }
    std::string name()          const override { return "DeePC"; }

    // ---- Diagnostics ---------------------------------------------------------

    /** @brief Number of ADMM iterations actually run on the last call. */
    int    lastADMMIter()       const noexcept { return p_.admm_iter; }
    /** @brief Primal residual ||H_uf.g - u|| at end of last ADMM solve. */
    double lastPrimalResidual() const noexcept { return primal_res_; }
    /** @brief true once the I/O buffer has been filled (warm-up complete). */
    bool   isWarmedUp()         const noexcept { return step_cnt_ > p_.T_ini; }

private:
    // Build an L * (N-L+1) Hankel matrix from a length-N data vector.
    static Eigen::MatrixXd buildHankel(const Eigen::VectorXd& data, int L);

    // Pre-factor the constant ADMM g-update Hessian.
    void factoriseHessian();

    Params p_;
    double Ts_;
    int    N_;      ///< Offline data length
    int    L_;      ///< L = T_ini + Np
    int    n_col_;  ///< Hankel columns = N - L + 1

    // Hankel matrix blocks
    Eigen::MatrixXd H_up_;  ///< T_ini * n_col  past input
    Eigen::MatrixXd H_uf_;  ///< Np    * n_col  future input
    Eigen::MatrixXd H_yp_;  ///< T_ini * n_col  past output  (for sigma_y diagnostic)
    Eigen::MatrixXd H_yf_;  ///< Np    * n_col  future output

    // Pre-factored ADMM g-update Hessian
    // M = rho_y.H_yf'H_yf + lambda_g.I + lambda_eq.H_up'H_up + rho_admm.H_uf'H_uf
    Eigen::LDLT<Eigen::MatrixXd> M_ldlt_;

    // ADMM warm-start variables (persist across steps for faster convergence)
    Eigen::VectorXd g_;       ///< Hankel coefficient vector (n_col)
    Eigen::VectorXd u_opt_;   ///< Optimal future input sequence (Np)
    Eigen::VectorXd lam_;     ///< Dual variable for H_uf.g = u  (Np)

    // Rolling I/O buffer (length T_ini, oldest index 0, newest index T_ini-1)
    Eigen::VectorXd u_buf_;
    Eigen::VectorXd y_buf_;

    double u_prev_     = 0.0;  ///< Input applied on the previous step
    double r_          = 0.0;  ///< Stored reference for compute(error) compatibility
    int    step_cnt_   = 0;    ///< Steps since reset (warm-up counter)
    double primal_res_ = 0.0;  ///< ||H_uf.g - u|| after last ADMM solve
};

} // namespace ctrl
