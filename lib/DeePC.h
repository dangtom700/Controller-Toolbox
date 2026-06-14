#pragma once
#include "IController.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <deque>
#include <string>
#include <stdexcept>

namespace ctrl {

/**
 * @brief Tuning parameters for the DeePC controller.
 *
 * @note Persistency of excitation requirement for reliable prediction:
 *       offline data length N_data should satisfy N_data >= T_ini + N + n_order,
 *       where n_order is the estimated system order. The Hankel matrix will have
 *       M = N_data - T_ini - N + 1 columns.
 */
struct DeePCParams {
    int    T_ini       = 20;    ///< Past trajectory window (steps).
    int    N           = 10;    ///< Prediction horizon (steps).
    double Q           = 1.0;   ///< Output tracking weight (scalar, applied to each horizon step).
    double R           = 0.1;   ///< Future input regularization weight.
    double lambda_g    = 1.0;   ///< Tikhonov regularization on Hankel coefficients g.
    double lambda_y    = 100.0; ///< Past output trajectory soft-constraint weight.
    double lambda_u    = 10.0;  ///< Past input trajectory soft-constraint weight.
    double uMin        = -1e9;  ///< Hard lower bound on control output.
    double uMax        =  1e9;  ///< Hard upper bound on control output.
    double rho         = 10.0;  ///< ADMM penalty parameter.
    int    admm_iters  = 200;   ///< Maximum ADMM iterations per control step.
    double admm_tol    = 1e-4;  ///< Primal/dual residual convergence tolerance. 1e-4 is practical for unnormalized Hankel matrices; tighten only if absolute solution accuracy matters more than control quality.
};

/**
 * @brief Data-Enabled Predictive Control (DeePC) - SISO.
 *
 * Implements Coulson, Lygeros & Dorfler (2019) using Willems' Fundamental Lemma.
 * All LTI system behaviours of length T_ini+N are contained in the column span of
 * the Hankel matrix built from a persistently exciting offline dataset; no explicit
 * system identification is needed.
 *
 * Formulation (soft-constrained):
 * @code
 *   min_{g, u_f}  Q||H_yf g - r||^2 + R||u_f||^2 + lambda_g||g||^2
 *               + lambda_y||H_yp g - y_ini||^2 + lambda_u||H_up g - u_ini||^2
 *   s.t.  H_uf g = u_f,  uMin <= u_f <= uMax
 * @endcode
 *
 * Solved via ADMM with a pre-factored constant Hessian (LDLT, computed once at
 * collectData() time).  Per-step cost: O(M^2) linear solve + O(M*N) products.
 *
 * Sign convention: compute(y_measured) - same as DiscretePID/DiscreteMPC.
 * setReference(r) sets the output setpoint.
 *
 * Usage:
 * @code
 *   DeePCParams p;  p.T_ini=5; p.N=10; p.Q=10.0; p.R=0.01; p.lambda_g=1.0;
 *   DeePC ctrl(p, Ts);
 *   ctrl.collectData(u_prbs, y_prbs);   // offline persistently exciting data
 *   ctrl.setReference(r);
 *   // control loop:
 *   double u = ctrl.compute(y_meas);
 * @endcode
 */
class DeePC : public IController {
public:
    /**
     * @brief Construct a DeePC controller.
     * @param params Tuning parameters.
     * @param Ts     Sample time [s].
     */
    DeePC(const DeePCParams& params, double Ts);

    /**
     * @brief Provide offline data and build Hankel matrices.
     *
     * Must be called once before compute(). Data should come from a PRBS or
     * random-input experiment on the real (or simulated) plant.
     *
     * @param u_data Offline input sequence, length N_data >= T_ini + N + 1.
     * @param y_data Corresponding output sequence, same length as u_data.
     * @throws std::invalid_argument if sizes mismatch or data is too short.
     */
    void collectData(const Eigen::VectorXd& u_data, const Eigen::VectorXd& y_data);

    /** @brief True once collectData() has been called successfully. */
    bool isDataCollected() const { return data_collected_; }

    /** @brief Number of Hankel columns M = N_data - T_ini - N + 1. */
    int hankelColumns() const { return M_; }

    /**
     * @brief Set the constant output reference for the prediction horizon.
     * @param r Desired output setpoint.
     */
    void setReference(double r);

    /**
     * @brief Advance one sample: update history, solve ADMM QP, return u[k].
     *
     * Returns the last finite output (hold-last) when y_meas is non-finite
     * or when collectData() has not yet been called.
     *
     * @param y_meas Measured plant output y[k].
     * @return Control action u[k].
     */
    double compute(double y_meas) override;

    /**
     * @brief Reset past trajectory buffers and ADMM warm-start vectors.
     * Collected Hankel data is preserved.
     */
    void reset() override;

    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "DeePC"; }

    /**
     * @brief False if the last ADMM solve did not converge within admm_iters.
     * The best-iterate output is still applied; consider a fallback controller.
     */
    bool isHealthy() const override { return healthy_; }

private:
    void buildHankel(const Eigen::VectorXd& u_data, const Eigen::VectorXd& y_data);
    void buildQPMatrices();
    double solveADMM();

    DeePCParams p_;
    double Ts_;
    bool   data_collected_ = false;
    bool   healthy_        = true;
    int    M_              = 0;   ///< Hankel column count

    // Hankel blocks (each has M_ columns)
    Eigen::MatrixXd H_up_, H_yp_;      ///< Past blocks  (T_ini x M_)
    Eigen::MatrixXd H_uf_, H_yf_;      ///< Future blocks (N x M_)

    // Pre-transposed views for efficient inner products
    Eigen::MatrixXd H_up_T_, H_yp_T_;
    Eigen::MatrixXd H_uf_T_, H_yf_T_;

    // Pre-factored constant QP Hessian
    Eigen::MatrixXd                   A_g_;
    Eigen::LDLT<Eigen::MatrixXd>      A_g_ldlt_;

    // Reference-dependent part of b_g (recomputed only when reference changes)
    Eigen::VectorXd b_g_ref_;   ///< 2*Q * H_yf' * r_stacked

    // Past trajectory buffers (length T_ini, ring-buffer via deque)
    std::deque<double> u_buf_;
    std::deque<double> y_buf_;

    // ADMM workspace (pre-allocated at collectData() time)
    Eigen::VectorXd g_;         ///< M-vector: Hankel coefficients (warm-started across steps)
    Eigen::VectorXd uf_;        ///< N-vector: future inputs
    Eigen::VectorXd dual_;      ///< N-vector: ADMM dual variable
    Eigen::VectorXd b_g_;       ///< M-vector: full RHS for g-update
    Eigen::VectorXd b_g_ini_;   ///< M-vector: past-trajectory contribution to b_g
    Eigen::VectorXd H_uf_g_;    ///< N-vector: H_uf * g (scratch)

    // Reference
    double          r_         = 0.0;
    Eigen::VectorXd r_stacked_;       ///< N-vector of r_

    // NaN hold-last
    double u_last_ = 0.0;
};

CTRL_REGISTER_FEATURE(deepc)

} // namespace ctrl
