#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>

// Discrete-time Generalised Predictive Controller (GPC).
//
// Extends DiscreteMPC with two features aimed at practical deployment:
//
//  1. Velocity-form (CARIMA) model - offset-free tracking without a separate
//     integral state. The augmented state xa = [Deltax; y] is propagated with
//     incremental inputs DeltaU, giving a built-in integrating disturbance model.
//
//     Augmented state-space (n_a = n + p):
//       xa[k+1] = Aa.xa[k] + Ba.Deltau[k]      Aa = [A, 0; C.A, I]
//       y[k]    = Ca.xa[k]                  Ba = [B; C.B],  Ca = [0, I]
//
//  2. Reference trajectory - setpoint is approached along a first-order filter
//     y*[k+j] = alpha^j.y[k] + (1-alpha^j).r[k],  alpha \in [0,1)
//     instead of a step. Reduces overshoot without tuning derivative weights.
//     alpha = 0: step reference (same as DiscreteMPC).  alpha -> 1: very soft approach.
//
// Prediction (condensed form, same QP structure as DiscreteMPC):
//   Y_pred = Fa.xa[k] + Ga.DeltaU
//   DeltaU*    = -(Ga'.Q.Ga + R)^-1 . Ga'.Q.(Fa.xa - R_traj)
//
// The plant model can be hot-swapped via setPlant() for adaptive GPC
// (pair with RecursiveLeastSquares::toStateSpace() for a self-tuning loop).
//
// Ref: Clarke et al. "Generalised Predictive Control" Automatica (1987);
//      Mohtadi & Clarke "GPC: further results" IEE (1987);
//      Maciejowski "Predictive Control with Constraints" (2002) Ch.3.
namespace ctrl
{

struct GPCParams
{
    int    Np    = 10;    // prediction horizon  (steps)
    int    Nu    = 3;     // control  horizon    (steps, Nu <= Np)
    double rho_y = 1.0;   // output tracking weight  (Qy = rho_y . I)
    double rho_u = 0.1;   // move suppression weight (Ru = rho_u . I)
    double alpha = 0.0;   // reference trajectory filter  0 = step, ->1 = soft
    double uMin  = -1e9;
    double uMax  =  1e9;
    double duMin = -1e9;
    double duMax =  1e9;
    int    qpMaxIter = 200;  // gradient-projection iteration limit
    double qpTol     = 1e-8; // convergence tolerance (||Deltax||_inf)
};

class GeneralizedPredictiveController : public IController
{
public:
    // plant:  discrete-time state-space model (A,B,C,D,Ts).
    //         D should be zero; non-zero D is accepted but ignored in the
    //         velocity-form augmentation (the direct-feedthrough is not captured).
    // params: GPC tuning parameters.
    GeneralizedPredictiveController(const StateSpace &plant, const GPCParams &params);

    // IController interface - takes error = r - y.
    // Uses setpoint stored via setReference(); defaults to r=0.
    double compute(double error) override;

    // Full interface: supply current plant output y and setpoint r separately.
    // This is preferred over compute(error) for GPC because the reference
    // trajectory requires y[k] independently of the error.
    double computeRef(double y, double r);

    // Hot-swap plant model (adaptive GPC). Rebuilds condensed matrices.
    void setPlant(const StateSpace &plant);

    void setParams(const GPCParams &p);
    const GPCParams &params() const { return p_; }

    void setReference(double r) { r_ref_ = r; }

    void   reset()            override;
    double sampleTime() const override { return Ts_; }

    // Augmented state estimate xa = [Deltax; y] (size n+p).
    const Eigen::VectorXd &augmentedState() const { return xa_; }

    // QP solver diagnostics from the most recent computeRef() call.
    // lastQPConverged() returns false when the gradient-projection loop exited
    // at p_.qpMaxIter without satisfying p_.qpTol. Monitor in production: repeated
    // non-convergence means qpMaxIter is too small or H_ is ill-conditioned.
    bool lastQPConverged() const { return last_qp_converged_; }
    int  lastQPIters()     const { return last_qp_iters_; }

    // IController health interface - returns false when the most recent QP exited
    // at p_.qpMaxIter without converging.  ControllerStack uses this to skip an
    // unhealthy GPC and fall back to the next eligible entry.
    bool isHealthy() const override { return last_qp_converged_; }

private:
    void buildCondensedMatrices();

    StateSpace plant_;
    GPCParams  p_;
    double     Ts_;

    // Augmented state-space (velocity form)
    Eigen::MatrixXd Aa_, Ba_;  // (n+p)*(n+p), (n+p)*m
    Eigen::MatrixXd Ca_;       // p*(n+p)

    // Condensed prediction matrices for augmented model
    Eigen::MatrixXd Fa_;       // (Np.p) * (n+p)
    Eigen::MatrixXd Ga_;       // (Np.p) * (Nu.m)
    Eigen::MatrixXd Qy_, Ru_;
    Eigen::MatrixXd H_;        // pre-built Hessian
    Eigen::LDLT<Eigen::MatrixXd> ldlt_; // pre-factored H_, refreshed in buildCondensedMatrices()

    // Pre-allocated work vectors (all sized at buildCondensedMatrices time)
    Eigen::VectorXd Rtraj_;    // reference trajectory stack (Np.p)
    Eigen::VectorXd err_;      // prediction error (Np.p)
    Eigen::VectorXd grad_;     // unconstrained gradient (Nu.m)
    Eigen::VectorXd DeltaU_;   // optimal increments (Nu.m)
    Eigen::VectorXd grad_k_;   // gradient at current DeltaU_ inside QP loop (Nu.m)
    Eigen::VectorXd DU_new_;   // proposed update inside QP loop (Nu.m)
    Eigen::VectorXd lb_;       // per-horizon lower bounds on DeltaU (Nu.m)
    Eigen::VectorXd ub_;       // per-horizon upper bounds on DeltaU (Nu.m)
    Eigen::VectorXd cumMin_;   // rolling cumulative lower bound (m)
    Eigen::VectorXd cumMax_;   // rolling cumulative upper bound (m)

    double          L_;        // max eigenvalue of H_ - Lipschitz constant for QP step

    bool last_qp_converged_ = true; // false when QP exited at p_.qpMaxIter
    int  last_qp_iters_     = 0;    // actual iterations used in last computeRef()

    Eigen::VectorXd xa_;       // augmented state estimate
    Eigen::VectorXd u_prev_;   // u[k-1] (needed for Deltau = u - u_prev)
    double          r_ref_;    // stored setpoint for compute(error) wrapper
};

} // namespace ctrl
