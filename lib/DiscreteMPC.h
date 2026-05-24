#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>

// Discrete-time Model Predictive Controller (condensed incremental QP formulation).
//
// Cost: J = Sigma_{i=1}^{Np} rho_y.||y[k+i|k] - r||^2 + Sigma_{j=0}^{Nc-1} rho_u.||Deltau[k+j]||^2
//
// Prediction (condensed form):
//   Y_pred = F.x[k] + G_u.u_prev + Phi.DeltaU
//   F(i,:)     = C.A^(i+1)                    i = 0,...,Np-1
//   G_u(i,:)   = Sigma_{j=0}^{i} C.A^j.B         (cumulative step response)
//   Phi(i,j,:)   = C.A^(i-j).B                  j <= i, else 0
//
// Unconstrained optimal solution (receding horizon, apply first move only):
//   DeltaU* = -(Phi'.Q_y.Phi + R_u)^-^1.Phi'.Q_y.(F.x[k] - R_stacked)
//   u[k] = u[k-1] + DeltaU*[0:m]
//
// Box constraints on Deltau and u are solved via gradient projection (Lipschitz step 1/L,
// L = max eigenvalue of H, precomputed once).  Bounds for the first control step
// are tightened to reflect both the Deltau limits and the absolute u limits simultaneously.
// qpMaxIter / qpTol tune convergence; defaults are adequate for horizons Nc <= 20.
//
// Ref: Camacho & Bordons "Model Predictive Control" (2007);
//      Maciejowski "Predictive Control with Constraints" (2002);
//      MATLAB mpc(), mpcDesigner.
namespace ctrl
{

    // Tuning parameters.
    struct MPCParams
    {
        int Np = 10;         // Prediction horizon (steps) - covers approx = settling time
        int Nc = 3;          // Control horizon  (steps, Nc <= Np) - fewer = smoother
        double rho_y = 1.0;  // Output tracking weight  (Q_y = rho_y.I_{Np.p})
        double rho_u = 0.1;  // Move suppression weight (R_u = rho_u.I_{Nc.m})
        double uMin = -1e9;  // Hard lower limit on u
        double uMax = 1e9;   // Hard upper limit on u
        double duMin = -1e9; // Hard lower limit on Deltau
        double duMax = 1e9;  // Hard upper limit on Deltau
        int    qpMaxIter = 200;   // Gradient-projection iteration limit
        double qpTol     = 1e-8;  // Convergence tolerance (||Deltax||_inf)
    };

    class DiscreteMPC : public IController
    {
    public:
        // Construct for the given state-space plant and initial tuning.
        // Internally pre-computes condensed matrices F, Phi, and the Hessian H.
        explicit DiscreteMPC(const StateSpace &plant, const MPCParams &params);

        // IController wrapper (SISO convenience).
        // Reconstructs reference as r = y_hat + error and calls computeRef internally.
        double compute(double error) override;

        // Full MIMO interface: optimise u[k] given current state and reference vector.
        Eigen::VectorXd computeRef(const Eigen::VectorXd &x_current,
                                   const Eigen::VectorXd &r_ref);

        void reset() override;
        double sampleTime() const override { return Ts_; }

        // Recompute condensed matrices when horizons or weights change.
        void setParams(const MPCParams &p);
        const MPCParams &params() const { return p_; }

        // Update the internal plant model for online successive linearization
        void setPlant(const StateSpace &plant);

        // Inject a known state estimate (e.g., from a Kalman filter).
        void setState(const Eigen::VectorXd &x) { x_hat_ = x; }

    private:
        StateSpace plant_;
        MPCParams p_;
        double Ts_;
        Eigen::VectorXd x_hat_;  // open-loop state estimate
        Eigen::VectorXd u_prev_; // u[k-1] for incremental form

        // Pre-computed condensed prediction matrices
        Eigen::MatrixXd F_;   // (Np.p) * n
        Eigen::MatrixXd Phi_; // (Np.p) * (Nc.m)
        Eigen::MatrixXd Gu_;  // (Np.p) * m  - cumulative step response for u_prev offset
        Eigen::MatrixXd H_;   // (Phi'.Q_y.Phi + R_u) - precomputed Hessian
        Eigen::MatrixXd Qy_;  // (Np.p) * (Np.p)
        Eigen::MatrixXd Ru_;  // (Nc.m) * (Nc.m)
        double          L_;   // max eigenvalue of H_ - Lipschitz constant for QP step

        // Pre-allocated work vectors - eliminate per-step heap allocation in computeRef()
        Eigen::VectorXd R_stack_;  // Np.p
        Eigen::VectorXd pred_err_; // Np.p
        Eigen::VectorXd grad_;     // Nc.m  (Phi'.Qy.pred_err)
        Eigen::VectorXd DeltaU_;   // Nc.m
        Eigen::VectorXd grad_k_;   // Nc.m  - gradient at current DeltaU_ inside QP loop
        Eigen::VectorXd DU_new_;   // Nc.m  - proposed update inside QP loop
        Eigen::VectorXd lb_;       // Nc.m  - per-horizon lower bounds on DeltaU
        Eigen::VectorXd ub_;       // Nc.m  - per-horizon upper bounds on DeltaU
        Eigen::VectorXd cumMin_;   // m     - rolling cumulative lower bound (bound construction)
        Eigen::VectorXd cumMax_;   // m     - rolling cumulative upper bound (bound construction)

        void buildCondensedMatrices();
    };

} // namespace ctrl
