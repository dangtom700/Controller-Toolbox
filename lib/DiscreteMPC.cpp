#include "DiscreteMPC.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace ctrl
{

    DiscreteMPC::DiscreteMPC(const StateSpace &plant, const MPCParams &params)
        : plant_(plant), p_(params), Ts_(plant.Ts)
    {
        if (plant.D.norm() > 1e-12)
            std::cerr << "[DiscreteMPC] WARNING: plant.D != 0. The compute(error) SISO "
                         "wrapper reconstructs the reference as r = C*x + D*u_prev + error, "
                         "which uses u[k-1] for the D*u term (one step stale). "
                         "For D != 0 plants, use computeRef(x, r) directly and supply "
                         "the current reference explicitly.\n";

        x_hat_ = Eigen::VectorXd::Zero(plant_.stateSize());
        u_prev_ = Eigen::VectorXd::Zero(plant_.inputSize());
        buildCondensedMatrices();
    }

    // ---------------------------------------------------------------------------
    // buildPredictionMatrices - plant-dependent only (A, B, C).
    // Call when the state-space model changes (setPlant).
    //
    //   F(i.p : (i+1).p, :) = C . A^{i+1}        i = 0...Np-1
    //   Phi(i.p, j.m)        = C . A^{i-j} . B    j <= i, 0 otherwise
    //   Gu(i,:)              = Sigma_{j=0}^{i} C.A^j.B
    // ---------------------------------------------------------------------------
    void DiscreteMPC::buildPredictionMatrices()
    {
        const int n  = plant_.stateSize();
        const int m  = plant_.inputSize();
        const int p  = plant_.outputSize();
        const int Np = p_.Np;
        const int Nc = p_.Nc;

        // Powers of A: Apow[k] = A^k
        std::vector<Eigen::MatrixXd> Apow(Np + 1);
        Apow[0] = Eigen::MatrixXd::Identity(n, n);
        for (int k = 1; k <= Np; ++k)
            Apow[k] = plant_.A * Apow[k - 1];

        // F: (Np.p) * n
        F_.resize(Np * p, n);
        for (int i = 0; i < Np; ++i)
            F_.block(i * p, 0, p, n) = plant_.C * Apow[i + 1];

        // Phi: (Np.p) * (Nc.m)
        Phi_.resize(Np * p, Nc * m);
        Phi_.setZero();
        for (int i = 0; i < Np; ++i)
            for (int j = 0; j <= std::min(i, Nc - 1); ++j)
                Phi_.block(i * p, j * m, p, m) = plant_.C * Apow[i - j] * plant_.B;

        // Gu: (Np.p) * m  - cumulative step response for u_prev offset
        Gu_.resize(Np * p, m);
        Gu_.block(0, 0, p, m) = plant_.C * plant_.B;
        for (int i = 1; i < Np; ++i)
            Gu_.block(i * p, 0, p, m) = Gu_.block((i - 1) * p, 0, p, m) + plant_.C * Apow[i] * plant_.B;
    }

    // ---------------------------------------------------------------------------
    // buildCostMatrix - weight-dependent only (rho_y, rho_u).
    // Requires Phi_ to be current. Call after buildPredictionMatrices or
    // whenever rho_y / rho_u change.
    // ---------------------------------------------------------------------------
    void DiscreteMPC::buildCostMatrix()
    {
        const int m  = plant_.inputSize();
        const int p  = plant_.outputSize();
        const int Np = p_.Np;
        const int Nc = p_.Nc;

        // Weight matrices
        Qy_ = p_.rho_y * Eigen::MatrixXd::Identity(Np * p, Np * p);
        Ru_ = p_.rho_u * Eigen::MatrixXd::Identity(Nc * m, Nc * m);

        // Precompute Hessian (positive definite for rho_u > 0)
        H_ = Phi_.transpose() * Qy_ * Phi_ + Ru_;

        // Lipschitz constant = max eigenvalue of H (gradient-projection step 1/L)
        L_ = H_.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff();

        // Pre-factor H_ once; reused every computeRef() call without re-factorisation.
        ldlt_ = H_.ldlt();

        // Pre-allocate work vectors so computeRef() is allocation-free per step
        R_stack_.resize(Np * p);
        pred_err_.resize(Np * p);
        grad_.resize(Nc * m);
        DeltaU_.resize(Nc * m);
        grad_k_.resize(Nc * m);
        DU_new_.resize(Nc * m);
        lb_.resize(Nc * m);
        ub_.resize(Nc * m);
        cumMin_.resize(m);
        cumMax_.resize(m);
    }

    // Full rebuild - plant model + cost matrices. Used by constructor and setPlant().
    void DiscreteMPC::buildCondensedMatrices()
    {
        buildPredictionMatrices();
        buildCostMatrix();
    }

    // IController wrapper - reconstructs reference from error and delegates to computeRef.
    double DiscreteMPC::compute(double error)
    {
        const Eigen::VectorXd y_hat = plant_.C * x_hat_ + plant_.D * u_prev_;
        const Eigen::VectorXd r_ref = y_hat.array() + error; // r = y + (r - y)
        return computeRef(x_hat_, r_ref)(0);
    }

    // Full interface: optimise and return u[k].
    //
    // Solves the box-constrained QP:
    //   min_{DeltaU}  0.5 DeltaU'HDeltaU + g'DeltaU
    //   s.t.      lb_ <= DeltaU <= ub_
    //
    // via gradient projection with constant step alpha = 1/L (L = max eigenvalue of H).
    // Warm-started from the clamped unconstrained optimum; typically converges in
    // a handful of iterations for MPC horizon sizes.
    //
    // Bounds per horizon step j (rolling worst-case tightening):
    //   u[j] = u_prev + Sigma_{i=0}^{j} Deltau_i  must satisfy  uMin <= u[j] <= uMax.
    //
    //   At step j, Deltau_j is bounded by the worst-case contribution from prior steps:
    //     lb_(j,k) = max(duMin, uMin_k - u_prev_k - cumMax_k)
    //     ub_(j,k) = min(duMax, uMax_k - u_prev_k - cumMin_k)
    //   where cumMin/cumMax track the tightest possible cumulative Deltau range.
    //   This guarantees u feasibility across the full horizon without a full-QP reformulation.
    Eigen::VectorXd DiscreteMPC::computeRef(const Eigen::VectorXd &x,
                                            const Eigen::VectorXd &r_ref)
    {
        const int p  = plant_.outputSize();
        const int m  = plant_.inputSize();
        const int Np = p_.Np;
        const int Nc = p_.Nc;

        // Stack reference for all prediction steps
        for (int i = 0; i < Np; ++i)
            R_stack_.segment(i * p, p) = r_ref;

        // Gradient at DeltaU = 0: g = Phi'.Qy.(F.x + Gu.u_prev - R)
        pred_err_.noalias() = F_ * x + Gu_ * u_prev_ - R_stack_;
        grad_.noalias()     = Phi_.transpose() * (Qy_ * pred_err_);

        // Build rolling worst-case tightened box bounds lb_/ub_ on DeltaU \in R^{Nc*m}.
        // cumMin_(k) / cumMax_(k) track the range of cumulative Deltau for input channel k
        // up to (but not including) the current step, assuming each prior Deltau_i was at
        // its own tightened lb/ub.  This is conservative (over-tightens slightly) but
        // guarantees that u = u_prev + Sigma Deltau stays within [uMin, uMax] for every step.
        cumMin_.setZero();
        cumMax_.setZero();
        for (int j = 0; j < Nc; ++j)
        {
            for (int k = 0; k < m; ++k)
            {
                const int idx = j * m + k;
                const double lo = std::max(p_.duMin, p_.uMin - u_prev_(k) - cumMax_(k));
                const double hi = std::min(p_.duMax, p_.uMax - u_prev_(k) - cumMin_(k));
                lb_(idx) = lo;
                ub_(idx) = (hi >= lo) ? hi : lo; // collapse infeasible interval to lo
                cumMin_(k) += lo;
                cumMax_(k) += ub_(idx);
            }
        }

        // Warm-start: clamped unconstrained optimum (ldlt_ pre-factored in buildCondensedMatrices)
        if (ldlt_.info() != Eigen::Success)
            return u_prev_; // degenerate Hessian - hold previous input

        DeltaU_ = (-ldlt_.solve(grad_)).cwiseMax(lb_).cwiseMin(ub_);

        // Gradient projection: DU <- clamp(DU - (1/L)*(H*DU + g), lb_, ub_)
        // All temporaries (grad_k_, DU_new_) are pre-allocated members - no per-iter alloc.
        const double alpha = 1.0 / L_;
        last_qp_converged_ = false;
        int iter = 0;
        for (; iter < p_.qpMaxIter; ++iter)
        {
            grad_k_.noalias() = H_ * DeltaU_ + grad_;
            DU_new_           = (DeltaU_ - alpha * grad_k_).cwiseMax(lb_).cwiseMin(ub_);

            const double delta = (DU_new_ - DeltaU_).cwiseAbs().maxCoeff();
            DeltaU_            = DU_new_;
            if (delta < p_.qpTol)
            {
                last_qp_converged_ = true;
                break;
            }
        }
        last_qp_iters_ = iter;

        if (!last_qp_converged_)
        {
            std::clog << "[DiscreteMPC] WARNING: QP solver reached max iterations (" << p_.qpMaxIter 
                      << ") without converging. Consider increasing qpMaxIter or relaxing tuning.\n";
        }

        // Apply first control increment
        const Eigen::VectorXd du = DeltaU_.head(m);
        const Eigen::VectorXd u  = (u_prev_ + du).cwiseMax(p_.uMin).cwiseMin(p_.uMax);

        // Advance open-loop state estimate (used by compute() wrapper next step).
        // This is a pure-prediction step with no measurement correction. For plants
        // with model mismatch, persistent disturbances, or integrating modes, x_hat_
        // will drift away from the real plant state over time. Call setState() with an
        // external observer estimate (e.g., KalmanFilter::state()) before each step to
        // avoid this. The compute(error) wrapper is only reliable for D=0, well-matched
        // models without significant unmeasured disturbances.
        x_hat_  = plant_.A * x + plant_.B * u;
        u_prev_ = u;

        return u;
    }

    void DiscreteMPC::setParams(const MPCParams &p)
    {
        const bool horizon_changed = (p.Np != p_.Np) || (p.Nc != p_.Nc);
        p_ = p;
        // Horizon changes affect F_, Phi_, Gu_ - full rebuild required.
        // Weight-only changes (rho_y, rho_u) only need to rebuild H_, L_, ldlt_.
        if (horizon_changed)
            buildCondensedMatrices();
        else
            buildCostMatrix();
    }

    void DiscreteMPC::setPlant(const StateSpace &plant)
    {
        plant_ = plant;
        Ts_ = plant.Ts;
        buildCondensedMatrices();
    }

    void DiscreteMPC::reset()
    {
        x_hat_.setZero();
        u_prev_.setZero();
    }

} // namespace ctrl
