#include "GeneralizedPredictiveControl.h"
#include "GradientProjectionQP.h"
#include <algorithm>
#include <iostream>

namespace ctrl
{

    GeneralizedPredictiveController::GeneralizedPredictiveController(
        const StateSpace &plant, const GPCParams &params)
        : plant_(plant), p_(params), Ts_(plant.Ts), r_ref_(0.0)
    {
        const int n = plant_.stateSize();
        const int p = plant_.outputSize();
        const int m = plant_.inputSize();
        xa_     = Eigen::VectorXd::Zero(n + p);
        u_prev_ = Eigen::VectorXd::Zero(m);
        buildCondensedMatrices();
    }

    // ---------------------------------------------------------------------------
    // Build augmented velocity-form model and condensed prediction matrices.
    //
    // Augmented state xa = [Deltax; y]  (size n+p):
    //   Aa = [A,   0 ]     Ba = [B  ]     Ca = [0, I]
    //        [C.A, I ]          [C.B]
    //
    // F, G built identically to DiscreteMPC but using Aa, Ba, Ca.
    // ---------------------------------------------------------------------------
    void GeneralizedPredictiveController::buildCondensedMatrices()
    {
        const int n  = plant_.stateSize();
        const int p  = plant_.outputSize();
        const int m  = plant_.inputSize();
        const int na = n + p;   // augmented state size
        const int Np = p_.Np;
        const int Nu = p_.Nu;

        // Augmented state-space matrices (velocity form / CARIMA)
        Aa_ = Eigen::MatrixXd::Zero(na, na);
        Aa_.topLeftCorner(n, n)     = plant_.A;
        Aa_.bottomLeftCorner(p, n)  = plant_.C * plant_.A;
        Aa_.bottomRightCorner(p, p) = Eigen::MatrixXd::Identity(p, p);

        Ba_.resize(na, m);
        Ba_.topRows(n)    = plant_.B;
        Ba_.bottomRows(p) = plant_.C * plant_.B;

        Ca_ = Eigen::MatrixXd::Zero(p, na);
        Ca_.rightCols(p) = Eigen::MatrixXd::Identity(p, p);

        // Powers of Aa
        std::vector<Eigen::MatrixXd> Apow(Np + 1);
        Apow[0] = Eigen::MatrixXd::Identity(na, na);
        for (int k = 1; k <= Np; ++k)
            Apow[k] = Aa_ * Apow[k - 1];

        // Fa: (Np.p) * na   -   Fa(i,:) = Ca . Aa^{i+1}
        Fa_.resize(Np * p, na);
        for (int i = 0; i < Np; ++i)
            Fa_.block(i * p, 0, p, na) = Ca_ * Apow[i + 1];

        // Ga: (Np.p) * (Nu.m)   -   lower Toeplitz of step response
        Ga_.resize(Np * p, Nu * m);
        Ga_.setZero();
        for (int i = 0; i < Np; ++i)
            for (int j = 0; j <= std::min(i, Nu - 1); ++j)
                Ga_.block(i * p, j * m, p, m) = Ca_ * Apow[i - j] * Ba_;

        // Cost matrices
        Qy_ = p_.rho_y * Eigen::MatrixXd::Identity(Np * p, Np * p);
        Ru_ = p_.rho_u * Eigen::MatrixXd::Identity(Nu * m, Nu * m);

        // Pre-built Hessian
        H_ = Ga_.transpose() * Qy_ * Ga_ + Ru_;

        // Pre-allocate work vectors
        Rtraj_.resize(Np * p);
        err_.resize(Np * p);
        grad_.resize(Nu * m);
        DeltaU_.resize(Nu * m);
        grad_k_.resize(Nu * m);
        DU_new_.resize(Nu * m);
        y_fista_.resize(Nu * m);
        lb_.resize(Nu * m);
        ub_.resize(Nu * m);
        cumMin_.resize(m);
        cumMax_.resize(m);

        // Lipschitz constant for gradient-projection QP step (1/L = safe step size)
        L_ = H_.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff();

        // Pre-factor H_ once; reused every computeRef() call without re-factorisation.
        ldlt_ = H_.ldlt();
    }

    double GeneralizedPredictiveController::computeRef(double y, double r)
    {
        const int m  = plant_.inputSize();
        const int Np = p_.Np;
        const int Nu = p_.Nu;

        // Reference trajectory: y*[k+j] = alpha^j * y[k] + (1 - alpha^j) * r
        // At j=1: y* = alpha*y + (1-alpha)*r  - one step toward r, starting from y[k].
        // As j -> inf: y* -> r  (setpoint is reached asymptotically).
        // alpha = 0: step reference (same as DiscreteMPC - jump immediately to r).
        // alpha -> 1: very slow approach - reduces overshoot but slows disturbance rejection.
        // Typical: alpha = exp(-Ts/tau_ref) where tau_ref is the desired approach time constant.
        double alpha_j = p_.alpha; // alpha^1 for j=1
        for (int i = 0; i < Np; ++i)
        {
            Rtraj_(i) = alpha_j * y + (1.0 - alpha_j) * r;
            alpha_j  *= p_.alpha; // alpha^{j+1} for next step
        }

        // Gradient at DeltaU = 0: g = Ga'.Qy.(Fa.xa - Rtraj)
        err_.noalias()  = Fa_ * xa_ - Rtraj_;
        grad_.noalias() = Ga_.transpose() * (Qy_ * err_);

        if (ldlt_.info() != Eigen::Success)
            return u_prev_(0); // degenerate Hessian - hold previous input

        // Build rolling worst-case tightened bounds - same strategy as DiscreteMPC.
        cumMin_.setZero();
        cumMax_.setZero();
        for (int j = 0; j < Nu; ++j)
        {
            for (int k = 0; k < m; ++k)
            {
                const int idx = j * m + k;
                const double lo = std::max(p_.duMin, p_.uMin - u_prev_(k) - cumMax_(k));
                const double hi = std::min(p_.duMax, p_.uMax - u_prev_(k) - cumMin_(k));
                lb_(idx) = lo;
                ub_(idx) = (hi >= lo) ? hi : lo;
                cumMin_(k) += lo;
                cumMax_(k) += ub_(idx);
            }
        }

        // Output constraint tightening (diagonal approximation).
        // For horizon step j, the SISO output prediction is approximately:
        //   y[j] = Fa[j,:]*xa + Ga[j,j]*DeltaU[j]  (diagonal term only)
        // where Ga[j,j] = Ca*Ba (first Markov parameter, constant on the G diagonal).
        // Rearranging gives a tightened box on DeltaU[j]:
        //   if Ga[j,j] > 0: (yMin - f_j)/g_jj <= DeltaU[j] <= (yMax - f_j)/g_jj
        //   if Ga[j,j] < 0: bounds are flipped.
        // This is exact for j=0; conservative for j>0 (off-diagonal coupling ignored).
        if (p_.yMin > -1e8 || p_.yMax < 1e8)
        {
            const int p = plant_.outputSize();
            for (int j = 0; j < Nu; ++j)
            {
                const double g = Ga_(j * p, j * m); // Ga diagonal element (= Ca*Ba)
                if (std::abs(g) < 1e-14) continue;
                const double f = Fa_.row(j * p).dot(xa_); // Fa[j,:]*xa
                double lo_y = (p_.yMin - f) / g;
                double hi_y = (p_.yMax - f) / g;
                if (g < 0.0) std::swap(lo_y, hi_y); // negative gain flips inequality
                const int idx = j * m;
                lb_(idx) = std::max(lb_(idx), lo_y);
                ub_(idx) = std::min(ub_(idx), hi_y);
                if (ub_(idx) < lb_(idx)) ub_(idx) = lb_(idx); // infeasible: hold
            }
        }

        // Gradient projection - all work vectors pre-allocated; zero per-step allocation.
        const auto qp = solveGradientProjectionQP(
            H_, grad_, lb_, ub_, ldlt_, L_, p_.qpMaxIter, p_.qpTol,
            DeltaU_, grad_k_, DU_new_, y_fista_);
        last_qp_converged_ = qp.converged;
        last_qp_iters_     = qp.iters;

        // Emit convergence telemetry via observer (M3/R3).
        notify_buf_(0) = static_cast<double>(last_qp_iters_);
        notifyObserverState("qp_iters", notify_buf_);
        if (!last_qp_converged_) {
            notify_buf_(0) = 0.0;
            notifyObserverState("health", notify_buf_);
        }

#ifndef NDEBUG
        if (!last_qp_converged_)
            std::clog << "[GPC] WARNING: QP solver reached max iterations ("
                      << p_.qpMaxIter << "). Use lastQPConverged() to check per step.\n";
#endif

        // Apply first control increment and enforce absolute u bounds.
        Eigen::VectorXd du = DeltaU_.head(m);
        Eigen::VectorXd u  = (u_prev_ + du).cwiseMax(p_.uMin).cwiseMin(p_.uMax);
        // Recompute du after saturation so the augmented state is advanced with the
        // ACTUAL increment applied, not the unconstrained optimal. Without this, the
        // velocity-form integrator in xa drifts from the real plant state during saturation.
        du = u - u_prev_;

        // Advance augmented state: xa[k+1] = Aa*xa + Ba*Deltau
        // xa = [Deltax; y] accumulates the integrated output in its lower p entries.
        // This built-in integrating action is what eliminates steady-state offset
        // without a separate integrator state (the CARIMA model advantage).
        xa_ = Aa_ * xa_ + Ba_ * du;

        u_prev_ = u;
        return u(0);
    }

    double GeneralizedPredictiveController::compute(double error)
    {
        if (!std::isfinite(error)) return u_prev_(0);
        // Reconstruct y from stored setpoint and error = r - y
        const double y = r_ref_ - error;
        return computeRef(y, r_ref_);
    }

    void GeneralizedPredictiveController::setPlant(const StateSpace &plant)
    {
        plant_ = plant;
        Ts_    = plant.Ts;
        const int na = plant.stateSize() + plant.outputSize();
        xa_     = Eigen::VectorXd::Zero(na);
        u_prev_ = Eigen::VectorXd::Zero(plant.inputSize());
        buildCondensedMatrices();
    }

    void GeneralizedPredictiveController::setParams(const GPCParams &p)
    {
        p_ = p;
        buildCondensedMatrices();
    }

    void GeneralizedPredictiveController::reset()
    {
        xa_.setZero();
        u_prev_.setZero();
        r_ref_ = 0.0;
    }

} // namespace ctrl
