#include "DiscreteMPC.h"
#include <algorithm>

namespace ctrl
{

    DiscreteMPC::DiscreteMPC(const StateSpace &plant, const MPCParams &params)
        : plant_(plant), p_(params), Ts_(plant.Ts)
    {
        x_hat_ = Eigen::VectorXd::Zero(plant_.stateSize());
        u_prev_ = Eigen::VectorXd::Zero(plant_.inputSize());
        buildCondensedMatrices();
    }

    // ---------------------------------------------------------------------------
    // Build condensed prediction matrices F and Φ.
    //
    //   F(i.p : (i+1).p, :) = C . A^{i+1}        i = 0...Np-1
    //   Φ(i.p, j.m)          = C . A^{i-j} . B   j <= i, 0 otherwise
    //
    // Then precompute the Hessian:
    //   H = (Φ'.Q_y.Φ + R_u)
    // ---------------------------------------------------------------------------
    void DiscreteMPC::buildCondensedMatrices()
    {
        const int n = plant_.stateSize();
        const int m = plant_.inputSize();
        const int p = plant_.outputSize();
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

        // Φ: (Np.p) * (Nc.m)
        Phi_.resize(Np * p, Nc * m);
        Phi_.setZero();
        for (int i = 0; i < Np; ++i)
            for (int j = 0; j <= std::min(i, Nc - 1); ++j)
                Phi_.block(i * p, j * m, p, m) = plant_.C * Apow[i - j] * plant_.B;

        // G_u: (Np.p) * m  — G_u(i) = Σ_{j=0}^{i} C.A^j.B  (cumulative step response)
        Gu_.resize(Np * p, m);
        Gu_.block(0, 0, p, m) = plant_.C * plant_.B; // j=0: C.A^0.B = C.B
        for (int i = 1; i < Np; ++i)
            Gu_.block(i * p, 0, p, m) = Gu_.block((i - 1) * p, 0, p, m) + plant_.C * Apow[i] * plant_.B;

        // Weight matrices
        Qy_ = p_.rho_y * Eigen::MatrixXd::Identity(Np * p, Np * p);
        Ru_ = p_.rho_u * Eigen::MatrixXd::Identity(Nc * m, Nc * m);

        // Precompute Hessian (positive definite for ρ_u > 0)
        H_ = Phi_.transpose() * Qy_ * Phi_ + Ru_;

        // Pre-allocate work vectors so computeRef() is allocation-free per step
        R_stack_.resize(Np * p);
        pred_err_.resize(Np * p);
        grad_.resize(Nc * m);
        DeltaU_.resize(Nc * m);
    }

    // IController wrapper - reconstructs reference from error and delegates to computeRef.
    double DiscreteMPC::compute(double error)
    {
        const Eigen::VectorXd y_hat = plant_.C * x_hat_ + plant_.D * u_prev_;
        const Eigen::VectorXd r_ref = y_hat.array() + error; // r = y + (r - y)
        return computeRef(x_hat_, r_ref)(0);
    }

    // Full interface: optimise and return u[k].
    Eigen::VectorXd DiscreteMPC::computeRef(const Eigen::VectorXd &x,
                                            const Eigen::VectorXd &r_ref)
    {
        const int p = plant_.outputSize();
        const int m = plant_.inputSize();
        const int Np = p_.Np;
        const int Nc = p_.Nc;

        // Stack reference for all prediction steps (uses pre-allocated buffer)
        for (int i = 0; i < Np; ++i)
            R_stack_.segment(i * p, p) = r_ref;

        // Unconstrained optimal ΔU* = -H^-¹.Φ'.Q_y.(F.x + G_u.u_prev - R_stack)
        pred_err_.noalias() = F_ * x + Gu_ * u_prev_ - R_stack_;
        const auto ldlt = H_.ldlt();
        if (ldlt.info() != Eigen::Success)
        {
            // Hessian is rank-deficient; hold previous input rather than emitting garbage
            return u_prev_;
        }
        grad_.noalias() = Phi_.transpose() * (Qy_ * pred_err_);
        DeltaU_ = -ldlt.solve(grad_);

        // Extract first control increment and apply box constraints
        Eigen::VectorXd du = DeltaU_.head(m);
        du = du.cwiseMax(p_.duMin).cwiseMin(p_.duMax);

        Eigen::VectorXd u = (u_prev_ + du).cwiseMax(p_.uMin).cwiseMin(p_.uMax);
        du = u - u_prev_; // recompute after u-saturation to keep du consistent

        // Advance open-loop state estimate (used by compute() wrapper next step)
        x_hat_ = plant_.A * x + plant_.B * u;
        u_prev_ = u;

        return u;
    }

    void DiscreteMPC::setParams(const MPCParams &p)
    {
        p_ = p;
        buildCondensedMatrices();
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
