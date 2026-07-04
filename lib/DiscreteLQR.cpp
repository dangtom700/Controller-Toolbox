#include "DiscreteLQR.h"
#include <Eigen/Eigenvalues>
#include <stdexcept>
#include <string>
#include <iostream>

namespace ctrl
{

    // ---------------------------------------------------------------------------
    // PBH (Popov-Belevitch-Hautus) stabilizability test for discrete-time (A, B).
    // A system is stabilisable iff for every eigenvalue lambda of A with |lambda| >= 1,
    // rank([lambdaI - A, B]) = n.  Returns false if any unstable mode is uncontrollable.
    // ---------------------------------------------------------------------------
    static bool isPBHStabilizable(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
    {
        const int n = A.rows();
        Eigen::EigenSolver<Eigen::MatrixXd> es(A, /*computeEigenvectors=*/false);
        const Eigen::VectorXcd &eigs = es.eigenvalues();

        for (int i = 0; i < eigs.size(); ++i)
        {
            if (std::abs(eigs[i]) < 1.0) continue; // stable mode - skip

            // Build [lambdaI - A | B] and check its rank
            Eigen::MatrixXcd PBH(n, n + B.cols());
            PBH.leftCols(n)  = eigs[i] * Eigen::MatrixXcd::Identity(n, n)
                               - A.cast<std::complex<double>>();
            PBH.rightCols(B.cols()) = B.cast<std::complex<double>>();

            Eigen::JacobiSVD<Eigen::MatrixXcd> svd(PBH);
            const double threshold = 1e-10 * svd.singularValues()(0);
            const int rnk = (svd.singularValues().array().abs() > threshold).count();
            if (rnk < n) return false; // unstable uncontrollable mode found
        }
        return true;
    }

    // ---------------------------------------------------------------------------
    // Doubling algorithm for DARE (Smith / Pappas-Laub-Sandell iteration).
    //
    // Maintains a triplet (X_k, L_k, G_k) such that X_k = T^{2^k}(Q) where T
    // is the Riccati operator.  Each iteration doubles the effective horizon:
    //
    //   W       = I + G_k X_k
    //   Z1      = W^{-1} L_k          (W^{-1} applied on the right of L_k)
    //   Z2      = W^{-1} G_k
    //   X_{k+1} = X_k + L_k' X_k Z1   (2^{k+1}-step cost)
    //   G_{k+1} = G_k + L_k  Z2 L_k'  (2^{k+1}-step controllability Gramian)
    //   L_{k+1} = L_k  Z1              (2^{k+1}-step closed-loop A)
    //
    // Converges quadratically (doubles correct digits each iteration); typically
    // 30-50 iterations suffice for double precision, vs. thousands for value iteration.
    //
    // Ref: Pappas, Laub & Sandell "On a Numerical Solution of the Discrete-Time
    //      Algebraic Riccati Equation" IEEE TAC (1980);
    //      Varga "On solving discrete-time periodic Riccati equations" (2006).
    // ---------------------------------------------------------------------------
    DareResult DiscreteLQR::solveDARE(const Eigen::MatrixXd &A,
                                      const Eigen::MatrixXd &B,
                                      const Eigen::MatrixXd &Q,
                                      const Eigen::MatrixXd &R)
    {
        const int    n      = A.rows();
        const int    maxIter = 100;   // quadratic convergence: far fewer needed than value-iter
        const double tol    = 1e-12;

        const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);

        // G0 = B R^{-1} B'  (initial "controllability Gramian" factor)
        const Eigen::MatrixXd G0 = B * R.ldlt().solve(B.transpose());

        Eigen::MatrixXd X  = Q;   // DARE solution estimate (X_0 = Q = T^1(0))
        Eigen::MatrixXd Lk = A;   // reduced closed-loop A  (L_0 = A)
        Eigen::MatrixXd Gk = G0;  // reduced Gramian        (G_0 = BR^{-1}B')

        for (int iter = 0; iter < maxIter; ++iter)
        {
            const Eigen::MatrixXd W = In + Gk * X; // I + G_k X_k
            const Eigen::PartialPivLU<Eigen::MatrixXd> Wlu(W);

            // W Z1 = Lk  -->  Z1 = W^{-1} Lk
            const Eigen::MatrixXd Z1 = Wlu.solve(Lk);
            // W Z2 = Gk  -->  Z2 = W^{-1} Gk
            const Eigen::MatrixXd Z2 = Wlu.solve(Gk);

            const Eigen::MatrixXd X_new = X  + Lk.transpose() * X * Z1;
            const Eigen::MatrixXd G_new = Gk + Lk * Z2 * Lk.transpose();
            const Eigen::MatrixXd L_new = Lk * Z1;

            const double err = (X_new - X).norm() / (1.0 + X.norm());
            // Enforce symmetry at each iteration: the Riccati solution is symmetric by
            // construction, but each matrix multiply introduces O(eps_mach) asymmetry.
            // Without this, X can drift asymmetric over 30-50 iterations, which causes
            // LDLT on S = R + B'*P*B to report NumericalIssue on near-rank-deficient systems.
            X  = 0.5 * (X_new + X_new.transpose());
            Gk = G_new;
            Lk = L_new;

            if (err < tol)
                return {X, true, iter + 1};
        }

        return {X, false, maxIter};
    }

    // ---------------------------------------------------------------------------
    // PBH detectability test for (A, Q).
    // Detectability requires (A, Q^{1/2}) to satisfy: for every eigenvalue lambda
    // of A with |lambda| >= 1, rank([lambda*I - A; Q^{1/2}]) = n.
    // Equivalently, (A', (Q^{1/2})') must be stabilizable (dual test).
    // We extract Q^{1/2} via eigendecomposition (Q is symmetric PSD).
    // ---------------------------------------------------------------------------
    static bool isPBHDetectable(const Eigen::MatrixXd &A, const Eigen::MatrixXd &Q)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Q);
        const Eigen::VectorXd &sv  = eig.eigenvalues();
        const Eigen::MatrixXd &V   = eig.eigenvectors();

        // Collect rows of Q^{1/2} for non-trivial eigenvalues
        const double eps = 1e-10 * sv.maxCoeff();
        std::vector<int> nz;
        for (int i = 0; i < sv.size(); ++i)
            if (sv(i) > eps) nz.push_back(i);

        if (nz.empty()) return false; // Q = 0: no information about any state

        const int n = A.rows();
        Eigen::MatrixXd CQ(static_cast<int>(nz.size()), n);
        for (int i = 0; i < static_cast<int>(nz.size()); ++i)
            CQ.row(i) = std::sqrt(sv(nz[i])) * V.col(nz[i]).transpose();

        // Detectability of (A, CQ) <=> stabilizability of (A', CQ')
        return isPBHStabilizable(A.transpose(), CQ.transpose());
    }

    DiscreteLQR::DiscreteLQR(const StateSpace &plant, const LQRParams &params)
        : Ts_(plant.Ts), n_states_(plant.stateSize()), n_inputs_(plant.inputSize()),
          dare_converged_(false), dare_iterations_(0)
    {
        if (!isPBHStabilizable(plant.A, plant.B))
        {
            std::cerr << "[DiscreteLQR] WARNING: (A,B) failed the PBH stabilizability test. "
                      << "DARE may not converge - check that all unstable modes are controllable.\n";
        }

        if (!isPBHDetectable(plant.A, params.Q))
        {
            std::cerr << "[DiscreteLQR] WARNING: (A,Q) failed the PBH detectability test. "
                      << "DARE may not converge - check that all unstable modes are penalised "
                      << "by Q (no zero-row in Q corresponding to an unstable state).\n";
        }

        DareResult res = solveDARE(plant.A, plant.B, params.Q, params.R);

        // Fallback: if the DARE did not converge, retry once with a Tikhonov-regularised
        // state cost Q + rho*I. A tiny rho restores detectability of marginal modes (the
        // usual non-convergence cause) without perturbing systems that already converged -
        // this branch never runs on the happy path. A genuinely unstabilisable (A,B) still
        // will not converge (controllability, not Q, is the obstacle), so those cases fall
        // through to the warning below exactly as before.
        if (!res.converged)
        {
            const double rho = 1e-9 * (1.0 + params.Q.norm());
            const Eigen::MatrixXd Qreg =
                params.Q + rho * Eigen::MatrixXd::Identity(n_states_, n_states_);
            const DareResult res_reg = solveDARE(plant.A, plant.B, Qreg, params.R);
            if (res_reg.converged)
                res = res_reg;
        }

        dare_converged_  = res.converged;
        dare_iterations_ = res.iterations;

        if (!res.converged)
        {
            std::cerr << "[DiscreteLQR] WARNING: DARE did not converge in "
                      << res.iterations << " iterations (including a regularised retry). "
                      << "Using last iterate - verify (A,B) is stabilisable "
                      << "and (A,sqrt(Q)) is detectable.\n";
        }

        P_ = res.P;
        const Eigen::MatrixXd S = params.R + plant.B.transpose() * P_ * plant.B;
        K_ = S.ldlt().solve(plant.B.transpose() * P_ * plant.A);
    }

    // u[k] = -K*(x - x_ref) + u_ff
    Eigen::VectorXd DiscreteLQR::compute(const Eigen::VectorXd &x,
                                         const Eigen::VectorXd &x_ref,
                                         const Eigen::VectorXd &u_ff) const
    {
        if (x.size() != n_states_)
            throw std::invalid_argument(
                "DiscreteLQR::compute: x has wrong size (" +
                std::to_string(x.size()) + "), expected " + std::to_string(n_states_));
        if (x_ref.size() != 0 && x_ref.size() != n_states_)
            throw std::invalid_argument(
                "DiscreteLQR::compute: x_ref has wrong size (" +
                std::to_string(x_ref.size()) + "), expected 0 or " + std::to_string(n_states_));
        if (u_ff.size() != 0 && u_ff.size() != n_inputs_)
            throw std::invalid_argument(
                "DiscreteLQR::compute: u_ff has wrong size (" +
                std::to_string(u_ff.size()) + "), expected 0 or " + std::to_string(n_inputs_));

        Eigen::VectorXd xe = x;
        if (x_ref.size() == n_states_)
            xe -= x_ref;

        Eigen::VectorXd u = -K_ * xe;
        if (u_ff.size() == n_inputs_)
            u += u_ff;
        return u;
    }

} // namespace ctrl
