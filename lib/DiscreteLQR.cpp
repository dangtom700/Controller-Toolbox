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
    // Value-iteration DARE solver
    //   P_{t+1} = A'P_t A - (A'P_t B)(R + B'P_t B)^-¹(B'P_t A) + Q
    // Converges to the stabilising solution Pinf for stabilisable (A,B) + detectable (A,Q½).
    // Ref: Bertsekas "Dynamic Programming and Optimal Control" Vol 1, Section 1.3.
    // ---------------------------------------------------------------------------
    DareResult DiscreteLQR::solveDARE(const Eigen::MatrixXd &A,
                                      const Eigen::MatrixXd &B,
                                      const Eigen::MatrixXd &Q,
                                      const Eigen::MatrixXd &R)
    {
        const int maxIter = 10000;
        const double tol = 1e-12;

        Eigen::MatrixXd P = Q;

        for (int iter = 0; iter < maxIter; ++iter)
        {
            const Eigen::MatrixXd AtP = A.transpose() * P;
            const Eigen::MatrixXd S = R + B.transpose() * P * B;
            const Eigen::MatrixXd Gain = S.ldlt().solve(B.transpose() * P * A);
            const Eigen::MatrixXd P_new = AtP * A - AtP * B * Gain + Q;

            const double err = (P_new - P).norm() / (1.0 + P.norm());
            P = P_new;

            if (err < tol)
                return {P, true, iter + 1};
        }

        // Return the best available iterate instead of throwing so the caller
        // can decide whether to accept an approximate solution or take other action.
        return {P, false, maxIter};
    }

    DiscreteLQR::DiscreteLQR(const StateSpace &plant, const LQRParams &params)
        : Ts_(plant.Ts), n_(plant.stateSize()), m_(plant.inputSize()),
          dare_converged_(false), dare_iterations_(0)
    {
        if (!isPBHStabilizable(plant.A, plant.B))
        {
            std::cerr << "[DiscreteLQR] WARNING: (A,B) failed the PBH stabilizability test. "
                      << "DARE may not converge - check that all unstable modes are controllable.\n";
        }

        DareResult res = solveDARE(plant.A, plant.B, params.Q, params.R);
        dare_converged_  = res.converged;
        dare_iterations_ = res.iterations;

        if (!res.converged)
        {
            std::cerr << "[DiscreteLQR] WARNING: DARE did not converge in "
                      << res.iterations << " iterations. "
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
        Eigen::VectorXd xe = x;
        if (x_ref.size() == n_)
            xe -= x_ref;

        Eigen::VectorXd u = -K_ * xe;
        if (u_ff.size() == m_)
            u += u_ff;
        return u;
    }

} // namespace ctrl
