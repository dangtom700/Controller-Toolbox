#include "GeneralizedPredictiveControl.h"
#include <algorithm>

namespace ctrl
{

    GeneralizedPredictiveController::GeneralizedPredictiveController(
        const StateSpace &plant, const GPCParams &params)
        : plant_(plant), p_(params), Ts_(plant.Ts), r_ref_(0.0), y_prev_(0.0)
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
    // Augmented state xa = [Δx; y]  (size n+p):
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
    }

    double GeneralizedPredictiveController::computeRef(double y, double r)
    {
        const int p  = plant_.outputSize();
        const int m  = plant_.inputSize();
        const int n  = plant_.stateSize();
        const int Np = p_.Np;
        const int Nu = p_.Nu;

        // Build reference trajectory: y*[k+j] = alpha^j.y + (1-alpha^j).r
        double alpha_j = p_.alpha;
        for (int i = 0; i < Np; ++i)
        {
            Rtraj_(i) = alpha_j * y + (1.0 - alpha_j) * r;
            alpha_j  *= p_.alpha;
        }

        // Unconstrained QP: ΔU* = -H^-1 . Ga' . Qy . (Fa.xa - Rtraj)
        err_.noalias() = Fa_ * xa_ - Rtraj_;
        const auto ldlt = H_.ldlt();
        if (ldlt.info() != Eigen::Success)
        {
            // Hessian singular - hold previous input
            y_prev_ = y;
            return u_prev_(0);
        }
        grad_.noalias() = Ga_.transpose() * (Qy_ * err_);
        DeltaU_         = -ldlt.solve(grad_);

        // First control increment with box constraints
        Eigen::VectorXd du = DeltaU_.head(m);
        du = du.cwiseMax(p_.duMin).cwiseMin(p_.duMax);
        Eigen::VectorXd u = (u_prev_ + du).cwiseMax(p_.uMin).cwiseMin(p_.uMax);
        du = u - u_prev_; // recompute after u-saturation

        // Advance augmented state: xa[k+1] = Aa.xa + Ba.Δu
        xa_ = Aa_ * xa_ + Ba_ * du;

        u_prev_ = u;
        y_prev_ = y;
        return u(0);
    }

    double GeneralizedPredictiveController::compute(double error)
    {
        // Reconstruct y from stored y_prev_ and error = r - y
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
        y_prev_ = 0.0;
        r_ref_  = 0.0;
    }

} // namespace ctrl
