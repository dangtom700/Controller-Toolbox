#include "PlantModel.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

namespace ctrl
{

    // ---------------------------------------------------------------------------
    // StateSpace::validate
    // ---------------------------------------------------------------------------
    void StateSpace::validate() const
    {
        if (A.rows() != A.cols())
            throw std::invalid_argument("StateSpace: A must be square (n x n).");
        const int n = static_cast<int>(A.rows());
        if (B.rows() != n)
            throw std::invalid_argument("StateSpace: B must have n rows.");
        if (C.cols() != n)
            throw std::invalid_argument("StateSpace: C must have n columns.");
        const int p = static_cast<int>(C.rows());
        const int m = static_cast<int>(B.cols());
        if (D.rows() != p)
            throw std::invalid_argument("StateSpace: D row count must match C row count (p).");
        if (D.cols() != m)
            throw std::invalid_argument("StateSpace: D column count must match B column count (m).");
        if (Ts < 0.0)
            throw std::invalid_argument("StateSpace: Ts must be >= 0 (0 = continuous-time).");
    }

    // ---------------------------------------------------------------------------
    // tf2ss - controllable canonical form
    //
    // For H(z^-^1) = (b0 + b1z^-^1 + ... + b_nz^-^n) / (1 + a1z^-^1 + ... + a_nz^-^n):
    //
    //   A = [[-a1, -a2, ..., -a_n],   (companion row)
    //        [ 1,   0,  ...,  0  ],
    //        ...
    //        [ 0,   0,  ...,  0  ]]   (n*n)
    //   B = [[1], [0], ..., [0]]      (n*1)
    //   C = [[b1-a1b0, b2-a2b0, ..., b_n-a_nb0]]  (1*n)
    //   D = [[b0]]                  (1*1)
    //
    // Equivalent MATLAB:  [A,B,C,D] = tf2ss(num,den) in z^-^1 convention.
    // ---------------------------------------------------------------------------
    StateSpace tf2ss(const TransferFunction &tf)
    {
        int n = tf.order(); // state dimension = denominator degree

        // Pad numerator to exactly n+1 coefficients (prepend zeros if shorter).
        std::vector<double> num = tf.num;
        if (static_cast<int>(num.size()) < n + 1) {
            std::vector<double> padded(n + 1 - static_cast<int>(num.size()), 0.0);
            padded.insert(padded.end(), num.begin(), num.end());
            num = std::move(padded);
        }

        double d0 = num[0]; // feed-through / direct term = b0

        // Build A (n*n companion matrix, first row = -a1...-a_n, sub-diagonal = 1).
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
        for (int j = 0; j < n; ++j)
            A(0, j) = -tf.den[j + 1];
        for (int i = 1; i < n; ++i)
            A(i, i - 1) = 1.0;

        // B = [1, 0, ..., 0]'
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, 1);
        B(0, 0) = 1.0;

        // C = [b1-a1b0, ..., b_n-a_nb0]  (long-division remainder numerator)
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(1, n);
        for (int j = 0; j < n; ++j)
            C(0, j) = num[j + 1] - d0 * tf.den[j + 1];

        Eigen::MatrixXd D(1, 1);
        D(0, 0) = d0;

        return StateSpace(A, B, C, D, tf.Ts);
    }

    // ---------------------------------------------------------------------------
    // ssStep - advance one discrete step and return current output.
    // Output is computed before state is updated so that y[k] = C.x[k] + D.u[k].
    // ---------------------------------------------------------------------------
    Eigen::VectorXd ssStep(const StateSpace &sys,
                           Eigen::Ref<Eigen::VectorXd> x,
                           const Eigen::VectorXd &u)
    {
        Eigen::VectorXd y = sys.C * x + sys.D * u; // y[k]
        x = sys.A * x + sys.B * u;                 // x[k+1]
        return y;
    }

    std::pair<Eigen::VectorXd, Eigen::VectorXd> ssStepCopy(
        const StateSpace &sys,
        const Eigen::VectorXd &x,
        const Eigen::VectorXd &u)
    {
        return {sys.C * x + sys.D * u,   // y[k]
                sys.A * x + sys.B * u};  // x[k+1]
    }

    // ---------------------------------------------------------------------------
    // ss2tf - SISO state-space to transfer function via Markov parameters.
    //
    // Denominator: characteristic polynomial of A via root expansion of eigenvalues.
    //   p(z) = Pi (z - lambda_i)  ->  a = [1, a_1, ..., a_n]
    //
    // Numerator via the impulse-response / Markov-parameter identity:
    //   h[0] = D,  h[k] = C A^{k-1} B  for k >= 1
    //   b[k] = Sigma_{j=0}^{k} a[j] h[k-j]
    //
    // This follows from H(z^{-1}) = N(z^{-1}) / D(z^{-1}) and the convolution
    // relation between the denominator coefficients and the impulse response.
    // ---------------------------------------------------------------------------
    TransferFunction ss2tf(const StateSpace &sys)
    {
        if (sys.inputSize() != 1 || sys.outputSize() != 1)
            throw std::invalid_argument("ss2tf: only SISO plants supported (inputSize == outputSize == 1).");

        const int n = sys.stateSize();

        // ---- Denominator: characteristic polynomial via Faddeev-LeVerrier recursion ----
        // Operates in real arithmetic on A directly; avoids eigenvalue computation
        // and the catastrophic cancellation of root-product expansion for clustered
        // or repeated eigenvalues.
        //
        // Recurrence (monic, coefficients of z^n + a[1]z^{n-1} + ... + a[n]):
        //   M_0 = I,  a[k] = -trace(A * M_{k-1}) / k,  M_k = A*M_{k-1} + a[k]*I
        // At the end, A * M_n = 0  (Cayley-Hamilton check).
        std::vector<double> a(n + 1, 0.0);
        {
            a[0] = 1.0;
            Eigen::MatrixXd M = Eigen::MatrixXd::Identity(n, n);
            for (int k = 1; k <= n; ++k)
            {
                M = sys.A * M;
                a[k] = -M.trace() / k;
                M.diagonal().array() += a[k];
            }
        }

        // ---- Markov parameters: h[0]=D, h[k]=C A^{k-1} B for k=1..n ----
        std::vector<double> h(n + 1);
        h[0] = sys.D(0, 0);
        Eigen::MatrixXd Apow = Eigen::MatrixXd::Identity(n, n); // A^0
        for (int k = 1; k <= n; ++k)
        {
            h[k] = (sys.C * Apow * sys.B)(0, 0);
            Apow = sys.A * Apow; // A^k for next iter
        }

        // ---- Numerator: b[k] = Sigma_{j=0}^{k} a[j] h[k-j] ----
        std::vector<double> num(n + 1, 0.0);
        for (int k = 0; k <= n; ++k)
            for (int j = 0; j <= k; ++j)
                num[k] += a[j] * h[k - j];

        // Convert to z^{-1} convention (monic denominator already has a[0]=1)
        std::vector<double> den(a.begin(), a.end());
        return TransferFunction(num, den, sys.Ts);
    }

    // ---------------------------------------------------------------------------
    // c2d internal helper: warn when a discrete A matrix has eigenvalues outside
    // the unit disk despite a stable continuous-time plant.  This can happen when
    // ||Ac|| * Ts is large (stiff system + coarse sample time) or when the matrix
    // exponential loses precision for ill-conditioned Ac.
    // ---------------------------------------------------------------------------
    static void checkDiscreteStability(const Eigen::MatrixXd &Ad, const char *context)
    {
        const auto eigs = Ad.eigenvalues();
        for (int i = 0; i < eigs.size(); ++i)
        {
#ifndef NDEBUG
            if (std::abs(eigs[i]) >= 1.0 + 1e-9)
            {
                std::cerr << "[PlantModel::" << context << "] WARNING: A_d eigenvalue "
                          << eigs[i] << " (|lambda| = " << std::abs(eigs[i])
                          << ") lies outside the unit disk.\n"
                          << "  This may indicate numerical loss of precision in the matrix "
                             "exponential (ZOH)\n"
                          << "  or a bilinear transform of a marginally stable plant (Tustin).\n"
                          << "  Check that Ts << 1 / |lambda_max(Ac)| for all fast modes.\n";
            }
#endif
        }
    }

    // ---------------------------------------------------------------------------
    // c2d
    // ---------------------------------------------------------------------------
    StateSpace c2d(const StateSpace &sys_c, double Ts, C2dMethod method,
                   double prewarp_freq)
    {
        if (Ts <= 0.0)
            throw std::invalid_argument("c2d: Ts must be positive.");
        if (sys_c.Ts != 0.0)
            throw std::invalid_argument("c2d: input model must be continuous-time (Ts == 0).");

        const int n = sys_c.stateSize();
        const int m = sys_c.inputSize();

        if (method == C2dMethod::ZOH)
        {
            // ZOH discretisation via the Van Loan (1978) matrix-exponential embedding.
            //
            // For a linear system x' = Ac*x + Bc*u with piecewise-constant input u,
            // the exact discrete-time equivalent is:
            //   x[k+1] = Ad*x[k] + Bd*u[k]
            //   Ad = expm(Ac*Ts),   Bd = (integral_0^Ts expm(Ac*t) dt) * Bc
            //
            // Both Ad and Bd are extracted in one shot from the (n+m)*(n+m) augmented
            // matrix exponential (Van Loan 1978, Eq. 2.4):
            //   expm([Ac  Bc] * Ts) = [Ad  Bd]
            //       ([0   0 ]      )   [0   I ]
            //
            // This is numerically preferred over Pade-approximant series for stiff Ac
            // (large spread of eigenvalues): the matrix exponential is computed via
            // Schur decomposition, which remains stable even when eigenvalues span many
            // decades (e.g., the Bell-Astrom boiler model where |lambda_fast|/|lambda_slow|
            // >> 100).  A forward-Euler or Pade approximant at such stiffness ratios
            // produces discrete eigenvalues outside the unit disk.
            //
            // Reference: Van Loan, C.F. (1978). "Computing integrals involving the matrix
            //   exponential." IEEE Trans. Automat. Control, 23(3):395-404.
            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n + m, n + m);
            M.topLeftCorner(n, n)  = sys_c.A * Ts;
            M.topRightCorner(n, m) = sys_c.B * Ts;
            const Eigen::MatrixXd eM = M.exp();

            const Eigen::MatrixXd Ad = eM.topLeftCorner(n, n);
            const Eigen::MatrixXd Bd = eM.topRightCorner(n, m);
            StateSpace result(Ad, Bd, sys_c.C, sys_c.D, Ts);
            checkDiscreteStability(result.A, "c2d (ZOH)");
            return result;
        }
        else // Tustin or TustinPrewarped
        {
            // Standard Tustin:  s = (2/Ts) * (z-1)/(z+1)  ->  alpha = Ts/2
            //
            // Prewarped Tustin: s = k_p * (z-1)/(z+1)
            //   k_p = prewarp_freq / tan(prewarp_freq * Ts / 2)
            //   Ensures |G_d(e^{j*prewarp_freq*Ts})| = |G_c(j*prewarp_freq)|  exactly.
            //   When prewarp_freq = 0 or method == Tustin: k_p = 2/Ts (standard).
            //
            // In both cases: alpha = 1/k_p, and the formulas are identical.
            double alpha;
            if (method == C2dMethod::TustinPrewarped && prewarp_freq > 0.0)
            {
                const double tan_half = std::tan(prewarp_freq * Ts * 0.5);
                if (tan_half < 1e-300)
                    throw std::invalid_argument("c2d TustinPrewarped: prewarp_freq * Ts / 2 too small.");
                alpha = tan_half / prewarp_freq; // = 1/k_p
            }
            else
            {
                alpha = Ts * 0.5; // standard Tustin: alpha = Ts/2
            }

            const Eigen::MatrixXd Im = Eigen::MatrixXd::Identity(n, n) - alpha * sys_c.A;
            const Eigen::MatrixXd Ip = Eigen::MatrixXd::Identity(n, n) + alpha * sys_c.A;
            const Eigen::PartialPivLU<Eigen::MatrixXd> lu(Im);
            const Eigen::MatrixXd Ad = lu.solve(Ip);
            const Eigen::MatrixXd Bd = lu.solve(sys_c.B * (2.0 * alpha)); // Bd = (2 alpha)*(I-alpha*A)^{-1}*B
            StateSpace result(Ad, Bd, sys_c.C, sys_c.D, Ts);
            checkDiscreteStability(result.A, method == C2dMethod::TustinPrewarped
                                             ? "c2d (TustinPrewarped)" : "c2d (Tustin)");
            return result;
        }
    }

    // ---------------------------------------------------------------------------
    // minreal - minimal realization via balanced truncation (Ho-Kalman)
    // ---------------------------------------------------------------------------
    StateSpace minreal(const StateSpace &sys, double tol)
    {
        const int n = sys.stateSize();
        const int m = sys.inputSize();
        const int p = sys.outputSize();

        if (n == 0) return sys;

        Eigen::MatrixXd Co(n, n * m);
        Eigen::MatrixXd Ob(n * p, n);
        
        Eigen::MatrixXd Apow = Eigen::MatrixXd::Identity(n, n);
        for (int i = 0; i < n; ++i) {
            Co.middleCols(i * m, m) = Apow * sys.B;
            Ob.middleRows(i * p, p) = sys.C * Apow;
            Apow = sys.A * Apow;
        }

        Eigen::MatrixXd H = Ob * Co;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        
        int r = 0;
        for (int i = 0; i < svd.singularValues().size(); ++i) {
            if (svd.singularValues()(i) > tol) r++;
        }

        if (r == n) return sys; // Already minimal
        if (r == 0) {
            // Completely unobservable/uncontrollable, return 0 state system
            return StateSpace(Eigen::MatrixXd::Zero(0, 0), Eigen::MatrixXd::Zero(0, m), Eigen::MatrixXd::Zero(p, 0), sys.D, sys.Ts);
        }

        Eigen::MatrixXd S_inv_sqrt = svd.singularValues().head(r).cwiseInverse().cwiseSqrt().asDiagonal();
        Eigen::MatrixXd Ur = svd.matrixU().leftCols(r);
        Eigen::MatrixXd Vr = svd.matrixV().leftCols(r);

        Eigen::MatrixXd H_shift = Ob * sys.A * Co;

        Eigen::MatrixXd A_min = S_inv_sqrt * Ur.transpose() * H_shift * Vr * S_inv_sqrt;
        Eigen::MatrixXd B_min = S_inv_sqrt * Ur.transpose() * H.leftCols(m);
        Eigen::MatrixXd C_min = H.topRows(p) * Vr * S_inv_sqrt;

        return StateSpace(A_min, B_min, C_min, sys.D, sys.Ts);
    }

} // namespace ctrl
