#include "PlantModel.h"
#include <algorithm>
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
            // Embed [Ac Bc; 0 0] * Ts and take matrix exponential.
            // expm([Ac Bc; 0 0]*Ts) = [Ad Bd; 0 I]
            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n + m, n + m);
            M.topLeftCorner(n, n)  = sys_c.A * Ts;
            M.topRightCorner(n, m) = sys_c.B * Ts;
            const Eigen::MatrixXd eM = M.exp();

            const Eigen::MatrixXd Ad = eM.topLeftCorner(n, n);
            const Eigen::MatrixXd Bd = eM.topRightCorner(n, m);
            return StateSpace(Ad, Bd, sys_c.C, sys_c.D, Ts);
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
            return StateSpace(Ad, Bd, sys_c.C, sys_c.D, Ts);
        }
    }

} // namespace ctrl
