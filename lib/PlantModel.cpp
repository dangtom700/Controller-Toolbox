#include "PlantModel.h"
#include <algorithm>

namespace ctrl
{

    // ---------------------------------------------------------------------------
    // tf2ss - controllable canonical form
    //
    // For H(z^-¹) = (b₀ + b₁z^-¹ + ... + bₙz^-ⁿ) / (1 + a₁z^-¹ + ... + aₙz^-ⁿ):
    //
    //   A = [[-a₁, -a₂, ..., -aₙ],   (companion row)
    //        [ 1,   0,  ...,  0  ],
    //        ...
    //        [ 0,   0,  ...,  0  ]]   (n*n)
    //   B = [[1], [0], ..., [0]]      (n*1)
    //   C = [[b₁-a₁b₀, b₂-a₂b₀, ..., bₙ-aₙb₀]]  (1*n)
    //   D = [[b₀]]                  (1*1)
    //
    // Equivalent MATLAB:  [A,B,C,D] = tf2ss(num,den) in z^-¹ convention.
    // ---------------------------------------------------------------------------
    StateSpace tf2ss(const TransferFunction &tf)
    {
        int n = tf.order(); // state dimension = denominator degree

        // Pad numerator to exactly n+1 coefficients (prepend zeros if shorter).
        std::vector<double> num = tf.num;
        while (static_cast<int>(num.size()) < n + 1)
            num.insert(num.begin(), 0.0);

        double d0 = num[0]; // feed-through / direct term = b₀

        // Build A (n*n companion matrix, first row = -a₁...-aₙ, sub-diagonal = 1).
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
        for (int j = 0; j < n; ++j)
            A(0, j) = -tf.den[j + 1];
        for (int i = 1; i < n; ++i)
            A(i, i - 1) = 1.0;

        // B = [1, 0, ..., 0]'
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, 1);
        B(0, 0) = 1.0;

        // C = [b₁-a₁b₀, ..., bₙ-aₙb₀]  (long-division remainder numerator)
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

} // namespace ctrl
