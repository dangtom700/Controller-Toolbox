#include "LPVSystemID.h"
#include "SubspaceID.h"
#include <stdexcept>
#include <Eigen/Dense>

namespace ctrl {

LPVModel identifyLPV(const Eigen::MatrixXd&    X,
                     const Eigen::MatrixXd&    U,
                     const Eigen::MatrixXd&    Y,
                     const std::vector<double>& sched,
                     int    degree,
                     double Ts)
{
    const int n     = static_cast<int>(X.rows()); // state dimension
    const int m     = static_cast<int>(U.rows()); // input dimension
    const int p_out = static_cast<int>(Y.rows()); // output dimension
    const int N     = static_cast<int>(X.cols()); // time steps

    if (N < 2)
        throw std::invalid_argument("identifyLPV: need at least 2 time steps.");
    if (static_cast<int>(U.cols()) != N || static_cast<int>(Y.cols()) != N)
        throw std::invalid_argument("identifyLPV: X, U, Y must have the same number of columns.");
    if (static_cast<int>(sched.size()) < N)
        throw std::invalid_argument("identifyLPV: sched.size() must be >= N.");
    if (degree < 0)
        throw std::invalid_argument("identifyLPV: degree must be >= 0.");

    const int d  = degree;
    const int d1 = d + 1;

    // -------------------------------------------------------------------------
    // State equation regression
    //   x[k+1] = A(p[k]) x[k] + B(p[k]) u[k]
    //
    // Row regressor phi_k = [x, p*x, ..., p^d*x | u, p*u, ..., p^d*u]
    //                         n*(d+1) terms           m*(d+1) terms
    //
    // Stacked system: Phi [(N-1) x (n+m)*d1] * Theta_AB [(n+m)*d1 x n] = Z_x
    // Solution: Theta_AB = QR-pinv(Phi) * Z_x
    //
    // Theta_AB row layout:
    //   rows 0..n-1         -> A0^T  (A_coeffs[0] = rows.T)
    //   rows n..2n-1        -> A1^T
    //   ...
    //   rows d*n..(d+1)*n-1 -> Ad^T
    //   rows (d+1)*n .. (d+1)*n+m-1  -> B0^T
    //   ...
    // -------------------------------------------------------------------------
    const int T_x = N - 1;
    const int cols_AB = (n + m) * d1;
    Eigen::MatrixXd Phi_AB(T_x, cols_AB);
    Eigen::MatrixXd Z_x(T_x, n);

    for (int k = 0; k < T_x; ++k) {
        const double p = sched[k];
        int col = 0;

        // A-part: p^i * x[k] for i = 0..d
        double pk = 1.0;
        for (int i = 0; i <= d; ++i, pk *= p) {
            Phi_AB.block(k, col, 1, n) = pk * X.col(k).transpose();
            col += n;
        }

        // B-part: p^i * u[k] for i = 0..d
        pk = 1.0;
        for (int i = 0; i <= d; ++i, pk *= p) {
            Phi_AB.block(k, col, 1, m) = pk * U.col(k).transpose();
            col += m;
        }

        Z_x.row(k) = X.col(k + 1).transpose();
    }

    // Solve via column-pivot QR (handles near-rank-deficiency when p is narrow)
    Eigen::MatrixXd Theta_AB = Phi_AB.colPivHouseholderQr().solve(Z_x);

    // -------------------------------------------------------------------------
    // Output equation regression
    //   y[k] = C(p[k]) x[k] + D(p[k]) u[k]
    // Same regressor structure over all N steps.
    // -------------------------------------------------------------------------
    const int cols_CD = (n + m) * d1;
    Eigen::MatrixXd Phi_CD(N, cols_CD);
    Eigen::MatrixXd Z_y(N, p_out);

    for (int k = 0; k < N; ++k) {
        const double p = sched[k];
        int col = 0;

        double pk = 1.0;
        for (int i = 0; i <= d; ++i, pk *= p) {
            Phi_CD.block(k, col, 1, n) = pk * X.col(k).transpose();
            col += n;
        }

        pk = 1.0;
        for (int i = 0; i <= d; ++i, pk *= p) {
            Phi_CD.block(k, col, 1, m) = pk * U.col(k).transpose();
            col += m;
        }

        Z_y.row(k) = Y.col(k).transpose();
    }

    Eigen::MatrixXd Theta_CD = Phi_CD.colPivHouseholderQr().solve(Z_y);

    // -------------------------------------------------------------------------
    // Extract polynomial coefficient matrices from Theta
    //   A_i = Theta_AB[i*n : (i+1)*n, :].transpose()   (shape n*n)
    //   B_i = Theta_AB[(d+1)*n + i*m : ..., :].transpose()  (shape n*m)
    //   C_i = Theta_CD[i*n : (i+1)*n, :].transpose()   (shape p_out*n)
    //   D_i = Theta_CD[(d+1)*n + i*m : ..., :].transpose()  (shape p_out*m)
    // -------------------------------------------------------------------------
    LPVModel model;
    model.degree    = d;
    model.n_states  = n;
    model.n_inputs  = m;
    model.n_outputs = p_out;
    model.Ts        = Ts;
    model.A_coeffs.resize(d1);
    model.B_coeffs.resize(d1);
    model.C_coeffs.resize(d1);
    model.D_coeffs.resize(d1);

    for (int i = 0; i <= d; ++i) {
        model.A_coeffs[i] = Theta_AB.block(i * n,             0, n, n).transpose();
        model.B_coeffs[i] = Theta_AB.block((d + 1) * n + i * m, 0, m, n).transpose();
        model.C_coeffs[i] = Theta_CD.block(i * n,             0, n, p_out).transpose();
        model.D_coeffs[i] = Theta_CD.block((d + 1) * n + i * m, 0, m, p_out).transpose();
    }

    return model;
}

LPVModel identifyLPVFromIO(const Eigen::MatrixXd&    U,
                           const Eigen::MatrixXd&    Y,
                           const std::vector<double>& sched,
                           int    n_states,
                           int    degree,
                           double Ts,
                           int    n4sid_i)
{
    const int N = static_cast<int>(U.cols());

    // Estimate state sequence using n4sid on the entire dataset.
    // n4sid gives a linear model valid near the average operating point;
    // we use it only to obtain state estimates for the LPV regression.
    SubspaceIDResult id = n4sid(Y, U, n_states, n4sid_i, Ts);

    if (!id.model.has_value())
        throw std::runtime_error("identifyLPVFromIO: n4sid failed to identify a model.");

    const ctrl::StateSpace& lti = id.model.value();

    // Propagate state sequence forward under the identified LTI model.
    Eigen::MatrixXd X(n_states, N);
    X.col(0).setZero();
    for (int k = 0; k < N - 1; ++k)
        X.col(k + 1) = lti.A * X.col(k) + lti.B * U.col(k);

    return identifyLPV(X, U, Y, sched, degree, Ts);
}

} // namespace ctrl
