#pragma once
#include <vector>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>

// Discrete-time plant model representations and utilities.
// Ref: MATLAB tf(), ss(), tf2ss(), c2d() documentation.
namespace ctrl
{

    // ---------------------------------------------------------------------------
    // Discrete-time transfer function H(z^-^1) = (b0 + b1z^-^1 + ... + b_mz^-^m)
    //                                         / (1  + a1z^-^1 + ... + a_nz^-^n)
    //
    // num = {b0, b1, ..., bm}  - numerator coefficients
    // den = {1,  a1, ..., an}  - monic denominator (den[0] must equal 1)
    //
    // Equivalent MATLAB:  G = tf(num, den, Ts, 'Variable', 'z^-1')
    // ---------------------------------------------------------------------------
    struct TransferFunction
    {
        std::vector<double> num; // numerator  [b0, b1, ..., bm]
        std::vector<double> den; // denominator [1,  a1, ..., an], den[0] == 1

        double Ts; // sample time (s)

        TransferFunction(std::vector<double> numerator,
                         std::vector<double> denominator,
                         double sampleTime)
            : num(std::move(numerator)), den(std::move(denominator)), Ts(sampleTime)
        {
            if (den.empty() || std::abs(den[0] - 1.0) > 1e-9)
                throw std::invalid_argument("TransferFunction: denominator must be monic (den[0] = 1).");
        }

        int order() const { return static_cast<int>(den.size()) - 1; }
    };

    // ---------------------------------------------------------------------------
    // Discrete-time state-space model
    //   x[k+1] = A.x[k] + B.u[k]
    //   y[k]   = C.x[k] + D.u[k]
    //
    // Equivalent MATLAB:  sys = ss(A, B, C, D, Ts)
    // ---------------------------------------------------------------------------
    struct StateSpace
    {
        Eigen::MatrixXd A; // n * n state-transition matrix
        Eigen::MatrixXd B; // n * m input matrix
        Eigen::MatrixXd C; // p * n output matrix
        Eigen::MatrixXd D; // p * m feedthrough matrix
        double Ts;

        StateSpace(Eigen::MatrixXd a,
                   Eigen::MatrixXd b,
                   Eigen::MatrixXd c,
                   Eigen::MatrixXd d,
                   double sampleTime)
            : A(std::move(a)), B(std::move(b)), C(std::move(c)), D(std::move(d)), Ts(sampleTime)
        {
            validate();
        }

        int stateSize() const { return static_cast<int>(A.rows()); }
        int inputSize() const { return static_cast<int>(B.cols()); }
        int outputSize() const { return static_cast<int>(C.rows()); }

        // Throw std::invalid_argument if any matrix dimension is inconsistent.
        // Called automatically by the constructor; call manually after modifying matrices directly.
        void validate() const;
    };

    // ---------------------------------------------------------------------------
    // Convert SISO discrete transfer function -> state-space (controllable canonical form).
    // Equivalent MATLAB:  [A,B,C,D] = tf2ss(num, den)  applied to the z^-^1 polynomial.
    // Ref: Ogata "Modern Control Engineering", MATLAB tf2ss documentation.
    // ---------------------------------------------------------------------------
    StateSpace tf2ss(const TransferFunction &tf);

    // ---------------------------------------------------------------------------
    // Simulate one step of a state-space model (in-place state update).
    //   1. Compute y[k] = C.x[k] + D.u[k]
    //   2. Advance  x[k+1] = A.x[k] + B.u[k]
    // Returns y[k].  x is updated in-place to x[k+1].
    // ---------------------------------------------------------------------------
    // x accepts both fixed-size (Vector2d) and dynamic (VectorXd) columns;
    // Eigen::Ref handles the binding and propagates the in-place x[k+1] update.
    Eigen::VectorXd ssStep(const StateSpace &sys,
                           Eigen::Ref<Eigen::VectorXd> x,
                           const Eigen::VectorXd &u);

    // ---------------------------------------------------------------------------
    // c2d - discretise a continuous-time state-space model.
    //
    // The input StateSpace must carry Ts == 0.0 to indicate continuous time.
    // Pass the desired sample period as the second argument.
    //
    // Methods:
    //   ZOH    - zero-order hold (exact for piecewise-constant inputs).
    //            Uses the matrix-exponential embedding:
    //              M = expm([Ac Bc; 0 0] * Ts)  ->  Ad = M[:n,:n], Bd = M[:n,n:]
    //            Requires <unsupported/Eigen/MatrixFunctions>.
    //
    //   Tustin - bilinear (Tustin) transform, s = (2/Ts)*(z-1)/(z+1).
    //            Ad = (I - alpha.Ac)^{-1}.(I + alpha.Ac)     alpha = Ts/2
    //            Bd = Ts.(I - alpha.Ac)^{-1}.Bc
    //            Preserves stability but distorts frequency axis.
    //
    // Cd and Dd are unchanged for both methods.
    //
    // Equivalent MATLAB: c2d(sys_c, Ts, 'zoh')  /  c2d(sys_c, Ts, 'tustin')
    // ---------------------------------------------------------------------------
    // ---------------------------------------------------------------------------
    // ss2tf - convert SISO discrete state-space model -> transfer function.
    //
    // Computes the denominator from the characteristic polynomial of A (via
    // Faddeev-LeVerrier / Leverrier's algorithm) and the numerator from the
    // Markov parameters h[0]=D, h[k]=C.A^{k-1}.B (k>=1):
    //
    //   b[k] = Sigma_{j=0}^{k} a[j] . h[k-j]   (a[0]=1, a[1..n] from char. poly)
    //
    // Only valid for SISO systems (stateSize() >= 1, inputSize() == outputSize() == 1).
    // Throws std::invalid_argument for non-SISO plants.
    //
    // Equivalent MATLAB:  tf(sys)  where sys is a discrete ss object.
    // ---------------------------------------------------------------------------
    TransferFunction ss2tf(const StateSpace &sys);

    // ---------------------------------------------------------------------------
    // c2d discretisation methods.
    //
    //   ZOH              - zero-order hold (exact for piecewise-constant inputs).
    //   Tustin           - bilinear transform  s = (2/Ts)(z-1)/(z+1).
    //   TustinPrewarped  - bilinear transform prewarped at prewarp_freq [rad/s]
    //                      so that the discrete frequency response matches the
    //                      continuous response exactly at that frequency.
    //                      Set prewarp_freq to the crossover or resonant frequency.
    //                      Degenerates to standard Tustin when prewarp_freq = 0.
    // ---------------------------------------------------------------------------
    enum class C2dMethod { ZOH, Tustin, TustinPrewarped };

    StateSpace c2d(const StateSpace &sys_c, double Ts,
                   C2dMethod method = C2dMethod::ZOH,
                   double prewarp_freq = 0.0);

} // namespace ctrl
