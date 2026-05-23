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
    // Discrete-time transfer function H(z^-¹) = (b₀ + b₁z^-¹ + ... + bₘz^-ᵐ)
    //                                         / (1  + a₁z^-¹ + ... + aₙz^-ⁿ)
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
            if (den.empty() || den[0] == 0.0)
                throw std::invalid_argument("TransferFunction: denominator must be monic (den[0]=1).");
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
        }

        int stateSize() const { return static_cast<int>(A.rows()); }
        int inputSize() const { return static_cast<int>(B.cols()); }
        int outputSize() const { return static_cast<int>(C.rows()); }
    };

    // ---------------------------------------------------------------------------
    // Convert SISO discrete transfer function -> state-space (controllable canonical form).
    // Equivalent MATLAB:  [A,B,C,D] = tf2ss(num, den)  applied to the z^-¹ polynomial.
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

} // namespace ctrl
