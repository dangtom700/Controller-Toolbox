#pragma once
#include "IController.h"
#include "LinearisationHelper.h"
#include <Eigen/Dense>
#include <functional>
#include <stdexcept>
#include <string>

namespace ctrl {

/**
 * @brief Online Neural PID - small network that adapts [Kp, Ki, Kd] each step.
 *
 * A 3-layer fully-connected network f: [e, e_dot, e_int] -> [Kp, Ki, Kd] adapts
 * its weights online using the gradient of the squared tracking error through the
 * linearised plant (Jacobian provided by `LinearisationHelper`).
 *
 * **Architecture:**
 * @code
 *   Input layer:   z = [e[k], ė[k], \inte dt]    (3 features)
 *   Hidden layer:  h = tanh(W1 . z + b1)       (n_hidden units)
 *   Output layer:  [Kp, Ki, Kd] = softplus(W2 . h + b2)   (ensures positive gains)
 *
 *   u[k] = Kp.e + Ki.e_int + Kd.(e[k]-e[k-1])/Ts
 *
 *   Loss:  J = e[k]^2
 *   Gradient:  dJ/dtheta via backprop through [u->y] Jacobian + chain rule
 * @endcode
 *
 * Weight norms are bounded at each step (`max_weight_norm`) to prevent divergence.
 *
 * **Usage:**
 * @code
 *   ctrl::NeuralPID::Params p;
 *   p.n_hidden = 8; p.lr = 1e-3; p.Ts = 0.01;
 *   p.uMin = -1.0; p.uMax = 1.0;
 *   // Plant I/O model for gradient path  (simple gain estimate)
 *   p.plant_gain = 0.2;   // dy/du at current operating point
 *   ctrl::NeuralPID npid(p);
 *
 *   double u = npid.compute(r - y);   // standard error convention
 * @endcode
 *
 * @note This is a research/demonstration controller.  Use `DiscretePID` for
 *       production; use `NeuralPID` when you need online gain adaptation.
 */
class NeuralPID : public IController {
public:
    struct Params {
        int    n_hidden      = 8;      ///< Hidden layer width
        double lr            = 1e-3;   ///< Online learning rate
        double Ts            = 0.01;   ///< Sample time [s]
        double plant_gain    = 1.0;    ///< Approximate dy/du for gradient path
        double max_weight_norm = 5.0;  ///< Max Frobenius norm per weight matrix (clipping)
        double uMin          = -1e9;   ///< Output lower bound
        double uMax          =  1e9;   ///< Output upper bound

        // Initial gain values (network output at initialisation)
        double Kp0 = 1.0;
        double Ki0 = 0.1;
        double Kd0 = 0.0;
    };

    explicit NeuralPID(const Params& p);

    // IController interface
    double compute(double error) override;
    void   reset()               override;
    double sampleTime() const    override { return p_.Ts; }
    std::string name()  const    override { return "NeuralPID"; }

    // Diagnostics
    double currentKp() const noexcept { return Kp_; }
    double currentKi() const noexcept { return Ki_; }
    double currentKd() const noexcept { return Kd_; }

private:
    Params p_;

    // Network weights  (3->n_h->3)
    Eigen::MatrixXd W1_;  // n_h * 3
    Eigen::VectorXd b1_;  // n_h
    Eigen::MatrixXd W2_;  // 3 * n_h
    Eigen::VectorXd b2_;  // 3

    // Runtime state
    double e_prev_  = 0.0;
    double e_int_   = 0.0;
    double Kp_      = 1.0;
    double Ki_      = 0.1;
    double Kd_      = 0.0;
    double u_prev_  = 0.0;

    // Forward pass; returns gains and hidden activations
    Eigen::Vector3d forward(const Eigen::Vector3d& z,
                             Eigen::VectorXd& h_out) const;

    // Clip weight matrices to max_weight_norm
    void clipWeights();

    // softplus: log(1 + exp(x)) - numerically stable two-branch form
    // prevents exp() overflow for large positive pre-activations (x > ~710 overflows double)
    static double softplus(double x) {
        return x > 0.0 ? x + std::log1p(std::exp(-x)) : std::log1p(std::exp(x));
    }
};

} // namespace ctrl
