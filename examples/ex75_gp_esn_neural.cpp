/**
 * @file ex75_gp_esn_neural.cpp
 * @brief Gaussian Process, Echo State Network, and NeuralPID demos.
 *
 * Part 1: GP regression on a noisy sine curve.
 * Part 2: ESN identifies a first-order nonlinear plant.
 * Part 3: NeuralPID adapts online to gain changes on a first-order plant.
 * Part 4: CEM-MPC vs PID on a double integrator.
 */

#include "ControllerToolbox.h"
#include "GaussianProcess.h"
#include "EchoStateNetwork.h"
#include "NeuralPID.h"
#include "CEMController.h"
#include <cmath>
#include <cstdio>

// ---- Part 1: GP regression -------------------------------------------------
static void runGP()
{
    std::printf("\n--- Part 1: Gaussian Process Regression ---\n");
    ctrl::GaussianProcess::Params gp;
    gp.length_scale = 1.5;
    gp.signal_var   = 1.0;
    gp.noise_var    = 0.01;
    gp.n_max        = 80;
    ctrl::GaussianProcess gpr(1, gp);

    // Training data: y = sin(2x) + noise
    for (int k = 0; k < 50; ++k) {
        double xv = k * 0.2 - 5.0;
        double yv = std::sin(2.0 * xv) + 0.1 * ((k % 3) - 1);
        Eigen::VectorXd xvec(1); xvec << xv;
        gpr.addPoint(xvec, yv);
    }
    gpr.fit();

    std::printf("  x     true y   GP mean   GP std\n");
    for (int k = 0; k < 5; ++k) {
        double xv = k * 0.5 - 1.0;
        Eigen::VectorXd xvec(1); xvec << xv;
        auto [mu, var] = gpr.predict(xvec);
        std::printf("  %5.2f  %7.4f   %7.4f  %6.4f\n",
                    xv, std::sin(2.0 * xv), mu, std::sqrt(var));
    }
}

// ---- Part 2: ESN identification --------------------------------------------
static void runESN()
{
    std::printf("\n--- Part 2: Echo State Network Identification ---\n");
    ctrl::EchoStateNetwork::Params ep;
    ep.n_res = 50; ep.n_in = 1; ep.n_out = 1;
    ep.spectral_radius = 0.85; ep.sparsity = 0.8;
    ep.washout = 30; ep.ridge = 1e-3;
    ctrl::EchoStateNetwork esn(ep);

    // Plant: y[k] = tanh(0.8*y[k-1] + 0.5*u[k-1])
    double y_prev = 0.0;
    for (int k = 0; k < 300; ++k) {
        double u  = (k % 3 == 0) ? 1.0 : -0.5;
        double y  = std::tanh(0.8 * y_prev + 0.5 * u);
        Eigen::VectorXd uv(1), yv(1); uv << u; yv << y;
        esn.stepReservoir(uv);
        esn.addTrainingTarget(yv);
        y_prev = y;
    }
    esn.fitReadout();
    std::printf("  ESN fitted (%d reservoir units)\n", esn.reservoirSize());

    // Predict test sequence
    y_prev = 0.0;
    esn.reset();  // reset reservoir state
    double mse = 0.0;
    for (int k = 0; k < 30; ++k) {
        double u   = (k % 4 == 0) ? 0.8 : -0.3;
        double y_t = std::tanh(0.8 * y_prev + 0.5 * u);
        Eigen::VectorXd uv(1); uv << u;
        double y_hat = esn.predict(uv)(0);
        mse += (y_t - y_hat) * (y_t - y_hat);
        y_prev = y_t;
    }
    std::printf("  Prediction MSE: %.5f (< 0.05 expected)\n", mse / 30);
}

// ---- Part 3: Neural PID ----------------------------------------------------
static void runNeuralPID()
{
    std::printf("\n--- Part 3: NeuralPID online adaptation ---\n");
    ctrl::NeuralPID::Params np;
    np.n_hidden   = 8;
    np.lr         = 5e-4;
    np.Ts         = 0.01;
    np.plant_gain = 0.2;
    np.Kp0 = 2.0; np.Ki0 = 0.2; np.Kd0 = 0.0;
    np.uMin = -3.0; np.uMax = 3.0;
    ctrl::NeuralPID npid(np);

    constexpr int N = 400;
    constexpr double r_val = 1.0;
    double x = 0.0;
    double iae = 0.0;

    for (int k = 0; k < N; ++k) {
        // Plant: gain changes at k=200
        double b = (k < 200) ? 0.2 : 0.4;
        double u = npid.compute(r_val - x);
        x = 0.8 * x + b * u;
        iae += std::abs(r_val - x) * np.Ts;
    }
    std::printf("  Final Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                npid.currentKp(), npid.currentKi(), npid.currentKd());
    std::printf("  Total IAE: %.4f\n", iae);
}

// ---- Part 4: CEM-MPC -------------------------------------------------------
static void runCEM()
{
    std::printf("\n--- Part 4: CEM-MPC on double integrator ---\n");
    constexpr double Ts = 0.05;

    // Discrete double integrator
    Eigen::Matrix2d A; A << 1, Ts, 0, 1;
    Eigen::Vector2d B; B << 0.5*Ts*Ts, Ts;
    Eigen::MatrixXd C(1, 2); C << 1.0, 0.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    ctrl::StateSpace ss(A, B, C, D, Ts);

    auto f = [ss](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        return ss.A * x + ss.B * u;
    };

    ctrl::CEMController::Params cp;
    cp.Np = 20; cp.N_samples = 80; cp.n_iter = 4;
    cp.Q = 100.0; cp.R = 0.1;
    cp.uMin = -2.0; cp.uMax = 2.0;
    cp.sigma_init = 0.5;
    ctrl::CEMController cem(cp, f, C, Ts);

    Eigen::Vector2d x_state; x_state.setZero();
    Eigen::VectorXd y_ref(1); y_ref << 1.0;
    double iae_cem = 0.0;

    for (int k = 0; k < 100; ++k) {
        cem.setState(x_state.cast<double>());
        cem.setReference(y_ref);
        auto u_vec = cem.computeRef(x_state.cast<double>(), y_ref);
        x_state = A * x_state + B * u_vec(0);
        iae_cem += std::abs(1.0 - x_state(0)) * Ts;
    }
    std::printf("  CEM-MPC IAE: %.4f,  final pos: %.4f\n", iae_cem, x_state(0));
}

int main()
{
    std::printf("=== ex75: GP / ESN / NeuralPID / CEM-MPC ===");
    runGP();
    runESN();
    runNeuralPID();
    runCEM();
    std::printf("\n");
    return 0;
}
