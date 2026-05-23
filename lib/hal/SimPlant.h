#pragma once
#include "../PlantModel.h"
#include <Eigen/Dense>

// SimPlant - discrete-time plant simulator shared by SimSensor and SimActuator.
//
// Owns the state-space model and its current state vector.  Both simulation
// adapters hold a reference to the same SimPlant instance so that sensor
// readings and actuator commands remain consistent within one sample step.
//
// Typical closed-loop simulation pattern (one step k):
//
//   double y = sensor.read();        // 1. read current output y[k]
//   double u = controller.compute(r - y); // 2. compute control action
//   actuator.write(u);               // 3. write u[k] -> advances plant to x[k+1]
//
// The step order matters: read() must be called BEFORE write() in the same
// sample because write() advances the state.  If the step order is reversed
// the simulation models a one-step computational delay (which is also valid
// for hardware - just document the convention and stick to it).
//
// Thread safety: not thread-safe.  Use one SimPlant per thread, or guard
// access with a mutex when stepping from a background thread.
namespace ctrl {

class SimPlant {
public:
    // Construct with a discrete-time state-space model.
    // x0 is optional; defaults to the zero state (plant starts at rest).
    explicit SimPlant(const StateSpace&       model,
                      const Eigen::VectorXd&  x0 = Eigen::VectorXd());

    // Advance one sample: x[k+1] = A.x[k] + B.u[k],  y[k] = C.x[k] + D.u[k].
    // The output y[k] is cached internally; call output() after step() to read it.
    void step(double u);

    // Returns the cached output from the last step() call.
    // On construction (before any step), returns C.x0 + D.0.
    double output() const { return y_cached_; }

    // Direct state access - useful for injecting Kalman estimates or
    // for initialising the plant at a non-zero operating point.
    const Eigen::VectorXd& state()                    const { return x_; }
    void                   setState(const Eigen::VectorXd& x);

    // Reset to the initial state (zero unless overridden by setState before reset).
    void reset();

    const StateSpace& model() const { return model_; }

private:
    StateSpace       model_;
    Eigen::VectorXd  x_;           // current state x[k]
    Eigen::VectorXd  x0_;          // initial state (stored for reset)
    double           y_cached_;    // last output - avoids recomputing in output()

    void updateOutput(double u);   // recomputes y_cached_ from current x_ and u
};

} // namespace ctrl
