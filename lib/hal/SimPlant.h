#pragma once
#include "../PlantModel.h"
#include <Eigen/Dense>
#include <mutex>

/**
 * @file SimPlant.h
 * @brief Discrete-time plant simulator shared by SimSensor and SimActuator.
 *
 * Owns the state-space model and its current state vector. Both simulation adapters
 * hold a reference to the same SimPlant so that sensor readings and actuator commands
 * remain consistent within one sample step.
 *
 * **Typical closed-loop simulation pattern (one step k):**
 * @code
 *   double y = sensor.read();              // 1. read current output y[k]
 *   double u = controller.compute(r - y);  // 2. compute control action
 *   actuator.write(u);                     // 3. write u[k] -> advances plant to x[k+1]
 * @endcode
 *
 * `read()` must be called **before** `write()` in the same sample, because `write()`
 * advances the state. Reversing the order models a one-step computational delay -
 * which is valid for hardware; just document the convention.
 *
 * **Thread safety:** `step()`, `setState()`, `reset()`, and `output()` are all
 * protected by an internal `std::mutex`. This enables Hardware-in-the-Loop (HIL)
 * setups where the plant runs in a background thread while a logger reads `output()`
 * concurrently.
 *
 * @note A read-then-step sequence is NOT atomic without an external lock. For
 *       simulation-only use from a single thread the mutex overhead is negligible.
 */

namespace ctrl {

/**
 * @brief Thread-safe discrete-time plant simulator.
 */
class SimPlant {
public:
    /**
     * @brief Construct with a discrete-time state-space model.
     * @param model  State-space model (A, B, C, D, Ts).
     * @param x0     Initial state vector. Defaults to the zero state (plant at rest).
     */
    explicit SimPlant(const StateSpace&       model,
                      const Eigen::VectorXd&  x0 = Eigen::VectorXd());

    /**
     * @brief Advance one sample: x[k+1] = A.x[k] + B.u[k], y[k] = C.x[k] + D.u[k].
     *
     * The output y[k] is cached internally; call output() after step() to read it.
     *
     * @param u Control input for this step.
     */
    void step(double u);

    /**
     * @brief Return the cached output from the last step() call.
     *
     * On construction (before any step), returns C.x0 + D.0.
     *
     * @return y[k] (mutex-protected).
     */
    double output() const;

    /**
     * @brief Read-only access to the current state x[k].
     * @return Copy of the state vector (mutex-protected).
     */
    Eigen::VectorXd state() const {
        std::lock_guard<std::mutex> lk(mu_);
        return x_;
    }

    /**
     * @brief Inject a state vector (for Kalman initialisation or non-zero operating point).
     * @param x New state vector.
     */
    void setState(const Eigen::VectorXd& x);

    /**
     * @brief Reset the plant to the initial state (zero unless overridden by setState before reset()).
     */
    void reset();

    /** @brief Read-only access to the state-space model. */
    const StateSpace& model() const { return model_; }

private:
    mutable std::mutex mu_;     ///< Guards x_ and y_cached_.
    StateSpace       model_;
    Eigen::VectorXd  x_;        ///< Current state x[k].
    Eigen::VectorXd  x0_;       ///< Initial state (stored for reset()).
    double           y_cached_; ///< Last output - avoids recomputing in output().

    void updateOutput(double u); ///< Recomputes y_cached_ from current x_ and u (caller holds mu_).
};

} // namespace ctrl
