#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "IController.h"
#include "IControllerObserver.h"

namespace py = pybind11;

// ---------------------------------------------------------------------------
// PyIController
//
// Trampoline that forwards pure-virtual calls from C++ to Python overrides.
// Bind IController as py::class_<ctrl::IController, PyIController>(m, "IController")
// so Python code can subclass it:
//
//   class MyController(ctrl.IController):
//       def compute(self, signal: float) -> float:
//           return -2.0 * signal
//       def reset(self): pass
//       def sample_time(self) -> float: return 0.01
// ---------------------------------------------------------------------------
class PyIController : public ctrl::IController
{
public:
    using ctrl::IController::IController;

    double compute(double signal) override
    {
        PYBIND11_OVERRIDE_PURE(double, ctrl::IController, compute, signal);
    }

    void reset() override
    {
        PYBIND11_OVERRIDE_PURE(void, ctrl::IController, reset);
    }

    double sampleTime() const override
    {
        PYBIND11_OVERRIDE_PURE(double, ctrl::IController, sampleTime);
    }

    Eigen::VectorXd computeVec(const Eigen::VectorXd &signal) override
    {
        PYBIND11_OVERRIDE(Eigen::VectorXd, ctrl::IController, computeVec, signal);
    }

    void bumplessInit(double u_target, double error) override
    {
        PYBIND11_OVERRIDE(void, ctrl::IController, bumplessInit, u_target, error);
    }

    bool isHealthy() const override
    {
        PYBIND11_OVERRIDE(bool, ctrl::IController, isHealthy);
    }
};

// ---------------------------------------------------------------------------
// PyIControllerObserver
//
// Trampoline for IControllerObserver, enabling Python subclassing:
//
//   class Logger(ctrl.IControllerObserver):
//       def on_compute(self, u: float, signal: float): print(f"u={u:.4f}")
//       def on_reset(self): print("reset")
//
// Attach via:
//   obs = Logger()
//   pid.attach_observer(obs)    # shared_ptr overload keeps obs alive
// ---------------------------------------------------------------------------
class PyIControllerObserver : public ctrl::IControllerObserver
{
public:
    using ctrl::IControllerObserver::IControllerObserver;

    void onCompute(double u, double signal) override
    {
        // Python name is "on_compute" (snake_case); use OVERRIDE_NAME to bridge.
        PYBIND11_OVERRIDE_NAME(void, ctrl::IControllerObserver, "on_compute", onCompute, u, signal);
    }

    void onComputeVec(const Eigen::VectorXd &u, const Eigen::VectorXd &signal) override
    {
        PYBIND11_OVERRIDE_NAME(void, ctrl::IControllerObserver, "on_compute_vec", onComputeVec, u, signal);
    }

    void onReset() override
    {
        PYBIND11_OVERRIDE_NAME(void, ctrl::IControllerObserver, "on_reset", onReset);
    }

    void onState(std::string_view key, const Eigen::VectorXd& value) override
    {
        PYBIND11_OVERRIDE_NAME(void, ctrl::IControllerObserver, "on_state", onState,
                               std::string(key), value);
    }
};
