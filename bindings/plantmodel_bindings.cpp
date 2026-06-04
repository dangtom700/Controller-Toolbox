#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "ControllerToolbox.h"
#include "trampoline.h"
#include "ControllerMonitor.h"
#include "ControllerRegistry.h"

namespace py = pybind11;

void bind_plantmodel(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // TransferFunction
    // -----------------------------------------------------------------------
    py::class_<ctrl::TransferFunction>(m, "TransferFunction", R"doc(
Discrete-time SISO transfer function in z^-1 polynomial form.

    H(z^-1) = (b0 + b1*z^-1 + ... + bm*z^-m)
              -----------------------------------
              (1  + a1*z^-1 + ... + an*z^-n)

Parameters
----------
num : list[float]
    Numerator coefficients [b0, b1, ..., bm].
den : list[float]
    Monic denominator [1, a1, ..., an].  den[0] must equal 1.
Ts  : float
    Sample time [s].
)doc")
        .def(py::init<std::vector<double>, std::vector<double>, double>(),
             py::arg("num"), py::arg("den"), py::arg("Ts"))
        .def_readwrite("num", &ctrl::TransferFunction::num, "Numerator coefficients.")
        .def_readwrite("den", &ctrl::TransferFunction::den, "Monic denominator coefficients.")
        .def_readwrite("Ts",  &ctrl::TransferFunction::Ts,  "Sample time [s].")
        .def("order", &ctrl::TransferFunction::order, "Denominator order n.")
        .def("__repr__", [](const ctrl::TransferFunction &tf) {
            return "<TransferFunction num=" + std::to_string(tf.num.size()) +
                   " den=" + std::to_string(tf.den.size()) +
                   " Ts=" + std::to_string(tf.Ts) + ">";
        });

    // -----------------------------------------------------------------------
    // StateSpace
    // -----------------------------------------------------------------------
    py::class_<ctrl::StateSpace>(m, "StateSpace", R"doc(
Discrete-time state-space model.

    x[k+1] = A*x[k] + B*u[k]
    y[k]   = C*x[k] + D*u[k]

All matrix members are Eigen::MatrixXd and map bidirectionally with NumPy arrays.
)doc")
        .def(py::init<Eigen::MatrixXd, Eigen::MatrixXd,
                      Eigen::MatrixXd, Eigen::MatrixXd, double>(),
             py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"), py::arg("Ts"))
        .def_readwrite("A",  &ctrl::StateSpace::A,  "State-transition matrix (n*n).")
        .def_readwrite("B",  &ctrl::StateSpace::B,  "Input matrix (n*m).")
        .def_readwrite("C",  &ctrl::StateSpace::C,  "Output matrix (p*n).")
        .def_readwrite("D",  &ctrl::StateSpace::D,  "Feedthrough matrix (p*m).")
        .def_readwrite("Ts", &ctrl::StateSpace::Ts, "Sample time [s].")
        .def("state_size",  &ctrl::StateSpace::stateSize,  "Number of states n.")
        .def("input_size",  &ctrl::StateSpace::inputSize,  "Number of inputs m.")
        .def("output_size", &ctrl::StateSpace::outputSize, "Number of outputs p.")
        .def("validate",    &ctrl::StateSpace::validate,
             "Validate matrix dimension consistency.  Raises ValueError on mismatch.")
        .def("__repr__", [](const ctrl::StateSpace &s) {
            return "<StateSpace n=" + std::to_string(s.stateSize()) +
                   " m=" + std::to_string(s.inputSize()) +
                   " p=" + std::to_string(s.outputSize()) +
                   " Ts=" + std::to_string(s.Ts) + ">";
        });

    // -----------------------------------------------------------------------
    // DareResult
    // -----------------------------------------------------------------------
    py::class_<ctrl::DareResult>(m, "DareResult",
        "Result of a Discrete Algebraic Riccati Equation solve.")
        .def_readonly("P",          &ctrl::DareResult::P,
                      "Riccati solution matrix (n*n).")
        .def_readonly("converged",  &ctrl::DareResult::converged,
                      "True if the solver reached its convergence criterion.")
        .def_readonly("iterations", &ctrl::DareResult::iterations,
                      "Number of solver iterations performed.");

    // -----------------------------------------------------------------------
    // Free functions
    // -----------------------------------------------------------------------
    m.def("tf2ss", &ctrl::tf2ss, py::arg("tf"),
          "Convert a SISO discrete transfer function to controllable canonical state-space form.");

    m.def("ss_step", [](const ctrl::StateSpace &sys,
                        Eigen::Ref<Eigen::VectorXd> x,
                        const Eigen::VectorXd &u) {
              return ctrl::ssStep(sys, x, u);
          },
          py::arg("sys"), py::arg("x"), py::arg("u"),
          R"doc(
Advance one discrete step, updating x in-place.

Returns y[k] = C*x[k] + D*u[k] and sets x to x[k+1].
The NumPy array passed as x must be writable and will be modified.
Prefer ss_step_copy() when in-place mutation is undesirable.
)doc");

    m.def("ss_step_copy", &ctrl::ssStepCopy,
          py::arg("sys"), py::arg("x"), py::arg("u"),
          R"doc(
Non-mutating variant of ss_step.

Returns (y, x_next) without modifying the input x.
Safe to use with read-only or shared NumPy arrays.
)doc");

    // C2dMethod enum
    py::enum_<ctrl::C2dMethod>(m, "C2dMethod")
        .value("ZOH",              ctrl::C2dMethod::ZOH,
               "Zero-order hold - exact for piecewise-constant inputs.")
        .value("Tustin",           ctrl::C2dMethod::Tustin,
               "Bilinear (Tustin) transform.")
        .value("TustinPrewarped",  ctrl::C2dMethod::TustinPrewarped,
               "Bilinear transform prewarped at a specified frequency [rad/s].")
        .export_values();

    m.def("c2d", &ctrl::c2d,
          py::arg("sys_c"), py::arg("Ts"),
          py::arg("method")       = ctrl::C2dMethod::ZOH,
          py::arg("prewarp_freq") = 0.0,
          R"doc(
Discretise a continuous-time state-space model.

Parameters
----------
sys_c        : StateSpace with Ts == 0.0 (continuous-time marker).
Ts           : Desired sample period [s].
method       : C2dMethod.ZOH (default), .Tustin, or .TustinPrewarped.
prewarp_freq : Prewarping frequency [rad/s] (only for TustinPrewarped).
)doc");

    // -----------------------------------------------------------------------
    // IControllerObserver
    // -----------------------------------------------------------------------
    py::class_<ctrl::IControllerObserver,
               PyIControllerObserver,
               std::shared_ptr<ctrl::IControllerObserver>>(m, "IControllerObserver", R"doc(
Observer interface for non-intrusive controller telemetry.

Subclass and override on_compute() / on_reset() to receive callbacks.

Example
-------
>>> class Logger(ctrl.IControllerObserver):
...     def on_compute(self, u, signal):
...         print(f"u={u:.4f}  e={signal:.4f}")
>>> obs = Logger()
>>> pid.attach_observer(obs)
)doc")
        .def(py::init<>())
        .def("on_compute",     &ctrl::IControllerObserver::onCompute,
             py::arg("u"), py::arg("signal"),
             "Called after every SISO compute() invocation.")
        .def("on_compute_vec", &ctrl::IControllerObserver::onComputeVec,
             py::arg("u"), py::arg("signal"),
             "Called after every MIMO computeVec() invocation.")
        .def("on_reset",       &ctrl::IControllerObserver::onReset,
             "Called after every reset() invocation.")
        .def("on_state",
             [](ctrl::IControllerObserver& self,
                const std::string& key,
                const Eigen::VectorXd& value) {
                 self.onState(key, value);
             },
             py::arg("key"), py::arg("value"),
             "Called by controllers that emit internal state (ESO z, SMC surface, etc.).");

    // -----------------------------------------------------------------------
    // ControllerMonitor - SPC charts (CUSUM + EWMA) on controller output (M3/SPC)
    // -----------------------------------------------------------------------
    py::class_<ctrl::CUSUMChart>(m, "CUSUMChart", "Two-sided CUSUM chart parameters and state.")
        .def(py::init<>())
        .def_readwrite("target",   &ctrl::CUSUMChart::target)
        .def_readwrite("sigma",    &ctrl::CUSUMChart::sigma)
        .def_readwrite("k",        &ctrl::CUSUMChart::k,   "Slack parameter (multiples of sigma).")
        .def_readwrite("h",        &ctrl::CUSUMChart::h,   "Threshold (multiples of sigma).")
        .def_readwrite("C_plus",   &ctrl::CUSUMChart::C_plus)
        .def_readwrite("C_minus",  &ctrl::CUSUMChart::C_minus)
        .def("reset",              &ctrl::CUSUMChart::reset)
        .def("update",             &ctrl::CUSUMChart::update,  py::arg("x"))
        .def("statistic",          &ctrl::CUSUMChart::statistic);

    py::class_<ctrl::EWMAChart>(m, "EWMAChart", "EWMA chart parameters and state.")
        .def(py::init<>())
        .def_readwrite("target",   &ctrl::EWMAChart::target)
        .def_readwrite("sigma",    &ctrl::EWMAChart::sigma)
        .def_property("lambda_",
             [](const ctrl::EWMAChart& e) { return e.lambda; },
             [](ctrl::EWMAChart& e, double v) { e.lambda = v; },
             "Smoothing weight lambda in (0, 1].")
        .def_readwrite("L",        &ctrl::EWMAChart::L,    "Control limit multiplier.")
        .def_readwrite("Z",        &ctrl::EWMAChart::Z,    "Current EWMA statistic.")
        .def("reset",              &ctrl::EWMAChart::reset)
        .def("update",             &ctrl::EWMAChart::update, py::arg("x"))
        .def("statistic",          &ctrl::EWMAChart::statistic);

    py::class_<ctrl::ControllerMonitor, ctrl::IControllerObserver,
               std::shared_ptr<ctrl::ControllerMonitor>>(m, "ControllerMonitor", R"doc(
Statistical Process Control (SPC) observer: CUSUM + EWMA charts on controller output.

Attach to any IController to monitor its output for mean shifts and drifts.

Usage
-----
>>> mon = ctrl.ControllerMonitor()
>>> mon.set_target(0.0); mon.set_sigma(0.05)
>>> mon.set_alarm_callback(lambda chart, stat: print(f"[ALARM] {chart}: {stat:.3f}"))
>>> pid.attach_observer(mon)
>>> # After running closed loop:
>>> print(mon.n_alarms(), mon.cusum_stat(), mon.ewma_stat())
)doc")
        .def(py::init<>())
        .def("set_target",          &ctrl::ControllerMonitor::setTarget,       py::arg("mu0"))
        .def("set_sigma",           &ctrl::ControllerMonitor::setSigma,        py::arg("sigma"))
        .def("set_cusum_params",    &ctrl::ControllerMonitor::setCUSUMParams,  py::arg("k"), py::arg("h"))
        .def("set_ewma_params",     &ctrl::ControllerMonitor::setEWMAParams,   py::arg("lambda_val"), py::arg("L"))
        .def("set_watch_key",
             [](ctrl::ControllerMonitor& self, const std::string& key, int idx) {
                 self.setWatchKey(key, idx);
             }, py::arg("key"), py::arg("index") = 0,
             "Monitor an onState channel instead of compute() output.")
        .def("set_alarm_callback",
             [](ctrl::ControllerMonitor& self, py::object cb_py) {
                 self.setAlarmCallback([cb_py](std::string_view key, double stat) {
                     cb_py(std::string(key), stat);
                 });
             }, py::arg("callback"))
        .def("on_compute",  &ctrl::ControllerMonitor::onCompute,  py::arg("u"), py::arg("signal"))
        .def("on_state",
             [](ctrl::ControllerMonitor& self,
                const std::string& key, const Eigen::VectorXd& value) {
                 self.onState(key, value);
             }, py::arg("key"), py::arg("value"))
        .def("on_reset",    &ctrl::ControllerMonitor::onReset)
        .def("n_samples",   &ctrl::ControllerMonitor::nSamples)
        .def("n_alarms",    &ctrl::ControllerMonitor::nAlarms)
        .def("cusum_stat",  &ctrl::ControllerMonitor::cusumStat)
        .def("ewma_stat",   &ctrl::ControllerMonitor::ewmaStat);

    // -----------------------------------------------------------------------
    // ControllerRegistry (M2: self-registration feature map)
    // -----------------------------------------------------------------------
    m.def("registry_has",   &ctrl::ControllerRegistry::has,   py::arg("name"),
          "Return True if the named feature is registered (header was included).");
    m.def("registry_count", &ctrl::ControllerRegistry::count,
          "Return the number of registered features.");

    // -----------------------------------------------------------------------
    // IController (abstract base - Python can subclass)
    // -----------------------------------------------------------------------
    py::class_<ctrl::IController, PyIController,
               std::shared_ptr<ctrl::IController>>(m, "IController", R"doc(
Abstract base for all discrete-time controllers.

Override compute(), reset(), and sample_time() to create a custom controller
that participates in ControllerStack and all tuning infrastructure.
)doc")
        .def(py::init<>())
        .def("compute",     &ctrl::IController::compute, py::arg("signal"),
             "Advance one step.  signal = error e[k] for tracking controllers.")
        .def("compute_vec", &ctrl::IController::computeVec, py::arg("signal"),
             "MIMO variant.  Returns a 1-D NumPy array u[k].")
        .def("reset",       &ctrl::IController::reset,
             "Reset all internal states.")
        .def("sample_time", &ctrl::IController::sampleTime,
             "Sample time Ts [s].")
        .def("bumpless_init", &ctrl::IController::bumplessInit,
             py::arg("u_target"), py::arg("error"),
             "Prepare the controller for bumpless activation in a ControllerStack.")
        .def("is_healthy",  &ctrl::IController::isHealthy,
             "False when the most recent compute() produced a suboptimal result "
             "(e.g. QP solver hit max iterations).")
        .def("attach_observer",
             [](ctrl::IController &c,
                std::shared_ptr<ctrl::IControllerObserver> obs) {
                 c.attachObserver(obs);
             },
             py::arg("observer"),
             "Attach a telemetry observer (shared ownership - observer stays alive "
             "as long as the controller holds a reference).")
        .def("detach_observer", &ctrl::IController::detachObserver,
             "Remove the attached observer.")
        .def("has_observer", &ctrl::IController::hasObserver,
             "True if an observer is currently attached.");
}
