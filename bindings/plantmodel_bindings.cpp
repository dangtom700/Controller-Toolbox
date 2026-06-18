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

    m.def("c2d", py::overload_cast<const ctrl::StateSpace &, double,
                                    ctrl::C2dMethod, double>(&ctrl::c2d),
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
    // DAESystem - Index-1 semi-explicit DAE (P1/P2)
    // -----------------------------------------------------------------------
    py::class_<ctrl::DAESystem>(m, "DAESystem", R"doc(
Index-1 semi-explicit Differential-Algebraic Equation system.

    x1' = f(x1, x2, u)     (n_diff differential states)
    0   = g(x1, x2, u)     (n_alg  algebraic states, Index-1: dg/dx2 invertible)
    y   = h(x1, x2, u)     (output map, optional)

Use dae2ode() to simulate, consistent_init() to find initial conditions,
and dae_c2d() to linearise and discretise for controller design.
)doc")
        .def(py::init<>())
        .def_readwrite("n_diff", &ctrl::DAESystem::n_diff,
                       "Number of differential states (x1 dimension).")
        .def_readwrite("n_alg",  &ctrl::DAESystem::n_alg,
                       "Number of algebraic states (x2 dimension).")
        .def_readwrite("Ts",     &ctrl::DAESystem::Ts,
                       "Sample time [s] used by dae2ode() for Euler stepping.")
        .def("set_f", [](ctrl::DAESystem &dae, py::object f_py) {
                 dae.f = [f_py](const Eigen::VectorXd &x1,
                                const Eigen::VectorXd &x2,
                                double u) -> Eigen::VectorXd {
                     return f_py(x1, x2, u).cast<Eigen::VectorXd>();
                 };
             }, py::arg("f"),
             "Set differential equations: f(x1, x2, u) -> VectorXd (x1 dimension).")
        .def("set_g", [](ctrl::DAESystem &dae, py::object g_py) {
                 dae.g = [g_py](const Eigen::VectorXd &x1,
                                const Eigen::VectorXd &x2,
                                double u) -> Eigen::VectorXd {
                     return g_py(x1, x2, u).cast<Eigen::VectorXd>();
                 };
             }, py::arg("g"),
             "Set algebraic constraints: g(x1, x2, u) -> VectorXd (n_alg dimension, must equal 0).")
        .def("set_h", [](ctrl::DAESystem &dae, py::object h_py) {
                 dae.h = [h_py](const Eigen::VectorXd &x1,
                                const Eigen::VectorXd &x2,
                                double u) -> Eigen::VectorXd {
                     return h_py(x1, x2, u).cast<Eigen::VectorXd>();
                 };
             }, py::arg("h"),
             "Set output map: h(x1, x2, u) -> VectorXd (output dimension).")
        .def("__repr__", [](const ctrl::DAESystem &d) {
            return "<DAESystem n_diff=" + std::to_string(d.n_diff) +
                   " n_alg="  + std::to_string(d.n_alg) +
                   " Ts="     + std::to_string(d.Ts) + ">";
        });

    m.def("consistent_init",
          [](const ctrl::DAESystem &dae, const Eigen::VectorXd &x1,
             double u, const Eigen::VectorXd &x2_guess,
             int max_iter, double tol) {
              return ctrl::consistentInit(dae, x1, u, x2_guess, max_iter, tol).x;
          },
          py::arg("dae"), py::arg("x1"), py::arg("u"), py::arg("x2_guess"),
          py::arg("max_iter") = 20, py::arg("tol") = 1e-9,
          R"doc(
Find consistent algebraic states: solve g(x1, x2, u) = 0 for x2.

Parameters
----------
dae      : DAESystem whose g defines the constraint.
x1       : Fixed differential states.
u        : Fixed scalar input.
x2_guess : Initial guess for x2.
max_iter : Newton iteration limit (default 20).
tol      : Convergence tolerance on norm(g) (default 1e-9).

Returns
-------
x2 satisfying norm(g(x1, x2, u)) < tol, or best Newton iterate.
)doc");

    m.def("dae2ode",
          [](const ctrl::DAESystem &dae, int max_iter, double tol) {
              auto step_fn = ctrl::dae2ode(dae, max_iter, tol);
              // Return a Python-callable wrapper around the C++ step function
              return py::cpp_function(
                  [step_fn](const Eigen::VectorXd &x_aug, double u) {
                      return step_fn(x_aug, u);
                  },
                  py::arg("x_aug"), py::arg("u"));
          },
          py::arg("dae"), py::arg("newton_max_iter") = 20, py::arg("newton_tol") = 1e-9,
          R"doc(
Convert a DAESystem to a discrete-time step function F(x_aug, u) -> x_aug_next.

The returned callable advances the augmented state [x1; x2] by one Euler step,
then Newton-solves for the consistent algebraic state x2_next.

Parameters
----------
dae            : Source DAESystem (must have Ts > 0).
newton_max_iter : Newton iteration limit (default 20).
newton_tol     : Newton convergence tolerance (default 1e-9).

Returns
-------
Callable (x_aug: ndarray, u: float) -> x_aug_next: ndarray
)doc");

    m.def("dae_c2d",
          [](const ctrl::DAESystem &dae, const Eigen::VectorXd &x1_op,
             const Eigen::VectorXd &x2_op, double u_op, double Ts,
             ctrl::C2dMethod method) {
              return ctrl::c2d(dae, x1_op, x2_op, u_op, Ts, method);
          },
          py::arg("dae"), py::arg("x1_op"), py::arg("x2_op"), py::arg("u_op"),
          py::arg("Ts"), py::arg("method") = ctrl::C2dMethod::ZOH,
          R"doc(
Linearise a DAESystem, eliminate algebraic states, and discretise.

Computes the reduced continuous-time model:
    A_red = A11 - A12 * inv(G2) * G1
    B_red = B1  - A12 * inv(G2) * B2
Then applies ZOH or Tustin discretisation.

Parameters
----------
dae    : Source DAESystem.
x1_op  : Operating-point differential states.
x2_op  : Operating-point algebraic states (must satisfy g approx = 0).
u_op   : Operating-point scalar input.
Ts     : Desired sample period [s].
method : C2dMethod.ZOH (default) or .Tustin.

Returns
-------
Discrete-time StateSpace of dimension n_diff.
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
