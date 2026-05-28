#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "ControllerToolbox.h"

namespace py = pybind11;

void bind_controllers(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // DiscretePID
    // -----------------------------------------------------------------------
    py::class_<ctrl::PIDParams>(m, "PIDParams",
        "Tuning parameters for DiscretePID (MATLAB parallel PID form).")
        .def(py::init<>())
        .def_readwrite("Kp",       &ctrl::PIDParams::Kp,       "Proportional gain.")
        .def_readwrite("Ki",       &ctrl::PIDParams::Ki,       "Integral gain.")
        .def_readwrite("Kd",       &ctrl::PIDParams::Kd,       "Derivative gain.")
        .def_readwrite("N",        &ctrl::PIDParams::N,        "Derivative filter coefficient [rad/s].")
        .def_readwrite("uMin",     &ctrl::PIDParams::uMin,     "Lower output saturation limit.")
        .def_readwrite("uMax",     &ctrl::PIDParams::uMax,     "Upper output saturation limit.")
        .def_readwrite("Kb",       &ctrl::PIDParams::Kb,       "Anti-windup back-calculation gain.")
        .def_readwrite("b_weight", &ctrl::PIDParams::b_weight,
                       "2DOF setpoint weight b in [0,1].  Only applied by compute_dom().");

    py::class_<ctrl::DiscretePID, ctrl::IController,
               std::shared_ptr<ctrl::DiscretePID>>(m, "DiscretePID", R"doc(
Discrete-time PID with backward-Euler integration, derivative filter, and anti-windup.

Example
-------
>>> p = ctrl.PIDParams(); p.Kp = 2.0; p.Ki = 0.5; p.N = 20.0
>>> pid = ctrl.DiscretePID(p, Ts=0.01)
>>> u = pid.compute(r - y)          # derivative on error
>>> u = pid.compute_dom(y, r)       # derivative on measurement (no setpoint kick)
)doc")
        .def(py::init<const ctrl::PIDParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",       &ctrl::DiscretePID::compute,   py::arg("error"))
        .def("compute_dom",   &ctrl::DiscretePID::computeDoM, py::arg("y"), py::arg("r"),
             "Derivative-on-measurement variant - avoids derivative kick on setpoint steps.")
        .def("reset",         &ctrl::DiscretePID::reset)
        .def("sample_time",   &ctrl::DiscretePID::sampleTime)
        .def("set_params",    &ctrl::DiscretePID::setParams, py::arg("params"))
        .def("params",        &ctrl::DiscretePID::params,
             py::return_value_policy::copy)
        .def("last_output",   &ctrl::DiscretePID::lastOutput)
        .def("bumpless_init", &ctrl::DiscretePID::bumplessInit,
             py::arg("u_target"), py::arg("error"));

    // -----------------------------------------------------------------------
    // DiscreteLeadLag
    // -----------------------------------------------------------------------
    py::class_<ctrl::LeadLagParams>(m, "LeadLagParams",
        "Parameters for a Tustin-discretised lead/lag compensator.")
        .def(py::init<>())
        .def_readwrite("continuous_zero", &ctrl::LeadLagParams::continuousZero,
                       "Continuous-time zero z_c [rad/s].")
        .def_readwrite("continuous_pole", &ctrl::LeadLagParams::continuousPole,
                       "Continuous-time pole p_c [rad/s].")
        .def_readwrite("gain", &ctrl::LeadLagParams::gain, "Static gain K.");

    py::class_<ctrl::DiscreteLeadLag, ctrl::IController,
               std::shared_ptr<ctrl::DiscreteLeadLag>>(m, "DiscreteLeadLag",
        "Tustin-discretised first-order lead/lag compensator C(s) = K*(s+z_c)/(s+p_c).")
        .def(py::init<const ctrl::LeadLagParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",     &ctrl::DiscreteLeadLag::compute,   py::arg("signal"))
        .def("reset",       &ctrl::DiscreteLeadLag::reset)
        .def("sample_time", &ctrl::DiscreteLeadLag::sampleTime)
        .def("set_params",  &ctrl::DiscreteLeadLag::setParams, py::arg("params"))
        .def("phase_at",    &ctrl::DiscreteLeadLag::phaseAt,   py::arg("omega_rad_s"),
             "Phase angle [rad] of the compensator at the given frequency.");

    // -----------------------------------------------------------------------
    // DiscreteSMC
    // -----------------------------------------------------------------------
    py::class_<ctrl::SMCParams>(m, "SMCParams",
        "Parameters for DiscreteSMC.  Sign convention: compute(y - ref).")
        .def(py::init<>())
        .def_readwrite("c_e",  &ctrl::SMCParams::c_e,  "Error coefficient in sliding surface.")
        .def_readwrite("c_de", &ctrl::SMCParams::c_de, "Error-rate coefficient (c_de absorbs Ts).")
        .def_readwrite("K",    &ctrl::SMCParams::K,    "Switching gain.")
        .def_readwrite("phi",  &ctrl::SMCParams::phi,  "Boundary-layer thickness.")
        .def_readwrite("uMin", &ctrl::SMCParams::uMin, "Lower output limit.")
        .def_readwrite("uMax", &ctrl::SMCParams::uMax, "Upper output limit.");

    py::class_<ctrl::DiscreteSMC, ctrl::IController,
               std::shared_ptr<ctrl::DiscreteSMC>>(m, "DiscreteSMC", R"doc(
First-order sliding-mode controller with sat() boundary layer.

Sign convention: pass compute(y - ref), NOT compute(ref - y).
For a positive-gain plant, positive (y - ref) must produce a negative control
action to drive output back to the reference.
)doc")
        .def(py::init<const ctrl::SMCParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",        &ctrl::DiscreteSMC::compute,        py::arg("error"))
        .def("reset",          &ctrl::DiscreteSMC::reset)
        .def("sample_time",    &ctrl::DiscreteSMC::sampleTime)
        .def("set_params",     &ctrl::DiscreteSMC::setParams,      py::arg("params"))
        .def("sliding_surface",&ctrl::DiscreteSMC::slidingSurface,
             "Current sliding surface value s[k].");

    // -----------------------------------------------------------------------
    // DiscreteADRC
    // -----------------------------------------------------------------------
    py::class_<ctrl::ADRCParams>(m, "ADRCParams",
        "Parameters for DiscreteADRC (bandwidth-parameterised 2nd-order LADRC).")
        .def(py::init<>())
        .def_readwrite("omega_c", &ctrl::ADRCParams::omega_c,
                       "Controller bandwidth [rad/s].")
        .def_readwrite("omega_o", &ctrl::ADRCParams::omega_o,
                       "ESO observer bandwidth [rad/s].  Typically 3-5x omega_c.")
        .def_readwrite("b0",      &ctrl::ADRCParams::b0,
                       "Nominal input gain (approximate).  b0 = 1.0 for a double integrator.")
        .def_readwrite("uMin",    &ctrl::ADRCParams::uMin, "Lower output limit.")
        .def_readwrite("uMax",    &ctrl::ADRCParams::uMax, "Upper output limit.");

    py::class_<ctrl::DiscreteADRC, ctrl::IController,
               std::shared_ptr<ctrl::DiscreteADRC>>(m, "DiscreteADRC", R"doc(
Bandwidth-parameterised 2nd-order LADRC.

The ESO models the plant as y'' = f + b0*u (double integrator + total disturbance).
For a double integrator plant, b0 = 1.0 is exact.

Prefer compute_tracking(y, r) over compute(error); the latter requires
a prior set_reference(r) call.
)doc")
        .def(py::init<const ctrl::ADRCParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",          &ctrl::DiscreteADRC::compute,         py::arg("error"),
             "compute(error) - requires set_reference(r) to have been called first.")
        .def("compute_tracking", &ctrl::DiscreteADRC::computeTracking,
             py::arg("y"), py::arg("r"),
             "Preferred interface: supply plant output y and setpoint r directly.")
        .def("set_reference",    &ctrl::DiscreteADRC::setReference,    py::arg("r"))
        .def("set_params",       &ctrl::DiscreteADRC::setParams,       py::arg("params"))
        .def("reset",            &ctrl::DiscreteADRC::reset)
        .def("sample_time",      &ctrl::DiscreteADRC::sampleTime)
        .def("eso_state",        &ctrl::DiscreteADRC::esoState,
             "ESO state [z1, z2, z3] as a NumPy array: "
             "z1=position estimate, z2=velocity estimate, z3=total disturbance estimate.");

    // -----------------------------------------------------------------------
    // ExtremumSeeker
    // -----------------------------------------------------------------------
    py::class_<ctrl::ExtremumSeekerParams>(m, "ExtremumSeekerParams",
        "Parameters for the perturbation-based ExtremumSeeker.")
        .def(py::init<>())
        .def_readwrite("perturb_amp",  &ctrl::ExtremumSeekerParams::perturbAmp)
        .def_readwrite("perturb_freq", &ctrl::ExtremumSeekerParams::perturbFreq)
        .def_readwrite("lpf_cutoff",   &ctrl::ExtremumSeekerParams::lpfCutoff)
        .def_readwrite("hpf_cutoff",   &ctrl::ExtremumSeekerParams::hpfCutoff)
        .def_readwrite("integ_gain",   &ctrl::ExtremumSeekerParams::integGain)
        .def_readwrite("seek_minimum", &ctrl::ExtremumSeekerParams::seekMinimum,
                       "True -> minimise cost.  False -> maximise.");

    py::class_<ctrl::ExtremumSeeker, ctrl::IController,
               std::shared_ptr<ctrl::ExtremumSeeker>>(m, "ExtremumSeeker",
        "Perturbation-based extremum-seeking controller.  Pass cost J = plant output to compute().")
        .def(py::init<const ctrl::ExtremumSeekerParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",          &ctrl::ExtremumSeeker::compute,        py::arg("cost"))
        .def("reset",            &ctrl::ExtremumSeeker::reset)
        .def("sample_time",      &ctrl::ExtremumSeeker::sampleTime)
        .def("set_params",       &ctrl::ExtremumSeeker::setParams,      py::arg("params"))
        .def("current_estimate", &ctrl::ExtremumSeeker::currentEstimate,
             "Current integrator state theta (estimate of the optimal operating point).");

    // -----------------------------------------------------------------------
    // SmithPredictor
    // -----------------------------------------------------------------------
    py::class_<ctrl::SmithPredictor, ctrl::IController,
               std::shared_ptr<ctrl::SmithPredictor>>(m, "SmithPredictor",
        "Dead-time compensator wrapping any IController.")
        .def(py::init<std::shared_ptr<ctrl::IController>,
                      const ctrl::StateSpace &, int>(),
             py::arg("inner_controller"), py::arg("delay_free_model"), py::arg("delay_steps"))
        .def("compute",     &ctrl::SmithPredictor::compute,    py::arg("error"))
        .def("reset",       &ctrl::SmithPredictor::reset)
        .def("sample_time", &ctrl::SmithPredictor::sampleTime)
        .def("set_model",
             py::overload_cast<const ctrl::StateSpace &, int>(
                 &ctrl::SmithPredictor::setModel),
             py::arg("delay_model"), py::arg("delay_steps"),
             "Hot-swap the internal plant model and integer dead-time length.");

    // -----------------------------------------------------------------------
    // DiscreteMPC + MPCParams
    // -----------------------------------------------------------------------
    py::class_<ctrl::MPCParams>(m, "MPCParams",
        "Tuning and horizon parameters for DiscreteMPC.")
        .def(py::init<>())
        .def_readwrite("Np",          &ctrl::MPCParams::Np,
                       "Prediction horizon [steps]. Cover approx. plant settling time.")
        .def_readwrite("Nc",          &ctrl::MPCParams::Nc,
                       "Control horizon [steps], Nc <= Np. Fewer = smoother moves.")
        .def_readwrite("rho_y",       &ctrl::MPCParams::rho_y,
                       "Output tracking weight (Qy = rho_y * I). Bryson: 1/e_max^2.")
        .def_readwrite("rho_u",       &ctrl::MPCParams::rho_u,
                       "Move suppression weight (Ru = rho_u * I). Bryson: 1/delta_u_max^2.")
        .def_readwrite("uMin",        &ctrl::MPCParams::uMin,  "Hard lower limit on u.")
        .def_readwrite("uMax",        &ctrl::MPCParams::uMax,  "Hard upper limit on u.")
        .def_readwrite("duMin",       &ctrl::MPCParams::duMin, "Hard lower limit on Delta u.")
        .def_readwrite("duMax",       &ctrl::MPCParams::duMax, "Hard upper limit on Delta u.")
        .def_readwrite("qp_max_iter", &ctrl::MPCParams::qpMaxIter,
                       "Gradient-projection iteration limit.")
        .def_readwrite("qp_tol",      &ctrl::MPCParams::qpTol,
                       "Convergence tolerance (||Delta x||_inf).");

    py::class_<ctrl::DiscreteMPC, ctrl::IController,
               std::shared_ptr<ctrl::DiscreteMPC>>(m, "DiscreteMPC", R"doc(
Discrete-time Model Predictive Controller (condensed incremental QP formulation).

Minimises a receding-horizon cost:
  J = sum_i rho_y * ||y_hat[k+i|k] - r||^2 + sum_j rho_u * ||Delta u[k+j]||^2

Preferred interface: compute_ref(x, r) for full MIMO state-reference control.
SISO interface: compute(error) is inherited from IController for stack compatibility.

Example
-------
>>> p = ctrl.MPCParams(); p.Np = 20; p.Nc = 5; p.rho_y = 1.0; p.rho_u = 0.1
>>> mpc = ctrl.DiscreteMPC(plant_ss, p)
>>> u = mpc.compute_ref(x_current, r_ref)   # returns np.ndarray (m,)
)doc")
        .def(py::init<const ctrl::StateSpace &, const ctrl::MPCParams &>(),
             py::arg("plant"), py::arg("params"))
        .def("compute",          &ctrl::DiscreteMPC::compute,       py::arg("error"),
             "SISO convenience: compute u[k] from scalar error (requires IController compat).")
        .def("compute_ref",      &ctrl::DiscreteMPC::computeRef,
             py::arg("x_current"), py::arg("r_ref"),
             "Full MIMO interface: optimise u[k] given state x[k] and reference r[k].")
        .def("reset",            &ctrl::DiscreteMPC::reset)
        .def("sample_time",      &ctrl::DiscreteMPC::sampleTime)
        .def("set_params",       &ctrl::DiscreteMPC::setParams,     py::arg("params"),
             "Update horizon/weight parameters and recompute condensed matrices.")
        .def("params",           &ctrl::DiscreteMPC::params,
             py::return_value_policy::copy)
        .def("set_plant",        &ctrl::DiscreteMPC::setPlant,      py::arg("plant"),
             "Hot-swap the plant model (successive linearisation). Rebuilds prediction matrices.")
        .def("set_state",        &ctrl::DiscreteMPC::setState,      py::arg("x"),
             "Inject a state estimate from an external observer (e.g., Kalman filter).")
        .def("set_last_applied", &ctrl::DiscreteMPC::setLastApplied, py::arg("u_applied"),
             "Correct u_prev after external actuator saturation/redistribution.")
        .def("last_qp_converged",&ctrl::DiscreteMPC::lastQPConverged,
             "True if the most recent QP converged within qp_max_iter iterations.")
        .def("last_qp_iters",    &ctrl::DiscreteMPC::lastQPIters,
             "Actual gradient-projection iterations used in the most recent compute_ref().")
        .def("is_healthy",       &ctrl::DiscreteMPC::isHealthy,
             "False when the most recent QP exited at qp_max_iter (suboptimal output).");

    // -----------------------------------------------------------------------
    // LQRParams + DiscreteLQR + LQRAdapter
    // -----------------------------------------------------------------------
    py::class_<ctrl::LQRParams>(m, "LQRParams",
        "LQR weighting matrices Q (state cost) and R (control cost).")
        .def(py::init<>())
        .def_readwrite("Q", &ctrl::LQRParams::Q,
                       "State cost matrix (n x n, positive semi-definite). "
                       "Increase Q_ii to tighten tracking of state i.")
        .def_readwrite("R", &ctrl::LQRParams::R,
                       "Control cost matrix (m x m, positive definite). "
                       "Increase R_jj to penalise actuator j.");

    py::class_<ctrl::DiscreteLQR>(m, "DiscreteLQR", R"doc(
Discrete-time Linear Quadratic Regulator (LQR).

Solves the Discrete Algebraic Riccati Equation (DARE) offline via value iteration,
then applies the optimal state-feedback law u[k] = -K* (x[k] - x_ref) + u_ff.

Example
-------
>>> lqr_p = ctrl.LQRParams()
>>> lqr_p.Q = np.diag([10.0, 1.0])
>>> lqr_p.R = np.array([[0.1]])
>>> lqr = ctrl.DiscreteLQR(plant_ss, lqr_p)
>>> u = lqr.compute(x)           # regulation to origin
>>> u = lqr.compute(x, x_ref)   # tracking with reference state
)doc")
        .def(py::init<const ctrl::StateSpace &, const ctrl::LQRParams &>(),
             py::arg("plant"), py::arg("params"))
        .def("compute",
             [](const ctrl::DiscreteLQR &lqr,
                const Eigen::VectorXd &x,
                py::object x_ref,
                py::object u_ff) {
                 Eigen::VectorXd xr, uf;
                 if (!x_ref.is_none()) xr = x_ref.cast<Eigen::VectorXd>();
                 if (!u_ff.is_none())  uf = u_ff.cast<Eigen::VectorXd>();
                 return lqr.compute(x, xr, uf);
             },
             py::arg("x"), py::arg("x_ref") = py::none(), py::arg("u_ff") = py::none(),
             "Compute u[k] = -K*(x - x_ref) + u_ff.  Returns VectorXd (m,).",
             py::return_value_policy::copy)
        .def("gain_matrix",      &ctrl::DiscreteLQR::gainMatrix,
             py::return_value_policy::copy,
             "Optimal feedback gain K* (m x n).")
        .def("riccati_solution", &ctrl::DiscreteLQR::riccatiSolution,
             py::return_value_policy::copy,
             "DARE stabilising solution Pinf (n x n).")
        .def("dare_converged",   &ctrl::DiscreteLQR::dareConverged,
             "True if the DARE converged to the requested tolerance.")
        .def("dare_iterations",  &ctrl::DiscreteLQR::dareIterations,
             "Number of value-iteration steps taken by the DARE solver.")
        .def("sample_time",      &ctrl::DiscreteLQR::sampleTime);

    py::class_<ctrl::LQRAdapter, ctrl::IController,
               std::shared_ptr<ctrl::LQRAdapter>>(m, "LQRAdapter", R"doc(
Adapter that wraps DiscreteLQR as an IController for use in ControllerStack.

State and reference are provided via Python callables (zero-argument lambdas).
The LQR object must outlive the adapter; pass both to the same ControllerStack to
ensure coincident lifetimes.

Example
-------
>>> lqr = ctrl.DiscreteLQR(plant_ss, lqr_p)
>>> adapter = ctrl.LQRAdapter(lqr,
...     state_fn=lambda: x_current,
...     ref_fn=lambda: x_reference)
>>> u_scalar = adapter.compute(0.0)   # signal ignored; callbacks supply x and x_ref
>>> u_vec    = adapter.compute_vec()  # full m-element control vector
)doc")
        .def(py::init([](ctrl::DiscreteLQR &lqr,
                         py::object state_fn_obj,
                         py::object ref_fn_obj) -> ctrl::LQRAdapter* {
                 // Capture py::object (not py::cpp_function) to avoid overload-deduction
                 // errors in pybind11's function_signature_t machinery.
                 std::function<Eigen::VectorXd()> state_fn =
                     [state_fn_obj]() -> Eigen::VectorXd {
                         return state_fn_obj().cast<Eigen::VectorXd>();
                     };

                 std::function<Eigen::VectorXd()> ref_fn;
                 if (!ref_fn_obj.is_none()) {
                     ref_fn = [ref_fn_obj]() -> Eigen::VectorXd {
                         return ref_fn_obj().cast<Eigen::VectorXd>();
                     };
                 }

                 return new ctrl::LQRAdapter(lqr,
                     std::move(state_fn), std::move(ref_fn));
             }),
             py::arg("lqr"), py::arg("state_fn"), py::arg("ref_fn") = py::none(),
             // Keep the LQR object alive as long as this adapter is alive.
             py::keep_alive<0, 1>())
        .def("compute",
             &ctrl::LQRAdapter::compute,
             py::arg("signal") = 0.0,
             "Compute u[0] from callbacks.  signal is ignored.")
        .def("compute_vec",
             [](ctrl::LQRAdapter &a) {
                 return a.computeVec(Eigen::VectorXd());
             },
             py::return_value_policy::copy,
             "Full MIMO control vector u[k] (m,) from callbacks.")
        .def("reset",       &ctrl::LQRAdapter::reset)
        .def("sample_time", &ctrl::LQRAdapter::sampleTime)
        .def("is_healthy",  &ctrl::LQRAdapter::isHealthy,
             "False if the underlying DARE did not converge at construction.");

    // -----------------------------------------------------------------------
    // DiscreteLQG
    // -----------------------------------------------------------------------
    py::class_<ctrl::DiscreteLQG>(m, "DiscreteLQG", R"doc(
Discrete-time Linear Quadratic Gaussian (LQG) controller (LQR + Kalman filter).

Combines optimal state-feedback (LQR) with optimal state estimation (Kalman filter)
via the separation principle: both can be tuned independently.

Preferred interface: step(y, u_prev) returns the full control vector u[k].
SISO convenience: compute(y_scalar) with setReference() / setUPrev() for stacks.

Example
-------
>>> lqg = ctrl.DiscreteLQG(plant_ss, lqr_p, Q_noise, R_noise)
>>> for k in range(N):
...     u = lqg.step(y[k], u_prev)
...     u_prev = u
)doc")
        .def(py::init(
             [](const ctrl::StateSpace &plant,
                const ctrl::LQRParams &lqr_p,
                const Eigen::MatrixXd &Q_noise,
                const Eigen::MatrixXd &R_noise,
                py::object P0) {
                 Eigen::MatrixXd p0;
                 if (!P0.is_none()) p0 = P0.cast<Eigen::MatrixXd>();
                 return new ctrl::DiscreteLQG(plant, lqr_p, Q_noise, R_noise, p0);
             }),
             py::arg("plant"), py::arg("lqr_params"),
             py::arg("Q_noise"), py::arg("R_noise"),
             py::arg("P0") = py::none(),
             "Construct LQG. P0=None initialises Kalman covariance to identity.")
        .def("step",
             [](ctrl::DiscreteLQG &lqg,
                const Eigen::VectorXd &y,
                const Eigen::VectorXd &u_prev,
                py::object x_ref) {
                 Eigen::VectorXd xr;
                 if (!x_ref.is_none()) xr = x_ref.cast<Eigen::VectorXd>();
                 return lqg.step(y, u_prev, xr);
             },
             py::arg("y"), py::arg("u_prev"), py::arg("x_ref") = py::none(),
             "Full step: Kalman predict/update then LQR u[k] = -K*(x^ - x_ref). "
             "Returns control vector (m,).",
             py::return_value_policy::copy)
        .def("compute",       &ctrl::DiscreteLQG::compute,       py::arg("y_scalar"),
             "SISO convenience: returns u[0]. Requires set_reference() and set_u_prev() first.")
        .def("set_reference", &ctrl::DiscreteLQG::setReference,  py::arg("x_ref"),
             "Set the reference state for the next compute() call.")
        .def("set_u_prev",    &ctrl::DiscreteLQG::setUPrev,      py::arg("u"),
             "Set the previous control input for the next compute() call.")
        .def("reset",         &ctrl::DiscreteLQG::reset)
        .def("sample_time",   &ctrl::DiscreteLQG::sampleTime)
        .def("state_estimate",&ctrl::DiscreteLQG::stateEstimate,
             py::return_value_policy::copy,
             "Current Kalman state estimate x^[k|k] as a NumPy array (n,).")
        .def("gain_matrix",   &ctrl::DiscreteLQG::gainMatrix,
             py::return_value_policy::copy,
             "Optimal LQR feedback gain K* (m x n).");

    // -----------------------------------------------------------------------
    // GPCParams + GeneralizedPredictiveController
    // -----------------------------------------------------------------------
    py::class_<ctrl::GPCParams>(m, "GPCParams",
        "Tuning and horizon parameters for GeneralizedPredictiveController.")
        .def(py::init<>())
        .def_readwrite("Np",          &ctrl::GPCParams::Np,
                       "Prediction horizon [steps].")
        .def_readwrite("Nu",          &ctrl::GPCParams::Nu,
                       "Control horizon [steps], Nu <= Np.")
        .def_readwrite("rho_y",       &ctrl::GPCParams::rho_y,
                       "Output tracking weight (Qy = rho_y * I).")
        .def_readwrite("rho_u",       &ctrl::GPCParams::rho_u,
                       "Move suppression weight (Ru = rho_u * I).")
        .def_readwrite("alpha",       &ctrl::GPCParams::alpha,
                       "Reference trajectory softening factor alpha in [0,1). "
                       "0 = step reference, -> 1 = very soft approach.")
        .def_readwrite("uMin",        &ctrl::GPCParams::uMin,  "Hard lower limit on u.")
        .def_readwrite("uMax",        &ctrl::GPCParams::uMax,  "Hard upper limit on u.")
        .def_readwrite("duMin",       &ctrl::GPCParams::duMin, "Hard lower limit on Delta u.")
        .def_readwrite("duMax",       &ctrl::GPCParams::duMax, "Hard upper limit on Delta u.")
        .def_readwrite("yMin",        &ctrl::GPCParams::yMin,
                       "Soft lower bound on predicted output (default -1e9 = inactive).")
        .def_readwrite("yMax",        &ctrl::GPCParams::yMax,
                       "Soft upper bound on predicted output (default +1e9 = inactive).")
        .def_readwrite("qp_max_iter", &ctrl::GPCParams::qpMaxIter,
                       "Gradient-projection iteration limit.")
        .def_readwrite("qp_tol",      &ctrl::GPCParams::qpTol,
                       "Convergence tolerance (||Delta x||_inf).");

    py::class_<ctrl::GeneralizedPredictiveController, ctrl::IController,
               std::shared_ptr<ctrl::GeneralizedPredictiveController>>(
            m, "GeneralizedPredictiveController", R"doc(
Discrete-time Generalised Predictive Controller (GPC - velocity-form / CARIMA).

Extends DiscreteMPC with two features:
  1. Velocity-form CARIMA model: built-in integrating disturbance -> offset-free tracking.
  2. Reference trajectory softening via alpha parameter.

Pair with RecursiveLeastSquares for a self-tuning adaptive GPC loop.

Example
-------
>>> p = ctrl.GPCParams(); p.Np = 20; p.Nu = 5; p.alpha = 0.3
>>> gpc = ctrl.GeneralizedPredictiveController(plant_ss, p)
>>> u = gpc.compute_ref(y_current, r_setpoint)  # preferred interface
>>> u = gpc.compute(r - y)                       # IController-compatible
)doc")
        .def(py::init<const ctrl::StateSpace &, const ctrl::GPCParams &>(),
             py::arg("plant"), py::arg("params"))
        .def("compute",          &ctrl::GeneralizedPredictiveController::compute,
             py::arg("error"),
             "IController-compatible interface. Uses stored reference from set_reference().")
        .def("compute_ref",      &ctrl::GeneralizedPredictiveController::computeRef,
             py::arg("y"), py::arg("r"),
             "Preferred interface: supply plant output y and setpoint r separately.")
        .def("set_plant",        &ctrl::GeneralizedPredictiveController::setPlant,
             py::arg("plant"),
             "Hot-swap the plant model for adaptive GPC. Rebuilds condensed matrices.")
        .def("set_params",       &ctrl::GeneralizedPredictiveController::setParams,
             py::arg("params"),
             "Update parameters and rebuild condensed matrices.")
        .def("params",           &ctrl::GeneralizedPredictiveController::params,
             py::return_value_policy::copy)
        .def("set_reference",    &ctrl::GeneralizedPredictiveController::setReference,
             py::arg("r"),
             "Set the reference setpoint for the next compute(error) call.")
        .def("reset",            &ctrl::GeneralizedPredictiveController::reset)
        .def("sample_time",      &ctrl::GeneralizedPredictiveController::sampleTime)
        .def("augmented_state",  &ctrl::GeneralizedPredictiveController::augmentedState,
             py::return_value_policy::copy,
             "Current augmented CARIMA state x_a = [Delta x; y] (n+p,).")
        .def("last_qp_converged",&ctrl::GeneralizedPredictiveController::lastQPConverged,
             "True if the most recent QP converged within qp_max_iter iterations.")
        .def("last_qp_iters",    &ctrl::GeneralizedPredictiveController::lastQPIters,
             "Actual gradient-projection iterations used in the most recent compute_ref().")
        .def("is_healthy",       &ctrl::GeneralizedPredictiveController::isHealthy);

    // -----------------------------------------------------------------------
    // StackMode + ControllerStack
    // -----------------------------------------------------------------------
    py::enum_<ctrl::StackMode>(m, "StackMode",
        "Dispatch strategy for ControllerStack.")
        .value("Supervisory", ctrl::StackMode::Supervisory,
               "First healthy, eligible entry wins.  Bumpless handover on switch.")
        .value("Additive",    ctrl::StackMode::Additive,
               "All enabled entries contribute; their outputs are summed.")
        .value("Weighted",    ctrl::StackMode::Weighted,
               "Normalised weighted average of active, gate-passing entries.")
        .export_values();

    py::class_<ctrl::ControllerStack, ctrl::IController,
               std::shared_ptr<ctrl::ControllerStack>>(m, "ControllerStack", R"doc(
Multi-controller orchestration in Supervisory, Additive, or Weighted mode.

ControllerStack itself satisfies IController so stacks can be nested.

Example (supervisory fallback chain)
--------------------------------------
>>> stack = ctrl.ControllerStack(ctrl.StackMode.Supervisory, Ts=0.01)
>>> stack.add_controller(mpc, "MPC", condition=lambda e, u: mpc.is_healthy())
>>> stack.add_controller(pid, "PID")   # always-eligible fallback
>>> u = stack.compute(error)
>>> print(stack.active_controller_name())

Example (weighted blend)
------------------------
>>> stack = ctrl.ControllerStack(ctrl.StackMode.Weighted, Ts=0.01)
>>> stack.add_controller(pid, "fast_pid", weight=0.7)
>>> stack.add_controller(mpc, "slow_mpc", weight=0.3)
>>> u = stack.compute(error)
)doc")
        .def(py::init<ctrl::StackMode, double>(),
             py::arg("mode"), py::arg("sample_time"),
             "Construct an empty stack.")
        .def("add_controller",
             [](ctrl::ControllerStack &stack,
                std::shared_ptr<ctrl::IController> ctrl,
                const std::string &name,
                double weight,
                py::object condition_fn) {
                 std::function<bool(double, double)> cond;
                 if (!condition_fn.is_none()) {
                     // Capture py::object to avoid overload deduction issues.
                     cond = [condition_fn](double e, double u) -> bool {
                         return condition_fn(e, u).cast<bool>();
                     };
                 }
                 stack.addController(ctrl, name, weight, std::move(cond));
             },
             py::arg("controller"), py::arg("name"),
             py::arg("weight") = 1.0, py::arg("condition") = py::none(),
             "Append a controller. condition(error, last_output) -> bool; None = always eligible.")
        .def("remove_controller",
             &ctrl::ControllerStack::removeController, py::arg("name"),
             "Remove the entry with the given name.")
        .def("set_active",
             &ctrl::ControllerStack::setActive, py::arg("name"), py::arg("active"),
             "Enable or disable an entry without removing it.")
        .def("set_weight",
             &ctrl::ControllerStack::setWeight, py::arg("name"), py::arg("weight"),
             "Update the weight of an entry (Weighted mode only).")
        .def("compute",     &ctrl::ControllerStack::compute, py::arg("error"))
        .def("reset",       &ctrl::ControllerStack::reset)
        .def("sample_time", &ctrl::ControllerStack::sampleTime)
        .def("mode",        &ctrl::ControllerStack::mode,
             "Current dispatch mode (StackMode enum).")
        .def("active_controller_name",
             &ctrl::ControllerStack::activeControllerName,
             "Name of the controller selected in the most recent compute() call (Supervisory mode).");

    // -----------------------------------------------------------------------
    // Fuzzy module (optional - guarded by CTRL_HAS_FUZZY)
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_FUZZY)
    // TODO: bind FuzzyPDParams, FuzzyPIDParams, SupervisorParams
    // TODO: bind FuzzyPD, FuzzyPID, FuzzySupervisor
    // TODO: bind FuzzySystem, LinguisticVariable, LinguisticTerm, Rule
    // TODO: bind membership-function factories (mfTriangular, mfGaussian, ...)
    // TODO: bind InferenceMethod and DefuzzMethod enums
    (void)m; // suppress unused-variable warning until stubs are filled
#endif
}
