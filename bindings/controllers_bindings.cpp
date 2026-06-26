#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "ControllerToolbox.h"
#include "IterativeLearningControl.h"
#include "SINDy.h"
#include "KoopmanEDMD.h"
#include "L1AdaptiveController.h"
#include "CBFSafetyFilter.h"
#include "GaussianProcess.h"
#include "GPResidualModel.h"
#include "EchoStateNetwork.h"
#include "NeuralPID.h"
#include "CEMController.h"
#include "DynaController.h"
#include "ScenarioMPC.h"
#include "BayesianOptimizer.h"
#include "GeneticAlgorithm.h"
#include "ParticleSwarmOptimizer.h"
#include "DifferentialEvolution.h"
#include "HybridModel.h"
#include "HybridMPC.h"
#include "HybridModelTrainer.h"

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
    // ResonantController
    // -----------------------------------------------------------------------
    py::class_<ctrl::ResonantParams>(m, "ResonantParams",
        "Tuning parameters for ResonantController (single-harmonic internal-model corrector).")
        .def(py::init<>())
        .def_readwrite("targetFreqHz",     &ctrl::ResonantParams::targetFreqHz,
                       "Harmonic frequency to reject/track [Hz].")
        .def_readwrite("dampingRadPerSec", &ctrl::ResonantParams::dampingRadPerSec,
                       "Bandwidth/peak-width parameter wc [rad/s].")
        .def_readwrite("Kr",               &ctrl::ResonantParams::Kr,
                       "Resonant gain - the exact steady-state gain at targetFreqHz.")
        .def_readwrite("uMin",             &ctrl::ResonantParams::uMin, "Output saturation lower limit.")
        .def_readwrite("uMax",             &ctrl::ResonantParams::uMax, "Output saturation upper limit.");

    py::class_<ctrl::ResonantController, ctrl::IController,
               std::shared_ptr<ctrl::ResonantController>>(m, "ResonantController", R"doc(
Discrete-time non-ideal (finite-Q) resonant controller for single-harmonic rejection.

Composes through ControllerStack(Additive) alongside a base controller, one instance per
target harmonic.

Example
-------
>>> rp = ctrl.ResonantParams(); rp.targetFreqHz = 50.0; rp.dampingRadPerSec = 5.0; rp.Kr = 2.0
>>> rc = ctrl.ResonantController(rp, Ts=1e-4)
>>> u = rc.compute(error)
)doc")
        .def(py::init<const ctrl::ResonantParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",     &ctrl::ResonantController::compute, py::arg("error"))
        .def("reset",       &ctrl::ResonantController::reset)
        .def("sample_time", &ctrl::ResonantController::sampleTime)
        .def("set_params",  &ctrl::ResonantController::setParams, py::arg("params"))
        .def("params",      &ctrl::ResonantController::params, py::return_value_policy::copy);

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

    m.def("make_lqr_controller",
          [](const ctrl::StateSpace& plant,
             const ctrl::LQRParams& params,
             py::object state_fn_obj,
             py::object ref_fn_obj) -> std::shared_ptr<ctrl::LQRAdapter> {
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
              return ctrl::makeLQRController(plant, params,
                                             std::move(state_fn), std::move(ref_fn));
          },
          py::arg("plant"), py::arg("params"),
          py::arg("state_provider"), py::arg("ref_provider") = py::none(),
          "Factory: construct an LQRAdapter (shared_ptr<IController>) in one call.");

    // -----------------------------------------------------------------------
    // DiscreteLQG
    // -----------------------------------------------------------------------
    py::class_<ctrl::DiscreteLQG, std::shared_ptr<ctrl::DiscreteLQG>>(m, "DiscreteLQG", R"doc(
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
    // MRACController  (E3 from CONTROL_STRATEGIES_DEEP_DIVE.md)
    // -----------------------------------------------------------------------
    py::class_<ctrl::MRACParams>(m, "MRACParams",
        "Parameters for MRACController (Lyapunov-based MRAC with sigma-modification).")
        .def(py::init<>())
        .def_readwrite("a_m",       &ctrl::MRACParams::a_m,
                       "Reference model pole |a_m| < 1. y_m[k+1]=a_m*y_m+b_m*r.")
        .def_readwrite("b_m",       &ctrl::MRACParams::b_m,
                       "Reference model gain. Set b_m = 1 - a_m for DC gain 1.")
        .def_readwrite("gamma_r",   &ctrl::MRACParams::gamma_r,
                       "Adaptation rate for theta_r (feedforward). Positive for positive-gain plant.")
        .def_readwrite("gamma_y",   &ctrl::MRACParams::gamma_y,
                       "Adaptation rate for theta_y (feedback). Positive for positive-gain plant.")
        .def_readwrite("sigma",     &ctrl::MRACParams::sigma,
                       "sigma-modification gain >= 0 (0 = off; 0.005-0.05 typical).")
        .def_readwrite("theta_max", &ctrl::MRACParams::theta_max,
                       "Parameter projection bound: ||[theta_r, theta_y]||_2 <= theta_max.")
        .def_readwrite("theta_r0",  &ctrl::MRACParams::theta_r0,
                       "Initial theta_r (0 -> library sets b_m at construction).")
        .def_readwrite("theta_y0",  &ctrl::MRACParams::theta_y0, "Initial theta_y.")
        .def_readwrite("uMin",      &ctrl::MRACParams::uMin, "Lower output saturation.")
        .def_readwrite("uMax",      &ctrl::MRACParams::uMax, "Upper output saturation.");

    py::class_<ctrl::MRACController, ctrl::IController,
               std::shared_ptr<ctrl::MRACController>>(m, "MRACController", R"doc(
Discrete-time MRAC with two-parameter adaptation (theta_r, theta_y).

Uses the ADRC convention: compute(y_plant) takes plant output, not error.
Call set_reference(r) before each compute().

Example
-------
>>> mp = ctrl.MRACParams(); mp.a_m=0.5; mp.b_m=0.5; mp.gamma_r=3.0; mp.gamma_y=1.5
>>> mrac = ctrl.MRACController(mp, Ts=0.1)
>>> for k in range(N):
...     mrac.set_reference(r)
...     u = mrac.compute(y)   # takes plant output, returns control
...     y = plant_step(y, u)
)doc")
        .def(py::init<const ctrl::MRACParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",        &ctrl::MRACController::compute, py::arg("y_plant"),
             "Compute control u[k] given plant output y[k]. Call set_reference(r) first.")
        .def("reset",          &ctrl::MRACController::reset)
        .def("sample_time",    &ctrl::MRACController::sampleTime)
        .def("set_reference",  &ctrl::MRACController::setReference, py::arg("r"),
             "Set the current reference r[k]. Must be called before each compute().")
        .def("theta_r",        &ctrl::MRACController::theta_r,
             "Current feedforward parameter theta_r.")
        .def("theta_y",        &ctrl::MRACController::theta_y,
             "Current feedback parameter theta_y.")
        .def("model_output",   &ctrl::MRACController::modelOutput,
             "Reference model output y_m[k].")
        .def("model_error",    &ctrl::MRACController::modelError,
             "Tracking error e_m = y - y_m from last compute().")
        .def("set_params",     &ctrl::MRACController::setParams, py::arg("params"),
             "Update MRACParams at runtime.")
        .def("params",         &ctrl::MRACController::params,
             "Current MRACParams.");

    // -----------------------------------------------------------------------
    // FeedbackLinearisationController  (E4 from CONTROL_STRATEGIES_DEEP_DIVE.md)
    // -----------------------------------------------------------------------
    py::class_<ctrl::FLParams>(m, "FLParams",
        "Parameters for FeedbackLinearisationController.")
        .def(py::init<>())
        .def_readwrite("uMin",               &ctrl::FLParams::uMin,
                       "Lower saturation on physical control u.")
        .def_readwrite("uMax",               &ctrl::FLParams::uMax,
                       "Upper saturation on physical control u.")
        .def_readwrite("regularisation_eps", &ctrl::FLParams::regularisationEps,
                       "Minimum |g(x)| before clamping (prevents division by zero).");

    py::class_<ctrl::FeedbackLinearisationController, ctrl::IController,
               std::shared_ptr<ctrl::FeedbackLinearisationController>>(
        m, "FeedbackLinearisationController", R"doc(
Exact feedback linearisation controller - SISO, relative degree 1.

Cancels nonlinear terms in xdot = f(x) + g(x).u, then applies an inner linear
IController on the resulting virtual integrator error dynamics.

Usage
-----
>>> def f(x, u_prev): return -float(x[0])**3
>>> def g(x, u_prev): return 1.0
>>> inner = ctrl.DiscretePID(pp, Ts)
>>> fl = ctrl.FeedbackLinearisationController(f, g, inner, flp, Ts)
>>> fl.set_state(x)        # must call before each compute()
>>> u = fl.compute(ref - y)

Feasibility conditions
----------------------
- Relative degree 1: u appears in ydot without further integration.
- Minimum phase: zero dynamics must be asymptotically stable.
- g(x) != 0 across the operating region; tune regularisation_eps accordingly.
- setState() must be called with the current plant state before each step.
)doc")
        .def(py::init<ctrl::FeedbackLinearisationController::DriftFn,
                      ctrl::FeedbackLinearisationController::GainFn,
                      std::shared_ptr<ctrl::IController>,
                      const ctrl::FLParams &,
                      double>(),
             py::arg("f"), py::arg("g"), py::arg("inner"),
             py::arg("params"), py::arg("Ts"),
             "Construct the FL controller with drift functor f, gain functor g, "
             "inner IController, FLParams, and sample time Ts.")
        .def("compute",      &ctrl::FeedbackLinearisationController::compute,
             py::arg("error"),
             "Compute u[k]. Call set_state(x) first each step.")
        .def("reset",        &ctrl::FeedbackLinearisationController::reset)
        .def("sample_time",  &ctrl::FeedbackLinearisationController::sampleTime)
        .def("set_state",    &ctrl::FeedbackLinearisationController::setState,
             py::arg("x"),
             "Inject current plant state x[k] (must be called before each compute()).")
        .def("state",        &ctrl::FeedbackLinearisationController::state,
             "Return the state last set by set_state().")
        .def("last_output",  &ctrl::FeedbackLinearisationController::lastOutput,
             "Return u[k-1] from the previous compute() call.")
        .def("set_params",   &ctrl::FeedbackLinearisationController::setParams,
             py::arg("params"), "Update FLParams at runtime.")
        .def("params",       &ctrl::FeedbackLinearisationController::params,
             "Current FLParams.");

    // -----------------------------------------------------------------------
    // BacksteppingController (Phase 3 NC1)
    // -----------------------------------------------------------------------
    py::class_<ctrl::BacksteppingParams>(m, "BacksteppingParams",
        "Parameters for BacksteppingController.")
        .def(py::init<>())
        .def_readwrite("k_gains", &ctrl::BacksteppingParams::k_gains,
                       "One stabilizing gain per recursion stage (size N).")
        .def_readwrite("uMin",    &ctrl::BacksteppingParams::uMin)
        .def_readwrite("uMax",    &ctrl::BacksteppingParams::uMax);

    py::class_<ctrl::BacksteppingController, ctrl::IController,
               std::shared_ptr<ctrl::BacksteppingController>>(
        m, "BacksteppingController", R"doc(
Recursive Lyapunov design for N-stage strict-feedback systems
(x1' = f_0(x)+g_0(x)*x2, ..., xN' = f_{N-1}(x)+g_{N-1}(x)*u).

Usage
-----
>>> f = [lambda x, s: ..., lambda x, s: ...]
>>> g = [lambda x, s: ..., lambda x, s: ...]
>>> bc = ctrl.BacksteppingController(f, g, params, Ts)
>>> bc.set_state(x)
>>> u = bc.compute(r - x[0])
)doc")
        .def(py::init<std::vector<ctrl::BacksteppingController::DriftFn>,
                      std::vector<ctrl::BacksteppingController::GainFn>,
                      const ctrl::BacksteppingParams &, double>(),
             py::arg("f"), py::arg("g"), py::arg("params"), py::arg("Ts"))
        .def("compute",     &ctrl::BacksteppingController::compute, py::arg("error"))
        .def("reset",       &ctrl::BacksteppingController::reset)
        .def("sample_time", &ctrl::BacksteppingController::sampleTime)
        .def("set_state",   &ctrl::BacksteppingController::setState, py::arg("x"))
        .def("state",       &ctrl::BacksteppingController::state);

    // -----------------------------------------------------------------------
    // PassivityBasedController (Phase 3 NC2)
    // -----------------------------------------------------------------------
    py::class_<ctrl::PBCParams>(m, "PBCParams",
        "Parameters for PassivityBasedController.")
        .def(py::init<>())
        .def_readwrite("Kp",   &ctrl::PBCParams::Kp,
                       "Energy-shaping (stiffness) injection gain, PSD (n x n).")
        .def_readwrite("Kd",   &ctrl::PBCParams::Kd,
                       "Damping injection gain, PSD (n x n).")
        .def_readwrite("uMin", &ctrl::PBCParams::uMin)
        .def_readwrite("uMax", &ctrl::PBCParams::uMax);

    py::class_<ctrl::PassivityBasedController, ctrl::IController,
               std::shared_ptr<ctrl::PassivityBasedController>>(
        m, "PassivityBasedController", R"doc(
PD+ energy-shaping / damping-injection regulation for Euler-Lagrange systems
(M(q)*qddot + C(q,qdot)*qdot + dV(q) = u). Regulates a constant desired configuration q_d.

Usage
-----
>>> pbc = ctrl.PassivityBasedController(M, dV, C, params, Ts)
>>> pbc.set_desired(q_d)
>>> u = pbc.compute_vec(np.concatenate([q, qdot]))
)doc")
        .def(py::init<ctrl::PassivityBasedController::MassMatrixFn,
                      ctrl::PassivityBasedController::PotentialGradFn,
                      ctrl::PassivityBasedController::CoriolisFn,
                      const ctrl::PBCParams &, double>(),
             py::arg("M"), py::arg("dV"), py::arg("C"), py::arg("params"), py::arg("Ts"))
        .def("compute_vec",     &ctrl::PassivityBasedController::computeVec, py::arg("state"),
             "Compute u[k] from the raw stacked state [q; qdot].")
        .def("compute",         &ctrl::PassivityBasedController::compute, py::arg("signal"),
             "Always raises - PassivityBasedController is MIMO-only, call compute_vec().")
        .def("reset",           &ctrl::PassivityBasedController::reset)
        .def("sample_time",     &ctrl::PassivityBasedController::sampleTime)
        .def("set_desired",     &ctrl::PassivityBasedController::setDesired, py::arg("q_d"))
        .def("storage_energy",  &ctrl::PassivityBasedController::storageEnergy,
             "Shaped total-energy storage function from the last successful compute_vec() call.");

    // -----------------------------------------------------------------------
    // CLFController (Phase 3 NC4)
    // -----------------------------------------------------------------------
    py::class_<ctrl::CLFParams>(m, "CLFParams",
        "Parameters for CLFController.")
        .def(py::init<>())
        .def_readwrite("alpha", &ctrl::CLFParams::alpha, "Decay rate in LfV + LgV*u <= -alpha*V.")
        .def_readwrite("uMin",  &ctrl::CLFParams::uMin)
        .def_readwrite("uMax",  &ctrl::CLFParams::uMax)
        .def_readwrite("useQP", &ctrl::CLFParams::useQP,
                       "No behavioral effect in v1 (SISO closed-form only).");

    py::class_<ctrl::CLFController, ctrl::IController,
               std::shared_ptr<ctrl::CLFController>>(
        m, "CLFController", R"doc(
CLF synthesis via Sontag's universal formula (SISO).

Given a candidate Lyapunov function V(x) and its Lie derivatives LfV/LgV, synthesizes a
stabilizing law toward V's equilibrium.

Usage
-----
>>> clf = ctrl.CLFController(V, LfV, LgV, params, Ts)
>>> clf.set_state(x)
>>> u = clf.compute(0.0)   # the argument is unused, see class docs
)doc")
        .def(py::init<ctrl::CLFController::VFn, ctrl::CLFController::LfVFn,
                      ctrl::CLFController::LgVFn, const ctrl::CLFParams &, double>(),
             py::arg("V"), py::arg("LfV"), py::arg("LgV"), py::arg("params"), py::arg("Ts"))
        .def("compute",     &ctrl::CLFController::compute, py::arg("error"))
        .def("reset",       &ctrl::CLFController::reset)
        .def("sample_time", &ctrl::CLFController::sampleTime)
        .def("set_state",   &ctrl::CLFController::setState, py::arg("x"))
        .def("state",       &ctrl::CLFController::state)
        .def("is_healthy",  &ctrl::CLFController::isHealthy);

    // -----------------------------------------------------------------------
    // NMPCParams + NonlinearMPC
    // -----------------------------------------------------------------------
    py::class_<ctrl::NMPCParams>(m, "NMPCParams",
        "Parameters for NonlinearMPC (RTI-based NMPC).")
        .def(py::init<>())
        .def_readwrite("Np",         &ctrl::NMPCParams::Np)
        .def_readwrite("Nu",         &ctrl::NMPCParams::Nu)
        .def_readwrite("rho_y",      &ctrl::NMPCParams::rho_y)
        .def_readwrite("rho_u",      &ctrl::NMPCParams::rho_u)
        .def_readwrite("uMin",       &ctrl::NMPCParams::uMin)
        .def_readwrite("uMax",       &ctrl::NMPCParams::uMax)
        .def_readwrite("qpMaxIter",  &ctrl::NMPCParams::qpMaxIter)
        .def_readwrite("qpTol",      &ctrl::NMPCParams::qpTol)
        .def_readwrite("Ts",         &ctrl::NMPCParams::Ts)
        .def_readwrite("n_states",   &ctrl::NMPCParams::n_states)
        .def_readwrite("n_inputs",   &ctrl::NMPCParams::n_inputs)
        .def_readwrite("n_outputs",  &ctrl::NMPCParams::n_outputs);

    py::class_<ctrl::NonlinearMPC, ctrl::IController,
               std::shared_ptr<ctrl::NonlinearMPC>>(m, "NonlinearMPC", R"doc(
Nonlinear MPC via Real-Time Iteration (RTI).

Usage
-----
>>> nmpc = ctrl.NonlinearMPC(params, f, C)   # C is optional (default: identity)
>>> nmpc.set_state(x)
>>> nmpc.set_reference(y_ref)
>>> u = nmpc.compute(error)                  # SISO
>>> u_vec = nmpc.compute_ref(x, y_ref)       # MIMO
)doc")
        .def(py::init([](const ctrl::NMPCParams &p,
                         py::object f_py) {
            auto f = [f_py](const Eigen::VectorXd &x,
                            const Eigen::VectorXd &u) -> Eigen::VectorXd {
                return f_py(x, u).cast<Eigen::VectorXd>();
            };
            return std::make_shared<ctrl::NonlinearMPC>(p, f);
        }), py::arg("params"), py::arg("f"),
            "Construct NMPC with identity output (y = x).")
        .def(py::init([](const ctrl::NMPCParams &p,
                         py::object f_py,
                         const Eigen::MatrixXd &C_out) {
            auto f = [f_py](const Eigen::VectorXd &x,
                            const Eigen::VectorXd &u) -> Eigen::VectorXd {
                return f_py(x, u).cast<Eigen::VectorXd>();
            };
            return std::make_shared<ctrl::NonlinearMPC>(p, f, C_out);
        }), py::arg("params"), py::arg("f"), py::arg("C"),
            "Construct NMPC with explicit output matrix C (p x n).")
        .def("compute",          &ctrl::NonlinearMPC::compute, py::arg("error"))
        .def("compute_ref",      &ctrl::NonlinearMPC::computeRef,
             py::arg("x"), py::arg("y_ref"), "MIMO interface - return u_0* (m x 1).")
        .def("set_state",        &ctrl::NonlinearMPC::setState, py::arg("x"))
        .def("set_reference",    &ctrl::NonlinearMPC::setReference, py::arg("y_ref"))
        .def("reset",            &ctrl::NonlinearMPC::reset)
        .def("sample_time",      &ctrl::NonlinearMPC::sampleTime)
        .def("last_output",      &ctrl::NonlinearMPC::lastOutput)
        .def("last_qp_converged", &ctrl::NonlinearMPC::lastQPConverged)
        .def("last_qp_iters",    &ctrl::NonlinearMPC::lastQPIters)
        .def("last_control_vec", &ctrl::NonlinearMPC::lastControlVec,
             py::return_value_policy::copy);

    // -----------------------------------------------------------------------
    // AdaptiveSPParams + AdaptiveSmithPredictor
    // -----------------------------------------------------------------------
    py::class_<ctrl::AdaptiveSPParams>(m, "AdaptiveSPParams",
        "Adaptation parameters for AdaptiveSmithPredictor.")
        .def(py::init<>())
        .def_readwrite("max_delay_steps",    &ctrl::AdaptiveSPParams::maxDelaySteps)
        .def_readwrite("estimate_interval",  &ctrl::AdaptiveSPParams::estimateInterval)
        .def_readwrite("buffer_len",         &ctrl::AdaptiveSPParams::bufferLen);

    py::class_<ctrl::AdaptiveSmithPredictor, ctrl::IController,
               std::shared_ptr<ctrl::AdaptiveSmithPredictor>>(
        m, "AdaptiveSmithPredictor", R"doc(
Smith Predictor with online dead-time estimation via cross-correlation.

Usage
-----
>>> asp = ctrl.AdaptiveSmithPredictor(inner, delay_model, 5, Ts, params)
>>> asp.set_plant_output(y_meas)
>>> u = asp.compute(r - y_meas)
>>> d_est = asp.estimated_delay_steps()
)doc")
        .def(py::init<std::shared_ptr<ctrl::IController>,
                      const ctrl::StateSpace &,
                      int, double,
                      const ctrl::AdaptiveSPParams &>(),
             py::arg("inner"), py::arg("delay_model"),
             py::arg("initial_delay_steps"), py::arg("Ts"),
             py::arg("params") = ctrl::AdaptiveSPParams())
        .def("compute",                  &ctrl::AdaptiveSmithPredictor::compute,
             py::arg("error"))
        .def("reset",                    &ctrl::AdaptiveSmithPredictor::reset)
        .def("sample_time",              &ctrl::AdaptiveSmithPredictor::sampleTime)
        .def("last_output",              &ctrl::AdaptiveSmithPredictor::lastOutput)
        .def("set_plant_output",         &ctrl::AdaptiveSmithPredictor::setPlantOutput,
             py::arg("y"), "Feed actual plant output for accurate cross-correlation.")
        .def("estimated_delay_steps",    &ctrl::AdaptiveSmithPredictor::estimatedDelaySteps)
        .def("estimated_delay_time",     &ctrl::AdaptiveSmithPredictor::estimatedDelayTime);

    // -----------------------------------------------------------------------
    // AutoTunerParams + TunerResult + AutoTuner
    // -----------------------------------------------------------------------
    py::class_<ctrl::AutoTunerParams>(m, "AutoTunerParams",
        "Parameters for AutoTuner (CMA-ES black-box optimizer).")
        .def(py::init<>())
        .def_readwrite("n",       &ctrl::AutoTunerParams::n)
        .def_readwrite("sigma0",  &ctrl::AutoTunerParams::sigma0)
        .def_readwrite("maxIter", &ctrl::AutoTunerParams::maxIter)
        .def_readwrite("tol",     &ctrl::AutoTunerParams::tol)
        .def_readwrite("lower",   &ctrl::AutoTunerParams::lower)
        .def_readwrite("upper",   &ctrl::AutoTunerParams::upper);

    py::class_<ctrl::TunerResult>(m, "TunerResult", "Result of an AutoTuner::tune() run.")
        .def_readonly("params",    &ctrl::TunerResult::params)
        .def_readonly("cost",      &ctrl::TunerResult::cost)
        .def_readonly("n_evals",   &ctrl::TunerResult::nEvals)
        .def_readonly("n_gens",    &ctrl::TunerResult::nGens)
        .def_readonly("converged", &ctrl::TunerResult::converged);

    py::class_<ctrl::AutoTuner, std::shared_ptr<ctrl::AutoTuner>>(m, "AutoTuner", R"doc(
CMA-ES black-box optimizer for controller parameter tuning.

Usage
-----
>>> atp = ctrl.AutoTunerParams(); atp.n = 3; atp.sigma0 = 0.5
>>> atp.lower = np.array([0., 0., 0.]); atp.upper = np.array([5., 2., 1.])
>>> tuner = ctrl.AutoTuner(atp, seed=42)
>>> result = tuner.tune(lambda p: iae_simulation(p), x0)
>>> print(result.params, result.cost, result.converged)
)doc")
        .def(py::init<const ctrl::AutoTunerParams &, unsigned>(),
             py::arg("params"), py::arg("seed") = 42u)
        .def("tune", [](ctrl::AutoTuner &self,
                        py::object cost_py,
                        const Eigen::VectorXd &x0) {
            auto cost_fn = [cost_py](const Eigen::VectorXd &p) -> double {
                return cost_py(p).cast<double>();
            };
            return self.tune(cost_fn, x0);
        }, py::arg("cost"), py::arg("x0"),
           "Run CMA-ES to minimize cost(params) starting from x0.");

    // -----------------------------------------------------------------------
    // BayesOptParams + BayesianOptimizer
    // -----------------------------------------------------------------------
    py::enum_<ctrl::BayesOptParams::Acquisition>(m, "BayesAcquisition",
        "Acquisition function for BayesianOptimizer.")
        .value("UCB", ctrl::BayesOptParams::Acquisition::UCB,
               "Lower confidence bound: mu - kappa*sigma.")
        .value("EI",  ctrl::BayesOptParams::Acquisition::EI,
               "Expected improvement over current best.")
        .export_values();

    py::class_<ctrl::BayesOptParams>(m, "BayesOptParams",
        "Parameters for BayesianOptimizer.")
        .def(py::init<>())
        .def_readwrite("n",              &ctrl::BayesOptParams::n)
        .def_readwrite("n_init",         &ctrl::BayesOptParams::n_init,
                       "Random initialisation evaluations before BO starts.")
        .def_readwrite("maxIter",        &ctrl::BayesOptParams::maxIter,
                       "BO iterations after initialisation. Total evals = n_init + maxIter.")
        .def_readwrite("n_acq_restarts", &ctrl::BayesOptParams::n_acq_restarts,
                       "Candidate points for acquisition maximisation each step.")
        .def_readwrite("kappa",          &ctrl::BayesOptParams::kappa,
                       "UCB exploration weight (larger = more exploration).")
        .def_readwrite("xi",             &ctrl::BayesOptParams::xi,
                       "EI improvement margin xi.")
        .def_readwrite("acq",            &ctrl::BayesOptParams::acq,
                       "Acquisition function: BayesAcquisition.UCB or .EI.")
        .def_readwrite("lower",          &ctrl::BayesOptParams::lower)
        .def_readwrite("upper",          &ctrl::BayesOptParams::upper)
        .def_readwrite("gp",             &ctrl::BayesOptParams::gp,
                       "GP surrogate hyperparameters (GaussianProcess.Params).")
        .def_readwrite("seed",           &ctrl::BayesOptParams::seed);

    py::class_<ctrl::BayesianOptimizer, std::shared_ptr<ctrl::BayesianOptimizer>>(m, "BayesianOptimizer", R"doc(
Bayesian Optimization with GP surrogate and UCB/EI acquisition.

Preferred over AutoTuner (CMA-ES) when cost evaluations are expensive
(hardware-in-the-loop, long simulations). Uses only n_init + maxIter evals.

Usage
-----
>>> bp = ctrl.BayesOptParams(); bp.n = 3; bp.maxIter = 40
>>> bp.lower = np.array([0., 0., 0.]); bp.upper = np.array([5., 2., 1.])
>>> bo = ctrl.BayesianOptimizer(bp)
>>> result = bo.tune(lambda p: iae_sim(p), x0)
>>> print(result.params, result.cost)
)doc")
        .def(py::init<const ctrl::BayesOptParams&>(), py::arg("params"))
        .def("tune",
             [](ctrl::BayesianOptimizer& self,
                py::object cost_py,
                const Eigen::VectorXd& x0) {
                 auto cost_fn = [cost_py](const Eigen::VectorXd& p) -> double {
                     return cost_py(p).cast<double>();
                 };
                 return self.tune(cost_fn, x0);
             },
             py::arg("cost"), py::arg("x0"),
             "Minimise cost(params) starting from x0. Returns TunerResult.");

    // -----------------------------------------------------------------------
    // GeneticAlgorithm
    // -----------------------------------------------------------------------
    py::class_<ctrl::GAParams>(m, "GAParams", "Parameters for GeneticAlgorithm.")
        .def(py::init<>())
        .def_readwrite("n_dim",       &ctrl::GAParams::n_dim)
        .def_readwrite("population",  &ctrl::GAParams::population)
        .def_readwrite("max_gen",     &ctrl::GAParams::max_gen)
        .def_readwrite("crossover",   &ctrl::GAParams::crossover)
        .def_readwrite("mutation",    &ctrl::GAParams::mutation)
        .def_readwrite("alpha",       &ctrl::GAParams::alpha)
        .def_readwrite("elite_frac",  &ctrl::GAParams::elite_frac)
        .def_readwrite("tol",         &ctrl::GAParams::tol)
        .def_readwrite("lower",       &ctrl::GAParams::lower)
        .def_readwrite("upper",       &ctrl::GAParams::upper)
        .def_readwrite("seed",        &ctrl::GAParams::seed);

    py::class_<ctrl::GeneticAlgorithm, std::shared_ptr<ctrl::GeneticAlgorithm>>(m, "GeneticAlgorithm", R"doc(
Real-valued Genetic Algorithm: BLX-alpha crossover, tournament selection, elitism.
Minimises a scalar cost function over a box-bounded parameter space.

Usage
-----
>>> p = ctrl.GAParams(); p.n_dim = 2; p.lower = np.array([0.,0.]); p.upper = np.array([5.,5.])
>>> ga = ctrl.GeneticAlgorithm(p)
>>> r = ga.optimize(lambda x: (x[0]-2.)**2 + (x[1]-3.)**2)
>>> print(r.params, r.cost)
)doc")
        .def(py::init<const ctrl::GAParams&>(), py::arg("params"))
        .def("optimize",
             [](ctrl::GeneticAlgorithm& self, py::object cost_py) {
                 auto fn = [cost_py](const Eigen::VectorXd& p) -> double {
                     return cost_py(p).cast<double>();
                 };
                 return self.optimize(fn);
             },
             py::arg("cost"),
             "Minimise cost(params) over the box [lower, upper]. Returns TunerResult.");

    // -----------------------------------------------------------------------
    // NelderMead (Phase 3 MO2)
    // -----------------------------------------------------------------------
    py::class_<ctrl::NelderMeadParams>(m, "NelderMeadParams", "Parameters for NelderMead.")
        .def(py::init<>())
        .def_readwrite("n_dim",    &ctrl::NelderMeadParams::n_dim)
        .def_readwrite("max_iter", &ctrl::NelderMeadParams::max_iter)
        .def_readwrite("tol",      &ctrl::NelderMeadParams::tol)
        .def_readwrite("alpha",    &ctrl::NelderMeadParams::alpha)
        .def_readwrite("gamma",    &ctrl::NelderMeadParams::gamma)
        .def_readwrite("rho",      &ctrl::NelderMeadParams::rho)
        .def_readwrite("sigma",    &ctrl::NelderMeadParams::sigma)
        .def_readwrite("seed",     &ctrl::NelderMeadParams::seed);

    py::class_<ctrl::NelderMead, std::shared_ptr<ctrl::NelderMead>>(m, "NelderMead", R"doc(
Derivative-free Nelder-Mead simplex search - no bounds, no population sizing, just an
initial point. A lighter-weight alternative to GeneticAlgorithm/AutoTuner for quick
low-dimensional (n < 10) tuning.

Usage
-----
>>> p = ctrl.NelderMeadParams(); p.n_dim = 2
>>> nm = ctrl.NelderMead(p)
>>> r = nm.optimize(lambda x: (x[0]-2.)**2 + (x[1]-3.)**2, np.array([0., 0.]))
>>> print(r.params, r.cost)
)doc")
        .def(py::init<const ctrl::NelderMeadParams&>(), py::arg("params"))
        .def("optimize",
             [](ctrl::NelderMead& self, py::object cost_py, const Eigen::VectorXd& x0) {
                 auto fn = [cost_py](const Eigen::VectorXd& p) -> double {
                     return cost_py(p).cast<double>();
                 };
                 return self.optimize(fn, x0);
             },
             py::arg("cost"), py::arg("x0"),
             "Minimise cost(params) starting from x0. Returns TunerResult.");

    // -----------------------------------------------------------------------
    // NSGA2 (Phase 3 Roadmap Phase 2 MO1)
    // -----------------------------------------------------------------------
    py::class_<ctrl::NSGA2Params>(m, "NSGA2Params", "Parameters for NSGA2.")
        .def(py::init<>())
        .def_readwrite("n_dim",        &ctrl::NSGA2Params::n_dim)
        .def_readwrite("n_objectives", &ctrl::NSGA2Params::n_objectives)
        .def_readwrite("population",   &ctrl::NSGA2Params::population)
        .def_readwrite("max_gen",      &ctrl::NSGA2Params::max_gen)
        .def_readwrite("crossover",    &ctrl::NSGA2Params::crossover)
        .def_readwrite("mutation",     &ctrl::NSGA2Params::mutation)
        .def_readwrite("alpha",        &ctrl::NSGA2Params::alpha)
        .def_readwrite("lower",        &ctrl::NSGA2Params::lower)
        .def_readwrite("upper",        &ctrl::NSGA2Params::upper)
        .def_readwrite("seed",         &ctrl::NSGA2Params::seed);

    py::class_<ctrl::ParetoResult>(m, "ParetoResult", "Result from NSGA2::optimize.")
        .def_readonly("front_params",     &ctrl::ParetoResult::front_params)
        .def_readonly("front_objectives", &ctrl::ParetoResult::front_objectives)
        .def_readonly("nGens",            &ctrl::ParetoResult::nGens)
        .def_readonly("nEvals",           &ctrl::ParetoResult::nEvals);

    py::class_<ctrl::NSGA2>(m, "NSGA2", R"doc(
NSGA-II multi-objective (Pareto) evolutionary optimizer - returns a Pareto front instead of a
single best point.

Usage
-----
>>> p = ctrl.NSGA2Params(); p.n_dim = 2; p.n_objectives = 2
>>> p.lower = np.zeros(2); p.upper = np.ones(2) * 10
>>> nsga = ctrl.NSGA2(p)
>>> result = nsga.optimize(lambda x: np.array([x[0]**2, (x[0]-2)**2]))
>>> print(result.front_params, result.front_objectives)
)doc")
        .def(py::init<const ctrl::NSGA2Params &>(), py::arg("params"))
        .def("optimize",
             [](ctrl::NSGA2 &self, py::object cost_py) {
                 auto fn = [cost_py](const Eigen::VectorXd &p) -> Eigen::VectorXd {
                     return cost_py(p).cast<Eigen::VectorXd>();
                 };
                 return self.optimize(fn);
             },
             py::arg("cost"),
             "Minimise every component of cost(params). Returns a ParetoResult.");

    // -----------------------------------------------------------------------
    // tuneConstrained (Phase 3 Roadmap Phase 2 MO3)
    // -----------------------------------------------------------------------
    py::class_<ctrl::ConstrainedTuneParams>(m, "ConstrainedTuneParams",
        "Parameters for tune_constrained.")
        .def(py::init<>())
        .def_readwrite("penalty_init",   &ctrl::ConstrainedTuneParams::penalty_init)
        .def_readwrite("penalty_growth", &ctrl::ConstrainedTuneParams::penalty_growth)
        .def_readwrite("outer_iters",    &ctrl::ConstrainedTuneParams::outer_iters)
        .def_readwrite("feasTol",        &ctrl::ConstrainedTuneParams::feasTol)
        .def_property("constraints",
            [](const ctrl::ConstrainedTuneParams &) -> py::object { return py::none(); },
            [](ctrl::ConstrainedTuneParams &self, py::object g_py) {
                self.constraints = [g_py](const Eigen::VectorXd &theta) -> Eigen::VectorXd {
                    return g_py(theta).cast<Eigen::VectorXd>();
                };
            },
            "Constraint function g(theta) -> array; feasible iff every entry <= 0.");

    m.def("tune_constrained",
          [](py::object optimizer_run_py, py::object objective_py,
             const ctrl::ConstrainedTuneParams &params, const Eigen::VectorXd &x0) {
              auto optimizerRun = [optimizer_run_py](const ctrl::AutoTuner::CostFn &c,
                                                       const Eigen::VectorXd &xx0) -> ctrl::TunerResult {
                  auto cPy = [c](const Eigen::VectorXd &x) -> double { return c(x); };
                  return optimizer_run_py(py::cpp_function(cPy), xx0).cast<ctrl::TunerResult>();
              };
              auto objective = [objective_py](const Eigen::VectorXd &x) -> double {
                  return objective_py(x).cast<double>();
              };
              return ctrl::tuneConstrained(optimizerRun, objective, params, x0);
          },
          py::arg("optimizer_run"), py::arg("objective"), py::arg("params"), py::arg("x0"),
          "Minimise objective(theta) subject to params.constraints(theta) <= 0, via an "
          "exterior-penalty wrapper around optimizer_run(cost_fn, x0) -> TunerResult.");

    // -----------------------------------------------------------------------
    // SelfTuningRegulator (Phase 3 Roadmap Phase 2 OC1)
    // -----------------------------------------------------------------------
    py::enum_<ctrl::STRMode>(m, "STRMode")
        .value("MinimumVariance", ctrl::STRMode::MinimumVariance)
        .value("PolePlacement",   ctrl::STRMode::PolePlacement);

    py::class_<ctrl::STRParams>(m, "STRParams", "Parameters for SelfTuningRegulator.")
        .def(py::init<>())
        .def_readwrite("na",             &ctrl::STRParams::na)
        .def_readwrite("nb",             &ctrl::STRParams::nb)
        .def_readwrite("mode",           &ctrl::STRParams::mode)
        .def_readwrite("desired_poles",  &ctrl::STRParams::desired_poles,
                       "PolePlacement mode only; size must equal na + nb - 1.")
        .def_readwrite("lambda_",        &ctrl::STRParams::lambda)
        .def_readwrite("bMin",           &ctrl::STRParams::bMin)
        .def_readwrite("uMin",           &ctrl::STRParams::uMin)
        .def_readwrite("uMax",           &ctrl::STRParams::uMax)
        .def_readwrite("probeAmplitude", &ctrl::STRParams::probeAmplitude,
                       "Persistent probing/dither amplitude added to u before saturation "
                       "(0 = disabled). See the class-level warning about persistent excitation.")
        .def_readwrite("probeSeed",      &ctrl::STRParams::probeSeed);

    py::class_<ctrl::SelfTuningRegulator, ctrl::IController,
               std::shared_ptr<ctrl::SelfTuningRegulator>>(
        m, "SelfTuningRegulator", R"doc(
Online self-tuning regulator: RecursiveLeastSquares identification + a selectable
minimum-variance or pole-placement control law, re-derived every step.

Usage (PlantOutput convention, like MRACController)
----------------------------------------------------
>>> p = ctrl.STRParams(); p.na = 2; p.nb = 1
>>> str_ctrl = ctrl.SelfTuningRegulator(p, Ts)
>>> str_ctrl.set_reference(r)
>>> u = str_ctrl.compute(y)
)doc")
        .def(py::init<const ctrl::STRParams &, double>(), py::arg("params"), py::arg("Ts"))
        .def("set_reference",          &ctrl::SelfTuningRegulator::setReference, py::arg("r"))
        .def("compute",                &ctrl::SelfTuningRegulator::compute, py::arg("y_plant"))
        .def("reset",                  &ctrl::SelfTuningRegulator::reset)
        .def("sample_time",            &ctrl::SelfTuningRegulator::sampleTime)
        .def("estimated_numerator",    &ctrl::SelfTuningRegulator::estimatedNumerator)
        .def("estimated_denominator",  &ctrl::SelfTuningRegulator::estimatedDenominator)
        .def("covariance",             &ctrl::SelfTuningRegulator::covariance);

    // -----------------------------------------------------------------------
    // FaultClassifier + FTCSupervisor (Phase 3 Roadmap Phase 2 DT4)
    // -----------------------------------------------------------------------
    py::enum_<ctrl::FaultType>(m, "FaultType")
        .value("None_",           ctrl::FaultType::None) // "None" shadows Python's None keyword
        .value("SensorBias",      ctrl::FaultType::SensorBias)
        .value("SensorNoise",     ctrl::FaultType::SensorNoise)
        .value("ActuatorLoss",    ctrl::FaultType::ActuatorLoss)
        .value("ActuatorStuck",   ctrl::FaultType::ActuatorStuck);

    py::class_<ctrl::FaultDetectorParams>(m, "FaultDetectorParams",
        "Parameters for FaultClassifier / FTCSupervisor.")
        .def(py::init<>())
        .def_readwrite("residual_threshold", &ctrl::FaultDetectorParams::residual_threshold)
        .def_readwrite("confirm_window",     &ctrl::FaultDetectorParams::confirm_window)
        .def_readwrite("corr_threshold",     &ctrl::FaultDetectorParams::corr_threshold)
        .def_readwrite("bias_threshold",     &ctrl::FaultDetectorParams::bias_threshold)
        .def_readwrite("stuck_du_threshold", &ctrl::FaultDetectorParams::stuck_du_threshold);

    py::class_<ctrl::FaultClassifier>(m, "FaultClassifier", R"doc(
Heuristic per-step fault-type classifier over a rolling (innovation, u_cmd, y_meas) window.

Example
-------
>>> fc = ctrl.FaultClassifier(ctrl.FaultDetectorParams())
>>> fault = fc.classify(innovation, u_cmd, y_meas)
)doc")
        .def(py::init<const ctrl::FaultDetectorParams &>(), py::arg("params") = ctrl::FaultDetectorParams())
        .def("classify", &ctrl::FaultClassifier::classify,
             py::arg("innovation"), py::arg("u_cmd"), py::arg("y_meas"))
        .def("reset",    &ctrl::FaultClassifier::reset);

    py::class_<ctrl::FTCSupervisor, ctrl::IController,
               std::shared_ptr<ctrl::FTCSupervisor>>(
        m, "FTCSupervisor", R"doc(
Fault-tolerant control reconfiguration: classifies faults and switches a ControllerStack's
active entry accordingly, reusing its existing Supervisory-mode bumpless transfer.

Usage
-----
>>> stack = ctrl.ControllerStack(ctrl.StackMode.Supervisory, Ts)
>>> stack.add_controller(primary_pid, "primary")
>>> stack.add_controller(fallback_pid, "fallback")
>>> ftc = ctrl.FTCSupervisor(stack, ctrl.FaultDetectorParams(), Ts)
>>> ftc.register_fault_response(ctrl.FaultType.None_, "primary")
>>> ftc.register_fault_response(ctrl.FaultType.ActuatorLoss, "fallback")
>>> ftc.feed_residual(innovation, u_prev, y_meas)
>>> u = ftc.compute(error)
)doc")
        .def(py::init<std::shared_ptr<ctrl::ControllerStack>, const ctrl::FaultDetectorParams &,
                      double>(),
             py::arg("stack"), py::arg("params"), py::arg("Ts"))
        .def("register_fault_response", &ctrl::FTCSupervisor::registerFaultResponse,
             py::arg("fault"), py::arg("controller_name"))
        .def("feed_residual", &ctrl::FTCSupervisor::feedResidual,
             py::arg("innovation"), py::arg("u_cmd"), py::arg("y_meas"))
        .def("compute",       &ctrl::FTCSupervisor::compute, py::arg("error"))
        .def("reset",         &ctrl::FTCSupervisor::reset)
        .def("sample_time",   &ctrl::FTCSupervisor::sampleTime)
        .def("current_fault", &ctrl::FTCSupervisor::currentFault);

    // -----------------------------------------------------------------------
    // ParticleSwarmOptimizer
    // -----------------------------------------------------------------------
    py::class_<ctrl::PSOParams>(m, "PSOParams", "Parameters for ParticleSwarmOptimizer.")
        .def(py::init<>())
        .def_readwrite("n_dim",        &ctrl::PSOParams::n_dim)
        .def_readwrite("n_particles",  &ctrl::PSOParams::n_particles)
        .def_readwrite("max_iter",     &ctrl::PSOParams::max_iter)
        .def_readwrite("w",            &ctrl::PSOParams::w)
        .def_readwrite("c1",           &ctrl::PSOParams::c1)
        .def_readwrite("c2",           &ctrl::PSOParams::c2)
        .def_readwrite("v_frac",       &ctrl::PSOParams::v_frac)
        .def_readwrite("tol",          &ctrl::PSOParams::tol)
        .def_readwrite("lower",        &ctrl::PSOParams::lower)
        .def_readwrite("upper",        &ctrl::PSOParams::upper)
        .def_readwrite("seed",         &ctrl::PSOParams::seed);

    py::class_<ctrl::ParticleSwarmOptimizer, std::shared_ptr<ctrl::ParticleSwarmOptimizer>>(m, "ParticleSwarmOptimizer", R"doc(
Particle Swarm Optimizer with Clerc-Kennedy constriction coefficients.
Suitable for continuous box-bounded optimisation problems.

Usage
-----
>>> p = ctrl.PSOParams(); p.n_dim = 2; p.lower = np.array([0.,0.]); p.upper = np.array([5.,5.])
>>> pso = ctrl.ParticleSwarmOptimizer(p)
>>> r = pso.optimize(lambda x: (x[0]-2.)**2 + (x[1]-3.)**2)
>>> print(r.params, r.cost)
)doc")
        .def(py::init<const ctrl::PSOParams&>(), py::arg("params"))
        .def("optimize",
             [](ctrl::ParticleSwarmOptimizer& self, py::object cost_py) {
                 auto fn = [cost_py](const Eigen::VectorXd& p) -> double {
                     return cost_py(p).cast<double>();
                 };
                 return self.optimize(fn);
             },
             py::arg("cost"),
             "Minimise cost(params) over the box [lower, upper]. Returns TunerResult.");

    // -----------------------------------------------------------------------
    // DifferentialEvolution
    // -----------------------------------------------------------------------
    py::class_<ctrl::DEParams>(m, "DEParams", "Parameters for DifferentialEvolution.")
        .def(py::init<>())
        .def_readwrite("n_dim",       &ctrl::DEParams::n_dim)
        .def_readwrite("population",  &ctrl::DEParams::population)
        .def_readwrite("max_gen",     &ctrl::DEParams::max_gen)
        .def_readwrite("F",           &ctrl::DEParams::F)
        .def_readwrite("CR",          &ctrl::DEParams::CR)
        .def_readwrite("tol",         &ctrl::DEParams::tol)
        .def_readwrite("lower",       &ctrl::DEParams::lower)
        .def_readwrite("upper",       &ctrl::DEParams::upper)
        .def_readwrite("seed",        &ctrl::DEParams::seed);

    py::class_<ctrl::DifferentialEvolution, std::shared_ptr<ctrl::DifferentialEvolution>>(m, "DifferentialEvolution", R"doc(
Differential Evolution (DE/rand/1/bin) with boundary reflection.
Robust global optimizer for noisy and multimodal cost functions.

Usage
-----
>>> p = ctrl.DEParams(); p.n_dim = 2; p.lower = np.array([0.,0.]); p.upper = np.array([5.,5.])
>>> de = ctrl.DifferentialEvolution(p)
>>> r = de.optimize(lambda x: (x[0]-2.)**2 + (x[1]-3.)**2)
>>> print(r.params, r.cost)
)doc")
        .def(py::init<const ctrl::DEParams&>(), py::arg("params"))
        .def("optimize",
             [](ctrl::DifferentialEvolution& self, py::object cost_py) {
                 auto fn = [cost_py](const Eigen::VectorXd& p) -> double {
                     return cost_py(p).cast<double>();
                 };
                 return self.optimize(fn);
             },
             py::arg("cost"),
             "Minimise cost(params) over the box [lower, upper]. Returns TunerResult.");

    // -----------------------------------------------------------------------
    // TubeMPC - robust MPC with mRPI tube (Mayne, Seron, Rakovic 2005)
    // -----------------------------------------------------------------------
    py::class_<ctrl::TubeMPCParams>(m, "TubeMPCParams",
        "Parameters for TubeMPC: horizon, weights, tube feedback K, and disturbance bound wMax.")
        .def(py::init<>())
        .def_readwrite("Np",            &ctrl::TubeMPCParams::Np)
        .def_readwrite("Nu",            &ctrl::TubeMPCParams::Nu)
        .def_readwrite("Q",             &ctrl::TubeMPCParams::Q)
        .def_readwrite("R",             &ctrl::TubeMPCParams::R)
        .def_readwrite("K",             &ctrl::TubeMPCParams::K)
        .def_readwrite("wMax",          &ctrl::TubeMPCParams::wMax)
        .def_readwrite("uMin",          &ctrl::TubeMPCParams::uMin)
        .def_readwrite("uMax",          &ctrl::TubeMPCParams::uMax)
        .def_readwrite("Ts",            &ctrl::TubeMPCParams::Ts)
        .def_readwrite("qp_max_iter",   &ctrl::TubeMPCParams::qpMaxIter)
        .def_readwrite("qp_tol",        &ctrl::TubeMPCParams::qpTol)
        .def_readwrite("mRPI_tol",      &ctrl::TubeMPCParams::mRPI_tol)
        .def_readwrite("mRPI_max_iter", &ctrl::TubeMPCParams::mRPI_maxIter);

    py::class_<ctrl::TubeMPC, ctrl::IController, std::shared_ptr<ctrl::TubeMPC>>(
        m, "TubeMPC", R"doc(
Tube MPC: robust MPC for plants with bounded additive disturbances.

Guarantees |x[k] - x_nom[k]|_i <= tube_radius()[i] for all k when w[k] in W.
Use setState() + setReference() then compute_control(), or the combined compute_ref().

Parameters
----------
sys : StateSpace
    Discrete-time plant model.
params : TubeMPCParams
    Tube-MPC configuration including tube feedback gain K and disturbance bound wMax.
)doc")
        .def(py::init<const ctrl::StateSpace &, const ctrl::TubeMPCParams &>(),
             py::arg("sys"), py::arg("params"))
        .def("set_state",     &ctrl::TubeMPC::setState,      py::arg("x"),
             "Set measured plant state x (required before compute_control each step).")
        .def("set_reference", &ctrl::TubeMPC::setReference,  py::arg("y_ref"),
             "Set output reference y_ref.")
        .def("compute_ref",   &ctrl::TubeMPC::computeRef,
             py::arg("x"), py::arg("y_ref"),
             py::return_value_policy::copy,
             "Solve tube-MPC QP from x and y_ref; returns composite u = K*(x-x_nom) + v_nom.")
        .def("compute_control", &ctrl::TubeMPC::computeControl,
             py::return_value_policy::copy,
             "Solve tube-MPC QP (requires prior set_state + set_reference).")
        .def("compute",       &ctrl::TubeMPC::compute,        py::arg("error"),
             "IController scalar interface (requires prior set_state).")
        .def("reset",         &ctrl::TubeMPC::reset,
             "Reset nominal state and warm start.")
        .def("sample_time",   &ctrl::TubeMPC::sampleTime,    "Sample time Ts [s].")
        .def("is_healthy",    &ctrl::TubeMPC::isHealthy,     "False if last QP did not converge.")
        .def("last_qp_converged", &ctrl::TubeMPC::lastQPConverged,
             "True if the last FISTA solve converged.")
        .def("tube_radius",   &ctrl::TubeMPC::tubeRadius,
             py::return_value_policy::copy,
             "mRPI tube radius z_max (n,): |x-x_nom|_i <= tube_radius[i] guaranteed.")
        .def("nominal_state", &ctrl::TubeMPC::nominalState,
             py::return_value_policy::copy,
             "Current nominal state x_nom (n,) - disturbance-free predicted state.")
        .def("tightened_umin", &ctrl::TubeMPC::tightenedUMin,
             py::return_value_policy::copy, "Tightened lower u bound after mRPI constraint tightening.")
        .def("tightened_umax", &ctrl::TubeMPC::tightenedUMax,
             py::return_value_policy::copy, "Tightened upper u bound after mRPI constraint tightening.");

    // -----------------------------------------------------------------------
    // ScenarioMPC - scenario-based stochastic MPC
    // -----------------------------------------------------------------------
    py::class_<ctrl::ScenarioMPCParams>(m, "ScenarioMPCParams",
        "Parameters for ScenarioMPC: horizon, weights, noise covariance, and sample count.")
        .def(py::init<>())
        .def_readwrite("Np",         &ctrl::ScenarioMPCParams::Np)
        .def_readwrite("Nu",         &ctrl::ScenarioMPCParams::Nu)
        .def_readwrite("Q",          &ctrl::ScenarioMPCParams::Q)
        .def_readwrite("R",          &ctrl::ScenarioMPCParams::R)
        .def_readwrite("Sigma_w",    &ctrl::ScenarioMPCParams::Sigma_w,
                       "Process noise covariance (n x n). N(0, Sigma_w) drawn per step and horizon.")
        .def_readwrite("N_samples",  &ctrl::ScenarioMPCParams::N_samples,
                       "Number of noise scenarios sampled per control step.")
        .def_readwrite("uMin",       &ctrl::ScenarioMPCParams::uMin)
        .def_readwrite("uMax",       &ctrl::ScenarioMPCParams::uMax)
        .def_readwrite("Ts",         &ctrl::ScenarioMPCParams::Ts)
        .def_readwrite("qp_max_iter",&ctrl::ScenarioMPCParams::qpMaxIter)
        .def_readwrite("qp_tol",     &ctrl::ScenarioMPCParams::qpTol)
        .def_readwrite("seed",       &ctrl::ScenarioMPCParams::seed);

    py::class_<ctrl::ScenarioMPC, ctrl::IController,
               std::shared_ptr<ctrl::ScenarioMPC>>(m, "ScenarioMPC", R"doc(
Scenario-based Stochastic MPC (Calafiore & Campi 2006).

Extends DiscreteMPC by averaging the tracking cost over N_samples Gaussian
noise trajectories. More flexible than TubeMPC (handles unbounded Gaussian
noise; not limited to worst-case bounded disturbances).

Usage
-----
>>> p = ctrl.ScenarioMPCParams()
>>> p.Np = 10; p.Nu = 3; p.Ts = 0.1; p.N_samples = 30
>>> p.Q = np.eye(n); p.R = np.eye(m)*0.1
>>> p.Sigma_w = np.eye(n)*0.01
>>> p.uMin = np.array([-1.0]); p.uMax = np.array([1.0])
>>> smpc = ctrl.ScenarioMPC(sys, p)
>>> smpc.set_state(x); smpc.set_reference(np.array([1.0]))
>>> u = smpc.compute_control()   # or smpc.compute(error) for SISO
)doc")
        .def(py::init<const ctrl::StateSpace &, const ctrl::ScenarioMPCParams &>(),
             py::arg("sys"), py::arg("params"))
        .def("set_state",       &ctrl::ScenarioMPC::setState,      py::arg("x"))
        .def("set_reference",   &ctrl::ScenarioMPC::setReference,  py::arg("y_ref"))
        .def("compute_ref",     &ctrl::ScenarioMPC::computeRef,
             py::arg("x"), py::arg("y_ref"), py::return_value_policy::copy)
        .def("compute_control", &ctrl::ScenarioMPC::computeControl,
             py::return_value_policy::copy)
        .def("compute",         &ctrl::ScenarioMPC::compute,     py::arg("error"))
        .def("reset",           &ctrl::ScenarioMPC::reset)
        .def("sample_time",     &ctrl::ScenarioMPC::sampleTime)
        .def("is_healthy",      &ctrl::ScenarioMPC::isHealthy)
        .def("last_qp_converged",&ctrl::ScenarioMPC::lastQPConverged)
        .def("nominal_state",   &ctrl::ScenarioMPC::nominalState,
             py::return_value_policy::copy,
             "Current nominal state x_nom (n,).")
        .def("last_avg_noise_bias", &ctrl::ScenarioMPC::lastAvgNoiseBias,
             py::return_value_policy::copy,
             "Average noise-propagated output bias from last solve (Np*pp,).");

    // -----------------------------------------------------------------------
    // AntiWindupWrapper - generic conditioning-technique decorator (Hanus 1987)
    // -----------------------------------------------------------------------
    py::class_<ctrl::AntiWindupWrapper, ctrl::IController,
               std::shared_ptr<ctrl::AntiWindupWrapper>>(
        m, "AntiWindupWrapper", R"doc(
Generic anti-windup decorator for any IController (conditioning technique).

Wraps an arbitrary IController with output saturation and the Hanus (1987)
conditioning technique. The wrapper is transparent when not saturating and
prevents integrator windup by injecting a correction into the error input:

    e_in[k] = e[k] + Kb * (u_sat[k-1] - u_raw[k-1])

Suitable for DiscreteLQG, DiscreteHinf, GPC, and any controller with hidden
integrators but no built-in anti-windup.

**Do NOT** wrap DiscretePID with this class - DiscretePID already has built-in
back-calculation anti-windup via PIDParams::Kb.

Parameters
----------
inner : IController
    The controller to wrap. Co-owned via shared_ptr.
uMin : float
    Lower actuator saturation limit.
uMax : float
    Upper saturation limit (must be > uMin).
Kb : float, optional
    Conditioning gain.  0 = pure clamp; 1.0 = standard (default).

Example
-------
>>> pid = ctrl.DiscretePID(pp, Ts)   # Kb=0 in pp to disable built-in AW
>>> aw  = ctrl.AntiWindupWrapper(pid, uMin=-1.0, uMax=1.0, Kb=1.0)
>>> u   = aw.compute(r - y)          # saturated + conditioned output
>>> print(aw.is_saturated(), aw.saturation_error())
)doc")
        .def(py::init<std::shared_ptr<ctrl::IController>, double, double, double>(),
             py::arg("inner"), py::arg("uMin"), py::arg("uMax"), py::arg("Kb") = 1.0,
             "Construct wrapper around inner controller with saturation limits and conditioning gain.")
        .def("compute",           &ctrl::AntiWindupWrapper::compute,
             py::arg("error"),
             "Advance one step with saturation and conditioning. Returns saturated u.")
        .def("reset",             &ctrl::AntiWindupWrapper::reset,
             "Reset the wrapper and the inner controller.")
        .def("sample_time",       &ctrl::AntiWindupWrapper::sampleTime,
             "Sample time [s] inherited from the inner controller.")
        .def("is_healthy",        &ctrl::AntiWindupWrapper::isHealthy,
             "Delegates health check to the inner controller (e.g. QP convergence).")
        .def("last_output",       &ctrl::AntiWindupWrapper::lastOutput,
             "Last saturated output u[k] returned by compute().")
        .def("is_saturated",      &ctrl::AntiWindupWrapper::isSaturated,
             "True when the previous compute() hit the saturation limits.")
        .def("saturation_error",  &ctrl::AntiWindupWrapper::saturationError,
             "Saturation error from the last step: u_sat - u_raw. Zero when not saturated.")
        .def("set_actual_output", &ctrl::AntiWindupWrapper::setActualOutput,
             py::arg("u_applied"),
             "Feed back the true applied output when external clamping overrides the internal one.");

    // -----------------------------------------------------------------------
    // ComputationalDelayWrapper - one-sample actuator delay decorator
    // -----------------------------------------------------------------------
    py::class_<ctrl::ComputationalDelayWrapper, ctrl::IController,
               std::shared_ptr<ctrl::ComputationalDelayWrapper>>(
        m, "ComputationalDelayWrapper", R"doc(
One-sample computational delay wrapper for any IController.

Models the real-world delay between compute() and actuator application:
u_out[k] = u_inner[k-1].  The first compute() returns initial_output (default 0).
)doc")
        .def(py::init<std::shared_ptr<ctrl::IController>, double>(),
             py::arg("inner"), py::arg("initial_output") = 0.0,
             "Wrap inner controller with a one-sample delay.")
        .def("compute",      &ctrl::ComputationalDelayWrapper::compute,
             py::arg("signal"),
             "Return last step's inner output; schedule this step's computation for next step.")
        .def("reset",        &ctrl::ComputationalDelayWrapper::reset,
             "Reset inner controller and clear the delayed output to 0.")
        .def("sample_time",  &ctrl::ComputationalDelayWrapper::sampleTime,
             "Sample time [s] inherited from the inner controller.")
        .def("last_output",  &ctrl::ComputationalDelayWrapper::lastOutput,
             "Output buffered from the previous step (will be returned on next compute()).");

    // -----------------------------------------------------------------------
    // EventTriggeredWrapper - aperiodic-sampling (deadband) decorator
    // -----------------------------------------------------------------------
    py::class_<ctrl::EventTriggeredParams>(m, "EventTriggeredParams",
        "Deadband threshold for EventTriggeredWrapper.")
        .def(py::init<>())
        .def_readwrite("sigma", &ctrl::EventTriggeredParams::sigma);

    py::class_<ctrl::EventTriggeredWrapper, ctrl::IController,
               std::shared_ptr<ctrl::EventTriggeredWrapper>>(
        m, "EventTriggeredWrapper", R"doc(
Aperiodic-sampling (event-triggered) decorator for any IController.

Holds the last output (zero-order hold) until the incoming signal drifts more than
`params.sigma` away from the signal value at the last triggered computation, then
recomputes once. Convention-agnostic: only ever compares signal to its own last-triggered
value, so it works regardless of the wrapped controller's sign convention.
)doc")
        .def(py::init<std::shared_ptr<ctrl::IController>, const ctrl::EventTriggeredParams&>(),
             py::arg("inner"), py::arg("params") = ctrl::EventTriggeredParams{},
             "Wrap inner controller with event-triggered (deadband) sampling.")
        .def("compute",        &ctrl::EventTriggeredWrapper::compute, py::arg("signal"),
             "Recompute if |signal - last_triggered_signal| > sigma; else hold last output.")
        .def("reset",          &ctrl::EventTriggeredWrapper::reset,
             "Reset inner controller and held state; next compute() triggers unconditionally.")
        .def("sample_time",    &ctrl::EventTriggeredWrapper::sampleTime,
             "Sample time [s] inherited from the inner controller.")
        .def("last_output",    &ctrl::EventTriggeredWrapper::lastOutput,
             "Output held since the last triggered computation.")
        .def("trigger_count",  &ctrl::EventTriggeredWrapper::triggerCount,
             "Number of compute() calls that triggered a real inner computation.")
        .def("hold_count",     &ctrl::EventTriggeredWrapper::holdCount,
             "Number of compute() calls that returned the held output unchanged.");

    // -----------------------------------------------------------------------
    // ILC
    // -----------------------------------------------------------------------
    py::enum_<ctrl::ILC::Mode>(m, "ILCMode")
        .value("PType",       ctrl::ILC::Mode::PType,       "u_{j+1}[k] = Q*u_j[k] + Lp*e_j[k]")
        .value("DType",       ctrl::ILC::Mode::DType,       "PType + derivative term Ld*(de/dt)")
        .value("NormOptimal", ctrl::ILC::Mode::NormOptimal, "Minimum-norm update using Markov matrix G")
        .export_values();

    py::class_<ctrl::ILC::Params>(m, "ILCParams",
        "Parameters for the Iterative Learning Controller.")
        .def(py::init<>())
        .def_readwrite("N",        &ctrl::ILC::Params::N,        "Trial length (steps per trial).")
        .def_readwrite("Ts",       &ctrl::ILC::Params::Ts,       "Sample time [s].")
        .def_readwrite("mode",     &ctrl::ILC::Params::mode,     "ILC update law (PType/DType/NormOptimal).")
        .def_readwrite("Lp",       &ctrl::ILC::Params::Lp,       "Proportional ILC gain in (0,1).")
        .def_readwrite("Ld",       &ctrl::ILC::Params::Ld,       "Derivative ILC gain.")
        .def_readwrite("Q_filter", &ctrl::ILC::Params::Q_filter, "Forgetting factor for old feedforward [0,1].")
        .def_readwrite("rho_u",    &ctrl::ILC::Params::rho_u,    "Input effort weight (norm-optimal only).")
        .def_readwrite("rho_e",    &ctrl::ILC::Params::rho_e,    "Tracking error weight (norm-optimal only).")
        .def_readwrite("uMin",     &ctrl::ILC::Params::uMin,     "Feedforward lower bound.")
        .def_readwrite("uMax",     &ctrl::ILC::Params::uMax,     "Feedforward upper bound.");

    py::class_<ctrl::ILC, std::shared_ptr<ctrl::ILC>>(m, "ILC", R"doc(
Iterative Learning Controller (ILC).

Learns a feedforward correction that eliminates repeating tracking errors across
trials of a fixed-duration task.  Supports P-type, D-type, and norm-optimal
update laws.

Example
-------
>>> p = ctrl.ILCParams(); p.N = 100; p.Ts = 0.01; p.Lp = 0.5
>>> ilc = ctrl.ILC(p)
>>> for k in range(p.N):
...     u = ilc.feedforward(k) + pid.compute(r[k] - y[k])
...     ilc.record_error(k, r[k] - y[k])
>>> ilc.update_feedforward()    # compute next-trial feedforward
)doc")
        .def(py::init<const ctrl::ILC::Params&>(),
             py::arg("params"),
             "Construct P-type or D-type ILC.")
        .def(py::init<const ctrl::ILC::Params&, const Eigen::MatrixXd&>(),
             py::arg("params"), py::arg("G"),
             "Construct norm-optimal ILC from N*N Markov matrix G.")
        .def("feedforward",        &ctrl::ILC::feedforward,        py::arg("k"),
             "Return the stored feedforward input for step k of the current trial.")
        .def("record_error",       &ctrl::ILC::recordError,        py::arg("k"), py::arg("error"),
             "Record tracking error at step k.")
        .def("update_feedforward", &ctrl::ILC::updateFeedforward,
             "Compute and store the updated feedforward for the next trial.")
        .def("reset",              &ctrl::ILC::reset,
             "Reset feedforward to zero and restart from trial 0.")
        .def("trial_index",        &ctrl::ILC::trialIndex,   "Current trial index.")
        .def("trial_length",       &ctrl::ILC::trialLength,  "Trial length N.")
        .def("last_rms_error",     &ctrl::ILC::lastRMSError, "RMS error of the last completed trial.");

    // -----------------------------------------------------------------------
    // SINDy
    // -----------------------------------------------------------------------
    py::enum_<ctrl::SINDyLibrary>(m, "SINDyLibrary",
        "Basis function set for the SINDy library Theta(x,u).")
        .value("PolyDeg1", ctrl::SINDyLibrary::PolyDeg1, "1 + x + u")
        .value("PolyDeg2", ctrl::SINDyLibrary::PolyDeg2, "PolyDeg1 + all degree-2 monomials")
        .value("PolyDeg3", ctrl::SINDyLibrary::PolyDeg3, "PolyDeg2 + all degree-3 monomials")
        .value("PolyTrig", ctrl::SINDyLibrary::PolyTrig, "PolyDeg2 + sin/cos of each state")
        .export_values();

    py::class_<ctrl::SINDy::Params>(m, "SINDyParams",
        "Parameters for the SINDy sparse identification algorithm.")
        .def(py::init<>())
        .def_readwrite("n_state",   &ctrl::SINDy::Params::n_state,   "State dimension.")
        .def_readwrite("n_input",   &ctrl::SINDy::Params::n_input,   "Input dimension.")
        .def_readwrite("library",   &ctrl::SINDy::Params::library,   "Basis function set.")
        .def_readwrite("threshold", &ctrl::SINDy::Params::threshold, "STLS sparsity threshold.")
        .def_readwrite("stls_iter", &ctrl::SINDy::Params::stls_iter, "STLS iterations.")
        .def_readwrite("use_ols",   &ctrl::SINDy::Params::use_ols,   "If True, use plain OLS (no sparsification).");

    py::class_<ctrl::SINDyModel>(m, "SINDyModel", R"doc(
Fitted SINDy sparse model: xdot = Theta(x,u) * Xi.

Returned by SINDy.fit().  Use predict(x, u) to evaluate the identified dynamics.
)doc")
        .def("predict",      &ctrl::SINDyModel::predict,
             py::arg("x_dev"), py::arg("u_dev"),
             "Evaluate identified dynamics at (x_dev, u_dev).  Returns xdot.")
        .def("n_state",      &ctrl::SINDyModel::nState)
        .def("n_input",      &ctrl::SINDyModel::nInput)
        .def("n_terms",      &ctrl::SINDyModel::nTerms)
        .def("sparsity",     &ctrl::SINDyModel::sparsity,
             "Non-zero fraction of Xi (1.0 = fully sparse, 0.0 = dense).")
        .def("coefficients", &ctrl::SINDyModel::coefficients,
             py::return_value_policy::copy,
             "Coefficient matrix Xi (n_terms x n_state).");

    py::class_<ctrl::SINDy, std::shared_ptr<ctrl::SINDy>>(m, "SINDy", R"doc(
Sparse Identification of Nonlinear Dynamics (SINDy).

Collects I/O snapshots, builds a polynomial (or trig) library Theta(x,u),
and solves for the sparsest coefficient matrix Xi via STLS.

Example
-------
>>> p = ctrl.SINDyParams(); p.n_state=2; p.n_input=1; p.threshold=0.05
>>> s = ctrl.SINDy(p)
>>> s.add_snapshot(x_dev, u_dev, xdot_measured)   # or add_snapshot_fd
>>> model = s.fit()                                 # SINDyModel
>>> xdot_hat = model.predict(x_test, u_test)
)doc")
        .def(py::init<const ctrl::SINDy::Params&>(), py::arg("params"))
        .def("add_snapshot",    &ctrl::SINDy::addSnapshot,
             py::arg("x_dev"), py::arg("u_dev"), py::arg("xdot_dev"),
             "Add a snapshot: (state_dev, input_dev) -> derivative.")
        .def("add_snapshot_fd", &ctrl::SINDy::addSnapshotFD,
             py::arg("x_k"), py::arg("x_k1"), py::arg("u_k"), py::arg("Ts"),
             "Add a snapshot using forward finite-difference: xdot approx = (x_{k+1} - x_k) / Ts.")
        .def("fit",             &ctrl::SINDy::fit,
             "Fit the sparse model.  Returns a SINDyModel.")
        .def("snapshot_count",  &ctrl::SINDy::snapshotCount)
        .def("n_terms",         &ctrl::SINDy::nTerms,
             "Number of library terms for the chosen basis.")
        .def("library_row",     &ctrl::SINDy::libraryRow,
             py::arg("x"), py::arg("u"),
             "Return the library row Theta(x, u) as a numpy array.");

    // -----------------------------------------------------------------------
    // KoopmanEDMD
    // -----------------------------------------------------------------------
    py::enum_<ctrl::KoopmanDict>(m, "KoopmanDict",
        "Dictionary type for Koopman/EDMD lifting map.")
        .value("PolyDeg1", ctrl::KoopmanDict::PolyDeg1, "Linear DMDc ([1;x;u])")
        .value("PolyDeg2", ctrl::KoopmanDict::PolyDeg2, "Quadratic lifting")
        .value("RBF",      ctrl::KoopmanDict::RBF,      "Gaussian RBF centres")
        .export_values();

    py::class_<ctrl::KoopmanEDMD::Params>(m, "KoopmanEDMDParams")
        .def(py::init<>())
        .def_readwrite("n_state",    &ctrl::KoopmanEDMD::Params::n_state)
        .def_readwrite("n_input",    &ctrl::KoopmanEDMD::Params::n_input)
        .def_readwrite("dict",       &ctrl::KoopmanEDMD::Params::dict)
        .def_readwrite("rbf_n_cent", &ctrl::KoopmanEDMD::Params::rbf_n_cent)
        .def_readwrite("rbf_width",  &ctrl::KoopmanEDMD::Params::rbf_width)
        .def_readwrite("tikhonov",   &ctrl::KoopmanEDMD::Params::tikhonov);

    py::class_<ctrl::KoopmanEDMD, std::shared_ptr<ctrl::KoopmanEDMD>>(m, "KoopmanEDMD",
        "Extended Dynamic Mode Decomposition (Koopman operator approximation).")
        .def(py::init<const ctrl::KoopmanEDMD::Params&>(), py::arg("params"))
        .def("add_snapshot",    &ctrl::KoopmanEDMD::addSnapshot,
             py::arg("x_k"), py::arg("u_k"), py::arg("x_k1"))
        .def("fit",             &ctrl::KoopmanEDMD::fit,
             "Fit full lifted StateSpace.")
        .def("fit_projected",   &ctrl::KoopmanEDMD::fitProjected,
             "Fit and project back to original n_state dimensions.")
        .def("n_lifted",        &ctrl::KoopmanEDMD::nLifted)
        .def("snapshot_count",  &ctrl::KoopmanEDMD::snapshotCount)
        .def("lift",            &ctrl::KoopmanEDMD::lift,
             py::arg("x"), py::arg("u"),
             "Return the lifted observable vector psi(x, u).");

    // -----------------------------------------------------------------------
    // L1 Adaptive Controller
    // -----------------------------------------------------------------------
    py::class_<ctrl::L1AdaptiveController::Params>(m, "L1AdaptiveParams")
        .def(py::init<>())
        .def_readwrite("a_m",       &ctrl::L1AdaptiveController::Params::a_m)
        .def_readwrite("b_m",       &ctrl::L1AdaptiveController::Params::b_m)
        .def_readwrite("k_g",       &ctrl::L1AdaptiveController::Params::k_g)
        .def_readwrite("Gamma",     &ctrl::L1AdaptiveController::Params::Gamma)
        .def_readwrite("omega_c",   &ctrl::L1AdaptiveController::Params::omega_c)
        .def_readwrite("sigma_max", &ctrl::L1AdaptiveController::Params::sigma_max)
        .def_readwrite("uMin",      &ctrl::L1AdaptiveController::Params::uMin)
        .def_readwrite("uMax",      &ctrl::L1AdaptiveController::Params::uMax);

    py::class_<ctrl::L1AdaptiveController, ctrl::IController,
               std::shared_ptr<ctrl::L1AdaptiveController>>(m, "L1AdaptiveController",
        "L1 Adaptive Controller (Hovakimyan 2010). Bounded-transient adaptive control.")
        .def(py::init<const ctrl::L1AdaptiveController::Params&, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",                 &ctrl::L1AdaptiveController::compute, py::arg("error"))
        .def("set_reference",           &ctrl::L1AdaptiveController::setReference, py::arg("r"))
        .def("reset",                   &ctrl::L1AdaptiveController::reset)
        .def("sample_time",             &ctrl::L1AdaptiveController::sampleTime)
        .def("estimated_disturbance",   &ctrl::L1AdaptiveController::estimatedDisturbance)
        .def("prediction_error",        &ctrl::L1AdaptiveController::predictionError);

    // -----------------------------------------------------------------------
    // CBF Safety Filter
    // -----------------------------------------------------------------------
    py::class_<ctrl::CBFSafetyFilter::Params>(m, "CBFParams")
        .def(py::init<>())
        .def_readwrite("alpha", &ctrl::CBFSafetyFilter::Params::alpha)
        .def_readwrite("uMin",  &ctrl::CBFSafetyFilter::Params::uMin)
        .def_readwrite("uMax",  &ctrl::CBFSafetyFilter::Params::uMax)
        .def_readwrite("slack", &ctrl::CBFSafetyFilter::Params::slack);

    py::class_<ctrl::CBFSafetyFilter, ctrl::IController,
               std::shared_ptr<ctrl::CBFSafetyFilter>>(m, "CBFSafetyFilter",
        "Control Barrier Function safety filter (Ames 2017). 1-QP safety wrapper.")
        .def(py::init<std::shared_ptr<ctrl::IController>,
                      std::function<double(double)>,
                      std::function<double(double)>,
                      std::function<double(double)>,
                      std::function<double(double)>,
                      const ctrl::CBFSafetyFilter::Params&, double>(),
             py::arg("nominal"), py::arg("h"), py::arg("dh_dx"),
             py::arg("f0"), py::arg("g"), py::arg("params"), py::arg("Ts"))
        .def("compute",       &ctrl::CBFSafetyFilter::compute,     py::arg("error"))
        .def("set_state",     &ctrl::CBFSafetyFilter::setState,    py::arg("x"))
        .def("reset",         &ctrl::CBFSafetyFilter::reset)
        .def("cbf_active",    &ctrl::CBFSafetyFilter::cbfActive)
        .def("barrier_value", &ctrl::CBFSafetyFilter::barrierValue)
        .def("nominal_output",&ctrl::CBFSafetyFilter::nominalOutput);

    // -----------------------------------------------------------------------
    // GaussianProcess
    // -----------------------------------------------------------------------
    py::class_<ctrl::GaussianProcess::Params>(m, "GPParams")
        .def(py::init<>())
        .def_readwrite("length_scale", &ctrl::GaussianProcess::Params::length_scale)
        .def_readwrite("signal_var",   &ctrl::GaussianProcess::Params::signal_var)
        .def_readwrite("noise_var",    &ctrl::GaussianProcess::Params::noise_var)
        .def_readwrite("n_max",        &ctrl::GaussianProcess::Params::n_max);

    py::class_<ctrl::GaussianProcess::Prediction>(m, "GPPrediction")
        .def_readonly("mean",     &ctrl::GaussianProcess::Prediction::mean)
        .def_readonly("variance", &ctrl::GaussianProcess::Prediction::variance);

    py::class_<ctrl::GaussianProcess, std::shared_ptr<ctrl::GaussianProcess>>(m, "GaussianProcess",
        "Gaussian Process Regression (SE kernel, Cholesky inference, fixed-budget).")
        .def(py::init<int, const ctrl::GaussianProcess::Params&>(),
             py::arg("x_dim"), py::arg("params"))
        .def("add_point", &ctrl::GaussianProcess::addPoint,
             py::arg("x"), py::arg("y"))
        .def("fit",       &ctrl::GaussianProcess::fit)
        .def("predict",   &ctrl::GaussianProcess::predict,   py::arg("x_test"))
        .def("size",      &ctrl::GaussianProcess::size)
        .def("is_fitted", &ctrl::GaussianProcess::isFitted)
        .def("reset",     &ctrl::GaussianProcess::reset);

    // -----------------------------------------------------------------------
    // GPResidualModel (E3) - GP mismatch correction for risk-aware MPC
    // -----------------------------------------------------------------------
    py::class_<ctrl::GPResidualModel::Params>(m, "GPResidualParams",
        "Parameters for GPResidualModel (E3).")
        .def(py::init<>())
        .def_readwrite("gp", &ctrl::GPResidualModel::Params::gp,
                       "GP hyperparameters (GPParams).");

    py::class_<ctrl::GPResidualModel::Prediction>(m, "GPResidualPrediction",
        "Prediction from GPResidualModel.predict_with_uncertainty().")
        .def_readonly("mean_total", &ctrl::GPResidualModel::Prediction::mean_total,
                      "model_pred + GP correction mean (bias-corrected estimate).")
        .def_readonly("gp_mean",    &ctrl::GPResidualModel::Prediction::gp_mean,
                      "GP correction mean only (the learned residual).")
        .def_readonly("variance",   &ctrl::GPResidualModel::Prediction::variance,
                      "GP posterior variance (>= 0). sqrt gives 1-sigma bound.");

    py::class_<ctrl::GPResidualModel, std::shared_ptr<ctrl::GPResidualModel>>(m, "GPResidualModel", R"doc(
GP Residual Model - learn model-plant mismatch for risk-aware MPC (E3).

Fits a GP to residuals epsilon = y_true - y_model.  At prediction time:
  mean_total = model_pred + GP_mean(x_feat)   -- bias-corrected output
  variance   = GP_variance(x_feat)            -- for constraint tightening

Example
-------
>>> p = ctrl.GPResidualParams(); p.gp.length_scale = 0.5
>>> grm = ctrl.GPResidualModel(x_dim=1, params=p)
>>> grm.add_residual_point(x_feat, y_true, y_model)
>>> grm.fit()
>>> pred = grm.predict_with_uncertainty(x_feat, model_output)
>>> print(pred.mean_total, pred.variance)

Or batch fit:
>>> grm.residual_fit(X_feat, Y_true, model_fn=lambda x: linear_model(x))
)doc")
        .def(py::init<int, const ctrl::GPResidualModel::Params&>(),
             py::arg("x_dim"), py::arg("params"),
             "Construct with feature dimension and GPResidualParams.")
        .def("add_residual_point",
             &ctrl::GPResidualModel::addResidualPoint,
             py::arg("x_feat"), py::arg("y_true"), py::arg("y_model"),
             "Add one (feature, y_true, y_model) sample online.")
        .def("residual_fit",
             [](ctrl::GPResidualModel& self,
                const Eigen::MatrixXd& X_feat,
                const Eigen::VectorXd& Y_true,
                py::object model_fn_py) {
                 self.residualFit(X_feat, Y_true,
                     [model_fn_py](const Eigen::VectorXd& x) -> double {
                         return model_fn_py(x).cast<double>();
                     });
             },
             py::arg("X_feat"), py::arg("Y_true"), py::arg("model_fn"),
             "Batch: compute residuals from model_fn and fit GP in one call.\n"
             "model_fn(x_feat: np.ndarray) -> float.")
        .def("fit",     &ctrl::GPResidualModel::fit,
             "(Re-)fit the GP on all accumulated residual data.")
        .def("predict_with_uncertainty",
             &ctrl::GPResidualModel::predictWithUncertainty,
             py::arg("x_feat"), py::arg("model_pred"),
             "Predict total output and GP uncertainty. Returns GPResidualPrediction.")
        .def("size",      &ctrl::GPResidualModel::size,
             "Number of residual data points stored.")
        .def("is_fitted", &ctrl::GPResidualModel::isFitted,
             "True if fit() has been called with at least one data point.")
        .def("reset",     &ctrl::GPResidualModel::reset,
             "Clear all stored data and reset the GP.");

    // -----------------------------------------------------------------------
    // EchoStateNetwork
    // -----------------------------------------------------------------------
    py::class_<ctrl::EchoStateNetwork::Params>(m, "ESNParams")
        .def(py::init<>())
        .def_readwrite("n_res",           &ctrl::EchoStateNetwork::Params::n_res)
        .def_readwrite("n_in",            &ctrl::EchoStateNetwork::Params::n_in)
        .def_readwrite("n_out",           &ctrl::EchoStateNetwork::Params::n_out)
        .def_readwrite("spectral_radius", &ctrl::EchoStateNetwork::Params::spectral_radius)
        .def_readwrite("sparsity",        &ctrl::EchoStateNetwork::Params::sparsity)
        .def_readwrite("alpha",           &ctrl::EchoStateNetwork::Params::alpha)
        .def_readwrite("ridge",           &ctrl::EchoStateNetwork::Params::ridge)
        .def_readwrite("washout",         &ctrl::EchoStateNetwork::Params::washout)
        .def_readwrite("seed",            &ctrl::EchoStateNetwork::Params::seed);

    py::class_<ctrl::EchoStateNetwork, std::shared_ptr<ctrl::EchoStateNetwork>>(m, "EchoStateNetwork",
        "Echo State Network / Reservoir Computing (Jaeger 2001). Random reservoir, trained readout.")
        .def(py::init<const ctrl::EchoStateNetwork::Params&>(), py::arg("params"))
        .def("step_reservoir",      &ctrl::EchoStateNetwork::stepReservoir, py::arg("u"))
        .def("add_training_target", &ctrl::EchoStateNetwork::addTrainingTarget, py::arg("y_target"))
        .def("fit_readout",         &ctrl::EchoStateNetwork::fitReadout)
        .def("predict",
             py::overload_cast<const Eigen::VectorXd&>(&ctrl::EchoStateNetwork::predict),
             py::arg("u"))
        .def("reset",            &ctrl::EchoStateNetwork::reset)
        .def("is_fitted",        &ctrl::EchoStateNetwork::isFitted)
        .def("reservoir_size",   &ctrl::EchoStateNetwork::reservoirSize)
        .def("reservoir_state",  &ctrl::EchoStateNetwork::reservoirState,
             py::return_value_policy::copy);

    // -----------------------------------------------------------------------
    // NeuralPID
    // -----------------------------------------------------------------------
    py::class_<ctrl::NeuralPID::Params>(m, "NeuralPIDParams")
        .def(py::init<>())
        .def_readwrite("n_hidden",       &ctrl::NeuralPID::Params::n_hidden)
        .def_readwrite("lr",             &ctrl::NeuralPID::Params::lr)
        .def_readwrite("Ts",             &ctrl::NeuralPID::Params::Ts)
        .def_readwrite("plant_gain",     &ctrl::NeuralPID::Params::plant_gain)
        .def_readwrite("max_weight_norm",&ctrl::NeuralPID::Params::max_weight_norm)
        .def_readwrite("uMin",           &ctrl::NeuralPID::Params::uMin)
        .def_readwrite("uMax",           &ctrl::NeuralPID::Params::uMax)
        .def_readwrite("Kp0",            &ctrl::NeuralPID::Params::Kp0)
        .def_readwrite("Ki0",            &ctrl::NeuralPID::Params::Ki0)
        .def_readwrite("Kd0",            &ctrl::NeuralPID::Params::Kd0);

    py::class_<ctrl::NeuralPID, ctrl::IController,
               std::shared_ptr<ctrl::NeuralPID>>(m, "NeuralPID",
        "Online Neural PID - small NN adapts Kp/Ki/Kd via online backpropagation.")
        .def(py::init<const ctrl::NeuralPID::Params&>(), py::arg("params"))
        .def("compute",     &ctrl::NeuralPID::compute, py::arg("error"))
        .def("reset",       &ctrl::NeuralPID::reset)
        .def("sample_time", &ctrl::NeuralPID::sampleTime)
        .def("current_kp",  &ctrl::NeuralPID::currentKp)
        .def("current_ki",  &ctrl::NeuralPID::currentKi)
        .def("current_kd",  &ctrl::NeuralPID::currentKd);

    // -----------------------------------------------------------------------
    // CEM-MPC
    // -----------------------------------------------------------------------
    py::class_<ctrl::CEMController::Params>(m, "CEMParams")
        .def(py::init<>())
        .def_readwrite("Np",         &ctrl::CEMController::Params::Np)
        .def_readwrite("N_samples",  &ctrl::CEMController::Params::N_samples)
        .def_readwrite("n_iter",     &ctrl::CEMController::Params::n_iter)
        .def_readwrite("elite_frac", &ctrl::CEMController::Params::elite_frac)
        .def_readwrite("sigma_init", &ctrl::CEMController::Params::sigma_init)
        .def_readwrite("sigma_min",  &ctrl::CEMController::Params::sigma_min)
        .def_readwrite("Q",          &ctrl::CEMController::Params::Q)
        .def_readwrite("R",          &ctrl::CEMController::Params::R)
        .def_readwrite("uMin",       &ctrl::CEMController::Params::uMin)
        .def_readwrite("uMax",       &ctrl::CEMController::Params::uMax)
        .def_readwrite("seed",       &ctrl::CEMController::Params::seed);

    py::class_<ctrl::CEMController, ctrl::IController,
               std::shared_ptr<ctrl::CEMController>>(m, "CEMController",
        "Cross-Entropy Method MPC (derivative-free stochastic rollout optimisation).")
        .def(py::init<const ctrl::CEMController::Params&,
                      ctrl::StateFunc,
                      Eigen::MatrixXd, double>(),
             py::arg("params"), py::arg("f"), py::arg("C"), py::arg("Ts"))
        .def("compute_ref",   &ctrl::CEMController::computeRef,
             py::arg("x_cur"), py::arg("y_ref"))
        .def("compute",       &ctrl::CEMController::compute,      py::arg("error"))
        .def("set_state",     &ctrl::CEMController::setState,     py::arg("x"))
        .def("set_reference", &ctrl::CEMController::setReference, py::arg("r_ref"))
        .def("reset",         &ctrl::CEMController::reset)
        .def("last_cost",     &ctrl::CEMController::lastCost);

    // -----------------------------------------------------------------------
    // DynaController (Sutton Dyna MBRL)
    // -----------------------------------------------------------------------
    py::class_<ctrl::DynaController::Params>(m, "DynaParams",
        "DynaController design parameters.")
        .def(py::init<>())
        .def_readwrite("n_collect",      &ctrl::DynaController::Params::n_collect,
                       "Transitions to collect before first SINDy model fit.")
        .def_readwrite("n_refit_every",  &ctrl::DynaController::Params::n_refit_every,
                       "Refit interval (new transitions) after initial fit.")
        .def_readwrite("Ts",             &ctrl::DynaController::Params::Ts,
                       "Sample time [s].")
        .def_readwrite("sindy",          &ctrl::DynaController::Params::sindy,
                       "SINDy model parameters (n_state=1 and n_input=1 are enforced).");

    py::class_<ctrl::DynaController, ctrl::IController,
               std::shared_ptr<ctrl::DynaController>>(m, "DynaController", R"doc(
Dyna model-based RL controller (Sutton 1991).

Wraps any IController base policy and augments it with online SINDy model learning
and model-based rollout for policy improvement.

Usage
-----
>>> pid   = ctrl.DiscretePID(pid_p, Ts)
>>> dp    = ctrl.DynaParams(); dp.Ts = Ts; dp.n_collect = 50
>>> dyna  = ctrl.DynaController(dp, pid)
>>> u     = dyna.compute(error)                   # run closed loop
>>> if dyna.model_fitted():
...     e_pred = dyna.model_rollout(e0, u_plan)   # synthetic rollout
)doc")
        .def(py::init<const ctrl::DynaController::Params&,
                      std::shared_ptr<ctrl::IController>>(),
             py::arg("params"), py::arg("inner"),
             "Wrap *inner* with Dyna online learning.")
        .def("compute",          &ctrl::DynaController::compute,       py::arg("error"))
        .def("reset",            &ctrl::DynaController::reset)
        .def("sample_time",      &ctrl::DynaController::sampleTime)
        .def("add_transition",   &ctrl::DynaController::addTransition,
             py::arg("e"), py::arg("u"), py::arg("e_next"),
             "Manually add a (e[k], u[k], e[k+1]) transition.")
        .def("fit_model",        &ctrl::DynaController::fitModel,
             "Fit SINDy on accumulated transitions. Returns True on success.")
        .def("model_rollout",    &ctrl::DynaController::modelRollout,
             py::arg("e0"), py::arg("u_sequence"),
             "Simulate fitted model from e0 given u_sequence. Returns predicted error vector.")
        .def("model_fitted",     &ctrl::DynaController::modelFitted)
        .def("buffer_size",      &ctrl::DynaController::bufferSize)
        .def("new_since_last_fit", &ctrl::DynaController::newSinceLastFit)
        .def("inner_controller", &ctrl::DynaController::innerController,
             py::return_value_policy::copy);

    // -----------------------------------------------------------------------
    // HybridModel (H1) - physics + data-driven plant model
    // -----------------------------------------------------------------------
    py::class_<ctrl::HybridModelParams>(m, "HybridModelParams",
        "Configuration for HybridModel (H1).")
        .def(py::init<>())
        .def_readwrite("n_states",  &ctrl::HybridModelParams::n_states,
                       "State dimension (must match f_phys output size).")
        .def_readwrite("n_inputs",  &ctrl::HybridModelParams::n_inputs,
                       "Input dimension.")
        .def_readwrite("n_outputs", &ctrl::HybridModelParams::n_outputs,
                       "Output dimension.")
        .def_readwrite("Ts",        &ctrl::HybridModelParams::Ts,
                       "Sample time [s].")
        .def_readwrite("rk4_steps", &ctrl::HybridModelParams::rk4_steps,
                       "RK4 substeps per Ts for physical integration.");

    py::class_<ctrl::HybridModel, std::shared_ptr<ctrl::HybridModel>>(m, "HybridModel", R"doc(
Hybrid plant model: x[k+1] = RK4(f_phys, x, u, p) + f_data(x, u)  (H1).

Combines a known physical ODE (integrated by RK4) with an optional
data-driven additive state correction f_data(x, u) -> delta_x.

f_phys  signature: (x: np.ndarray, u: np.ndarray, p: np.ndarray) -> np.ndarray
f_data  signature: (x: np.ndarray, u: np.ndarray) -> np.ndarray

Example
-------
>>> def f_phys(x, u, p):
...     return np.array([x[1], -p[0]*x[0] - p[1]*x[1] + u[0]])
>>> hmp = ctrl.HybridModelParams()
>>> hmp.n_states = 2; hmp.n_inputs = 1; hmp.Ts = 0.01
>>> model = ctrl.HybridModel(f_phys, hmp, p_phys=np.array([4.0, 0.8]))
>>> x_next = model.predict(np.zeros(2), np.array([1.0]))
)doc")
        .def(py::init([](py::object f_py,
                          const ctrl::HybridModelParams& params,
                          const Eigen::VectorXd& p_phys) {
                auto f = [f_py](const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u,
                                const Eigen::VectorXd& p) -> Eigen::VectorXd {
                    return f_py(x, u, p).cast<Eigen::VectorXd>();
                };
                return std::make_shared<ctrl::HybridModel>(std::move(f), params, p_phys);
            }),
            py::arg("f_phys"), py::arg("params"),
            py::arg("p_phys") = Eigen::VectorXd{},
            "Construct with physical ODE, params, and optional physical parameters.")
        .def("set_data_model",
             [](ctrl::HybridModel& m, py::object f_py) {
                 m.setDataModel(
                     [f_py](const Eigen::VectorXd& x,
                            const Eigen::VectorXd& u) -> Eigen::VectorXd {
                         return f_py(x, u).cast<Eigen::VectorXd>();
                     });
             },
             py::arg("f_data"),
             "Attach a Python data model f_data(x, u) -> delta_x.")
        .def("clear_data_model",   &ctrl::HybridModel::clearDataModel,
             "Remove the data-driven correction (physical model only).")
        .def("has_data_model",     &ctrl::HybridModel::hasDataModel)
        .def("set_phys_params",    &ctrl::HybridModel::setPhysParams, py::arg("p"),
             "Update physical model parameters p passed to f_phys.")
        .def("phys_params",        &ctrl::HybridModel::physParams,
             py::return_value_policy::copy,
             "Current physical model parameters (n_p,).")
        .def("predict",            &ctrl::HybridModel::predict,
             py::arg("x"), py::arg("u"),
             py::return_value_policy::copy,
             "Combined discrete-time prediction x[k+1] (n_states,).")
        .def("predict_phys",       &ctrl::HybridModel::predictPhys,
             py::arg("x"), py::arg("u"),
             py::return_value_policy::copy,
             "Physical-only prediction (no data correction) (n_states,).")
        .def("state_size",         &ctrl::HybridModel::stateSize)
        .def("input_size",         &ctrl::HybridModel::inputSize)
        .def("output_size",        &ctrl::HybridModel::outputSize)
        .def("sample_time",        &ctrl::HybridModel::sampleTime);

    // -----------------------------------------------------------------------
    // HybridMPCParams
    // -----------------------------------------------------------------------
    py::class_<ctrl::HybridMPCParams>(m, "HybridMPCParams",
        "Parameters for HybridMPC (H2).")
        .def(py::init<>())
        .def_readwrite("nmpc",                 &ctrl::HybridMPCParams::nmpc,
                       "Base NMPCParams (Np, Nu, rho_y, rho_u, uMin, uMax, Ts, n_states, n_inputs, n_outputs).")
        .def_readwrite("data_update_interval", &ctrl::HybridMPCParams::data_update_interval,
                       "Auto-refit data model every N addStateObservation() calls (0 = manual).")
        .def_readwrite("min_observations",     &ctrl::HybridMPCParams::min_observations,
                       "Minimum observations before first refit.")
        .def_readwrite("ridge_lambda",         &ctrl::HybridMPCParams::ridge_lambda,
                       "Ridge regression regularisation for online linear data model.");

    // -----------------------------------------------------------------------
    // HybridMPC (H2) - inherits NonlinearMPC
    // -----------------------------------------------------------------------
    py::class_<ctrl::HybridMPC, ctrl::NonlinearMPC,
               std::shared_ptr<ctrl::HybridMPC>>(m, "HybridMPC", R"doc(
Hybrid Model Predictive Controller (H2).

Inherits NonlinearMPC. Uses a shared HybridModel for prediction and supports
online data accumulation with periodic ridge-regression refitting of f_data.

Example
-------
>>> model = ctrl.HybridModel(f_phys, hmp, p_phys)
>>> hp = ctrl.HybridMPCParams()
>>> hp.nmpc.Np = 10; hp.nmpc.Ts = 0.01
>>> hp.nmpc.n_states = 2; hp.nmpc.n_inputs = 1
>>> hp.data_update_interval = 30
>>> hmpc = ctrl.HybridMPC(hp, model)
>>> hmpc.set_state(x_k)
>>> u = hmpc.compute(error)
>>> hmpc.add_state_observation(x_k, u_vec, x_next_true)
)doc")
        .def(py::init<const ctrl::HybridMPCParams&, std::shared_ptr<ctrl::HybridModel>>(),
             py::arg("params"), py::arg("model"),
             "Construct HybridMPC with params and shared HybridModel.")
        .def("add_state_observation", &ctrl::HybridMPC::addStateObservation,
             py::arg("x"), py::arg("u"), py::arg("x_next_true"),
             "Record a state transition (x, u) -> x_next_true for online learning.")
        .def("refit_data_model",      &ctrl::HybridMPC::refitDataModel,
             "Manually trigger ridge-regression refit. Returns True if successful.")
        .def("observation_count",     &ctrl::HybridMPC::observationCount,
             "Number of state observations accumulated.")
        .def("is_data_model_fitted",  &ctrl::HybridMPC::isDataModelFitted,
             "True if the data model has been fitted at least once.")
        .def("hybrid_model",          &ctrl::HybridMPC::hybridModel,
             "Return the shared HybridModel.");

    // -----------------------------------------------------------------------
    // GPMPCParams
    // -----------------------------------------------------------------------
    py::class_<ctrl::GPMPCParams>(m, "GPMPCParams",
        "Parameters for GPMPC (ML3).")
        .def(py::init<>())
        .def_readwrite("nmpc",                  &ctrl::GPMPCParams::nmpc,
                       "Base NMPCParams (Np, Nu, rho_y, rho_u, uMin, uMax, Ts, n_states, n_inputs, n_outputs).")
        .def_readwrite("uncertainty_inflation",  &ctrl::GPMPCParams::uncertainty_inflation,
                       "Tightening = inflation * sqrt(GP variance). Default 2.0.");

    // -----------------------------------------------------------------------
    // GPMPC (ML3) - inherits NonlinearMPC
    // -----------------------------------------------------------------------
    py::class_<ctrl::GPMPC, ctrl::NonlinearMPC,
               std::shared_ptr<ctrl::GPMPC>>(m, "GPMPC", R"doc(
GP-uncertainty-aware tightening of NonlinearMPC's input bounds (Phase 3 ML3).

Inherits NonlinearMPC. Per RTI step, tightens the input box bounds proportional to
sqrt(GPResidualModel's predicted variance) at the predicted state/input feature for that
step -- a no-op (identical to plain NonlinearMPC) whenever the GP is unfitted.

Example
-------
>>> gp = ctrl.GPResidualModel(x_dim=2, params=p)  # x_dim = n_states + n_inputs
>>> params = ctrl.GPMPCParams()
>>> params.nmpc.Np = 5; params.nmpc.Nu = 3; params.nmpc.n_states = 1; params.nmpc.n_inputs = 1
>>> gpmpc = ctrl.GPMPC(params, f_dynamics, gp)
>>> gpmpc.set_state(x_k)
>>> u = gpmpc.compute(error)
>>> gpmpc.last_tightening()  # per-step shrink amounts (Nu*m vector)
)doc")
        .def(py::init<const ctrl::GPMPCParams&, ctrl::NonlinearMPC::DiscreteDynamics,
                      std::shared_ptr<ctrl::GPResidualModel>>(),
             py::arg("params"), py::arg("f_d"), py::arg("gp"),
             "Construct GPMPC with params, discrete dynamics, and a shared GPResidualModel.")
        .def("last_tightening", &ctrl::GPMPC::lastTightening,
             "Per-step shrink amounts applied to the input bounds on the most recent compute() "
             "(Nu*m vector). All zero if the GP is unfitted.");

    // -----------------------------------------------------------------------
    // HybridModelTrainer (H4) - off-line trainer (Ridge / GP / ESN)
    // -----------------------------------------------------------------------
    py::enum_<ctrl::HybridModelTrainer::Method>(m, "HybridTrainerMethod",
        "Training algorithm for HybridModelTrainer.")
        .value("Ridge", ctrl::HybridModelTrainer::Method::Ridge,
               "Ridge regression (linear, fast, always converges).")
        .value("GP",    ctrl::HybridModelTrainer::Method::GP,
               "Independent GPs per state dimension (nonlinear + uncertainty).")
        .value("ESN",   ctrl::HybridModelTrainer::Method::ESN,
               "Echo State Network with zero-state stateless evaluation.");

    py::class_<ctrl::HybridModelTrainer::Params>(m, "HybridTrainerParams",
        "Parameters for HybridModelTrainer (H4).")
        .def(py::init<>())
        .def_readwrite("method",       &ctrl::HybridModelTrainer::Params::method,
                       "Training method (HybridTrainerMethod.Ridge / GP / ESN).")
        .def_readwrite("ridge_lambda", &ctrl::HybridModelTrainer::Params::ridge_lambda,
                       "Ridge regularisation coefficient.")
        .def_readwrite("gp",           &ctrl::HybridModelTrainer::Params::gp,
                       "GP hyperparameters (GPParams): length_scale, signal_var, noise_var, n_max.")
        .def_readwrite("esn",          &ctrl::HybridModelTrainer::Params::esn,
                       "ESN hyperparameters (EchoStateNetworkParams): n_res, spectral_radius, etc.");

    py::class_<ctrl::HybridModelTrainer::Result>(m, "HybridTrainerResult",
        "Result from HybridModelTrainer.train_hybrid_model().")
        .def_readonly("success",     &ctrl::HybridModelTrainer::Result::success)
        .def_readonly("train_rmse",  &ctrl::HybridModelTrainer::Result::train_rmse,
                      "Root-mean-square residual prediction error on training data.")
        .def_readonly("method",      &ctrl::HybridModelTrainer::Result::method,
                      "Method name string ('Ridge', 'GP', 'ESN').")
        .def_readonly("n_samples",   &ctrl::HybridModelTrainer::Result::n_samples);

    py::class_<ctrl::HybridModelTrainer>(m, "HybridModelTrainer", R"doc(
Off-line trainer for the data component of a HybridModel (H4).

Computes state residuals delta_x = x_next_true - predictPhys(x, u) for each
observation and fits a data model (Ridge / GP / ESN) on feature [x; u] -> delta_x.

Example
-------
>>> tp = ctrl.HybridTrainerParams()
>>> tp.method = ctrl.HybridTrainerMethod.GP
>>> tp.gp.length_scale = 0.5
>>> trainer = ctrl.HybridModelTrainer(tp)
>>> result = trainer.train_hybrid_model(model, X_obs, U_obs, X_next_obs)
>>> print(f"RMSE={result.train_rmse:.4f}  samples={result.n_samples}")
>>> rmse_val = trainer.validate(model, X_val, U_val, X_next_val)
)doc")
        .def(py::init<const ctrl::HybridModelTrainer::Params&>(),
             py::arg("params") = ctrl::HybridModelTrainer::Params{},
             "Construct trainer. params defaults to Ridge method.")
        .def("train_hybrid_model",
             &ctrl::HybridModelTrainer::trainHybridModel,
             py::arg("model"), py::arg("X_obs"), py::arg("U_obs"), py::arg("X_next_obs"),
             "Train data model and install in model via setDataModel(). Returns HybridTrainerResult.")
        .def("validate",
             &ctrl::HybridModelTrainer::validate,
             py::arg("model"), py::arg("X_obs"), py::arg("U_obs"), py::arg("X_next_obs"),
             "Evaluate prediction RMSE of model against observed transitions.");

    // -----------------------------------------------------------------------
    // DeePC - Data-Enabled Predictive Control (Coulson, Lygeros, Dorfler 2019)
    // -----------------------------------------------------------------------
    py::class_<ctrl::DeePCParams>(m, "DeePCParams", R"doc(
Tuning parameters for the DeePC controller.

Attributes:
    T_ini (int):      Past trajectory window (steps). Default 20.
    N (int):          Prediction horizon (steps). Default 10.
    Q (float):        Output tracking weight. Default 1.0.
    R (float):        Future input regularization weight. Default 0.1.
    lambda_g (float): Tikhonov weight on Hankel coefficients. Default 1.0.
    lambda_y (float): Past output soft-constraint weight. Default 100.0.
    lambda_u (float): Past input soft-constraint weight. Default 10.0.
    uMin (float):     Hard lower bound on control output. Default -1e9.
    uMax (float):     Hard upper bound on control output. Default +1e9.
    rho (float):      ADMM penalty parameter. Default 10.0.
    admm_iters (int): Max ADMM iterations per step. Default 200.
    admm_tol (float): Primal/dual residual convergence tolerance. Default 1e-6.
)doc")
        .def(py::init<>())
        .def_readwrite("T_ini",       &ctrl::DeePCParams::T_ini)
        .def_readwrite("N",           &ctrl::DeePCParams::N)
        .def_readwrite("Q",           &ctrl::DeePCParams::Q)
        .def_readwrite("R",           &ctrl::DeePCParams::R)
        .def_readwrite("lambda_g",    &ctrl::DeePCParams::lambda_g)
        .def_readwrite("lambda_y",    &ctrl::DeePCParams::lambda_y)
        .def_readwrite("lambda_u",    &ctrl::DeePCParams::lambda_u)
        .def_readwrite("uMin",        &ctrl::DeePCParams::uMin)
        .def_readwrite("uMax",        &ctrl::DeePCParams::uMax)
        .def_readwrite("rho",         &ctrl::DeePCParams::rho)
        .def_readwrite("admm_iters",  &ctrl::DeePCParams::admm_iters)
        .def_readwrite("admm_tol",    &ctrl::DeePCParams::admm_tol);

    py::class_<ctrl::DeePC, ctrl::IController, std::shared_ptr<ctrl::DeePC>>(m, "DeePC", R"doc(
Data-Enabled Predictive Control (DeePC) - SISO.

Implements Coulson, Lygeros & Dorfler (2019): uses Willems' Fundamental Lemma to
represent all LTI system behaviours via a Hankel matrix built from offline data,
without explicit system identification.  Solves a regularised QP via ADMM with a
pre-factored constant Hessian (LDLT) so per-step cost is O(M^2).

Example::

    import ctrl_toolbox as ctrl
    import numpy as np

    p = ctrl.DeePCParams()
    p.T_ini = 5; p.N = 10; p.Q = 10.0; p.R = 0.1; p.lambda_g = 1.0
    p.uMin = -1.0; p.uMax = 1.0
    deepc = ctrl.DeePC(p, Ts=0.1)

    # Collect offline PRBS data
    deepc.collect_data(u_prbs, y_prbs)   # numpy 1-D arrays, length >= T_ini+N+1
    deepc.set_reference(1.0)

    # Control loop
    u = deepc.compute(y_meas)

Notes:
    - collect_data() must be called before compute().
    - is_healthy() returns False if the ADMM solver did not converge within admm_iters;
      the best-iterate output is still applied.
)doc")
        .def(py::init<const ctrl::DeePCParams&, double>(),
             py::arg("params"), py::arg("Ts"),
             "Construct a DeePC controller with the given params and sample time.")
        .def("collect_data",
             &ctrl::DeePC::collectData,
             py::arg("u_data"), py::arg("y_data"),
             "Provide offline persistently exciting data and build Hankel matrices. "
             "u_data and y_data must be numpy 1-D arrays of equal length >= T_ini+N+1.")
        .def("is_data_collected",
             &ctrl::DeePC::isDataCollected,
             "True once collect_data() has been called successfully.")
        .def("hankel_columns",
             &ctrl::DeePC::hankelColumns,
             "Number of Hankel columns M = N_data - T_ini - N + 1.")
        .def("set_reference",
             &ctrl::DeePC::setReference,
             py::arg("r"),
             "Set the constant output reference for all horizon steps.")
        .def("compute",
             &ctrl::DeePC::compute,
             py::arg("y_meas"),
             "Advance one step: update history, solve ADMM QP, return u[k].")
        .def("reset",        &ctrl::DeePC::reset,
             "Reset past trajectory buffers (collected Hankel data is preserved).")
        .def("sample_time",  &ctrl::DeePC::sampleTime)
        .def("name",         &ctrl::DeePC::name)
        .def("is_healthy",   &ctrl::DeePC::isHealthy,
             "False if the last ADMM solve did not converge within admm_iters.");

    // -----------------------------------------------------------------------
    // NeuralNetworkController (Phase 3 ML1)
    // -----------------------------------------------------------------------
    py::class_<ctrl::NNLayerSpec> nnlayer(m, "NNLayerSpec",
        "One fully-connected layer: activation(W @ a + b).");
    py::enum_<ctrl::NNLayerSpec::Activation>(nnlayer, "Activation")
        .value("Tanh",     ctrl::NNLayerSpec::Activation::Tanh)
        .value("ReLU",     ctrl::NNLayerSpec::Activation::ReLU)
        .value("Sigmoid",  ctrl::NNLayerSpec::Activation::Sigmoid)
        .value("Linear",   ctrl::NNLayerSpec::Activation::Linear)
        .value("Softplus", ctrl::NNLayerSpec::Activation::Softplus)
        .export_values();
    nnlayer
        .def(py::init<>())
        .def_readwrite("W",          &ctrl::NNLayerSpec::W,          "Weight matrix (out_dim x in_dim).")
        .def_readwrite("b",          &ctrl::NNLayerSpec::b,          "Bias vector (out_dim).")
        .def_readwrite("activation", &ctrl::NNLayerSpec::activation, "Elementwise activation for this layer.");

    py::class_<ctrl::NeuralControllerParams>(m, "NeuralControllerParams",
        "Architecture/saturation parameters for NeuralNetworkController.")
        .def(py::init<>())
        .def_readwrite("layers",           &ctrl::NeuralControllerParams::layers,
                       "Layers in forward order; the last must output a scalar.")
        .def_readwrite("uMin",             &ctrl::NeuralControllerParams::uMin)
        .def_readwrite("uMax",             &ctrl::NeuralControllerParams::uMax)
        .def_readwrite("n_input_features", &ctrl::NeuralControllerParams::n_input_features);

    py::class_<ctrl::NeuralNetworkController, ctrl::IController,
               std::shared_ptr<ctrl::NeuralNetworkController>>(
        m, "NeuralNetworkController", R"doc(
Generic feedforward neural-network controller (fixed forward pass, scalar output).
Import offline-trained weights; run as an IController.

Usage
-----
>>> layer = ctrl.NNLayerSpec(); layer.W = np.array([[-1.0, -2.0]]); layer.b = np.zeros(1)
>>> layer.activation = ctrl.NNLayerSpec.Activation.Linear
>>> p = ctrl.NeuralControllerParams(); p.layers = [layer]; p.n_input_features = 2
>>> nn = ctrl.NeuralNetworkController(p, Ts=0.01)
>>> u = nn.compute_vec(np.array([x1, x2]))[0]
)doc")
        .def(py::init<const ctrl::NeuralControllerParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",      &ctrl::NeuralNetworkController::compute,     py::arg("signal"))
        .def("compute_vec",  &ctrl::NeuralNetworkController::computeVec,  py::arg("features"))
        .def("reset",        &ctrl::NeuralNetworkController::reset)
        .def("sample_time",  &ctrl::NeuralNetworkController::sampleTime)
        .def("name",         &ctrl::NeuralNetworkController::name)
        .def("load_weights", &ctrl::NeuralNetworkController::loadWeights, py::arg("layers"))
        .def("input_features", &ctrl::NeuralNetworkController::inputFeatures)
        .def("num_layers",     &ctrl::NeuralNetworkController::numLayers);

    // -----------------------------------------------------------------------
    // NNAdaptiveController (Phase 3 ML2)
    // -----------------------------------------------------------------------
    py::class_<ctrl::NNAdaptiveParams>(m, "NNAdaptiveParams",
        "Parameters for NNAdaptiveController (online output-weight adaptation).")
        .def(py::init<>())
        .def_readwrite("nn",          &ctrl::NNAdaptiveParams::nn,
                       "Base ML1 network; hidden layers fixed, the output layer adapts online.")
        .def_readwrite("gamma_adapt", &ctrl::NNAdaptiveParams::gamma_adapt, "Adaptation gain.")
        .def_readwrite("sigma_mod",   &ctrl::NNAdaptiveParams::sigma_mod,
                       "sigma-modification leakage (bounds weight drift; same role as MRAC sigma).")
        .def_readwrite("a_m",         &ctrl::NNAdaptiveParams::a_m, "Reference-model pole (|a_m| < 1).")
        .def_readwrite("b_m",         &ctrl::NNAdaptiveParams::b_m, "Reference-model gain.")
        .def_readwrite("uMin",        &ctrl::NNAdaptiveParams::uMin)
        .def_readwrite("uMax",        &ctrl::NNAdaptiveParams::uMax);

    py::class_<ctrl::NNAdaptiveController, ctrl::IController,
               std::shared_ptr<ctrl::NNAdaptiveController>>(
        m, "NNAdaptiveController", R"doc(
Lyapunov-stable online adaptation of a neural network's output-layer weights, layered on
the ML1 forward-pass core. compute(y_plant) takes the plant output (MRAC convention);
call set_reference(r) once per step first.
)doc")
        .def(py::init<const ctrl::NNAdaptiveParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",        &ctrl::NNAdaptiveController::compute, py::arg("y_plant"))
        .def("set_reference",  &ctrl::NNAdaptiveController::setReference, py::arg("r"))
        .def("reset",          &ctrl::NNAdaptiveController::reset)
        .def("sample_time",    &ctrl::NNAdaptiveController::sampleTime)
        .def("name",           &ctrl::NNAdaptiveController::name)
        .def("model_output",   &ctrl::NNAdaptiveController::modelOutput)
        .def("output_weight_norm", &ctrl::NNAdaptiveController::outputWeightNorm);

    // -----------------------------------------------------------------------
    // NonlinearIMC (Phase 3 NC3)
    // -----------------------------------------------------------------------
    py::class_<ctrl::NonlinearIMCParams>(m, "NonlinearIMCParams",
        "Parameters for NonlinearIMC.")
        .def(py::init<>())
        .def_readwrite("filter_lambda", &ctrl::NonlinearIMCParams::filter_lambda,
                       "IMC filter pole in (0,1): y_f[k] = lambda*y_f[k-1] + (1-lambda)*u.")
        .def_readwrite("uMin",          &ctrl::NonlinearIMCParams::uMin)
        .def_readwrite("uMax",          &ctrl::NonlinearIMCParams::uMax);

    py::class_<ctrl::NonlinearIMC, ctrl::IController,
               std::shared_ptr<ctrl::NonlinearIMC>>(
        m, "NonlinearIMC", R"doc(
Nonlinear Internal Model Control: a user-supplied nonlinear process model runs in parallel
with the plant; the controller inverts the model for feedforward and corrects on the
model-mismatch residual (the nonlinear analogue of the SmithPredictor structure).

Usage
-----
>>> model   = lambda x, u: ...   # one-step process model y_hat = f(x, u)
>>> inverse = lambda x, y_t: ... # u that drives the model output to y_t
>>> imc = ctrl.NonlinearIMC(model, inverse, params, Ts=0.01)
>>> imc.set_state(x)
>>> u = imc.compute(r - y)
)doc")
        .def(py::init<ctrl::NonlinearIMC::ModelFn, ctrl::NonlinearIMC::InverseModelFn,
                      const ctrl::NonlinearIMCParams &, double>(),
             py::arg("model"), py::arg("inverse"), py::arg("params"), py::arg("Ts"))
        .def("compute",     &ctrl::NonlinearIMC::compute, py::arg("error"))
        .def("reset",       &ctrl::NonlinearIMC::reset)
        .def("sample_time", &ctrl::NonlinearIMC::sampleTime)
        .def("name",        &ctrl::NonlinearIMC::name)
        .def("set_state",   &ctrl::NonlinearIMC::setState, py::arg("x"));

    // -----------------------------------------------------------------------
    // ValueIterationSolver (Phase 4 OC2)
    // -----------------------------------------------------------------------
    py::class_<ctrl::DPGridParams>(m, "DPGridParams",
        "Grid and solver parameters for ValueIterationSolver.")
        .def(py::init<>())
        .def_readwrite("x_min",               &ctrl::DPGridParams::x_min)
        .def_readwrite("x_max",               &ctrl::DPGridParams::x_max)
        .def_readwrite("n_grid_per_dim",      &ctrl::DPGridParams::n_grid_per_dim)
        .def_readwrite("u_min",               &ctrl::DPGridParams::u_min)
        .def_readwrite("u_max",               &ctrl::DPGridParams::u_max)
        .def_readwrite("n_grid_u",            &ctrl::DPGridParams::n_grid_u)
        .def_readwrite("discount",            &ctrl::DPGridParams::discount)
        .def_readwrite("max_iter",            &ctrl::DPGridParams::max_iter)
        .def_readwrite("tol",                 &ctrl::DPGridParams::tol)
        .def_readwrite("out_of_grid_penalty", &ctrl::DPGridParams::out_of_grid_penalty);

    py::class_<ctrl::ValueIterationSolver>(m, "ValueIterationSolver", R"doc(
Grid-based dynamic programming / value iteration for low-dimensional (n<=3-4 states)
discounted-infinite-horizon optimal control, where a globally optimal policy is wanted and
MPC's local, continuous optimization isn't required or trusted.

Usage
-----
>>> gp = ctrl.DPGridParams()
>>> gp.x_min = np.array([-1.0, -1.0]); gp.x_max = np.array([1.0, 1.0])
>>> gp.n_grid_per_dim = np.array([41, 41])
>>> gp.u_min = np.array([-5.0]); gp.u_max = np.array([5.0])
>>> vi = ctrl.ValueIterationSolver(dynamics_fn, cost_fn, gp)
>>> vi.solve()
>>> u = vi.policy(x)
)doc")
        .def(py::init<ctrl::ValueIterationSolver::DynamicsFn,
                      ctrl::ValueIterationSolver::StageCost,
                      const ctrl::DPGridParams &>(),
             py::arg("dynamics_fn"), py::arg("cost_fn"), py::arg("params"))
        .def("solve",       &ctrl::ValueIterationSolver::solve)
        .def("policy",      &ctrl::ValueIterationSolver::policy, py::arg("x"))
        .def("value",       &ctrl::ValueIterationSolver::value, py::arg("x"))
        .def("converged",   &ctrl::ValueIterationSolver::converged)
        .def("iterations",  &ctrl::ValueIterationSolver::iterations)
        .def("final_delta", &ctrl::ValueIterationSolver::finalDelta);

    // Fuzzy bindings are fully implemented in advanced_bindings.cpp (bind_advanced).
    // The stale TODO stub that used to live here was removed (audit 2026-06-13, Finding 34).
}
