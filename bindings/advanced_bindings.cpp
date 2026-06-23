#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "ControllerToolbox.h"

namespace py = pybind11;

void bind_advanced(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // RepetitiveController + RepetitiveParams
    // -----------------------------------------------------------------------
    py::class_<ctrl::RepetitiveParams>(m, "RepetitiveParams",
        "Tuning parameters for RepetitiveController.")
        .def(py::init<>())
        .def_readwrite("period_steps", &ctrl::RepetitiveParams::periodSteps,
                       "Period N in samples (T = N * Ts).")
        .def_readwrite("Krc",          &ctrl::RepetitiveParams::Krc,
                       "Learning gain. Small = slow but stable. Typical: 0.3-0.8.")
        .def_readwrite("Q",            &ctrl::RepetitiveParams::Q,
                       "Forgetting/robustness factor in (0, 1]. Typical: 0.95-1.0.")
        .def_readwrite("uMin",         &ctrl::RepetitiveParams::uMin,
                       "Output saturation lower limit.")
        .def_readwrite("uMax",         &ctrl::RepetitiveParams::uMax,
                       "Output saturation upper limit.");

    py::class_<ctrl::RepetitiveController, ctrl::IController,
               std::shared_ptr<ctrl::RepetitiveController>>(m, "RepetitiveController", R"doc(
Discrete-time plug-in Repetitive Controller (RC).

Cancels periodic references and disturbances with known period T = N * Ts by wrapping
any IController base and adding a learned periodic correction.

Learning law:
    v[k]  = Q * v[k-N] + Krc * e[k]
    u[k]  = u_base[k] + v[k]

Stability: Q < 1 provides robustness margin against model mismatch.

Example
-------
>>> pid = ctrl.DiscretePID(pid_p, Ts)          # base stabilising controller
>>> rp  = ctrl.RepetitiveParams()
>>> rp.period_steps = 100; rp.Krc = 0.5; rp.Q = 0.98
>>> rc  = ctrl.RepetitiveController(pid, rp, Ts)
>>> u   = rc.compute(error)                     # replaces pid.compute(error)
>>> print(rc.correction())                      # current repetitive correction v[k]
)doc")
        .def(py::init<std::shared_ptr<ctrl::IController>,
                      const ctrl::RepetitiveParams &, double>(),
             py::arg("inner"), py::arg("params"), py::arg("Ts"),
             "Wrap any IController with a repetitive learning layer.")
        .def("compute",      &ctrl::RepetitiveController::compute,     py::arg("error"))
        .def("reset",        &ctrl::RepetitiveController::reset)
        .def("sample_time",  &ctrl::RepetitiveController::sampleTime)
        .def("set_params",   &ctrl::RepetitiveController::setParams,   py::arg("params"),
             "Update parameters and resize the learning buffer if period_steps changed.")
        .def("params",       &ctrl::RepetitiveController::params,
             py::return_value_policy::copy)
        .def("correction",   &ctrl::RepetitiveController::correction,
             "Current repetitive correction v[k] - approaches negative of periodic disturbance.");

    // -----------------------------------------------------------------------
    // Fuzzy module (CTRL_HAS_FUZZY)
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_FUZZY)
    // --- MF type enum --------------------------------------------------------
    py::enum_<ctrl::MF::Type>(m, "MFType", "Membership function shape selector.")
        .value("Triangular",   ctrl::MF::Type::Triangular)
        .value("Trapezoidal",  ctrl::MF::Type::Trapezoidal)
        .value("Gaussian",     ctrl::MF::Type::Gaussian)
        .value("Singleton",    ctrl::MF::Type::Singleton)
        .value("ShoulderLeft", ctrl::MF::Type::ShoulderLeft)
        .value("ShoulderRight",ctrl::MF::Type::ShoulderRight)
        .export_values();

    // --- MF struct + factory functions ---------------------------------------
    py::class_<ctrl::MF>(m, "MF",
        "Membership function (value-type tagged union, stack-allocatable).")
        .def(py::init<>())
        .def_readwrite("type", &ctrl::MF::type)
        .def("__call__", [](const ctrl::MF &mf, double x) { return mf(x); },
             py::arg("x"), "Evaluate membership degree at x.");

    m.def("mf_triangular",    &ctrl::mfTriangular,   py::arg("a"), py::arg("c"), py::arg("b"),
          "Triangular MF: peaks at c, zero at a and b.");
    m.def("mf_trapezoidal",   &ctrl::mfTrapezoidal,  py::arg("a"), py::arg("b"), py::arg("c"), py::arg("d"),
          "Trapezoidal MF: flat-top from b to c, zero outside [a, d].");
    m.def("mf_gaussian",      &ctrl::mfGaussian,     py::arg("mean"), py::arg("sigma"),
          "Gaussian MF: exp(-0.5*((x-mean)/sigma)^2).");
    m.def("mf_singleton",     &ctrl::mfSingleton,    py::arg("value"),
          "Singleton MF: 1 at x == value, 0 elsewhere (Takagi-Sugeno consequents).");
    m.def("mf_shoulder_left", &ctrl::mfShoulderLeft, py::arg("a"), py::arg("b"),
          "Left-shoulder MF: 1 for x <= a, linear 1->0 from a to b.");
    m.def("mf_shoulder_right",&ctrl::mfShoulderRight,py::arg("a"), py::arg("b"),
          "Right-shoulder MF: linear 0->1 from a to b, 1 for x >= b.");

    // --- LinguisticTerm + LinguisticVariable ---------------------------------
    py::class_<ctrl::LinguisticTerm>(m, "LinguisticTerm",
        "A named fuzzy term with its membership function.")
        .def(py::init<>())
        .def_readwrite("name", &ctrl::LinguisticTerm::name)
        .def_readwrite("mf",   &ctrl::LinguisticTerm::mf);

    m.def("lt_singleton", &ctrl::ltSingleton, py::arg("name"), py::arg("value"),
          "Build a LinguisticTerm for a Takagi-Sugeno singleton output.");

    py::class_<ctrl::LinguisticVariable>(m, "LinguisticVariable",
        "A fuzzy variable with a universe of discourse and linguistic terms.")
        .def(py::init<>())
        .def_readwrite("name",  &ctrl::LinguisticVariable::name)
        .def_readwrite("lo",    &ctrl::LinguisticVariable::lo,
                       "Lower bound of universe of discourse.")
        .def_readwrite("hi",    &ctrl::LinguisticVariable::hi,
                       "Upper bound of universe of discourse.")
        .def_readwrite("terms", &ctrl::LinguisticVariable::terms,
                       "List of LinguisticTerm objects for this variable.")
        .def("fuzzify",    &ctrl::LinguisticVariable::fuzzify,   py::arg("x"),
             "Evaluate all membership degrees for crisp input x.")
        .def("term_index", &ctrl::LinguisticVariable::termIndex, py::arg("name"),
             "Find term index by name; returns -1 if not found.");

    // --- Rule ----------------------------------------------------------------
    py::class_<ctrl::Antecedent>(m, "Antecedent",
        "One input condition in a fuzzy rule antecedent.")
        .def(py::init<>())
        .def_readwrite("input_idx", &ctrl::Antecedent::input_idx,
                       "Index of the input variable (0-based).")
        .def_readwrite("term_idx",  &ctrl::Antecedent::term_idx,
                       "Index of the linguistic term within that variable.");

    py::class_<ctrl::Rule>(m, "Rule", "One fuzzy IF-THEN rule.")
        .def(py::init<>())
        .def_readwrite("antecedents",       &ctrl::Rule::antecedents,
                       "AND-connected list of Antecedent conditions.")
        .def_readwrite("consequent_term_idx",&ctrl::Rule::consequent_term_idx,
                       "Index of the output term that fires.")
        .def_readwrite("weight",            &ctrl::Rule::weight,
                       "Optional rule weight in (0, 1].");

    // --- Inference/Defuzz enums ----------------------------------------------
    py::enum_<ctrl::InferenceMethod>(m, "InferenceMethod")
        .value("Mamdani",      ctrl::InferenceMethod::Mamdani)
        .value("TakagiSugeno", ctrl::InferenceMethod::TakagiSugeno)
        .export_values();

    py::enum_<ctrl::DefuzzMethod>(m, "DefuzzMethod")
        .value("CoG",            ctrl::DefuzzMethod::CoG)
        .value("WeightedAverage",ctrl::DefuzzMethod::WeightedAverage)
        .export_values();

    // --- FuzzySystemParams + FuzzySystem -------------------------------------
    py::class_<ctrl::FuzzySystemParams>(m, "FuzzySystemParams",
        "Configuration parameters for FuzzySystem.")
        .def(py::init<>())
        .def_readwrite("inference",       &ctrl::FuzzySystemParams::inference)
        .def_readwrite("defuzz",          &ctrl::FuzzySystemParams::defuzz)
        .def_readwrite("cog_resolution",  &ctrl::FuzzySystemParams::cog_resolution,
                       "Discretisation points for CoG defuzzification (Mamdani).")
        .def_readwrite("uMin",            &ctrl::FuzzySystemParams::uMin)
        .def_readwrite("uMax",            &ctrl::FuzzySystemParams::uMax);

    py::class_<ctrl::FuzzySystem>(m, "FuzzySystem", R"doc(
Core Mamdani / Takagi-Sugeno fuzzy inference engine.

Build: addInput() / addOutput() / addRule() -> evaluate([crisp inputs]).

Example (simple 2-input Mamdani)
---------------------------------
>>> fs = ctrl.FuzzySystem()
>>> fs.params.uMin = -1.0; fs.params.uMax = 1.0
>>> e_var  = ctrl.LinguisticVariable()
>>> e_var.name = "error"; e_var.lo = -1.0; e_var.hi = 1.0
>>> t = ctrl.LinguisticTerm(); t.name = "Negative"; t.mf = ctrl.mf_shoulder_left(-1.0, 0.0)
>>> e_var.terms.append(t)
>>> # ... add more terms and rules ...
>>> fs.addInput(e_var)
>>> u = fs.evaluate([error_val])
)doc")
        .def(py::init<>())
        .def_readwrite("params", &ctrl::FuzzySystem::params)
        .def("add_input",    &ctrl::FuzzySystem::addInput,  py::arg("var"),
             "Register an input linguistic variable.")
        .def("add_output",   &ctrl::FuzzySystem::addOutput, py::arg("var"),
             "Register the single output linguistic variable.")
        .def("add_rule",     &ctrl::FuzzySystem::addRule,   py::arg("rule"),
             "Add a rule (validates all indices against registered variables).")
        .def("evaluate",     &ctrl::FuzzySystem::evaluate,  py::arg("inputs"),
             "Run inference and return the crisp defuzzified output.")
        .def("num_inputs",   &ctrl::FuzzySystem::numInputs)
        .def("num_outputs",  &ctrl::FuzzySystem::numOutputs)
        .def("num_rules",    &ctrl::FuzzySystem::numRules)
        .def("input_var",    &ctrl::FuzzySystem::inputVar,  py::arg("i"),
             py::return_value_policy::copy)
        .def("output_var",   &ctrl::FuzzySystem::outputVar, py::arg("i"),
             py::return_value_policy::copy);

    // --- FuzzyPDParams + FuzzyPD ---------------------------------------------
    py::class_<ctrl::FuzzyPDParams>(m, "FuzzyPDParams",
        "Tuning parameters for FuzzyPD.")
        .def(py::init<>())
        .def_readwrite("e_scale",  &ctrl::FuzzyPDParams::e_scale,
                       "Error normalisation: maps +/-e_scale to universe [-1, 1].")
        .def_readwrite("de_scale", &ctrl::FuzzyPDParams::de_scale,
                       "Error-rate normalisation.")
        .def_readwrite("u_scale",  &ctrl::FuzzyPDParams::u_scale,
                       "Output scaling: universe [-1, 1] -> +/-u_scale.")
        .def_readwrite("uMin",     &ctrl::FuzzyPDParams::uMin)
        .def_readwrite("uMax",     &ctrl::FuzzyPDParams::uMax);

    py::class_<ctrl::FuzzyPD, ctrl::IController,
               std::shared_ptr<ctrl::FuzzyPD>>(m, "FuzzyPD", R"doc(
5-term PD fuzzy controller (25-rule diagonal Mamdani table).

Inputs: tracking error e[k] and error-rate de[k] = (e[k]-e[k-1])/Ts.
Scaling: e_norm = e/e_scale, de_norm = de/de_scale; u = u_norm * u_scale.

Example
-------
>>> p = ctrl.FuzzyPDParams(); p.e_scale=1.0; p.de_scale=0.1; p.u_scale=10.0
>>> fpd = ctrl.FuzzyPD(p, Ts=0.01)
>>> u = fpd.compute(error)
)doc")
        .def(py::init<const ctrl::FuzzyPDParams &, double>(),
             py::arg("params"), py::arg("sample_time"))
        .def("compute",      &ctrl::FuzzyPD::compute,     py::arg("error"))
        .def("reset",        &ctrl::FuzzyPD::reset)
        .def("sample_time",  &ctrl::FuzzyPD::sampleTime)
        .def("set_params",   &ctrl::FuzzyPD::setParams,   py::arg("params"))
        .def("params",       &ctrl::FuzzyPD::params,
             py::return_value_policy::copy)
        .def("last_output",  &ctrl::FuzzyPD::lastOutput);

    // --- FuzzyPIDParams + FuzzyPID -------------------------------------------
    py::class_<ctrl::FuzzyPIDParams>(m, "FuzzyPIDParams",
        "Tuning parameters for FuzzyPID (FuzzyPD + crisp integral + anti-windup).")
        .def(py::init<>())
        .def_readwrite("pd",   &ctrl::FuzzyPIDParams::pd,
                       "FuzzyPD scaling parameters.")
        .def_readwrite("Ki",   &ctrl::FuzzyPIDParams::Ki,
                       "Integral gain (0 = pure FuzzyPD).")
        .def_readwrite("Kb",   &ctrl::FuzzyPIDParams::Kb,
                       "Anti-windup back-calculation gain.")
        .def_readwrite("uMin", &ctrl::FuzzyPIDParams::uMin)
        .def_readwrite("uMax", &ctrl::FuzzyPIDParams::uMax);

    py::class_<ctrl::FuzzyPID, ctrl::IController,
               std::shared_ptr<ctrl::FuzzyPID>>(m, "FuzzyPID", R"doc(
FuzzyPD extended with a crisp integral term and back-calculation anti-windup.

u = clamp(FuzzyPD(e, de) + integral, uMin, uMax)
integral += Ki * Ts * e + Kb * (u_sat - u_unsat)    (anti-windup)

Example
-------
>>> fp = ctrl.FuzzyPIDParams()
>>> fp.pd.e_scale = 1.0; fp.pd.u_scale = 10.0; fp.Ki = 0.1; fp.uMin = -10; fp.uMax = 10
>>> fpid = ctrl.FuzzyPID(fp, Ts=0.01)
>>> u = fpid.compute(error)
)doc")
        .def(py::init<const ctrl::FuzzyPIDParams &, double>(),
             py::arg("params"), py::arg("sample_time"))
        .def("compute",       &ctrl::FuzzyPID::compute,      py::arg("error"))
        .def("reset",         &ctrl::FuzzyPID::reset)
        .def("sample_time",   &ctrl::FuzzyPID::sampleTime)
        .def("set_params",    &ctrl::FuzzyPID::setParams,    py::arg("params"))
        .def("params",        &ctrl::FuzzyPID::params,
             py::return_value_policy::copy)
        .def("last_output",   &ctrl::FuzzyPID::lastOutput)
        .def("bumpless_init", &ctrl::FuzzyPID::bumplessInit,
             py::arg("u_target"), py::arg("error"));

    // --- SupervisorParams + SupervisorDecision + FuzzySupervisor -------------
    py::class_<ctrl::SupervisorParams>(m, "SupervisorParams",
        "Tuning parameters for FuzzySupervisor.")
        .def(py::init<>())
        .def_readwrite("e_threshold",      &ctrl::SupervisorParams::e_threshold,
                       "|e| at which error is considered 'Large'.")
        .def_readwrite("trend_threshold",  &ctrl::SupervisorParams::trend_threshold,
                       "d|e|/dt considered 'Increasing fast' [units/s].")
        .def_readwrite("signal_threshold", &ctrl::SupervisorParams::signal_threshold,
                       "relinearize_signal > this triggers the flag.")
        .def_readwrite("cooldown_steps",   &ctrl::SupervisorParams::cooldown_steps,
                       "Minimum steps between consecutive relinearisation triggers.");

    py::class_<ctrl::SupervisorDecision>(m, "SupervisorDecision",
        "Output of a FuzzySupervisor::update() call.")
        .def_readonly("relinearize_signal", &ctrl::SupervisorDecision::relinearize_signal,
                      "Raw fuzzy output in [0, 1].")
        .def_readonly("relinearize",        &ctrl::SupervisorDecision::relinearize,
                      "True when signal > signal_threshold and cooldown expired.")
        .def_readonly("error_norm",         &ctrl::SupervisorDecision::error_norm,
                      "Normalised |e| / e_threshold used as fuzzy input.")
        .def_readonly("trend",              &ctrl::SupervisorDecision::trend,
                      "Normalised d|e|/dt / trend_threshold used as fuzzy input.");

    py::class_<ctrl::FuzzySupervisor>(m, "FuzzySupervisor", R"doc(
Fuzzy supervisory monitor for adaptive plant re-linearisation.

Inputs: |e| / e_threshold (Small/Medium/Large) and d|e|/dt / trend_threshold.
Output: relinearize_signal in [0, 1]; relinearize=True when signal exceeds threshold.

Example (adaptive MPC loop)
----------------------------
>>> sup = ctrl.FuzzySupervisor(ctrl.SupervisorParams(), Ts)
>>> decision = sup.update(abs(error))
>>> if decision.relinearize:
...     mpc.set_plant(new_linearisation)
)doc")
        .def(py::init<const ctrl::SupervisorParams &, double>(),
             py::arg("params"), py::arg("sample_time"))
        .def("update",     &ctrl::FuzzySupervisor::update,    py::arg("abs_error"),
             "Feed current absolute error; returns SupervisorDecision.")
        .def("reset",      &ctrl::FuzzySupervisor::reset)
        .def("params",     &ctrl::FuzzySupervisor::params,
             py::return_value_policy::copy)
        .def("set_params", &ctrl::FuzzySupervisor::setParams, py::arg("params"));
#endif  // CTRL_HAS_FUZZY

    // -----------------------------------------------------------------------
    // SubspaceID  (CTRL_HAS_SUBSPACE)
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_SUBSPACE)
    py::class_<ctrl::SubspaceIDResult>(m, "SubspaceIDResult", R"doc(
Result of SubspaceID::n4sid() identification.

Check success before using model. Plot singular_values to verify order selection.
)doc")
        .def_property_readonly("model",
             [](const ctrl::SubspaceIDResult &r) -> py::object {
                 if (!r.model) return py::none();
                 return py::cast(*r.model);
             },
             "Identified StateSpace model or None if identification failed.")
        .def_readonly("singular_values", &ctrl::SubspaceIDResult::singularValues,
                      "All singular values of the oblique projection matrix L32.")
        .def_readonly("kalman_gain",    &ctrl::SubspaceIDResult::kalmanGain,
                      "Estimated Kalman gain (n x p). Use directly in KalmanFilter.")
        .def_readonly("innov_cov",      &ctrl::SubspaceIDResult::innovCov,
                      "Innovation covariance Lambda (p x p). Use as R_noise in KalmanFilter.")
        .def_readonly("success",        &ctrl::SubspaceIDResult::success,
                      "True when identification succeeded.")
        .def_readonly("message",        &ctrl::SubspaceIDResult::message,
                      "Error description on failure.")
        .def("get_model", [](const ctrl::SubspaceIDResult &r) -> ctrl::StateSpace {
             if (!r.success || !r.model)
                 throw std::runtime_error("SubspaceIDResult: identification failed - " + r.message);
             return *r.model;
        }, "Return the identified StateSpace; raises RuntimeError if success=False.");

    m.def("n4sid",
          &ctrl::n4sid,
          py::arg("Y"), py::arg("U"),
          py::arg("n_order"), py::arg("i_horizon"),
          py::arg("Ts"), py::arg("svd_tol") = -1.0,
          R"doc(
Batch subspace state-space identification (N4SID / MOESP).

Parameters
----------
Y         : Output data (p x N) - rows=outputs, cols=time.
U         : Input data  (m x N) - rows=inputs,  cols=time.
n_order   : Desired model order.  Choose by inspecting result.singular_values.
i_horizon : Block-row count (recommend >= 2*n_order/p, minimum n_order+1).
Ts        : Sample time [s].
svd_tol   : SVD truncation floor (pass -1 to disable).

Returns SubspaceIDResult. Check result.success before using result.get_model().

Example
-------
>>> res = ctrl.n4sid(Y, U, n_order=2, i_horizon=10, Ts=0.01)
>>> if res.success:
...     plant = res.get_model()
...     kf = ctrl.KalmanFilter(plant, Q, res.innov_cov)
)doc");

    m.def("suggest_order",
          [](const Eigen::VectorXd &sv, double threshold, int max_order) {
              return ctrl::suggestOrder(sv, threshold, max_order);
          },
          py::arg("sv"), py::arg("threshold") = 0.01, py::arg("max_order") = -1,
          R"doc(
Automated system order selection from the singular-value spectrum.

Uses the maximum-consecutive-ratio heuristic: returns the index where
sv(i)/sv(i+1) is largest (sharpest drop) as the suggested order.

Parameters
----------
sv         : singular_values from a SubspaceIDResult.
threshold  : Relative floor; values below threshold*sv(0) are noise.
max_order  : Hard cap on the returned order (-1 = no cap).

Returns suggested model order >= 1.
)doc");
#endif  // CTRL_HAS_SUBSPACE

    // -----------------------------------------------------------------------
    // DiscreteHinf + MixedSensitivity  (CTRL_HAS_HINF)
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_HINF)
    // --- GeneralisedPlant ----------------------------------------------------
    py::class_<ctrl::GeneralisedPlant>(m, "GeneralisedPlant", R"doc(
Generalised plant for Hinf synthesis (n states, nw exogenous inputs, nu control inputs,
nz performance outputs, ny measurement outputs).

Use MixedSensitivity.build() to assemble from a SISO plant G + weighting functions.
)doc")
        .def(py::init<>())
        .def_readwrite("A",   &ctrl::GeneralisedPlant::A)
        .def_readwrite("B1",  &ctrl::GeneralisedPlant::B1)
        .def_readwrite("B2",  &ctrl::GeneralisedPlant::B2)
        .def_readwrite("C1",  &ctrl::GeneralisedPlant::C1)
        .def_readwrite("C2",  &ctrl::GeneralisedPlant::C2)
        .def_readwrite("D11", &ctrl::GeneralisedPlant::D11)
        .def_readwrite("D12", &ctrl::GeneralisedPlant::D12)
        .def_readwrite("D21", &ctrl::GeneralisedPlant::D21)
        .def_readwrite("D22", &ctrl::GeneralisedPlant::D22)
        .def_readwrite("Ts",  &ctrl::GeneralisedPlant::Ts)
        .def("state_size", &ctrl::GeneralisedPlant::stateSize)
        .def("nw",         &ctrl::GeneralisedPlant::nw)
        .def("nu",         &ctrl::GeneralisedPlant::nu)
        .def("nz",         &ctrl::GeneralisedPlant::nz)
        .def("ny",         &ctrl::GeneralisedPlant::ny);

    // --- HinfParams ----------------------------------------------------------
    py::class_<ctrl::HinfParams>(m, "HinfParams",
        "Hinf synthesis algorithm parameters.")
        .def(py::init<>())
        .def_readwrite("gamma_init",    &ctrl::HinfParams::gammaInit,
                       "Initial upper bound for gamma bisection.")
        .def_readwrite("gamma_tol",     &ctrl::HinfParams::gammaTol,
                       "Stop bisection when bracket < gamma_tol.")
        .def_readwrite("max_iter",      &ctrl::HinfParams::maxIter,
                       "Maximum bisection iterations.")
        .def_readwrite("dare_tol",      &ctrl::HinfParams::dareTol,
                       "DARE convergence tolerance.")
        .def_readwrite("dare_max_iter", &ctrl::HinfParams::dareMaxIter,
                       "DARE iteration limit.");

    // --- HinfResult ----------------------------------------------------------
    py::class_<ctrl::HinfResult>(m, "HinfResult", R"doc(
Result returned by DiscreteHinf.solve().

Check feasible before constructing a DiscreteHinf controller.
Controller matrices: xk[k+1] = Ak*xk + Bk*y,  u = Ck*xk + Dk*y.
)doc")
        .def_readonly("feasible",         &ctrl::HinfResult::feasible)
        .def_readonly("achieved_gamma",   &ctrl::HinfResult::achievedGamma)
        .def_readonly("Ts",               &ctrl::HinfResult::Ts)
        .def_readonly("Ak",               &ctrl::HinfResult::Ak)
        .def_readonly("Bk",               &ctrl::HinfResult::Bk)
        .def_readonly("Ck",               &ctrl::HinfResult::Ck)
        .def_readonly("Dk",               &ctrl::HinfResult::Dk)
        .def_readonly("X_inf",            &ctrl::HinfResult::X_inf,
                      "Control Riccati solution (diagnostics).")
        .def_readonly("Y_inf",            &ctrl::HinfResult::Y_inf,
                      "Filter Riccati solution (diagnostics).")
        .def_readonly("dare_iters_x",     &ctrl::HinfResult::dareItersX)
        .def_readonly("dare_iters_y",     &ctrl::HinfResult::dareItersY)
        .def_readonly("dare_conv_x",      &ctrl::HinfResult::dareConvX)
        .def_readonly("dare_conv_y",      &ctrl::HinfResult::dareConvY)
        .def_readonly("spectral_radius",  &ctrl::HinfResult::spectralRadius,
                      "rho(Xinf * Yinf); must be < gamma^2 for feasibility.");

    // --- DiscreteHinf --------------------------------------------------------
    py::class_<ctrl::DiscreteHinf, ctrl::IController,
               std::shared_ptr<ctrl::DiscreteHinf>>(m, "DiscreteHinf", R"doc(
Discrete-time dynamic output-feedback Hinf controller.

Use DiscreteHinf.solve() to synthesise, then construct with the HinfResult.

Example (mixed-sensitivity design)
------------------------------------
>>> G  = ctrl.c2d(plant_cont, Ts, ctrl.C2dMethod.ZOH)
>>> W1 = ctrl.MixedSensitivity.make_W1(omega_B=1.0, M=2.0, eps=0.001, Ts=Ts)
>>> W2 = ctrl.MixedSensitivity.make_W2_constant(gain=0.1, Ts=Ts)
>>> W3 = ctrl.MixedSensitivity.make_W3(omega_T=50.0, Mt=1.5, eps=0.001, Ts=Ts)
>>> P  = ctrl.MixedSensitivity.build(G, W1, W2, W3)
>>> hp = ctrl.HinfParams(); hp.gamma_init = 10.0
>>> result = ctrl.DiscreteHinf.solve(P, hp)
>>> if result.feasible:
...     ctrl_hinf = ctrl.DiscreteHinf(result)
...     u = ctrl_hinf.compute(y_measurement)
)doc")
        .def(py::init<const ctrl::HinfResult &>(),
             py::arg("result"),
             "Construct from a completed synthesis result (result.feasible must be True).")
        .def_static("solve",
                    &ctrl::DiscreteHinf::solve,
                    py::arg("P"), py::arg("params") = ctrl::HinfParams{},
                    "Solve the discrete-time Hinf optimal control problem via gamma bisection.")
        .def("compute",          &ctrl::DiscreteHinf::compute,        py::arg("signal"),
             "SISO interface: signal = measurement y[k] (NOT the error).")
        .def("compute_vec",      &ctrl::DiscreteHinf::computeVec,     py::arg("y"),
             py::return_value_policy::copy,
             "MIMO interface: y = measurement vector (ny,); returns u (nu,).")
        .def("reset",            &ctrl::DiscreteHinf::reset)
        .def("sample_time",      &ctrl::DiscreteHinf::sampleTime)
        .def("controller_state", &ctrl::DiscreteHinf::controllerState,
             py::return_value_policy::copy,
             "Current internal controller state xk (nk,).")
        .def("achieved_gamma",   &ctrl::DiscreteHinf::achievedGamma,
             "Achieved Hinf bound gamma from synthesis.")
        .def("Ak", &ctrl::DiscreteHinf::Ak, py::return_value_policy::copy)
        .def("Bk", &ctrl::DiscreteHinf::Bk, py::return_value_policy::copy)
        .def("Ck", &ctrl::DiscreteHinf::Ck, py::return_value_policy::copy)
        .def("Dk", &ctrl::DiscreteHinf::Dk, py::return_value_policy::copy);

    // --- H2Params --------------------------------------------------------------
    py::class_<ctrl::H2Params>(m, "H2Params",
        "H2 synthesis parameters (currently empty - see DiscreteH2 docs).")
        .def(py::init<>());

    // --- H2Result --------------------------------------------------------------
    py::class_<ctrl::H2Result>(m, "H2Result", R"doc(
Result returned by DiscreteH2.solve().

Check feasible before constructing a DiscreteH2 controller.
Controller matrices: xk[k+1] = Ak*xk + Bk*y,  u = Ck*xk + Dk*y.
)doc")
        .def_readonly("feasible",         &ctrl::H2Result::feasible)
        .def_readonly("achieved_h2_norm", &ctrl::H2Result::achievedH2Norm)
        .def_readonly("Ts",               &ctrl::H2Result::Ts)
        .def_readonly("Ak",               &ctrl::H2Result::Ak)
        .def_readonly("Bk",               &ctrl::H2Result::Bk)
        .def_readonly("Ck",               &ctrl::H2Result::Ck)
        .def_readonly("Dk",               &ctrl::H2Result::Dk)
        .def_readonly("X",                &ctrl::H2Result::X,
                      "Control Riccati solution (diagnostics).")
        .def_readonly("Y",                &ctrl::H2Result::Y,
                      "Filter Riccati solution (diagnostics).")
        .def_readonly("dare_conv_x",       &ctrl::H2Result::dareConvX)
        .def_readonly("dare_conv_y",       &ctrl::H2Result::dareConvY);

    // --- DiscreteH2 --------------------------------------------------------------
    py::class_<ctrl::DiscreteH2, ctrl::IController,
               std::shared_ptr<ctrl::DiscreteH2>>(m, "DiscreteH2", R"doc(
Discrete-time dynamic output-feedback H2-optimal (LQG) controller.

Use DiscreteH2.solve() to synthesise, then construct with the H2Result. Requires a
GeneralisedPlant with D11 = 0 and D22 = 0 (see class docs) - most MixedSensitivity-built
plants are not usable here since their W1/W3 weights give D11 != 0.

Example
-------
>>> P = ctrl.GeneralisedPlant()  # hand-built, D11 = D22 = 0
>>> result = ctrl.DiscreteH2.solve(P)
>>> if result.feasible:
...     h2 = ctrl.DiscreteH2(result)
...     u = h2.compute(y_measurement)
)doc")
        .def(py::init<const ctrl::H2Result &>(),
             py::arg("result"),
             "Construct from a completed synthesis result (result.feasible must be True).")
        .def_static("solve",
                    &ctrl::DiscreteH2::solve,
                    py::arg("P"), py::arg("params") = ctrl::H2Params{},
                    "Solve the discrete-time H2-optimal output-feedback control problem.")
        .def("compute",          &ctrl::DiscreteH2::compute,        py::arg("signal"),
             "SISO interface: signal = measurement y[k] (NOT the error).")
        .def("compute_vec",      &ctrl::DiscreteH2::computeVec,     py::arg("y"),
             py::return_value_policy::copy,
             "MIMO interface: y = measurement vector (ny,); returns u (nu,).")
        .def("reset",            &ctrl::DiscreteH2::reset)
        .def("sample_time",      &ctrl::DiscreteH2::sampleTime)
        .def("controller_state", &ctrl::DiscreteH2::controllerState,
             py::return_value_policy::copy,
             "Current internal controller state xk (n,).")
        .def("achieved_h2_norm", &ctrl::DiscreteH2::achievedH2Norm,
             "Achieved H2 norm from synthesis.")
        .def("Ak", &ctrl::DiscreteH2::Ak, py::return_value_policy::copy)
        .def("Bk", &ctrl::DiscreteH2::Bk, py::return_value_policy::copy)
        .def("Ck", &ctrl::DiscreteH2::Ck, py::return_value_policy::copy)
        .def("Dk", &ctrl::DiscreteH2::Dk, py::return_value_policy::copy);

    // --- MixedSensitivity ----------------------------------------------------
    py::class_<ctrl::MixedSensitivity>(m, "MixedSensitivity", R"doc(
Helper that assembles a GeneralisedPlant for S/KS/T mixed-sensitivity design.

All static factory methods. Call build() to assemble the plant.
)doc")
        .def_static("make_W1",
                    &ctrl::MixedSensitivity::makeW1,
                    py::arg("omega_B"), py::arg("M"), py::arg("eps"), py::arg("Ts"),
                    "Tracking/disturbance weight W1(s) = (s/M + omegaB)/(s + omegaB*eps). "
                    "Low-freq gain ~ 1/eps, crossover at omegaB.")
        .def_static("make_W2_constant",
                    &ctrl::MixedSensitivity::makeW2constant,
                    py::arg("gain"), py::arg("Ts"),
                    "Flat control effort weight W2 = gain at all frequencies.")
        .def_static("make_W2_highpass",
                    &ctrl::MixedSensitivity::makeW2highpass,
                    py::arg("omega_u"), py::arg("eps"), py::arg("Ts"),
                    "First-order high-pass control effort weight. Rolloff at omega_u.")
        .def_static("make_W3",
                    &ctrl::MixedSensitivity::makeW3,
                    py::arg("omega_T"), py::arg("Mt"), py::arg("eps"), py::arg("Ts"),
                    "Robustness weight W3 for complementary sensitivity rolloff. "
                    "Rollup at omega_T, max complementary sensitivity Mt.")
        .def_static("build",
                    &ctrl::MixedSensitivity::build,
                    py::arg("G"), py::arg("W1"), py::arg("W2"), py::arg("W3"),
                    "Assemble GeneralisedPlant from G + W1 + W2 + W3 (all SISO, same Ts).");
#endif  // CTRL_HAS_HINF
}
