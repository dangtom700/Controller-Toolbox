#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "ControllerToolbox.h"

namespace py = pybind11;

void bind_analysis(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // SOPDTIdentifier + SOPDTModel  (Part 18)
    // -----------------------------------------------------------------------
    py::class_<ctrl::SOPDTModel>(m, "SOPDTModel",
        "Identified SOPDT model parameters: K, tau1, tau2, theta, fitRMSE.")
        .def_readwrite("K",       &ctrl::SOPDTModel::K)
        .def_readwrite("tau1",    &ctrl::SOPDTModel::tau1)
        .def_readwrite("tau2",    &ctrl::SOPDTModel::tau2)
        .def_readwrite("theta",   &ctrl::SOPDTModel::theta)
        .def_readwrite("fitRMSE", &ctrl::SOPDTModel::fitRMSE);

    py::enum_<ctrl::SOPDTMethod>(m, "SOPDTMethod")
        .value("Graphical",    ctrl::SOPDTMethod::Graphical)
        .value("Optimization", ctrl::SOPDTMethod::Optimization)
        .export_values();

    py::class_<ctrl::SOPDTIdentifier>(m, "SOPDTIdentifier", R"doc(
Second-Order Plus Dead-Time step-response identifier.

Example
-------
>>> id = ctrl.SOPDTIdentifier(t, y, step_mag)
>>> m  = id.identify()             # graphical (default)
>>> mo = id.identify(ctrl.SOPDTMethod.Optimization)
>>> pp = ctrl.SOPDTIdentifier.imc_tuning(mo, 2*mo.theta, Ts)
)doc")
        .def(py::init<const std::vector<double> &,
                      const std::vector<double> &,
                      double, double>(),
             py::arg("times"), py::arg("outputs"),
             py::arg("stepMagnitude"), py::arg("stepTime") = 0.0,
             "Construct from time/output vectors and step magnitude.")
        .def("identify",
             &ctrl::SOPDTIdentifier::identify,
             py::arg("method") = ctrl::SOPDTMethod::Graphical,
             "Identify SOPDT parameters. Returns SOPDTModel.")
        .def("evaluate",
             &ctrl::SOPDTIdentifier::evaluate,
             py::arg("m"), py::arg("t"),
             "Evaluate fitted SOPDT step response at time t.")
        .def_static("imc_tuning",
             &ctrl::SOPDTIdentifier::imcTuning,
             py::arg("model"), py::arg("lambdaC"), py::arg("Ts"),
             py::arg("piOnly") = false,
             "IMC-PID tuning from a SOPDT model (Rivera 1986 extended).");

    // -----------------------------------------------------------------------
    // MetricsAnalyzer
    // -----------------------------------------------------------------------
    py::class_<ctrl::TimeDomainMetrics>(m, "TimeDomainMetrics",
        "Time-domain step-response metrics extracted by MetricsAnalyzer.")
        .def_readonly("rise_time",         &ctrl::TimeDomainMetrics::riseTime,
                      "Time to first reach 100% of the final value [s].")
        .def_readonly("settling_time",     &ctrl::TimeDomainMetrics::settlingTime,
                      "Time to enter and remain within the settling band [s].")
        .def_readonly("peak_overshoot",    &ctrl::TimeDomainMetrics::peakOvershoot,
                      "Peak overshoot as a fraction of the final value [0, 1].")
        .def_readonly("steady_state_error",&ctrl::TimeDomainMetrics::steadyStateError,
                      "Steady-state tracking error.");

    py::class_<ctrl::MetricsAnalyzer>(m, "MetricsAnalyzer",
        "Extract time-domain metrics from step-response data.  All methods are static.")
        .def_static("calculate", &ctrl::MetricsAnalyzer::calculate,
             py::arg("t_data"), py::arg("y_data"),
             py::arg("reference") = 1.0, py::arg("final_value_window") = 10,
             R"doc(
Compute step-response metrics.

Parameters
----------
t_data             : 1-D NumPy array of time samples [s].
y_data             : 1-D NumPy array of output samples.
reference          : Setpoint (final desired value).
final_value_window : Fraction of reference used for settling-time band (default 5%).

Returns
-------
TimeDomainMetrics
)doc");

    // -----------------------------------------------------------------------
    // SystemAnalysis
    // -----------------------------------------------------------------------
    py::class_<ctrl::StabilityMargins>(m, "StabilityMargins",
        "Gain and phase margins from SystemAnalysis::calculate_margins().")
        .def_readonly("gain_margin_db",      &ctrl::StabilityMargins::gainMarginDb,
                      "Gain margin [dB].")
        .def_readonly("phase_margin_deg",    &ctrl::StabilityMargins::phaseMarginDeg,
                      "Phase margin [degrees].")
        .def_readonly("w_crossover_gain",    &ctrl::StabilityMargins::wCrossoverGain,
                      "Gain crossover frequency [rad/s] (|L| = 1).")
        .def_readonly("w_crossover_phase",   &ctrl::StabilityMargins::wCrossoverPhase,
                      "Phase crossover frequency [rad/s] (angle(L) = -180 deg).");

    py::class_<ctrl::SystemAnalysis>(m, "SystemAnalysis",
        "Frequency-domain and stability analysis utilities (all static methods).")
        .def_static("get_poles",        &ctrl::SystemAnalysis::getPoles,
                    py::arg("sys"),
                    "Return eigenvalues of A as a complex NumPy array.")
        .def_static("is_discrete_stable", &ctrl::SystemAnalysis::isDiscreteStable,
                    py::arg("sys"),
                    "True if all eigenvalues of A lie strictly inside the unit disk.")
        .def_static("solve_discrete_lyapunov",
                    &ctrl::SystemAnalysis::solveDiscreteLyapunov,
                    py::arg("A"), py::arg("Q"),
                    "Solve A*P*A' - P + Q = 0.  Returns P.")
        .def_static("get_frequency_response",
                    &ctrl::SystemAnalysis::getFrequencyResponse,
                    py::arg("sys"), py::arg("frequencies"),
                    "Evaluate complex frequency response H(e^{j*omega*Ts}) at each frequency.")
        .def_static("calculate_margins",
                    &ctrl::SystemAnalysis::calculateMargins,
                    py::arg("sys"),
                    "Compute gain and phase margins.  Returns StabilityMargins.")
        .def_static("calculate_h_infinity_norm",
                    &ctrl::SystemAnalysis::calculateHInfinityNorm,
                    py::arg("sys"),
                    "Grid approximation of the H-infinity norm (lower bound).");

    // -----------------------------------------------------------------------
    // RepetitiveController
    // -----------------------------------------------------------------------
    // TODO: bind RepetitiveController
    // Constructor takes shared_ptr<IController> inner, period_steps, learning_gain, Q
    // Key methods: compute(error), reset(), stability() -> bool, learning_gain()

    // -----------------------------------------------------------------------
    // DiscreteHinf  (optional - CTRL_HAS_HINF)
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_HINF)
    // TODO: bind HinfResult struct and DiscreteHinf
    // Key challenge: HinfResult contains two StateSpace objects (K and CL)
    // TODO: bind MixedSensitivity helper class
#endif
}
