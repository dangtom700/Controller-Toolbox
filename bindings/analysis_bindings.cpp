#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

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
    // LinearisationHelper  (jacobian_x, jacobian_u, linearise_at_point)
    // -----------------------------------------------------------------------
    m.def("jacobian_x",
        [](const ctrl::StateFunc &f,
           const Eigen::VectorXd &x0,
           const Eigen::VectorXd &u0,
           double eps_scale) {
            return ctrl::jacobianX(f, x0, u0, eps_scale);
        },
        py::arg("f"), py::arg("x0"), py::arg("u0"), py::arg("eps_scale") = 1e-4,
        R"doc(
Central-difference Jacobian of f(x, u) with respect to x at (x0, u0).

Parameters
----------
f         : callable(x: ndarray, u: ndarray) -> ndarray  - continuous-time dynamics.
x0        : Operating-point state  (n,).
u0        : Operating-point input  (m,).
eps_scale : Relative step size (default 1e-4).

Returns
-------
A_c : (output_dim, n) Jacobian matrix  df/dx.
)doc");

    m.def("jacobian_u",
        [](const ctrl::StateFunc &f,
           const Eigen::VectorXd &x0,
           const Eigen::VectorXd &u0,
           double eps_scale) {
            return ctrl::jacobianU(f, x0, u0, eps_scale);
        },
        py::arg("f"), py::arg("x0"), py::arg("u0"), py::arg("eps_scale") = 1e-4,
        R"doc(
Central-difference Jacobian of f(x, u) with respect to u at (x0, u0).

Returns
-------
B_c : (output_dim, m) Jacobian matrix  df/du.
)doc");

    m.def("linearise_at_point",
        [](const ctrl::StateFunc &f,
           const Eigen::VectorXd &x0,
           const Eigen::VectorXd &u0,
           double Ts,
           double eps_scale) {
            return ctrl::lineariseAtPoint(f, x0, u0, Ts, eps_scale);
        },
        py::arg("f"), py::arg("x0"), py::arg("u0"),
        py::arg("Ts"), py::arg("eps_scale") = 1e-4,
        R"doc(
Linearise a continuous-time nonlinear system at (x0, u0) and ZOH-discretise.

Uses identity output (y = x, C = I, D = 0).  For a custom output function use
linearise_at_point_with_output().

Parameters
----------
f    : callable(x: ndarray, u: ndarray) -> ndarray - continuous-time xdot = f(x, u).
x0   : Operating-point state  (n,).
u0   : Operating-point input  (m,).
Ts   : Sample time [s] for ZOH discretisation.
eps_scale : Jacobian step scale (default 1e-4).

Returns
-------
StateSpace : Discrete-time linearised model.

Example
-------
>>> sys = ctrl.linearise_at_point(vdp, x0, u0, Ts=0.05)
>>> lqr = ctrl.DiscreteLQR(sys, lqr_params)
)doc");

    m.def("linearise_at_point_with_output",
        [](const ctrl::StateFunc &f,
           const ctrl::MeasFunc  &h,
           const Eigen::VectorXd &x0,
           const Eigen::VectorXd &u0,
           double Ts,
           double eps_scale) {
            return ctrl::lineariseAtPoint(f, h, x0, u0, Ts, eps_scale);
        },
        py::arg("f"), py::arg("h"), py::arg("x0"), py::arg("u0"),
        py::arg("Ts"), py::arg("eps_scale") = 1e-4,
        R"doc(
Linearise with a custom output function h(x, u) and ZOH-discretise.

Parameters
----------
f  : callable(x, u) -> ndarray - continuous-time dynamics.
h  : callable(x, u) -> ndarray - output map y = h(x, u).
x0 : Operating-point state.
u0 : Operating-point input.
Ts : Sample time [s].

Returns
-------
StateSpace : Discrete-time linearised model with C = dh/dx, D = dh/du.
)doc");

    // -----------------------------------------------------------------------
    // BalancedTruncation
    // -----------------------------------------------------------------------
    py::class_<ctrl::TruncationResult>(m, "TruncationResult",
        "Result of balanced truncation: reduced-order StateSpace model + diagnostics.")
        .def_readonly("reduced",                &ctrl::TruncationResult::reduced,
                      "Reduced-order StateSpace model (order r).")
        .def_readonly("hankel_singular_values", &ctrl::TruncationResult::hankelSingularValues,
                      "All n Hankel singular values in descending order.")
        .def_readonly("error_bound",            &ctrl::TruncationResult::errorBound,
                      "H-infinity error bound 2*sum(sigma_{r+1}..sigma_n).")
        .def_readonly("is_stable",              &ctrl::TruncationResult::isStable,
                      "True if all poles of reduced model are strictly inside the unit disk.");

    m.def("balanced_truncate",
          [](const ctrl::StateSpace &sys, int r) { return ctrl::balancedTruncate(sys, r); },
          py::arg("sys"), py::arg("r"),
          R"doc(
Balanced truncation of a discrete-time system to order r.

Parameters
----------
sys : StateSpace  - strictly stable discrete-time system (all |poles| < 1).
r   : int         - target reduced order (1 <= r < n).

Returns
-------
TruncationResult with reduced model, Hankel singular values, H-infinity error bound.

Raises
------
ValueError if sys is not stable, r is out of range, or gramians are ill-conditioned.
)doc");

    m.def("suggest_order",
          [](const ctrl::TruncationResult &res, double tol) { return ctrl::suggestOrder(res, tol); },
          py::arg("result"), py::arg("tol") = 0.01,
          R"doc(
Suggest a reduced order based on a relative H-infinity tolerance.

Returns the smallest r such that the error bound for the truncated modes is
less than tol * total_norm (where total_norm = 2*sum(all HSVs)).

Parameters
----------
result : TruncationResult - from balanced_truncate (any r; all HSVs are returned).
tol    : float            - relative tolerance (default 0.01 = 1%).

Returns
-------
int : suggested order, clamped to [1, n-1].
)doc");

    // -----------------------------------------------------------------------
    // ZeroPhaseTrackingFilter
    // -----------------------------------------------------------------------
    py::class_<ctrl::ZPETCResult>(m, "ZPETCResult",
        "Result of ZPETC prefilter design.")
        .def_readonly("filter",            &ctrl::ZPETCResult::filter,
                      "Causal ZPETC prefilter G_ff(z) as a StateSpace.")
        .def_readonly("dc_amplitude_error",&ctrl::ZPETCResult::dcAmplitudeError,
                      "Max ||G*G_ff|(omega) - 1| over [0, Nyquist]. 0 for min-phase.")
        .def_readonly("has_nmp_zeros",     &ctrl::ZPETCResult::hasNMPZeros,
                      "True if any transmission zero satisfies |z| >= 1.")
        .def_readonly("zeros",             &ctrl::ZPETCResult::zeros,
                      "All transmission zeros of the plant (complex list).")
        .def_readonly("nmp_zeros",         &ctrl::ZPETCResult::nmpZeros,
                      "Non-minimum-phase zeros only (|z| >= 1).");

    m.def("transmission_zeros",
          [](const ctrl::StateSpace &sys) { return ctrl::transmissionZeros(sys); },
          py::arg("sys"),
          R"doc(
Compute the transmission zeros of a SISO discrete-time system.

Uses the generalised eigenvalue problem: det([[A-lambdaI, B],[C, D]]) = 0.
Returns finite eigenvalues only (infinite eigenvalues are excluded).

Parameters
----------
sys : SISO StateSpace.

Returns
-------
list of complex : transmission zeros.
)doc");

    m.def("design_zpetc",
          [](const ctrl::StateSpace &plant) { return ctrl::designZPETC(plant); },
          py::arg("plant"),
          R"doc(
Design a ZPETC prefilter for a SISO discrete-time plant.

The causal prefilter G_ff(z) satisfies:
  G(z) * G_ff(z) = B^-(z) / B^-(1)   (unit DC, zero phase from min-phase zeros)

For minimum-phase plants (all zeros inside UC): G*G_ff = z^{-d} (unit magnitude).
For NMP plants: G*G_ff has unit DC gain but amplitude error away from DC.

Parameters
----------
plant : SISO StateSpace.

Returns
-------
ZPETCResult with filter StateSpace and diagnostics.
)doc");

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
