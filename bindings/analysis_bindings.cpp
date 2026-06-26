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

    py::class_<ctrl::DiskMargin>(m, "DiskMargin",
        "Simultaneous gain/phase disk margin from SystemAnalysis::calculate_disk_margin().")
        .def_readonly("alpha",            &ctrl::DiskMargin::alpha,
                      "Disk radius alpha = 1 / ||S||_inf.")
        .def_readonly("gain_margin",      &ctrl::DiskMargin::gain_margin,
                      "Simultaneous gain margin (linear, NOT dB).")
        .def_readonly("phase_margin_deg", &ctrl::DiskMargin::phase_margin_deg,
                      "Simultaneous phase margin [degrees].");

    py::class_<ctrl::GangOfFour>(m, "GangOfFour",
        "Closed-loop S/T/GS/KS state-space matrices from SystemAnalysis::gang_of_four().")
        .def_readonly("S",  &ctrl::GangOfFour::S,  "Sensitivity (I + G*K)^-1.")
        .def_readonly("T",  &ctrl::GangOfFour::T,  "Complementary sensitivity (= I - S).")
        .def_readonly("GS", &ctrl::GangOfFour::GS, "Load-disturbance sensitivity S*G.")
        .def_readonly("KS", &ctrl::GangOfFour::KS, "Control/noise sensitivity K*S.");

    py::class_<ctrl::GangOfFourNorms>(m, "GangOfFourNorms",
        "Peak H-infinity norms of the Gang-of-Four matrices.")
        .def_readonly("norm_S",  &ctrl::GangOfFourNorms::norm_S)
        .def_readonly("norm_T",  &ctrl::GangOfFourNorms::norm_T)
        .def_readonly("norm_GS", &ctrl::GangOfFourNorms::norm_GS)
        .def_readonly("norm_KS", &ctrl::GangOfFourNorms::norm_KS);

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
        .def_static("get_singular_values",
                    &ctrl::SystemAnalysis::getSingularValues,
                    py::arg("sys"), py::arg("frequencies"),
                    "Singular values of G(e^{j*omega*Ts}) at each frequency (descending order).")
        .def_static("calculate_margins",
                    &ctrl::SystemAnalysis::calculateMargins,
                    py::arg("sys"),
                    "Compute gain and phase margins.  Returns StabilityMargins.")
        .def_static("calculate_h_infinity_norm",
                    &ctrl::SystemAnalysis::calculateHInfinityNorm,
                    py::arg("sys"),
                    "Grid approximation of the H-infinity norm (lower bound).")
        .def_static("series",
                    &ctrl::SystemAnalysis::series,
                    py::arg("g1"), py::arg("g2"),
                    "Cascade connection: output of g1 feeds the input of g2 (matrix product g2*g1).")
        .def_static("parallel",
                    &ctrl::SystemAnalysis::parallel,
                    py::arg("g1"), py::arg("g2"),
                    "Parallel connection: sys_out = g1 + g2 (same input feeds both).")
        .def_static("feedback",
                    &ctrl::SystemAnalysis::feedback,
                    py::arg("gk"),
                    "Close a unity-negative-feedback loop around forward-path gk. "
                    "Returns the complementary sensitivity T (reference -> output).")
        .def_static("gang_of_four",
                    &ctrl::SystemAnalysis::gangOfFour,
                    py::arg("g"), py::arg("k"),
                    "Build the Gang of Four (S, T, GS, KS) for plant g and controller k.")
        .def_static("gang_of_four_norms",
                    &ctrl::SystemAnalysis::gangOfFourNorms,
                    py::arg("g4"),
                    "Peak H-infinity norms of all four Gang-of-Four matrices.")
        .def_static("calculate_disk_margin",
                    &ctrl::SystemAnalysis::calculateDiskMargin,
                    py::arg("open_loop_l"),
                    "Simultaneous gain/phase disk margin of open-loop transfer open_loop_l.");

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

    // -----------------------------------------------------------------------
    // Gap metric + LPV identification + gain scheduling  (Part 20)
    // -----------------------------------------------------------------------

    // nuGap - two overloads
    m.def("nu_gap",
        py::overload_cast<const ctrl::StateSpace&, const ctrl::StateSpace&, int, double>(
            &ctrl::nuGap),
        py::arg("P1"), py::arg("P2"),
        py::arg("freq_points") = 200, py::arg("omega_min") = 1e-2,
        R"doc(
Nu-gap upper bound between two SISO discrete-time state-space systems.

Parameters
----------
P1, P2      : ctrl.StateSpace (both SISO, same Ts)
freq_points : frequency grid size (default 200)
omega_min   : lowest frequency [rad/s] (default 0.01)

Returns
-------
float in [0, 1] -- nu-gap upper bound (chordal metric, Vinnicombe 2001).
)doc");

    m.def("nu_gap_matrix",
        &ctrl::nuGapMatrix,
        py::arg("models"), py::arg("freq_points") = 200,
        R"doc(
Compute the symmetric N*N nu-gap distance matrix for a list of SISO models.

Parameters
----------
models      : list of ctrl.StateSpace (all SISO, same Ts)
freq_points : frequency resolution per pair (default 200)

Returns
-------
numpy.ndarray of shape (N, N), dtype float64
)doc");

    // ClusterResult
    py::class_<ctrl::ClusterResult>(m, "ClusterResult",
        "Result of gap-metric agglomerative clustering.")
        .def_readonly("labels",          &ctrl::ClusterResult::labels,
            "List[int]: cluster label (0..k-1) for each input model.")
        .def_readonly("representatives", &ctrl::ClusterResult::representatives,
            "List[int]: index in original list of each cluster's representative.")
        .def_readonly("max_intra_gap",   &ctrl::ClusterResult::maxIntraGap,
            "List[float]: maximum nu-gap within each cluster.")
        .def_readonly("num_clusters",    &ctrl::ClusterResult::numClusters,
            "int: total number of clusters.")
        .def_readonly("threshold",       &ctrl::ClusterResult::threshold,
            "float: threshold used for clustering.");

    m.def("cluster_by_gap",
        py::overload_cast<const Eigen::MatrixXd&, double>(&ctrl::clusterByGap),
        py::arg("gap_matrix"), py::arg("threshold") = 0.5,
        R"doc(
Cluster N models using a pre-computed nu-gap distance matrix.

Parameters
----------
gap_matrix : numpy.ndarray (N, N) symmetric distance matrix.
threshold  : maximum intra-cluster gap (default 0.5).

Returns
-------
ClusterResult
)doc");

    m.def("suggest_gap_threshold",
        &ctrl::suggestGapThreshold,
        py::arg("gap_matrix"),
        py::arg("lo") = 0.1, py::arg("hi") = 0.9, py::arg("steps") = 20,
        "Suggest a clustering threshold by finding the knee of the cluster-count curve.");

    // LPVModel
    py::class_<ctrl::LPVModel>(m, "LPVModel", R"doc(
Identified LPV model with polynomial parameter dependence.

Attributes
----------
degree, n_states, n_inputs, n_outputs, Ts : model dimensions.
A_coeffs, B_coeffs, C_coeffs, D_coeffs   : list of coefficient matrices.

Methods
-------
eval_a(p), eval_b(p), eval_c(p), eval_d(p) : evaluate at scheduling param p.
frozen(p)                                    : return frozen StateSpace at p.
)doc")
        .def(py::init<>(), "Default-construct an empty LPVModel (populate manually or use identify_lpv).")
        .def_readonly("degree",    &ctrl::LPVModel::degree)
        .def_readonly("n_states",  &ctrl::LPVModel::n_states)
        .def_readonly("n_inputs",  &ctrl::LPVModel::n_inputs)
        .def_readonly("n_outputs", &ctrl::LPVModel::n_outputs)
        .def_readonly("Ts",        &ctrl::LPVModel::Ts)
        .def_readonly("A_coeffs",  &ctrl::LPVModel::A_coeffs)
        .def_readonly("B_coeffs",  &ctrl::LPVModel::B_coeffs)
        .def_readonly("C_coeffs",  &ctrl::LPVModel::C_coeffs)
        .def_readonly("D_coeffs",  &ctrl::LPVModel::D_coeffs)
        .def("eval_a",  &ctrl::LPVModel::evalA,  py::arg("p"),
             "Evaluate A(p) at scheduling parameter p.")
        .def("eval_b",  &ctrl::LPVModel::evalB,  py::arg("p"))
        .def("eval_c",  &ctrl::LPVModel::evalC,  py::arg("p"))
        .def("eval_d",  &ctrl::LPVModel::evalD,  py::arg("p"))
        .def("frozen",  &ctrl::LPVModel::frozen, py::arg("p"),
             "Return the frozen (LTI) StateSpace at scheduling parameter p.");

    m.def("identify_lpv",
        &ctrl::identifyLPV,
        py::arg("X"), py::arg("U"), py::arg("Y"), py::arg("sched"),
        py::arg("degree") = 1, py::arg("Ts") = 0.01,
        R"doc(
Identify an LPV model from state, input, output, and scheduling data.

Parameters
----------
X      : numpy.ndarray (n, N) - state sequence.
U      : numpy.ndarray (m, N) - input sequence.
Y      : numpy.ndarray (p, N) - output sequence.
sched  : List[float] of length >= N - scheduling parameter values.
degree : polynomial degree (default 1 = affine).
Ts     : sample time [s].

Returns
-------
LPVModel
)doc");

    m.def("identify_lpv_from_io",
        &ctrl::identifyLPVFromIO,
        py::arg("U"), py::arg("Y"), py::arg("sched"),
        py::arg("n_states"), py::arg("degree") = 1,
        py::arg("Ts") = 0.01, py::arg("n4sid_i") = 10,
        R"doc(
Identify an LPV model from I/O data only (state sequence estimated via n4sid).

Parameters
----------
U       : numpy.ndarray (m, N)
Y       : numpy.ndarray (p_out, N)
sched   : List[float]
n_states: desired model order
degree  : polynomial degree (default 1)
Ts      : sample time
n4sid_i : N4SID block-rows parameter (default 10)

Returns
-------
LPVModel
)doc");

    // GainScheduleMode enum
    py::enum_<ctrl::GainScheduleMode>(m, "GainScheduleMode")
        .value("NearestNeighbor", ctrl::GainScheduleMode::NearestNeighbor,
               "Hard-switch: only closest schedule point is evaluated.")
        .value("LinearBlend",     ctrl::GainScheduleMode::LinearBlend,
               "Weighted blend: outputs of two adjacent controllers are combined.")
        .export_values();

    // GainScheduledController
    py::class_<ctrl::GainScheduledController, ctrl::IController,
               std::shared_ptr<ctrl::GainScheduledController>>(
        m, "GainScheduledController", R"doc(
Gain-scheduled IController: blends or switches between a set of controllers
parameterised by a scalar scheduling variable p.

Usage
-----
sched = ctrl.GainScheduledController(Ts)
sched.add_schedule_point(0.0, lqr_0)
sched.add_schedule_point(1.0, lqr_1)

# In the control loop:
sched.set_scheduling_param(p_measured)
u = sched.compute(error)
)doc")
        .def(py::init<double, ctrl::GainScheduleMode>(),
             py::arg("Ts"),
             py::arg("mode") = ctrl::GainScheduleMode::LinearBlend)
        .def("add_schedule_point", &ctrl::GainScheduledController::addSchedulePoint,
             py::arg("p"), py::arg("ctrl"),
             "Add a controller at scheduling parameter value p.")
        .def("set_scheduling_param", &ctrl::GainScheduledController::setSchedulingParam,
             py::arg("p"), "Update the scheduling variable before compute().")
        .def("scheduling_param", &ctrl::GainScheduledController::schedulingParam)
        .def_property_readonly("num_points", &ctrl::GainScheduledController::numPoints)
        .def("point_p",          &ctrl::GainScheduledController::pointP, py::arg("i"))
        .def("compute",          &ctrl::GainScheduledController::compute, py::arg("error"))
        .def("reset",            &ctrl::GainScheduledController::reset)
        .def("sample_time",      &ctrl::GainScheduledController::sampleTime)
        .def("last_output",      &ctrl::GainScheduledController::lastOutput);

    // -----------------------------------------------------------------------
    // VectorFitting (T3)
    // -----------------------------------------------------------------------
    py::class_<ctrl::VectorFittingParams>(m, "VectorFittingParams",
        "Tuning parameters for VectorFitting::fit_magnitude().")
        .def(py::init<>())
        .def_readwrite("max_iter", &ctrl::VectorFittingParams::max_iter,
                       "SK iteration limit.")
        .def_readwrite("tol",      &ctrl::VectorFittingParams::tol,
                       "Stop when max pole displacement < tol.")
        .def_readwrite("eps_mag",  &ctrl::VectorFittingParams::eps_mag,
                       "Regularisation floor for near-zero magnitudes.");

    py::class_<ctrl::VectorFittingResult>(m, "VectorFittingResult",
        "Result of VectorFitting::fit_magnitude().")
        .def_readonly("converged",   &ctrl::VectorFittingResult::converged)
        .def_readonly("iterations",  &ctrl::VectorFittingResult::iterations)
        .def_readonly("rms_error",   &ctrl::VectorFittingResult::rms_error);

    py::class_<ctrl::VectorFitting>(m, "VectorFitting",
        "Rational approximation of frequency-domain magnitude data (Sanathanan-Koerner).")
        .def_static("fit_magnitude",
            [](const std::vector<double>& omega,
               const std::vector<double>& magnitude,
               int n_poles, double Ts,
               const ctrl::VectorFittingParams& params) {
                ctrl::VectorFittingResult res;
                ctrl::StateSpace ss = ctrl::VectorFitting::fitMagnitude(
                    omega, magnitude, n_poles, Ts, res, params);
                return std::make_pair(res, ss);
            },
            py::arg("omega_grid"), py::arg("magnitude"),
            py::arg("n_poles"), py::arg("Ts"),
            py::arg("params") = ctrl::VectorFittingParams{},
            "Fit a stable rational filter to the magnitude profile. Returns (VectorFittingResult, StateSpace).")
        .def_static("eval_magnitude",
            &ctrl::VectorFitting::evalMagnitude,
            py::arg("sys"), py::arg("omega"),
            "Evaluate |H(e^{j*omega*Ts})| for a StateSpace at a given frequency.");

    // -----------------------------------------------------------------------
    // RobustnessAnalysis - Monte-Carlo closed-loop robustness (Phase 1)
    // -----------------------------------------------------------------------
    py::class_<ctrl::PerturbationSpec> perturb_spec(m, "PerturbationSpec",
        "Per-parameter uncertainty description (distribution, relative sigma, bounds).");
    py::enum_<ctrl::PerturbationSpec::Distribution>(perturb_spec, "Distribution")
        .value("Uniform", ctrl::PerturbationSpec::Distribution::Uniform)
        .value("Normal",  ctrl::PerturbationSpec::Distribution::Normal)
        .export_values();
    perturb_spec
        .def(py::init<>())
        .def_readwrite("dist",        &ctrl::PerturbationSpec::dist)
        .def_readwrite("sigma",       &ctrl::PerturbationSpec::sigma)
        .def_readwrite("lower_bound", &ctrl::PerturbationSpec::lower_bound)
        .def_readwrite("upper_bound", &ctrl::PerturbationSpec::upper_bound);

    py::class_<ctrl::RobustnessSample>(m, "RobustnessSample",
        "Closed-loop robustness metrics from one perturbed plant sample.")
        .def_readonly("sample_id",          &ctrl::RobustnessSample::sample_id)
        .def_readonly("is_stable",          &ctrl::RobustnessSample::is_stable)
        .def_readonly("gain_margin_db",     &ctrl::RobustnessSample::gain_margin_db)
        .def_readonly("phase_margin_deg",   &ctrl::RobustnessSample::phase_margin_deg)
        .def_readonly("hinf_sensitivity",   &ctrl::RobustnessSample::hinf_sensitivity)
        .def_readonly("hinf_comp_sens",     &ctrl::RobustnessSample::hinf_comp_sens)
        .def_readonly("iae",                &ctrl::RobustnessSample::iae)
        .def_readonly("settling_time_s",    &ctrl::RobustnessSample::settling_time_s)
        .def_readonly("overshoot_pct",      &ctrl::RobustnessSample::overshoot_pct)
        .def_readonly("nu_gap_from_nominal",&ctrl::RobustnessSample::nu_gap_from_nominal);

    py::class_<ctrl::MonteCarloResult::MetricStats>(m, "MetricStats",
        "Summary statistics for one robustness metric (finite values only).")
        .def_readonly("mean",    &ctrl::MonteCarloResult::MetricStats::mean)
        .def_readonly("std_dev", &ctrl::MonteCarloResult::MetricStats::std_dev)
        .def_readonly("p5",      &ctrl::MonteCarloResult::MetricStats::p5)
        .def_readonly("p25",     &ctrl::MonteCarloResult::MetricStats::p25)
        .def_readonly("p50",     &ctrl::MonteCarloResult::MetricStats::p50)
        .def_readonly("p75",     &ctrl::MonteCarloResult::MetricStats::p75)
        .def_readonly("p95",     &ctrl::MonteCarloResult::MetricStats::p95)
        .def_readonly("worst",   &ctrl::MonteCarloResult::MetricStats::worst);

    py::class_<ctrl::MonteCarloResult>(m, "MonteCarloResult",
        "Aggregated robustness statistics over a perturbed-plant ensemble.")
        .def_readonly("n_samples",                   &ctrl::MonteCarloResult::n_samples)
        .def_readonly("n_unstable",                  &ctrl::MonteCarloResult::n_unstable)
        .def_readonly("instability_probability",     &ctrl::MonteCarloResult::instability_probability)
        .def_readonly("gm_stats",                    &ctrl::MonteCarloResult::gm_stats)
        .def_readonly("pm_stats",                    &ctrl::MonteCarloResult::pm_stats)
        .def_readonly("sensitivity_peak_stats",      &ctrl::MonteCarloResult::sensitivity_peak_stats)
        .def_readonly("comp_sensitivity_peak_stats", &ctrl::MonteCarloResult::comp_sensitivity_peak_stats)
        .def_readonly("iae_stats",                   &ctrl::MonteCarloResult::iae_stats)
        .def_readonly("settling_stats",              &ctrl::MonteCarloResult::settling_stats)
        .def_readonly("nu_gap_stats",                &ctrl::MonteCarloResult::nu_gap_stats)
        .def_readonly("samples",                     &ctrl::MonteCarloResult::samples);

    m.def("spawn_ss_samples", &ctrl::spawn_SS_samples,
          py::arg("nominal"), py::arg("num_samples"), py::arg("sigma_A"),
          py::arg("sigma_B") = 0.0, py::arg("sigma_C") = 0.0, py::arg("sigma_D") = 0.0,
          py::arg("seed") = 42,
          R"doc(
Spawn an ensemble of perturbed state-space models.

Each entry of A, B, C, D is scaled by (1 + sigma_M * N(0,1)) independently.
sigma_B = sigma_C = sigma_D = 0 perturbs only A (state-matrix uncertainty).

Returns
-------
list of ctrl.StateSpace
)doc");

    m.def("spawn_tf_samples", &ctrl::spawn_TF_samples,
          py::arg("nominal"), py::arg("num_samples"),
          py::arg("numerator_sigma"), py::arg("denominator_sigma"),
          py::arg("seed") = 42,
          R"doc(
Spawn an ensemble of perturbed transfer functions.

numerator_sigma / denominator_sigma are per-coefficient relative std-devs
(length 1 = uniform). den[0] is held at 1 to keep the denominator monic.

Returns
-------
list of ctrl.TransferFunction
)doc");

    m.def("evaluate_sample", &ctrl::evaluateSample,
          py::arg("sample_id"), py::arg("plant"), py::arg("controller_ss"),
          py::arg("nominal_plant"), py::arg("step_amplitude") = 1.0,
          py::arg("sim_duration_s") = 50.0, py::arg("settling_band") = 0.05,
          "Close the loop on one (plant, controller) pair and measure robustness. Returns RobustnessSample.");

    m.def("run_monte_carlo", &ctrl::runMonteCarlo,
          py::arg("ensemble"), py::arg("controller_ss"), py::arg("nominal_plant"),
          py::arg("step_amplitude") = 1.0, py::arg("sim_duration_s") = 50.0,
          "Evaluate an ensemble of plants against a fixed controller. Returns MonteCarloResult.");

    m.def("monte_carlo_analysis", &ctrl::monteCarloAnalysis,
          py::arg("nominal_plant"), py::arg("controller_ss"), py::arg("num_samples"),
          py::arg("sigma_A"), py::arg("sigma_B") = 0.0, py::arg("sigma_C") = 0.0,
          py::arg("sigma_D") = 0.0, py::arg("seed") = 42,
          R"doc(
Spawn a perturbed ensemble and run the full Monte-Carlo robustness analysis.

The nominal plant is used as both the spawn centre and the nu-gap reference.

Returns
-------
ctrl.MonteCarloResult
)doc");

    // -----------------------------------------------------------------------
    // MuAnalysis - Structured Singular Value (Robustness Phase 3)
    // -----------------------------------------------------------------------
    py::class_<ctrl::UncertaintyBlock> uncertainty_block(m, "UncertaintyBlock",
        "One block of a block-diagonal structured uncertainty Delta = blkdiag(blocks).");
    py::enum_<ctrl::UncertaintyBlock::Type>(uncertainty_block, "Type")
        .value("RealScalar",    ctrl::UncertaintyBlock::Type::RealScalar)
        .value("ComplexScalar", ctrl::UncertaintyBlock::Type::ComplexScalar)
        .value("ComplexFull",   ctrl::UncertaintyBlock::Type::ComplexFull)
        .export_values();
    uncertainty_block
        .def(py::init<>())
        .def_readwrite("type",  &ctrl::UncertaintyBlock::type)
        .def_readwrite("r_out", &ctrl::UncertaintyBlock::r_out)
        .def_readwrite("r_in",  &ctrl::UncertaintyBlock::r_in);

    py::class_<ctrl::UncertaintyStructure>(m, "UncertaintyStructure",
        "Full block-diagonal uncertainty structure Delta = blkdiag(blocks). "
        "M passed to compute_mu/peak_mu must be square with rows == total_outputs() "
        "and cols == total_inputs().")
        .def(py::init<>())
        .def_readwrite("blocks", &ctrl::UncertaintyStructure::blocks)
        .def("total_inputs",  &ctrl::UncertaintyStructure::totalInputs)
        .def("total_outputs", &ctrl::UncertaintyStructure::totalOutputs);

    py::class_<ctrl::MuBound>(m, "MuBound",
        "Upper (D-scaling) and optional lower (spectral-radius) bound on mu at one frequency.")
        .def_readonly("upper", &ctrl::MuBound::upper)
        .def_readonly("lower", &ctrl::MuBound::lower);

    py::class_<ctrl::PeakMuResult>(m, "PeakMuResult",
        "Peak structured singular value over a frequency grid, plus the full curve.")
        .def_readonly("peak",             &ctrl::PeakMuResult::peak)
        .def_readonly("peak_omega_rad_s", &ctrl::PeakMuResult::peak_omega_rad_s)
        .def_readonly("mu_curve",         &ctrl::PeakMuResult::mu_curve);

    m.def("compute_mu", &ctrl::computeMu,
          py::arg("m_freq"), py::arg("struc"), py::arg("compute_lower_bound") = false,
          R"doc(
Compute the D-scaling (and, for RealScalar blocks, G-scaling) upper bound (and optional
spectral-radius lower bound) on the structured singular value mu_Delta(M) at each supplied
frequency-response matrix.

ComplexFull blocks are scaled exactly (textbook-tight); ComplexScalar and RealScalar blocks
use a valid-but-possibly-loose single-scalar approximation (exact only for block size 1).

Returns
-------
list of ctrl.MuBound
)doc");

    m.def("peak_mu", &ctrl::peakMu,
          py::arg("G"), py::arg("K"), py::arg("struc"),
          py::arg("sigma_rel") = 0.1, py::arg("freq_points") = 200,
          py::arg("omega_min") = 1e-2,
          R"doc(
Peak structured singular value of a unity-feedback loop (G, K) under scaled output
multiplicative uncertainty M(jw) = sigma_rel * T(jw), where T is the complementary
sensitivity (see SystemAnalysis.gang_of_four). The loop is robust to this uncertainty
class iff the returned peak.upper < 1.

Returns
-------
ctrl.PeakMuResult
)doc");

    m.def("robust_stability_radius", &ctrl::robustStabilityRadius,
          py::arg("G"), py::arg("K"), py::arg("struc"),
          py::arg("sigma_max") = 2.0, py::arg("bisect_iters") = 30,
          "Largest sigma_rel for which peak_mu(...).peak.upper stays below 1 "
          "(bisection search; returns sigma_max if even that bound is robust).");

    // -----------------------------------------------------------------------
    // LFTSystem - general multi-block LFT/Delta channel-gather (Phase 3 RC1)
    // -----------------------------------------------------------------------
    py::class_<ctrl::LFTChannelMap>(m, "LFTChannelMap",
        "Per-block row/column placement of each UncertaintyStructure block within M0's "
        "own frequency-response matrix.")
        .def(py::init<>())
        .def_readwrite("row_start", &ctrl::LFTChannelMap::rowStart,
                       "Per-block output-row offset into M0 (size == struc.blocks size).")
        .def_readwrite("col_start", &ctrl::LFTChannelMap::colStart,
                       "Per-block input-column offset into M0 (size == struc.blocks size).");

    py::class_<ctrl::LFTSystem>(m, "LFTSystem", R"doc(
General multi-block LFT/Delta channel-gather for mu-analysis.

Generalizes peak_mu()'s hardcoded single canonical "M = sigma_rel * T" loop into
arbitrary placement of one or more Delta blocks against an open-loop map's own channels.

Example
-------
>>> chmap = ctrl.LFTChannelMap()
>>> chmap.row_start = [0, 1]; chmap.col_start = [0, 1]
>>> lft = ctrl.LFTSystem(M0, struc, chmap)
>>> result = lft.peak_mu()
)doc")
        .def(py::init<const ctrl::StateSpace &, const ctrl::UncertaintyStructure &,
                      const ctrl::LFTChannelMap &>(),
             py::arg("M0"), py::arg("struc"), py::arg("map"))
        .def("closed_loop_freq_response", &ctrl::LFTSystem::closedLoopFreqResponse,
             py::arg("omegas"),
             "Gathered M(jw) at each frequency, block-ordered to match the uncertainty "
             "structure - directly feedable to compute_mu().")
        .def("peak_mu", &ctrl::LFTSystem::peakMu,
             py::arg("freq_points") = 200, py::arg("omega_min") = 1e-2,
             "Peak structured singular value over a log-spaced frequency grid.");

    // -----------------------------------------------------------------------
    // WorstCaseSearch - CMA-ES worst-case parameter search (Robustness Phase 4)
    // -----------------------------------------------------------------------
    py::class_<ctrl::WorstCaseSearchParams>(m, "WorstCaseSearchParams",
        "Parameters for the CMA-ES worst-case parameter search.")
        .def(py::init<>())
        .def_readwrite("max_evals",  &ctrl::WorstCaseSearchParams::max_evals,
                       "Approximate cost-function evaluation budget.")
        .def_readwrite("sigma_init", &ctrl::WorstCaseSearchParams::sigma_init,
                       "Initial CMA-ES step size, in relative (search_width) units.")
        .def_readwrite("population", &ctrl::WorstCaseSearchParams::population,
                       "Reserved; AutoTuner does not currently expose a population override.")
        .def_readwrite("seed",       &ctrl::WorstCaseSearchParams::seed);

    py::class_<ctrl::WorstCaseResult>(m, "WorstCaseResult",
        "Result of a findWorstCase* search.")
        .def_readonly("worst_params", &ctrl::WorstCaseResult::worst_params)
        .def_readonly("worst_cost",   &ctrl::WorstCaseResult::worst_cost,
                      "Metric value at worst_params (the actual metric, not negated).")
        .def_readonly("converged",    &ctrl::WorstCaseResult::converged)
        .def_readonly("n_evals",      &ctrl::WorstCaseResult::n_evals);

    m.def("find_worst_case_sensitivity",
          [](py::object plant_factory_py,
             const ctrl::StateSpace& controller_ss,
             const Eigen::VectorXd& param_nominal,
             const Eigen::VectorXd& param_sigma,
             const Eigen::VectorXd& lower_bounds,
             const Eigen::VectorXd& upper_bounds,
             const ctrl::WorstCaseSearchParams& p) {
              auto plant_factory = [plant_factory_py](const Eigen::VectorXd& params) -> ctrl::StateSpace {
                  return plant_factory_py(params).cast<ctrl::StateSpace>();
              };
              return ctrl::findWorstCaseSensitivity(plant_factory, controller_ss, param_nominal,
                                                     param_sigma, lower_bounds, upper_bounds, p);
          },
          py::arg("plant_factory"), py::arg("controller_ss"),
          py::arg("param_nominal"), py::arg("param_sigma"),
          py::arg("lower_bounds") = Eigen::VectorXd(),
          py::arg("upper_bounds") = Eigen::VectorXd(),
          py::arg("params") = ctrl::WorstCaseSearchParams{},
          R"doc(
Find the uncertain plant (plant_factory(params) -> StateSpace) that maximises the
closed-loop peak sensitivity ||S||_inf against the fixed controller_ss.

Returns
-------
ctrl.WorstCaseResult
)doc");

    m.def("find_worst_case_iae",
          [](py::object plant_factory_py,
             const ctrl::StateSpace& controller_ss,
             const Eigen::VectorXd& param_nominal,
             const Eigen::VectorXd& param_sigma,
             const Eigen::VectorXd& lower_bounds,
             const Eigen::VectorXd& upper_bounds,
             double sim_duration_s,
             const ctrl::WorstCaseSearchParams& p) {
              auto plant_factory = [plant_factory_py](const Eigen::VectorXd& params) -> ctrl::StateSpace {
                  return plant_factory_py(params).cast<ctrl::StateSpace>();
              };
              return ctrl::findWorstCaseIAE(plant_factory, controller_ss, param_nominal,
                                            param_sigma, lower_bounds, upper_bounds,
                                            sim_duration_s, p);
          },
          py::arg("plant_factory"), py::arg("controller_ss"),
          py::arg("param_nominal"), py::arg("param_sigma"),
          py::arg("lower_bounds") = Eigen::VectorXd(),
          py::arg("upper_bounds") = Eigen::VectorXd(),
          py::arg("sim_duration_s") = 50.0,
          py::arg("params") = ctrl::WorstCaseSearchParams{},
          R"doc(
Find the uncertain plant that maximises closed-loop step-response IAE against the
fixed controller_ss. An unstable sample reports iae = +inf, which the search
correctly treats as the worst possible outcome.

Returns
-------
ctrl.WorstCaseResult
)doc");

    m.def("find_worst_case",
          [](py::object plant_factory_py,
             py::object metric_fn_py,
             const Eigen::VectorXd& param_nominal,
             const Eigen::VectorXd& param_sigma,
             const Eigen::VectorXd& lower_bounds,
             const Eigen::VectorXd& upper_bounds,
             const ctrl::WorstCaseSearchParams& p) {
              auto plant_factory = [plant_factory_py](const Eigen::VectorXd& params) -> ctrl::StateSpace {
                  return plant_factory_py(params).cast<ctrl::StateSpace>();
              };
              auto metric_fn = [metric_fn_py](const ctrl::StateSpace& ss) -> double {
                  return metric_fn_py(ss).cast<double>();
              };
              return ctrl::findWorstCase(plant_factory, metric_fn, param_nominal,
                                         param_sigma, lower_bounds, upper_bounds, p);
          },
          py::arg("plant_factory"), py::arg("metric_fn"),
          py::arg("param_nominal"), py::arg("param_sigma"),
          py::arg("lower_bounds") = Eigen::VectorXd(),
          py::arg("upper_bounds") = Eigen::VectorXd(),
          py::arg("params") = ctrl::WorstCaseSearchParams{},
          R"doc(
Generic worst-case search: maximise metric_fn(plant_factory(params)) over the relative
search box around param_nominal. metric_fn is free to close its own loop via capture.

Returns
-------
ctrl.WorstCaseResult
)doc");

    // -----------------------------------------------------------------------
    // LyapunovRobustness - common quadratic Lyapunov function (Robustness Phase 5)
    // -----------------------------------------------------------------------
    py::class_<ctrl::LyapunovSearchParams>(m, "LyapunovSearchParams",
        "Parameters for the common-Lyapunov search.")
        .def(py::init<>())
        .def_readwrite("max_iter",   &ctrl::LyapunovSearchParams::max_iter)
        .def_readwrite("tol",        &ctrl::LyapunovSearchParams::tol)
        .def_readwrite("regularise", &ctrl::LyapunovSearchParams::regularise);

    py::class_<ctrl::LyapunovResult>(m, "LyapunovResult",
        "Result of findCommonLyapunov().")
        .def_readonly("found",      &ctrl::LyapunovResult::found)
        .def_readonly("P",          &ctrl::LyapunovResult::P)
        .def_readonly("residual",   &ctrl::LyapunovResult::residual,
                      "Worst-case (max over vertices) largest eigenvalue of A_i^T P A_i - P + Q; "
                      "negative means satisfied with margin.")
        .def_readonly("iterations", &ctrl::LyapunovResult::iterations);

    m.def("find_common_lyapunov", &ctrl::findCommonLyapunov,
          py::arg("vertices"), py::arg("Q") = Eigen::MatrixXd(),
          py::arg("params") = ctrl::LyapunovSearchParams{},
          R"doc(
Search for a common quadratic Lyapunov function V(x) = x'Px over a polytope of vertex
state matrices (vertices = conv hull of A_1..A_L). Q defaults to the identity.

Returns
-------
ctrl.LyapunovResult
)doc");

    m.def("is_quadratically_stable", &ctrl::isQuadraticallyStable,
          py::arg("vertices"), py::arg("params") = ctrl::LyapunovSearchParams{},
          "True iff every vertex is individually Schur-stable AND a common Lyapunov "
          "function is found across the whole polytope.");

    m.def("build_box_vertices", &ctrl::buildBoxVertices,
          py::arg("A_nominal"), py::arg("uncertainty_cols"),
          R"doc(
Build the 2^m vertex matrices of a box uncertainty around A_nominal. uncertainty_cols
is n*n x m; column j is a flattened n x n perturbation direction delta_j. Returns
{A_nominal + sum_j (+-1)*delta_j} for every sign combination (2^m matrices).

Returns
-------
list of numpy.ndarray
)doc");
}
