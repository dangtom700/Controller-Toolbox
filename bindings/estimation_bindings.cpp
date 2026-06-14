#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "ControllerToolbox.h"
#include "GreyBoxEstimator.h"
#include "RecursiveGreyBoxEstimator.h"

namespace py = pybind11;

void bind_estimation(py::module_ &m)
{
    // -----------------------------------------------------------------------
    // KalmanFilter
    // -----------------------------------------------------------------------
    py::class_<ctrl::KalmanFilter>(m, "KalmanFilter", R"doc(
Linear discrete Kalman filter (predict / update) with Joseph-form covariance update.

Example
-------
>>> kf = ctrl.KalmanFilter(sys, Q, R)
>>> for k in range(N):
...     kf.predict(u_prev)
...     kf.update(y, u_current)
...     x_hat = kf.state()
...
... # Or equivalently, one-call:
...     kf.step(y, u_prev)            # D=0 plants
...     kf.step(y, u_prev, u_current) # D!=0 plants
)doc")
        .def(py::init<const ctrl::StateSpace &,
                      const Eigen::MatrixXd &,
                      const Eigen::MatrixXd &>(),
             py::arg("plant"), py::arg("Q_noise"), py::arg("R_noise"),
             "Construct with identity initial covariance P0 = I.")
        .def(py::init<const ctrl::StateSpace &,
                      const Eigen::MatrixXd &,
                      const Eigen::MatrixXd &,
                      const Eigen::MatrixXd &>(),
             py::arg("plant"), py::arg("Q_noise"), py::arg("R_noise"), py::arg("P0"),
             "Construct with explicit initial covariance P0.")
        .def("predict",     &ctrl::KalmanFilter::predict,  py::arg("u"),
             "Prediction step: advance x^ and inflate P with process noise Q.")
        .def("update",      &ctrl::KalmanFilter::update,
             py::arg("y"), py::arg("u_current"),
             "Update step: incorporate measurement y[k] using D*u_current feedthrough.")
        .def("step",
             // Plain-reference overload (three-argument form) - preferred from Python.
             py::overload_cast<const Eigen::VectorXd &,
                               const Eigen::VectorXd &,
                               const Eigen::VectorXd &>(&ctrl::KalmanFilter::step),
             py::arg("y"), py::arg("u_prev"), py::arg("u_current"),
             "Combined predict+update with explicit u_current (for D != 0 plants).")
        .def("step",
             // Two-argument form: u_current defaults to u_prev (correct for D=0).
             [](ctrl::KalmanFilter &kf,
                const Eigen::VectorXd &y,
                const Eigen::VectorXd &u_prev) {
                 kf.step(y, u_prev);
             },
             py::arg("y"), py::arg("u_prev"),
             "Combined predict+update.  u_current = u_prev (correct for D = 0 plants).")
        .def("reset",       &ctrl::KalmanFilter::reset,
             "Reset state estimate to zero and covariance to P0.")
        .def("state",       &ctrl::KalmanFilter::state,
             py::return_value_policy::copy,
             "Current state estimate x^[k|k] as a NumPy array (n,).")
        .def("covariance",  &ctrl::KalmanFilter::covariance,
             py::return_value_policy::copy,
             "Current error covariance P[k|k] as a NumPy array (n, n).")
        .def("sample_time", &ctrl::KalmanFilter::sampleTime,
             "Sample time Ts [s].")
        // D1: mismatch detection
        .def("enable_mismatch_detection",
             [](ctrl::KalmanFilter& kf,
                double sigma, double k_cusum, double h_threshold) {
                 ctrl::MismatchDetectorParams p;
                 p.sigma = sigma; p.k_cusum = k_cusum; p.h_threshold = h_threshold;
                 kf.enableMismatchDetection(p);
             },
             py::arg("sigma") = 1.0, py::arg("k_cusum") = 0.5, py::arg("h_threshold") = 5.0,
             "Enable CUSUM mismatch detection on the KF innovation sequence (D1). "
             "sigma: expected in-control innovation RMS; k/h_threshold: CUSUM parameters.")
        .def("mismatch_detected",        &ctrl::KalmanFilter::mismatchDetected,
             "True if CUSUM has exceeded h_threshold*sigma since last reset.")
        .def("mismatch_score",           &ctrl::KalmanFilter::mismatchScore,
             "Current CUSUM statistic max(C+, C-).")
        .def("reset_mismatch_detector",  &ctrl::KalmanFilter::resetMismatchDetector,
             "Reset CUSUM accumulators without disabling detection.");

    // -----------------------------------------------------------------------
    // RecursiveLeastSquares
    // -----------------------------------------------------------------------
    py::class_<ctrl::RecursiveLeastSquares>(m, "RecursiveLeastSquares", R"doc(
Online ARX system identification with exponential forgetting.

Identifies  A(z^-1) y[k] = B(z^-1) u[k]  in real time.

Example
-------
>>> rls = ctrl.RecursiveLeastSquares(na=2, nb=1, lambda_f=0.98, Ts=0.01)
>>> for k in range(N):
...     rls.update(y[k], u[k])
>>> tf = rls.to_transfer_function()
)doc")
        .def(py::init<int, int, double, double>(),
             py::arg("na"), py::arg("nb"),
             py::arg("lambda_forgetting") = 1.0,
             py::arg("Ts") = 1.0,
             "na: denominator order, nb: numerator order (B has nb+1 terms).")
        .def("update",               &ctrl::RecursiveLeastSquares::update,
             py::arg("y"), py::arg("u"),
             "Incorporate one new (y[k], u[k]) sample.")
        .def("reset",                &ctrl::RecursiveLeastSquares::reset,
             "Reset parameter estimate and covariance to initial values.")
        .def("params",               &ctrl::RecursiveLeastSquares::params,
             py::return_value_policy::copy,
             "Current parameter vector theta as a NumPy array.")
        .def("covariance",           &ctrl::RecursiveLeastSquares::covariance,
             py::return_value_policy::copy,
             "Current parameter covariance matrix P.")
        .def("denominator",          &ctrl::RecursiveLeastSquares::denominator,
             py::return_value_policy::copy,
             "Monic denominator coefficients [1, a1, ..., ana].")
        .def("numerator",            &ctrl::RecursiveLeastSquares::numerator,
             py::return_value_policy::copy,
             "Numerator coefficients [b0, b1, ..., bnb].")
        .def("to_transfer_function", &ctrl::RecursiveLeastSquares::toTransferFunction,
             "Return identified model as a TransferFunction.")
        .def("to_state_space",       &ctrl::RecursiveLeastSquares::toStateSpace,
             "Return identified model as a StateSpace.")
        .def("sample_count",         &ctrl::RecursiveLeastSquares::sampleCount,
             "Number of samples processed since construction or last reset.");

    // -----------------------------------------------------------------------
    // ExtendedKalmanFilter  (optional - CTRL_HAS_ADVANCED_KALMAN)
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_ADVANCED_KALMAN)
    py::class_<ctrl::ExtendedKalmanFilter>(m, "ExtendedKalmanFilter", R"doc(
Extended Kalman filter for nonlinear state estimation.

Linearises the dynamics at each step using analytical or numerical Jacobians.
Process model:  x[k+1] = f(x[k], u[k]) + w,   w ~ N(0, Q)
Measurement:    y[k]   = h(x[k], u[k]) + v,   v ~ N(0, R)

f, h, F_jac, H_jac are Python callables with signature (x: np.ndarray, u: np.ndarray) -> np.ndarray.
Use numerical_jacobian() to generate Jacobians automatically when analytical forms are unavailable.

Example
-------
>>> def f(x, u): return A @ x + B @ u
>>> def h(x, u): return C @ x
>>> def Fj(x, u): return A   # analytical Jacobian df/dx
>>> def Hj(x, u): return C   # analytical Jacobian dh/dx
>>>
>>> ekf = ctrl.ExtendedKalmanFilter(n=2, p=1, f=f, h=h,
...         F_jac=Fj, H_jac=Hj, Q=Q, R=R, Ts=0.01)
>>> for k in range(N):
...     ekf.step(y[k], u[k-1])
...     x_hat = ekf.state()

Numerical Jacobian example
--------------------------
>>> Fj_num = lambda x, u: ctrl.ExtendedKalmanFilter.numerical_jacobian(
...     lambda xx: f(xx, u), x)
)doc")
        .def(py::init(
             [](int n, int p,
                py::object f_obj,  py::object h_obj,
                py::object Fj_obj, py::object Hj_obj,
                const Eigen::MatrixXd &Q,
                const Eigen::MatrixXd &R,
                double Ts,
                py::object P0_obj) -> ctrl::ExtendedKalmanFilter* {
                 // Capture py::object (not py::cpp_function) to avoid overload-deduction
                 // errors in pybind11's function_signature_t machinery.
                 ctrl::StateFunc  f  = [f_obj] (const Eigen::VectorXd &x, const Eigen::VectorXd &u)
                     -> Eigen::VectorXd { return f_obj(x, u).cast<Eigen::VectorXd>(); };
                 ctrl::MeasFunc   h  = [h_obj] (const Eigen::VectorXd &x, const Eigen::VectorXd &u)
                     -> Eigen::VectorXd { return h_obj(x, u).cast<Eigen::VectorXd>(); };
                 ctrl::JacobianFn Fj = [Fj_obj](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
                     -> Eigen::MatrixXd { return Fj_obj(x, u).cast<Eigen::MatrixXd>(); };
                 ctrl::JacobianFn Hj = [Hj_obj](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
                     -> Eigen::MatrixXd { return Hj_obj(x, u).cast<Eigen::MatrixXd>(); };

                 Eigen::MatrixXd P0;
                 if (!P0_obj.is_none()) P0 = P0_obj.cast<Eigen::MatrixXd>();

                 return new ctrl::ExtendedKalmanFilter(
                     n, p, std::move(f), std::move(h),
                     std::move(Fj), std::move(Hj), Q, R, Ts, P0);
             }),
             py::arg("n"), py::arg("p"),
             py::arg("f"), py::arg("h"),
             py::arg("F_jac"), py::arg("H_jac"),
             py::arg("Q"), py::arg("R"),
             py::arg("Ts"),
             py::arg("P0") = py::none(),
             "Construct EKF. P0=None initialises covariance to identity.")
        .def("predict",    &ctrl::ExtendedKalmanFilter::predict, py::arg("u"),
             "Prediction step: x^[k+1|k] = f(x^[k|k], u), P propagated via F Jacobian.")
        .def("update",     &ctrl::ExtendedKalmanFilter::update,
             py::arg("y"), py::arg("u"),
             "Update step: incorporate measurement y[k] via H Jacobian.")
        .def("step",       &ctrl::ExtendedKalmanFilter::step,
             py::arg("y"), py::arg("u_prev"),
             "Combined predict+update.  u_prev is the input applied at the previous step.")
        .def("reset",      &ctrl::ExtendedKalmanFilter::reset)
        .def("set_state",  &ctrl::ExtendedKalmanFilter::setState, py::arg("x0"),
             "Inject an initial state estimate.")
        .def("state",      &ctrl::ExtendedKalmanFilter::state,
             py::return_value_policy::copy,
             "Current state estimate x^[k|k] (n,).")
        .def("covariance", &ctrl::ExtendedKalmanFilter::covariance,
             py::return_value_policy::copy,
             "Current error covariance P[k|k] (n, n).")
        .def("sample_time",&ctrl::ExtendedKalmanFilter::sampleTime)
        .def("set_algebraic_constraint",
             [](ctrl::ExtendedKalmanFilter &ekf, py::object g_py,
                int n_diff, int n_alg, double tol) {
                 ekf.setAlgebraicConstraint(
                     [g_py](const Eigen::VectorXd &x1,
                            const Eigen::VectorXd &x2,
                            double u) -> Eigen::VectorXd {
                         return g_py(x1, x2, u).cast<Eigen::VectorXd>();
                     },
                     n_diff, n_alg, tol);
             },
             py::arg("g_alg"), py::arg("n_diff"), py::arg("n_alg"),
             py::arg("newton_tol") = 1e-9,
             R"doc(
Attach a DAE algebraic constraint for post-update state projection (P3).

After every update()/step() call, the algebraic block x2 = state[n_diff:]
is re-solved from g_alg(x1, x2, u) = 0, keeping the estimate on the
constraint manifold. Covariance is projected accordingly.

Parameters
----------
g_alg      : Callable (x1, x2, u: float) -> residual ndarray (n_alg,).
n_diff     : Number of differential states in the EKF state vector.
n_alg      : Number of algebraic states (must be state_size - n_diff).
newton_tol : Newton convergence tolerance (default 1e-9).
)doc")
        .def("has_algebraic_constraint",
             &ctrl::ExtendedKalmanFilter::hasAlgebraicConstraint,
             "True if an algebraic constraint has been attached.")
        .def_static("numerical_jacobian",
             [](py::object func, const Eigen::VectorXd &x, double eps_scale)
                 -> Eigen::MatrixXd {
                 return ctrl::ExtendedKalmanFilter::numericalJacobian(
                     [func](const Eigen::VectorXd &xx) -> Eigen::VectorXd {
                         return func(xx).cast<Eigen::VectorXd>();
                     }, x, eps_scale);
             },
             py::arg("func"), py::arg("x"), py::arg("eps_scale") = 1e-4,
             "Central-difference numerical Jacobian of func(x) -> array at point x. "
             "eps_i = eps_scale * max(|x_i|, 1) per element.",
             py::return_value_policy::copy);

    py::class_<ctrl::UnscentedKalmanFilter>(m, "UnscentedKalmanFilter", R"doc(
Unscented Kalman filter (UKF) - sigma-point nonlinear estimator (no Jacobians required).

Propagates 2n+1 deterministically chosen sigma points through the exact nonlinear
functions, capturing mean and covariance to third order.

f, h are Python callables with signature (x: np.ndarray, u: np.ndarray) -> np.ndarray.

Tuning guidance:
  alpha = 1e-3 (default): tightly clustered sigma points, good for n <= 3.
  alpha = 0.1-1.0: better for n >= 4 (avoids large negative Wc0 weight).
  beta  = 2.0 (default): optimal for Gaussian priors (encodes kurtosis in Wc0).
  kappa = 0.0 (default): standard scaling. Use 3-n for minimum-variance when n <= 3.

Example
-------
>>> def f(x, u): return np.array([x[0] + dt*x[1], x[1] + dt*u[0]])
>>> def h(x, u): return x[:1]   # measure position only
>>> ukf = ctrl.UnscentedKalmanFilter(n=2, p=1, f=f, h=h, Q=Q, R=R, Ts=dt)
>>> for k in range(N):
...     ukf.step(y[k], u[k-1])
...     x_hat = ukf.state()
)doc")
        .def(py::init(
             [](int n, int p,
                py::object f_obj, py::object h_obj,
                const Eigen::MatrixXd &Q,
                const Eigen::MatrixXd &R,
                double Ts,
                py::object P0_obj,
                double alpha, double beta, double kappa)
                 -> ctrl::UnscentedKalmanFilter* {
                 std::function<Eigen::VectorXd(const Eigen::VectorXd &,
                                               const Eigen::VectorXd &)>
                     f_wrap = [f_obj](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
                         -> Eigen::VectorXd { return f_obj(x, u).cast<Eigen::VectorXd>(); };
                 std::function<Eigen::VectorXd(const Eigen::VectorXd &,
                                               const Eigen::VectorXd &)>
                     h_wrap = [h_obj](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
                         -> Eigen::VectorXd { return h_obj(x, u).cast<Eigen::VectorXd>(); };

                 Eigen::MatrixXd P0;
                 if (!P0_obj.is_none()) P0 = P0_obj.cast<Eigen::MatrixXd>();

                 return new ctrl::UnscentedKalmanFilter(
                     n, p, std::move(f_wrap), std::move(h_wrap),
                     Q, R, Ts, P0, alpha, beta, kappa);
             }),
             py::arg("n"), py::arg("p"),
             py::arg("f"), py::arg("h"),
             py::arg("Q"), py::arg("R"),
             py::arg("Ts"),
             py::arg("P0")    = py::none(),
             py::arg("alpha") = 1e-3,
             py::arg("beta")  = 2.0,
             py::arg("kappa") = 0.0,
             "Construct UKF. P0=None initialises covariance to identity.")
        .def("predict",    &ctrl::UnscentedKalmanFilter::predict, py::arg("u"),
             "Prediction step: propagate sigma points through f.")
        .def("update",     &ctrl::UnscentedKalmanFilter::update,
             py::arg("y"), py::arg("u"),
             "Update step: propagate predicted sigma points through h, compute Kalman gain.")
        .def("step",       &ctrl::UnscentedKalmanFilter::step,
             py::arg("y"), py::arg("u_prev"),
             "Combined predict+update.")
        .def("reset",      &ctrl::UnscentedKalmanFilter::reset)
        .def("set_state",  &ctrl::UnscentedKalmanFilter::setState, py::arg("x0"),
             "Inject an initial state estimate.")
        .def("state",      &ctrl::UnscentedKalmanFilter::state,
             py::return_value_policy::copy,
             "Current state estimate x^[k|k] (n,).")
        .def("covariance", &ctrl::UnscentedKalmanFilter::covariance,
             py::return_value_policy::copy,
             "Current error covariance P[k|k] (n, n).")
        .def("sample_time",&ctrl::UnscentedKalmanFilter::sampleTime);
#endif

    // -----------------------------------------------------------------------
    // MovingHorizonEstimator + MHEParams  (Part 18)
    // -----------------------------------------------------------------------
    py::class_<ctrl::MHEParams>(m, "MHEParams",
        "Parameters for MovingHorizonEstimator.")
        .def(py::init<>())
        .def_readwrite("N",          &ctrl::MHEParams::N,
                       "Estimation horizon [steps].")
        .def_readwrite("wMin",       &ctrl::MHEParams::wMin,
                       "Per-element lower bound on process noise w.")
        .def_readwrite("wMax",       &ctrl::MHEParams::wMax,
                       "Per-element upper bound on process noise w.")
        .def_readwrite("qpMaxIter",  &ctrl::MHEParams::qpMaxIter,
                       "Gradient-projection iteration limit.")
        .def_readwrite("qpTol",      &ctrl::MHEParams::qpTol,
                       "QP convergence tolerance.")
        .def_readwrite("xMin",       &ctrl::MHEParams::xMin,
                       "Per-element lower box bound on x_0 (n,), empty = unconstrained.")
        .def_readwrite("xMax",       &ctrl::MHEParams::xMax,
                       "Per-element upper box bound on x_0 (n,), empty = unconstrained.")
        .def_readwrite("C_ineq",     &ctrl::MHEParams::C_ineq,
                       "Polytopic constraint LHS (m_c x n): C_ineq * x_0 <= d_ineq. "
                       "Empty = no polytopic constraint (E4).")
        .def_readwrite("d_ineq",     &ctrl::MHEParams::d_ineq,
                       "Polytopic constraint RHS (m_c,). Must be set with C_ineq (E4).")
        .def_readwrite("ineq_proj_iters", &ctrl::MHEParams::ineq_proj_iters,
                       "Hildreth iterations for polytope projection (default 20).");

    py::class_<ctrl::MovingHorizonEstimator>(m, "MovingHorizonEstimator", R"doc(
Moving Horizon Estimator (MHE) - batch state estimator via condensed QP.

Minimises a windowed sum of measurement residuals and process noise cost
over a receding horizon of N past steps. Equivalent to Kalman filter for
linear Gaussian systems without constraints; strictly better with constraints.

Example
-------
>>> mhe = ctrl.MovingHorizonEstimator(plant, Q_proc, R_meas, params)
>>> mhe.initialize(np.zeros(n), np.eye(n))
>>> for k in range(N):
...     x_hat = mhe.estimate(y[k], u[k])
)doc")
        .def(py::init<const ctrl::StateSpace &,
                      const Eigen::MatrixXd &,
                      const Eigen::MatrixXd &,
                      const ctrl::MHEParams &>(),
             py::arg("plant"), py::arg("Q_proc"), py::arg("R_meas"),
             py::arg("params") = ctrl::MHEParams(),
             "Construct MHE and precompute condensed QP matrices.")
        .def("initialize",
             &ctrl::MovingHorizonEstimator::initialize,
             py::arg("x0"), py::arg("P0"),
             "Set initial state estimate and arrival-cost covariance. Call before estimate().")
        .def("estimate",
             &ctrl::MovingHorizonEstimator::estimate,
             py::arg("y"), py::arg("u"),
             py::return_value_policy::copy,
             "Run one MHE step. Returns state estimate x^[k|k] (n,).")
        .def("set_horizon",
             &ctrl::MovingHorizonEstimator::setHorizon,
             py::arg("N"),
             "Change horizon N (clears history; call initialize() again).")
        .def("set_weight_matrices",
             &ctrl::MovingHorizonEstimator::setWeightMatrices,
             py::arg("Q_proc"), py::arg("R_meas"),
             "Update process/measurement noise covariances and rebuild condensed matrices.")
        .def("reset",         &ctrl::MovingHorizonEstimator::reset,
             "Reset history and state estimate to zero.")
        .def("state",         &ctrl::MovingHorizonEstimator::state,
             py::return_value_policy::copy,
             "Last returned state estimate (n,).")
        .def("last_converged",&ctrl::MovingHorizonEstimator::lastConverged,
             "True if the most recent QP converged.")
        .def("last_qp_iters", &ctrl::MovingHorizonEstimator::lastQPIters,
             "Gradient-projection iterations used in the most recent solve.")
        .def("sample_time",   &ctrl::MovingHorizonEstimator::sampleTime,
             "Sample time Ts [s].")
        // D1: mismatch detection
        .def("enable_mismatch_detection",
             [](ctrl::MovingHorizonEstimator& mhe,
                double sigma, double k_cusum, double h_threshold) {
                 ctrl::MismatchDetectorParams p;
                 p.sigma = sigma; p.k_cusum = k_cusum; p.h_threshold = h_threshold;
                 mhe.enableMismatchDetection(p);
             },
             py::arg("sigma") = 1.0, py::arg("k_cusum") = 0.5, py::arg("h_threshold") = 5.0,
             "Enable CUSUM mismatch detection on the MHE residual sequence (D1).")
        .def("mismatch_detected",        &ctrl::MovingHorizonEstimator::mismatchDetected,
             "True if CUSUM has exceeded h_threshold*sigma since last reset.")
        .def("mismatch_score",           &ctrl::MovingHorizonEstimator::mismatchScore,
             "Current CUSUM statistic max(C+, C-).")
        .def("reset_mismatch_detector",  &ctrl::MovingHorizonEstimator::resetMismatchDetector,
             "Reset CUSUM accumulators without disabling detection.");

    // SubspaceID bindings are implemented in bind_advanced (advanced_bindings.cpp).
    // The stale TODO stub was removed here (audit 2026-06-13, Finding 36).

    // -----------------------------------------------------------------------
    // ParticleFilter - SIR sequential importance resampling
    // -----------------------------------------------------------------------
    py::class_<ctrl::ParticleFilterParams>(m, "ParticleFilterParams",
        "Configuration for the SIR particle filter.")
        .def(py::init<>())
        .def_readwrite("n_particles",         &ctrl::ParticleFilterParams::n_particles)
        .def_readwrite("Q",                   &ctrl::ParticleFilterParams::Q)
        .def_readwrite("R",                   &ctrl::ParticleFilterParams::R)
        .def_readwrite("resample_threshold",  &ctrl::ParticleFilterParams::resample_threshold)
        .def_readwrite("Ts",                  &ctrl::ParticleFilterParams::Ts)
        .def_readwrite("seed",                &ctrl::ParticleFilterParams::seed);

    py::class_<ctrl::ParticleFilter>(m, "ParticleFilter", R"doc(
SIR particle filter for nonlinear/non-Gaussian state estimation.

Usage
-----
>>> pfp = ctrl.ParticleFilterParams(); pfp.n_particles = 500
>>> pfp.Q = np.eye(n) * q; pfp.R = np.eye(p) * r
>>> pf = ctrl.ParticleFilter(pfp, n_states, n_meas, f, h)
>>> pf.initialise(x0, P0)
>>> pf.step(y, u_prev)   # predict + update each step
>>> x_hat = pf.state()
)doc")
        .def(py::init([](const ctrl::ParticleFilterParams &p, int n_states, int n_meas,
                          py::object f_py, py::object h_py) {
            auto f = [f_py](const Eigen::VectorXd &x,
                            const Eigen::VectorXd &u) -> Eigen::VectorXd {
                return f_py(x, u).cast<Eigen::VectorXd>();
            };
            auto h = [h_py](const Eigen::VectorXd &x,
                            const Eigen::VectorXd &u) -> Eigen::VectorXd {
                return h_py(x, u).cast<Eigen::VectorXd>();
            };
            return std::make_unique<ctrl::ParticleFilter>(p, n_states, n_meas,
                                                          std::move(f), std::move(h));
        }), py::arg("params"), py::arg("n_states"), py::arg("n_meas"),
            py::arg("f"), py::arg("h"))
        .def("initialise", [](ctrl::ParticleFilter &self,
                               const Eigen::VectorXd &x0,
                               const Eigen::MatrixXd &P0) {
            self.initialise(x0, P0);
        }, py::arg("x0"), py::arg("P0") = Eigen::MatrixXd(),
           "Initialise particle cloud from N(x0, P0).")
        .def("predict", &ctrl::ParticleFilter::predict, py::arg("u"),
             "Propagate all particles through f and add process noise.")
        .def("update",  &ctrl::ParticleFilter::update,  py::arg("y"), py::arg("u"),
             "Update weights from measurement likelihood; resample if needed.")
        .def("step",    &ctrl::ParticleFilter::step,    py::arg("y"), py::arg("u_prev"),
             "Combined predict(u_prev) + update(y, u_prev).")
        .def("resample",&ctrl::ParticleFilter::resample,
             "Trigger systematic resampling immediately.")
        .def("state",   &ctrl::ParticleFilter::state,
             py::return_value_policy::copy, "Weighted mean estimate x_hat (n,).")
        .def("covariance", &ctrl::ParticleFilter::covariance,
             py::return_value_policy::copy, "Weighted sample covariance P (n x n).")
        .def("effective_sample_size", &ctrl::ParticleFilter::effectiveSampleSize,
             "N_eff = 1/sum(w_i^2). High -> good diversity; Low -> degeneracy.")
        .def_property_readonly("resample_count", &ctrl::ParticleFilter::resampleCount,
             "Number of times resampling has been triggered since construction.")
        .def("is_initialised", &ctrl::ParticleFilter::isInitialised,
             "True if initialise() has been called.")
        .def("sample_time",    &ctrl::ParticleFilter::sampleTime,
             "Sample time Ts [s] from parameters.")
        .def("reset",          &ctrl::ParticleFilter::reset,
             "Reset particles and weights; requires re-initialisation.");

    // -----------------------------------------------------------------------
    // GreyBoxEstimator (E1) - LM parameter estimation for ODE f(x,u,p)
    // -----------------------------------------------------------------------
    py::class_<ctrl::GreyBoxEstimator::Params>(m, "GreyBoxParams",
        "Parameters for GreyBoxEstimator (E1).")
        .def(py::init<>())
        .def_readwrite("p0",         &ctrl::GreyBoxEstimator::Params::p0,
                       "Initial parameter guess (n_p x 1).")
        .def_readwrite("lower",      &ctrl::GreyBoxEstimator::Params::lower,
                       "Per-element lower bounds (empty = unconstrained).")
        .def_readwrite("upper",      &ctrl::GreyBoxEstimator::Params::upper,
                       "Per-element upper bounds (empty = unconstrained).")
        .def_readwrite("Ts",         &ctrl::GreyBoxEstimator::Params::Ts,
                       "Sample / integration time [s].")
        .def_readwrite("rk4_steps",  &ctrl::GreyBoxEstimator::Params::rk4_steps,
                       "RK4 substeps per Ts.")
        .def_readwrite("lm_lambda0", &ctrl::GreyBoxEstimator::Params::lm_lambda0,
                       "Initial LM damping factor.")
        .def_readwrite("lm_nu",      &ctrl::GreyBoxEstimator::Params::lm_nu,
                       "LM damping scale factor (> 1).")
        .def_readwrite("max_iter",   &ctrl::GreyBoxEstimator::Params::max_iter,
                       "Maximum LM iterations.")
        .def_readwrite("tol_grad",   &ctrl::GreyBoxEstimator::Params::tol_grad,
                       "Convergence: stop when max|J'r| < tol_grad.")
        .def_readwrite("eps_jac",    &ctrl::GreyBoxEstimator::Params::eps_jac,
                       "FD step scale for parameter Jacobian.");

    py::class_<ctrl::GreyBoxEstimator::Result>(m, "GreyBoxResult",
        "Result from GreyBoxEstimator.fit().")
        .def_readonly("params",     &ctrl::GreyBoxEstimator::Result::params,
                      "Best-fit parameter vector (n_p x 1).")
        .def_readonly("cost",       &ctrl::GreyBoxEstimator::Result::cost,
                      "Final cost 0.5 * ||r||^2.")
        .def_readonly("iterations", &ctrl::GreyBoxEstimator::Result::iterations,
                      "LM iterations used.")
        .def_readonly("converged",  &ctrl::GreyBoxEstimator::Result::converged,
                      "True if gradient convergence criterion met.");

    py::class_<ctrl::GreyBoxEstimator>(m, "GreyBoxEstimator", R"doc(
Grey-box nonlinear parameter estimator (E1).

Minimises sum_k ||y_k - h(x_k(p), p)||^2 w.r.t. p via Levenberg-Marquardt.

ODE: x_dot = f(x, u, p)  [continuous-time, integrated with RK4]
Measurement: y = h(x, p)

f, h are Python callables with signatures:
  f(x: np.ndarray, u: np.ndarray, p: np.ndarray) -> np.ndarray  (xdot)
  h(x: np.ndarray, p: np.ndarray) -> np.ndarray                  (y)

Example
-------
>>> par = ctrl.GreyBoxParams()
>>> par.p0 = np.array([1.0, 0.5])   # initial [k, c] guess
>>> par.lower = np.array([0.1, 0.01]); par.upper = np.array([10.0, 5.0])
>>> par.Ts = 0.01
>>> est = ctrl.GreyBoxEstimator(f, h, par)
>>> res = est.fit(x0, U, Y)
>>> print(res.params, res.converged)
)doc")
        .def(py::init([](py::object f_py, py::object h_py,
                         const ctrl::GreyBoxEstimator::Params& par) {
                auto f = [f_py](const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u,
                                const Eigen::VectorXd& p) -> Eigen::VectorXd {
                    return f_py(x, u, p).cast<Eigen::VectorXd>();
                };
                auto h = [h_py](const Eigen::VectorXd& x,
                                const Eigen::VectorXd& p) -> Eigen::VectorXd {
                    return h_py(x, p).cast<Eigen::VectorXd>();
                };
                return std::make_unique<ctrl::GreyBoxEstimator>(
                    std::move(f), std::move(h), par);
             }),
             py::arg("f"), py::arg("h"), py::arg("params"),
             "Construct with ODE f, measurement h, and GreyBoxParams.")
        .def("fit", &ctrl::GreyBoxEstimator::fit,
             py::arg("x0"), py::arg("U"), py::arg("Y"),
             "Run LM estimation. Returns GreyBoxResult.")
        .def("predict", &ctrl::GreyBoxEstimator::predict,
             py::arg("x0"), py::arg("U"),
             py::return_value_policy::copy,
             "Simulate and return predicted output trajectory Y_hat (n_y x N).")
        .def("params",    &ctrl::GreyBoxEstimator::params,
             py::return_value_policy::copy,
             "Current estimated parameter vector (n_p x 1).")
        .def("is_fitted", &ctrl::GreyBoxEstimator::isFitted,
             "True after a successful fit() call.");

    // -----------------------------------------------------------------------
    // RecursiveGreyBoxEstimator (E2) - online param tracking via augmented UKF
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_ADVANCED_KALMAN) || (!defined(CTRL_DISABLE_ADVANCED_KALMAN))
    py::class_<ctrl::RecursiveGreyBoxEstimator::Params>(m, "RecursiveGreyBoxParams",
        "Parameters for RecursiveGreyBoxEstimator (E2).")
        .def(py::init<>())
        .def_readwrite("p0",        &ctrl::RecursiveGreyBoxEstimator::Params::p0,
                       "Initial parameter estimate (n_p x 1).")
        .def_readwrite("Q_state",   &ctrl::RecursiveGreyBoxEstimator::Params::Q_state,
                       "Process noise for state (n_x x n_x).")
        .def_readwrite("Q_param",   &ctrl::RecursiveGreyBoxEstimator::Params::Q_param,
                       "Process noise for params (n_p x n_p). Small = slow tracking.")
        .def_readwrite("R_meas",    &ctrl::RecursiveGreyBoxEstimator::Params::R_meas,
                       "Measurement noise covariance (n_y x n_y).")
        .def_readwrite("P0_param",  &ctrl::RecursiveGreyBoxEstimator::Params::P0_param,
                       "Initial param covariance (n_p x n_p). Empty -> 100 * Q_param.")
        .def_readwrite("Ts",        &ctrl::RecursiveGreyBoxEstimator::Params::Ts)
        .def_readwrite("rk4_steps", &ctrl::RecursiveGreyBoxEstimator::Params::rk4_steps)
        .def_readwrite("alpha",     &ctrl::RecursiveGreyBoxEstimator::Params::alpha,
                       "UKF sigma-point spread (0.1 recommended for n_aug >= 4).")
        .def_readwrite("beta",      &ctrl::RecursiveGreyBoxEstimator::Params::beta)
        .def_readwrite("kappa",     &ctrl::RecursiveGreyBoxEstimator::Params::kappa);

    py::class_<ctrl::RecursiveGreyBoxEstimator>(m, "RecursiveGreyBoxEstimator", R"doc(
Online parameter tracking via augmented-state UKF (E2).

Augments state x with parameter vector p: z = [x; p].
UKF process model integrates the ODE for x and holds p constant (+ diffusion Q_param).

f, h are Python callables (same signature as GreyBoxEstimator).

Example
-------
>>> par = ctrl.RecursiveGreyBoxParams()
>>> par.p0 = np.array([1.0])
>>> par.Q_state = 1e-4 * np.eye(2); par.Q_param = 1e-6 * np.eye(1)
>>> par.R_meas  = 0.01 * np.eye(1); par.Ts = 0.01
>>> rge = ctrl.RecursiveGreyBoxEstimator(f, h, n_state=2, n_meas=1, params=par)
>>> rge.initialize(x0, 0.1 * np.eye(2))
>>> for k in range(N):
...     x_hat = rge.step(y[k], u[k])
...     p_hat = rge.param_estimate()
)doc")
        .def(py::init([](py::object f_py, py::object h_py,
                         int n_state, int n_meas,
                         const ctrl::RecursiveGreyBoxEstimator::Params& par) {
                auto f = [f_py](const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u,
                                const Eigen::VectorXd& p) -> Eigen::VectorXd {
                    return f_py(x, u, p).cast<Eigen::VectorXd>();
                };
                auto h = [h_py](const Eigen::VectorXd& x,
                                const Eigen::VectorXd& p) -> Eigen::VectorXd {
                    return h_py(x, p).cast<Eigen::VectorXd>();
                };
                return std::make_unique<ctrl::RecursiveGreyBoxEstimator>(
                    std::move(f), std::move(h), n_state, n_meas, par);
             }),
             py::arg("f"), py::arg("h"),
             py::arg("n_state"), py::arg("n_meas"),
             py::arg("params"),
             "Construct with ODE f, measurement h, dimensions, and RecursiveGreyBoxParams.")
        .def("initialize", &ctrl::RecursiveGreyBoxEstimator::initialize,
             py::arg("x0"), py::arg("P0_state"),
             "Set initial state and build augmented UKF. Call before step().")
        .def("step", &ctrl::RecursiveGreyBoxEstimator::step,
             py::arg("y"), py::arg("u"),
             py::return_value_policy::copy,
             "Run one UKF predict+update. Returns state estimate x^[k|k] (n_x,).")
        .def("state_estimate",  &ctrl::RecursiveGreyBoxEstimator::stateEstimate,
             py::return_value_policy::copy,
             "Current state estimate x^[k|k] (n_x,).")
        .def("param_estimate",  &ctrl::RecursiveGreyBoxEstimator::paramEstimate,
             py::return_value_policy::copy,
             "Current parameter estimate p^[k|k] (n_p,).")
        .def("covariance", &ctrl::RecursiveGreyBoxEstimator::covariance,
             py::return_value_policy::copy,
             "Full augmented covariance P_aug[k|k] (n_aug x n_aug).")
        .def("is_initialized", &ctrl::RecursiveGreyBoxEstimator::isInitialized,
             "True after initialize() has been called.");
#endif
}
