#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "ControllerToolbox.h"

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
             "Sample time Ts [s].");

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
    // SubspaceID  (optional - CTRL_HAS_SUBSPACE)
    // -----------------------------------------------------------------------
#if defined(CTRL_HAS_SUBSPACE)
    // TODO: bind N4SIDResult struct (model, order, singular_values, success)
    // TODO: bind SubspaceID::n4sid(y_data, u_data, n_order, n_block)
    // TODO: bind SubspaceID::suggestOrder(y_data, u_data, n_block, max_order)
    // Note: data matrices are MatrixXd (rows = channels, cols = time steps)
#endif
}
