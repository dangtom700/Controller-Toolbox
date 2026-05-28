#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

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
)doc")
        // TODO: bind constructor (takes f, h callables, Q, R, P0, Ts)
        // TODO: bind predict(u), update(y, u), step(y, u_prev, u_current)
        // TODO: bind state(), covariance(), sample_time(), reset()
        // TODO: bind static numerical_jacobian(f, x, eps_scale=1e-4)
        // Note: f and h are std::function<VectorXd(VectorXd)>;
        //       wrap Python callables via py::cpp_function
        ;

    py::class_<ctrl::UnscentedKalmanFilter>(m, "UnscentedKalmanFilter", R"doc(
Unscented Kalman filter - sigma-point nonlinear estimator (no Jacobians required).
)doc")
        // TODO: bind constructor and methods (same pattern as EKF)
        ;
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
