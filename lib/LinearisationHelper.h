#pragma once
#include <Eigen/Dense>
#include <functional>
#include "PlantModel.h"   // StateSpace, c2d, C2dMethod

/**
 * @file LinearisationHelper.h
 * @brief Jacobian-based linearisation utilities for nonlinear discrete-time control.
 *
 * Provides:
 *  - Central-difference numerical Jacobians with state-scaled step sizes (same
 *    scaled-epsilon algorithm as ExtendedKalmanFilter::numericalJacobian).
 *  - lineariseAtPoint(): build a ZOH-discretised StateSpace from a continuous-time
 *    nonlinear model at a given operating point - the standard first step of any
 *    gain-scheduling or successive-linearisation MPC design.
 *
 * @par Typical workflow
 * @code
 *   // Continuous-time Van der Pol: xdot1=x2, xdot2=mu(1-x1^2)x2-x1+u
 *   auto f = [mu](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
 *       Eigen::VectorXd xdot(2);
 *       xdot(0) = x(1);
 *       xdot(1) = mu*(1 - x(0)*x(0))*x(1) - x(0) + u(0);
 *       return xdot;
 *   };
 *
 *   // Linearise at x=(0,0), u=0
 *   Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
 *   Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);
 *   ctrl::StateSpace sys = ctrl::lineariseAtPoint(f, x0, u0, Ts);
 *
 *   // Design LQR on the linearised model
 *   ctrl::LQRParams lqr_p; lqr_p.Q = ...; lqr_p.R = ...;
 *   ctrl::DiscreteLQR lqr(sys, lqr_p);
 * @endcode
 *
 * @note The type aliases ctrl::StateFunc and ctrl::MeasFunc declared here are
 *   identical to those declared in ExtendedKalmanFilter.h. Both declarations refer
 *   to the same underlying std::function instantiation, so including both headers
 *   in the same translation unit is safe.
 */

namespace ctrl
{

/** @brief Continuous-time dynamics: x_dot = f(x, u). */
using StateFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                const Eigen::VectorXd &u)>;

/** @brief Measurement function: y = h(x, u). */
using MeasFunc  = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                const Eigen::VectorXd &u)>;

/**
 * @brief Central-difference Jacobian of func with respect to x at (x0, u0).
 *
 * Step size per element: h_i = eps_scale * max(|x0_i|, 1). This gives O(eps_scale^2)
 * truncation error independent of state magnitude - critical for heterogeneous states
 * (e.g., positions in metres and angles in radians in the same vector).
 *
 * @param func       Function (x, u) -> VectorXd to differentiate.
 * @param x0         Operating point state (n * 1).
 * @param u0         Operating point input (m * 1).
 * @param eps_scale  Relative step scale (default 1e-4).
 * @return           Jacobian matrix A_c = df/dx  (output_dim * n).
 */
Eigen::MatrixXd jacobianX(const StateFunc &func,
                           const Eigen::VectorXd &x0,
                           const Eigen::VectorXd &u0,
                           double eps_scale = 1e-4);

/**
 * @brief Central-difference Jacobian of func with respect to u at (x0, u0).
 *
 * @param func       Function (x, u) -> VectorXd to differentiate.
 * @param x0         Operating point state (n * 1).
 * @param u0         Operating point input (m * 1).
 * @param eps_scale  Relative step scale (default 1e-4).
 * @return           Jacobian matrix B_c = df/du  (output_dim * m).
 */
Eigen::MatrixXd jacobianU(const StateFunc &func,
                           const Eigen::VectorXd &x0,
                           const Eigen::VectorXd &u0,
                           double eps_scale = 1e-4);

/**
 * @brief Linearise a continuous-time nonlinear system at (x0, u0) and discretise via ZOH.
 *
 * Computes continuous-time Jacobians:
 * @code
 *   A_c = df/dx|_(x0,u0),   B_c = df/du|_(x0,u0)
 *   C   = dh/dx|_(x0,u0),   D   = dh/du|_(x0,u0)
 * @endcode
 *
 * Then applies ctrl::c2d(sys_c, Ts, C2dMethod::ZOH) to produce a discrete-time StateSpace.
 *
 * @param f_continuous  Continuous-time dynamics xdot = f(x, u).
 * @param h             Output function y = h(x, u).
 * @param x0            Operating point state.
 * @param u0            Operating point input.
 * @param Ts            Sample time [s] for ZOH discretisation.
 * @param eps_scale     Jacobian step scale (default 1e-4).
 * @return              Discrete-time StateSpace (ZOH).
 */
StateSpace lineariseAtPoint(const StateFunc &f_continuous,
                             const MeasFunc  &h,
                             const Eigen::VectorXd &x0,
                             const Eigen::VectorXd &u0,
                             double Ts,
                             double eps_scale = 1e-4);

/**
 * @brief Linearise a continuous-time system at (x0, u0) with identity output (y = x).
 *
 * Equivalent to the two-argument overload with h(x,u) = x (C = I, D = 0).
 * Convenient when full-state measurement is assumed (e.g., for LQR design).
 *
 * @param f_continuous  Continuous-time dynamics xdot = f(x, u).
 * @param x0            Operating point state.
 * @param u0            Operating point input.
 * @param Ts            Sample time [s].
 * @param eps_scale     Jacobian step scale (default 1e-4).
 * @return              Discrete-time StateSpace with C = I, D = 0.
 */
StateSpace lineariseAtPoint(const StateFunc &f_continuous,
                             const Eigen::VectorXd &x0,
                             const Eigen::VectorXd &u0,
                             double Ts,
                             double eps_scale = 1e-4);

} // namespace ctrl
