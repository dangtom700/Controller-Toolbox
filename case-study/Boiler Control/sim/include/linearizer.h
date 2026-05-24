#pragma once
#include "boiler_plant.h"
#include <PlantModel.h>

// Linearise the Bell-Astrom boiler-turbine around an operating point.
//
// Returns a discrete-time ctrl::StateSpace (ZOH, double precision).
// The continuous Jacobians are computed analytically then discretised via
// ctrl::c2d(..., ctrl::C2dMethod::ZOH).
//
// State deviation:    dx = x - x_op
// Control deviation:  du = u - u_op
// Output deviation:   dy = y - y_op
//
//   dx[k+1] = Ad . dx[k] + Bd . du[k]
//   dy[k]   = Cd . dx[k] + Dd . du[k]

namespace boiler {

ctrl::StateSpace linearize(const OperatingPoint& op, double Ts = 1.0);

// Compute feedforward gain Nbar for steady-state setpoint tracking.
// Nbar = -(Cd . (Ad - I)^{-1} . Bd)^{-1}
// du_ff = Nbar . dy_ref   (adds to LQR feedback to eliminate steady-state error)
// Returns a 3x3 matrix; returns Identity if inversion fails.
Eigen::Matrix3d computeNbar(const ctrl::StateSpace& ss);

// Extract a SISO diagonal-channel state-space for axis i (i=0,1,2).
// Used by Smith Predictor, GPC (per-axis), Lead-Lag tuning.
ctrl::StateSpace diagonalChannel(const ctrl::StateSpace& ss, int axis);

} // namespace boiler
