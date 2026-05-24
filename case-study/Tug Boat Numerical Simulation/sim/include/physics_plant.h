#pragma once
#include "plant_parameters.h"
#include <Eigen/Dense>

// 3-DOF barge-tugboat unified plant (Li et al. 2026, Eq. 21).
// Integrates the combined system dynamics via classical RK4 at fixed dt.
//
// State: X = [x, y, psi, u, v, r]  (world-frame position, body-frame velocity)
// Input: tau_main = [tau_x, tau_y, tau_psi]  body-frame generalised force from tugs
//        tau_env  = [tau_x, tau_y, tau_psi]  body-frame environmental disturbance

namespace tug {

class PhysicsPlant {
public:
    explicit PhysicsPlant(const PlantParameters& p);

    // Advance one step.
    // tau_main: output of thrust allocator (body frame, N and N·m)
    // tau_env:  wind + current + wave drift (body frame)
    void step(const Eigen::Vector3d& tau_main,
              const Eigen::Vector3d& tau_env);

    // Full 6-element state [x, y, psi, u, v, r]
    const Eigen::Matrix<double,6,1>& state() const { return X_; }

    // Position/heading sub-vector
    Eigen::Vector3d eta() const { return X_.head<3>(); }
    // Body-frame velocity sub-vector
    Eigen::Vector3d nu()  const { return X_.tail<3>(); }

    void reset(const Eigen::Matrix<double,6,1>& X0 = Eigen::Matrix<double,6,1>::Zero());

private:
    const PlantParameters& pp_;
    Eigen::Matrix<double,6,1> X_;  // current state

    // RK4 derivative function f(X, tau_total)
    Eigen::Matrix<double,6,1> f(const Eigen::Matrix<double,6,1>& X,
                                 const Eigen::Vector3d& tau_total) const;

    // Rotation matrix R(psi)
    static Eigen::Matrix3d R(double psi);

    // Coriolis–centripetal matrix C(nu) for barge rigid body
    Eigen::Matrix3d C_rb(const Eigen::Vector3d& nu) const;
};

} // namespace tug
