#pragma once
#include "plant_parameters.h"
#include <Eigen/Dense>
#include <array>

// Thrust allocator: maps body-frame generalised force tau_c = [tau_x, tau_y, tau_psi]
// to per-tug thrust magnitudes T[4] using pseudo-inverse allocation with box and
// rate constraints.
//
// Direction matrix B (3x4): B[:,i] = [cos(a_i), sin(a_i), lever_i]^T
// Pseudo-inverse:  T* = B^T (B B^T)^-1 tau_c
// Post-clamped to [T_min, T_max] with rate limiting |delta_T| <= dT_max per step.
//
// Constraint violations are logged via sat_count output.

namespace tug {

struct AllocResult {
    std::array<double, NUM_TUGS> T;  // per-tug thrust (N), after constraints
    int sat_count;                   // number of tugs that hit a constraint this step
};

class ThrustAllocator {
public:
    explicit ThrustAllocator(const PlantParameters& p);

    // Allocate tau_c (body frame) to per-tug thrusts.
    // T_prev: thrusts from previous step (for rate limiting).
    AllocResult allocate(const Eigen::Vector3d& tau_c,
                         const std::array<double, NUM_TUGS>& T_prev) const;

    // Reconstruct the achieved body-frame force from allocated thrusts.
    Eigen::Vector3d achieved(const std::array<double, NUM_TUGS>& T) const;

    // 3x4 direction matrix
    const Eigen::Matrix<double, 3, NUM_TUGS>& B() const { return B_; }

private:
    const PlantParameters& pp_;
    Eigen::Matrix<double, 3, NUM_TUGS> B_;           // direction matrix
    Eigen::Matrix<double, NUM_TUGS, 3> B_pinv_;      // pseudo-inverse B^T(BB^T)^-1

    void buildDirectionMatrix();
};

} // namespace tug
