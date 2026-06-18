#pragma once
#include "plant_parameters.h"
#include <Eigen/Dense>
#include <array>
#include <cstdint>

// Environmental disturbance generator for the tug-barge simulation.
// Implements Li et al. (2026) Eqs. (3)-(6):
//   - Wind load
//   - Ocean current load
//   - Simplified JONSWAP wave drift (2-component)
// All forces are returned in the BODY frame.

namespace tug {

struct EnvConditions {
    double wind_speed;    // m/s, absolute
    double wind_bearing;  // rad, direction wind comes FROM (world frame)
    double current_speed; // m/s
    double current_bearing; // rad, direction current flows TO (world frame)
    double Hs;            // significant wave height (m); 0 = no waves, < 0 = use plant default
    double Tp;            // peak period (s); < 0 = use plant default
};

class Environment {
public:
    explicit Environment(const PlantParameters& p, const EnvConditions& cond,
                         uint32_t seed = 42u);

    // Compute total environmental force vector in BODY frame at time t (s).
    // eta = [x, y, psi], nu = [u, v, r]
    Eigen::Vector3d compute(double t,
                            const Eigen::Vector3d& eta,
                            const Eigen::Vector3d& nu) const;

    // Individual components (body frame).
    Eigen::Vector3d windLoad(const Eigen::Vector3d& nu) const;
    Eigen::Vector3d currentLoad(const Eigen::Vector3d& nu) const;
    Eigen::Vector3d waveDrift(double t) const;

private:
    const PlantParameters& pp_;
    EnvConditions cond_;

    // JONSWAP 2-component wave parameters (pre-computed at construction)
    struct WaveComponent { double amplitude, omega, phase; };
    std::array<WaveComponent, 2> waves_;

    void initWaves(uint32_t seed);
};

} // namespace tug
