#pragma once
#include "plant_parameters.h"
#include <Eigen/Dense>
#include <array>
#include <fstream>
#include <string>

// Per-tick CSV logger and performance metric accumulator.
// Columns: t, x, y, psi, u, v, r,
//          tau_x, tau_y, tau_psi,
//          T1, T2, T3, T4,
//          IAE_x, IAE_y, IAE_psi, E_fuel, sat_count_step, sat_count_total

namespace tug {

struct TickData {
    double t;
    Eigen::Matrix<double,6,1> state;   // [x,y,psi,u,v,r]
    Eigen::Vector3d tau_c;             // commanded (N, N·m)
    std::array<double, NUM_TUGS> T;    // per-tug thrusts (N)
    Eigen::Vector3d ref;               // target [x,y,psi]
    int sat_count;                     // saturations this step
};

class TelemetryLogger {
public:
    // Open CSV file for writing.  Writes header on construction.
    explicit TelemetryLogger(const std::string& filepath);
    ~TelemetryLogger();

    // Log one tick and accumulate metrics.
    void log(const TickData& d);

    // Final aggregated metrics
    double IAE_x()    const { return IAE_[0]; }
    double IAE_y()    const { return IAE_[1]; }
    double IAE_psi()  const { return IAE_[2]; }
    double E_fuel()   const { return E_fuel_; }
    int    sat_total()const { return sat_total_; }

    void flush();

private:
    std::ofstream file_;
    double IAE_[3]{};
    double E_fuel_ = 0.0;
    int    sat_total_ = 0;
    double dt_ = 0.5;   // set from first tick delta
    double t_prev_ = 0.0;
    bool   first_  = true;
};

} // namespace tug
