#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <string>

// Per-tick CSV logger and performance metric accumulator.
//
// Columns: t, y1, y2, y3, u1, u2, u3, du1, du2, du3,
//          ref_y1, ref_y2, ref_y3, e1, e2, e3,
//          IAE_y1, IAE_y2, IAE_y3, ISE_y1, ISE_y2, ISE_y3, E_valve

namespace boiler {

struct TickData {
    double t;
    Eigen::Vector3d y;        // absolute plant outputs
    Eigen::Vector3d u;        // absolute valve positions
    Eigen::Vector3d du;       // valve increments this step
    Eigen::Vector3d ref_y;    // absolute setpoints
};

class TelemetryLogger {
public:
    explicit TelemetryLogger(const std::string& filepath);
    ~TelemetryLogger();

    void log(const TickData& d);

    // Print one-liner summary to stdout.
    void flush();

    double IAE_y1() const { return IAE_[0]; }
    double IAE_y2() const { return IAE_[1]; }
    double IAE_y3() const { return IAE_[2]; }
    double ISE_y1() const { return ISE_[0]; }
    double ISE_y2() const { return ISE_[1]; }
    double ISE_y3() const { return ISE_[2]; }
    double E_valve() const { return E_valve_; }

private:
    std::ofstream file_;
    double IAE_[3]{};
    double ISE_[3]{};
    double E_valve_ = 0.0;
    double Ts_ = 1.0;
    double t_prev_ = 0.0;
    bool first_ = true;
};

} // namespace boiler
