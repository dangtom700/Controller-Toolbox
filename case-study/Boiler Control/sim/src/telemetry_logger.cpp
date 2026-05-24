#include "telemetry_logger.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace boiler {

TelemetryLogger::TelemetryLogger(const std::string& filepath)
{
    file_.open(filepath);
    if (!file_.is_open())
        throw std::runtime_error("Cannot open log file: " + filepath);

    file_ << "t,y1,y2,y3,u1,u2,u3,du1,du2,du3,"
          << "ref_y1,ref_y2,ref_y3,e1,e2,e3,"
          << "IAE_y1,IAE_y2,IAE_y3,ISE_y1,ISE_y2,ISE_y3,E_valve\n";
    file_ << std::scientific << std::setprecision(6);
}

TelemetryLogger::~TelemetryLogger()
{
    if (file_.is_open()) file_.flush();
}

void TelemetryLogger::log(const TickData& d)
{
    if (first_) {
        t_prev_ = d.t;
        first_  = false;
    }
    double dt = (d.t > t_prev_ + 1e-9) ? (d.t - t_prev_) : Ts_;
    t_prev_ = d.t;

    double e1 = d.ref_y(0) - d.y(0);
    double e2 = d.ref_y(1) - d.y(1);
    double e3 = d.ref_y(2) - d.y(2);

    IAE_[0] += std::abs(e1) * dt;
    IAE_[1] += std::abs(e2) * dt;
    IAE_[2] += std::abs(e3) * dt;
    ISE_[0] += e1 * e1 * dt;
    ISE_[1] += e2 * e2 * dt;
    ISE_[2] += e3 * e3 * dt;
    E_valve_ += (d.du(0)*d.du(0) + d.du(1)*d.du(1) + d.du(2)*d.du(2)) * dt;

    file_ << d.t
          << ',' << d.y(0)     << ',' << d.y(1)     << ',' << d.y(2)
          << ',' << d.u(0)     << ',' << d.u(1)     << ',' << d.u(2)
          << ',' << d.du(0)    << ',' << d.du(1)    << ',' << d.du(2)
          << ',' << d.ref_y(0) << ',' << d.ref_y(1) << ',' << d.ref_y(2)
          << ',' << e1         << ',' << e2          << ',' << e3
          << ',' << IAE_[0]    << ',' << IAE_[1]    << ',' << IAE_[2]
          << ',' << ISE_[0]    << ',' << ISE_[1]    << ',' << ISE_[2]
          << ',' << E_valve_   << '\n';
}

void TelemetryLogger::flush()
{
    if (file_.is_open()) file_.flush();
    std::cout << std::fixed << std::setprecision(4)
              << "  IAE=[" << IAE_[0] << ", " << IAE_[1] << ", " << IAE_[2] << "]"
              << "  ISE=[" << ISE_[0] << ", " << ISE_[1] << ", " << ISE_[2] << "]"
              << "  E_valve=" << E_valve_ << "\n";
}

} // namespace boiler
