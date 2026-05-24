#include "telemetry_logger.h"
#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace tug {

TelemetryLogger::TelemetryLogger(const std::string& filepath)
{
    file_.open(filepath);
    if (!file_.is_open())
        throw std::runtime_error("Cannot open log file: " + filepath);

    file_ << "t,x,y,psi,u,v,r,"
          << "tau_x,tau_y,tau_psi,"
          << "T1,T2,T3,T4,"
          << "ref_x,ref_y,ref_psi,"
          << "IAE_x,IAE_y,IAE_psi,E_fuel,sat_step,sat_total\n";
    file_ << std::scientific << std::setprecision(6);
}

TelemetryLogger::~TelemetryLogger()
{
    flush();
}

void TelemetryLogger::log(const TickData& d)
{
    // Determine dt from first two ticks
    if (first_) {
        t_prev_ = d.t;
        first_  = false;
    }
    double dt = (d.t > t_prev_ + 1e-9) ? (d.t - t_prev_) : 0.5;
    t_prev_ = d.t;

    // Error relative to reference
    double ex   = d.ref(0) - d.state(0);
    double ey   = d.ref(1) - d.state(1);
    double epsi = d.ref(2) - d.state(2);
    // Wrap heading error
    while (epsi >  M_PI) epsi -= 2.0 * M_PI;
    while (epsi <= -M_PI) epsi += 2.0 * M_PI;

    // Accumulate metrics
    IAE_[0]   += std::abs(ex)   * dt;
    IAE_[1]   += std::abs(ey)   * dt;
    IAE_[2]   += std::abs(epsi) * dt;

    for (auto T : d.T)
        E_fuel_ += T * T * dt;

    sat_total_ += d.sat_count;

    // Write row
    file_ << d.t
          << ',' << d.state(0) << ',' << d.state(1) << ',' << d.state(2)
          << ',' << d.state(3) << ',' << d.state(4) << ',' << d.state(5)
          << ',' << d.tau_c(0) << ',' << d.tau_c(1) << ',' << d.tau_c(2);

    for (auto T : d.T) file_ << ',' << T;

    file_ << ',' << d.ref(0) << ',' << d.ref(1) << ',' << d.ref(2)
          << ',' << IAE_[0]  << ',' << IAE_[1]  << ',' << IAE_[2]
          << ',' << E_fuel_
          << ',' << d.sat_count
          << ',' << sat_total_
          << '\n';
}

void TelemetryLogger::flush()
{
    if (file_.is_open()) file_.flush();
}

} // namespace tug
