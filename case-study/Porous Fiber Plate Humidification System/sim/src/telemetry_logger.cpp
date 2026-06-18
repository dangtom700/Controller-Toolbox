#include "telemetry_logger.h"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace humid {

static const char* kHeader =
    "t_s,"
    "T_out_C,phi_out,n_occ,"
    "u_fan_ms,Ta_sp_C,"
    "phi_room,phi_measured,omega_room_gkg,"
    "phi_in,H_gh,Tp_C,Re,Sh,hm,"
    "ref_phi,error_pct\n";

TelemetryLogger::TelemetryLogger(const std::string& filepath)
    : file_(filepath)
{
    if (!file_.is_open())
        throw std::runtime_error("TelemetryLogger: cannot open: " + filepath);
    file_ << kHeader;
    file_ << std::fixed << std::setprecision(6);
}

TelemetryLogger::~TelemetryLogger()
{
    if (file_.is_open()) file_.close();
}

void TelemetryLogger::log(const TickData& d)
{
    const auto& dist = d.dist;
    const auto& u    = d.control;
    const auto& o    = d.output;
    double err_pct   = (d.ref_phi - o.phi_measured) * 100.0;

    if (!first_) {
        double dt = d.t - t_prev_;
        IAE_ += std::abs(err_pct) * dt;
        ISE_ += err_pct * err_pct * dt;
    }
    t_prev_    = d.t;
    first_     = false;
    phi_final_ = o.phi_room;

    file_
        << d.t           << ','
        << (dist.T_out_K - 273.15) << ',' << dist.phi_out << ',' << dist.n_occ << ','
        << u.u_fan << ',' << (u.Ta_sp - 273.15) << ','
        << o.phi_room << ',' << o.phi_measured << ',' << o.omega_room << ','
        << o.phi_in   << ',' << o.H_gh << ',' << (o.Tp_K - 273.15) << ','
        << o.Re << ',' << o.Sh << ',' << o.hm << ','
        << d.ref_phi << ',' << err_pct
        << '\n';
}

void TelemetryLogger::flush()
{
    file_.flush();
    std::cout << std::fixed << std::setprecision(1)
              << "  IAE=" << IAE_
              << "  phi_final=" << phi_final_ * 100.0 << "%\n";
}

} // namespace humid
