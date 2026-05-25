#include "telemetry_logger.h"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace solar {

static const char* kHeader =
    "t_s,G_Wm2,T_amb_C,phi_amb,v_w_ms,"
    "m_dot_w_kgs,kr,"
    "Tw1_C,Tw2_C,T_ai_C,ma_dot_kgs,"
    "Q_cond_kW,W_comp_kW,EER,"
    "Me_calc,Me_target,"
    "Tc_C,eta_PV,W_PV_W,vi_ms,"
    "Q_op_ls,W_pump_W,eta_pump,"
    "EER_grid,W_net_kW,"
    "ref_Tw1_C,error_Tw1_C\n";

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
    const auto& w = d.weather;
    const auto& u = d.control;
    const auto& o = d.output;
    double err    = d.ref_Tw1 - o.Tw1_C;

    // Accumulate metrics (trapezoidal integration using step size inferred from t)
    if (!first_) {
        double dt = d.t - Ts_prev_;
        IAE_ += std::abs(err) * dt;
        ISE_ += err * err   * dt;
    }
    Ts_prev_ = d.t;
    first_   = false;

    // Clamp EER_grid to a printable value if infinite
    double eer_grid = std::isfinite(o.EER_grid) ? o.EER_grid : 999.0;

    file_
        << d.t           << ',' << w.G        << ',' << w.T_amb   << ','
        << w.phi_amb     << ',' << w.v_w      << ','
        << u.m_dot_w     << ',' << u.kr       << ','
        << o.Tw1_C       << ',' << o.Tw2_C    << ',' << o.T_ai_C  << ',' << o.ma_dot   << ','
        << o.Q_cond_kW   << ',' << o.W_comp_kW << ',' << o.EER    << ','
        << o.Me_calc     << ',' << o.Me_target << ','
        << o.Tc_C        << ',' << o.eta_PV   << ',' << o.W_PV_W  << ',' << o.vi_ms    << ','
        << o.Q_op_ls     << ',' << o.W_pump_W << ',' << o.eta_pump << ','
        << eer_grid      << ',' << o.W_net_kW << ','
        << d.ref_Tw1     << ',' << err
        << '\n';
}

void TelemetryLogger::flush()
{
    file_.flush();
    std::cout << std::fixed << std::setprecision(3)
              << "  IAE=" << IAE_ << "  ISE=" << ISE_ << '\n';
}

} // namespace solar
