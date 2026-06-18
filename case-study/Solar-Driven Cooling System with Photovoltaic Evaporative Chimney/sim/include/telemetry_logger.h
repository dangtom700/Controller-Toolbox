#pragma once
#include "solar_plant.h"
#include <fstream>
#include <string>

// Per-tick CSV logger for the solar cooling simulation.
//
// CSV columns:
//   t_s, G_Wm2, T_amb_C, phi_amb, v_w_ms,
//   m_dot_w_kgs, kr,
//   Tw1_C, Tw2_C, T_ai_C, ma_dot_kgs,
//   Q_cond_kW, W_comp_kW, EER,
//   Me_calc, Me_target,
//   Tc_C, eta_PV, W_PV_W, vi_ms,
//   Q_op_ls, W_pump_W, eta_pump,
//   EER_grid, W_net_kW,
//   ref_Tw1_C, error_Tw1_C

namespace solar {

struct TickData {
    double       t;
    WeatherInput weather;
    ControlInput control;
    PlantOutput  output;
    double       ref_Tw1;
};

class TelemetryLogger {
public:
    explicit TelemetryLogger(const std::string& filepath);
    ~TelemetryLogger();

    void log(const TickData& d);
    void flush();   // print one-line summary to stdout

    double IAE() const { return IAE_; }
    double ISE() const { return ISE_; }

private:
    std::ofstream file_;
    double IAE_    = 0.0;
    double ISE_    = 0.0;
    double Ts_prev_ = 0.0;
    bool   first_  = true;
};

} // namespace solar
