#pragma once
#include "humid_plant.h"
#include <fstream>
#include <string>

namespace humid {

struct TickData {
    double       t;
    Disturbance  dist;
    ControlInput control;
    PlantOutput  output;
    double       ref_phi;
};

class TelemetryLogger {
public:
    explicit TelemetryLogger(const std::string& filepath);
    ~TelemetryLogger();

    void log(const TickData& d);
    void flush();  // print one-line summary to stdout

    double IAE()      const { return IAE_; }
    double ISE()      const { return ISE_; }
    double phi_final() const { return phi_final_; }

private:
    std::ofstream file_;
    double IAE_        = 0.0;
    double ISE_        = 0.0;
    double phi_final_  = 0.0;
    double t_prev_     = 0.0;
    bool   first_      = true;
};

} // namespace humid
