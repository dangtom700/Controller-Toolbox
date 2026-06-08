#pragma once
#include "sotec_plant.h"
#include "controllers.h"
#include <string>

namespace sotec {

// Run one scenario with one controller; write CSV to log_dir/id_ctrlname.csv.
// CSV columns: t,T_h_ref,T_h,T_coll,T_c,G_b,m_dot_f_cmd,m_dot_wf_cmd,
//              W_net,P_inlet,eta_th,delta_T_super,iae_cumulative
void runSimulation(const PlantParams&    p,
                   const ScenarioConfig& sc,
                   ControllerBase&       ctrl,
                   const std::string&    log_dir);

} // namespace sotec
