#pragma once
#include "smismo_plant.h"
#include "controllers.h"
#include <string>

namespace smismo {

// Run one scenario with one controller; write CSV to log_dir/run_id_ctrlname.csv.
// CSV columns: t,x_ref,x_p,v_p,P1_bar,P2_bar,u1,u2,F_ext,Q_s_lpm,energy_J,
//              iae_cumulative
//
// The controller produces the scalar working-side command u_ctrl; the shared
// ValveAllocator (mode selection + off-side backpressure regulation to P_bd)
// maps it to the two PDCV commands each step.
void runSimulation(const PlantParams&    p,
                   const ScenarioConfig& sc,
                   ControllerBase&       ctrl,
                   const std::string&    log_dir);

} // namespace smismo
