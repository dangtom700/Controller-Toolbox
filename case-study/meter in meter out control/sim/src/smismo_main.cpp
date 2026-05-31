#include "smismo_plant.h"
#include "smismo_controllers.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <memory>
#include <string>
#include <iomanip>
#include <cmath>

#ifndef SMISMO_SIM_SOURCE_DIR
#define SMISMO_SIM_SOURCE_DIR "."
#endif

namespace smismo {

// ---------------------------------------------------------------------------
// Scenario definitions
// ---------------------------------------------------------------------------

struct Scenario {
    std::string id;
    std::string description;
    double duration_s;
    double Ts;
    double x_start;    // initial piston position [m]
    double F_ext_step; // load step applied at t = load_time_s [N]
    double load_time_s;

    // Reference trajectory: called at each t to get r(t)
    std::function<double(double)> reference;
};

static std::vector<Scenario> makeScenarios(double Ts)
{
    constexpr double x_eq = 0.15;   // operating point position [m]

    return {
        {
            "S1_step",
            "5 cm position step",
            8.0, Ts, 0.10, 0.0, 1e9,
            [](double t) { return t < 1.0 ? 0.10 : 0.20; }
        },
        {
            "S2_multi_step",
            "Multi-step sequence",
            12.0, Ts, 0.05, 0.0, 1e9,
            [](double t) {
                if (t < 1.0)  return 0.05;
                if (t < 4.0)  return 0.15;
                if (t < 7.0)  return 0.10;
                if (t < 10.0) return 0.25;
                return 0.20;
            }
        },
        {
            "S3_disturbance",
            "1 kN load step at t=4s during tracking",
            10.0, Ts, 0.10, 1000.0, 4.0,
            [](double t) { return t < 2.0 ? 0.10 : 0.20; }
        }
    };
}

// ---------------------------------------------------------------------------
// Run one (scenario, controller) pair, write CSV
// ---------------------------------------------------------------------------

static void runSimulation(const Scenario& sc,
                          ControllerBase& ctrl,
                          const std::string& log_dir)
{
    SmismoPlant plant;
    plant.reset(sc.x_start);

    const double Ts = sc.Ts;
    const int    N  = static_cast<int>(sc.duration_s / Ts);

    // CSV file
    std::string fname = log_dir + "/" + sc.id + "_" + ctrl.name() + ".csv";
    std::ofstream csv(fname);
    csv << std::fixed << std::setprecision(6);
    csv << "t,r,x,v,p1_bar,p2_bar,ps_bar,u_cmd,xv1,xv2,F_ext\n";

    ctrl.reset();

    for (int k = 0; k < N; ++k) {
        const double t = k * Ts;

        if (t >= sc.load_time_s) plant.setLoad(sc.F_ext_step);

        const double r = sc.reference(t);
        Eigen::Vector4d y = plant.measure();
        double u_cmd = ctrl.compute(y, r);

        // Log
        const auto& s = plant.state();
        csv << t << ','
            << r << ','
            << s[0] << ','
            << s[1] << ','
            << (s[2] * 1e-5) << ','    // Pa -> bar
            << (s[3] * 1e-5) << ','
            << (s[4] * 1e-5) << ','
            << u_cmd << ','
            << s[5] << ','
            << s[7] << ','
            << plant.params().F_ext << '\n';

        // Sub-step: hydraulic resonance ~60 Hz with beta_e=300 MPa;
        // RK4 needs dt < 3ms. Use 5 inner steps of 1ms each.
        Eigen::Matrix<double,2,1> u_vec(u_cmd, u_cmd);
        constexpr int N_inner = 5;
        const double dt_inner = Ts / N_inner;
        for (int i = 0; i < N_inner; ++i)
            plant.step(dt_inner, u_vec);
    }

    csv.close();
}

// ---------------------------------------------------------------------------
// Metrics summary
// ---------------------------------------------------------------------------

static void printMetrics(const Scenario& sc,
                         const std::string& ctrl_name,
                         const std::string& log_dir)
{
    std::string fname = log_dir + "/" + sc.id + "_" + ctrl_name + ".csv";
    std::ifstream f(fname);
    if (!f) return;

    std::string line;
    std::getline(f, line); // header

    double ise = 0.0, iae = 0.0, t_settle = -1.0;
    const double settle_band = 0.003;   // +/-3 mm
    double last_t = 0.0, last_r = 0.0;

    while (std::getline(f, line)) {
        double t, r, x;
        // parse only first 3 columns
        auto p1 = line.find(',');
        auto p2 = line.find(',', p1+1);
        auto p3 = line.find(',', p2+1);
        t = std::stod(line.substr(0, p1));
        r = std::stod(line.substr(p1+1, p2-p1-1));
        x = std::stod(line.substr(p2+1, p3-p2-1));
        double e = r - x;
        const double Ts = sc.Ts;
        ise += e*e * Ts;
        iae += std::abs(e) * Ts;
        if (std::abs(e) <= settle_band) {
            if (t_settle < 0.0) t_settle = t;
        } else {
            t_settle = -1.0;
        }
        last_t = t; last_r = r; (void)last_r;
    }

    std::cout << "  " << std::left << std::setw(12) << ctrl_name
              << " ISE=" << std::scientific << std::setprecision(3) << ise
              << " IAE=" << iae
              << " Ts=" << (t_settle >= 0 ? std::to_string(t_settle) + "s" : "N/A")
              << '\n';
}

} // namespace smismo

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    std::string base_dir = (argc > 1) ? argv[1] : SMISMO_SIM_SOURCE_DIR;
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    constexpr double Ts = 0.005;   // 200 Hz control loop

    // Build linearised models once (shared across controllers)
    smismo::SmismoPlant ref_plant;
    ctrl::StateSpace ss2 = ref_plant.linearise2(Ts);
    ctrl::StateSpace ss4 = ref_plant.linearise4(Ts);
    const double b0 = ref_plant.accelGain();
    const smismo::SmismoOperatingPoint& op = ref_plant.op();

    std::cout << "Hydraulic accel gain b0 = " << b0 << " m/s^2/unit\n";

    // Controllers
    using namespace smismo;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    // Original 6
    ctrls.push_back(std::make_unique<PIDCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<LQRCtrl>(ss2, op, Ts));
    ctrls.push_back(std::make_unique<LQGCtrl>(ss2, op, Ts));
    ctrls.push_back(std::make_unique<SMCCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<ADRCCtrl>(b0, op, Ts));
    ctrls.push_back(std::make_unique<TubeMPCCtrl>(ss2, op));
    // Wave 1 additions
    ctrls.push_back(std::make_unique<LeadLagPIDCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<GPCRLSCtrl>(ss2, op, Ts));
    ctrls.push_back(std::make_unique<MRACSmismoCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<GainScheduledSmismoCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<EKFLQRSmismoCtrl>(ss4, op, Ts));
    ctrls.push_back(std::make_unique<HinfSmismoCtrl>(ss2, op, Ts));
    // Wave 2 additions
    ctrls.push_back(std::make_unique<NMPCSmismoCtrl>(ss4, op, Ts));
    ctrls.push_back(std::make_unique<FLSmismoCtrl>(b0, op, Ts));

    auto scenarios = makeScenarios(Ts);

    std::cout << "SMISMO Hydraulic Actuator - Controller Comparison\n";
    std::cout << "==================================================\n";
    std::cout << "Controllers : " << ctrls.size() << "\n";
    std::cout << "Scenarios   : " << scenarios.size() << "\n";
    std::cout << "Sample time : " << Ts * 1e3 << " ms\n\n";

    int done = 0, total = static_cast<int>(ctrls.size() * scenarios.size());

    for (auto& sc : scenarios) {
        std::cout << "\n--- " << sc.id << ": " << sc.description << " ---\n";

        for (auto& c : ctrls) {
            try {
                runSimulation(sc, *c, log_dir);
                printMetrics(sc, c->name(), log_dir);
            } catch (const std::exception& e) {
                std::cerr << "  ERROR [" << sc.id << " | " << c->name()
                          << "]: " << e.what() << '\n';
            }
            ++done;
        }
    }

    std::cout << "\nDone. " << done << '/' << total
              << " runs completed. CSV logs in: " << log_dir << '\n';
    return 0;
}
