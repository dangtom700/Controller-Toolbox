// ============================================================
//  realtime_all.cpp  -  Real-time simulation of all tuned controllers
//
//  Simulates every controller against the example plant in a
//  wall-clock-accurate loop:
//    - Each step is paced to Ts seconds of real time.
//    - Missed deadlines are detected and reported.
//    - Live console output shows y, error, u, and timing.
//
//  Input:  tuned controller parameters (hard-coded from tune_all.cpp)
//          + example plant G(s)=1/(s^2+1.5s+1), ZOH Ts=0.01 s
//  Output: real-time console feed + rt_<name>.csv per controller
//
//  Build:  cmake target  realtime_all
//
//  Note:   True real-time guarantees require OS-level scheduling
//          (RTOS, SCHED_FIFO, etc.). This program demonstrates
//          the correct pacing structure; deadline violations on
//          a general-purpose OS are expected and non-fatal.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <functional>
#include <atomic>
#include <algorithm>

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

static constexpr double Ts = 0.01; // sample period  [s]
static constexpr int N_STEP = 500; // steps per controller
static constexpr double REF = 1.0; // unit step reference

// ---- Example plant ---------------------------------------------------------
static ctrl::StateSpace make_plant()
{
    ctrl::TransferFunction tf(
        {0.0, 4.9625e-5, 4.9125e-5},
        {1.0, -1.98511, 0.98522},
        Ts);
    return ctrl::tf2ss(tf);
}

// ---- FOPDT from step response (shared init) --------------------------------
static ctrl::StepResponseTuner::FOPDTModel get_fopdt(const ctrl::StateSpace &plant)
{
    std::vector<double> t_data(1500), y_data(1500);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd uv(1);
    uv << 1.0;
    for (int k = 0; k < 1500; ++k)
    {
        y_data[k] = ctrl::ssStep(plant, x, uv)(0);
        t_data[k] = k * Ts;
    }
    return ctrl::StepResponseTuner::identify(t_data, y_data, 1.0);
}

// ---- Timing statistics -----------------------------------------------------
struct RTStats
{
    double max_jitter_us = 0.0;  // worst-case timing jitter [micro-s]
    int missed = 0;              // steps that ran > 1.5*Ts late
    double avg_compute_us = 0.0; // average compute() wall time [micro-s]
};

// ---- Real-time runner for IController (error-based) -----------------------
struct RTResult
{
    std::string name;
    std::vector<double> t, y, e, u;
    RTStats stats;
    double y_final;
};

RTResult run_realtime_error(const std::string &name,
                            ctrl::IController &ctrl_obj,
                            ctrl::StateSpace plant,
                            bool verbose = false)
{
    RTResult res;
    res.name = name;
    res.t.reserve(N_STEP);
    res.y.reserve(N_STEP);
    res.e.reserve(N_STEP);
    res.u.reserve(N_STEP);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;

    const auto Ts_ns = std::chrono::nanoseconds(static_cast<long long>(Ts * 1e9));
    RTStats stats;
    double total_compute_us = 0.0;

    if (verbose)
    {
        std::cout << "\n  " << std::string(56, '-') << "\n";
        std::cout << "  " << std::setw(5) << "k"
                  << std::setw(8) << "t(s)"
                  << std::setw(10) << "y"
                  << std::setw(10) << "error"
                  << std::setw(10) << "u"
                  << std::setw(12) << "jitter(micro-s)" << "\n";
        std::cout << "  " << std::string(56, '-') << "\n";
    }

    TimePoint loop_start = Clock::now();

    for (int k = 0; k < N_STEP; ++k)
    {
        // Deadline for this step
        TimePoint deadline = loop_start + k * Ts_ns;

        // Wait until deadline
        auto now = Clock::now();
        if (now < deadline)
            std::this_thread::sleep_until(deadline);

        // Measure actual step start time
        TimePoint t_start = Clock::now();

        // Jitter from ideal
        double jitter_us = Duration(t_start - deadline).count() * 1e6;
        stats.max_jitter_us = std::max(stats.max_jitter_us, jitter_us);
        if (jitter_us > 1.5 * Ts * 1e6)
            ++stats.missed;

        // ---- Control computation ----
        auto compute_start = Clock::now();
        double err = REF - y;
        double u = ctrl_obj.compute(err);
        auto compute_end = Clock::now();
        double compute_us = Duration(compute_end - compute_start).count() * 1e6;
        total_compute_us += compute_us;

        // ---- Plant step ----
        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);

        res.t.push_back(k * Ts);
        res.y.push_back(y);
        res.e.push_back(err);
        res.u.push_back(u);

        if (verbose && (k % 50 == 0 || k == N_STEP - 1))
        {
            std::cout << "  "
                      << std::setw(5) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(10) << y
                      << std::setw(10) << err
                      << std::setw(10) << u
                      << std::setprecision(1)
                      << std::setw(12) << jitter_us << "\n";
        }
    }

    stats.avg_compute_us = total_compute_us / N_STEP;
    res.stats = stats;
    res.y_final = y;

    // Write CSV
    std::string fname = "rt_" + name + ".csv";
    std::ofstream f(fname);
    f << std::fixed << std::setprecision(6);
    f << "t,y,error,u\n";
    for (int k = 0; k < N_STEP; ++k)
        f << res.t[k] << "," << res.y[k] << "," << res.e[k] << "," << res.u[k] << "\n";

    return res;
}

// ---- Print real-time stats --------------------------------------------------
static void print_rt_stats(const RTResult &r)
{
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  y_final       = " << r.y_final << "\n"
              << "  avg_compute   = " << r.stats.avg_compute_us << " micro-s\n"
              << "  max_jitter    = " << r.stats.max_jitter_us << " micro-s\n"
              << "  missed steps  = " << r.stats.missed << " / " << N_STEP << "\n";
}

// ============================================================
int main()
{
    std::cout << "============================================================\n";
    std::cout << "  Real-Time Controller Runner  -  All lib/ controllers\n";
    std::cout << "  Ts=" << Ts << " s  N=" << N_STEP << " steps  ref=" << REF << "\n";
    std::cout << "============================================================\n";

    auto plant = make_plant();
    auto fopdt = get_fopdt(plant);
    const int n = plant.stateSize();
    const int m = plant.inputSize();

    std::vector<RTResult> all_results;

    // =========================================================
    //  1. DiscretePID (IMC)
    // =========================================================
    {
        std::cout << "\n[1] DiscretePID (IMC)  - real-time\n";
        ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::IMC);
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        pp.N = 20.0;
        ctrl::DiscretePID pid(pp, Ts);
        auto r = run_realtime_error("pid", pid, plant, true);
        print_rt_stats(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  2. DiscreteLQR (via LQRAdapter - runs at real time)
    // =========================================================
    {
        std::cout << "\n[2] DiscreteLQR (Bryson)  - real-time\n";
        Eigen::VectorXd xmax(n);
        xmax << 1.0, 1.0;
        Eigen::VectorXd umax(m);
        umax << 5.0;
        ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
        ctrl::DiscreteLQR lqr_obj(plant, lp);

        // Real-time loop using LQR directly (full state from plant)
        RTResult res;
        res.name = "lqr";
        res.t.reserve(N_STEP);
        res.y.reserve(N_STEP);
        res.e.reserve(N_STEP);
        res.u.reserve(N_STEP);

        auto plant2 = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
        double y = 0.0;
        Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(n);
        x_ref(0) = REF;

        const auto Ts_ns = std::chrono::nanoseconds(static_cast<long long>(Ts * 1e9));
        RTStats stats;
        double total_compute = 0.0;
        TimePoint loop_start = Clock::now();

        std::cout << "  " << std::string(56, '-') << "\n";
        std::cout << "  " << std::setw(5) << "k" << std::setw(8) << "t(s)"
                  << std::setw(10) << "y" << std::setw(10) << "error"
                  << std::setw(10) << "u" << std::setw(12) << "jitter(micro-s)" << "\n";
        std::cout << "  " << std::string(56, '-') << "\n";

        for (int k = 0; k < N_STEP; ++k)
        {
            TimePoint deadline = loop_start + k * Ts_ns;
            auto now = Clock::now();
            if (now < deadline)
                std::this_thread::sleep_until(deadline);

            TimePoint t_start = Clock::now();
            double jitter_us = Duration(t_start - deadline).count() * 1e6;
            stats.max_jitter_us = std::max(stats.max_jitter_us, jitter_us);
            if (jitter_us > 1.5 * Ts * 1e6)
                ++stats.missed;

            auto cs = Clock::now();
            auto u_vec = lqr_obj.compute(x, x_ref);
            double u = u_vec(0);
            auto ce = Clock::now();
            total_compute += Duration(ce - cs).count() * 1e6;

            Eigen::VectorXd uv(1);
            uv << u;
            y = ctrl::ssStep(plant2, x, uv)(0);

            res.t.push_back(k * Ts);
            res.y.push_back(y);
            res.e.push_back(REF - y);
            res.u.push_back(u);

            if (k % 50 == 0 || k == N_STEP - 1)
            {
                std::cout << "  " << std::setw(5) << k
                          << std::fixed << std::setprecision(3)
                          << std::setw(8) << k * Ts << std::setw(10) << y
                          << std::setw(10) << REF - y << std::setw(10) << u
                          << std::setprecision(1) << std::setw(12) << jitter_us << "\n";
            }
        }
        stats.avg_compute_us = total_compute / N_STEP;
        res.stats = stats;
        res.y_final = y;

        std::ofstream f("rt_lqr.csv");
        f << "t,y,error,u\n";
        for (int k = 0; k < N_STEP; ++k)
            f << std::fixed << std::setprecision(6)
              << res.t[k] << "," << res.y[k] << "," << res.e[k] << "," << res.u[k] << "\n";

        print_rt_stats(res);
        all_results.push_back(res);
    }

    // =========================================================
    //  3. DiscreteMPC
    // =========================================================
    {
        std::cout << "\n[3] DiscreteMPC  - real-time\n";
        auto rec = ctrl::MPCHorizonTuner::recommend(plant, Ts);
        ctrl::MPCParams mp;
        mp.Np = rec.Np;
        mp.Nc = rec.Nc;
        mp.rho_y = 1.0;
        mp.rho_u = 0.1;
        mp.uMin = -5.0;
        mp.uMax = 5.0;
        ctrl::DiscreteMPC mpc(plant, mp);
        auto r = run_realtime_error("mpc", mpc, plant, true);
        print_rt_stats(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  4. DiscreteLeadLag
    // =========================================================
    {
        std::cout << "\n[4] DiscreteLeadLag  - real-time\n";
        const double omega_c = 3.0;
        const double w = omega_c;
        const double g = 1.0 / std::sqrt((1.0 - w * w) * (1.0 - w * w) + (1.5 * w) * (1.5 * w));
        ctrl::LeadLagParams lp = ctrl::LoopShapingTuner::tuneImpl({omega_c, 40.0, g});
        ctrl::DiscreteLeadLag ll(lp, Ts);
        auto r = run_realtime_error("leadlag", ll, plant, false);
        print_rt_stats(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  5. DiscreteSMC
    // =========================================================
    {
        std::cout << "\n[5] DiscreteSMC  - real-time\n";
        ctrl::SMCParams sp;
        sp.c_e = 1.0;
        sp.c_de = 0.1;
        sp.K = 5.0;
        sp.phi = 0.5;
        sp.uMin = -10.0;
        sp.uMax = 10.0;
        ctrl::DiscreteSMC smc(sp, Ts);
        auto r = run_realtime_error("smc", smc, plant, false);
        print_rt_stats(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  6. DiscreteADRC
    // =========================================================
    {
        std::cout << "\n[6] DiscreteADRC  - real-time\n";
        ctrl::ADRCParams ap;
        ap.omega_o = 15.0;
        ap.omega_c = 3.0;
        ap.b0 = fopdt.K / fopdt.tau;
        ap.uMin = -20.0;
        ap.uMax = 20.0;
        ctrl::DiscreteADRC adrc(ap, Ts);

        // ADRC uses computeTracking (not error-based) - manual loop
        RTResult res;
        res.name = "adrc";
        res.t.reserve(N_STEP);
        res.y.reserve(N_STEP);
        res.e.reserve(N_STEP);
        res.u.reserve(N_STEP);

        auto plant2 = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
        double y = 0.0;

        const auto Ts_ns = std::chrono::nanoseconds(static_cast<long long>(Ts * 1e9));
        RTStats stats;
        double total_compute = 0.0;
        TimePoint loop_start = Clock::now();

        for (int k = 0; k < N_STEP; ++k)
        {
            TimePoint deadline = loop_start + k * Ts_ns;
            auto now = Clock::now();
            if (now < deadline)
                std::this_thread::sleep_until(deadline);
            TimePoint t_start = Clock::now();
            double jitter_us = Duration(t_start - deadline).count() * 1e6;
            stats.max_jitter_us = std::max(stats.max_jitter_us, jitter_us);
            if (jitter_us > 1.5 * Ts * 1e6)
                ++stats.missed;

            auto cs = Clock::now();
            double u = adrc.computeTracking(y, REF);
            auto ce = Clock::now();
            total_compute += Duration(ce - cs).count() * 1e6;

            Eigen::VectorXd uv(1);
            uv << u;
            y = ctrl::ssStep(plant2, x, uv)(0);
            res.t.push_back(k * Ts);
            res.y.push_back(y);
            res.e.push_back(REF - y);
            res.u.push_back(u);
        }
        stats.avg_compute_us = total_compute / N_STEP;
        res.stats = stats;
        res.y_final = y;

        std::ofstream f("rt_adrc.csv");
        f << "t,y,error,u\n";
        for (int k = 0; k < N_STEP; ++k)
            f << std::fixed << std::setprecision(6)
              << res.t[k] << "," << res.y[k] << "," << res.e[k] << "," << res.u[k] << "\n";

        print_rt_stats(res);
        all_results.push_back(res);
    }

    // =========================================================
    //  7. SmithPredictor
    // =========================================================
    {
        std::cout << "\n[7] SmithPredictor (inner IMC-PID, delay=5)  - real-time\n";
        ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::IMC);
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        pp.N = 20.0;
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::SmithPredictor sp(inner, plant, 5);
        auto r = run_realtime_error("smith", sp, plant, false);
        print_rt_stats(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  8. ControllerStack
    // =========================================================
    {
        std::cout << "\n[8] ControllerStack (Supervisory: SMC->PID)  - real-time\n";
        ctrl::PIDParams pp;
        pp.Kp = 3.0;
        pp.Ki = 2.0;
        pp.Kd = 0.1;
        pp.N = 20.0;
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        ctrl::SMCParams sp;
        sp.c_e = 1.0;
        sp.c_de = 0.1;
        sp.K = 6.0;
        sp.phi = 0.5;
        sp.uMin = -10.0;
        sp.uMax = 10.0;

        ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
        stack.addController(std::make_shared<ctrl::DiscreteSMC>(sp, Ts), "SMC",
                            1.0, [](double e, double)
                            { return std::abs(e) > 0.3; });
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "PID");

        auto r = run_realtime_error("stack", stack, plant, false);
        print_rt_stats(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  Summary table
    // =========================================================
    std::cout << "\n"
              << std::string(72, '=') << "\n";
    std::cout << "  Real-Time Summary  (Ts=" << Ts << "s, "
              << N_STEP * Ts << "s total per controller)\n";
    std::cout << std::string(72, '=') << "\n";
    std::cout << std::left
              << std::setw(14) << "Controller"
              << std::setw(10) << "y_final"
              << std::setw(14) << "avg_cmpt(micro-s)"
              << std::setw(14) << "max_jit(micro-s)"
              << std::setw(10) << "missed"
              << "\n";
    std::cout << std::string(72, '-') << "\n";

    // Write RT summary CSV
    std::ofstream fsum("rt_summary.csv");
    fsum << "controller,y_final,avg_compute_us,max_jitter_us,missed_steps\n";

    for (auto &r : all_results)
    {
        std::cout << std::left << std::setw(14) << r.name
                  << std::right
                  << std::fixed << std::setprecision(4)
                  << std::setw(10) << r.y_final
                  << std::setprecision(2)
                  << std::setw(14) << r.stats.avg_compute_us
                  << std::setw(14) << r.stats.max_jitter_us
                  << std::setw(10) << r.stats.missed
                  << "\n";
        fsum << r.name << "," << r.y_final << ","
             << r.stats.avg_compute_us << "," << r.stats.max_jitter_us
             << "," << r.stats.missed << "\n";
    }

    std::cout << std::string(72, '=') << "\n";
    std::cout << "  rt_summary.csv written.  Individual: rt_<name>.csv\n";

    return 0;
}
