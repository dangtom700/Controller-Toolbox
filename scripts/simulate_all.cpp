// ============================================================
//  simulate_all.cpp  -  Closed-loop simulation for every controller
//
//  Input:  tuned controller parameters (from tune_all)  +
//          example plant G(s)=1/(s^2+1.5s+1), ZOH Ts=0.01 s
//  Output: CSV files sim_<name>.csv (t, y, error, u) for each
//          controller; summary printed to stdout.
//
//  Scenario: unit step reference from t=0; 1500 steps = 15 s.
//            Disturbance: +0.2 step injected at k=750.
//
//  Build: cmake target  simulate_all
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <memory>

static constexpr double Ts = 0.01;
static constexpr int N = 1500;      // total steps
static constexpr int N_DT = 750;    // disturbance injection step
static constexpr double DIST = 0.2; // disturbance magnitude
static constexpr double REF = 1.0;  // step reference

// ---------------------------------------------------------------------------
// Plant
// ---------------------------------------------------------------------------
static ctrl::StateSpace make_plant()
{
    ctrl::TransferFunction tf(
        {0.0, 4.9625e-5, 4.9125e-5},
        {1.0, -1.98511, 0.98522},
        Ts);
    return ctrl::tf2ss(tf);
}

// Collect open-loop step response
static void collect_step(const ctrl::StateSpace &plant,
                         std::vector<double> &t_out, std::vector<double> &y_out)
{
    t_out.resize(1500);
    y_out.resize(1500);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd uv(1);
    uv << 1.0;
    for (int k = 0; k < 1500; ++k)
    {
        y_out[k] = ctrl::ssStep(plant, x, uv)(0);
        t_out[k] = k * Ts;
    }
}

// ---------------------------------------------------------------------------
// Generic simulation runner for IController (error-based)
// Returns: {t_vec, y_vec, e_vec, u_vec}
// ---------------------------------------------------------------------------
struct SimResult
{
    std::vector<double> t, y, e, u;
    std::string name;
    double ise;       // integral squared error
    double overshoot; // peak overshoot (%)
    double settling;  // 2% settling time (s)
};

SimResult run_error_based(const std::string &name,
                          ctrl::IController &ctrl_obj,
                          ctrl::StateSpace plant)
{
    SimResult res;
    res.name = name;
    res.t.resize(N);
    res.y.resize(N);
    res.e.resize(N);
    res.u.resize(N);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    double ise = 0.0;
    double peak = 0.0;
    double t_settle = N * Ts;

    for (int k = 0; k < N; ++k)
    {
        double dist = (k >= N_DT) ? DIST : 0.0;
        double err = REF - y;
        double u = ctrl_obj.compute(err);

        Eigen::VectorXd uv(1);
        uv << (u + dist);
        y = ctrl::ssStep(plant, x, uv)(0);

        res.t[k] = k * Ts;
        res.y[k] = y;
        res.e[k] = err;
        res.u[k] = u;

        ise += err * err * Ts;
        if (y > peak)
            peak = y;
        // Settling: track last time |error| > 2% of REF
        if (std::abs(err) > 0.02 * REF)
            t_settle = k * Ts;
    }

    res.ise = ise;
    res.overshoot = (peak > REF) ? (peak - REF) / REF * 100.0 : 0.0;
    res.settling = t_settle;
    return res;
}

// ---------------------------------------------------------------------------
// Write CSV file
// ---------------------------------------------------------------------------
static void write_csv(const SimResult &r)
{
    std::string fname = "sim_" + r.name + ".csv";
    std::ofstream f(fname);
    f << std::fixed << std::setprecision(6);
    f << "t,y,error,u\n";
    for (size_t i = 0; i < r.t.size(); ++i)
        f << r.t[i] << "," << r.y[i] << "," << r.e[i] << "," << r.u[i] << "\n";
    std::cout << "  -> Written: " << fname << "\n";
}

// ---------------------------------------------------------------------------
// Print result summary
// ---------------------------------------------------------------------------
static void print_summary(const SimResult &r)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  ISE = " << r.ise
              << "  Overshoot = " << r.overshoot << " %"
              << "  t_settle = " << r.settling << " s"
              << "  y_final = " << r.y.back() << "\n";
}

// ============================================================
int main()
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "============================================================\n";
    std::cout << "  Controller Simulator  -  All controllers in lib/\n";
    std::cout << "  Plant: G(s)=1/(s^2+1.5s+1),  Ts=" << Ts << " s,  N=" << N << " steps\n";
    std::cout << "  Ref=" << REF << ",  Disturbance=" << DIST << " at k=" << N_DT << "\n";
    std::cout << "============================================================\n";

    auto plant = make_plant();
    const int n = plant.stateSize();
    const int m = plant.inputSize();
    const int p = plant.outputSize();

    // Shared: FOPDT from step response
    std::vector<double> t_data, y_data;
    collect_step(plant, t_data, y_data);
    auto fopdt = ctrl::StepResponseTuner::identify(t_data, y_data, 1.0);

    std::vector<SimResult> all_results;

    // =========================================================
    //  1. DiscretePID (IMC)
    // =========================================================
    {
        std::cout << "\n[1] DiscretePID (IMC)\n";
        ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::IMC);
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        pp.N = 20.0;
        ctrl::DiscretePID pid(pp, Ts);
        auto r = run_error_based("pid", pid, plant);
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  2. DiscretePID (ZN)
    // =========================================================
    {
        std::cout << "\n[2] DiscretePID (Ziegler-Nichols)\n";
        ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::ZieglerNichols);
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        pp.N = 20.0;
        ctrl::DiscretePID pid(pp, Ts);
        auto r = run_error_based("pid_zn", pid, plant);
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  3. DiscreteLQR via LQRAdapter
    // =========================================================
    {
        std::cout << "\n[3] DiscreteLQR (Bryson)\n";
        Eigen::VectorXd xmax(n);
        xmax << 1.0, 1.0;
        Eigen::VectorXd umax(m);
        umax << 5.0;
        ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
        ctrl::DiscreteLQR lqr(plant, lp);

        // Simulate manually (LQR needs full state)
        SimResult r;
        r.name = "lqr";
        r.t.resize(N);
        r.y.resize(N);
        r.e.resize(N);
        r.u.resize(N);
        auto plant2 = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
        double y = 0.0;
        double ise = 0.0, peak = 0.0, t_settle = N * Ts;

        Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(n);
        x_ref(0) = REF; // track output = first state in controllable canonical form

        for (int k = 0; k < N; ++k)
        {
            double dist = (k >= N_DT) ? DIST : 0.0;
            auto u_vec = lqr.compute(x, x_ref);
            double u = u_vec(0) + dist;
            Eigen::VectorXd uv(1);
            uv << u;
            y = ctrl::ssStep(plant2, x, uv)(0);

            r.t[k] = k * Ts;
            r.y[k] = y;
            r.e[k] = REF - y;
            r.u[k] = u_vec(0);
            ise += r.e[k] * r.e[k] * Ts;
            if (y > peak)
                peak = y;
            if (std::abs(r.e[k]) > 0.02 * REF)
                t_settle = k * Ts;
        }
        r.ise = ise;
        r.overshoot = (peak > REF) ? (peak - REF) / REF * 100.0 : 0.0;
        r.settling = t_settle;
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  4. DiscreteLQG
    // =========================================================
    {
        std::cout << "\n[4] DiscreteLQG (Bryson + isotropic Kalman)\n";
        Eigen::VectorXd xmax(n);
        xmax << 1.0, 1.0;
        Eigen::VectorXd umax(m);
        umax << 5.0;
        ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
        auto kp = ctrl::KalmanWeightTuner::isotropic(n, p, 0.01, 0.1);
        ctrl::DiscreteLQG lqg(plant, lp, kp.Qf, kp.Rf, kp.P0);

        SimResult r;
        r.name = "lqg";
        r.t.resize(N);
        r.y.resize(N);
        r.e.resize(N);
        r.u.resize(N);
        auto plant2 = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
        double y_out = 0.0;
        double ise = 0.0, peak = 0.0, t_settle = N * Ts;
        Eigen::VectorXd u_prev(m);
        u_prev.setZero();
        Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(n);
        x_ref(0) = REF;

        for (int k = 0; k < N; ++k)
        {
            double dist = (k >= N_DT) ? DIST : 0.0;
            Eigen::VectorXd ymeas(p);
            ymeas << y_out;
            auto u_vec = lqg.step(ymeas, u_prev, x_ref);
            double u_out = u_vec(0) + dist;
            u_prev = u_vec;
            Eigen::VectorXd uv(1);
            uv << u_out;
            y_out = ctrl::ssStep(plant2, x, uv)(0);

            r.t[k] = k * Ts;
            r.y[k] = y_out;
            r.e[k] = REF - y_out;
            r.u[k] = u_vec(0);
            ise += r.e[k] * r.e[k] * Ts;
            if (y_out > peak)
                peak = y_out;
            if (std::abs(r.e[k]) > 0.02 * REF)
                t_settle = k * Ts;
        }
        r.ise = ise;
        r.overshoot = (peak > REF) ? (peak - REF) / REF * 100.0 : 0.0;
        r.settling = t_settle;
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  5. DiscreteMPC
    // =========================================================
    {
        std::cout << "\n[5] DiscreteMPC (MPCHorizonTuner)\n";
        auto rec = ctrl::MPCHorizonTuner::recommend(plant, Ts);
        ctrl::MPCParams mp;
        mp.Np = rec.Np;
        mp.Nc = rec.Nc;
        mp.rho_y = 1.0;
        mp.rho_u = 0.1;
        mp.uMin = -5.0;
        mp.uMax = 5.0;
        ctrl::DiscreteMPC mpc(plant, mp);
        auto r = run_error_based("mpc", mpc, plant);
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  6. DiscreteLeadLag
    // =========================================================
    {
        std::cout << "\n[6] DiscreteLeadLag (LoopShaping)\n";
        const double omega_c = 3.0;
        const double w = omega_c;
        const double g = 1.0 / std::sqrt((1.0 - w * w) * (1.0 - w * w) + (1.5 * w) * (1.5 * w));
        ctrl::LeadLagParams lp = ctrl::LoopShapingTuner::tuneImpl({omega_c, 40.0, g});
        ctrl::DiscreteLeadLag ll(lp, Ts);
        auto r = run_error_based("leadlag", ll, plant);
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  7. DiscreteSMC
    // =========================================================
    {
        std::cout << "\n[7] DiscreteSMC\n";
        ctrl::SMCParams sp;
        sp.c_e = 1.0;
        sp.c_de = 0.1;
        sp.K = 5.0;
        sp.phi = 0.5;
        sp.uMin = -10.0;
        sp.uMax = 10.0;
        ctrl::DiscreteSMC smc(sp, Ts);
        auto r = run_error_based("smc", smc, plant);
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  8. DiscreteADRC
    // =========================================================
    {
        std::cout << "\n[8] DiscreteADRC\n";
        ctrl::ADRCParams ap;
        ap.omega_o = 15.0;
        ap.omega_c = 3.0;
        ap.b0 = fopdt.K / fopdt.tau;
        ap.uMin = -20.0;
        ap.uMax = 20.0;
        ctrl::DiscreteADRC adrc(ap, Ts);

        SimResult r;
        r.name = "adrc";
        r.t.resize(N);
        r.y.resize(N);
        r.e.resize(N);
        r.u.resize(N);
        auto plant2 = make_plant();
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
        double y = 0.0;
        double ise = 0.0, peak = 0.0, t_settle = N * Ts;

        for (int k = 0; k < N; ++k)
        {
            double dist = (k >= N_DT) ? DIST : 0.0;
            double u = adrc.computeTracking(y, REF);
            Eigen::VectorXd uv(1);
            uv << (u + dist);
            y = ctrl::ssStep(plant2, x, uv)(0);
            r.t[k] = k * Ts;
            r.y[k] = y;
            r.e[k] = REF - y;
            r.u[k] = u;
            ise += r.e[k] * r.e[k] * Ts;
            if (y > peak)
                peak = y;
            if (std::abs(r.e[k]) > 0.02 * REF)
                t_settle = k * Ts;
        }
        r.ise = ise;
        r.overshoot = (peak > REF) ? (peak - REF) / REF * 100.0 : 0.0;
        r.settling = t_settle;
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  9. ExtremumSeeker (quadratic cost surface J=(u-u*)^2)
    // =========================================================
    {
        std::cout << "\n[9] ExtremumSeeker (quadratic J=(theta-1)^2)\n";
        const double plant_bw_hz = 1.0 / (2.0 * M_PI * fopdt.tau);
        ctrl::ExtremumSeekerParams ep;
        ep.perturbFreq = std::max(5.0 * plant_bw_hz, 1.0);
        ep.perturbAmp = 0.05;
        ep.lpfCutoff = ep.perturbFreq / 10.0;
        ep.hpfCutoff = ep.lpfCutoff / 5.0;
        ep.integGain = 0.5;
        ep.seekMinimum = true;

        ctrl::ExtremumSeeker esc(ep, Ts);

        // ESC operates on a static cost: J = (theta - 1)^2
        SimResult r;
        r.name = "esc";
        r.t.resize(N);
        r.y.resize(N);
        r.e.resize(N);
        r.u.resize(N);
        for (int k = 0; k < N; ++k)
        {
            double theta = esc.currentEstimate();
            double cost = (theta - 1.0) * (theta - 1.0); // J
            double u = esc.compute(cost);
            r.t[k] = k * Ts;
            r.y[k] = theta;       // "output" = current estimate
            r.e[k] = theta - 1.0; // "error" = distance from optimum
            r.u[k] = u;
        }
        r.ise = 0.0;
        for (auto e : r.e)
            r.ise += e * e * Ts;
        r.overshoot = 0.0;
        r.settling = N * Ts;
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  10. SmithPredictor (inner IMC-PID, delay=5 steps)
    // =========================================================
    {
        std::cout << "\n[10] SmithPredictor (inner IMC-PID, delay=5 steps)\n";
        ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::IMC);
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        pp.N = 20.0;
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::SmithPredictor sp(inner, plant, 5);
        auto r = run_error_based("smith", sp, plant);
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  11. ControllerStack - Supervisory (SMC + PID)
    // =========================================================
    {
        std::cout << "\n[11] ControllerStack (Supervisory: SMC->PID)\n";
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

        auto r = run_error_based("stack", stack, plant);
        print_summary(r);
        write_csv(r);
        all_results.push_back(r);
    }

    // =========================================================
    //  Summary table
    // =========================================================
    std::cout << "\n"
              << std::string(70, '=') << "\n";
    std::cout << "  Simulation Summary\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << std::left
              << std::setw(16) << "Controller"
              << std::setw(12) << "ISE"
              << std::setw(16) << "Overshoot(%)"
              << std::setw(14) << "t_settle(s)"
              << std::setw(10) << "y_final"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    // Write summary CSV
    std::ofstream fsum("sim_summary.csv");
    fsum << "controller,ISE,overshoot_pct,t_settle_s,y_final\n";

    for (auto &r : all_results)
    {
        std::cout << std::left << std::setw(16) << r.name
                  << std::right << std::setw(12) << std::setprecision(4) << r.ise
                  << std::setw(16) << r.overshoot
                  << std::setw(14) << r.settling
                  << std::setw(10) << r.y.back()
                  << "\n";
        fsum << r.name << "," << r.ise << "," << r.overshoot << ","
             << r.settling << "," << r.y.back() << "\n";
    }
    std::cout << std::string(70, '=') << "\n";
    std::cout << "  sim_summary.csv written.\n";

    return 0;
}
