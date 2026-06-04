// bench_controllers.cpp  - v2
// Full per-step latency benchmark for every Controller Toolbox controller.
//
// Build:  cmake -B build -DCTRL_BUILD_BENCHMARKS=ON
//         cmake --build build --target bench_controllers
// Run:    build/benchmark/bench_controllers[.exe]
//
// Output: tabular avg / stddev / min / max / throughput per controller,
//         followed by a summary table sorted fastest-to-slowest.
//
// Method: steady_clock hot-loop; Welford online variance; 10 % warm-up discarded.
// Volatile: the return value of fn(k) is stored in a volatile to prevent
// dead-code elimination without adding OS fence overhead.

#include "ControllerToolbox.h"
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;
using Ns    = std::chrono::nanoseconds;

// ---------------------------------------------------------------------------
// Benchmark result
// ---------------------------------------------------------------------------
struct BenchResult
{
    std::string name;
    long long   avg_ns    = 0;
    long long   std_ns    = 0;
    long long   min_ns    = 0;
    long long   max_ns    = 0;
    long long   steps     = 0;
    double      msteps_s  = 0.0; ///< throughput: million steps per second
};

void printHeader()
{
    std::cout << "  " << std::left  << std::setw(32) << "Controller"
              << std::right << std::setw(10) << "Steps"
              << std::setw(10) << "Avg[ns]"
              << std::setw(10) << "Std[ns]"
              << std::setw(10) << "Min[ns]"
              << std::setw(10) << "Max[ns]"
              << std::setw(13) << "Throughput"
              << "\n"
              << "  " << std::string(95, '-') << "\n";
}

void printRow(const BenchResult &r)
{
    std::cout << "  " << std::left  << std::setw(32) << r.name
              << std::right << std::setw(10) << r.steps
              << std::setw(10) << r.avg_ns
              << std::setw(10) << r.std_ns
              << std::setw(10) << r.min_ns
              << std::setw(10) << r.max_ns
              << std::setw(10) << std::fixed << std::setprecision(2) << r.msteps_s
              << " Mst/s\n";
}

// ---------------------------------------------------------------------------
// bench() - time a lambda fn(step_index) -> double
//   Warmup fraction is discarded, then Welford online mean/variance.
// ---------------------------------------------------------------------------
template <typename Fn>
BenchResult bench(const std::string &name, long long total_steps,
                  double warmup_frac, Fn fn)
{
    const long long warmup = static_cast<long long>(total_steps * warmup_frac);
    for (long long k = 0; k < warmup; ++k)
        (void)fn(k);

    long long lo = LLONG_MAX, hi = 0;
    double    welford_mean = 0.0, welford_M2 = 0.0;
    const long long timed = total_steps - warmup;

    for (long long k = 0; k < timed; ++k)
    {
        auto t0 = Clock::now();
        volatile double u = fn(k);
        auto t1 = Clock::now();
        (void)u;

        const long long dt = std::chrono::duration_cast<Ns>(t1 - t0).count();

        // Welford's online mean + variance
        const double delta = static_cast<double>(dt) - welford_mean;
        welford_mean += delta / static_cast<double>(k + 1);
        welford_M2   += delta * (static_cast<double>(dt) - welford_mean);

        lo = std::min(lo, dt);
        hi = std::max(hi, dt);
    }

    const double var    = (timed > 1) ? welford_M2 / static_cast<double>(timed - 1) : 0.0;
    const double std_d  = std::sqrt(var);
    const double tput   = (welford_mean > 0.0) ? 1e3 / welford_mean : 0.0; // Mstep/s

    BenchResult r;
    r.name      = name;
    r.avg_ns    = static_cast<long long>(welford_mean + 0.5);
    r.std_ns    = static_cast<long long>(std_d + 0.5);
    r.min_ns    = lo;
    r.max_ns    = hi;
    r.steps     = timed;
    r.msteps_s  = tput;
    return r;
}

// ---------------------------------------------------------------------------
// makePlant - stable n-state SISO discrete plant (chain + damping)
// ---------------------------------------------------------------------------
ctrl::StateSpace makePlant(int n, double Ts = 0.01)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n - 1; ++i)
        A(i, i + 1) = 1.0;
    for (int i = 0; i < n; ++i)
        A(i, i) -= 1.0;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, 1);
    B(n - 1, 0) = 1.0;
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(1, n);
    C(0, 0) = 1.0;
    const Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);

    return ctrl::StateSpace(A, B, C, D, Ts);
}

} // anonymous namespace

// ===========================================================================
// main
// ===========================================================================
int main()
{
    const long long STEPS     = 1'000'000;
    const long long STEPS_OPT = 100'000;   // slower QP-based controllers
    const double    Ts        = 0.01;
    const double    WARMUP    = 0.10;

    std::cout << "======================================================================"
                 "================\n"
              << "  Controller Toolbox  -  Per-Step Latency Benchmark  (v2)\n"
              << "  Steps: " << STEPS << " (QP-based: " << STEPS_OPT << ")"
              << "   Warm-up: " << static_cast<int>(WARMUP * 100) << "%"
              << "   Platform: " << sizeof(void *) * 8 << "-bit\n"
              << "======================================================================"
                 "================\n\n";

    std::vector<BenchResult> all_results;
    auto record = [&](BenchResult r) {
        printRow(r);
        all_results.push_back(std::move(r));
    };

    // =======================================================================
    // Section 1 - SISO fixed (state-dimension-independent)
    // =======================================================================
    std::cout << "-- SISO Fixed -----------------------------------------------------------"
                 "----------------\n";
    printHeader();

    // DiscretePID
    {
        ctrl::PIDParams pp;
        pp.Kp = 1.0; pp.Ki = 0.1; pp.Kd = 0.01; pp.N = 10.0;
        ctrl::DiscretePID pid(pp, Ts);
        record(bench("DiscretePID", STEPS, WARMUP,
                     [&](long long) { return pid.compute(1.0); }));
    }

    // DiscreteLeadLag
    {
        ctrl::LeadLagParams lp;
        lp.gain = 2.0; lp.continuousZero = 1.0; lp.continuousPole = 10.0;
        ctrl::DiscreteLeadLag ll(lp, Ts);
        record(bench("DiscreteLeadLag", STEPS, WARMUP,
                     [&](long long) { return ll.compute(1.0); }));
    }

    // DiscreteSMC
    {
        ctrl::SMCParams sp;
        sp.K = 5.0; sp.phi = 0.5;
        ctrl::DiscreteSMC smc(sp, Ts);
        record(bench("DiscreteSMC", STEPS, WARMUP,
                     [&](long long) { return smc.compute(1.0); }));
    }

    // DiscreteADRC
    {
        ctrl::ADRCParams ap;
        ap.omega_o = 10.0; ap.omega_c = 3.0; ap.b0 = 1.0;
        ctrl::DiscreteADRC adrc(ap, Ts);
        record(bench("DiscreteADRC", STEPS, WARMUP,
                     [&](long long) { return adrc.computeTracking(0.0, 1.0); }));
    }

    // ExtremumSeeker
    {
        ctrl::ExtremumSeekerParams ep;
        ep.perturbAmp = 0.05; ep.perturbFreq = 5.0;
        ep.lpfCutoff  = 1.0;  ep.hpfCutoff  = 0.5;
        ep.integGain  = 0.1;
        ctrl::ExtremumSeeker esc(ep, Ts);
        record(bench("ExtremumSeeker", STEPS, WARMUP,
                     [&](long long)
                     {
                         const double theta = esc.currentEstimate();
                         const double cost  = (theta - 2.0) * (theta - 2.0);
                         return esc.compute(cost);
                     }));
    }

    // FuzzyPD
    {
        ctrl::FuzzyPDParams fp;
        fp.e_scale = 5.0; fp.de_scale = 1.0; fp.u_scale = 2.0;
        ctrl::FuzzyPD fpd(fp, Ts);
        record(bench("FuzzyPD", STEPS, WARMUP,
                     [&](long long) { return fpd.compute(1.0); }));
    }

    // FuzzyPID
    {
        ctrl::FuzzyPIDParams fpp;
        fpp.pd.e_scale = 5.0; fpp.pd.de_scale = 1.0; fpp.pd.u_scale = 2.0;
        fpp.Ki = 0.05; fpp.Kb = 1.0; fpp.uMin = -10.0; fpp.uMax = 10.0;
        ctrl::FuzzyPID fpid(fpp, Ts);
        record(bench("FuzzyPID", STEPS, WARMUP,
                     [&](long long) { return fpid.compute(1.0); }));
    }

    // SmithPredictor (delay = 8 steps)
    {
        ctrl::StateSpace sys2 = makePlant(2, Ts);
        ctrl::PIDParams  pp;
        pp.Kp = 1.0; pp.Ki = 0.1;
        auto pid_inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::SmithPredictor sp(pid_inner, sys2, /*delaySteps=*/8);
        record(bench("SmithPredictor (d=8)", STEPS, WARMUP,
                     [&](long long) { return sp.compute(1.0); }));
    }

    // RepetitiveController (period = 100 steps)
    {
        ctrl::PIDParams pp;
        pp.Kp = 1.0; pp.Ki = 0.1;
        auto pid_inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::RepetitiveParams rp;
        rp.periodSteps = 100; rp.Krc = 0.5; rp.Q = 0.98;
        ctrl::RepetitiveController rc(pid_inner, rp, Ts);
        record(bench("RepetitiveController (N=100)", STEPS, WARMUP,
                     [&](long long) { return rc.compute(1.0); }));
    }

    // ControllerStack - 2 PIDs in Supervisory mode
    {
        ctrl::PIDParams pp;
        pp.Kp = 1.0; pp.Ki = 0.1;
        ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "PID-A");
        stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "PID-B");
        record(bench("ControllerStack (2 PIDs)", STEPS, WARMUP,
                     [&](long long) { return stack.compute(1.0); }));
    }

    // RecursiveLeastSquares (online update, na=1, nb=1)
    {
        ctrl::RecursiveLeastSquares rls(1, 1, Ts, 0.99);
        double y = 0.5, u_in = 0.1;
        record(bench("RecursiveLeastSquares", STEPS, WARMUP,
                     [&](long long k) {
                         y += 0.01 * static_cast<double>(k & 63) - 0.32;
                         return rls.update(y, u_in);
                     }));
    }

    std::cout << "\n";

    // =======================================================================
    // Section 2 - State-dimension-parametric controllers  (n = 2, 4, 8)
    // =======================================================================
    for (const int n : {2, 4, 8})
    {
        std::cout << "-- n = " << n
                  << " ------------------------------------------------------------"
                     "----------------\n";
        printHeader();

        ctrl::StateSpace sys = makePlant(n, Ts);

        // DiscreteLQR
        {
            ctrl::LQRParams lp;
            lp.Q = Eigen::MatrixXd::Identity(n, n);
            lp.R = Eigen::MatrixXd::Identity(1, 1);
            ctrl::DiscreteLQR lqr(sys, lp);
            Eigen::VectorXd x = Eigen::VectorXd::Ones(n) * 0.1;
            record(bench("DiscreteLQR (n=" + std::to_string(n) + ")", STEPS, WARMUP,
                         [&](long long) { return lqr.compute(x)(0); }));
        }

        // DiscreteLQG
        {
            ctrl::LQRParams lp;
            lp.Q = Eigen::MatrixXd::Identity(n, n);
            lp.R = Eigen::MatrixXd::Identity(1, 1);
            const Eigen::MatrixXd Q_noise = Eigen::MatrixXd::Identity(n, n) * 0.01;
            const Eigen::MatrixXd R_noise = Eigen::MatrixXd::Identity(1, 1) * 0.1;
            ctrl::DiscreteLQG lqg(sys, lp, Q_noise, R_noise);
            record(bench("DiscreteLQG (n=" + std::to_string(n) + ")", STEPS, WARMUP,
                         [&](long long) { return lqg.compute(0.5); }));
        }

        // DiscreteHinf (controller step only; synthesis is done once at setup)
        {
            ctrl::HinfResult hr;
            hr.feasible = true;
            hr.Ts       = Ts;
            hr.Ak = Eigen::MatrixXd::Identity(n, n) * 0.9;
            hr.Bk = Eigen::MatrixXd::Ones(n, 1) * 0.1;
            hr.Ck = Eigen::MatrixXd::Ones(1, n) * 0.05;
            hr.Dk = Eigen::MatrixXd::Zero(1, 1);
            ctrl::DiscreteHinf hinf(hr);
            record(bench("DiscreteHinf step (n=" + std::to_string(n) + ")", STEPS, WARMUP,
                         [&](long long) { return hinf.compute(0.5); }));
        }

        // DiscreteMPC
        {
            ctrl::MPCParams mp;
            mp.Np = 5; mp.Nc = 2; mp.rho_y = 1.0; mp.rho_u = 0.1;
            ctrl::DiscreteMPC mpc(sys, mp);
            record(bench("DiscreteMPC Np=5 (n=" + std::to_string(n) + ")", STEPS_OPT, WARMUP,
                         [&](long long) { return mpc.compute(1.0); }));
        }

        // GPC
        {
            ctrl::GPCParams gp;
            gp.Np = 5; gp.Nu = 2; gp.rho_y = 1.0; gp.rho_u = 0.1;
            ctrl::GeneralizedPredictiveController gpc(sys, gp);
            record(bench("GPC Np=5 (n=" + std::to_string(n) + ")", STEPS_OPT, WARMUP,
                         [&](long long) { return gpc.computeRef(0.5, 1.0); }));
        }

        std::cout << "\n";
    }

    // =======================================================================
    // Section 3 - Estimators  (n = 2, 4, 8)
    // =======================================================================
    for (const int n : {2, 4, 8})
    {
        std::cout << "-- Estimators n = " << n
                  << " --------------------------------------------------------"
                     "----------------\n";
        printHeader();

        ctrl::StateSpace sys = makePlant(n, Ts);

        const Eigen::MatrixXd Q_n = Eigen::MatrixXd::Identity(n, n) * 0.01;
        const Eigen::MatrixXd R_n = Eigen::MatrixXd::Identity(1, 1) * 0.1;
        Eigen::VectorXd y_obs(1);  y_obs << 0.5;
        Eigen::VectorXd u_obs(1);  u_obs << 0.1;

        // KalmanFilter
        {
            ctrl::KalmanFilter kf(sys, Q_n, R_n, Eigen::MatrixXd::Identity(n, n));
            record(bench("KalmanFilter (n=" + std::to_string(n) + ")", STEPS, WARMUP,
                         [&](long long)
                         {
                             kf.step(y_obs, u_obs);
                             return kf.state()(0);
                         }));
        }

        // ExtendedKalmanFilter - uses the same linear model as a sanity check
        {
            const Eigen::MatrixXd A_lin = sys.A;
            const Eigen::MatrixXd B_lin = sys.B;
            const Eigen::MatrixXd C_lin = sys.C;

            ctrl::StateFunc f_ekf = [A_lin, B_lin](const Eigen::VectorXd &x,
                                                    const Eigen::VectorXd &u)
            { return A_lin * x + B_lin * u; };

            ctrl::MeasFunc h_ekf = [C_lin](const Eigen::VectorXd &x,
                                           const Eigen::VectorXd &)
            { return C_lin * x; };

            ctrl::JacobianFn Fjac = [A_lin](const Eigen::VectorXd &,
                                            const Eigen::VectorXd &)
            { return A_lin; };

            ctrl::JacobianFn Hjac = [C_lin](const Eigen::VectorXd &,
                                            const Eigen::VectorXd &)
            { return C_lin; };

            ctrl::ExtendedKalmanFilter ekf(n, 1, f_ekf, h_ekf, Fjac, Hjac, Q_n, R_n, Ts);
            record(bench("EKF (n=" + std::to_string(n) + ")", STEPS, WARMUP,
                         [&](long long)
                         {
                             ekf.step(y_obs, u_obs);
                             return ekf.state()(0);
                         }));
        }

        // UnscentedKalmanFilter
        {
            const Eigen::MatrixXd A_lin = sys.A;
            const Eigen::MatrixXd B_lin = sys.B;
            const Eigen::MatrixXd C_lin = sys.C;

            auto f_ukf = [A_lin, B_lin](const Eigen::VectorXd &x,
                                        const Eigen::VectorXd &u)
            { return A_lin * x + B_lin * u; };

            auto h_ukf = [C_lin](const Eigen::VectorXd &x,
                                 const Eigen::VectorXd &)
            { return C_lin * x; };

            ctrl::UnscentedKalmanFilter ukf(n, 1, f_ukf, h_ukf, Q_n, R_n, Ts);
            record(bench("UKF (n=" + std::to_string(n) + ")", STEPS, WARMUP,
                         [&](long long)
                         {
                             ukf.step(y_obs, u_obs);
                             return ukf.state()(0);
                         }));
        }

        std::cout << "\n";
    }

    // =======================================================================
    // Section 4 - Part 22 additions: NMPC and AdaptiveSmithPredictor
    // =======================================================================
    std::cout << "-- Part 22 additions ---------------------------------------------------"
                 "----------------\n";
    printHeader();

    // NonlinearMPC (Np=5, Nu=2, n=2, 1 RTI step per sample)
    {
        const int n = 2;
        ctrl::NMPCParams np;
        np.Np = 5; np.Nu = 2;
        np.rho_y = 1.0; np.rho_u = 0.1;
        np.uMin = -10.0; np.uMax = 10.0;
        np.qpMaxIter = 200; np.qpTol = 1e-5;
        np.Ts = Ts; np.n_states = n; np.n_inputs = 1; np.n_outputs = 1;

        auto f_nl = [](const Eigen::VectorXd &x,
                       const Eigen::VectorXd &u) -> Eigen::VectorXd {
            Eigen::VectorXd xn(2);
            xn(0) = 0.9 * x(0) + 0.1 * x(1);
            xn(1) = 0.8 * x(1) + u(0);
            return xn;
        };
        Eigen::MatrixXd C_out(1, 2); C_out << 1.0, 0.0;
        ctrl::NonlinearMPC nmpc(np, f_nl, C_out);

        Eigen::VectorXd x_st = Eigen::VectorXd::Constant(n, 0.1);
        Eigen::VectorXd yref(1); yref << 1.0;
        record(bench("NonlinearMPC Np=5 (n=2)", STEPS_OPT, WARMUP,
                     [&](long long) {
                         nmpc.setState(x_st);
                         return nmpc.computeRef(x_st, yref)(0);
                     }));
    }

    // AdaptiveSmithPredictor (d=5, estimateInterval=1000 so no re-estimate mid-bench)
    {
        ctrl::StateSpace sys2 = makePlant(2, Ts);
        ctrl::PIDParams  pp;
        pp.Kp = 1.0; pp.Ki = 0.1;
        auto pid_inner2 = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::AdaptiveSPParams asp;
        asp.maxDelaySteps    = 10;
        asp.estimateInterval = 100000; // disable re-estimation during bench
        asp.bufferLen        = 200;
        ctrl::AdaptiveSmithPredictor asp_sp(pid_inner2, sys2, 5, Ts, asp);
        record(bench("AdaptiveSmithPredictor (d=5)", STEPS, WARMUP,
                     [&](long long) { return asp_sp.compute(1.0); }));
    }

    std::cout << "\n";

    // =======================================================================
    // Section 5 - Part 23-28 additions: MRAC / TubeMPC / ParticleFilter
    // =======================================================================
    std::cout << "-- Part 23-28 (MRAC / TubeMPC / ParticleFilter) ---------------------"
                 "----------------\n";
    printHeader();

    // MRACController (SISO, 1st-order reference model)
    {
        ctrl::MRACParams mp;
        mp.a_m = 0.85; mp.b_m = 0.15; mp.gamma_r = 5.0; mp.gamma_y = 5.0;
        ctrl::MRACController mrac(mp, Ts);
        mrac.setReference(1.0);
        record(bench("MRACController", STEPS, WARMUP,
                     [&](long long) { return mrac.compute(0.9); }));
    }

    // TubeMPC SISO (1-state plant, Np=10, same setup as ex67)
    {
        Eigen::MatrixXd At(1,1), Bt(1,1), Ct(1,1), Dt(1,1);
        At << 0.8; Bt << 0.2; Ct << 1.0; Dt << 0.0;
        ctrl::StateSpace sys_t(At, Bt, Ct, Dt, Ts);

        ctrl::TubeMPCParams tp;
        tp.Np   = 10; tp.Nu = 3;
        tp.Q    = Eigen::MatrixXd::Identity(1,1) * 2.0;
        tp.R    = Eigen::MatrixXd::Identity(1,1) * 0.3;
        tp.K    = Eigen::MatrixXd::Constant(1,1,-0.3);   // A+BK = 0.74 (stable)
        tp.wMax = Eigen::VectorXd::Constant(1, 0.1);
        tp.uMin = Eigen::VectorXd::Constant(1,-2.0);
        tp.uMax = Eigen::VectorXd::Constant(1, 2.0);
        tp.Ts   = Ts;

        ctrl::TubeMPC tmpc(sys_t, tp);
        Eigen::VectorXd x_t(1); x_t(0) = 0.1;
        const Eigen::VectorXd yref_t = Eigen::VectorXd::Constant(1, 1.0);
        record(bench("TubeMPC Np=10 (n=1)", STEPS_OPT, WARMUP,
                     [&](long long) {
                         return tmpc.computeRef(x_t, yref_t)(0);
                     }));
    }

    // ParticleFilter (n=1, N=200)
    {
        const int n_pf = 1, m_pf = 1;
        ctrl::ParticleFilterParams pfp;
        pfp.Q = Eigen::MatrixXd::Identity(n_pf, n_pf) * 0.01;
        pfp.R = Eigen::MatrixXd::Identity(m_pf, m_pf) * 0.1;
        pfp.n_particles = 200;

        ctrl::ParticleFilter pf(
            pfp, n_pf, m_pf,
            [](const Eigen::VectorXd& x, const Eigen::VectorXd&) {
                return Eigen::VectorXd(x * 0.8);
            },
            [](const Eigen::VectorXd& x, const Eigen::VectorXd&) {
                return x;
            }
        );
        Eigen::VectorXd x0_pf = Eigen::VectorXd::Zero(n_pf);
        pf.initialise(x0_pf, Eigen::MatrixXd::Identity(n_pf, n_pf) * 0.1);

        Eigen::VectorXd y_pf(m_pf); y_pf(0) = 0.5;
        Eigen::VectorXd u_pf(1);    u_pf(0) = 0.1;
        record(bench("ParticleFilter (N=200, n=1)", STEPS, WARMUP,
                     [&](long long) {
                         pf.step(y_pf, u_pf);
                         return pf.state()(0);
                     }));
    }

    std::cout << "\n";

    // =======================================================================
    // Section 6 - Part 30-31 ML / Data-Driven controllers
    // =======================================================================
    std::cout << "-- Part 31 (ML / Data-Driven: ILC/L1/CBF/GP/ESN/NeuralPID/CEM) --------"
                 "----------------\n";
    printHeader();


    // ILC - per-step cost: feedforward lookup + recordError (P-type, N=100)
    {
        ctrl::ILC::Params ip;
        ip.N        = 100;
        ip.Ts       = Ts;
        ip.Lp       = 0.5;
        ip.Q_filter = 0.95;
        ctrl::ILC ilc(ip);
        record(bench("ILC step (N=100)", STEPS, WARMUP,
                     [&](long long k) {
                         const int kk = static_cast<int>(k % ip.N);
                         ilc.recordError(kk, 0.01 * std::sin(0.1 * static_cast<double>(k)));
                         if (kk == ip.N - 1)
                             ilc.updateFeedforward();
                         return ilc.feedforward(kk);
                     }));
    }

    // L1AdaptiveController
    {
        ctrl::L1AdaptiveController::Params lp;
        lp.a_m = 0.85; lp.b_m = 0.15; lp.Gamma = 50.0; lp.omega_c = 2.0;
        ctrl::L1AdaptiveController l1(lp, Ts);
        l1.setReference(1.0);
        record(bench("L1AdaptiveController", STEPS, WARMUP,
                     [&](long long) { return l1.compute(0.9); }));
    }

    // CBFSafetyFilter (wrapping DiscretePID, SISO 1-state)
    {
        ctrl::PIDParams cbf_pp; cbf_pp.Kp = 2.0; cbf_pp.Ki = 0.1;
        auto pid_cbf = std::make_shared<ctrl::DiscretePID>(cbf_pp, Ts);

        const double x_max = 1.5;
        ctrl::CBFSafetyFilter::Params cbfp;
        cbfp.alpha = 1.0; cbfp.uMin = -2.0; cbfp.uMax = 2.0;
        ctrl::CBFSafetyFilter cbf(
            pid_cbf,
            [x_max](double x) { return x_max - x; },      // h(x)
            [](double)        { return -1.0; },             // dh/dx
            [](double x)      { return 0.8 * x; },         // f0(x)
            [](double)        { return 0.2; },              // g(x)
            cbfp, Ts);
        cbf.setState(0.5);
        record(bench("CBFSafetyFilter", STEPS, WARMUP,
                     [&](long long) { return cbf.compute(0.5); }));
    }

    // GaussianProcess predict (50 training points, 1-D input)
    {
        ctrl::GaussianProcess::Params gpp;
        gpp.length_scale = 1.0; gpp.signal_var = 1.0; gpp.noise_var = 0.01;
        gpp.n_max = 200;
        ctrl::GaussianProcess gp(1, gpp);
        for (int i = 0; i < 50; ++i) {
            Eigen::VectorXd xi(1); xi(0) = -2.5 + 0.1 * i;
            gp.addPoint(xi, std::sin(xi(0)));
        }
        gp.fit();
        Eigen::VectorXd x_gp(1); x_gp(0) = 0.5;
        record(bench("GaussianProcess predict (n=50)", STEPS_OPT, WARMUP,
                     [&](long long) {
                         auto pred = gp.predict(x_gp);
                         return pred.mean;
                     }));
    }

    // EchoStateNetwork predict (n_res=50, trained readout)
    {
        ctrl::EchoStateNetwork::Params ep;
        ep.n_res = 50; ep.n_in = 1; ep.n_out = 1; ep.washout = 10;
        ctrl::EchoStateNetwork esn(ep);
        for (int i = 0; i < 300; ++i) {
            Eigen::VectorXd u_tr(1); u_tr(0) = std::sin(0.1 * i);
            esn.stepReservoir(u_tr);
            Eigen::VectorXd y_tr(1); y_tr(0) = std::sin(0.1 * (i + 1));
            esn.addTrainingTarget(y_tr);
        }
        esn.fitReadout();
        Eigen::VectorXd u_esn(1); u_esn(0) = 0.5;
        record(bench("EchoStateNetwork (n_res=50)", STEPS, WARMUP,
                     [&](long long) { return esn.predict(u_esn)(0); }));
    }

    // NeuralPID (3->8->3, online backprop)
    {
        ctrl::NeuralPID::Params np;
        np.n_hidden = 8; np.lr = 1e-3; np.Ts = Ts;
        ctrl::NeuralPID npid(np);
        record(bench("NeuralPID (h=8)", STEPS, WARMUP,
                     [&](long long) { return npid.compute(0.5); }));
    }

    // CEMController (n=2, Np=10, N_samples=50, 3 CEM iters)
    {
        ctrl::CEMController::Params cp;
        cp.Np        = 10;
        cp.N_samples = 50;
        cp.n_iter    = 3;
        cp.Q = 1.0; cp.R = 0.1;
        cp.uMin = -5.0; cp.uMax = 5.0;
        auto f_cem = [](const Eigen::VectorXd& x,
                        const Eigen::VectorXd& u) -> Eigen::VectorXd {
            Eigen::VectorXd xn(2);
            xn(0) = 0.9 * x(0) + 0.1 * x(1);
            xn(1) = 0.8 * x(1) + u(0);
            return xn;
        };
        Eigen::MatrixXd C_cem(1, 2); C_cem << 1.0, 0.0;
        ctrl::CEMController cem(cp, f_cem, C_cem, Ts);
        Eigen::VectorXd x_cem = Eigen::VectorXd::Constant(2, 0.1);
        Eigen::VectorXd yref_cem(1); yref_cem(0) = 1.0;
        record(bench("CEMController Np=10 (n=2)", STEPS_OPT, WARMUP,
                     [&](long long) {
                         cem.setState(x_cem);
                         return cem.computeRef(x_cem, yref_cem)(0);
                     }));
    }

    // SINDy inference (PolyDeg2, n=2, m=1 - per-step predict() cost after offline fit)
    {
        ctrl::SINDy::Params sp;
        sp.n_state = 2; sp.n_input = 1;
        sp.library = ctrl::SINDyLibrary::PolyDeg2;
        sp.use_ols = true;
        ctrl::SINDy sindy(sp);

        Eigen::VectorXd x_tr(2), xn_tr(2), u_tr(1);
        x_tr << 0.5, 0.1;
        for (int k = 0; k < 300; ++k) {
            u_tr(0) = (k % 2 == 0) ? 0.3 : -0.3;
            xn_tr(0) = 0.9 * x_tr(0) + 0.1 * x_tr(1) + 0.01 * u_tr(0);
            xn_tr(1) = -0.05 * x_tr(0) + 0.8 * x_tr(1) + 0.1 * u_tr(0);
            sindy.addSnapshot(x_tr, u_tr, (xn_tr - x_tr) / Ts);
            x_tr = xn_tr;
        }
        ctrl::SINDyModel sindy_model = sindy.fit();

        Eigen::VectorXd x_b(2), u_b(1);
        x_b << 0.1, 0.05;  u_b << 0.5;
        record(bench("SINDy predict (n=2, PolyDeg2)", STEPS, WARMUP,
                     [&](long long) {
                         return sindy_model.predict(x_b, u_b)(0);
                     }));
    }

    // KoopmanEDMD lift (PolyDeg2, n=2, m=1 - per-step lift() cost; fit is offline)
    {
        ctrl::KoopmanEDMD::Params kp;
        kp.n_state = 2; kp.n_input = 1;
        kp.dict = ctrl::KoopmanEDMD::Dict::PolyDeg2;
        ctrl::KoopmanEDMD edmd(kp);

        Eigen::VectorXd x_tr(2), xn_tr(2), u_tr(1);
        x_tr << 0.5, 0.1;
        for (int k = 0; k < 300; ++k) {
            u_tr(0) = 0.5 * std::sin(0.2 * static_cast<double>(k));
            xn_tr(0) = 0.9 * x_tr(0) + 0.1 * x_tr(1) + 0.01 * u_tr(0);
            xn_tr(1) = -0.05 * x_tr(0) + 0.8 * x_tr(1) + 0.1 * u_tr(0);
            edmd.addSnapshot(x_tr, u_tr, xn_tr);
            x_tr = xn_tr;
        }

        Eigen::VectorXd x_b(2), u_b(1);
        x_b << 0.1, 0.05;  u_b << 0.5;
        record(bench("KoopmanEDMD lift (n=2, PolyDeg2)", STEPS, WARMUP,
                     [&](long long) {
                         return edmd.lift(x_b, u_b)(0);
                     }));
    }

    std::cout << "\n";

    // =======================================================================
    // Summary - ranked fastest to slowest
    // =======================================================================
    std::sort(all_results.begin(), all_results.end(),
              [](const BenchResult &a, const BenchResult &b)
              { return a.avg_ns < b.avg_ns; });

    std::cout << "== Summary (fastest to slowest) ======================================="
                 "================\n";
    printHeader();
    for (const auto &r : all_results)
        printRow(r);

    std::cout << "\nDone.\n";
    return 0;
}
