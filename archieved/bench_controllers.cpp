// bench_controllers.cpp
// Measures per-step compute() wall-clock latency for every controller.
//
// Build: included automatically via CMake (target: bench_controllers).
// Run:   build/benchmarks/bench_controllers[.exe]
//
// Output (example):
//   [DiscretePID       ]  1 000 000 steps  |  avg  123 ns  |  min   98 ns  |  max  450 ns
//   [DiscreteLeadLag   ]  1 000 000 steps  |  avg  145 ns  |  ...
//
// Method: hot-loop timing with steady_clock to prevent OS scheduler noise.
// The first 10% of iterations are discarded as warm-up.
#include "ControllerToolbox.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{

    // -- helpers ----------------------------------------------------------------

    using Clock = std::chrono::steady_clock;
    using Ns = std::chrono::nanoseconds;

    struct BenchResult
    {
        std::string name;
        long long avg_ns;
        long long min_ns;
        long long max_ns;
        int steps;
    };

    void printResult(const BenchResult &r)
    {
        std::cout << "  [" << std::left << std::setw(22) << r.name << "]"
                  << "  " << std::right << std::setw(9) << r.steps << " steps"
                  << "  |  avg " << std::setw(6) << r.avg_ns << " ns"
                  << "  |  min " << std::setw(6) << r.min_ns << " ns"
                  << "  |  max " << std::setw(6) << r.max_ns << " ns\n";
    }

    // Time a lambda that takes (step_index) and returns a double.
    // warmup_frac of total_steps are discarded before timing starts.
    template <typename Fn>
    BenchResult bench(const std::string &name, int total_steps, double warmup_frac, Fn fn)
    {
        const int warmup = static_cast<int>(total_steps * warmup_frac);
        for (int k = 0; k < warmup; ++k)
            fn(k);

        long long sum = 0;
        long long lo = std::numeric_limits<long long>::max();
        long long hi = 0;
        const int timed = total_steps - warmup;

        for (int k = 0; k < timed; ++k)
        {
            auto t0 = Clock::now();
            volatile double u = fn(k); // volatile prevents the call being optimised away
            auto t1 = Clock::now();
            (void)u;

            long long dt = std::chrono::duration_cast<Ns>(t1 - t0).count();
            sum += dt;
            if (dt < lo)
                lo = dt;
            if (dt > hi)
                hi = dt;
        }

        return {name, sum / timed, lo, hi, timed};
    }

    // -- build a generic n-state SISO stable plant for testing ------------------
    // Chain of integrators with unit damping coefficients so the plant is stable.
    ctrl::StateSpace makePlant(int n, double Ts = 0.01)
    {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
        for (int i = 0; i < n - 1; ++i)
            A(i, i + 1) = 1.0;
        for (int i = 0; i < n; ++i)
            A(i, i) -= 1.0; // stable diagonal

        Eigen::MatrixXd B(n, 1);
        B.setZero();
        B(n - 1, 0) = 1.0;
        Eigen::MatrixXd C(1, n);
        C.setZero();
        C(0, 0) = 1.0;
        Eigen::MatrixXd D(1, 1);
        D.setZero();

        return ctrl::StateSpace(A, B, C, D, Ts);
    }

} // anonymous namespace

// -- main -------------------------------------------------------------------

int main()
{
    const int STEPS = 1'000'000;
    const double Ts = 0.01;

    std::cout << "====================================================================\n"
              << "  Controller Toolbox - Per-Step Latency Benchmark\n"
              << "  Steps: " << STEPS << "  (10 % warm-up discarded)\n"
              << "  Platform: " << sizeof(void *) * 8 << "-bit\n"
              << "====================================================================\n\n";

    for (int n : {2, 4, 8})
    {
        std::cout << "-- State dimension n = " << n
                  << " ----------------------------------------\n";

        ctrl::StateSpace sys = makePlant(n, Ts);

        // DiscretePID (independent of n - always SISO)
        if (n == 2)
        {
            ctrl::PIDParams pp;
            pp.Kp = 1.0;
            pp.Ki = 0.1;
            pp.Kd = 0.01;
            pp.N = 10.0;
            ctrl::DiscretePID pid(pp, Ts);
            printResult(bench("DiscretePID", STEPS, 0.1,
                              [&](int)
                              { return pid.compute(1.0); }));

            ctrl::LeadLagParams lp;
            lp.gain = 2.0;
            lp.continuousZero = 1.0;
            lp.continuousPole = 10.0;
            ctrl::DiscreteLeadLag ll(lp, Ts);
            printResult(bench("DiscreteLeadLag", STEPS, 0.1,
                              [&](int)
                              { return ll.compute(1.0); }));

            ctrl::SMCParams sp;
            sp.K = 5.0;
            sp.phi = 0.5;
            ctrl::DiscreteSMC smc(sp, Ts);
            printResult(bench("DiscreteSMC", STEPS, 0.1,
                              [&](int)
                              { return smc.compute(1.0); }));

            ctrl::ADRCParams ap;
            ap.omega_o = 10.0;
            ap.omega_c = 3.0;
            ap.b0 = 1.0;
            ctrl::DiscreteADRC adrc(ap, Ts);
            printResult(bench("DiscreteADRC", STEPS, 0.1,
                              [&](int k)
                              { return adrc.computeTracking(0.0, 1.0); }));

            ctrl::ExtremumSeekerParams ep;
            ep.perturbAmp = 0.05;
            ep.perturbFreq = 5.0;
            ep.lpfCutoff = 1.0;
            ep.hpfCutoff = 0.5;
            ep.integGain = 0.1;
            ctrl::ExtremumSeeker esc(ep, Ts);
            printResult(bench("ExtremumSeeker", STEPS, 0.1,
                              [&](int k)
                              {
                                  double theta = esc.currentEstimate();
                                  double cost = (theta - 2.0) * (theta - 2.0); // quadratic cost
                                  return esc.compute(cost);
                              }));
        }

        // DiscreteLQR
        {
            ctrl::LQRParams lp;
            lp.Q = Eigen::MatrixXd::Identity(n, n);
            lp.R = Eigen::MatrixXd::Identity(1, 1);
            ctrl::DiscreteLQR lqr(sys, lp);
            Eigen::VectorXd x = Eigen::VectorXd::Ones(n) * 0.1;
            printResult(bench("DiscreteLQR (n=" + std::to_string(n) + ")", STEPS, 0.1,
                              [&](int)
                              { return lqr.compute(x)(0); }));
        }

        // KalmanFilter
        {
            ctrl::KalmanFilter kf(sys,
                                  Eigen::MatrixXd::Identity(n, n) * 0.01,
                                  Eigen::MatrixXd::Identity(1, 1) * 0.1,
                                  Eigen::MatrixXd::Identity(n, n));
            Eigen::VectorXd y(1);
            y << 0.5;
            Eigen::VectorXd u(1);
            u << 0.1;
            printResult(bench("KalmanFilter (n=" + std::to_string(n) + ")", STEPS, 0.1,
                              [&](int)
                              {
                                  kf.step(y, u);
                                  return kf.state()(0);
                              }));
        }

        // DiscreteMPC
        {
            ctrl::MPCParams mp;
            mp.Np = 5;
            mp.Nc = 2;
            mp.rho_y = 1.0;
            mp.rho_u = 0.1;
            ctrl::DiscreteMPC mpc(sys, mp);
            printResult(bench("DiscreteMPC (n=" + std::to_string(n) + ")", STEPS / 10, 0.1,
                              [&](int)
                              { return mpc.compute(1.0); }));
        }

        std::cout << "\n";
    }

    // SmithPredictor - delay model is always 2-state
    {
        ctrl::StateSpace sys2 = makePlant(2, Ts);
        ctrl::PIDParams pp;
        pp.Kp = 1.0;
        pp.Ki = 0.1;
        auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);
        ctrl::SmithPredictor sp(pid, sys2, /*delaySteps=*/8);
        printResult(bench("SmithPredictor (d=8)", STEPS, 0.1,
                          [&](int)
                          { return sp.compute(1.0); }));
    }

    std::cout << "\nDone.\n";
    return 0;
}
