// ex136_autoscheduling_pipeline.cpp - Data to deployable C: the whole auto-scheduling pipeline.
//
// Fusion: ctrl::identifyLPV + ctrl::nuGapMatrix/clusterByGap + ctrl::GainScheduledController
//         + ctrl::buildAutoGainScheduler + ctrl::generateControllerC
//
// Every stage of this chain already had an example - ex61 (LPV ID), ex60 (gap clustering),
// ex62 (auto gain scheduler), ex120 (code generation) - but each showed exactly one stage in
// isolation, and tests/test_autoscheduling.cpp exercises the pieces without ever running them
// end to end. Nothing demonstrated the workflow an engineer actually wants: measured data in,
// compilable MCU code out.
//
// The demo runs BOTH front ends onto the same back end:
//
//   Part 1  DATA-DRIVEN front end (exogenous scheduling parameter)
//           logged (u, y, p) -> identifyLPV -> frozen models on a p-grid -> nuGapMatrix
//           -> clusterByGap -> one PID per cluster -> GainScheduledController
//
//   Part 2  MODEL-BASED front end (state-dependent operating point)
//           nonlinear ODE -> buildAutoGainScheduler (equilibrium sweep + linearise + the SAME
//           clustering internally) -> GainScheduledController
//
//   Part 3  SHARED back end
//           generateControllerC() on each cluster representative -> flat C99
//
// Three things this fusion establishes that no single-stage example could:
//
//   1. The two front ends are NOT interchangeable, and the backlog entry that chained them
//      (identifyLPV -> clusterByGap -> buildAutoGainScheduler) cannot be built as written.
//      buildAutoGainScheduler() takes a CONTINUOUS-TIME StateFunc f(x, u) and derives its own
//      operating points by solving f(x_eq, u_eq) = 0 - the scheduling variable is a property of
//      the equilibrium. An LPVModel is DISCRETE-time and scheduled by an EXOGENOUS signal p that
//      never appears in f at all. Feeding one to the other is a category error, not a missing
//      overload. Both legitimately end at a GainScheduledController; they just start from
//      different information. Part 1 therefore reimplements the clustering steps that
//      buildAutoGainScheduler performs internally - that duplication is the finding, not an
//      oversight.
//
//   2. identifyLPVFromIO() is the wrong call here even though it needs less data. It runs n4sid
//      internally, which returns a state sequence in an ARBITRARY basis. nu-gap is
//      basis-invariant so the clustering would still be correct, but the per-cluster PID design
//      below reads physical (a, b) straight out of the frozen model - and in a similarity-
//      transformed basis those numbers mean nothing. Use identifyLPV() with a measured state
//      whenever the state IS the sensed quantity (level, temperature, position), as here.
//
//   3. generateControllerC() emits ONE controller per translation unit: the include guard is
//      always CONTROLLER_GEN_H and the reset function is always controller_reset(). A scheduled
//      family of N clusters therefore needs N separately-compiled units - only the step function
//      name is configurable. Part 3 asserts this rather than papering over it.

#include <ControllerToolbox.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kTs = 0.02;

// -- The true plant: a first-order process whose speed and gain vary with an -------------
// -- exogenous operating point p in [0, 1] (throughput, valve position, load ...) --------
//
//    x[k+1] = a(p) x[k] + b(p) u[k],   y = x
//    a(p) = kA0 + kA1 p     0.990 (tau ~ 2.0 s)  ->  0.700 (tau ~ 0.056 s)
//    b(p) = kB0 + kB1 p     DC gain 1.00         ->  4.00
//
// BOTH must vary for this demo to mean anything. An earlier revision varied only the
// bandwidth (35x) and left the DC gain nearly flat at 1.0 -> 1.5; a single mid-envelope PI
// then performed within 6 % of the full schedule, because what destabilises or detunes a PI
// loop is the GAIN it sees, not how fast the plant is. The demo passed its clustering
// assertions while proving nothing about whether scheduling was worth building. A 4x DC-gain
// spread on top of the bandwidth spread is what makes one fixed controller genuinely wrong.
constexpr double kA0 = 0.990, kA1 = -0.290;
constexpr double kB0 = 0.010, kB1 = 1.190;

double truthA(double p) { return kA0 + kA1 * p; }
double truthB(double p) { return kB0 + kB1 * p; }

// -- Stage 4 design rule: IMC-PI from a frozen first-order discrete model ----------------
// tau = -Ts/ln(a), K = b/(1-a), closed-loop time constant lambda = 0.4 tau (floored at
// 5 Ts so the fast end of the envelope does not ask for a deadbeat controller).
//   Kp = tau/(K lambda),  Ki = Kp/tau = 1/(K lambda)
ctrl::PIDParams designPI(double a, double b)
{
    const double a_c  = std::clamp(a, 1e-6, 0.999999);
    const double tau  = -kTs / std::log(a_c);
    const double K    = b / (1.0 - a_c);
    const double lam  = std::max(0.4 * tau, 5.0 * kTs);

    ctrl::PIDParams p;
    p.Kp = tau / (K * lam);
    p.Ki = 1.0 / (K * lam);
    p.Kd = 0.0;
    p.N  = 50.0;
    p.uMin = -5.0; p.uMax = 5.0; p.Kb = 1.0;
    return p;
}

ctrl::PIDParams designPIFrom(const ctrl::StateSpace &sys)
{
    return designPI(sys.A(0, 0), sys.B(0, 0));
}

// ---------------------------------------------------------------------------------------
// Part 1, stage 1 - commissioning data
// ---------------------------------------------------------------------------------------
struct CommissioningLog {
    Eigen::MatrixXd X, U, Y;
    std::vector<double> sched;
};

// The regressor identifyLPV() builds is phi = [x, p*x, u, p*u]. It is well conditioned only
// when u moves FAST while p moves SLOWLY - a plant excited at the same rate its operating
// point drifts cannot separate "the gain changed" from "the input changed". Hence a slow
// triangular p sweep under a three-tone input.
CommissioningLog collectData(int N)
{
    CommissioningLog log;
    log.X.resize(1, N); log.U.resize(1, N); log.Y.resize(1, N);
    log.sched.resize(N);

    std::mt19937 rng(20260802u);                       // fixed: run.py must be reproducible
    std::normal_distribution<double> meas_noise(0.0, 1e-3);

    double x = 0.0;
    for (int k = 0; k < N; ++k) {
        const double frac = static_cast<double>(k) / (N - 1);
        const double p    = 1.0 - std::abs(2.0 * frac - 1.0);       // 0 -> 1 -> 0 triangle
        const double u    = 0.8 * std::sin(2.0 * M_PI * 0.037 * k)
                          + 0.6 * std::sin(2.0 * M_PI * 0.011 * k + 1.1)
                          + 0.5 * std::sin(2.0 * M_PI * 0.089 * k + 2.3);

        const double x_meas = x + meas_noise(rng);
        log.X(0, k)  = x_meas;
        log.Y(0, k)  = x_meas;
        log.U(0, k)  = u;
        log.sched[k] = p;

        x = truthA(p) * x + truthB(p) * u;
    }
    return log;
}

// ---------------------------------------------------------------------------------------
// Part 1, stage 6 - closed-loop validation on the TRUE plant
// ---------------------------------------------------------------------------------------
struct ArmResult {
    double iae      = 0.0;
    double max_dev  = 0.0;   // worst |e| once past the first transient
    double tv       = 0.0;   // total command variation - the price of the tracking
    bool   finite   = true;
};

// Both arms are driven by the identical p profile and setpoint train. `sched` is null for the
// fixed-gain arm, which is the entire point of the comparison.
ArmResult runClosedLoop(const std::shared_ptr<ctrl::IController> &ctl,
                        ctrl::GainScheduledController *sched)
{
    constexpr int kN = 6000;                     // 120 s
    ArmResult r;
    double x = 0.0, u_prev = 0.0;

    for (int k = 0; k < kN; ++k) {
        const double t = k * kTs;

        // Operating point walks the full envelope: 0 -> 1 -> 0 over the run.
        const double frac = static_cast<double>(k) / (kN - 1);
        const double p    = std::clamp(1.0 - std::abs(2.0 * frac - 1.0), 0.0, 1.0);

        // Setpoint train, deliberately out of phase with the p sweep so every step lands at a
        // different operating point.
        const double ref = (std::fmod(t, 20.0) < 10.0) ? 1.0 : 0.4;

        if (sched) sched->setSchedulingParam(p);
        const double u = ctl->compute(ref - x);

        x = truthA(p) * x + truthB(p) * u;

        if (!std::isfinite(x) || !std::isfinite(u)) { r.finite = false; return r; }

        r.iae += std::abs(ref - x) * kTs;
        r.tv  += std::abs(u - u_prev);
        if (t > 1.0) r.max_dev = std::max(r.max_dev, std::abs(ref - x));
        u_prev = u;
    }
    return r;
}

// ---------------------------------------------------------------------------------------
// Part 2 - the model-based front end
// ---------------------------------------------------------------------------------------
// Gravity-drained tank: A dh/dt = u - c sqrt(h). The operating point IS the state, so the
// scheduling variable falls out of the equilibrium sweep - exactly the shape
// buildAutoGainScheduler() expects, and exactly what an LPVModel cannot supply.
constexpr double kTankArea = 1.0;
constexpr double kTankC    = 0.6;

ctrl::StateFunc tankDynamics()
{
    return [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xd(1);
        const double h = std::max(x(0), 1e-9);        // sqrt guard; never let Newton go negative
        xd(0) = (u(0) - kTankC * std::sqrt(h)) / kTankArea;
        return xd;
    };
}

}  // namespace

int main()
{
    std::printf("=== ex136: auto-scheduling pipeline, measured data -> deployable C ===\n\n");

    bool ok = true;

    // ===================================================================================
    // PART 1 - data-driven front end
    // ===================================================================================
    std::printf("-- Part 1: data-driven front end (exogenous scheduling parameter) --\n\n");
    std::printf("true plant : x[k+1] = a(p) x[k] + b(p) u[k],  p in [0, 1]\n");
    std::printf("             a(p) = %.3f %+.3f p   (tau %.2f s -> %.3f s)\n",
                kA0, kA1, -kTs / std::log(truthA(0.0)), -kTs / std::log(truthA(1.0)));
    std::printf("             b(p) = %.3f %+.3f p   (DC gain %.2f -> %.2f)\n\n",
                kB0, kB1,
                truthB(0.0) / (1.0 - truthA(0.0)), truthB(1.0) / (1.0 - truthA(1.0)));

    // ---- stage 1: commissioning data -------------------------------------------------
    constexpr int kNData = 4000;
    const CommissioningLog log = collectData(kNData);
    std::printf("  [1] logged %d samples (Ts = %.3f s, %.0f s of operation),"
                " 1 mm measurement noise\n", kNData, kTs, kNData * kTs);

    // ---- stage 2: identify the LPV model ----------------------------------------------
    const ctrl::LPVModel model = ctrl::identifyLPV(log.X, log.U, log.Y, log.sched, 1, kTs);

    const double a0_hat = model.A_coeffs[0](0, 0), a1_hat = model.A_coeffs[1](0, 0);
    const double b0_hat = model.B_coeffs[0](0, 0), b1_hat = model.B_coeffs[1](0, 0);
    const double id_err = std::max(std::max(std::abs(a0_hat - kA0), std::abs(a1_hat - kA1)),
                                   std::max(std::abs(b0_hat - kB0), std::abs(b1_hat - kB1)));

    std::printf("  [2] identifyLPV (degree 1):  a = %.4f %+.4f p   b = %.4f %+.4f p\n",
                a0_hat, a1_hat, b0_hat, b1_hat);
    std::printf("      truth                 :  a = %.4f %+.4f p   b = %.4f %+.4f p"
                "   (worst coeff error %.2e)\n\n", kA0, kA1, kB0, kB1, id_err);

    // ---- stage 3: freeze on a grid ----------------------------------------------------
    constexpr int kGrid = 11;
    std::vector<ctrl::StateSpace> models;
    std::vector<double> p_grid(kGrid);
    models.reserve(kGrid);
    for (int i = 0; i < kGrid; ++i) {
        p_grid[i] = static_cast<double>(i) / (kGrid - 1);
        models.push_back(model.frozen(p_grid[i]));
    }
    std::printf("  [3] froze the identified model at %d operating points\n", kGrid);

    // ---- stage 4: nu-gap clustering, and where it falls down ---------------------------
    const Eigen::MatrixXd G  = ctrl::nuGapMatrix(models, 200);
    const double suggested   = ctrl::suggestGapThreshold(G);
    constexpr double kGapThr = 0.35;                  // fixed, so run.py is deterministic
    const ctrl::ClusterResult cl = ctrl::clusterByGap(G, kGapThr);

    std::printf("  [4] nu-gap neighbour profile (p_i to p_i+1):\n        ");
    for (int i = 0; i + 1 < kGrid; ++i) std::printf("%.3f ", G(i, i + 1));
    std::printf("\n      end-to-end gap G(p=0, p=1) = %.3f\n", G(0, kGrid - 1));
    std::printf("      clusterByGap(threshold = %.2f) -> %d clusters from %d models"
                "  (suggestGapThreshold said %.2f)\n", kGapThr, cl.numClusters, kGrid, suggested);

    double worst_intra = 0.0;
    for (int c = 0; c < cl.numClusters; ++c) {
        worst_intra = std::max(worst_intra, cl.maxIntraGap[c]);
        std::printf("        cluster %d: rep at p = %.2f, max intra-cluster gap %.3f%s\n",
                    c, p_grid[cl.representatives[c]], cl.maxIntraGap[c],
                    cl.maxIntraGap[c] >= kGapThr ? "   <-- EXCEEDS THE THRESHOLD" : "");
    }

    // FINDING. clusterByGap() is SINGLE-LINKAGE: it merges i and j whenever gap(i,j) <
    // threshold, so membership propagates transitively and a cluster's DIAMETER is unbounded.
    // LinearModelCluster.h states the opposite - "all models within a cluster are within
    // `threshold` nu-gap distance of each other" - and rests Vinnicombe's robust-stability
    // argument on it: a controller stabilising the representative is claimed to stabilise
    // every member. On a smoothly-varying 1-D family that claim fails by construction, and
    // the number printed above is the counterexample. It gets worse as the grid gets denser,
    // because a finer grid gives the chain more places to link.
    const bool chaining_observed = worst_intra >= kGapThr;

    // ---- stage 5: two ways to place schedule breakpoints --------------------------------
    // (a) cluster representatives - what the shipped pipeline hands you.
    // (b) greedy gap-COVERING - walk the grid and open a new breakpoint as soon as the gap
    //     from the current anchor reaches the threshold. Every model is then within the
    //     threshold of the breakpoint it is actually served by, which is the property the
    //     robust-stability argument needs and the property single-linkage does not give.
    std::vector<int> cluster_reps(cl.representatives.begin(), cl.representatives.end());
    std::sort(cluster_reps.begin(), cluster_reps.end());

    std::vector<int> covering;
    {
        int anchor = 0;
        covering.push_back(0);
        for (int i = 1; i < kGrid; ++i)
            if (G(anchor, i) >= kGapThr) { covering.push_back(i); anchor = i; }
    }

    // Coverage check: assign each model to its nearest breakpoint by nu-gap and take the worst.
    auto worstCoverage = [&](const std::vector<int> &bps) {
        double worst = 0.0;
        for (int i = 0; i < kGrid; ++i) {
            double best = 1e9;
            for (int b : bps) best = std::min(best, G(i, b));
            worst = std::max(worst, best);
        }
        return worst;
    };
    const double cov_cluster  = worstCoverage(cluster_reps);
    const double cov_covering = worstCoverage(covering);

    std::printf("\n  [5] breakpoint placement:\n");
    std::printf("        cluster representatives : %zu points at p =", cluster_reps.size());
    for (int i : cluster_reps) std::printf(" %.2f", p_grid[i]);
    std::printf("   worst model-to-breakpoint gap %.3f\n", cov_cluster);
    std::printf("        greedy gap-covering     : %zu points at p =", covering.size());
    for (int i : covering) std::printf(" %.2f", p_grid[i]);
    std::printf("   worst model-to-breakpoint gap %.3f\n", cov_covering);

    auto buildScheduler = [&](const std::vector<int> &bps, ctrl::GainScheduleMode mode,
                              std::vector<ctrl::PIDParams> *sink) {
        auto s = std::make_shared<ctrl::GainScheduledController>(kTs, mode);
        for (int i : bps) {
            const ctrl::PIDParams pp = designPIFrom(models[i]);
            s->addSchedulePoint(p_grid[i], std::make_shared<ctrl::DiscretePID>(pp, kTs));
            if (sink) sink->push_back(pp);
        }
        return s;
    };

    std::vector<ctrl::PIDParams> cluster_gains;
    std::vector<double>          cluster_p;
    for (int i : covering) cluster_p.push_back(p_grid[i]);

    auto sched_cluster = buildScheduler(cluster_reps, ctrl::GainScheduleMode::LinearBlend, nullptr);
    auto sched_blend   = buildScheduler(covering,     ctrl::GainScheduleMode::LinearBlend, nullptr);
    auto sched_nearest = buildScheduler(covering,     ctrl::GainScheduleMode::NearestNeighbor,
                                        &cluster_gains);

    std::printf("        per-breakpoint IMC-PI (lambda = 0.4 tau, floored at 5 Ts):");
    for (size_t i = 0; i < cluster_gains.size(); ++i)
        std::printf("%s          p = %.2f -> Kp %6.3f  Ki %6.3f",
                    i == 0 ? "\n" : "\n", cluster_p[i], cluster_gains[i].Kp, cluster_gains[i].Ki);
    std::printf("\n");

    // ---- stage 6: paired closed-loop validation ----------------------------------------
    // Baseline is the honest alternative an engineer would otherwise ship: ONE controller,
    // tuned at the middle of the envelope, used everywhere. Every arm sees the identical p
    // sweep and setpoint train.
    const ctrl::PIDParams mid_gains = designPI(truthA(0.5), truthB(0.5));
    auto fixed_pid = std::make_shared<ctrl::DiscretePID>(mid_gains, kTs);

    const ArmResult fixed   = runClosedLoop(fixed_pid,     nullptr);
    const ArmResult chained = runClosedLoop(sched_cluster, sched_cluster.get());
    const ArmResult blend   = runClosedLoop(sched_blend,   sched_blend.get());
    const ArmResult gs      = runClosedLoop(sched_nearest, sched_nearest.get());

    std::printf("\n  [6] closed loop on the TRUE plant, identical p sweep and setpoint train:\n");
    std::printf("      %-40s %10s %12s %10s\n", "arm", "IAE", "worst |e|", "TV(u)");
    std::printf("      %-40s %10.4f %12.4f %10.2f\n",
                "fixed PID, tuned at p = 0.50", fixed.iae, fixed.max_dev, fixed.tv);
    std::printf("      %-40s %10.4f %12.4f %10.2f\n",
                "clusterByGap reps + LinearBlend", chained.iae, chained.max_dev, chained.tv);
    std::printf("      %-40s %10.4f %12.4f %10.2f\n",
                "gap-covering + LinearBlend", blend.iae, blend.max_dev, blend.tv);
    std::printf("      %-40s %10.4f %12.4f %10.2f\n",
                "gap-covering + NearestNeighbor", gs.iae, gs.max_dev, gs.tv);
    std::printf("\n      Mode note, measured rather than assumed. GainScheduledController.h warns\n"
                "      that LinearBlend advances EVERY bracketing controller's integrator each\n"
                "      tick, which suggests NearestNeighbor for PI controllers - and that is\n"
                "      wrong here. Hard switching re-seeds an integrator at each of the ~6\n"
                "      crossings and each re-seed costs a transient, while gap-covering already\n"
                "      places the breakpoints close enough to apply the header's own mitigation.\n"
                "      Smoothness wins, so buildAutoGainScheduler()'s hardcoded LinearBlend is\n"
                "      the right default after all.\n");

    // ===================================================================================
    // PART 2 - model-based front end
    // ===================================================================================
    std::printf("\n-- Part 2: model-based front end (state-dependent operating point) --\n\n");
    std::printf("nonlinear plant : A dh/dt = u - c sqrt(h)   (A = %.1f, c = %.1f)\n",
                kTankArea, kTankC);
    std::printf("scheduling var  : the level h itself - derived from the equilibrium sweep,\n"
                "                  which is why an exogenously-scheduled LPVModel cannot"
                " drive this path\n\n");

    // design_fn is called once per cluster, but addSchedulePoint() then sorts the schedule by p.
    // Record the operating point alongside the gains and re-sort, or the reported gains pair
    // with the wrong level whenever cluster order and p order differ.
    std::vector<std::pair<double, ctrl::PIDParams>> tank_design;
    auto design_fn = [&tank_design](const ctrl::StateSpace &sys, double p)
            -> std::shared_ptr<ctrl::IController> {
        const ctrl::PIDParams pp = designPIFrom(sys);
        tank_design.emplace_back(p, pp);            // recorded so Part 3 can emit C for it
        return std::make_shared<ctrl::DiscretePID>(pp, kTs);
    };

    constexpr int    kTankGrid = 9;
    constexpr double kHLo = 0.05, kHHi = 4.00;
    const auto u_eq_fn = [](double p) {
        Eigen::VectorXd u(1); u(0) = kTankC * std::sqrt(p); return u; };
    const auto x0_fn   = [](double p) {
        Eigen::VectorXd x(1); x(0) = p; return x; };

    auto tank_sched = ctrl::buildAutoGainScheduler(
        tankDynamics(), kHLo, kHHi, kTankGrid, u_eq_fn, x0_fn,
        design_fn, kTs, kGapThr, 200);

    std::sort(tank_design.begin(), tank_design.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });

    std::printf("  buildAutoGainScheduler swept %d levels in [%.2f, %.2f] and returned"
                " %d schedule point(s)\n", kTankGrid, kHLo, kHHi, tank_sched->numPoints());
    for (const auto &[h, pp] : tank_design)
        std::printf("        h = %.2f m  ->  Kp = %6.3f   Ki = %6.3f\n", h, pp.Kp, pp.Ki);

    // Replicate the equilibrium sweep buildAutoGainScheduler runs internally, so the same
    // gap-covering yardstick from Part 1 can be applied to its output. Both helpers are public.
    std::vector<ctrl::StateSpace> tank_models;
    tank_models.reserve(kTankGrid);
    for (int i = 0; i < kTankGrid; ++i) {
        const double h = kHLo + (kHHi - kHLo) * i / (kTankGrid - 1);
        const Eigen::VectorXd u_eq = u_eq_fn(h);
        const Eigen::VectorXd x_eq = ctrl::findEquilibrium(tankDynamics(), u_eq, x0_fn(h));
        tank_models.push_back(ctrl::lineariseAtPoint(tankDynamics(), x_eq, u_eq, kTs));
    }
    const Eigen::MatrixXd Gt = ctrl::nuGapMatrix(tank_models, 200);
    int tank_cov = 1;
    {
        int anchor = 0;
        for (int i = 1; i < kTankGrid; ++i)
            if (Gt(anchor, i) >= kGapThr) { ++tank_cov; anchor = i; }
    }
    std::printf("      end-to-end nu-gap across the level range : %.3f\n", Gt(0, kTankGrid - 1));
    std::printf("      gap-covering would need %d breakpoint(s) at the same %.2f threshold%s\n",
                tank_cov, kGapThr,
                tank_sched->numPoints() < tank_cov
                    ? "  <-- the same chaining defect, inherited" : "");

    bool tank_finite = true;
    for (double h = kHLo; h <= kHHi + 1e-9; h += 0.25) {
        tank_sched->setSchedulingParam(h);
        if (!std::isfinite(tank_sched->compute(0.1))) tank_finite = false;
    }

    // ===================================================================================
    // PART 3 - the shared back end
    // ===================================================================================
    std::printf("\n-- Part 3: shared back end - generateControllerC() on every representative --\n\n");

    int    emitted     = 0;
    size_t total_bytes = 0;
    bool   codegen_ok  = true;
    std::string sample_source;

    for (size_t c = 0; c < cluster_gains.size(); ++c) {
        ctrl::CodeGenParams cfg;
        // Only the STEP function name is configurable. controller_reset() and the
        // CONTROLLER_GEN_H include guard are fixed, so these units cannot share a build -
        // one cluster per separately-compiled translation unit is the deployment shape.
        cfg.function_name = "pid_step_cluster" + std::to_string(c);

        const ctrl::GeneratedCode code = ctrl::generateControllerC(cluster_gains[c], kTs, cfg);

        const bool has_step  = code.source.find(cfg.function_name) != std::string::npos;
        const bool has_reset = code.source.find("controller_reset") != std::string::npos;
        const bool has_guard = code.header.find("CONTROLLER_GEN_H") != std::string::npos;
        const bool no_deps   = code.source.find("Eigen")  == std::string::npos &&
                               code.source.find("#include <std") == std::string::npos;

        if (!has_step || !has_reset || !has_guard || !no_deps ||
            code.header.empty() || code.source.empty())
            codegen_ok = false;

        ++emitted;
        total_bytes += code.header.size() + code.source.size();
        if (c == 0) sample_source = code.source;

        std::printf("  cluster %zu (p = %.2f)  ->  %s()  %4zu B header + %4zu B source\n",
                    c, cluster_p[c], cfg.function_name.c_str(),
                    code.header.size(), code.source.size());
    }

    // Also emit the model-based path's controllers, proving both front ends land on the
    // same generator with no special-casing.
    for (size_t i = 0; i < tank_design.size(); ++i) {
        ctrl::CodeGenParams cfg;
        cfg.function_name = "tank_step_level" + std::to_string(i);
        const ctrl::GeneratedCode code =
            ctrl::generateControllerC(tank_design[i].second, kTs, cfg);
        if (code.source.find(cfg.function_name) == std::string::npos) codegen_ok = false;
        ++emitted;
        total_bytes += code.header.size() + code.source.size();
    }
    std::printf("  + %zu more from the model-based path (same generator, no special-casing)\n",
                tank_design.size());

    std::printf("\n  emitted %d flat-C99 units, %zu bytes total. First unit, gains baked in:\n\n",
                emitted, total_bytes);
    {   // print the head of one generated source so the artefact is visible, not just counted
        int lines = 0;
        size_t pos = 0;
        while (lines < 14 && pos < sample_source.size()) {
            const size_t nl = sample_source.find('\n', pos);
            const size_t end = (nl == std::string::npos) ? sample_source.size() : nl;
            std::printf("      | %s\n", sample_source.substr(pos, end - pos).c_str());
            if (nl == std::string::npos) break;
            pos = nl + 1;
            ++lines;
        }
        std::printf("      | ... (%zu bytes total)\n", sample_source.size());
    }

    // ===================================================================================
    // Acceptance
    // ===================================================================================
    // The pipeline is only worth anything if every stage did real work: the ID has to be
    // accurate, the clustering has to actually COMPRESS (a "cluster" per grid point is just
    // the grid back again, and one cluster for everything means the envelope never needed
    // scheduling), the schedule has to beat the fixed alternative by a margin that is not
    // noise, and the generator has to produce something a compiler would accept.
    const bool id_accurate  = id_err < 5e-3;
    const bool compressed   = static_cast<int>(covering.size()) > 1 &&
                              static_cast<int>(covering.size()) < kGrid;
    const bool covered      = cov_covering < kGapThr;
    const bool arms_finite  = fixed.finite && chained.finite && blend.finite && gs.finite;
    // `blend` (gap-covering + LinearBlend) is the arm under test - chosen on the design
    // argument above, not by picking the best number after the fact.
    const bool worth_it     = arms_finite && blend.iae < 0.70 * fixed.iae;
    const bool fixes_matter = arms_finite && blend.iae < chained.iae;
    const bool tank_ran     = tank_finite && tank_sched->numPoints() >= 1 && tank_cov >= 2;
    const bool emitted_all  = codegen_ok &&
                              emitted == static_cast<int>(cluster_gains.size() + tank_design.size());

    std::printf("\n  LPV ID accurate        = %s (worst coefficient error %.2e < 5e-3)\n",
                id_accurate ? "yes" : "no", id_err);
    std::printf("  schedule compresses    = %s (%zu breakpoints cover %d grid models)\n",
                compressed ? "yes" : "no", covering.size(), kGrid);
    std::printf("  every model covered    = %s (worst model-to-breakpoint gap %.3f < %.2f;\n"
                "                                clusterByGap's reps leave %.3f)\n",
                covered ? "yes" : "no", cov_covering, kGapThr, cov_cluster);
    std::printf("  schedule earns itself  = %s (IAE %.4f < 70%% of the fixed arm's %.4f)\n",
                worth_it ? "yes" : "no", blend.iae, fixed.iae);
    std::printf("  gap-covering matters   = %s (IAE %.4f < clusterByGap reps' %.4f;"
                " NearestNeighbor measured %.4f)\n",
                fixes_matter ? "yes" : "no", blend.iae, chained.iae, gs.iae);
    std::printf("  model-based front end  = %s (%d point(s) returned, coverage needs %d,"
                " finite across range)\n", tank_ran ? "yes" : "no",
                tank_sched->numPoints(), tank_cov);
    std::printf("  deployable C emitted   = %s (%d units, guard + reset + step all present,"
                " no external deps)\n", emitted_all ? "yes" : "no", emitted);
    std::printf("  single-linkage chaining= %s (worst intra-cluster gap %.3f vs threshold %.2f)"
                "\n                           reported as a finding, not asserted against\n",
                chaining_observed ? "OBSERVED" : "not seen", worst_intra, kGapThr);

    ok = id_accurate && compressed && covered && worth_it && fixes_matter &&
         tank_ran && emitted_all;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
