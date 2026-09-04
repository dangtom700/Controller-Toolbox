// ex140_plc_sensor_voting.cpp - Fault-tolerant sensor voting over three redundant links.
//
// Fusion: ctrl::UnscentedKalmanFilter + ctrl::FTCSupervisor (+ ctrl::ControllerStack)
//         + 3 x ctrl::NetworkChannel                      [fusion backlog item A2]
//
// Architecture:
//
//     plant (tank) : h' = (u - cd*sqrt(h) - q_load)/A     <- nonlinear, hence UKF not KF
//        |  y1 -> NetworkChannel #1   fast,   lossless
//        |  y2 -> NetworkChannel #2   medium, 2 % loss
//        |  y3 -> NetworkChannel #3   slow,  10 % loss
//     voter         : median of the FRESH held values -> y_vote
//     UKF           : filters y_vote; its one-step prediction is the model reference
//     FTCSupervisor : reconfigures the ControllerStack on a classified residual
//     PLC (slave)   : applies u
//
// -- What the A2 backlog row asked for, and why it is not what got built -------------
//
// The row reads "three sensors on independent links; FTCSupervisor::feedResidual() +
// registerFaultResponse() drop a faulted channel". Two API facts make that shape
// unbuildable, and both are worth recording:
//
//   1. UnscentedKalmanFilter fixes its measurement dimension p and its R at
//      CONSTRUCTION and exposes no setter for either. A sensor therefore cannot be
//      dropped *inside* the estimator - there is no way to shrink y or re-weight R at
//      runtime. Voting has to be a pre-filter in front of a p = 1 UKF, not a filter mode.
//
//   2. FTCSupervisor switches CONTROLLERS, not sensors: registerFaultResponse() maps a
//      FaultType onto a ControllerStack entry name. It has no notion of a channel. The
//      exclusion is the voter's job; the supervisor's job is what to do about it.
//
// -- The finding this demo exists to measure ------------------------------------------
//
// Two structurally different residuals are available, and NEITHER alone is sufficient:
//
//   disagreement  d = max_i |y_i - y_vote|   (sensor vs sensor)
//   innovation    r = y_vote - h_model       (consensus vs model)
//
// With ONE faulty sensor the vote is still correct: d is large, r stays in its null
// distribution. Feed the supervisor the innovation and it sees nothing - correctly, since
// nothing needs reconfiguring. With TWO faulty sensors biased the same way the vote
// carries the fault: d is still large (the healthy sensor is now the outlier) but so is r.
// d says "someone is lying"; only r says "the majority is lying". A voted architecture
// that monitors only one of the two is blind to one of the two failures.
//
// -- The trap that cost a rebuild -----------------------------------------------------
//
// Feeding the raw UKF innovation to FTCSupervisor flip-flops. The filter absorbs the bias
// within a few ticks, the residual decays below threshold, the classification returns to
// None, the supervisor switches BACK, the loop re-engages on the lying measurement and the
// residual stays at zero because the estimate now agrees with the lie. The reference must
// stop tracking once a fault is flagged - the same "frozen reference model" rule ex135
// hit from the other direction (docs/fusion_opportunity_backlog.md, lesson 4b).

#include <ControllerToolbox.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>

namespace {

// -- Plant: gravity-drained tank ------------------------------------------------------
constexpr double kA   = 1.5;    // cross-section [m^2]
constexpr double kCd  = 0.6;    // discharge coefficient [m^2.5/s]
constexpr double kTs  = 0.10;   // PLC tick [s]
constexpr double kH0  = 2.0;    // initial level [m]

constexpr double kTsim   = 90.0;
constexpr int    kNSteps = static_cast<int>(kTsim / kTs + 0.5);

// Setpoint moves, slowly and deliberately. FaultClassifier's causality test correlates
// d(u_cmd) against d(y_meas); a regulator parked at steady state gives it nothing to
// correlate and trips its own ActuatorStuck branch on stddev(u) alone.
constexpr double kRefMean   = 2.00;   // [m]
constexpr double kRefAmp    = 0.20;   // [m]
constexpr double kRefPeriod = 60.0;   // [s]

// Load disturbance during the HEALTHY window: proves the loop still needs feedback, so
// that "switch to open loop" cannot be dismissed as open loop simply being better here.
constexpr double kLoadQ     = 0.10;   // extra outflow [m^3/s]
constexpr double kLoadStart = 25.0;
constexpr double kLoadEnd   = 40.0;

// Fault schedule: one sensor, then a second one biased the same way (the case 2-out-of-3
// voting detects but provably cannot resolve).
constexpr double kFault1Time = 45.0;  // sensor 2 -> +0.35 m
constexpr double kFault1Bias = 0.35;
constexpr double kFault2Time = 62.0;  // sensor 3 -> +0.30 m
constexpr double kFault2Bias = 0.30;

constexpr double kSensorSigma = 0.010;  // [m], per sensor
constexpr unsigned kNoiseSeed = 90210u;

// Three genuinely independent links - different quality AND different seeds. Sharing one
// seed would correlate the loss patterns, which is the one thing redundancy assumes away.
constexpr int kNSensors = 3;
constexpr double   kLat[kNSensors]  = {0.020, 0.060, 0.150};
constexpr double   kJit[kNSensors]  = {0.005, 0.015, 0.040};
constexpr double   kLoss[kNSensors] = {0.000, 0.020, 0.100};
constexpr unsigned kSeed[kNSensors] = {101u,  202u,  303u};

constexpr double kMaxAge = 0.45;   // [s] a held sample older than this leaves the vote

// Quiet baseline: after start-up, before the load disturbance. The "healthy" window
// contains the load transient, so it is the wrong thing to compare a fault against.
constexpr double kQuietStart = 10.0;
constexpr double kQuietEnd   = kLoadStart;

// Alarm threshold, sized from BOTH sides against measurement rather than guessed: the
// worst healthy residual is the load-disturbance transient (~0.08 m - the estimator's
// model carries no disturbance term) and the fault-2 residual is ~0.33 m. 0.15 sits about
// 2x clear of each. The first draft used 0.08 and cleared the false-alarm side by 2 %,
// which is not a margin, it is a coincidence waiting to be re-tuned.
constexpr double kResidThreshold = 0.15;   // FaultDetectorParams::residual_threshold

// Window boundaries used for every reported metric.
constexpr double kWinHealthyEnd = kFault1Time;   // [0, 45)   all three sensors good
constexpr double kWinSingleEnd  = kFault2Time;   // [45, 62)  one bad  - vote still right
                                                 // [62, 90)  two bad  - vote is wrong

double refAt(double t)
{
    return kRefMean + kRefAmp * std::sin(2.0 * M_PI * t / kRefPeriod);
}

double refRateAt(double t)
{
    const double w = 2.0 * M_PI / kRefPeriod;
    return kRefAmp * w * std::cos(w * t);
}

double loadAt(double t)
{
    return (t >= kLoadStart && t < kLoadEnd) ? kLoadQ : 0.0;
}

/// Full model inverse of the tank: the inflow that makes h track (ref, dref/dt) exactly.
/// h' = (u - cd*sqrt(h))/A  =>  u = A*dref + cd*sqrt(ref). The rate term matters: with the
/// steady-state part alone the fallback lags a moving setpoint by the plant's own 7 s
/// time constant, which would show up as a fault-tolerance failure that is really just a
/// missing feedforward term.
double modelInverse(double ref, double dref)
{
    return std::clamp(kA * dref + kCd * std::sqrt(std::max(ref, 0.0)), 0.0, 3.0);
}

/**
 * Open-loop fallback entry: commands the model inverse of the current setpoint and
 * ignores the measurement entirely. This is the only reconfiguration that actually helps
 * once the measurement is known-bad - see the note in main(). Any feedback law, at any
 * gain, drives the *measured* level to the setpoint, so detuning changes the transient
 * and not the steady-state error.
 */
class ModelOpenLoopController : public ctrl::IController
{
public:
    explicit ModelOpenLoopController(double Ts) : Ts_(Ts) {}

    void setRef(double ref, double dref) { ref_ = ref; dref_ = dref; }

    double compute(double error) override
    {
        if (!std::isfinite(error)) return last_;   // NaN contract: hold last output
        last_ = modelInverse(ref_, dref_);
        return last_;
    }
    void   reset() override { last_ = 0.0; ref_ = kRefMean; dref_ = 0.0; }
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "ModelOpenLoop"; }

private:
    double Ts_;
    double ref_  = kRefMean;
    double dref_ = 0.0;
    double last_ = 0.0;
};

ctrl::PIDParams nominalTuning()
{
    ctrl::PIDParams p;
    p.Kp = 1.20; p.Ki = 0.50; p.Kd = 0.0; p.N = 20.0;
    p.uMin = 0.0; p.uMax = 3.0; p.Kb = 1.0;   // a pump cannot suck
    return p;
}

enum class Arm {
    NoVote,          ///< mean of the fresh held values - no outlier rejection
    Vote,            ///< median vote, no reconfiguration
    VoteFTC,         ///< median vote + FTCSupervisor reconfiguration
    OpenLoopAlways   ///< control case: never uses the measurement at all
};

const char *armName(Arm a)
{
    switch (a) {
    case Arm::NoVote:         return "mean fuse, no vote";
    case Arm::Vote:           return "median vote";
    case Arm::VoteFTC:        return "median vote + FTC";
    case Arm::OpenLoopAlways: return "open loop (control)";
    }
    return "?";
}

struct ArmResult {
    double mae_healthy = 0.0;   // mean |h_true - ref| per window
    double mae_single  = 0.0;
    double mae_double  = 0.0;
    double mae_load    = 0.0;   // over the load-disturbance window only
    double mae_quiet   = 0.0;   // [kQuietStart, kQuietEnd): healthy AND undisturbed

    double max_disagree_healthy = 0.0;   // max_i |y_i - y_vote|
    double max_disagree_single  = 0.0;
    double max_disagree_double  = 0.0;

    double max_resid_healthy = 0.0;      // |y_vote - h_model|
    double max_resid_single  = 0.0;
    double max_resid_double  = 0.0;

    double switch_time   = -1.0;         // first move off "Nominal"
    double transfer_bump = 0.0;          // |du| on the switching tick

    std::string entry_at_load;           // active entry mid load disturbance
    std::string entry_at_single;         // active entry mid single-fault window
    std::string entry_at_double;         // active entry mid double-fault window

    int degraded_votes = 0;              // ticks with fewer than 3 fresh sensors
    std::array<unsigned, kNSensors> delivered{}, dropped{};
};

ArmResult runArm(Arm arm)
{
    ArmResult r;

    // -- three independent links -------------------------------------------------
    std::array<std::shared_ptr<ctrl::NetworkChannel<double>>, kNSensors> link{};
    for (int i = 0; i < kNSensors; ++i) {
        ctrl::NetworkChannelParams lp;
        lp.latency_mean = kLat[i];
        lp.jitter_sigma = kJit[i];
        lp.loss_prob    = kLoss[i];
        lp.seed         = kSeed[i];
        link[i] = std::make_shared<ctrl::NetworkChannel<double>>(lp);
    }

    // -- UKF: p = 1, fed the VOTED level (see the header note on why p = 3 is not an
    //    option). alpha = 1, kappa = 2 keeps Wc0 positive at n = 1; the library default
    //    alpha = 1e-3 makes (n + lambda) ~ 0 and the sigma-point scaling degenerate.
    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(1);
        const double h = std::max(x(0), 0.0);
        xn(0) = x(0) + kTs * (u(0) - kCd * std::sqrt(h)) / kA;
        return xn;
    };
    auto hm = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        Eigen::VectorXd y(1);
        y(0) = x(0);
        return y;
    };
    Eigen::MatrixXd Q(1, 1), R(1, 1), P0(1, 1);
    Q(0, 0)  = 1.6e-5;   // (0.004 m)^2 - model trust
    R(0, 0)  = 1.2e-3;   // (0.035 m)^2 - deliberately loose: a filter that swallows the
                         //   bias faster than the classifier confirms it hides the fault
    P0(0, 0) = 1.0e-2;
    ctrl::UnscentedKalmanFilter ukf(1, 1, f, hm, Q, R, kTs, P0,
                                    /*alpha=*/1.0, /*beta=*/2.0, /*kappa=*/2.0);
    ukf.setState(Eigen::VectorXd::Constant(1, kH0));

    // -- controller stack + supervisor -------------------------------------------
    auto pi = std::make_shared<ctrl::DiscretePID>(nominalTuning(), kTs);
    auto ol = std::make_shared<ModelOpenLoopController>(kTs);

    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, kTs);
    stack->addController(pi, "Nominal");        // no activationCondition: FTCSupervisor
    stack->addController(ol, "ModelOpenLoop");  // owns eligibility entirely

    ctrl::FaultDetectorParams fp;
    fp.residual_threshold = kResidThreshold;
    fp.confirm_window     = 5;
    fp.bias_threshold     = 2.0;
    // Both defaults below are calibrated for a signal that swings; they need re-scaling
    // for a level loop holding a ~0.85 m^3/s flow:
    //  - stuck_du_threshold 1e-3 exceeds this loop's entire command excursion, so every
    //    healthy tick would classify as ActuatorStuck.
    //  - corr_threshold 0.3 assumes the command reacts to the RAW measurement. It reacts
    //    to the UKF's filtered estimate, so command/measurement increments decorrelate
    //    even with a perfectly healthy actuator.
    fp.stuck_du_threshold = 1e-6;
    fp.corr_threshold     = 0.05;

    ctrl::FTCSupervisor ftc(stack, fp, kTs);
    ftc.registerFaultResponse(ctrl::FaultType::None,       "Nominal");
    ftc.registerFaultResponse(ctrl::FaultType::SensorBias, "ModelOpenLoop");

    // -- state ---------------------------------------------------------------------
    std::mt19937 noise(kNoiseSeed);
    std::normal_distribution<double> nd(0.0, kSensorSigma);

    double h_true    = kH0;
    double h_twin    = kH0;                  // model reference; frozen once a fault fires
    double u_prev    = modelInverse(kH0, 0.0);
    double u_applied = u_prev;
    double y_vote    = kH0;

    std::array<double, kNSensors> hold{}, age{};
    for (int i = 0; i < kNSensors; ++i) { hold[i] = kH0; age[i] = 0.0; }

    int n_healthy = 0, n_single = 0, n_double = 0, n_load = 0, n_quiet = 0;
    std::string prev_entry = "Nominal";

    Eigen::VectorXd uv(1), yv(1);

    for (int k = 0; k < kNSteps; ++k) {
        const double t   = k * kTs;
        const double ref = refAt(t);

        // ---- sensors -> links (every arm sends every tick, so all four arms see
        //      byte-identical delivery traces; the RNG draws are payload-independent) --
        for (int i = 0; i < kNSensors; ++i) {
            double bias = 0.0;
            if (i == 1 && t >= kFault1Time) bias = kFault1Bias;
            if (i == 2 && t >= kFault2Time) bias = kFault2Bias;
            link[i]->send(h_true + bias + nd(noise), t);
        }

        // ---- receive + age ------------------------------------------------------
        for (int i = 0; i < kNSensors; ++i) {
            double rx = 0.0;
            if (link[i]->tryReceive(rx, t)) { hold[i] = rx; age[i] = 0.0; }
            else                            { age[i] += kTs; }
        }

        // ---- vote ---------------------------------------------------------------
        std::array<double, kNSensors> fresh{};
        int nf = 0;
        for (int i = 0; i < kNSensors; ++i)
            if (age[i] <= kMaxAge) fresh[nf++] = hold[i];
        if (nf < kNSensors) ++r.degraded_votes;

        if (nf == 0) {
            // total starvation: hold the previous vote
        } else if (arm == Arm::NoVote) {
            double s = 0.0;
            for (int i = 0; i < nf; ++i) s += fresh[i];
            y_vote = s / nf;
        } else if (nf == 1) {
            y_vote = fresh[0];
        } else if (nf == 2) {
            y_vote = 0.5 * (fresh[0] + fresh[1]);   // two sensors cannot outvote each other
        } else {
            std::sort(fresh.begin(), fresh.begin() + nf);
            y_vote = fresh[nf / 2];                  // median of three
        }

        double disagree = 0.0;
        for (int i = 0; i < kNSensors; ++i)
            if (age[i] <= kMaxAge)
                disagree = std::max(disagree, std::abs(hold[i] - y_vote));

        // ---- estimator: predict, score the residual, then correct ---------------
        uv(0) = u_applied;
        ukf.predict(uv);
        const double h_prior = ukf.state()(0);

        const ctrl::FaultType fault_now = ftc.currentFault();
        if (fault_now == ctrl::FaultType::None) {
            h_twin = h_prior;                       // track while the vote is trusted
        } else {
            const double hh = std::max(h_twin, 0.0);
            h_twin += kTs * (u_applied - kCd * std::sqrt(hh)) / kA;   // frozen: open loop
        }
        const double resid = y_vote - h_twin;

        yv(0) = y_vote;
        ukf.update(yv, uv);
        const double h_hat = ukf.state()(0);

        // ---- control ------------------------------------------------------------
        double u_cmd;
        if (arm == Arm::OpenLoopAlways) {
            ol->setRef(ref, refRateAt(t));
            u_cmd = ol->compute(0.0);
        } else if (arm == Arm::VoteFTC) {
            ol->setRef(ref, refRateAt(t));
            ftc.feedResidual(resid, u_prev, y_vote);
            u_cmd = ftc.compute(ref - h_hat);

            const std::string entry = stack->activeControllerName();
            if (entry != prev_entry && entry == "ModelOpenLoop" && r.switch_time < 0.0) {
                r.switch_time   = t;
                r.transfer_bump = std::abs(u_cmd - u_prev);
            }
            prev_entry = entry;

            // sampled snapshots, one per window
            if (std::abs(t - 0.5 * (kLoadStart + kLoadEnd)) < 0.5 * kTs)
                r.entry_at_load = entry;
            if (std::abs(t - 0.5 * (kFault1Time + kFault2Time)) < 0.5 * kTs)
                r.entry_at_single = entry;
            if (std::abs(t - 0.5 * (kFault2Time + kTsim)) < 0.5 * kTs)
                r.entry_at_double = entry;
        } else {
            u_cmd = pi->compute(ref - h_hat);       // no supervisor in these arms
        }

        u_prev    = u_cmd;
        u_applied = u_cmd;

        // ---- plant ---------------------------------------------------------------
        const double hh = std::max(h_true, 0.0);
        h_true += kTs * (u_applied - kCd * std::sqrt(hh) - loadAt(t)) / kA;

        // ---- metrics -------------------------------------------------------------
        const double err = std::abs(h_true - ref);
        if (t < kWinHealthyEnd) {
            r.mae_healthy += err; ++n_healthy;
            r.max_disagree_healthy = std::max(r.max_disagree_healthy, disagree);
            r.max_resid_healthy    = std::max(r.max_resid_healthy, std::abs(resid));
        } else if (t < kWinSingleEnd) {
            r.mae_single += err; ++n_single;
            r.max_disagree_single = std::max(r.max_disagree_single, disagree);
            r.max_resid_single    = std::max(r.max_resid_single, std::abs(resid));
        } else {
            r.mae_double += err; ++n_double;
            r.max_disagree_double = std::max(r.max_disagree_double, disagree);
            r.max_resid_double    = std::max(r.max_resid_double, std::abs(resid));
        }
        if (t >= kLoadStart  && t < kLoadEnd)  { r.mae_load  += err; ++n_load; }
        if (t >= kQuietStart && t < kQuietEnd) { r.mae_quiet += err; ++n_quiet; }
    }

    if (n_healthy) r.mae_healthy /= n_healthy;
    if (n_single)  r.mae_single  /= n_single;
    if (n_double)  r.mae_double  /= n_double;
    if (n_load)    r.mae_load    /= n_load;
    if (n_quiet)   r.mae_quiet   /= n_quiet;

    for (int i = 0; i < kNSensors; ++i) {
        r.delivered[i] = link[i]->delivered();
        r.dropped[i]   = link[i]->dropped();
    }
    return r;
}

}  // namespace

int main()
{
    std::printf("=== ex140: fault-tolerant sensor voting over redundant links ===\n\n");
    std::printf("plant    : h' = (u - %.1f sqrt(h) - q_load)/%.1f   (nonlinear -> UKF)\n",
                kCd, kA);
    std::printf("setpoint : %.2f + %.2f sin(2 pi t / %.0f) m  (moving: the classifier's\n"
                "           causality test needs a command that actually varies)\n",
                kRefMean, kRefAmp, kRefPeriod);
    std::printf("sensors  : 3 x level, sigma = %.3f m, on independent links\n", kSensorSigma);
    for (int i = 0; i < kNSensors; ++i)
        std::printf("           #%d  latency %5.0f ms  jitter %4.0f ms  loss %4.1f %%  seed %u\n",
                    i + 1, kLat[i] * 1e3, kJit[i] * 1e3, kLoss[i] * 100.0, kSeed[i]);
    std::printf("load     : +%.2f m^3/s outflow over t = [%.0f, %.0f) s, sensors HEALTHY\n",
                kLoadQ, kLoadStart, kLoadEnd);
    std::printf("fault 1  : sensor 2 bias +%.2f m at t = %.0f s   (vote still correct)\n",
                kFault1Bias, kFault1Time);
    std::printf("fault 2  : sensor 3 bias +%.2f m at t = %.0f s   (vote now carries it)\n\n",
                kFault2Bias, kFault2Time);

    const ArmResult novote = runArm(Arm::NoVote);
    const ArmResult vote   = runArm(Arm::Vote);
    const ArmResult ftc    = runArm(Arm::VoteFTC);
    const ArmResult openl  = runArm(Arm::OpenLoopAlways);

    // -- tracking error per window ------------------------------------------------
    std::printf("  mean |h_true - ref| [m] by window\n");
    std::printf("  ('quiet' is the undisturbed slice of 'healthy'; 'load dist.' is the rest)\n");
    std::printf("  %-22s %10s %10s %10s %10s %10s\n",
                "arm", "quiet", "load dist.", "healthy", "1 bad", "2 bad");
    const ArmResult *all[] = {&novote, &vote, &ftc, &openl};
    const Arm        ids[] = {Arm::NoVote, Arm::Vote, Arm::VoteFTC, Arm::OpenLoopAlways};
    for (int i = 0; i < 4; ++i)
        std::printf("  %-22s %10.4f %10.4f %10.4f %10.4f %10.4f\n", armName(ids[i]),
                    all[i]->mae_quiet, all[i]->mae_load, all[i]->mae_healthy,
                    all[i]->mae_single, all[i]->mae_double);

    // -- the two residuals ---------------------------------------------------------
    // All three rows come from the FTC arm so the reference is the same throughout: its
    // twin tracks the filter while no fault is flagged (so the healthy and 1-bad rows are
    // the plain innovation) and freezes afterwards (so the 2-bad row does not decay away).
    std::printf("\n  the two residuals, FTC arm (alarm threshold %.2f)\n", kResidThreshold);
    std::printf("  %-22s %14s %14s\n", "window", "disagreement", "innovation");
    std::printf("  %-22s %14.4f %14.4f\n", "healthy",
                ftc.max_disagree_healthy, ftc.max_resid_healthy);
    std::printf("  %-22s %14.4f %14.4f\n", "1 bad sensor",
                ftc.max_disagree_single, ftc.max_resid_single);
    std::printf("  %-22s %14.4f %14.4f\n", "2 bad sensors",
                ftc.max_disagree_double, ftc.max_resid_double);
    std::printf("  threshold margins: %.2fx clear of the worst healthy residual,"
                " %.2fx over it on fault 2\n",
                kResidThreshold / ftc.max_resid_healthy,
                ftc.max_resid_double / kResidThreshold);

    // -- link accounting -----------------------------------------------------------
    std::printf("\n  link traffic (identical across arms by construction)\n");
    for (int i = 0; i < kNSensors; ++i)
        std::printf("    #%d  delivered %4u   dropped %3u\n",
                    i + 1, vote.delivered[i], vote.dropped[i]);
    std::printf("    ticks voting on fewer than 3 fresh sensors: %d of %d\n",
                vote.degraded_votes, kNSteps);
    if (vote.degraded_votes == 0)
        std::printf("    -> the %.0f ms staleness gate never had to fire: %.0f %% per-packet\n"
                    "       loss on a %.0f ms link never opens a gap that wide. The 2-sensor\n"
                    "       and 1-sensor vote paths are therefore present but UNEXERCISED\n"
                    "       here - a link outage, not packet loss, is what reaches them.\n",
                    kMaxAge * 1e3, kLoss[2] * 100.0, kLat[2] * 1e3);

    std::printf("\n  FTC arm: active entry mid-window  load=%s  1bad=%s  2bad=%s\n",
                ftc.entry_at_load.c_str(), ftc.entry_at_single.c_str(),
                ftc.entry_at_double.c_str());
    if (ftc.switch_time >= 0.0)
        std::printf("           reconfigured at t = %.2f s, transfer bump %.4f m^3/s.\n"
                    "           NOT bumpless, and by construction: ControllerStack does call\n"
                    "           bumplessInit() on the incoming entry, but a stateless\n"
                    "           feedforward has no integrator to re-seed, so it steps.\n",
                    ftc.switch_time, ftc.transfer_bump);
    else
        std::printf("           never reconfigured\n");

    // -- acceptance ------------------------------------------------------------------
    bool traces_identical = true;
    for (int i = 0; i < kNSensors; ++i)
        traces_identical &= (novote.delivered[i] == vote.delivered[i]) &&
                            (vote.delivered[i]   == ftc.delivered[i])  &&
                            (ftc.delivered[i]    == openl.delivered[i]) &&
                            (novote.dropped[i]   == vote.dropped[i])   &&
                            (vote.dropped[i]     == ftc.dropped[i])    &&
                            (ftc.dropped[i]      == openl.dropped[i]);

    // Voting earns its place only if the unvoted arm actually suffers.
    const bool fault1_hurts_unvoted = novote.mae_single > 3.0 * novote.mae_quiet;
    const bool vote_masks_fault1    = vote.mae_single < 0.40 * novote.mae_single;

    // Direction 1: with one bad sensor the disagreement fires and the innovation does not.
    const bool d_fires_1 = ftc.max_disagree_single > 4.0 * ftc.max_disagree_healthy;
    const bool r_quiet_1 = ftc.max_resid_single < kResidThreshold;

    // Direction 2: with two bad sensors BOTH fire - which is what makes the pair, and only
    // the pair, able to tell "someone is lying" from "the majority is lying".
    const bool r_fires_2 = ftc.max_resid_double > kResidThreshold;

    // No false alarm on the load disturbance, and a real one on fault 2.
    const bool no_false_alarm = (ftc.entry_at_load == "Nominal") &&
                                (ftc.entry_at_single == "Nominal");
    const bool reconfigured   = (ftc.entry_at_double == "ModelOpenLoop") &&
                                ftc.switch_time >= kFault2Time;
    const bool ftc_helps      = ftc.mae_double < 0.40 * vote.mae_double;

    // Control case: open loop is NOT simply better - it fails the disturbance the closed
    // loop rejects. Without this the reconfiguration proves nothing about the fault.
    const bool feedback_needed = openl.mae_load > 3.0 * vote.mae_load;

    std::printf("\n  paired on one trace     = %s (delivered/dropped identical across all 4 arms)\n",
                traces_identical ? "yes" : "no");
    std::printf("  fault 1 hurts unvoted   = %s (%.4f vs %.4f quiet baseline, >3x)\n",
                fault1_hurts_unvoted ? "yes" : "no", novote.mae_single, novote.mae_quiet);
    std::printf("  vote masks fault 1      = %s (%.4f < 40%% of %.4f)\n",
                vote_masks_fault1 ? "yes" : "no", vote.mae_single, novote.mae_single);
    std::printf("  1 bad: d fires, r quiet = %s / %s (d %.4f vs %.4f healthy; r %.4f < %.2f)\n",
                d_fires_1 ? "yes" : "no", r_quiet_1 ? "yes" : "no",
                ftc.max_disagree_single, ftc.max_disagree_healthy,
                ftc.max_resid_single, kResidThreshold);
    std::printf("  2 bad: r fires too      = %s (r %.4f > %.2f - the vote now carries it)\n",
                r_fires_2 ? "yes" : "no", ftc.max_resid_double, kResidThreshold);
    std::printf("  no false alarm          = %s (Nominal through the load disturbance"
                " and the 1-bad window)\n", no_false_alarm ? "yes" : "no");
    std::printf("  reconfigured on fault 2 = %s (t = %.2f s >= %.0f s)\n",
                reconfigured ? "yes" : "no", ftc.switch_time, kFault2Time);
    std::printf("  reconfiguration helps   = %s (%.4f < 40%% of %.4f)\n",
                ftc_helps ? "yes" : "no", ftc.mae_double, vote.mae_double);
    std::printf("  feedback still needed   = %s (open loop %.4f vs voted %.4f on the load)\n",
                feedback_needed ? "yes" : "no", openl.mae_load, vote.mae_load);

    const bool ok = traces_identical && fault1_hurts_unvoted && vote_masks_fault1 &&
                    d_fires_1 && r_quiet_1 && r_fires_2 && no_false_alarm &&
                    reconfigured && ftc_helps && feedback_needed;

    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
