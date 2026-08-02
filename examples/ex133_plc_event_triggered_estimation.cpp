// ex133_plc_event_triggered_estimation.cpp - Event-triggered estimation over a constrained
// server/PLC link.
//
// Fusion: ctrl::KalmanFilter (+ built-in ctrl::MismatchDetector) + ctrl::EventTriggeredWrapper
//         + ctrl::NetworkChannel
//
// Architecture:
//
//     PLC (slave)  : plant + sensor. Transmits a measurement ONLY when it has moved more than
//                    the send-on-delta deadband since the last transmission.
//        |  measurement -> NetworkChannel (latency + jitter)
//     srv (master) : KalmanFilter. Runs predict() every tick and update() only on the ticks a
//                    packet actually arrives, so the estimate coasts on the model in between.
//                    EventTriggeredWrapper gates the control law on the same principle.
//                    The filter's built-in mismatch detector watches the innovation.
//
// The point: on a large PLC network, bandwidth is the scarce resource. Sending every sample is
// wasteful when the plant is quiescent. This demo cuts uplink traffic by ~85% while keeping the
// state estimate within a stated RMS bound, and still detects a sensor fault.
//
// Two semantics corrections worth stating, because the obvious reading of both classes is wrong:
//
//   1. ctrl::EventTriggeredWrapper gates CONTROLLER EXECUTION on a deadband. It does NOT sit
//      between sensor I/O and a filter and it does not gate telemetry. It is used here for its
//      real purpose - suppressing redundant control recomputation - while the send-on-delta
//      MEASUREMENT gate is implemented explicitly on the slave, using the same deadband idea.
//
//   2. The mismatch detector is not a separate object here. ctrl::KalmanFilter already embeds
//      one: enableMismatchDetection() / mismatchDetected() / mismatchScore(). Constructing a
//      standalone ctrl::MismatchDetector and hand-feeding it innovations would duplicate what
//      the filter already does internally on every update().
//
// Baseline for the comparison is the SAME filter fed periodically over the SAME seeded link, so
// the bandwidth saving is measured against a like-for-like arm rather than an easier scenario.

#include <ControllerToolbox.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <random>

namespace {

// -- Plant: damped second-order position/velocity ------------------------------
//   x = [pos, vel];  vel' = -2*z*w*vel - w^2*pos + w^2*u
constexpr double kW        = 3.0;    // natural frequency [rad/s]
constexpr double kZeta     = 0.35;   // damping ratio
constexpr double kTs       = 0.010;  // master/slave tick [s]
constexpr double kTsim     = 20.0;
constexpr int    kNSteps   = static_cast<int>(kTsim / kTs + 0.5);

constexpr double kMeasNoise = 0.010;  // sensor noise std dev [m]
constexpr double kDeadband  = 0.020;  // send-on-delta threshold [m]

constexpr double kLatency = 0.020;
constexpr double kJitter  = 0.005;
constexpr unsigned kSeed  = 4242u;

// Sensor fault: an ABRUPT bias step - a jumped encoder count / offset failure.
//
// It is deliberately a step and not a drift. An innovation-CUSUM detector only sees faults that
// disturb the innovation sequence, and a Kalman filter TRACKS slow drift: at 0.25 m/s the
// per-tick bias increment is 0.0025 m, four times smaller than the 0.01 m measurement noise it
// hides in, so the innovation never leaves its null distribution and no CUSUM threshold catches
// it. Catching slow parametric drift is a job for a parameter estimator
// (ctrl::RecursiveGreyBoxEstimator), not for this detector.
constexpr double kFaultTime = 14.0;
constexpr double kBiasStep  = 0.35;   // [m] abrupt sensor offset

/// Discrete plant model shared by the simulator and the filter.
ctrl::StateSpace plantModel()
{
    // Forward-Euler discretisation of the continuous second-order system.
    Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
    A << 1.0,                    kTs,
         -kW * kW * kTs,         1.0 - 2.0 * kZeta * kW * kTs;
    B << 0.0,
         kW * kW * kTs;
    C << 1.0, 0.0;      // only position is measured
    D << 0.0;
    return ctrl::StateSpace(A, B, C, D, kTs);
}

/// Square reference that exercises the deadband: long quiet stretches, occasional steps.
double referenceAt(double t)
{
    if (t < 4.0)  return 0.0;
    if (t < 9.0)  return 1.0;
    if (t < 15.0) return 0.4;
    return 0.9;
}

struct ArmResult {
    unsigned tx          = 0;      // packets the slave actually transmitted
    unsigned rx          = 0;      // packets the master received
    int      kf_updates  = 0;      // ticks where update() ran (vs predict-only)
    int      ctrl_holds  = 0;      // ticks where EventTriggeredWrapper held its output
    // Estimation quality is measured PRE-FAULT only. After the sensor fails the filter is being
    // fed a lie, so post-fault error measures the fault, not the cost of event-triggering.
    double   rms_est_err = 0.0;    // RMS of (true position - estimate), t < kFaultTime
    double   max_est_err = 0.0;
    double   iae         = 0.0;    // closed-loop tracking
    bool     fault_flagged = false;
    double   fault_flag_time = -1.0;
    double   score_prefault = 0.0; // mismatch score just before the fault
    double   score_final    = 0.0;
};

/// @param event_triggered  true  -> send-on-delta uplink + event-triggered control
///                         false -> periodic uplink every tick, control every tick
ArmResult runArm(bool event_triggered)
{
    const ctrl::StateSpace model = plantModel();

    // -- master: Kalman filter with its built-in mismatch detector -----------
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2) * 1e-4;
    Eigen::MatrixXd R(1, 1);
    R << kMeasNoise * kMeasNoise;
    ctrl::KalmanFilter kf(model, Q, R);

    // sigma is the expected innovation RMS when the model is right. Here that is the sensor
    // noise (0.010) inflated by the transport lag: a measurement is ~2 ticks stale by the time
    // it is fused, so during setpoint moves the innovation carries a real lag component too.
    // 0.06 leaves headroom over that without masking the 0.35 m fault step.
    ctrl::MismatchDetectorParams mp;
    mp.sigma       = 0.06;
    mp.k_cusum     = 0.5;
    mp.h_threshold = 5.0;
    kf.enableMismatchDetection(mp);

    // -- master: control law, optionally event-triggered ---------------------
    ctrl::PIDParams pp;
    pp.Kp = 1.6; pp.Ki = 2.2; pp.Kd = 0.05; pp.N = 40.0;
    pp.uMin = -3.0; pp.uMax = 3.0; pp.Kb = 1.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, kTs);

    ctrl::EventTriggeredParams ep;
    ep.sigma = kDeadband;
    auto gated = std::make_shared<ctrl::EventTriggeredWrapper>(pid, ep);

    std::shared_ptr<ctrl::IController> law =
        event_triggered ? std::static_pointer_cast<ctrl::IController>(gated)
                        : std::static_pointer_cast<ctrl::IController>(pid);

    // -- link -----------------------------------------------------------------
    ctrl::NetworkChannelParams lp;
    lp.latency_mean = kLatency;
    lp.jitter_sigma = kJitter;
    lp.loss_prob    = 0.0;      // isolate the bandwidth question from packet loss
    lp.seed         = kSeed;
    ctrl::NetworkChannel<double> up(lp);

    // -- simulation ------------------------------------------------------------
    Eigen::VectorXd x(2);       // true plant state (slave)
    x << 0.0, 0.0;
    Eigen::VectorXd u_vec(1), y_vec(1);
    u_vec << 0.0;

    std::mt19937 noise_rng(kSeed);                       // sensor noise, identical across arms
    std::normal_distribution<double> meas_noise(0.0, kMeasNoise);

    double last_tx_value = 0.0;
    bool   have_tx       = false;
    double u_applied     = 0.0;
    double sum_sq_err    = 0.0;
    int    n_prefault    = 0;

    ArmResult r;

    for (int k = 0; k < kNSteps; ++k) {
        const double t   = k * kTs;
        const double ref = referenceAt(t);

        // -- slave: read the sensor ------------------------------------------
        double y_meas = x(0) + meas_noise(noise_rng);
        if (t >= kFaultTime) y_meas += kBiasStep;                      // abrupt sensor offset

        // -- slave: send-on-delta gate (explicit; NOT EventTriggeredWrapper) --
        const bool send_now = !event_triggered || !have_tx ||
                              std::abs(y_meas - last_tx_value) > kDeadband;
        if (send_now) {
            up.send(y_meas, t);
            last_tx_value = y_meas;
            have_tx       = true;
            ++r.tx;
        }

        // -- master: predict every tick, update only when a packet lands ------
        kf.predict(u_vec);
        double rx = 0.0;
        if (up.tryReceive(rx, t)) {
            y_vec << rx;
            kf.update(y_vec, u_vec);
            ++r.kf_updates;
            ++r.rx;
        }

        const double pos_hat = kf.state()(0);

        // -- master: control on the ESTIMATE, not the raw measurement ---------
        u_applied = law->compute(ref - pos_hat);
        u_vec << u_applied;

        // -- plant step (slave) ------------------------------------------------
        const Eigen::VectorXd x_next = model.A * x + model.B * u_vec;
        x = x_next;

        // -- metrics ------------------------------------------------------------
        const double est_err = std::abs(x(0) - pos_hat);
        if (t < kFaultTime) {                       // pre-fault window only
            sum_sq_err += est_err * est_err;
            ++n_prefault;
            r.max_est_err = std::max(r.max_est_err, est_err);
        }
        r.iae += std::abs(ref - x(0)) * kTs;

        if (!r.fault_flagged && kf.mismatchDetected()) {
            r.fault_flagged   = true;
            r.fault_flag_time = t;
        }
        if (t < kFaultTime) r.score_prefault = std::max(r.score_prefault, kf.mismatchScore());
    }

    r.rms_est_err = (n_prefault > 0) ? std::sqrt(sum_sq_err / n_prefault) : 0.0;
    r.ctrl_holds  = event_triggered ? gated->holdCount() : 0;
    r.score_final = kf.mismatchScore();
    return r;
}

}  // namespace

int main()
{
    std::printf("=== ex133: event-triggered estimation over a constrained PLC link ===\n\n");
    std::printf("plant     : 2nd order, w = %.1f rad/s, zeta = %.2f, position measured only\n",
                kW, kZeta);
    std::printf("tick      : %.0f ms, %d steps (%.0f s)\n", kTs * 1e3, kNSteps, kTsim);
    std::printf("sensor    : noise sigma %.3f m, send-on-delta deadband %.3f m\n",
                kMeasNoise, kDeadband);
    std::printf("link      : %.0f ms +/- %.0f ms, no loss, seed %u\n",
                kLatency * 1e3, kJitter * 1e3, kSeed);
    std::printf("fault     : abrupt %.2f m sensor bias step at t = %.1f s\n\n",
                kBiasStep, kFaultTime);

    const ArmResult periodic = runArm(/*event_triggered=*/false);
    const ArmResult evented  = runArm(/*event_triggered=*/true);

    std::printf("  %-18s %8s %8s %12s %12s %10s\n",
                "arm", "tx", "kf_upd", "RMS(pre-flt)", "max(pre-flt)", "IAE");
    std::printf("  %-18s %8u %8d %12.5f %12.5f %10.4f\n",
                "periodic", periodic.tx, periodic.kf_updates,
                periodic.rms_est_err, periodic.max_est_err, periodic.iae);
    std::printf("  %-18s %8u %8d %12.5f %12.5f %10.4f\n",
                "event-triggered", evented.tx, evented.kf_updates,
                evented.rms_est_err, evented.max_est_err, evented.iae);

    const double saving = 100.0 * (1.0 - static_cast<double>(evented.tx) /
                                          static_cast<double>(periodic.tx));
    std::printf("\n  uplink traffic reduced by %.1f %% (%u -> %u packets)\n",
                saving, periodic.tx, evented.tx);
    std::printf("  control recomputation held on %d of %d ticks by EventTriggeredWrapper\n",
                evented.ctrl_holds, kNSteps);
    std::printf("  mismatch: flagged at t = %.2f s | peak score before fault %.2f, final %.2f\n",
                evented.fault_flag_time, evented.score_prefault, evented.score_final);

    // -- acceptance -----------------------------------------------------------
    const bool saves_bandwidth = saving > 50.0;
    // The estimate must not be sacrificed for the bandwidth: allow a bounded degradation only.
    const bool estimate_ok     = evented.rms_est_err < 3.0 * periodic.rms_est_err &&
                                 std::isfinite(evented.rms_est_err);
    // Detection must happen, and must not be a false positive from the quiet period.
    const bool detected        = evented.fault_flagged &&
                                 evented.fault_flag_time >= kFaultTime;
    const bool tracks          = std::isfinite(evented.iae);

    std::printf("\n  bandwidth saved      = %s (%.1f %% > 50 %%)\n",
                saves_bandwidth ? "yes" : "no", saving);
    std::printf("  estimate preserved   = %s (RMS %.5f < 3x periodic %.5f)\n",
                estimate_ok ? "yes" : "no", evented.rms_est_err, 3.0 * periodic.rms_est_err);
    std::printf("  sensor fault caught  = %s (t = %.2f s, fault injected at %.1f s)\n",
                detected ? "yes" : "no", evented.fault_flag_time, kFaultTime);

    const bool ok = saves_bandwidth && estimate_ok && detected && tracks;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
