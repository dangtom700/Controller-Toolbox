// ex134_plc_bumpless_redundancy.cpp - Bumpless transfer between redundant control servers.
//
// Fusion: ctrl::ControllerStack (Weighted) + ctrl::AtomicParamBuffer + ctrl::NetworkChannel
//
// Architecture:
//
//     srv A (primary)  : runs the control law; its output is the one in command
//     srv B (standby)  : runs the SAME law on the SAME inputs, output not applied (hot standby)
//     supervisor       : watches the primary's heartbeat; on loss, publishes a failover command
//                        through an AtomicParamBuffer
//     PLC (slave)      : applies the blended command arriving over the link
//
// Hot standby only works if the handover is bumpless. Hard-switching to a controller whose
// integrator has been tracking in the background but was never in command produces a step in
// the actuator - exactly what a Weighted ControllerStack avoids: it ramps the primary's weight
// 1 -> 0 while the standby's goes 0 -> 1 over kRampTime, so the applied command is a continuous
// blend throughout.
//
// This is the first example in the repository to exercise ctrl::AtomicParamBuffer.
//
// Three implementation notes:
//
//   1. AtomicParamBuffer.h is NOT pulled in by the ControllerToolbox.h umbrella - it is
//      deliberately opt-in there (grouped with hal/HAL.h), so it must be included directly.
//
//   2. The buffer's payload must be trivially copyable; the template static_asserts it. That is
//      what makes the seqlock safe: a torn read is detected by the sequence counter and retried,
//      with no locking on the reader side.
//
//   3. The supervisor runs inline in this demo rather than on a real std::thread, so the example
//      stays deterministic for run.py. The publish/read API exercised here is exactly the one a
//      genuine background thread would use - the seqlock is what makes the same code safe there.
//
// The primary "server hang" is modelled by HangingController: a decorator that stops refreshing
// its output at a chosen time, which is what a wedged control process looks like downstream.

#include <ControllerToolbox.h>
#include <AtomicParamBuffer.h>      // opt-in: not in the umbrella header

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <type_traits>

namespace {

// -- Plant: first-order thermal-ish lag ----------------------------------------
constexpr double kA   = 1.6;     // 1/tau
constexpr double kB   = 1.6;     // unity DC gain
constexpr double kTs  = 0.010;
constexpr double kTsim   = 16.0;
constexpr int    kNSteps = static_cast<int>(kTsim / kTs + 0.5);

// Timing matters here. The primary must wedge MID-TRANSIENT, not at steady state: a command
// frozen at the correct steady-state value costs nothing, so failing over from it proves
// nothing. Stepping the setpoint first and wedging 50 ms later means the frozen command is
// already wrong when the supervisor reacts, and the live standby's command has diverged from
// it - which is what makes both the benefit and the bumplessness real rather than vacuous.
constexpr double kRefInitial = 1.0;
constexpr double kRefStep    = 1.8;
constexpr double kStepTime   = 8.00;   // setpoint move
constexpr double kHangTime   = 8.05;   // primary wedges mid-transient
constexpr double kDetectLag  = 0.15;   // heartbeat timeout before the supervisor reacts
constexpr double kRampTime   = 0.10;   // bumpless ramp duration (paper-typical 100 ms)

constexpr double kLatency = 0.008;
constexpr double kJitter  = 0.002;
constexpr unsigned kSeed  = 31337u;

/// Command published by the supervisor to the real-time loop through AtomicParamBuffer.
/// Trivially copyable by construction - AtomicParamBuffer static_asserts this.
struct FailoverCmd {
    bool     primary_healthy;   ///< false once the heartbeat has timed out
    double   target_weight;     ///< desired primary weight in [0, 1]
    unsigned seq;               ///< monotone publish counter (diagnostics)
};
static_assert(std::is_trivially_copyable_v<FailoverCmd>,
              "FailoverCmd must be trivially copyable to travel through AtomicParamBuffer");

/// Decorator modelling a wedged control process: after `hang_time` the output stops being
/// refreshed and the last computed value is repeated forever.
class HangingController : public ctrl::IController
{
public:
    HangingController(std::shared_ptr<ctrl::IController> inner, double hang_time, double Ts)
        : inner_(std::move(inner)), hang_time_(hang_time), Ts_(Ts) {}

    double compute(double error) override
    {
        const double t = static_cast<double>(k_++) * Ts_;
        if (t >= hang_time_) return last_;        // wedged: repeat the stale command
        last_ = inner_->compute(error);
        return last_;
    }
    void   reset() override { inner_->reset(); last_ = 0.0; k_ = 0; }
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "PrimaryServer"; }

private:
    std::shared_ptr<ctrl::IController> inner_;
    double hang_time_;
    double Ts_;
    double last_ = 0.0;
    long   k_    = 0;
};

ctrl::PIDParams lawTuning()
{
    ctrl::PIDParams p;
    p.Kp = 2.2; p.Ki = 3.0; p.Kd = 0.0; p.N = 40.0;
    p.uMin = -4.0; p.uMax = 4.0; p.Kb = 1.0;
    return p;
}

struct ArmResult {
    double iae            = 0.0;
    double final_err      = 0.0;
    double max_bump       = 0.0;   // largest single-tick |du| during the handover window
    double handover_gap   = 0.0;   // total |u| change across the handover - what a hard
                                   // switch would have delivered in ONE tick
    double failover_time  = -1.0;
    double w_primary_end  = 1.0;
    unsigned publishes    = 0;
};

/// @param do_failover  true  -> supervisor detects the hang and ramps to the standby
///                     false -> no supervisor; the wedged primary stays in command
ArmResult runArm(bool do_failover)
{
    // -- the two redundant servers, running identical laws --------------------
    auto primary_law = std::make_shared<ctrl::DiscretePID>(lawTuning(), kTs);
    auto primary     = std::make_shared<HangingController>(primary_law, kHangTime, kTs);
    auto standby     = std::make_shared<ctrl::DiscretePID>(lawTuning(), kTs);

    // -- weighted stack: both compute every tick, output is the blend ---------
    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Weighted, kTs);
    stack->addController(primary, "Primary", 1.0);
    stack->addController(standby, "Standby", 0.0);

    // -- lock-free channel from the supervisor to the RT loop -----------------
    // The buffer has no default constructor by design: both slots are seeded with the initial
    // value so a reader can never observe an uninitialised Params before the first publish().
    ctrl::AtomicParamBuffer<FailoverCmd> cmd_buf(FailoverCmd{true, 1.0, 0u});

    ctrl::NetworkChannelParams lp;
    lp.latency_mean = kLatency;
    lp.jitter_sigma = kJitter;
    lp.loss_prob    = 0.0;
    lp.seed         = kSeed;
    ctrl::NetworkChannel<double> down(lp);

    double x = 0.0, u_hold = 0.0, u_prev_applied = 0.0;
    double w_primary = 1.0;
    double last_published_w = 1.0;
    double u_before_handover = 0.0;
    bool   captured_before   = false;
    unsigned seq = 0;
    ArmResult r;

    const double t_detect  = kHangTime + kDetectLag;
    const double t_settled = t_detect + kRampTime + 5.0 * kTs;

    for (int k = 0; k < kNSteps; ++k) {
        const double t   = k * kTs;
        const double ref = (t >= kStepTime) ? kRefStep : kRefInitial;

        // ---- supervisor side (in a deployment: a separate thread) -----------
        // Heartbeat times out kDetectLag after the primary wedges. Publish only while the
        // target is actually moving - a supervisor that republishes an unchanged command every
        // tick would be wasting the buffer's whole point.
        if (do_failover && t >= kHangTime + kDetectLag) {
            const double ramp_t = t - (kHangTime + kDetectLag);
            const double target = std::clamp(1.0 - ramp_t / kRampTime, 0.0, 1.0);
            if (std::abs(target - last_published_w) > 1e-12) {
                cmd_buf.publish(FailoverCmd{false, target, ++seq});
                last_published_w = target;
                ++r.publishes;
            }
            if (r.failover_time < 0.0) r.failover_time = t;
        }

        // ---- real-time side: one lock-free read per tick --------------------
        const FailoverCmd cmd = cmd_buf.read();
        w_primary = std::clamp(cmd.target_weight, 0.0, 1.0);
        stack->setWeight("Primary", w_primary);
        stack->setWeight("Standby", 1.0 - w_primary);

        // ---- both servers compute; the stack blends -------------------------
        const double u_cmd = stack->compute(ref - x);

        // ---- master -> PLC ---------------------------------------------------
        down.send(u_cmd, t);
        double rx = 0.0;
        if (down.tryReceive(rx, t)) u_hold = rx;

        // ---- bump accounting -------------------------------------------------
        // Two numbers matter: the worst SINGLE-TICK step during the handover, and the TOTAL
        // command change across it. A hard switch would deliver the whole total in one tick;
        // a bumpless ramp spreads it over kRampTime/kTs ticks.
        if (do_failover) {
            if (!captured_before && t >= t_detect - kTs) {
                u_before_handover = u_hold;
                captured_before   = true;
            }
            if (t >= t_detect - kTs && t <= t_settled) {
                r.max_bump    = std::max(r.max_bump, std::abs(u_hold - u_prev_applied));
                r.handover_gap = std::abs(u_hold - u_before_handover);
            }
        }
        u_prev_applied = u_hold;

        // ---- plant ------------------------------------------------------------
        x += kTs * (-kA * x + kB * u_hold);

        r.iae += std::abs(ref - x) * kTs;
    }

    r.final_err     = std::abs(kRefStep - x);
    r.w_primary_end = w_primary;
    return r;
}

}  // namespace

int main()
{
    std::printf("=== ex134: bumpless transfer between redundant control servers ===\n\n");
    std::printf("plant     : x' = -%.1f x + %.1f u   (tau = %.2f s)\n", kA, kB, 1.0 / kA);
    std::printf("servers   : primary + hot standby, identical laws, Weighted ControllerStack\n");
    std::printf("scenario  : setpoint %.1f -> %.1f at t = %.2f s; primary wedges MID-TRANSIENT\n"
                "            at t = %.2f s (output freezes on a command that is already wrong)\n",
                kRefInitial, kRefStep, kStepTime, kHangTime);
    std::printf("supervisor: heartbeat timeout %.0f ms, then a %.0f ms bumpless weight ramp\n",
                kDetectLag * 1e3, kRampTime * 1e3);
    std::printf("transport : AtomicParamBuffer (supervisor -> RT loop), NetworkChannel"
                " (master -> PLC)\n\n");

    const ArmResult stuck  = runArm(/*do_failover=*/false);
    const ArmResult failed = runArm(/*do_failover=*/true);

    std::printf("  %-24s %10s %12s %12s\n", "arm", "IAE", "final |e|", "w_primary");
    std::printf("  %-24s %10.4f %12.4f %12.2f\n",
                "wedged, no failover", stuck.iae, stuck.final_err, stuck.w_primary_end);
    std::printf("  %-24s %10.4f %12.4f %12.2f\n",
                "wedged + failover", failed.iae, failed.final_err, failed.w_primary_end);

    std::printf("\n  failover started at t = %.3f s, %u supervisor publishes through"
                " AtomicParamBuffer\n", failed.failover_time, failed.publishes);
    std::printf("  total command change across handover : %.5f"
                " (a hard switch delivers this in ONE tick)\n", failed.handover_gap);
    std::printf("  worst single-tick step during it     : %.5f\n", failed.max_bump);

    // -- acceptance -------------------------------------------------------------
    // Bumpless is measured against what a HARD switch would have done: the ramp must spread the
    // command change over many ticks rather than delivering it in one. The gap must also be
    // non-trivial, otherwise there was nothing to transfer and the test proves nothing - which
    // is exactly what happened when the primary wedged at steady state instead of mid-transient.
    const bool gap_nontrivial = failed.handover_gap > 0.05;
    const bool bumpless       = failed.max_bump < 0.35 * failed.handover_gap;
    const bool recovered      = failed.final_err < 0.02;
    const bool worth_it       = failed.iae < 0.75 * stuck.iae;   // a real margin, not noise
    const bool switched       = failed.w_primary_end < 1e-9;
    const bool published      = failed.publishes > 0;

    std::printf("\n  handover non-trivial = %s (gap %.5f > 0.05, else nothing to transfer)\n",
                gap_nontrivial ? "yes" : "no", failed.handover_gap);
    std::printf("  handover bumpless    = %s (worst tick %.5f < 35%% of the %.5f gap)\n",
                bumpless ? "yes" : "no", failed.max_bump, failed.handover_gap);
    std::printf("  standby took over    = %s (final w_primary = %.2f)\n",
                switched ? "yes" : "no", failed.w_primary_end);
    std::printf("  recovered setpoint   = %s (final |e| = %.4f < 0.02)\n",
                recovered ? "yes" : "no", failed.final_err);
    std::printf("  better than wedged   = %s (IAE %.4f < 75%% of %.4f)\n",
                worth_it ? "yes" : "no", failed.iae, stuck.iae);
    std::printf("  AtomicParamBuffer    = %s (%u publishes, one lock-free read per tick)\n",
                published ? "exercised" : "unused", failed.publishes);

    const bool ok = gap_nontrivial && bumpless && recovered && worth_it && switched && published;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
