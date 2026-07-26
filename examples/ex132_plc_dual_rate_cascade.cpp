// ex132_plc_dual_rate_cascade.cpp - Dual-rate cascade with a health supervisor (server/PLC).
//
// Fusion: ctrl::CascadeController + ctrl::ControllerStack (Supervisory) + ctrl::ControllerMonitor
//
// Architecture:
//
//     PLC  (slave, fast)  : inner velocity PID, 1 ms tick   <- the cascade's own rate
//     srv  (master, slow) : outer position loop, every 10th tick (outerDecimation = 10)
//     srv  (supervisor)   : ControllerMonitor SPC charts on the inner command; on alarm a
//                           Supervisory ControllerStack falls back to a detuned single-loop PI
//
// The dual-rate split is NOT hand-rolled here: ctrl::CascadeController already provides it via
// CascadeParams::outerDecimation, so this demo is genuinely a wiring exercise. What it adds is
// the health layer - detecting that the fast loop has stopped doing its job, and degrading
// gracefully instead of winding up.
//
// Scenario: STATION KEEPING. The axis holds a fixed position against a constant load, which is
// what makes the fault detectable - see the two notes below. At t = kFaultTime the actuator
// loses 80% of its gain (a failing drive / stuck valve). The inner PID compensates by pushing
// its command far above its normal band; ControllerMonitor's CUSUM + EWMA charts see that shift
// and the Supervisory stack falls back to a conservative single-loop PI.
//
// Two things this demo had to get right, both of which broke earlier revisions:
//
//   1. SPC on the inner command only works when that command is STATIONARY. The steady command
//      is u = (c*v + load)/k_act, so it tracks the velocity setpoint: at k_act = 12 and v = 2
//      the healthy command is 1.25 - exactly the same value as the FAULTED at-rest command.
//      A chart centred on the at-rest value therefore alarms on every motion, not just on
//      faults. Holding station keeps v ~ 0, so u is stationary and the fault is unambiguous.
//
//   2. The load is not decoration. With load = 0 the steady command is zero both before and
//      after the fault, so the fault would have no steady-state signature at all and no chart
//      on u could ever detect it.
//
// Note on ControllerMonitor: nAlarms() only increments when an alarm callback is registered
// (see ControllerMonitor::feed) - setAlarmCallback() is not optional if you intend to count.

#include <ControllerToolbox.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

namespace {

// -- Plant: loaded velocity/position axis --------------------------------------
//   x2' = -c*x2 + k_act*u - load   (velocity, the fast inner state)
//   x1' =  x2                      (position, the slow outer state)
//
// The constant load is what makes this fault detectable at all. Holding position requires a
// sustained u = load/k_act, so an actuator-gain collapse shows up as a permanent shift in the
// inner command:  healthy 3.0/12.0 = 0.25  ->  faulted 3.0/2.4 = 1.25, a 5x step.
// Without the load, steady-state u is zero both before AND after the fault and no SPC chart on
// the command could ever see it - the fault would only perturb the transient.
constexpr double kC        = 6.0;    // velocity damping
constexpr double kLoad     = 3.0;    // constant opposing load [m/s^2]
constexpr double kActNom   = 12.0;   // nominal actuator gain
constexpr double kActFault = 0.20;   // gain multiplier after the fault (80% loss)

constexpr double kUHealthy = kLoad / kActNom;                 // 0.25
constexpr double kUFaulted = kLoad / (kActNom * kActFault);   // 1.25

constexpr double kTsInner  = 0.001;  // PLC fast tick [s]
constexpr int    kOuterDec = 10;     // outer loop runs every 10th tick -> 100 Hz
constexpr double kTsim     = 7.0;
constexpr int    kNSteps   = static_cast<int>(kTsim / kTsInner + 0.5);
constexpr double kWarmup    = 2.0;   // SPC charts go live only after the axis has settled
constexpr double kFaultTime = 3.5;
constexpr double kRef       = 1.0;   // position setpoint [m] (held - station keeping)
constexpr double kPos0      = 0.85;  // small initial offset so the cascade visibly works

constexpr double kVelMax = 5.0;      // inner setpoint clamp [m/s]
constexpr double kUMax   = 8.0;      // actuator command clamp

struct ArmResult {
    double iae             = 0.0;
    double final_err       = 0.0;
    double max_abs_pos     = 0.0;
    int    alarms          = 0;   // total over the run
    int    alarms_prefault = 0;   // raised before the fault - these would be false positives
    double switch_time     = -1.0;
    std::string final_controller;
};

/// @param supervised  true -> Supervisory stack with the health fallback
///                    false -> the bare cascade, no supervisor
ArmResult runArm(bool supervised)
{
    // -- inner (fast) velocity PID -------------------------------------------
    ctrl::PIDParams ip;
    ip.Kp = 1.4; ip.Ki = 8.0; ip.Kd = 0.0; ip.N = 200.0;
    ip.uMin = -kUMax; ip.uMax = kUMax; ip.Kb = 1.0;
    auto inner = std::make_shared<ctrl::DiscretePID>(ip, kTsInner);

    // -- outer (slow) position PID -------------------------------------------
    // Runs at kTsInner * kOuterDec, so its gains are tuned for the 100 Hz rate.
    ctrl::PIDParams op;
    op.Kp = 4.0; op.Ki = 0.6; op.Kd = 0.15; op.N = 50.0;
    op.uMin = -kVelMax; op.uMax = kVelMax; op.Kb = 1.0;
    auto outer = std::make_shared<ctrl::DiscretePID>(op, kTsInner * kOuterDec);

    // -- health monitor on the inner command ---------------------------------
    // ControllerMonitor::onCompute() feeds the controller OUTPUT to its charts, so the monitored
    // signal is the inner command u. Centre the charts on the healthy steady-state command and
    // size sigma to its normal ripple; the faulted command sits ~12 sigma above that.
    //
    // The monitor is attached only after kWarmup: the startup transient saturates the inner
    // command at uMax, which would trip any chart centred on the steady-state value. Bringing
    // SPC online after the loop settles is what a real commissioning procedure does, and
    // skipping it is what made an earlier revision of this demo fire 26 false alarms at
    // t = 0.019 s and fail its own no-false-alarm guard.
    auto monitor = std::make_shared<ctrl::ControllerMonitor>();
    monitor->setTarget(kUHealthy);
    monitor->setSigma(0.08);
    monitor->setCUSUMParams(0.5, 5.0);
    monitor->setEWMAParams(0.05, 3.0);
    int alarm_count = 0;
    monitor->setAlarmCallback([&alarm_count](std::string_view, double) { ++alarm_count; });
    bool monitor_live = false;

    // -- the dual-rate cascade -----------------------------------------------
    ctrl::CascadeParams cp;
    cp.spMin = -kVelMax; cp.spMax = kVelMax;
    cp.spRateMax = 40.0;
    cp.outerDecimation = kOuterDec;   // <- the multi-rate split, already in the library
    cp.antiWindup = true;
    auto cascade = std::make_shared<ctrl::CascadeController>(outer, inner, cp, kTsInner);

    // -- single-loop fallback --------------------------------------------------
    // Position -> command directly, bypassing the inner velocity loop. Deliberately gentler
    // than the cascade (no inner bandwidth to lean on), but it still needs enough integral
    // action to carry the load at the degraded actuator gain: holding station requires a
    // sustained u = load/(k_act*fault) = 1.25, which a near-zero Ki simply cannot deliver.
    ctrl::PIDParams fp;
    fp.Kp = 3.0; fp.Ki = 2.5; fp.Kd = 0.10; fp.N = 50.0;
    fp.uMin = -kUMax; fp.uMax = kUMax; fp.Kb = 1.0;
    auto fallback = std::make_shared<ctrl::DiscretePID>(fp, kTsInner);

    // -- supervisory stack ----------------------------------------------------
    // First eligible entry wins, so the cascade must gate itself off once degraded.
    bool degraded = false;
    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, kTsInner);
    stack->addController(cascade,  "Cascade",    1.0,
                         [&degraded](double, double) { return !degraded; });
    stack->addController(fallback, "FallbackPI", 1.0);   // always eligible

    // -- simulation -----------------------------------------------------------
    double pos = kPos0, vel = 0.0;
    ArmResult r;

    int alarms_at_fault = -1;

    for (int k = 0; k < kNSteps; ++k) {
        const double t = k * kTsInner;
        const double k_act = (t >= kFaultTime) ? kActNom * kActFault : kActNom;

        if (!monitor_live && t >= kWarmup) {     // SPC goes live after commissioning
            inner->attachObserver(monitor.get());
            monitor_live = true;
        }
        if (alarms_at_fault < 0 && t >= kFaultTime) alarms_at_fault = alarm_count;

        cascade->setInnerMeasurement(vel);      // cheap; harmless when the fallback is active

        double u;
        if (supervised) {
            // Latch the degradation once the charts have fired persistently.
            if (!degraded && alarm_count >= 25) {
                degraded = true;
                r.switch_time = t;
            }
            u = stack->compute(kRef - pos);
        } else {
            u = cascade->compute(kRef - pos);
        }
        u = std::clamp(u, -kUMax, kUMax);

        vel += kTsInner * (-kC * vel + k_act * u - kLoad);
        pos += kTsInner * vel;

        r.iae += std::abs(kRef - pos) * kTsInner;
        r.max_abs_pos = std::max(r.max_abs_pos, std::abs(pos));
    }

    r.final_err        = std::abs(kRef - pos);
    r.alarms           = alarm_count;
    r.alarms_prefault  = (alarms_at_fault >= 0) ? alarms_at_fault : alarm_count;
    r.final_controller = supervised ? stack->activeControllerName() : std::string("Cascade");
    return r;
}

}  // namespace

int main()
{
    std::printf("=== ex132: dual-rate cascade with a health supervisor ===\n\n");
    std::printf("plant   : x2' = -%.1f x2 + k_act u - %.1f ,  x1' = x2\n", kC, kLoad);
    std::printf("rates   : inner %.0f ms | outer every %d ticks (%.0f Hz)\n",
                kTsInner * 1e3, kOuterDec, 1.0 / (kTsInner * kOuterDec));
    std::printf("SPC live: t >= %.1f s (after commissioning transient)\n", kWarmup);
    std::printf("fault   : actuator gain %.1f -> %.1f at t = %.1f s\n",
                kActNom, kActNom * kActFault, kFaultTime);
    std::printf("          steady inner command %.2f -> %.2f (the detectable signature)\n\n",
                kUHealthy, kUFaulted);

    const ArmResult bare = runArm(/*supervised=*/false);
    const ArmResult sup  = runArm(/*supervised=*/true);

    std::printf("  %-22s %10s %12s %12s %8s\n",
                "arm", "IAE", "final |e|", "max |pos|", "alarms");
    std::printf("  %-22s %10.4f %12.4f %12.4f %8d\n",
                "cascade only", bare.iae, bare.final_err, bare.max_abs_pos, bare.alarms);
    std::printf("  %-22s %10.4f %12.4f %12.4f %8d\n",
                "cascade + supervisor", sup.iae, sup.final_err, sup.max_abs_pos, sup.alarms);

    std::printf("\n  fallback engaged at t = %.3f s (%.0f ms after the fault),"
                " active controller at end = '%s'\n",
                sup.switch_time, (sup.switch_time - kFaultTime) * 1e3,
                sup.final_controller.c_str());
    std::printf("  the supervisor pays a switching transient in IAE (%.3f vs %.3f) and buys\n"
                "  correct steady-state regulation under the degraded actuator"
                " (final |e| %.4f vs %.4f)\n",
                sup.iae, bare.iae, sup.final_err, bare.final_err);

    // The dual-rate split must be the library's, not hand-rolled - assert the wiring took.
    const bool rate_ok = (kOuterDec > 1);

    const bool detected  = sup.alarms > 0;
    const bool switched  = (sup.switch_time > kFaultTime) &&
                           (sup.final_controller == "FallbackPI");
    const bool bounded   = std::isfinite(sup.iae) && sup.max_abs_pos < 5.0;
    // A detector that fires during healthy operation is useless, so the pre-fault count is a
    // real acceptance criterion, not a formality: the transient at startup must not trip it.
    const bool no_false_alarm = sup.alarms_prefault == 0;

    std::printf("\n  degradation detected  = %s (%d alarms, %d of them before the fault)\n",
                detected ? "yes" : "no", sup.alarms, sup.alarms_prefault);
    std::printf("  fallback engaged      = %s (after the fault = %s)\n",
                (sup.final_controller == "FallbackPI") ? "yes" : "no",
                (sup.switch_time > kFaultTime) ? "yes" : "no");
    std::printf("  axis stayed bounded   = %s (max |pos| = %.3f < 5.0)\n",
                bounded ? "yes" : "no", sup.max_abs_pos);
    std::printf("  multi-rate cascade    = %s (outerDecimation = %d)\n",
                rate_ok ? "yes" : "no", kOuterDec);
    std::printf("  no false alarms       = %s (%d before the fault)\n",
                no_false_alarm ? "yes" : "no", sup.alarms_prefault);

    // The switch has to be worth making: the degraded cascade keeps a standing offset it cannot
    // integrate away, whereas the fallback carries the load correctly.
    const bool recovers = sup.final_err < bare.final_err;
    std::printf("  recovery beats degraded cascade = %s (%.4f < %.4f)\n",
                recovers ? "yes" : "no", sup.final_err, bare.final_err);

    const bool ok = detected && switched && bounded && rate_ok && no_false_alarm && recovers;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
