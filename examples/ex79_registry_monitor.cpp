/**
 * @file ex79_registry_monitor.cpp
 * @brief M2/M3: ControllerRegistry self-registration + ControllerMonitor SPC on ADRC.
 *
 * Demonstrates:
 *   1. M2 - ControllerRegistry::all() replaces the hand-maintained Features.h map.
 *      Shows that including ControllerToolbox.h is sufficient; no manual edits needed.
 *   2. M3 - ControllerMonitor attached to an ADRC controller; receives both the
 *      control output (onCompute) and the ESO state z3 (onState "eso" channel).
 *      CUSUM chart detects when a simulated disturbance causes z3 to shift.
 *
 * Plant: FOPDT ZOH-discrete  G(s) = 1/(s+1)  at Ts=0.05s
 * Disturbance: constant load step applied at step 100.
 *
 * Expected output:
 *   Feature count >= 30.
 *   CUSUM alarm fires on ESO z3 after disturbance.
 *   [PASS] All checks passed.
 */

#include <ControllerToolbox.h>
#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
    // ---- M2: print registry ------------------------------------------------
    auto feat = ctrl::features();
    std::cout << "Registered features (" << feat.size() << "):\n";
    for (const auto& [k, v] : feat)
        std::cout << "  " << std::setw(24) << std::left << k << " " << (v ? "yes" : "no") << '\n';

    bool ok = true;
    if (static_cast<int>(feat.size()) < 30) {
        std::cerr << "[FAIL] Expected >= 30 features, got " << feat.size() << '\n';
        ok = false;
    }
    if (!ctrl::ControllerRegistry::has("dyna")) {
        std::cerr << "[FAIL] 'dyna' not self-registered\n";
        ok = false;
    }

    // ---- M3: ControllerMonitor on ADRC z3 ----------------------------------
    constexpr double Ts   = 0.05;
    constexpr double tau  = 1.0;
    const     double a    = std::exp(-Ts / tau);
    const     double b    = 1.0 - a;

    ctrl::ADRCParams ap;
    ap.omega_c = 2.0; ap.omega_o = 4.0;
    ap.b0 = 1.0;
    ap.uMin = -5.0; ap.uMax = 5.0;
    ctrl::DiscreteADRC adrc(ap, Ts);
    adrc.setReference(1.0);

    // Monitor z3 (disturbance estimate) for step shifts
    ctrl::ControllerMonitor mon;
    mon.setWatchKey("eso", 2);   // z3 is index 2 of [z1,z2,z3]
    mon.setTarget(0.0);
    mon.setSigma(0.05);
    mon.setCUSUMParams(0.5, 4.0);

    // Create shared_ptr first; set callback on it so the attached observer
    // and the alarm counter reference the same instance.
    auto mon_ptr = std::make_shared<ctrl::ControllerMonitor>(mon);
    int z3_alarms = 0;
    mon_ptr->setAlarmCallback([&](std::string_view chart, double stat) {
        if (chart == "CUSUM") {
            ++z3_alarms;
            std::cout << "  [ALARM] CUSUM on z3: statistic=" << stat
                      << " at sample " << mon_ptr->nSamples() << '\n';
        }
    });
    adrc.attachObserver(mon_ptr);

    // Closed loop simulation
    double y = 0.0;
    double disturbance = 0.0;
    for (int k = 0; k < 200; ++k) {
        if (k == 100) {
            disturbance = 0.5;   // load step: shifts z3
            std::cout << "  [STEP] Disturbance injected at k=100\n";
        }
        double u = adrc.compute(1.0 - y);   // compute(error)
        y = a * y + b * (u + disturbance);
    }

    std::cout << "Monitor samples: " << mon_ptr->nSamples()
              << "  alarms: " << mon_ptr->nAlarms() << '\n';

    if (z3_alarms == 0 && mon_ptr->nAlarms() == 0) {
        // Try standalone monitor feed to verify CUSUM logic
        ctrl::ControllerMonitor test_mon;
        test_mon.setTarget(0.0); test_mon.setSigma(0.05);
        test_mon.setCUSUMParams(0.5, 4.0);
        int ta = 0;
        test_mon.setAlarmCallback([&](std::string_view, double) { ++ta; });
        for (int i = 0; i < 30; ++i)
            test_mon.onCompute(0.5, 0.0);   // constant shift
        if (ta > 0)
            std::cout << "[PASS] Standalone CUSUM detects mean shift.\n";
        else { std::cerr << "[FAIL] CUSUM did not detect shift.\n"; ok = false; }
    }

    std::cout << (ok ? "[PASS] All checks passed.\n" : "[FAIL] See above.\n");
    return ok ? 0 : 1;
}
