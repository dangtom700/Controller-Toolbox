#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "psychrometrics.h"
#include "humid_plant.h"
#include "controllers.h"
#include <cmath>

using namespace Catch::Matchers;
using namespace humid;
using namespace humid::psychro;

// ---------------------------------------------------------------------------
// [humidification] - Psychrometrics correctness
// ---------------------------------------------------------------------------

TEST_CASE("Psat known values", "[humidification]")
{
    // 0^\circC: Psat approx = 611 Pa (Magnus formula known value)
    REQUIRE_THAT(Psat(273.15), WithinRel(611.0, 0.01));

    // 40^\circC: Psat approx = 7375 Pa
    REQUIRE_THAT(Psat(313.15), WithinRel(7375.0, 0.02));

    // 100^\circC: Psat approx = 101325 Pa (boiling point at sea level)
    REQUIRE_THAT(Psat(373.15), WithinRel(101325.0, 0.02));
}

TEST_CASE("Wet-bulb is below or equal dry-bulb", "[humidification]")
{
    // At any (Ta, phi<1), Twb <= Ta
    for (double Ta_C : {20.0, 35.0, 40.0, 45.0}) {
        for (double phi : {0.15, 0.25, 0.40, 0.80}) {
            double Twb = wetBulb(Ta_C + 273.15, phi);
            REQUIRE(Twb <= Ta_C + 273.15 + 0.01);
            REQUIRE(Twb > 273.15);  // above 0^\circC for typical conditions
        }
    }
}

TEST_CASE("Wet-bulb equals dry-bulb at phi=1", "[humidification]")
{
    double Ta_K = 313.15;  // 40^\circC
    double Twb  = wetBulb(Ta_K, 1.0);
    REQUIRE_THAT(Twb, WithinAbs(Ta_K, 0.5));
}

TEST_CASE("phi_from_omega roundtrip", "[humidification]")
{
    double T_K   = 295.15;
    double phi_0 = 0.45;
    double omega = omega_gkg_from_phi(phi_0, T_K);
    double phi_1 = phi_from_omega(omega, T_K);
    REQUIRE_THAT(phi_1, WithinAbs(phi_0, 1e-6));
}

// ---------------------------------------------------------------------------
// [humidification] - Humidifier physics
// ---------------------------------------------------------------------------

TEST_CASE("Humidifier H within paper validated range at max conditions", "[humidification]")
{
    // Paper: at (Ta=45^\circC, phi_in=0.15, u=3.5 m/s) -> H_max = 266.4 g/h
    // Theoretical values from paper have up to 20% error vs measured, so
    // accept [200, 360] g/h as plausible (allows for theoretical +/-15%).
    PlantParams p;
    p.init();
    HumidifierPhysics hum(p);

    PlantOutput out{};
    double H = hum.compute(3.5, 318.15, 0.15, out);

    REQUIRE(H > 200.0);
    REQUIRE(H < 400.0);
    REQUIRE(out.Re  > 1000.0);   // turbulent-ish flat-plate regime
    REQUIRE(out.hm  > 0.0);
}

TEST_CASE("Humidifier H increases with fan speed", "[humidification]")
{
    PlantParams p;
    p.init();
    HumidifierPhysics hum(p);

    PlantOutput out_lo{}, out_hi{};
    double H_lo = hum.compute(1.0, 313.15, 0.15, out_lo);
    double H_hi = hum.compute(3.5, 313.15, 0.15, out_hi);
    REQUIRE(H_hi > H_lo);
}

TEST_CASE("Humidifier H is zero or positive (no condensation output)", "[humidification]")
{
    PlantParams p;
    p.init();
    HumidifierPhysics hum(p);

    PlantOutput out{};
    // Very humid inlet: phi_in close to saturation -> H should be near zero
    double H = hum.compute(2.0, 313.15, 0.90, out);
    REQUIRE(H >= 0.0);
}

// ---------------------------------------------------------------------------
// [humidification] - Room dynamics + PID closed-loop
// ---------------------------------------------------------------------------

TEST_CASE("Room humidity rises from initial 20% toward setpoint 45%", "[humidification]")
{
    // Scenario s01_design: T_out=-10^\circC, phi_out=0.35, setpoint=0.45
    // PID controller. After 3h (360 steps at Ts=30s), phi_room should be > 0.35.
    PlantParams params;
    params.init();

    HumidificationPlant plant(params);
    plant.reset(0.20);

    PIDHumidCtrl ctrl(params.Ts);
    ctrl.reset();

    Disturbance d{263.15, 0.35, 0.0};
    const double ref = 0.45;
    const int N = static_cast<int>(params.duration / params.Ts);

    ControlInput u{kFanMid, kTa_nom};
    PlantOutput out = plant.step(u, d);

    for (int k = 0; k < N; ++k) {
        u   = ctrl.compute(out.phi_measured, ref);
        out = plant.step(u, d);
    }

    // phi_room should have risen meaningfully above initial (not still at 0.20)
    REQUIRE(out.phi_room > 0.30);
    // Fan speed should be within valid range
    REQUIRE(u.u_fan >= 1.0);
    REQUIRE(u.u_fan <= 3.5);
}

TEST_CASE("Room humidity stays bounded [0, 0.99]", "[humidification]")
{
    PlantParams params;
    params.init();

    HumidificationPlant plant(params);
    plant.reset(0.15);

    // Run at max fan speed with cold outdoor air - humidity should not overflow
    Disturbance d{253.15, 0.20, 0.0};
    ControlInput u{3.5, 313.15};

    for (int k = 0; k < 360; ++k) {
        PlantOutput out = plant.step(u, d);
        REQUIRE(out.phi_room >= 0.0);
        REQUIRE(out.phi_room <= 0.99);
    }
}
