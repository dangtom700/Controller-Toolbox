#pragma once
#include <string>
#include <cmath>
#include <cstdint>

namespace susp {

enum class RoadType {
    STEP,           // single step bump (z_r = A for t >= t_step)
    SINE_RESONANCE, // sinusoidal at body resonance frequency
    ROUGH_ROAD,     // high-frequency sine near wheel hop
    SPEED_BUMP,     // versine profile (one bump at given speed)
    COMPOUND        // sinusoidal body resonance + step at t_step
};

struct ScenarioConfig {
    std::string id;
    std::string description;
    RoadType    road_type       = RoadType::STEP;
    double      amplitude       = 0.025;  // road amplitude [m]
    double      frequency       = 1.3;    // sinusoidal frequency [Hz]
    double      vehicle_speed   = 0.0;    // vehicle speed for bump [m/s]
    double      bump_length     = 0.5;    // versine bump length [m]
    double      step_time       = 0.5;    // time of step onset [s]
    double      step_height     = 0.015;  // step height for compound [m]
    double      duration_s      = 5.0;    // scenario duration [s]
    uint32_t    seed            = 42;

    static ScenarioConfig fromJson(const std::string& path);
};

// Road surface height z_r(t) generator.
class RoadProfile {
public:
    explicit RoadProfile(const ScenarioConfig& cfg) : cfg_(cfg) {}

    // Road displacement [m] at time t [s].
    double compute(double t) const;

private:
    ScenarioConfig cfg_;
};

} // namespace susp
