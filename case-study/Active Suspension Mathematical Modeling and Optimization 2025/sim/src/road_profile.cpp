#include "road_profile.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

using json = nlohmann::json;

namespace susp {

// ---------------------------------------------------------------------------
// ScenarioConfig JSON loader
// ---------------------------------------------------------------------------

static RoadType parseRoadType(const std::string& s)
{
    if (s == "step")            return RoadType::STEP;
    if (s == "sine_resonance")  return RoadType::SINE_RESONANCE;
    if (s == "rough_road")      return RoadType::ROUGH_ROAD;
    if (s == "speed_bump")      return RoadType::SPEED_BUMP;
    if (s == "compound")        return RoadType::COMPOUND;
    throw std::runtime_error("Unknown road_type: " + s);
}

ScenarioConfig ScenarioConfig::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open scenario: " + path);
    json cfg = json::parse(f);

    ScenarioConfig s;
    s.id            = cfg.value("id",            "unknown");
    s.description   = cfg.value("description",   "");
    s.road_type     = parseRoadType(cfg.value("road_type", "step"));
    s.amplitude     = cfg.value("amplitude",      0.025);
    s.frequency     = cfg.value("frequency",      1.3);
    s.vehicle_speed = cfg.value("vehicle_speed",  8.33);
    s.bump_length   = cfg.value("bump_length",    0.5);
    s.step_time     = cfg.value("step_time",      0.5);
    s.step_height   = cfg.value("step_height",    0.015);
    s.duration_s    = cfg.value("duration_s",     5.0);
    s.seed          = cfg.value("seed",           42u);
    return s;
}

// ---------------------------------------------------------------------------
// RoadProfile::compute
// ---------------------------------------------------------------------------

double RoadProfile::compute(double t) const
{
    const auto& c = cfg_;

    switch (c.road_type) {

    case RoadType::STEP:
        // Smooth-step: blend over 10 ms to avoid instantaneous acceleration impulse
        if (t < c.step_time)       return 0.0;
        if (t < c.step_time + 0.01) {
            double tau = (t - c.step_time) / 0.01;
            return c.amplitude * (3.0*tau*tau - 2.0*tau*tau*tau);  // smoothstep
        }
        return c.amplitude;

    case RoadType::SINE_RESONANCE:
        // Sinusoidal at body resonance (~1.3 Hz)
        return c.amplitude * std::sin(2.0 * M_PI * c.frequency * t);

    case RoadType::ROUGH_ROAD:
        // High-frequency sinusoidal near wheel hop (~8 Hz)
        return c.amplitude * std::sin(2.0 * M_PI * c.frequency * t);

    case RoadType::SPEED_BUMP: {
        // Versine profile: z_r = A/2*(1-cos(pi*v*t/L)) for 0 <= t <= L/v
        const double t_bump = c.bump_length / c.vehicle_speed;
        if (t < 0.0 || t > t_bump) return 0.0;
        return (c.amplitude / 2.0) * (1.0 - std::cos(M_PI * t / t_bump));
    }

    case RoadType::COMPOUND:
        // Sinusoidal at body resonance + step at t_step
        {
            double sine_part = c.amplitude * std::sin(2.0 * M_PI * c.frequency * t);
            double step_part = 0.0;
            if (t >= c.step_time) {
                double tau = std::min((t - c.step_time) / 0.01, 1.0);
                step_part = c.step_height * (3.0*tau*tau - 2.0*tau*tau*tau);
            }
            return sine_part + step_part;
        }

    default:
        return 0.0;
    }
}

} // namespace susp
