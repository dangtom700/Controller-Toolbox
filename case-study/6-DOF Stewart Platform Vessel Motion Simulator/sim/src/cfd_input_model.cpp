#include "cfd_input_model.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>

namespace stewart {

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// MatrixTuning::fromJson
// ---------------------------------------------------------------------------
MatrixTuning MatrixTuning::fromJson(const std::string& path)
{
    MatrixTuning t;
    std::ifstream f(path);
    if (!f.is_open())
        return t; // code defaults are sufficient (regression tests rely on this)
    json j; f >> j;

    if (j.contains("duration_min_s"))             t.duration_min_s             = j["duration_min_s"].get<double>();
    if (j.contains("duration_max_s"))             t.duration_max_s             = j["duration_max_s"].get<double>();
    if (j.contains("duration_period_multiplier")) t.duration_period_multiplier = j["duration_period_multiplier"].get<double>();
    if (j.contains("workspace_margin"))           t.workspace_margin           = j["workspace_margin"].get<double>();
    if (j.contains("swell_period_ratio"))         t.swell_period_ratio         = j["swell_period_ratio"].get<double>();
    if (j.contains("swell_amplitude_ratio"))      t.swell_amplitude_ratio      = j["swell_amplitude_ratio"].get<double>();

    auto readWeights = [&](const char* key, MatrixTuning::DirWeights& w) {
        if (!j.contains("direction_weights") || !j["direction_weights"].contains(key)) return;
        const auto& dw = j["direction_weights"][key];
        if (dw.contains("pitch")) w.pitch = dw["pitch"].get<double>();
        if (dw.contains("roll"))  w.roll  = dw["roll"].get<double>();
        if (dw.contains("surge")) w.surge = dw["surge"].get<double>();
        if (dw.contains("sway"))  w.sway  = dw["sway"].get<double>();
        if (dw.contains("yaw"))   w.yaw   = dw["yaw"].get<double>();
    };
    readWeights("head",      t.head);
    readWeights("following", t.following);
    readWeights("beam",      t.beam);

    return t;
}

// ---------------------------------------------------------------------------
// Empirical laws calibrated against the paper's 3 (Hs, T)/(Hs, amplitude) points
// (1.25 m -> 3.8 s / 0.112 m / 0.148 deg / 1.18 deg;
//  2.50 m -> 5.4 s / 0.208 m / 0.307 deg / 2.27 deg;
//  4.00 m -> 7.0 s / 0.991 m / 0.662 deg / 4.32 deg).
// ---------------------------------------------------------------------------

double periodFromHs(double Hs)
{
    return 3.45 * std::sqrt(std::max(Hs, 0.05));
}

namespace {

// Piecewise-linear interpolation through anchor points, extrapolating the last
// segment's slope beyond the final anchor (preserves the paper's observed
// accelerating growth rather than washing it out with a single power-law fit).
double piecewiseLinear(double x, const std::array<double, 4>& xs, const std::array<double, 4>& ys)
{
    if (x <= xs[0]) return ys[0];
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        if (x <= xs[i + 1]) {
            const double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }
    // Extrapolate using the final segment's slope.
    const size_t n = xs.size();
    const double slope = (ys[n - 1] - ys[n - 2]) / (xs[n - 1] - xs[n - 2]);
    return ys[n - 1] + slope * (x - xs[n - 1]);
}

} // namespace

double heaveAmplitudeFromHs(double Hs)
{
    static const std::array<double, 4> xs{0.0, 1.25, 2.5, 4.0};
    static const std::array<double, 4> ys{0.0, 0.112, 0.208, 0.991};
    return piecewiseLinear(Hs, xs, ys);
}

double rollAmplitudeFromHs(double Hs)
{
    static const std::array<double, 4> xs{0.0, 1.25, 2.5, 4.0};
    static const std::array<double, 4> ys{0.0, 0.148, 0.307, 0.662};
    return piecewiseLinear(Hs, xs, ys);
}

double pitchAmplitudeFromHs(double Hs)
{
    static const std::array<double, 4> xs{0.0, 1.25, 2.5, 4.0};
    static const std::array<double, 4> ys{0.0, 1.18, 2.27, 4.32};
    return piecewiseLinear(Hs, xs, ys);
}

// ---------------------------------------------------------------------------
// CFDInputModel
// ---------------------------------------------------------------------------
CFDInputModel::CFDInputModel(const SeaStateConfig& cfg, const PlantParams& plant,
                             const MatrixTuning& tuning)
    : cfg_(cfg), z0_mid_(plant.z0_mid)
{
    T_primary_ = periodFromHs(cfg_.Hs);
    T_swell_   = tuning.swell_period_ratio * T_primary_;
    swell_amplitude_ratio_ = tuning.swell_amplitude_ratio;

    const double heave_m   = heaveAmplitudeFromHs(cfg_.Hs);
    const double roll_deg  = rollAmplitudeFromHs(cfg_.Hs);
    const double pitch_deg = pitchAmplitudeFromHs(cfg_.Hs);

    // Surge/sway/yaw are not given numerically in the paper - small documented
    // fractions of heave/roll (decision #7).
    const double surge_base_m = 0.05 * heave_m;
    const double sway_base_m  = 0.03 * heave_m;
    const double yaw_base_deg = 0.10 * roll_deg;

    const MatrixTuning::DirWeights* w = nullptr;
    switch (cfg_.direction) {
        case WaveDirection::Head:      w = &tuning.head;      break;
        case WaveDirection::Following: w = &tuning.following; break;
        case WaveDirection::Beam:      w = &tuning.beam;      break;
    }

    double surge_m  = surge_base_m * w->surge;
    double sway_m   = sway_base_m  * w->sway;
    double heave_m_w= heave_m;            // heave is direction-independent
    double roll_deg_w  = roll_deg  * w->roll;
    double pitch_deg_w = pitch_deg * w->pitch;
    double yaw_deg_w   = yaw_base_deg * w->yaw;

    // Workspace-aware scaling (decision #8): include the swell contribution in
    // the peak-amplitude estimate so the combined (primary+swell) signal never
    // exceeds the limit either.
    const double swell_mult = cfg_.swell ? (1.0 + tuning.swell_amplitude_ratio) : 1.0;
    const auto& lim = plant.workspace;

    double worst_ratio = 0.0;
    worst_ratio = std::max(worst_ratio, (surge_m * swell_mult)      / lim.surge_max);
    worst_ratio = std::max(worst_ratio, (sway_m  * swell_mult)      / lim.sway_max);
    worst_ratio = std::max(worst_ratio, (heave_m_w * swell_mult)    / lim.heave_max);
    worst_ratio = std::max(worst_ratio, (roll_deg_w * swell_mult)   / lim.roll_max_deg);
    worst_ratio = std::max(worst_ratio, (pitch_deg_w * swell_mult)  / lim.pitch_max_deg);
    worst_ratio = std::max(worst_ratio, (yaw_deg_w * swell_mult)    / lim.yaw_max_deg);

    scale_factor_ = (worst_ratio > tuning.workspace_margin)
                        ? (tuning.workspace_margin / worst_ratio)
                        : 1.0;

    amp_surge_     = surge_m  * scale_factor_;
    amp_sway_      = sway_m   * scale_factor_;
    amp_heave_     = heave_m_w* scale_factor_;
    amp_roll_rad_  = (roll_deg_w  * scale_factor_) * DEG2RAD;
    amp_pitch_rad_ = (pitch_deg_w * scale_factor_) * DEG2RAD;
    amp_yaw_rad_   = (yaw_deg_w   * scale_factor_) * DEG2RAD;
}

PoseRef CFDInputModel::poseAt(double t) const
{
    const double w1 = 2.0 * PI / T_primary_;
    const double w2 = 2.0 * PI / T_swell_;

    // Fixed per-DOF phase offsets (documented heuristic - real vessel motions
    // are not all in phase; e.g. pitch typically leads heave).
    constexpr double PH_SURGE = 0.0;
    constexpr double PH_SWAY  = PI / 2.0;
    constexpr double PH_HEAVE = 0.0;
    constexpr double PH_ROLL  = PI / 2.0;
    constexpr double PH_PITCH = PI / 4.0;
    constexpr double PH_YAW   = 3.0 * PI / 4.0;

    auto wave = [&](double amp, double phase) {
        double v = amp * std::sin(w1 * t + phase);
        if (cfg_.swell)
            v += swell_amplitude_ratio_ * amp * std::sin(w2 * t + phase);
        return v;
    };

    PoseRef pose;
    pose.P(0) = wave(amp_surge_, PH_SURGE);
    pose.P(1) = wave(amp_sway_,  PH_SWAY);
    pose.P(2) = z0_mid_ + wave(amp_heave_, PH_HEAVE);
    pose.rpy(0) = wave(amp_roll_rad_,  PH_ROLL);
    pose.rpy(1) = wave(amp_pitch_rad_, PH_PITCH);
    pose.rpy(2) = wave(amp_yaw_rad_,   PH_YAW);
    return pose;
}

// ---------------------------------------------------------------------------
// buildSeaStateMatrix
// ---------------------------------------------------------------------------
std::vector<SeaStateConfig> buildSeaStateMatrix(const std::string& matrix_json_path,
                                                  const PlantParams& plant)
{
    static const std::array<double, 10> kDouglasHs{
        0.0, 0.05, 0.30, 0.875, 1.875, 3.25, 5.0, 7.5, 11.5, 16.0
    };

    MatrixTuning tuning = matrix_json_path.empty()
                             ? MatrixTuning{}
                             : MatrixTuning::fromJson(matrix_json_path);

    const std::array<std::pair<WaveDirection, std::string>, 3> directions{{
        {WaveDirection::Head,      "head"},
        {WaveDirection::Following, "following"},
        {WaveDirection::Beam,      "beam"},
    }};

    std::vector<SeaStateConfig> matrix;
    matrix.reserve(60);

    for (int s = 0; s < 10; ++s) {
        const double Hs = kDouglasHs[s];
        const double T  = periodFromHs(Hs);
        const double duration_s = std::clamp(tuning.duration_period_multiplier * T,
                                              tuning.duration_min_s, tuning.duration_max_s);

        for (const auto& [dir, dir_name] : directions) {
            for (bool swell : {false, true}) {
                SeaStateConfig cfg;
                cfg.douglas_state  = s;
                cfg.Hs             = Hs;
                cfg.direction      = dir;
                cfg.swell          = swell;
                cfg.equipment_load = true;
                cfg.duration_s     = duration_s;

                std::ostringstream id;
                id << "ss" << (s < 10 ? "0" : "") << s << "_" << dir_name
                   << (swell ? "_swell" : "_noswell");
                cfg.id = id.str();

                std::ostringstream desc;
                desc << "Douglas sea state " << s << " (Hs=" << Hs << " m), "
                     << dir_name << " seas, " << (swell ? "with swell" : "no swell");
                cfg.description = desc.str();

                matrix.push_back(cfg);
            }
        }
    }
    (void)plant; // reserved for future plant-dependent tuning
    return matrix;
}

} // namespace stewart
