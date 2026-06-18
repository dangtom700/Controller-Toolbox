#pragma once
#include "stewart_plant.h"
#include <string>
#include <vector>

// Synthetic CFD-input stand-in - architecturally separated from stewart_plant.h.
//
// Replaces the paper's LBM/XFlow CFD simulation with a parametric 6-DOF motion
// generator driven by a Douglas Sea Scale matrix: 10 sea states (0-9) x 3 wave
// directions (Head/Following/Beam) x 2 swell conditions = 60 configurations.
// See README "Implementation Notes" decisions #7/#8 for the full derivation:
//  - wave period law T(Hs) = 3.45*sqrt(max(Hs,0.05)), calibrated against the
//    paper's 3 actual (Hs,T) data points (1.25m->3.8s, 2.5m->5.4s, 4.0m->7.0s).
//  - amplitude law: piecewise-linear interpolation/extrapolation through the
//    paper's 3 actual (Hs, amplitude) points per DOF (heave/roll/pitch), anchored
//    at (0,0).
//  - direction weighting: fixed heuristic redistributing energy between
//    pitch/surge (head-dominant) and roll/sway (beam-dominant).
//  - swell: bichromatic superposition (primary + longer-period secondary).
//  - workspace-aware scaling: keeps every generated trajectory within the
//    paper's Table 1 workspace limits ("appropriate ratio" scaling, sec. 4.1).

namespace stewart {

enum class WaveDirection { Head, Following, Beam };

struct SeaStateConfig {
    int           douglas_state = 5;              // 0 (calm) .. 9 (phenomenal)
    double        Hs            = 3.25;            // representative significant wave height [m]
    WaveDirection direction     = WaveDirection::Head;
    bool          swell         = false;
    bool          equipment_load = true;
    double        duration_s   = 30.0;             // derived from T(Hs), clamped
    std::string   id           = "ss05_head_noswell";
    std::string   description  = "Douglas sea state 5 (Hs=3.25 m), Head seas, no swell";
};

// Tunable knobs read from config/scenarios/sea_state_matrix.json (all have code
// defaults so buildSeaStateMatrix() also works without the JSON file).
struct MatrixTuning {
    double duration_min_s             = 20.0;
    double duration_max_s             = 150.0;
    double duration_period_multiplier = 10.0;
    double workspace_margin           = 0.90;
    double swell_period_ratio         = 1.8;
    double swell_amplitude_ratio      = 0.4;

    struct DirWeights { double pitch, roll, surge, sway, yaw; };
    DirWeights head      {1.0, 0.3, 1.0, 0.2, 0.2};
    DirWeights following {0.8, 0.3, 0.8, 0.2, 0.3};
    DirWeights beam      {0.3, 1.0, 0.2, 1.0, 0.4};

    static MatrixTuning fromJson(const std::string& path);
};

// Wave period law T(Hs) = 3.45*sqrt(max(Hs,0.05)) [s].
double periodFromHs(double Hs);

// Piecewise-linear amplitude law through (0,0) + the paper's 3 (Hs,amplitude)
// anchor points, extrapolating the final segment's slope beyond Hs=4.0 m.
double heaveAmplitudeFromHs(double Hs); // [m]
double rollAmplitudeFromHs(double Hs);  // [deg]
double pitchAmplitudeFromHs(double Hs); // [deg]

class CFDInputModel {
public:
    CFDInputModel(const SeaStateConfig& cfg, const PlantParams& plant,
                  const MatrixTuning& tuning = MatrixTuning{});

    PoseRef poseAt(double t) const;

    // Workspace-margin scale factor applied to the raw amplitudes (<=1; <1 means
    // the requested sea state was clamped to stay within the Table-1 workspace).
    double scaleFactor() const { return scale_factor_; }

private:
    SeaStateConfig cfg_;
    double T_primary_ = 1.0;
    double T_swell_   = 1.0;
    double scale_factor_ = 1.0;
    double swell_amplitude_ratio_ = 0.4;

    // Post-direction-weighting, post-workspace-scaling amplitudes.
    double amp_surge_ = 0.0, amp_sway_ = 0.0, amp_heave_ = 0.0;
    double amp_roll_rad_ = 0.0, amp_pitch_rad_ = 0.0, amp_yaw_rad_ = 0.0;
    double z0_mid_ = 0.803;
};

// Cross-product builder: 10 Douglas states x 3 directions x 2 swell flags = 60
// SeaStateConfig entries. matrix_json_path may be empty to use code defaults.
std::vector<SeaStateConfig> buildSeaStateMatrix(const std::string& matrix_json_path,
                                                  const PlantParams& plant);

} // namespace stewart
