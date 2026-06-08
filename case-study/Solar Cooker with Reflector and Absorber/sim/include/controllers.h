#pragma once
#include "solar_cooker_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

// Solar cooker controller wrappers.
//
// Interface: compute(state, T_ref) -> f_shade in [0, 1]
//   state : [T_abs, T_pot]  (absorber and pot temperatures [^\circC])
//   T_ref : cooking target temperature [^\circC]
//
// Sign convention: e = T_pot - T_ref (positive = too hot -> shade more)
// All controllers clamp output to [0, 1] before returning.

namespace cooker {

static constexpr double F_MIN = 0.0;
static constexpr double F_MAX = 1.0;

inline double clampShade(double v) { return std::clamp(v, F_MIN, F_MAX); }

// ---------------------------------------------------------------------------
// Base class
// ---------------------------------------------------------------------------
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double      compute(const Eigen::Vector2d& state, double T_ref) = 0;
    virtual void        reset() = 0;
    virtual std::string name() const = 0;
};

// ---------------------------------------------------------------------------
// 1. OpenLoop - always fully open (f_shade = 0)
// ---------------------------------------------------------------------------
class OpenLoopCtrl : public ControllerBase {
public:
    explicit OpenLoopCtrl(const PlantParams&) {}
    double      compute(const Eigen::Vector2d&, double) override { return 0.0; }
    void        reset() override {}
    std::string name() const override { return "OpenLoop"; }
};

// ---------------------------------------------------------------------------
// 2. PID - e = T_pot - T_ref (direct acting; positive = too hot = more shade)
// ---------------------------------------------------------------------------
class PIDCookerCtrl : public ControllerBase {
public:
    explicit PIDCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "PID"; }
private:
    ctrl::DiscretePID pid_;
};

// ---------------------------------------------------------------------------
// 3. ADRC - direct acting: compute(T_pot - T_ref); b0 tuned for negative gain
// ---------------------------------------------------------------------------
class ADRCCookerCtrl : public ControllerBase {
public:
    explicit ADRCCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "ADRC"; }
private:
    ctrl::DiscreteADRC adrc_;
};

// ---------------------------------------------------------------------------
// 4. MPC - FOPDT linearised model; B negative (more shade -> lower T_pot)
// ---------------------------------------------------------------------------
class MPCCookerCtrl : public ControllerBase {
public:
    explicit MPCCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "MPC"; }
private:
    std::unique_ptr<ctrl::DiscreteMPC> mpc_;
    double T_pot_nom_;
};

// ---------------------------------------------------------------------------
// 5. FuzzyPID - e = T_pot - T_ref, direct acting
// ---------------------------------------------------------------------------
class FuzzyPIDCookerCtrl : public ControllerBase {
public:
    explicit FuzzyPIDCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "FuzzyPID"; }
private:
    ctrl::FuzzyPID fpid_;
};

// ---------------------------------------------------------------------------
// 6. SMC - compute(T_pot - T_ref) matches compute(y - ref) convention
// ---------------------------------------------------------------------------
class SMCCookerCtrl : public ControllerBase {
public:
    explicit SMCCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "SMC"; }
private:
    ctrl::DiscreteSMC smc_;
};

// ---------------------------------------------------------------------------
// 7. GainScheduled - 3 brackets on T_pot:
//    <T_melt: PID_low, <80^\circC: PID_mid, >=80^\circC: PID_hi
// ---------------------------------------------------------------------------
class GainScheduledCookerCtrl : public ControllerBase {
public:
    explicit GainScheduledCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "GainScheduled"; }
private:
    ctrl::GainScheduledController gs_;
};

// ---------------------------------------------------------------------------
// 8. MRAC - set_reference(T_ref), compute(T_pot); uMin=0, uMax=1
// ---------------------------------------------------------------------------
class MRACCookerCtrl : public ControllerBase {
public:
    explicit MRACCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "MRAC"; }
private:
    ctrl::MRACController mrac_;
};

// ---------------------------------------------------------------------------
// 9. L1Adaptive - set_reference(T_ref), compute(T_pot); uMin=0, uMax=1
// ---------------------------------------------------------------------------
class L1AdaptiveCookerCtrl : public ControllerBase {
public:
    explicit L1AdaptiveCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    ctrl::L1AdaptiveController l1_;
};

// ---------------------------------------------------------------------------
// 10. NeuralPID - compute(T_pot - T_ref); plant_gain negative (shade->lower T)
// ---------------------------------------------------------------------------
class NeuralPIDCookerCtrl : public ControllerBase {
public:
    explicit NeuralPIDCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "NeuralPID"; }
private:
    ctrl::NeuralPID npid_;
};

// ---------------------------------------------------------------------------
// 11. DynaCtrl - DynaController wrapping PID; compute(T_pot - T_ref)
// ---------------------------------------------------------------------------
class DynaCookerCtrl : public ControllerBase {
public:
    explicit DynaCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "DynaCtrl"; }
private:
    ctrl::DynaController dyna_;
};

// ---------------------------------------------------------------------------
// 12. ScenarioMPC - 2-state linearised model; Sigma_w from G/T_amb noise
// ---------------------------------------------------------------------------
class ScenarioMPCCookerCtrl : public ControllerBase {
public:
    explicit ScenarioMPCCookerCtrl(const PlantParams& p);
    double      compute(const Eigen::Vector2d& state, double T_ref) override;
    void        reset() override;
    std::string name() const override { return "ScenarioMPC"; }
private:
    ctrl::ScenarioMPC smpc_;
    static ctrl::StateSpace buildSS(const PlantParams& p);
    double T_abs_nom_;
    double T_pot_nom_;
};

} // namespace cooker
