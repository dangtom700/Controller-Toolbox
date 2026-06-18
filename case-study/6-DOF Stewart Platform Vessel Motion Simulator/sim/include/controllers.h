#pragma once
#include "stewart_plant.h"
#include <ControllerToolbox.h>
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>

// Twelve per-rod controllers for the 6-DOF Stewart platform case study.
//
// Interface: compute(L_cmd, L, dL, t, z_ref_global) -> u (6 rod forces [N])
//   L_cmd        : commanded rod lengths [m] (from CFDInputModel + inverse kinematics)
//   L, dL        : measured rod lengths / velocities [m], [m/s]
//   z_ref_global : |heave deviation from neutral| [m] (precomputed by the runner);
//                  used only by GainScheduledStewartCtrl's scheduling variable
//
// All controllers output a direct force in Newtons (decision #2 - no normalised
// [-1,1] command). LQR/MPC/TubeMPC/ScenarioMPC operate in the rod DEVIATION
// coordinate z_i = L_i - l0_i (true LTI form of the spring-damper rod dynamics);
// PID/ADRC/SMC/MRAC/L1Adaptive/NeuralPID/FuzzyPID/GainScheduled operate directly
// on the absolute error e_i = L_cmd_i - L_i (or y-ref convention where noted).

namespace stewart {

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual Vec6        compute(const Vec6& L_cmd, const Vec6& L, const Vec6& dL,
                                double t, double z_ref_global) = 0;
    virtual void        reset() = 0;
    virtual std::string name() const = 0;
};

// 1. PID - per-rod, e_i = L_cmd_i - L_i
class PIDStewartCtrl : public ControllerBase {
public:
    explicit PIDStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "PID"; }
private:
    std::array<ctrl::DiscretePID, N_RODS> pids_;
};

// 2. FuzzyPID - per-rod, e/ec scaled to +/-3 mm
class FuzzyPIDStewartCtrl : public ControllerBase {
public:
    explicit FuzzyPIDStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "FuzzyPID"; }
private:
    std::array<ctrl::FuzzyPID, N_RODS> fpids_;
};

// 3. ADRC - per-rod, omega_o=60, omega_c=20, b0=1/m_rod
class ADRCStewartCtrl : public ControllerBase {
public:
    explicit ADRCStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "ADRC"; }
private:
    std::array<ctrl::DiscreteADRC, N_RODS> adrcs_;
};

// 4. SMC - per-rod, compute(L_i - L_cmd_i) [y-ref convention]
class SMCStewartCtrl : public ControllerBase {
public:
    explicit SMCStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "SMC"; }
private:
    std::array<ctrl::DiscreteSMC, N_RODS> smcs_;
};

// 5. LQR - single 12-state block-diagonal model (decision #6), deviation coords
class LQRStewartCtrl : public ControllerBase {
public:
    explicit LQRStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override {}
    std::string name() const override { return "LQR"; }
private:
    Vec6 l0_;
    std::unique_ptr<ctrl::DiscreteLQR> lqr_;
};

// 6. MPC - 6x 2-state per-rod DiscreteMPC, deviation coords
class MPCStewartCtrl : public ControllerBase {
public:
    explicit MPCStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "MPC"; }
private:
    Vec6 l0_;
    std::array<std::unique_ptr<ctrl::DiscreteMPC>, N_RODS> mpcs_;
};

// 7. MRAC - per-rod, setReference(L_cmd_i) + compute(L_i)
class MRACStewartCtrl : public ControllerBase {
public:
    explicit MRACStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "MRAC"; }
private:
    std::array<ctrl::MRACController, N_RODS> mracs_;
};

// 8. L1Adaptive - per-rod, setReference + compute(L_i)
class L1AdaptiveStewartCtrl : public ControllerBase {
public:
    explicit L1AdaptiveStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "L1Adaptive"; }
private:
    std::array<ctrl::L1AdaptiveController, N_RODS> l1s_;
};

// 9. GainScheduled - per-rod, scheduled on |heave deviation| (z_ref_global)
class GainScheduledStewartCtrl : public ControllerBase {
public:
    explicit GainScheduledStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "GainScheduled"; }
private:
    std::array<ctrl::GainScheduledController, N_RODS> gscs_;
};

// 10. TubeMPC - 6x 2-state per-rod TubeMPC, deviation coords
class TubeMPCStewartCtrl : public ControllerBase {
public:
    explicit TubeMPCStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "TubeMPC"; }
private:
    Vec6 l0_;
    std::array<ctrl::TubeMPC, N_RODS> tmpcs_;
};

// 11. NeuralPID - per-rod, online gain adaptation
class NeuralPIDStewartCtrl : public ControllerBase {
public:
    explicit NeuralPIDStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "NeuralPID"; }
private:
    std::array<ctrl::NeuralPID, N_RODS> npids_;
};

// 12. ScenarioMPC - 6x 2-state per-rod ScenarioMPC, deviation coords
class ScenarioMPCStewartCtrl : public ControllerBase {
public:
    explicit ScenarioMPCStewartCtrl(const PlantParams& p);
    Vec6        compute(const Vec6&, const Vec6&, const Vec6&, double, double) override;
    void        reset() override;
    std::string name() const override { return "ScenarioMPC"; }
private:
    Vec6 l0_;
    std::array<ctrl::ScenarioMPC, N_RODS> smpcs_;
};

} // namespace stewart
