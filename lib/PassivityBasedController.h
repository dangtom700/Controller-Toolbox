#pragma once
#include "IController.h"
#include "Features.h"
#include <Eigen/Dense>
#include <functional>

/**
 * @file PassivityBasedController.h
 * @brief Energy-shaping + damping-injection regulation for Euler-Lagrange systems (Phase 3 NC2).
 *
 * Euler-Lagrange system: `M(q)*qddot + C(q,qdot)*qdot + dV(q) = u`. Regulates a constant
 * desired configuration `q_d` via the classical PD+ law (Takegaki & Arimoto 1981):
 * @code
 *   u = dV(q) - Kp*(q - q_d) - Kd*qdot
 * @endcode
 * `Kp` reshapes the total potential so its minimum sits at `q_d` ("energy shaping"); `Kd`
 * dissipates kinetic energy ("damping injection"). The Lyapunov/storage-function proof
 * (`V = 0.5*qdot'*M(q)*qdot + 0.5*(q-q_d)'*Kp*(q-q_d)`, `V' = -qdot'*Kd*qdot <= 0`) needs
 * **no explicit Coriolis cancellation in `u`** - the cross term vanishes via the standard
 * Lagrangian skew-symmetry property `qdot'*(Mdot(q) - 2*C(q,qdot))*qdot = 0`, which holds
 * when `C` is the conventional Christoffel-symbol factorization of `M`. `M`/`C` are still
 * evaluated every step to feed `storageEnergy()` (passivity monitoring/testing) even though
 * `u` itself never inverts or otherwise uses them beyond that diagnostic.
 *
 * @see docs/superpowers/specs/2026-06-24-nonlinear-control-trio-design.md
 */

namespace ctrl {

/** @brief Tuning parameters for PassivityBasedController. */
struct PBCParams
{
    Eigen::MatrixXd Kp; ///< Energy-shaping (stiffness) injection gain, PSD (n x n).
    Eigen::MatrixXd Kd; ///< Damping injection gain, PSD (n x n).
    double uMin = -1e9, uMax = 1e9;
};

/**
 * @brief Passivity-based (PD+) regulation controller for Euler-Lagrange systems.
 */
class PassivityBasedController : public IController
{
public:
    using MassMatrixFn    = std::function<Eigen::MatrixXd(const Eigen::VectorXd &q)>;
    using PotentialGradFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &q)>;
    using CoriolisFn      = std::function<Eigen::MatrixXd(const Eigen::VectorXd &q,
                                                            const Eigen::VectorXd &qdot)>;

    PassivityBasedController(MassMatrixFn M, PotentialGradFn dV, CoriolisFn C,
                              const PBCParams &params, double Ts);

    /**
     * @brief Compute one control step.
     * @param state Raw stacked plant state `[q; qdot]` (size 2n).
     * @return Control vector u[k] (size n), clamped to [uMin, uMax] elementwise.
     */
    Eigen::VectorXd computeVec(const Eigen::VectorXd &state) override;

    /**
     * @brief Not supported - PassivityBasedController is MIMO-only.
     * @throws std::logic_error Always; call computeVec() instead.
     */
    double compute(double signal) override;

    SignConvention signConvention() const override { return SignConvention::PlantOutput; }

    void reset() override;
    double sampleTime() const override { return Ts_; }

    /** @brief Set the desired (constant) configuration q_d. */
    void setDesired(const Eigen::VectorXd &q_d) { q_d_ = q_d; }

    /**
     * @brief Shaped total-energy storage function from the last successful computeVec() call:
     *        `0.5*qdot'*M(q)*qdot + 0.5*(q-q_d)'*Kp*(q-q_d)`.
     */
    double storageEnergy() const { return storageEnergy_; }

private:
    MassMatrixFn    M_;
    PotentialGradFn dV_;
    CoriolisFn      C_;
    PBCParams       params_;
    double          Ts_;

    int             n_ = -1; ///< Configuration-space dimension, inferred from the first state.
    Eigen::VectorXd q_d_;
    Eigen::VectorXd u_prev_;
    double          storageEnergy_ = 0.0;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(passivity_based_controller)
