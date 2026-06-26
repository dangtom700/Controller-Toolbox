#pragma once
#include "NonlinearMPC.h"
#include "GPResidualModel.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <memory>
#include <string>

/**
 * @file GPMPC.h
 * @brief GP-uncertainty-aware tightening of NonlinearMPC's input bounds (Phase 3 ML3).
 *
 * Scoped-down from the textbook "tighten output/state constraints by GP variance" GP-MPC:
 * NonlinearMPC has no output/state constraints at all (only input box bounds), and its QP
 * solver only handles box constraints. GPMPC instead tightens the INPUT box bounds per
 * predicted step, mirroring TubeMPC's existing input-tightening precedent but made
 * time-varying across the horizon (driven by GP variance) instead of TubeMPC's static
 * disturbance-derived radius.
 *
 * @see NonlinearMPC for the base RTI-MPC this inherits.
 * @see GPResidualModel for the GP residual/uncertainty model consumed here.
 * @see docs/superpowers/specs/2026-06-25-gp-mpc-design.md
 */

namespace ctrl {

/**
 * @brief Parameters for @ref GPMPC.
 */
struct GPMPCParams
{
    NMPCParams nmpc;                     ///< Base NonlinearMPC params (horizons, weights, bounds).
    double uncertainty_inflation = 2.0;  ///< Tightening = inflation * sqrt(GP variance).
};

/**
 * @brief NonlinearMPC variant that tightens input bounds per step by GP-predicted variance.
 */
class GPMPC : public NonlinearMPC
{
public:
    /**
     * @brief Construct GPMPC with full-state output (C = I).
     * @param params Horizon/cost/bound params plus uncertainty_inflation.
     * @param f_d    Discrete-time dynamics x[k+1] = f(x[k], u[k]).
     * @param gp     Shared GP residual model; its xDim() must equal
     *               params.nmpc.n_states + params.nmpc.n_inputs.
     * @throws std::invalid_argument If gp is null or gp->xDim() doesn't match n_states + n_inputs.
     */
    GPMPC(const GPMPCParams &params, NonlinearMPC::DiscreteDynamics f_d,
          std::shared_ptr<GPResidualModel> gp);

    std::string name() const override { return "GPMPC"; }

    /**
     * @brief Per-step shrink amount applied to lb_qp_/ub_qp_ on the most recent compute()
     *        (length Nu*m, one block per held control step). Zero everywhere if the GP
     *        is unfitted. Mirrors TubeMPC's tightenedUMin()/tightenedUMax() convention.
     */
    const Eigen::VectorXd &lastTightening() const { return shrink_; }

protected:
    void tightenStepBounds() override;

private:
    GPMPCParams gp_params_;
    std::shared_ptr<GPResidualModel> gp_;
    Eigen::VectorXd shrink_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(gp_mpc)
