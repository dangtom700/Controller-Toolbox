#pragma once
#include "IController.h"
#include "SINDy.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <memory>
#include <stdexcept>
#include <string>

namespace ctrl {

/**
 * @brief Dyna model-based reinforcement learning controller (Sutton 1991).
 *
 * Interleaves three phases at every sample step:
 *   1. **React** - delegate to a wrapped base policy (PID, MPC, ...) for the
 *      actual control output.
 *   2. **Learn** - accumulate (e[k-1], u[k-1]) -> e[k] transitions and fit
 *      a SINDy model of the error dynamics once enough data is collected.
 *   3. **Plan** - expose modelRollout() so external code (Python or C++) can
 *      simulate the fitted model forward and improve the wrapped policy without
 *      waiting for more real data.
 *
 * **State / model convention (SISO):**
 * @code
 *   edot = f(e[k], u[k])           (error dynamics fitted by SINDy)
 *   e[k+1] approx = e[k] + Ts * edot      (Euler forward integration in rollout)
 * @endcode
 *
 * **Typical C++ usage:**
 * @code
 *   auto pid = std::make_shared<ctrl::DiscretePID>(pid_p, Ts);
 *   ctrl::DynaController::Params dp;
 *   dp.Ts = Ts; dp.n_collect = 50; dp.n_refit_every = 25;
 *   ctrl::DynaController dyna(dp, pid);
 *
 *   for (int k = 0; k < N; ++k) {
 *       double e = ref - y;
 *       double u = dyna.compute(e);      // collects data, auto-fits model
 *       // advance plant...
 *   }
 *
 *   if (dyna.modelFitted()) {
 *       Eigen::VectorXd u_plan(20); u_plan.setConstant(0.1);
 *       Eigen::VectorXd e_pred = dyna.modelRollout(e_current, u_plan);
 *   }
 * @endcode
 *
 * @see Sutton, R.S. (1991). Dyna, an integrated architecture for learning,
 *      planning, and reacting. SIGART Bulletin 2(4), 160-163.
 * @see SINDy for the sparse dynamics identification back-end.
 */
class DynaController : public IController {
public:
    struct Params {
        int    n_collect     = 50;   ///< Transitions before first model fit
        int    n_refit_every = 25;   ///< Refit interval after initial fit [transitions]
        double Ts            = 0.01; ///< Sample time [s]
        SINDy::Params sindy;         ///< SINDy model params (n_state=1, n_input=1 enforced)
    };

    /**
     * @brief Construct DynaController wrapping a base policy.
     * @param p      Design parameters.
     * @param inner  Base controller for real-time output. Must not be null.
     */
    DynaController(const Params& p, std::shared_ptr<IController> inner);

    // ---- IController interface -----------------------------------------------

    /** @brief Call inner controller, collect one transition, auto-refit if due. */
    double compute(double error) override;
    void   reset()               override;
    double sampleTime() const    override { return p_.Ts; }
    std::string name()  const    override { return "Dyna"; }

    // ---- Data collection -----------------------------------------------------

    /**
     * @brief Manually add a SISO error-dynamics transition.
     * @param e      Error at time k.
     * @param u      Control action at time k.
     * @param e_next Error at time k+1.
     */
    void addTransition(double e, double u, double e_next);

    /**
     * @brief Fit the SINDy model on all accumulated transitions.
     * @return true  if fitting succeeded (enough data present).
     * @return false if fewer than n_collect transitions have been collected.
     */
    bool fitModel();

    // ---- Model-based planning ------------------------------------------------

    /**
     * @brief Simulate the fitted model forward from an initial error.
     *
     * @param e0         Initial tracking error.
     * @param u_sequence Column vector of planned control actions (length = n_steps).
     * @return           Column vector of predicted errors at each future step.
     * @throws std::logic_error if the model has not been fitted yet.
     */
    Eigen::VectorXd modelRollout(double                  e0,
                                  const Eigen::VectorXd&  u_sequence) const;

    // ---- Accessors -----------------------------------------------------------

    /** @brief True after fitModel() has succeeded at least once. */
    bool modelFitted()       const noexcept { return model_fitted_; }

    /** @brief Total transitions accumulated (real + any manual addTransition calls). */
    int  bufferSize()        const noexcept { return n_total_; }

    /** @brief Transitions added since the last model fit. */
    int  newSinceLastFit()   const noexcept { return steps_since_fit_; }

    /** @brief Fitted SINDy model. @throws std::logic_error if not fitted. */
    const SINDyModel& model() const;

    /** @brief The wrapped base policy. */
    std::shared_ptr<IController> innerController() const { return inner_; }

private:
    Params                       p_;
    std::shared_ptr<IController> inner_;
    SINDy                        sindy_;
    SINDyModel                   model_;
    bool                         model_fitted_    = false;
    int                          steps_since_fit_ = 0;
    int                          n_total_         = 0;

    double e_prev_    = 0.0;
    double u_prev_    = 0.0;
    bool   has_prev_  = false;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(dyna)
