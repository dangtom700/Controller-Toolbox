#pragma once
#include "NonlinearMPC.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <functional>

/**
 * @file HybridModel.h
 * @brief Hybrid plant model combining a physical ODE with a data-driven correction (H1).
 *
 * Represents the plant as:
 * @code
 *   x[k+1] = RK4( f_phys(x, u, p), Ts, rk4_steps ) + f_data(x, u)
 * @endcode
 *
 * where f_phys is the known continuous-time physical model (integrated by RK4)
 * and f_data is an optional discrete-time state correction learned from data
 * (e.g. GP residual, ESN, or ridge regression).
 *
 * **Typical workflow:**
 * @code
 *   // 1. Define physical ODE
 *   auto f_phys = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u,
 *                    const Eigen::VectorXd& p) -> Eigen::VectorXd {
 *       Eigen::VectorXd xd(2);
 *       xd(0) = x(1);
 *       xd(1) = -p(0)*x(0) - p(1)*x(1) + u(0);
 *       return xd;
 *   };
 *
 *   ctrl::HybridModelParams hmp;
 *   hmp.n_states = 2; hmp.n_inputs = 1; hmp.Ts = 0.01;
 *   Eigen::VectorXd p_phys(2); p_phys << 4.0, 0.8;
 *   auto model = std::make_shared<ctrl::HybridModel>(f_phys, hmp, p_phys);
 *
 *   // 2. (Optional) attach a trained data correction
 *   model->setDataModel([](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
 *       return Eigen::VectorXd::Zero(x.size());  // placeholder
 *   });
 *
 *   // 3. Use in NonlinearMPC or HybridMPC
 *   ctrl::NMPCParams np; np.n_states = 2; np.n_inputs = 1; np.Np = 10; np.Ts = 0.01;
 *   ctrl::NonlinearMPC nmpc(np, model->dynamicsFunc());
 * @endcode
 *
 * @see HybridMPC   - NMPC variant that owns a HybridModel and refits f_data online.
 * @see HybridModelTrainer - Trains f_data from state-transition observations (H4).
 */

namespace ctrl {

/** @brief Configuration for HybridModel. */
struct HybridModelParams
{
    int    n_states  = 1;    ///< State dimension (must match f_phys output size).
    int    n_inputs  = 1;    ///< Input dimension.
    int    n_outputs = 1;    ///< Output dimension (y = x[:n_outputs] by default).
    double Ts        = 0.01; ///< Sample time [s].
    int    rk4_steps = 4;    ///< RK4 substeps per Ts for physical integration.
};

/**
 * @brief Hybrid plant model: physics + data-driven additive correction.
 *
 * f_phys is integrated with RK4 over rk4_steps substeps.
 * f_data is a discrete-time state correction (same units as x) added after integration.
 *
 * Both functions must return Eigen::VectorXd of size n_states.
 */
class HybridModel
{
public:
    /** @brief Continuous-time physical ODE: x_dot = f_phys(x, u, p). */
    using PhysFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd& x,
                                                    const Eigen::VectorXd& u,
                                                    const Eigen::VectorXd& p)>;

    /** @brief Discrete-time state correction: delta_x_next = f_data(x, u). */
    using DataFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd& x,
                                                    const Eigen::VectorXd& u)>;

    /**
     * @brief Construct with physical model only.
     * @param f_phys Continuous-time ODE (integrated by RK4).
     * @param params Dimensions and integration settings.
     * @param p_phys Initial physical parameter vector (may be empty).
     */
    HybridModel(PhysFunc f_phys, const HybridModelParams& params,
                Eigen::VectorXd p_phys = {})
        : f_phys_(std::move(f_phys)), p_(params), p_phys_(std::move(p_phys))
    {}

    // -------------------------------------------------------------------------
    // Data model management
    // -------------------------------------------------------------------------

    /** @brief Attach (or replace) the data-driven correction function. */
    void setDataModel(DataFunc f_data) {
        f_data_   = std::move(f_data);
        has_data_ = true;
    }

    /** @brief Remove the data-driven correction (physical model only). */
    void clearDataModel() {
        f_data_   = DataFunc{};
        has_data_ = false;
    }

    /** @brief True if a data model has been attached. */
    bool hasDataModel() const noexcept { return has_data_; }

    // -------------------------------------------------------------------------
    // Physical parameter management
    // -------------------------------------------------------------------------

    /** @brief Update the physical model parameters p passed to f_phys. */
    void setPhysParams(const Eigen::VectorXd& p) { p_phys_ = p; }

    /** @brief Current physical model parameters. */
    const Eigen::VectorXd& physParams() const { return p_phys_; }

    // -------------------------------------------------------------------------
    // Prediction
    // -------------------------------------------------------------------------

    /**
     * @brief Combined discrete-time prediction.
     *
     * x[k+1] = RK4(f_phys, x, u, p, Ts/rk4_steps) + f_data(x, u)
     *
     * If no data model is attached, returns the physical-only prediction.
     */
    Eigen::VectorXd predict(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        Eigen::VectorXd x_next = rk4Step(x, u);
        if (has_data_) x_next += f_data_(x, u);
        return x_next;
    }

    /**
     * @brief Physical-only discrete-time prediction (no data correction).
     * Useful for computing residuals in online learning.
     */
    Eigen::VectorXd predictPhys(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        return rk4Step(x, u);
    }

    /**
     * @brief Return a NonlinearMPC::DiscreteDynamics capturing this model by shared_ptr.
     *
     * The returned lambda holds a shared_ptr copy of @p self so the HybridModel
     * remains alive as long as the lambda (and thus the NonlinearMPC) lives.
     * Any subsequent calls to setDataModel() on @p self are reflected in the lambda.
     *
     * @param self  shared_ptr to this object (pass std::shared_ptr<HybridModel>(this)
     *              when the model is heap-allocated, or use make_shared<HybridModel>).
     */
    static NonlinearMPC::DiscreteDynamics makeDynamicsFunc(std::shared_ptr<HybridModel> self) {
        return [m = std::move(self)](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
            return m->predict(x, u);
        };
    }

    /**
     * @brief Return a NonlinearMPC::DiscreteDynamics capturing this model by raw pointer.
     *
     * @deprecated Use HybridModel::makeDynamicsFunc(shared_ptr<HybridModel>) instead.
     *             The raw-pointer capture causes UB if the HybridModel is destroyed before
     *             the lambda (e.g. when passed to NonlinearMPC which outlives the model).
     */
    [[deprecated("use HybridModel::makeDynamicsFunc(shared_ptr<HybridModel>)")]]
    NonlinearMPC::DiscreteDynamics dynamicsFunc() {
        return [this](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
            return this->predict(x, u);
        };
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    int    stateSize()  const noexcept { return p_.n_states;  }
    int    inputSize()  const noexcept { return p_.n_inputs;  }
    int    outputSize() const noexcept { return p_.n_outputs; }
    double sampleTime() const noexcept { return p_.Ts;        }

private:
    Eigen::VectorXd rk4Step(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        const double h = p_.Ts / static_cast<double>(p_.rk4_steps);
        Eigen::VectorXd xs = x;
        for (int s = 0; s < p_.rk4_steps; ++s) {
            const auto k1 = f_phys_(xs,            u, p_phys_);
            const auto k2 = f_phys_(xs + 0.5*h*k1, u, p_phys_);
            const auto k3 = f_phys_(xs + 0.5*h*k2, u, p_phys_);
            const auto k4 = f_phys_(xs + h*k3,      u, p_phys_);
            xs += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
        }
        return xs;
    }

    PhysFunc         f_phys_;
    DataFunc         f_data_;
    HybridModelParams p_;
    Eigen::VectorXd  p_phys_;
    bool             has_data_ = false;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(hybrid_model)
