#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>

/**
 * @file SetMembershipEstimator.h
 * @brief Bounded-error (set-membership) ellipsoidal state estimation (Phase 3 Roadmap Phase 2 EF2).
 *
 * Given known noise *bounds* (not a probability distribution), maintains a guaranteed feasible
 * ellipsoid containing the true state - structurally distinct from every probabilistic filter
 * in `lib/` (KalmanFilter, EKF/UKF, ParticleFilter).
 *
 * **predict()**: outer-bounding ellipsoid Minkowski sum (Kurzhanski & Valyi, "Ellipsoidal
 * Calculus for Estimation and Control", 1997), trace-optimal scalar weighting.
 * **update()**: outer-bounding ellipsoid intersection via the S-procedure (the technique
 * underlying the classical Schweppe 1968 / Fogel & Huang 1982 bounding-ellipsoid update),
 * optimized over a scalar combination weight on a fixed grid.
 *
 * @see docs/superpowers/specs/2026-06-25-estimation-extensions-design.md
 */

namespace ctrl
{

/** @brief Noise bounds for SetMembershipEstimator (isotropic ellipsoids). */
struct SetMembershipParams
{
    double w_bound; ///< ||process noise||_inf <= w_bound (isotropic: Qw = w_bound^2 * I).
    double v_bound;  ///< ||measurement noise||_inf <= v_bound (isotropic: Rv = v_bound^2 * I).
};

/**
 * @brief Ellipsoidal bounded-error state estimator for linear discrete-time plants.
 */
class SetMembershipEstimator
{
public:
    /**
     * @brief Construct the estimator.
     * @param plant Linear discrete-time model (A, B, C, D, Ts). D is ignored (state estimation
     *        only, no direct-feedthrough term needed in the bounding recursion).
     * @param params Process/measurement noise bounds.
     * @param x0_center Initial ellipsoid center.
     * @param E0_shape Initial ellipsoid shape matrix P0 (n x n, positive definite):
     *        {x : (x-x0_center)'.P0^-1.(x-x0_center) <= 1}.
     * @throws std::invalid_argument on size mismatches or non-positive bounds.
     */
    SetMembershipEstimator(const StateSpace &plant, const SetMembershipParams &params,
                            const Eigen::VectorXd &x0_center, const Eigen::MatrixXd &E0_shape);

    /** @brief Propagate the ellipsoid through the plant dynamics + bounded process noise. */
    void predict(const Eigen::VectorXd &u);

    /**
     * @brief Intersect the current ellipsoid with the measurement's consistency set.
     * If the intersection is empty (see isConsistent()), the ellipsoid is left unchanged
     * (the pre-update, i.e. predicted, ellipsoid is kept rather than corrupted).
     */
    void update(const Eigen::VectorXd &y);

    /** @brief Current ellipsoid center estimate. */
    const Eigen::VectorXd &centerEstimate() const { return c_; }

    /** @brief Current ellipsoid shape matrix P: {x : (x-c)'.P^-1.(x-c) <= 1}. */
    const Eigen::MatrixXd &ellipsoidShape() const { return P_; }

    /** @brief False if the most recent update() found an empty ellipsoid/measurement intersection. */
    bool isConsistent() const { return consistent_; }

    /** @brief Reset to the initial ellipsoid (x0_center, E0_shape). */
    void reset();

private:
    Eigen::MatrixXd A_, B_, C_;
    double Ts_;
    SetMembershipParams params_;
    Eigen::VectorXd x0_;
    Eigen::MatrixXd P0_;

    Eigen::VectorXd c_;
    Eigen::MatrixXd P_;
    bool consistent_ = true;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(set_membership_estimator)
