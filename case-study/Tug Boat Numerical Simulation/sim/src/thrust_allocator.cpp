#include "thrust_allocator.h"
#include <cmath>
#include <algorithm>

using namespace Eigen;

namespace tug {

ThrustAllocator::ThrustAllocator(const PlantParameters& p) : pp_(p)
{
    buildDirectionMatrix();
}

void ThrustAllocator::buildDirectionMatrix()
{
    // B[:,i] = [cos(alpha_i), sin(alpha_i), -yi*cos(alpha_i) + xi*sin(alpha_i)]^T
    for (int i = 0; i < NUM_TUGS; ++i) {
        double xi = pp_.stations[i].x;
        double yi = pp_.stations[i].y;
        double a  = pp_.stations[i].alpha;
        double ca = std::cos(a);
        double sa = std::sin(a);
        B_(0, i) = ca;
        B_(1, i) = sa;
        B_(2, i) = -yi * ca + xi * sa;
    }

    // Pseudo-inverse: B_pinv = B^T (B B^T)^-1
    Matrix3d BBT = B_ * B_.transpose();
    B_pinv_ = B_.transpose() * BBT.inverse();
}

AllocResult ThrustAllocator::allocate(const Vector3d& tau_c,
                                       const std::array<double, NUM_TUGS>& T_prev) const
{
    // Unconstrained minimum-norm solution
    Matrix<double, NUM_TUGS, 1> T_unc = B_pinv_ * tau_c;

    AllocResult res;
    res.sat_count = 0;

    for (int i = 0; i < NUM_TUGS; ++i) {
        double T = T_unc(i);

        // Rate limit: delta T <= dT_max per timestep
        double delta = T - T_prev[i];
        double dT_max = pp_.dT_max;
        if (delta >  dT_max) { T = T_prev[i] + dT_max; ++res.sat_count; }
        if (delta < -dT_max) { T = T_prev[i] - dT_max; ++res.sat_count; }

        // Box constraint
        double T_clamped = std::clamp(T, pp_.T_min, pp_.T_max);
        if (T_clamped != T) ++res.sat_count;

        res.T[i] = T_clamped;
    }

    return res;
}

Vector3d ThrustAllocator::achieved(const std::array<double, NUM_TUGS>& T) const
{
    Matrix<double, NUM_TUGS, 1> Tvec;
    for (int i = 0; i < NUM_TUGS; ++i) Tvec(i) = T[i];
    return B_ * Tvec;
}

} // namespace tug
