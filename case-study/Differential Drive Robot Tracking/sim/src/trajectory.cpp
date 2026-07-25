#include "trajectory.h"
#include "differential_drive_robot_tracking_plant.h"   // wrapAngle
#include <algorithm>
#include <cmath>

namespace differentialdriverobottracking {

namespace {
double sgn(double x) { return (x > 0.0) - (x < 0.0); }
}

PathType pathTypeFromString(const std::string& s) {
    if (s == "circle")  return PathType::Circle;
    if (s == "diamond") return PathType::Diamond;
    return PathType::Lemniscate;
}

const char* pathTypeName(PathType p) {
    switch (p) {
        case PathType::Circle:  return "circle";
        case PathType::Diamond: return "diamond";
        default:                return "lemniscate";
    }
}

void refPosition(PathType path, double a, double t_path, double& x, double& y) {
    const double c = std::cos(t_path), s = std::sin(t_path);
    switch (path) {
        case PathType::Circle:
            x = a * c;
            y = a * s;
            break;
        case PathType::Diamond:
            x = a * sgn(c) * (1.0 - std::abs(s));
            y = a * sgn(s) * (1.0 - std::abs(c));
            break;
        case PathType::Lemniscate:
        default: {
            const double den = 1.0 + s * s;
            x = a * c / den;
            y = a * s * c / den;
            break;
        }
    }
}

Trajectory::Trajectory(PathType path, double a, double time_scale, double v_max, double w_max)
    : path_(path), a_(a),
      time_scale_((time_scale > 1e-9) ? time_scale : 1.0),
      v_max_(v_max), w_max_(w_max),
      h_(1e-4)
{}

RefPoint Trajectory::at(double t) const {
    RefPoint rp;
    if (!std::isfinite(t)) return rp;

    const double tp = t * time_scale_;

    double x0 = 0.0, y0 = 0.0, xm = 0.0, ym = 0.0, xp = 0.0, yp = 0.0;
    refPosition(path_, a_, tp,      x0, y0);
    refPosition(path_, a_, tp - h_, xm, ym);
    refPosition(path_, a_, tp + h_, xp, yp);

    rp.x = x0;
    rp.y = y0;

    // d/dt = (d/dt_path) * time_scale
    const double dx = (xp - xm) / (2.0 * h_) * time_scale_;
    const double dy = (yp - ym) / (2.0 * h_) * time_scale_;

    const double th_m = std::atan2((y0 - ym) / h_, (x0 - xm) / h_);
    const double th_p = std::atan2((yp - y0) / h_, (xp - x0) / h_);

    rp.theta = std::atan2(dy, dx);
    rp.v     = std::hypot(dx, dy);
    // th_m is the heading over [tp-h, tp] and th_p over [tp, tp+h], so their effective
    // sample points are centred at tp-h/2 and tp+h/2 - separated by h, NOT 2h. Unwrap
    // before differencing: atan2 jumps by 2*pi are branch artefacts, not rotation.
    rp.w     = wrapAngle(th_p - th_m) / h_ * time_scale_;

    if (!std::isfinite(rp.v)) rp.v = 0.0;
    if (!std::isfinite(rp.w)) rp.w = 0.0;
    if (!std::isfinite(rp.theta)) rp.theta = 0.0;

    // The diamond's four sgn() corners send omega_r to infinity in the limit; clamp both
    // feedforward channels to what the actuator layer could ever deliver.
    rp.v = std::clamp(rp.v, -v_max_, v_max_);
    rp.w = std::clamp(rp.w, -w_max_, w_max_);
    return rp;
}

}  // namespace differentialdriverobottracking
