#pragma once
// trajectory.h - the three benchmark reference paths from the FUHAC paper (Sec. "Results and
// discussion"):
//
//   Lemniscate: x_ref = a*cos(t)/(1 + sin^2 t),   y_ref = a*sin(t)*cos(t)/(1 + sin^2 t),  a = 2.0
//   Circle:     x_ref = a*cos(t),                 y_ref = a*sin(t),                       a = 3.0
//   Diamond:    x_ref = a*sgn(cos t)*(1-|sin t|), y_ref = a*sgn(sin t)*(1-|cos t|),       a = 1.0
//
// theta_r = atan2(yr', xr'), v_r = hypot(xr', yr'), omega_r = d(theta_r)/dt - all obtained by
// central difference on the analytic path with atan2-based angle unwrapping, because the
// diamond's sgn() terms make an analytic derivative undefined at the four corners.
//
// The diamond corners produce unbounded omega_r in the limit; v_r and omega_r are clamped to
// the plant's v_max / w_max. That clamp is a deliberate, documented deviation - see README
// "Deviations from the paper".
#include <string>

namespace differentialdriverobottracking {

enum class PathType { Lemniscate, Circle, Diamond };

/// Parse "lemniscate" | "circle" | "diamond" (case-sensitive); defaults to Lemniscate.
PathType pathTypeFromString(const std::string& s);
const char* pathTypeName(PathType p);

/// One sample of the reference trajectory.
struct RefPoint {
    double x = 0.0;      ///< x_ref [m]
    double y = 0.0;      ///< y_ref [m]
    double theta = 0.0;  ///< theta_r = atan2(yr', xr') [rad]
    double v = 0.0;      ///< v_r [m/s]
    double w = 0.0;      ///< omega_r [rad/s]
};

/// Analytic reference position only (no derivatives).
void refPosition(PathType path, double a, double t_path, double& x, double& y);

// Reference generator. `time_scale` maps simulation time to path parameter: t_path = t*time_scale.
// time_scale = 1.0 reproduces the paper's literal parameterisation (period 2*pi s), which for
// the a = 2 m lemniscate demands ~2-3 m/s - above real Pioneer limits, and the reason the
// paper's torques are as large as they are.
class Trajectory {
public:
    Trajectory(PathType path, double a, double time_scale, double v_max, double w_max);

    /// Reference pose + feedforward velocities at simulation time t [s].
    RefPoint at(double t) const;

    PathType path()      const { return path_; }
    double   amplitude() const { return a_; }

private:
    PathType path_;
    double   a_;
    double   time_scale_;
    double   v_max_;
    double   w_max_;
    double   h_;   ///< central-difference half-step in path-parameter units
};

}  // namespace differentialdriverobottracking
