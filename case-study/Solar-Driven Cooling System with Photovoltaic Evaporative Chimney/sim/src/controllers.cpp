#include "controllers.h"
#include <algorithm>
#include <cmath>

namespace solar {

// Clamp helpers
static constexpr double kMwMin = 0.02;
static constexpr double kMwMax = 0.22;
static constexpr double kKrMin = 0.30;
static constexpr double kKrMax = 1.00;

static double clampMw(double v) { return std::clamp(v, kMwMin, kMwMax); }
static double clampKr(double v) { return std::clamp(v, kKrMin, kKrMax); }

// ---------------------------------------------------------------------------
// 1. PIDController
// Gains tuned for a nominal Tw1 range of ~15-45 ^\circC with Ts = 10 s.
// Tracking error e = ref_Tw1 - measured_Tw1.
// Positive e (too cold) -> reduce m_dot_w; negative e (too warm) -> increase.
// ---------------------------------------------------------------------------
PIDController::PIDController(double Ts)
    : pid_(ctrl::PIDParams{
        .Kp   =  0.002,
        .Ki   =  0.0003,
        .Kd   =  0.005,
        .N    =  5.0,
        .uMin = kMwMin,
        .uMax = kMwMax,
        .Kb   =  0.5
      }, Ts)
{}

void PIDController::reset()
{
    pid_.reset();
}

ControlInput PIDController::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double e        = ref_Tw1 - measured_Tw1;
    double m_dot_w  = clampMw(pid_.compute(e));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 2. FFPIDController
// Feedforward: delta_mw = G_to_mw_ * G  anticipates increased thermal load.
// PID trims the remaining error on a +/-0.10 kg/s authority around nominal.
// Pump speed scheduled linearly from the tracking error.
// ---------------------------------------------------------------------------
FFPIDController::FFPIDController(double Ts)
    : pid_(ctrl::PIDParams{
        .Kp   =  0.002,
        .Ki   =  0.0003,
        .Kd   =  0.005,
        .N    =  5.0,
        .uMin = -0.10,
        .uMax =  0.10,
        .Kb   =  0.5
      }, Ts)
{}

void FFPIDController::reset()
{
    pid_.reset();
}

ControlInput FFPIDController::compute(double ref_Tw1, double measured_Tw1, double G)
{
    double e       = ref_Tw1 - measured_Tw1;
    double u_ff    = G_to_mw_ * std::max(G, 0.0);
    double u_fb    = pid_.compute(e);
    double m_dot_w = clampMw(mw_nom_ + u_ff + u_fb);
    double kr      = clampKr(kr_base_ + kr_e_slope_ * (e / 10.0));
    return {m_dot_w, kr};
}

// ---------------------------------------------------------------------------
// 3. MPCController
// FOPDT model of Tw1 vs m_dot_w linearised around Tw1_nom.
// Continuous plant: dTw1/dt = -a*Tw1 + b*m_dot_w
// Discrete (Euler): Tw1[k+1] = (1-a*Ts)*Tw1[k] + b*Ts*m_dot_w[k]
// DiscreteMPC from the toolbox handles the condensed QP internally.
// ---------------------------------------------------------------------------
MPCController::MPCController(double Ts, double Tw1_nom, double a, double b)
    : mpc_(
        // Build a SISO discrete-time state-space from the FOPDT model
        ctrl::StateSpace(
            (Eigen::Matrix<double,1,1>() << 1.0 - a*Ts).finished(),  // A (1x1)
            (Eigen::Matrix<double,1,1>() << b*Ts).finished(),         // B (1x1)
            (Eigen::Matrix<double,1,1>() << 1.0).finished(),          // C (1x1)
            (Eigen::Matrix<double,1,1>() << 0.0).finished(),          // D (1x1)
            Ts
        ),
        ctrl::MPCParams{
            .Np    = 8,
            .Nc    = 3,
            .rho_y = 10.0,
            .rho_u = 50.0,
            .uMin  = kMwMid - kMwHalf,
            .uMax  = kMwMid + kMwHalf,
            .duMin = -0.02,
            .duMax =  0.02,
        }
      ),
      Tw1_nom_(Tw1_nom)
{}

void MPCController::reset()
{
    mpc_.reset();
    // Seed state estimate at design-point deviation = 0
    Eigen::VectorXd x0(1);
    x0 << 0.0;
    mpc_.setState(x0);
}

ControlInput MPCController::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    // Work in deviation variables so that u = 0 maps to m_dot_w = kMwMid
    double x_dev = measured_Tw1 - Tw1_nom_;
    double r_dev = ref_Tw1      - Tw1_nom_;

    Eigen::VectorXd x(1); x << x_dev;
    Eigen::VectorXd r(1); r << r_dev;

    mpc_.setState(x);
    Eigen::VectorXd u = mpc_.computeRef(x, r);

    double m_dot_w = clampMw(u(0));
    return {m_dot_w, kr_nom_};
}

} // namespace solar
