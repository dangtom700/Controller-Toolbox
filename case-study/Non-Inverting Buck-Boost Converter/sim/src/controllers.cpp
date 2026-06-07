#include "controllers.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace Eigen;

namespace conv {

// ---------------------------------------------------------------------------
// ControllerBase: hysteresis-based mode detection
// ---------------------------------------------------------------------------
void ControllerBase::updateMode(double v_ref, double V_in, double hysteresis)
{
    if (v_ref > V_in + hysteresis)
        mode_ = ConvMode::BOOST;
    else if (v_ref < V_in - hysteresis)
        mode_ = ConvMode::BUCK;
    // else: stay in current mode (hysteresis)
}

// ---------------------------------------------------------------------------
// Shared helper: build discrete-time 2-state buck SS
//
// Buck linearised around (d0, i_L0, v_C0, V_in):
//   L * di_L/dt = d*V_in - v_C    =>  A_c[0,1]=-1/L, B_c[0,0]=V_in/L
//   C * dv_C/dt = i_L - v_C/R    =>  A_c[1,0]=1/C, A_c[1,1]=-1/(R*C)
//   y = v_C                        =>  C=[0,1]
// ---------------------------------------------------------------------------
static ctrl::StateSpace makeBuckSS(const PlantParams& p)
{
    MatrixXd Ac(2,2);
    Ac << 0.0,           -1.0/p.L,
          1.0/p.C,       -1.0/(p.R*p.C);

    MatrixXd Bc(2,1);
    Bc << p.V_in_nom/p.L,
          0.0;

    MatrixXd Cc(1,2); Cc << 0.0, 1.0;
    MatrixXd Dc(1,1); Dc << 0.0;
    ctrl::StateSpace ss_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ss_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// ===========================================================================
// 1. OpenLoop
// ===========================================================================
double OpenLoopCtrl::compute(const Eigen::Vector2d&, double v_ref, double V_in)
{
    updateMode(v_ref, V_in, hyst_);
    return dSS(v_ref, V_in, mode_);
}

// ===========================================================================
// 2. PI_Buck  (Kp=0.0163, Ki=29.86 from paper; applied in all modes)
// ===========================================================================
PIBuckCtrl::PIBuckCtrl(const PlantParams& p)
    : pid_(ctrl::PIDParams{
        .Kp   = 0.0163,
        .Ki   = 29.86,
        .Kd   = 0.0,
        .N    = 0.0,
        .uMin = D_MIN,
        .uMax = D_MAX,
        .Kb   = 1.0
      }, p.Ts)
{}

double PIBuckCtrl::compute(const Vector2d& state, double v_ref, double /*V_in*/)
{
    return pid_.compute(v_ref - state(1));
}

void PIBuckCtrl::reset() { pid_.reset(); }

// ===========================================================================
// 3. PI_Boost  (Kp=0.0058, Ki=15.05 from paper; applied in all modes)
// ===========================================================================
PIBoostCtrl::PIBoostCtrl(const PlantParams& p)
    : pid_(ctrl::PIDParams{
        .Kp   = 0.0058,
        .Ki   = 15.05,
        .Kd   = 0.0,
        .N    = 0.0,
        .uMin = D_MIN,
        .uMax = D_MAX,
        .Kb   = 1.0
      }, p.Ts)
{}

double PIBoostCtrl::compute(const Vector2d& state, double v_ref, double /*V_in*/)
{
    return pid_.compute(v_ref - state(1));
}

void PIBoostCtrl::reset() { pid_.reset(); }

// ===========================================================================
// 4. TLCS_ClassicPI  (two classic PIs with bumpless mode-switch transfer)
//
// Bump-less: inactive controller's integral is synchronised each step so that
// its next output would equal the active controller's output.  Switching bump
// is eliminated.
// ===========================================================================
TLCSClassicPICtrl::TLCSClassicPICtrl(const PlantParams& p)
    : pid_buck_(ctrl::PIDParams{
        .Kp   = 0.0163,
        .Ki   = 29.86,
        .Kd   = 0.0,
        .N    = 0.0,
        .uMin = D_MIN,
        .uMax = D_MAX,
        .Kb   = 1.0
      }, p.Ts)
    , pid_boost_(ctrl::PIDParams{
        .Kp   = 0.0058,
        .Ki   = 15.05,
        .Kd   = 0.0,
        .N    = 0.0,
        .uMin = D_MIN,
        .uMax = D_MAX,
        .Kb   = 1.0
      }, p.Ts)
    , hyst_(p.hysteresis)
{}

double TLCSClassicPICtrl::compute(const Vector2d& state, double v_ref, double V_in)
{
    updateMode(v_ref, V_in, hyst_);
    const double e = v_ref - state(1);
    double d_active;

    if (mode_ == ConvMode::BUCK) {
        d_active = pid_buck_.compute(e);
        pid_boost_.bumplessInit(d_active, e);   // sync inactive
    } else {
        d_active = pid_boost_.compute(e);
        pid_buck_.bumplessInit(d_active, e);    // sync inactive
    }
    return d_active;
}

void TLCSClassicPICtrl::reset()
{
    pid_buck_.reset();
    pid_boost_.reset();
    mode_ = ConvMode::BUCK;
}

// ===========================================================================
// 5. FuzzyPD  (FuzzyPD correction + steady-state feedforward)
//
// Scaling for 8-15 V range at 50 kHz:
//   e_scale  = 15 V  (full-range error)
//   de_scale = 1e5 V/s  (transient rate at 15 V / 150 us settling)
//   u_scale  = 0.25  (correction magnitude around feedforward d_ff)
// ===========================================================================
FuzzyPDCtrl::FuzzyPDCtrl(const PlantParams& p)
    : fpd_(ctrl::FuzzyPDParams{
        .e_scale  = 15.0,
        .de_scale = 1.0e5,
        .u_scale  = 0.25,
        .uMin     = -0.4,
        .uMax     =  0.4
      }, p.Ts)
    , hyst_(p.hysteresis)
{}

double FuzzyPDCtrl::compute(const Vector2d& state, double v_ref, double V_in)
{
    updateMode(v_ref, V_in, hyst_);
    const double e    = v_ref - state(1);
    const double d_ff = dSS(v_ref, V_in, mode_);
    const double corr = fpd_.compute(e);
    return clampD(d_ff + corr);
}

void FuzzyPDCtrl::reset()
{
    fpd_.reset();
    mode_ = ConvMode::BUCK;
}

// ===========================================================================
// 6. FuzzyPID_Buck  (FuzzyPID with buck gains, single mode across all scenarios)
//
// Scaling derived from paper buck PI (Kp=0.0163):
//   e_scale  = 15 V  (15/e_scale = 1 at full step)
//   de_scale = 1e5 V/s
//   u_scale  = 0.163  (matches Kp at e_norm=1)
//   Ki       = 29.86
// ===========================================================================
FuzzyPIDBuckCtrl::FuzzyPIDBuckCtrl(const PlantParams& p)
    : fpid_(ctrl::FuzzyPIDParams{
        .pd = ctrl::FuzzyPDParams{
            .e_scale  = 15.0,
            .de_scale = 1.0e5,
            .u_scale  = 0.163,
            .uMin     = -1.0,   // loose inner bounds; outer PID clamps to [0,1]
            .uMax     =  1.0
        },
        .Ki   = 29.86,
        .Kb   = 1.0,
        .uMin = D_MIN,
        .uMax = D_MAX
      }, p.Ts)
{}

double FuzzyPIDBuckCtrl::compute(const Vector2d& state, double v_ref, double /*V_in*/)
{
    return fpid_.compute(v_ref - state(1));
}

void FuzzyPIDBuckCtrl::reset() { fpid_.reset(); }

// ===========================================================================
// 7. FuzzyPID_Boost  (FuzzyPID with boost gains, single mode across all scenarios)
//
// Scaling derived from paper boost PI (Kp=0.0058):
//   u_scale = 0.058  (matches Kp at e_norm=1)
//   Ki      = 15.05
// ===========================================================================
FuzzyPIDBoostCtrl::FuzzyPIDBoostCtrl(const PlantParams& p)
    : fpid_(ctrl::FuzzyPIDParams{
        .pd = ctrl::FuzzyPDParams{
            .e_scale  = 15.0,
            .de_scale = 1.0e5,
            .u_scale  = 0.058,
            .uMin     = -1.0,   // loose inner bounds; outer PID clamps to [0,1]
            .uMax     =  1.0
        },
        .Ki   = 15.05,
        .Kb   = 1.0,
        .uMin = D_MIN,
        .uMax = D_MAX
      }, p.Ts)
{}

double FuzzyPIDBoostCtrl::compute(const Vector2d& state, double v_ref, double /*V_in*/)
{
    return fpid_.compute(v_ref - state(1));
}

void FuzzyPIDBoostCtrl::reset() { fpid_.reset(); }

// ===========================================================================
// 8. TLCS_FuzzyPI  (paper's main result)
//
// Two FuzzyPID controllers - one tuned for buck, one for boost.
// On each step:
//   - Active controller computes the duty cycle.
//   - Inactive controller is synchronised via bumplessInit() so that on a
//     mode switch its first output matches the departing controller's output.
// This implements the bump-less two-level control scheme (TLCS) of the paper.
// ===========================================================================
TLCSFuzzyPICtrl::TLCSFuzzyPICtrl(const PlantParams& p)
    : fpid_buck_(ctrl::FuzzyPIDParams{
        .pd = ctrl::FuzzyPDParams{
            .e_scale  = 15.0,
            .de_scale = 1.0e5,
            .u_scale  = 0.163,
            .uMin     = -1.0,   // loose inner bounds; outer PID clamps to [0,1]
            .uMax     =  1.0
        },
        .Ki   = 29.86,
        .Kb   = 1.0,
        .uMin = D_MIN,
        .uMax = D_MAX
      }, p.Ts)
    , fpid_boost_(ctrl::FuzzyPIDParams{
        .pd = ctrl::FuzzyPDParams{
            .e_scale  = 15.0,
            .de_scale = 1.0e5,
            .u_scale  = 0.058,
            .uMin     = -1.0,   // loose inner bounds; outer PID clamps to [0,1]
            .uMax     =  1.0
        },
        .Ki   = 15.05,
        .Kb   = 1.0,
        .uMin = D_MIN,
        .uMax = D_MAX
      }, p.Ts)
    , hyst_(p.hysteresis)
{}

double TLCSFuzzyPICtrl::compute(const Vector2d& state, double v_ref, double V_in)
{
    updateMode(v_ref, V_in, hyst_);
    const double e = v_ref - state(1);
    double d_active;

    if (mode_ == ConvMode::BUCK) {
        d_active = fpid_buck_.compute(e);
        fpid_boost_.bumplessInit(d_active, e);  // sync inactive boost controller
    } else {
        d_active = fpid_boost_.compute(e);
        fpid_buck_.bumplessInit(d_active, e);   // sync inactive buck controller
    }
    return d_active;
}

void TLCSFuzzyPICtrl::reset()
{
    fpid_buck_.reset();
    fpid_boost_.reset();
    mode_ = ConvMode::BUCK;
}

// ===========================================================================
// 9. GainScheduled  (scheduling variable xi = V_in/V_ref; range [0.1, 2.5])
//
// xi < 1  -> V_ref > V_in -> boost region -> schedule point 0.0 (boost PI)
// xi > 1  -> V_ref < V_in -> buck region  -> schedule point 2.5 (buck PI)
// LinearBlend mode: smooth interpolation across the boundary.
// ===========================================================================
GainScheduledCtrl::GainScheduledCtrl(const PlantParams& p)
    : gs_(p.Ts, ctrl::GainScheduleMode::LinearBlend)
{
    // Boost-tuned PI at xi=0.4 (V_ref >> V_in)
    auto pid_boost = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{
            .Kp   = 0.0058,
            .Ki   = 15.05,
            .Kd   = 0.0,
            .N    = 0.0,
            .uMin = D_MIN,
            .uMax = D_MAX,
            .Kb   = 1.0
        }, p.Ts);

    // Buck-tuned PI at xi=2.5 (V_ref << V_in)
    auto pid_buck = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{
            .Kp   = 0.0163,
            .Ki   = 29.86,
            .Kd   = 0.0,
            .N    = 0.0,
            .uMin = D_MIN,
            .uMax = D_MAX,
            .Kb   = 1.0
        }, p.Ts);

    gs_.addSchedulePoint(0.4, pid_boost);
    gs_.addSchedulePoint(2.5, pid_buck);
}

double GainScheduledCtrl::compute(const Vector2d& state, double v_ref, double V_in)
{
    const double xi = std::clamp(V_in / std::max(v_ref, 0.1), 0.4, 2.5);
    gs_.setSchedulingParam(xi);
    return clampD(gs_.compute(v_ref - state(1)));
}

void GainScheduledCtrl::reset() { gs_.reset(); }

// ===========================================================================
// 10. ADRC  (2nd-order LADRC; treats mode nonlinearity as total disturbance)
//
// Plant approximation (buck linearised):  v_C'' = b0 * d + f_total
// b0 = V_in / (L*C) = 10 / (50e-6 * 1.8e-3) = 1.111e8
//
// omega_c = 2000 rad/s (BW ~320 Hz, well within f_s/10 = 5 kHz limit)
// omega_o = 6000 rad/s  ->  omega_o * Ts = 6000 * 20e-6 = 0.12 < 0.5  (stable)
// ===========================================================================
ADRCConvCtrl::ADRCConvCtrl(const PlantParams& p)
    : adrc_(ctrl::ADRCParams{
        .omega_o = 6000.0,
        .omega_c = 2000.0,
        .b0      = p.V_in_nom / (p.L * p.C),  // ~1.111e8
        .uMin    = D_MIN,
        .uMax    = D_MAX
      }, p.Ts)
{}

double ADRCConvCtrl::compute(const Vector2d& state, double v_ref, double /*V_in*/)
{
    // DiscreteADRC convention: compute(r - y) = compute(v_ref - v_C)
    return adrc_.compute(v_ref - state(1));
}

void ADRCConvCtrl::reset() { adrc_.reset(); }

// ===========================================================================
// 11. MPC  (Np=30 Nc=5 on buck linearised SS)
// ===========================================================================
ctrl::StateSpace MPCConvCtrl::buildBuckSS(const PlantParams& p)
{
    return makeBuckSS(p);
}

MPCConvCtrl::MPCConvCtrl(const PlantParams& p)
{
    ctrl::StateSpace ss = buildBuckSS(p);

    ctrl::MPCParams mp;
    mp.Np       = 30;
    mp.Nc       = 5;
    mp.rho_y    = 10.0;   // output tracking weight
    mp.rho_u    = 1.0e4;  // move suppression (smooth duty-cycle changes)
    mp.uMin     = D_MIN;
    mp.uMax     = D_MAX;
    mp.duMin    = -0.3;
    mp.duMax    =  0.3;
    mp.qpMaxIter = 100;

    mpc_ = std::make_unique<ctrl::DiscreteMPC>(ss, mp);
}

double MPCConvCtrl::compute(const Vector2d& state, double v_ref, double /*V_in*/)
{
    Eigen::VectorXd x(2);
    x << state(0), state(1);
    Eigen::VectorXd r(1);
    r << v_ref;
    Eigen::VectorXd u = mpc_->computeRef(x, r);
    return clampD(u(0));
}

void MPCConvCtrl::reset() { mpc_->reset(); }

// ===========================================================================
// 12. LQR  (DiscreteLQR on buck model + steady-state feed-forward)
//
// u = d_ss + lqr_.compute(x - x_ref)(0)
//   = d_ss - K*(x - x_ref)
// where d_ss = V_ref/V_in (buck) or 1 - V_in/V_ref (boost)
//       x_ref = [V_ref/R, V_ref]  (nominal operating-point state)
// ===========================================================================
ctrl::StateSpace LQRConvCtrl::buildBuckSS(const PlantParams& p)
{
    return makeBuckSS(p);
}

static ctrl::DiscreteLQR makeBuckLQR(const PlantParams& p)
{
    ctrl::StateSpace ss = makeBuckSS(p);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2,2);
    Q(0,0) = 1.0 / (5.0*5.0);    // Bryson: i_L_max = 5 A
    Q(1,1) = 1.0 / (15.0*15.0);  // Bryson: v_C_max = 15 V
    Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1,1, 1.0/(0.5*0.5)); // d_max = 0.5 perturbation
    ctrl::LQRParams lp;
    lp.Q = Q;
    lp.R = R;
    return ctrl::DiscreteLQR(ss, lp);
}

LQRConvCtrl::LQRConvCtrl(const PlantParams& p)
    : lqr_(makeBuckLQR(p))
    , R_load_(p.R)
{}

double LQRConvCtrl::compute(const Vector2d& state, double v_ref, double V_in)
{
    updateMode(v_ref, V_in);

    // Nominal state at V_ref
    const double i_L_ref = (mode_ == ConvMode::BUCK)
                         ? (v_ref / R_load_)                       // buck: i_L_ss = V_ref/R
                         : (V_in * V_in / (std::max(v_ref, V_in+0.01) * R_load_)); // boost: i_L_ss
    Eigen::VectorXd x_ref(2);
    x_ref << i_L_ref, v_ref;

    Eigen::VectorXd xs(2);
    xs << state(0), state(1);

    // lqr_.compute(xs - x_ref) returns -K*(xs - x_ref)
    Eigen::VectorXd delta_d = lqr_.compute(xs, x_ref);
    const double d_ff = dSS(v_ref, V_in, mode_);
    return clampD(d_ff + delta_d(0));
}

} // namespace conv
