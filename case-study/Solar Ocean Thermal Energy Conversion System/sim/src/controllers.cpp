#include "controllers.h"
#include <cmath>
#include <algorithm>

using namespace Eigen;

namespace sotec {

// Nominal operating point used for model-based controllers
static constexpr double T_H_NOM    = 63.0;   // [^\circC]
static constexpr double T_COLL_NOM = 70.0;   // [^\circC]
static constexpr double M_F_NOM    = 0.30;   // collector pump [kg/s]
static constexpr double M_WF_NOM   = 0.04;   // WF pump [kg/s]

// FOPDT parameters for T_h model (used by PID/MPC/Dyna/NeuralPID)
//   tau = 600s, K = 30^\circC/(kg/s), a = exp(-Ts/tau), b = K*(1-a)
static double fopdt_a(double Ts) { return std::exp(-Ts / 600.0); }
static double fopdt_b(double Ts) { const double a = fopdt_a(Ts); return 30.0 * (1.0 - a); }

// ---------------------------------------------------------------------------
// Linearised 2-state SS around (T_H_NOM, T_COLL_NOM, M_F_NOM, M_WF_NOM)
// State: [T_h, T_coll],  Input: m_dot_f,  Output: T_h
// ---------------------------------------------------------------------------
static ctrl::StateSpace buildThermalSS(const PlantParams& p,
                                        double T_h_n, double T_coll_n)
{
    // dT_h/dt   = [m_dot_f*cp_f*(T_coll-T_h) - m_dot_wf*cp_wf*(T_h-T_c)] / (m_tank*cp_f)
    // dT_coll/dt= [eta*A*G - U*A*(T_coll-T_amb) - m_dot_f*cp_f*(T_coll-T_h)] / (m_coll*cp_f)
    const double cp_f  = p.cp_f;
    const double cp_wf = p.cp_wf;
    const double mf_n  = M_F_NOM;
    const double mwf_n = M_WF_NOM;

    MatrixXd Ac(2, 2);
    Ac(0, 0) = -(mf_n * cp_f + mwf_n * cp_wf) / (p.m_tank * cp_f);
    Ac(0, 1) =   mf_n / p.m_tank;
    Ac(1, 0) =   mf_n / p.m_coll;
    Ac(1, 1) = -(p.U_coll * p.A_coll / cp_f + mf_n) / p.m_coll;

    const double dT = T_coll_n - T_h_n;
    MatrixXd Bc(2, 1);
    Bc(0, 0) =  dT / p.m_tank;
    Bc(1, 0) = -dT / p.m_coll;

    MatrixXd Cc(1, 2); Cc << 1.0, 0.0;
    MatrixXd Dc(1, 1); Dc << 0.0;

    ctrl::StateSpace ss_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ss_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// ===========================================================================
// 1. OpenLoop - constant mid-range flows
// ===========================================================================
CtrlOutput OpenLoopCtrl::compute(const Eigen::Vector2d& state, double, double, double)
{
    return { clampMdotF(M_F_NOM, p_), m_dot_wf_ff(state(0), p_) };
}

// ===========================================================================
// 2. PID
// ===========================================================================
PIDThCtrl::PIDThCtrl(const PlantParams& p)
    : pid_(ctrl::PIDParams{
        .Kp   = 0.008,
        .Ki   = 0.0002,
        .Kd   = 0.05,
        .N    = 5.0,
        .uMin = p.m_dot_f_min,
        .uMax = p.m_dot_f_max,
        .Kb   = 0.5
      }, p.Ts)
    , p_(p)
{}

CtrlOutput PIDThCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                               double T_c, double)
{
    double e    = T_h_ref - state(0);
    double mf   = clampMdotF(pid_.compute(e), p_);
    double mwf  = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void PIDThCtrl::reset() { pid_.reset(); }

// ===========================================================================
// 3. ADRC
// omega_o * Ts = 0.013 * 30 = 0.39 < 0.5  (backward-Euler stable)
// ===========================================================================
ADRCCtrl::ADRCCtrl(const PlantParams& p)
    : adrc_(ctrl::ADRCParams{
        .omega_o = 0.013,
        .omega_c = 0.004,
        .b0      = fopdt_b(p.Ts),   // approximate plant gain per step
        .uMin    = p.m_dot_f_min,
        .uMax    = p.m_dot_f_max
      }, p.Ts)
    , p_(p)
{}

CtrlOutput ADRCCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                              double T_c, double)
{
    double mf  = clampMdotF(adrc_.compute(T_h_ref - state(0)), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void ADRCCtrl::reset() { adrc_.reset(); }

// ===========================================================================
// 4. MPC - FOPDT T_h model in deviation form
// ===========================================================================
static std::unique_ptr<ctrl::DiscreteMPC> makeSotecMPC(const PlantParams& p)
{
    const double a = fopdt_a(p.Ts);
    const double b = fopdt_b(p.Ts);

    MatrixXd Ad(1,1); Ad << a;
    MatrixXd Bd(1,1); Bd << b;
    MatrixXd Cd(1,1); Cd << 1.0;
    MatrixXd Dd(1,1); Dd << 0.0;

    ctrl::StateSpace ss(Ad, Bd, Cd, Dd, p.Ts);

    ctrl::MPCParams mp;
    mp.Np    = 20;
    mp.Nc    = 5;
    mp.rho_y = 1.0;
    mp.rho_u = 0.05;
    mp.uMin  = p.m_dot_f_min - M_F_NOM;
    mp.uMax  = p.m_dot_f_max - M_F_NOM;
    mp.duMin = -0.05;
    mp.duMax =  0.05;

    return std::make_unique<ctrl::DiscreteMPC>(ss, mp);
}

MPCCtrl::MPCCtrl(const PlantParams& p)
    : mpc_(makeSotecMPC(p))
    , p_(p)
    , T_h_nom_(T_H_NOM)
{}

CtrlOutput MPCCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                             double T_c, double)
{
    VectorXd x_dev(1); x_dev(0) = state(0) - T_h_nom_;
    double   r_dev = T_h_ref - T_h_nom_;
    mpc_->setState(x_dev);
    VectorXd u_dev = mpc_->computeRef(x_dev, VectorXd::Constant(1, r_dev));
    double mf   = clampMdotF(M_F_NOM + u_dev(0), p_);
    double mwf  = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void MPCCtrl::reset() {
    mpc_->reset();
    VectorXd x0(1); x0 << 0.0;
    mpc_->setState(x0);
}

// ===========================================================================
// 5. LQR - Bryson 2-state
// ===========================================================================
ctrl::StateSpace LQRCtrl::buildSS(const PlantParams& p,
                                   double T_h_n, double T_coll_n)
{
    return buildThermalSS(p, T_h_n, T_coll_n);
}

LQRCtrl::LQRCtrl(const PlantParams& p)
    : p_(p)
    , T_h_nom_(T_H_NOM)
    , T_coll_nom_(T_COLL_NOM)
    , m_dot_f_nom_(M_F_NOM)
{
    ctrl::StateSpace ss_d = buildSS(p, T_H_NOM, T_COLL_NOM);

    ctrl::LQRParams lp;
    lp.Q = MatrixXd::Zero(2, 2);
    lp.Q(0, 0) = 1.0 / (5.0  * 5.0);   // T_h: 5^\circC max deviation
    lp.Q(1, 1) = 1.0 / (10.0 * 10.0);  // T_coll: 10^\circC max deviation
    lp.R = MatrixXd::Constant(1, 1, 1.0 / (0.2 * 0.2));  // Deltam_dot_f: 0.2 kg/s max

    ctrl::DiscreteLQR lqr_design(ss_d, lp);
    K_ = lqr_design.gainMatrix();   // 1x2

    x_nom_ << T_H_NOM, T_COLL_NOM;
}

CtrlOutput LQRCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                              double T_c, double)
{
    // Track T_h_ref: adjust x_nom_(0) to current reference
    x_nom_(0) = T_h_ref;

    VectorXd x_dev = state.cast<double>() - x_nom_;
    double u_dev = -(K_ * x_dev)(0);
    double mf    = clampMdotF(m_dot_f_nom_ + u_dev, p_);
    double mwf   = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

// ===========================================================================
// 6. FuzzyPID
// Output is a m_dot_f deviation; clamp around M_F_NOM.
// ===========================================================================
FuzzyPIDCtrl::FuzzyPIDCtrl(const PlantParams& p)
    : fpid_(ctrl::FuzzyPIDParams{
        .pd = ctrl::FuzzyPDParams{
            .e_scale  = 10.0,
            .de_scale = 0.5,
            .u_scale  = 0.15,
            .uMin     = -(M_F_NOM - p.m_dot_f_min),   // loose inner bounds
            .uMax     =   p.m_dot_f_max - M_F_NOM
        },
        .Ki   = 0.0002,
        .Kb   = 0.5,
        .uMin = -(M_F_NOM - p.m_dot_f_min),
        .uMax =   p.m_dot_f_max - M_F_NOM
      }, p.Ts)
    , p_(p)
{}

CtrlOutput FuzzyPIDCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                                   double T_c, double)
{
    double e   = T_h_ref - state(0);
    double mf  = clampMdotF(M_F_NOM + fpid_.compute(e), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void FuzzyPIDCtrl::reset() { fpid_.reset(); }

// ===========================================================================
// 7. MRAC
// Reference model: a_m approx = exp(-Ts/600), b_m = 1 - a_m  (DC gain = 1)
// setReference(T_h_ref) then compute(T_h) -> outputs m_dot_f directly.
// ===========================================================================
MRACCtrl::MRACCtrl(const PlantParams& p)
    : mrac_(ctrl::MRACParams{
        .a_m       = fopdt_a(p.Ts),
        .b_m       = 1.0 - fopdt_a(p.Ts),
        .gamma_r   = 0.5,
        .gamma_y   = 0.5,
        .sigma     = 0.02,
        .theta_max = 20.0,
        .uMin      = p.m_dot_f_min,
        .uMax      = p.m_dot_f_max
      }, p.Ts)
    , p_(p)
{}

CtrlOutput MRACCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                               double T_c, double)
{
    mrac_.setReference(T_h_ref);
    double mf  = clampMdotF(mrac_.compute(state(0)), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void MRACCtrl::reset() { mrac_.reset(); }

// ===========================================================================
// 8. L1Adaptive
// ===========================================================================
L1AdaptiveCtrl::L1AdaptiveCtrl(const PlantParams& p)
    : l1_(ctrl::L1AdaptiveController::Params{
        .a_m       = fopdt_a(p.Ts),
        .b_m       = 1.0 - fopdt_a(p.Ts),
        .k_g       = 1.0,
        .Gamma     = 10.0,
        .omega_c   = 0.003,
        .sigma_max = 20.0,
        .Q_lyap    = 1.0,
        .uMin      = p.m_dot_f_min,
        .uMax      = p.m_dot_f_max
      }, p.Ts)
    , p_(p)
{}

CtrlOutput L1AdaptiveCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                                    double T_c, double)
{
    l1_.setReference(T_h_ref);
    double mf  = clampMdotF(l1_.compute(state(0)), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void L1AdaptiveCtrl::reset() { l1_.reset(); }

// ===========================================================================
// 9. GainScheduled - 3 PIDs scheduled on T_h
// ===========================================================================
GainScheduledCtrl::GainScheduledCtrl(const PlantParams& p)
    : gs_(p.Ts)
    , p_(p)
{
    // Low T_h (cold startup): aggressive gain
    gs_.addSchedulePoint(50.0,
        std::make_shared<ctrl::DiscretePID>(
            ctrl::PIDParams{.Kp=0.020,.Ki=0.0004,.Kd=0.1,.N=5,
                            .uMin=p.m_dot_f_min,.uMax=p.m_dot_f_max,.Kb=0.5}, p.Ts));

    // Nominal operating point (around setpoint 63^\circC)
    gs_.addSchedulePoint(63.0,
        std::make_shared<ctrl::DiscretePID>(
            ctrl::PIDParams{.Kp=0.008,.Ki=0.0002,.Kd=0.05,.N=5,
                            .uMin=p.m_dot_f_min,.uMax=p.m_dot_f_max,.Kb=0.5}, p.Ts));

    // High T_h (near/above max setpoint): soft correction, avoid pressure limit
    gs_.addSchedulePoint(72.0,
        std::make_shared<ctrl::DiscretePID>(
            ctrl::PIDParams{.Kp=0.003,.Ki=0.0001,.Kd=0.0,.N=5,
                            .uMin=p.m_dot_f_min,.uMax=p.m_dot_f_max,.Kb=0.5}, p.Ts));
}

CtrlOutput GainScheduledCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                                       double T_c, double)
{
    double e  = T_h_ref - state(0);
    gs_.setSchedulingParam(state(0));  // schedule on current T_h
    double mf  = clampMdotF(gs_.compute(e), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void GainScheduledCtrl::reset() { gs_.reset(); }

// ===========================================================================
// 10. ScenarioMPC
// ===========================================================================
static ctrl::ScenarioMPC makeSotecScenarioMPC(const PlantParams& p)
{
    ctrl::StateSpace ss_d = buildThermalSS(p, T_H_NOM, T_COLL_NOM);

    ctrl::ScenarioMPCParams sp;
    sp.Np        = 20;
    sp.Nu        = 5;
    sp.Q         = MatrixXd::Identity(1, 1) * 2.0;          // T_h output tracking (pp=1)
    sp.R         = MatrixXd::Constant(1, 1, 50.0);           // m_dot_f deviation effort
    sp.Sigma_w   = MatrixXd::Identity(2, 2) * 0.25;          // 0.5^\circC std state noise (n=2)
    sp.N_samples = 20;
    sp.uMin      = VectorXd::Constant(1, p.m_dot_f_min - M_F_NOM);
    sp.uMax      = VectorXd::Constant(1, p.m_dot_f_max - M_F_NOM);
    sp.Ts        = p.Ts;

    return ctrl::ScenarioMPC(ss_d, sp);
}

ScenarioMPCCtrl::ScenarioMPCCtrl(const PlantParams& p)
    : smpc_(makeSotecScenarioMPC(p))
    , p_(p)
    , T_h_nom_(T_H_NOM)
    , T_coll_nom_(T_COLL_NOM)
{}

CtrlOutput ScenarioMPCCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                                     double T_c, double)
{
    VectorXd x_dev(2);
    x_dev(0) = state(0) - T_h_nom_;
    x_dev(1) = state(1) - T_coll_nom_;

    VectorXd r_dev(1); r_dev(0) = T_h_ref - T_h_nom_;

    smpc_.setState(x_dev);
    smpc_.setReference(r_dev);
    VectorXd u_dev = smpc_.computeControl();

    double mf  = clampMdotF(M_F_NOM + u_dev(0), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void ScenarioMPCCtrl::reset() { smpc_.reset(); }

// ===========================================================================
// 11. DynaCtrl - DynaController wrapping PID, deviation output
// ===========================================================================
DynaCtrl::DynaCtrl(const PlantParams& p)
    : dyna_(ctrl::DynaController::Params{
        .n_collect     = 50,
        .n_refit_every = 25,
        .Ts            = p.Ts
      },
      std::make_shared<ctrl::DiscretePID>(
          ctrl::PIDParams{
              .Kp   =  0.008,
              .Ki   =  0.0002,
              .Kd   =  0.0,
              .N    =  5.0,
              .uMin = -(M_F_NOM - p.m_dot_f_min),
              .uMax =   p.m_dot_f_max - M_F_NOM,
              .Kb   =  0.5
          }, p.Ts))
    , p_(p)
{}

CtrlOutput DynaCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                               double T_c, double)
{
    double e   = T_h_ref - state(0);
    double mf  = clampMdotF(M_F_NOM + dyna_.compute(e), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void DynaCtrl::reset() { dyna_.reset(); }

// ===========================================================================
// 12. NeuralPID
// plant_gain approx = fopdt_b(Ts) = K*(1-a) [^\circC/(kg/s)] per time step
// uMin/uMax are deviation bounds around M_F_NOM.
// ===========================================================================
NeuralPIDCtrl::NeuralPIDCtrl(const PlantParams& p)
    : npid_(ctrl::NeuralPID::Params{
        .n_hidden   = 6,
        .lr         = 1e-4,
        .Ts         = p.Ts,
        .plant_gain = fopdt_b(p.Ts),
        .uMin       = -(M_F_NOM - p.m_dot_f_min),
        .uMax       =   p.m_dot_f_max - M_F_NOM,
        .Kp0        = 0.008,
        .Ki0        = 0.0002,
        .Kd0        = 0.0
      })
    , p_(p)
{}

CtrlOutput NeuralPIDCtrl::compute(const Eigen::Vector2d& state, double T_h_ref,
                                   double T_c, double)
{
    double e   = T_h_ref - state(0);
    double mf  = clampMdotF(M_F_NOM + npid_.compute(e), p_);
    double mwf = m_dot_wf_ff(state(0), p_);
    return { mf, mwf };
}

void NeuralPIDCtrl::reset() { npid_.reset(); }

} // namespace sotec
