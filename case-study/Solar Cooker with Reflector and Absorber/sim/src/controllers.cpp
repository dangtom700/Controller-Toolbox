#include "controllers.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace Eigen;

namespace cooker {

// ---------------------------------------------------------------------------
// Helpers: build linearised 2-state SS at nominal operating point
// State: [T_abs, T_pot],  Input: f_shade,  Output: T_pot
//
// Nominal: T_abs=120^\circC, T_pot=95^\circC, T_amb=30^\circC, V=1 m/s, G=800 W/m^2, f_shade=0
// ---------------------------------------------------------------------------
static ctrl::StateSpace make2StateSS(const PlantParams& p)
{
    // Jacobian computed analytically at nominal:
    //   T_abs_nom=120^\circC, T_pot_nom=95^\circC, T_amb=30^\circC, V=1 m/s, G=800 W/m^2, f_shade=0
    const double C_abs = p.m_abs * p.cp_abs;            // 1800 J/K
    const double C_pot = p.m_pot * p.cp_pot;            // 8372 J/K
    const double h_wind_nom = 5.0 + 4.0 * 1.0;         // 9 W/(m^2K)
    const double T_abs_nom  = 120.0;                    // ^\circC
    const double T_abs_nom_K = T_abs_nom + 273.15;

    // dQ_rad/dT_abs = 4 * eps * sigma * A_abs * T_abs^3
    const double dQrad_dT = 4.0 * p.eps_abs * p.sigma * p.A_abs
                            * T_abs_nom_K * T_abs_nom_K * T_abs_nom_K;

    // A_c[0,0] = dT_abs_dot / dT_abs
    const double a00 = -((p.h_abs_amb + h_wind_nom) * p.A_abs
                        + p.h_abs_pot * p.A_c + dQrad_dT) / C_abs;
    // A_c[0,1] = dT_abs_dot / dT_pot
    const double a01 =  p.h_abs_pot * p.A_c / C_abs;
    // A_c[1,0] = dT_pot_dot / dT_abs
    const double a10 =  p.h_abs_pot * p.A_c / C_pot;
    // A_c[1,1] = dT_pot_dot / dT_pot
    const double a11 = -(p.h_abs_pot * p.A_c + p.U_loss * p.A_pot) / C_pot;

    // B_c[0,0] = dT_abs_dot / d(f_shade): shade REDUCES solar input
    // Q_solar = (1-f_shade)*(eta_box+eta_TBPR)*G*A_box
    const double G_nom     = 800.0;
    const double P_solar   = (p.eta_box + p.eta_TBPR) * G_nom * p.A_box;
    const double b0_c      = -P_solar / C_abs;   // negative: more shade -> less T_abs
    const double b1_c      = 0.0;                // f_shade doesn't directly affect T_pot

    MatrixXd Ac(2,2);
    Ac << a00, a01,
          a10, a11;
    MatrixXd Bc(2,1);
    Bc << b0_c, b1_c;
    MatrixXd Cc(1,2); Cc << 0.0, 1.0;  // output = T_pot
    MatrixXd Dc(1,1); Dc << 0.0;

    ctrl::StateSpace ss_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ss_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// FOPDT model of T_pot vs f_shade:
//   tau = 600 s, K_gain = 0.8 (sign accounted for below)
//   a = exp(-Ts/tau), b_discrete = -K*(1-a)  [negative gain]
static ctrl::StateSpace makeFOPDTSS(const PlantParams& p)
{
    const double tau = 600.0;    // thermal time constant [s]
    const double K   = 0.8;     // steady-state gain magnitude (^\circC/unit_shade)
    const double a   = std::exp(-p.Ts / tau);
    const double b   = -K * (1.0 - a);  // negative: more shade -> lower T_pot

    MatrixXd Ad(1,1); Ad << a;
    MatrixXd Bd(1,1); Bd << b;
    MatrixXd Cd(1,1); Cd << 1.0;
    MatrixXd Dd(1,1); Dd << 0.0;
    return ctrl::StateSpace(Ad, Bd, Cd, Dd, p.Ts);
}

// ===========================================================================
// 2. PID
// Kp=0.005, Ki=0.0001, Kd=0.02, N=5
// Direct acting: e = T_pot - T_ref -> positive output = more shade.
// ===========================================================================
PIDCookerCtrl::PIDCookerCtrl(const PlantParams& p)
    : pid_(ctrl::PIDParams{
        .Kp   = 0.005,
        .Ki   = 0.0001,
        .Kd   = 0.02,
        .N    = 5.0,
        .uMin = F_MIN,
        .uMax = F_MAX,
        .Kb   = 1.0
      }, p.Ts)
{}

double PIDCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    return clampShade(pid_.compute(state(1) - T_ref));
}

void PIDCookerCtrl::reset() { pid_.reset(); }

// ===========================================================================
// 3. ADRC
// Direct acting: compute(T_pot - T_ref) == compute(y - r).
// omega_o=0.013, omega_c=0.004, b0=0.002.
// omega_o * Ts = 0.013 * 30 = 0.39 < 0.5 (check)
// ===========================================================================
ADRCCookerCtrl::ADRCCookerCtrl(const PlantParams& p)
    : adrc_(ctrl::ADRCParams{
        .omega_o = 0.013,
        .omega_c = 0.004,
        .b0      = 0.002,
        .uMin    = F_MIN,
        .uMax    = F_MAX
      }, p.Ts)
{}

double ADRCCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    // compute(y - r): positive when T_pot > T_ref -> shade more (check)
    return clampShade(adrc_.compute(state(1) - T_ref));
}

void ADRCCookerCtrl::reset() { adrc_.reset(); }

// ===========================================================================
// 4. MPC
// FOPDT model in deviation form around T_ref.
// B is negative -> MPC increases f_shade deviation when T_pot > T_ref.
// Np=20, Nu=5, rho_y=1.0, rho_u=10.
// ===========================================================================
MPCCookerCtrl::MPCCookerCtrl(const PlantParams& p)
    : T_pot_nom_(95.0)
{
    ctrl::StateSpace ss = makeFOPDTSS(p);

    ctrl::MPCParams mp;
    mp.Np        = 20;
    mp.Nc        = 5;
    mp.rho_y     = 1.0;
    mp.rho_u     = 10.0;
    mp.uMin      = F_MIN - 0.5;   // deviation can be negative (clamp externally)
    mp.uMax      = F_MAX + 0.5;
    mp.duMin     = -0.2;
    mp.duMax     =  0.2;
    mp.qpMaxIter = 200;

    mpc_ = std::make_unique<ctrl::DiscreteMPC>(ss, mp);
}

double MPCCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    // Deviation variables
    const double x_dev = state(1) - T_pot_nom_;
    const double r_dev = T_ref    - T_pot_nom_;

    VectorXd x(1); x << x_dev;
    VectorXd r(1); r << r_dev;

    mpc_->setState(x);
    VectorXd u = mpc_->computeRef(x, r);
    return clampShade(u(0));
}

void MPCCookerCtrl::reset()
{
    mpc_->reset();
    VectorXd x0(1); x0 << 0.0;
    mpc_->setState(x0);
}

// ===========================================================================
// 5. FuzzyPID
// e_scale=30 -> +/-30^\circC maps to [-1,1]; u_scale=0.3; direct acting.
// ===========================================================================
FuzzyPIDCookerCtrl::FuzzyPIDCookerCtrl(const PlantParams& p)
    : fpid_(ctrl::FuzzyPIDParams{
        .pd = ctrl::FuzzyPDParams{
            .e_scale  = 30.0,
            .de_scale = 2.0,
            .u_scale  = 0.3,
            .uMin     = -0.5,   // loose inner bounds; outer clamp to [0,1]
            .uMax     =  0.5
        },
        .Ki   = 0.0001,
        .Kb   = 1.0,
        .uMin = F_MIN,
        .uMax = F_MAX
      }, p.Ts)
{}

double FuzzyPIDCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    return clampShade(fpid_.compute(state(1) - T_ref));
}

void FuzzyPIDCookerCtrl::reset() { fpid_.reset(); }

// ===========================================================================
// 6. SMC
// compute(y - ref) = compute(T_pot - T_ref) (check) matches convention.
// phi=2.0 (boundary layer in ^\circC), K=0.05.
// ===========================================================================
SMCCookerCtrl::SMCCookerCtrl(const PlantParams& p)
    : smc_(ctrl::SMCParams{
        .c_e  = 1.0,
        .c_de = 0.0,    // first-order sliding surface on e alone
        .K    = 0.05,
        .phi  = 2.0,
        .uMin = F_MIN,
        .uMax = F_MAX
      }, p.Ts)
{}

double SMCCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    return clampShade(smc_.compute(state(1) - T_ref));
}

void SMCCookerCtrl::reset() { smc_.reset(); }

// ===========================================================================
// 7. GainScheduled
// Scheduling on T_pot:
//   T_pot < T_melt (55^\circC):  PID_low  (gentle heating phase)
//   T_pot < 80^\circC:           PID_mid  (approaching boil)
//   T_pot >= 80^\circC:          PID_hi   (near-boil regulation)
// Each PID direct acting: compute(T_pot - T_ref).
// ===========================================================================
GainScheduledCookerCtrl::GainScheduledCookerCtrl(const PlantParams& p)
    : gs_(p.Ts, ctrl::GainScheduleMode::LinearBlend)
{
    // Low-temp bracket: small gains, mostly open (T_pot below PCM melt)
    auto pid_low = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{
            .Kp   = 0.001,
            .Ki   = 0.00002,
            .Kd   = 0.005,
            .N    = 5.0,
            .uMin = F_MIN,
            .uMax = F_MAX,
            .Kb   = 1.0
        }, p.Ts);

    // Mid-temp bracket: standard PID gains
    auto pid_mid = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{
            .Kp   = 0.005,
            .Ki   = 0.0001,
            .Kd   = 0.02,
            .N    = 5.0,
            .uMin = F_MIN,
            .uMax = F_MAX,
            .Kb   = 1.0
        }, p.Ts);

    // High-temp bracket: stronger gains for near-boil regulation
    auto pid_hi = std::make_shared<ctrl::DiscretePID>(
        ctrl::PIDParams{
            .Kp   = 0.012,
            .Ki   = 0.0003,
            .Kd   = 0.05,
            .N    = 5.0,
            .uMin = F_MIN,
            .uMax = F_MAX,
            .Kb   = 1.0
        }, p.Ts);

    gs_.addSchedulePoint(p.T_melt,  pid_low);   // below PCM melt
    gs_.addSchedulePoint(80.0,       pid_mid);   // between melt and near-boil
    gs_.addSchedulePoint(95.0,       pid_hi);    // near-boil regulation
}

double GainScheduledCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    const double T_pot = state(1);
    gs_.setSchedulingParam(std::clamp(T_pot,
                                       1.0,     // lower bound (below PCM melt)
                                       95.0));   // upper bound (near-boil)
    return clampShade(gs_.compute(T_pot - T_ref));
}

void GainScheduledCookerCtrl::reset() { gs_.reset(); }

// ===========================================================================
// 8. MRAC
// setReference(T_ref), compute(T_pot).
// a_m=exp(-0.002*Ts)=exp(-0.06)approx =0.9418, b_m=0.0582 (DC gain = 1).
// uMin=0, uMax=1 (absolute f_shade output).
// ===========================================================================
MRACCookerCtrl::MRACCookerCtrl(const PlantParams& p)
    : mrac_([&]() {
        const double a_m = std::exp(-0.002 * p.Ts);  // approx = 0.9418
        ctrl::MRACParams mp;
        mp.a_m       = a_m;
        mp.b_m       = 1.0 - a_m;
        mp.gamma_r   = 0.01;
        mp.gamma_y   = 0.01;
        mp.sigma     = 0.01;
        mp.theta_max = 50.0;
        mp.uMin      = F_MIN;
        mp.uMax      = F_MAX;
        return ctrl::MRACController(mp, p.Ts);
      }())
{}

double MRACCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    mrac_.setReference(T_ref);
    return clampShade(mrac_.compute(state(1)));
}

void MRACCookerCtrl::reset() { mrac_.reset(); }

// ===========================================================================
// 9. L1Adaptive
// setReference(T_ref), compute(T_pot).
// a_m=exp(-0.002*30)=0.9418, b_m=0.0582, omega_c=0.005.
// uMin=0, uMax=1 (absolute f_shade output).
// ===========================================================================
L1AdaptiveCookerCtrl::L1AdaptiveCookerCtrl(const PlantParams& p)
    : l1_([&]() {
        const double a_m = std::exp(-0.002 * p.Ts);
        ctrl::L1AdaptiveController::Params lp;
        lp.a_m       = a_m;
        lp.b_m       = 1.0 - a_m;
        lp.k_g       = 1.0;
        lp.Gamma     = 10.0;
        lp.omega_c   = 0.005;
        lp.sigma_max = 5.0;
        lp.Q_lyap    = 1.0;
        lp.uMin      = F_MIN;
        lp.uMax      = F_MAX;
        return ctrl::L1AdaptiveController(lp, p.Ts);
      }())
{}

double L1AdaptiveCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    l1_.setReference(T_ref);
    return clampShade(l1_.compute(state(1)));
}

void L1AdaptiveCookerCtrl::reset() { l1_.reset(); }

// ===========================================================================
// 10. NeuralPID
// compute(T_pot - T_ref); plant_gain negative (more shade -> lower T_pot).
// n_h=6, lr=1e-5; initial gains tuned to match PID #2.
// ===========================================================================
NeuralPIDCookerCtrl::NeuralPIDCookerCtrl(const PlantParams& p)
    : npid_(ctrl::NeuralPID::Params{
        .n_hidden        = 6,
        .lr              = 1e-5,
        .Ts              = p.Ts,
        .plant_gain      = -0.002 * p.Ts,  // negative: shade -> lower T_pot
        .max_weight_norm = 5.0,
        .uMin            = F_MIN,
        .uMax            = F_MAX,
        .Kp0             = 0.005,
        .Ki0             = 0.0001,
        .Kd0             = 0.0
      })
{}

double NeuralPIDCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    return clampShade(npid_.compute(state(1) - T_ref));
}

void NeuralPIDCookerCtrl::reset() { npid_.reset(); }

// ===========================================================================
// 11. DynaCtrl
// DynaController wrapping PID. n_collect=50, n_refit_every=25.
// Same error convention as PID: e = T_pot - T_ref.
// ===========================================================================
DynaCookerCtrl::DynaCookerCtrl(const PlantParams& p)
    : dyna_(ctrl::DynaController::Params{
        .n_collect     = 50,
        .n_refit_every = 25,
        .Ts            = p.Ts
      },
      std::make_shared<ctrl::DiscretePID>(
          ctrl::PIDParams{
              .Kp   = 0.005,
              .Ki   = 0.0001,
              .Kd   = 0.02,
              .N    = 5.0,
              .uMin = F_MIN,
              .uMax = F_MAX,
              .Kb   = 1.0
          }, p.Ts))
{}

double DynaCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    return clampShade(dyna_.compute(state(1) - T_ref));
}

void DynaCookerCtrl::reset() { dyna_.reset(); }

// ===========================================================================
// 12. ScenarioMPC
// 2-state linearised model [T_abs, T_pot], output = T_pot.
// Sigma_w = diag(4.0, 0.25) [G/T_amb noise on T_abs and T_pot states].
// Np=20, Nu=5, N_samples=15.
// ===========================================================================
ctrl::StateSpace ScenarioMPCCookerCtrl::buildSS(const PlantParams& p)
{
    return make2StateSS(p);
}

ScenarioMPCCookerCtrl::ScenarioMPCCookerCtrl(const PlantParams& p)
    : smpc_([&]() {
        ctrl::StateSpace sys = buildSS(p);

        ctrl::ScenarioMPCParams sp;
        sp.Np        = 20;
        sp.Nu        = 5;
        sp.Q         = Eigen::MatrixXd::Constant(1, 1, 1.0);   // T_pot output weight
        sp.R         = Eigen::MatrixXd::Constant(1, 1, 10.0);  // f_shade move weight
        sp.Sigma_w   = Eigen::MatrixXd::Zero(2, 2);
        sp.Sigma_w(0,0) = 4.0;   // T_abs state noise variance (G uncertainty -> 2^\circC std)
        sp.Sigma_w(1,1) = 0.25;  // T_pot state noise variance (T_amb -> 0.5^\circC std)
        sp.N_samples = 15;
        sp.uMin      = Eigen::VectorXd::Constant(1, F_MIN - 0.5);
        sp.uMax      = Eigen::VectorXd::Constant(1, F_MAX + 0.5);
        sp.Ts        = p.Ts;
        return ctrl::ScenarioMPC(sys, sp);
      }())
    , T_abs_nom_(120.0)
    , T_pot_nom_(95.0)
{}

double ScenarioMPCCookerCtrl::compute(const Vector2d& state, double T_ref)
{
    // Deviation variables
    VectorXd x_dev(2);
    x_dev << state(0) - T_abs_nom_,
              state(1) - T_pot_nom_;

    VectorXd r(1);
    r(0) = T_ref - T_pot_nom_;

    smpc_.setState(x_dev);
    smpc_.setReference(r);
    VectorXd u = smpc_.computeControl();
    return clampShade(u(0));
}

void ScenarioMPCCookerCtrl::reset() { smpc_.reset(); }

} // namespace cooker
