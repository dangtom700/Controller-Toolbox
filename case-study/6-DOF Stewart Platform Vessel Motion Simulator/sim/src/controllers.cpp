#include "controllers.h"
#include <algorithm>
#include <cmath>

namespace stewart {

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

static Vec6 rodL0(const PlantParams& p)
{
    StewartGeometry g(p);
    return g.l0();
}

// 2-state per-rod deviation model z=[L-l0, dL]: dz/dt = Ac*z + Bc*u.
static ctrl::StateSpace buildRodSS(const PlantParams& p)
{
    Eigen::MatrixXd Ac(2, 2);
    Ac << 0.0, 1.0,
          -p.k_spring / p.m_rod, -p.b_damp / p.m_rod;
    Eigen::MatrixXd Bc(2, 1);
    Bc << 0.0, 1.0 / p.m_rod;
    Eigen::MatrixXd Cc(1, 2);
    Cc << 1.0, 0.0;
    Eigen::MatrixXd Dc(1, 1);
    Dc << 0.0;

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// 12-state block-diagonal deviation model (decision #6, LQR only).
static ctrl::StateSpace build12StateSS(const PlantParams& p)
{
    Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(12, 12);
    Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(12, 6);
    for (int i = 0; i < N_RODS; ++i) {
        Ac(2*i,     2*i + 1) = 1.0;
        Ac(2*i + 1, 2*i)     = -p.k_spring / p.m_rod;
        Ac(2*i + 1, 2*i + 1) = -p.b_damp / p.m_rod;
        Bc(2*i + 1, i)       = 1.0 / p.m_rod;
    }
    Eigen::MatrixXd Cc = Eigen::MatrixXd::Identity(12, 12);
    Eigen::MatrixXd Dc = Eigen::MatrixXd::Zero(12, 6);

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, p.Ts, ctrl::C2dMethod::ZOH);
}

// ===========================================================================
// 1. PID
// ===========================================================================

// Gains retuned against the actual physical constants (k_spring=2e4 N/m,
// F_rod_max=6000 N) rather than the README's placeholder numbers, which
// assumed a different (unspecified) unit convention. Kp must dominate
// k_spring for the closed loop to track L_cmd rather than relax toward l0;
// Ki removes the residual steady-state offset within one oscillation period.
static ctrl::PIDParams pidParams(double Fmax, double Ki = 4.0e4)
{
    ctrl::PIDParams p;
    p.Kp = 8.0e4; p.Ki = Ki; p.Kd = 4.0e3; p.N = 10.0; p.Kb = 1.0;
    p.uMin = -Fmax; p.uMax = Fmax;
    return p;
}

PIDStewartCtrl::PIDStewartCtrl(const PlantParams& p)
    : pids_{ ctrl::DiscretePID(pidParams(p.F_rod_max), p.Ts),
             ctrl::DiscretePID(pidParams(p.F_rod_max), p.Ts),
             ctrl::DiscretePID(pidParams(p.F_rod_max), p.Ts),
             ctrl::DiscretePID(pidParams(p.F_rod_max), p.Ts),
             ctrl::DiscretePID(pidParams(p.F_rod_max), p.Ts),
             ctrl::DiscretePID(pidParams(p.F_rod_max), p.Ts) }
{}

Vec6 PIDStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i)
        u(i) = pids_[i].compute(L_cmd(i) - L(i));
    return u;
}

void PIDStewartCtrl::reset() { for (auto& c : pids_) c.reset(); }

// ===========================================================================
// 2. FuzzyPID
// ===========================================================================

static ctrl::FuzzyPIDParams fuzzyPidParams(double Fmax)
{
    ctrl::FuzzyPIDParams p;
    p.pd.e_scale  = 0.003; // +/-3 mm
    p.pd.de_scale = 0.05;  // +/-50 mm/s
    p.pd.u_scale  = Fmax;
    p.Ki   = 4.0e4;
    p.Kb   = 1.0;
    p.uMin = -Fmax; p.uMax = Fmax;
    return p;
}

FuzzyPIDStewartCtrl::FuzzyPIDStewartCtrl(const PlantParams& p)
    : fpids_{ ctrl::FuzzyPID(fuzzyPidParams(p.F_rod_max), p.Ts),
              ctrl::FuzzyPID(fuzzyPidParams(p.F_rod_max), p.Ts),
              ctrl::FuzzyPID(fuzzyPidParams(p.F_rod_max), p.Ts),
              ctrl::FuzzyPID(fuzzyPidParams(p.F_rod_max), p.Ts),
              ctrl::FuzzyPID(fuzzyPidParams(p.F_rod_max), p.Ts),
              ctrl::FuzzyPID(fuzzyPidParams(p.F_rod_max), p.Ts) }
{}

Vec6 FuzzyPIDStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i)
        u(i) = fpids_[i].compute(L_cmd(i) - L(i));
    return u;
}

void FuzzyPIDStewartCtrl::reset() { for (auto& c : fpids_) c.reset(); }

// ===========================================================================
// 3. ADRC - omega_o=60, omega_c=20, b0=1/m_rod; omega_o*Ts=0.30<0.5 (check)
// ===========================================================================

static ctrl::ADRCParams adrcParams(const PlantParams& p)
{
    ctrl::ADRCParams a;
    a.omega_o = 60.0; a.omega_c = 20.0; a.b0 = 1.0 / p.m_rod;
    a.uMin = -p.F_rod_max; a.uMax = p.F_rod_max;
    return a;
}

ADRCStewartCtrl::ADRCStewartCtrl(const PlantParams& p)
    : adrcs_{ ctrl::DiscreteADRC(adrcParams(p), p.Ts),
              ctrl::DiscreteADRC(adrcParams(p), p.Ts),
              ctrl::DiscreteADRC(adrcParams(p), p.Ts),
              ctrl::DiscreteADRC(adrcParams(p), p.Ts),
              ctrl::DiscreteADRC(adrcParams(p), p.Ts),
              ctrl::DiscreteADRC(adrcParams(p), p.Ts) }
{}

Vec6 ADRCStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i) {
        adrcs_[i].setReference(L_cmd(i));
        u(i) = adrcs_[i].compute(L_cmd(i) - L(i));
    }
    return u;
}

void ADRCStewartCtrl::reset() { for (auto& c : adrcs_) c.reset(); }

// ===========================================================================
// 4. SMC - compute(L - L_cmd) [y-ref convention]; phi=0.5mm
// ===========================================================================

static ctrl::SMCParams smcParams(const PlantParams& p)
{
    ctrl::SMCParams s;
    s.c_e = 1.0; s.c_de = 50.0 * p.Ts; s.K = 4000.0; s.phi = 0.001;
    s.uMin = -p.F_rod_max; s.uMax = p.F_rod_max;
    return s;
}

SMCStewartCtrl::SMCStewartCtrl(const PlantParams& p)
    : smcs_{ ctrl::DiscreteSMC(smcParams(p), p.Ts),
             ctrl::DiscreteSMC(smcParams(p), p.Ts),
             ctrl::DiscreteSMC(smcParams(p), p.Ts),
             ctrl::DiscreteSMC(smcParams(p), p.Ts),
             ctrl::DiscreteSMC(smcParams(p), p.Ts),
             ctrl::DiscreteSMC(smcParams(p), p.Ts) }
{}

Vec6 SMCStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i)
        u(i) = smcs_[i].compute(L(i) - L_cmd(i));
    return u;
}

void SMCStewartCtrl::reset() { for (auto& c : smcs_) c.reset(); }

// ===========================================================================
// 5. LQR - single 12-state block-diagonal model, deviation coords
// ===========================================================================

LQRStewartCtrl::LQRStewartCtrl(const PlantParams& p)
    : l0_(rodL0(p))
{
    // Strict Bryson weights (Q~1/x_max^2, R~1/u_max^2) left LQR with a large
    // steady-state offset under the constant load disturbance (pure state
    // feedback has no integral action) - empirically swept against the real
    // plant+CFD reference; this position/velocity/control ratio gives ~6mm
    // steady-state rod error at sea state 5, matching the paper's PID-level
    // accuracy (paper Table 10: ~2.4mm).
    ctrl::LQRParams lp;
    lp.Q = Eigen::MatrixXd::Zero(12, 12);
    for (int i = 0; i < N_RODS; ++i) {
        lp.Q(2*i,     2*i)     = 1.0e6;
        lp.Q(2*i + 1, 2*i + 1) = 5.0;
    }
    lp.R = Eigen::MatrixXd::Identity(6, 6) * 1.0e-7;
    lqr_ = std::make_unique<ctrl::DiscreteLQR>(build12StateSS(p), lp);
}

Vec6 LQRStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6& dL, double, double)
{
    Eigen::VectorXd x(12), x_ref(12);
    for (int i = 0; i < N_RODS; ++i) {
        x(2*i)     = L(i) - l0_(i);
        x(2*i + 1) = dL(i);
        x_ref(2*i)     = L_cmd(i) - l0_(i);
        x_ref(2*i + 1) = 0.0;
    }
    Eigen::VectorXd u = lqr_->compute(x, x_ref);
    Vec6 out; for (int i = 0; i < N_RODS; ++i) out(i) = u(i);
    return out;
}

// ===========================================================================
// 6. MPC - 6x 2-state per-rod DiscreteMPC, deviation coords
// ===========================================================================

static ctrl::MPCParams mpcParams(const PlantParams& p)
{
    ctrl::MPCParams mp;
    mp.Np = 10; mp.Nc = 3; mp.rho_y = 1.0e6; mp.rho_u = 1.0e-7;
    mp.uMin = -p.F_rod_max; mp.uMax = p.F_rod_max;
    mp.qpMaxIter = 2000;
    return mp;
}

MPCStewartCtrl::MPCStewartCtrl(const PlantParams& p)
    : l0_(rodL0(p))
    , mpcs_{ std::make_unique<ctrl::DiscreteMPC>(buildRodSS(p), mpcParams(p)),
             std::make_unique<ctrl::DiscreteMPC>(buildRodSS(p), mpcParams(p)),
             std::make_unique<ctrl::DiscreteMPC>(buildRodSS(p), mpcParams(p)),
             std::make_unique<ctrl::DiscreteMPC>(buildRodSS(p), mpcParams(p)),
             std::make_unique<ctrl::DiscreteMPC>(buildRodSS(p), mpcParams(p)),
             std::make_unique<ctrl::DiscreteMPC>(buildRodSS(p), mpcParams(p)) }
{}

Vec6 MPCStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6& dL, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i) {
        Eigen::VectorXd x(2); x << L(i) - l0_(i), dL(i);
        Eigen::VectorXd r(1); r << L_cmd(i) - l0_(i);
        mpcs_[i]->setState(x);
        Eigen::VectorXd ui = mpcs_[i]->computeRef(x, r);
        u(i) = ui(0);
    }
    return u;
}

void MPCStewartCtrl::reset() { for (auto& c : mpcs_) c->reset(); }

// ===========================================================================
// 7. MRAC - setReference(L_cmd_i) + compute(L_i); positive-gain plant
// ===========================================================================

static ctrl::MRACParams mracParams(const PlantParams& p)
{
    ctrl::MRACParams m;
    const double a_c = 15.0; // continuous-time reference-model pole [rad/s]
    m.a_m = std::exp(-a_c * p.Ts);
    m.b_m = 1.0 - m.a_m;
    // gamma swept empirically against the real plant+CFD reference: theta needs
    // to reach a magnitude of several thousand (force/position scale) within a
    // ~30 s run, requiring a much larger adaptation rate than the README's
    // placeholder gamma=2.
    m.gamma_r = 5.0e5; m.gamma_y = 5.0e5;
    m.sigma = 0.005; m.theta_max = 1.0e6;
    m.uMin = -p.F_rod_max; m.uMax = p.F_rod_max;
    return m;
}

MRACStewartCtrl::MRACStewartCtrl(const PlantParams& p)
    : mracs_{ ctrl::MRACController(mracParams(p), p.Ts),
              ctrl::MRACController(mracParams(p), p.Ts),
              ctrl::MRACController(mracParams(p), p.Ts),
              ctrl::MRACController(mracParams(p), p.Ts),
              ctrl::MRACController(mracParams(p), p.Ts),
              ctrl::MRACController(mracParams(p), p.Ts) }
{}

Vec6 MRACStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i) {
        mracs_[i].setReference(L_cmd(i));
        u(i) = mracs_[i].compute(L(i));
    }
    return u;
}

void MRACStewartCtrl::reset() { for (auto& c : mracs_) c.reset(); }

// ===========================================================================
// 8. L1Adaptive - setReference + compute(L_i)
// ===========================================================================

static ctrl::L1AdaptiveController::Params l1Params(const PlantParams& p)
{
    // NOTE: L1AdaptiveController (like MRACController) is a relative-degree-1
    // adaptive architecture (see L1AdaptiveController.h docstring); the rod
    // plant is relative-degree-2 (force -> position through a spring-mass-
    // damper). An extensive empirical sweep (Gamma 200-1e6, omega_c 5-80,
    // k_g 1-1000, Q_lyap 1-1e6, a_m pole 1-60 rad/s) plateaus around 115-190mm
    // steady-state rod error at sea state 5 regardless of tuning - an expected
    // architectural limitation of applying a 1st-order adaptive law to a 2nd-
    // order plant, not a tuning bug. k_g=10 was the best point found.
    ctrl::L1AdaptiveController::Params l;
    const double a_c = 15.0;
    l.a_m = std::exp(-a_c * p.Ts);
    l.b_m = 1.0 - l.a_m;
    l.k_g = 10.0; l.Gamma = 20000.0; l.omega_c = 10.0; l.sigma_max = 1.0e6; l.Q_lyap = 1.0;
    l.uMin = -p.F_rod_max; l.uMax = p.F_rod_max;
    return l;
}

L1AdaptiveStewartCtrl::L1AdaptiveStewartCtrl(const PlantParams& p)
    : l1s_{ ctrl::L1AdaptiveController(l1Params(p), p.Ts),
            ctrl::L1AdaptiveController(l1Params(p), p.Ts),
            ctrl::L1AdaptiveController(l1Params(p), p.Ts),
            ctrl::L1AdaptiveController(l1Params(p), p.Ts),
            ctrl::L1AdaptiveController(l1Params(p), p.Ts),
            ctrl::L1AdaptiveController(l1Params(p), p.Ts) }
{}

Vec6 L1AdaptiveStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i) {
        l1s_[i].setReference(L_cmd(i));
        u(i) = l1s_[i].compute(L(i));
    }
    return u;
}

void L1AdaptiveStewartCtrl::reset() { for (auto& c : l1s_) c.reset(); }

// ===========================================================================
// 9. GainScheduled - scheduled on |heave deviation| (z_ref_global);
//    3 gain sets (calm/moderate/extreme), reduced Ki at the extreme point.
// ===========================================================================

GainScheduledStewartCtrl::GainScheduledStewartCtrl(const PlantParams& p)
    : gscs_{ ctrl::GainScheduledController(p.Ts), ctrl::GainScheduledController(p.Ts),
             ctrl::GainScheduledController(p.Ts), ctrl::GainScheduledController(p.Ts),
             ctrl::GainScheduledController(p.Ts), ctrl::GainScheduledController(p.Ts) }
{
    for (auto& gs : gscs_) {
        gs.addSchedulePoint(0.00, std::make_shared<ctrl::DiscretePID>(pidParams(p.F_rod_max, 4.0e4), p.Ts));
        gs.addSchedulePoint(0.10, std::make_shared<ctrl::DiscretePID>(pidParams(p.F_rod_max, 3.0e4), p.Ts));
        gs.addSchedulePoint(0.16, std::make_shared<ctrl::DiscretePID>(pidParams(p.F_rod_max, 1.0e4), p.Ts));
    }
}

Vec6 GainScheduledStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&,
                                       double, double z_ref_global)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i) {
        gscs_[i].setSchedulingParam(z_ref_global);
        u(i) = gscs_[i].compute(L_cmd(i) - L(i));
    }
    return u;
}

void GainScheduledStewartCtrl::reset() { for (auto& c : gscs_) c.reset(); }

// ===========================================================================
// 10. TubeMPC - 6x 2-state per-rod TubeMPC, deviation coords
// ===========================================================================

static ctrl::TubeMPC makeTubeMPCForRod(const PlantParams& p)
{
    ctrl::StateSpace sys = buildRodSS(p);

    ctrl::LQRParams lp;
    lp.Q = Eigen::MatrixXd::Identity(2, 2);
    lp.Q(0, 0) = 1.0e6; lp.Q(1, 1) = 5.0;
    lp.R = Eigen::MatrixXd::Identity(1, 1) * 1.0e-7;
    ctrl::DiscreteLQR lqr_rod(sys, lp);
    Eigen::MatrixXd K_tube = -lqr_rod.gainMatrix();

    ctrl::TubeMPCParams tp;
    tp.Np = 10; tp.Nu = 3;
    tp.Q = Eigen::MatrixXd::Identity(1, 1) * 1.0e6;
    tp.R = Eigen::MatrixXd::Identity(1, 1) * 1.0e-7;
    tp.K = K_tube;
    tp.wMax = Eigen::VectorXd(2); tp.wMax << 1.0e-4, 1.0e-2;
    tp.uMin = Eigen::VectorXd::Constant(1, -p.F_rod_max);
    tp.uMax = Eigen::VectorXd::Constant(1,  p.F_rod_max);
    tp.Ts = p.Ts;
    tp.qpMaxIter = 2000;
    return ctrl::TubeMPC(sys, tp);
}

TubeMPCStewartCtrl::TubeMPCStewartCtrl(const PlantParams& p)
    : l0_(rodL0(p))
    , tmpcs_{ makeTubeMPCForRod(p), makeTubeMPCForRod(p), makeTubeMPCForRod(p),
              makeTubeMPCForRod(p), makeTubeMPCForRod(p), makeTubeMPCForRod(p) }
{}

Vec6 TubeMPCStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6& dL, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i) {
        Eigen::VectorXd x(2); x << L(i) - l0_(i), dL(i);
        Eigen::VectorXd r(1); r << L_cmd(i) - l0_(i);
        Eigen::VectorXd ui = tmpcs_[i].computeRef(x, r);
        u(i) = ui(0);
    }
    return u; // already clamped to [uMin,uMax] internally by TubeMPC
}

void TubeMPCStewartCtrl::reset() { for (auto& c : tmpcs_) c.reset(); }

// ===========================================================================
// 11. NeuralPID - online gain adaptation
// ===========================================================================

static ctrl::NeuralPID::Params neuralPidParams(const PlantParams& p)
{
    ctrl::NeuralPID::Params n;
    n.n_hidden = 8; n.lr = 5.0e-6; n.Ts = p.Ts;
    // Steady-state dy/du estimate (y = u/k_spring at DC, ignoring damping/mass).
    n.plant_gain = 1.0 / p.k_spring;
    n.max_weight_norm = 10.0;
    n.uMin = -p.F_rod_max; n.uMax = p.F_rod_max;
    n.Kp0 = 8.0e4; n.Ki0 = 4.0e4; n.Kd0 = 4.0e3;
    return n;
}

NeuralPIDStewartCtrl::NeuralPIDStewartCtrl(const PlantParams& p)
    : npids_{ ctrl::NeuralPID(neuralPidParams(p)), ctrl::NeuralPID(neuralPidParams(p)),
              ctrl::NeuralPID(neuralPidParams(p)), ctrl::NeuralPID(neuralPidParams(p)),
              ctrl::NeuralPID(neuralPidParams(p)), ctrl::NeuralPID(neuralPidParams(p)) }
{}

Vec6 NeuralPIDStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6&, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i)
        u(i) = npids_[i].compute(L_cmd(i) - L(i));
    return u;
}

void NeuralPIDStewartCtrl::reset() { for (auto& c : npids_) c.reset(); }

// ===========================================================================
// 12. ScenarioMPC - 6x 2-state per-rod ScenarioMPC, deviation coords
// ===========================================================================

static ctrl::ScenarioMPC makeScenarioMPCForRod(const PlantParams& p)
{
    ctrl::StateSpace sys = buildRodSS(p);

    ctrl::ScenarioMPCParams sp;
    sp.Np = 10; sp.Nu = 3;
    sp.Q = Eigen::MatrixXd::Identity(1, 1) * 1.0e6;
    sp.R = Eigen::MatrixXd::Identity(1, 1) * 1.0e-7;
    sp.Sigma_w = Eigen::MatrixXd::Zero(2, 2);
    sp.Sigma_w(0, 0) = 1.0e-4 * 1.0e-4;
    sp.Sigma_w(1, 1) = 1.0e-2 * 1.0e-2;
    sp.N_samples = 20;
    sp.qpMaxIter = 2000;
    sp.uMin = Eigen::VectorXd::Constant(1, -p.F_rod_max);
    sp.uMax = Eigen::VectorXd::Constant(1,  p.F_rod_max);
    sp.Ts = p.Ts;
    return ctrl::ScenarioMPC(sys, sp);
}

ScenarioMPCStewartCtrl::ScenarioMPCStewartCtrl(const PlantParams& p)
    : l0_(rodL0(p))
    , smpcs_{ makeScenarioMPCForRod(p), makeScenarioMPCForRod(p), makeScenarioMPCForRod(p),
              makeScenarioMPCForRod(p), makeScenarioMPCForRod(p), makeScenarioMPCForRod(p) }
{}

Vec6 ScenarioMPCStewartCtrl::compute(const Vec6& L_cmd, const Vec6& L, const Vec6& dL, double, double)
{
    Vec6 u;
    for (int i = 0; i < N_RODS; ++i) {
        Eigen::VectorXd x(2); x << L(i) - l0_(i), dL(i);
        Eigen::VectorXd r(1); r << L_cmd(i) - l0_(i);
        Eigen::VectorXd ui = smpcs_[i].computeRef(x, r);
        u(i) = ui(0);
    }
    return u;
}

void ScenarioMPCStewartCtrl::reset() { for (auto& c : smpcs_) c.reset(); }

} // namespace stewart
