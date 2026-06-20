#include "controllers.h"
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace Eigen;

namespace bouyancydrivenairshipinverticalplan {

namespace {

// Numerically linearize the fixed-center 4-state design model (Eq. 24) at trim
// (theta_ref, q=0, rp1_ref, w=0), u=u_ss, via central finite differences on the plant's own
// exact closed-form ODE (mirrors AirshipFBLCtrl's "exact dynamics, numerical differentiation"
// approach - avoids hand-deriving the Jacobian). Shared by LQR and MPC.
ctrl::StateSpace buildDesignModel(const PlantParams& p, double theta_ref, double rp1_ref,
                                   double& u_ss_out) {
    u_ss_out = trimInput(p, theta_ref, rp1_ref);
    Eigen::Vector4d z0;
    z0 << theta_ref, 0.0, rp1_ref, 0.0;
    const double h = 1e-4;

    Eigen::MatrixXd Ac(4, 4);
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4d zp = z0, zm = z0;
        zp(i) += h;
        zm(i) -= h;
        Ac.col(i) = (fixedCenterOde(p, zp, u_ss_out) - fixedCenterOde(p, zm, u_ss_out)) / (2.0 * h);
    }
    Eigen::MatrixXd Bc(4, 1);
    Bc.col(0) = (fixedCenterOde(p, z0, u_ss_out + h) - fixedCenterOde(p, z0, u_ss_out - h)) / (2.0 * h);

    Eigen::MatrixXd Cc(1, 4);
    Cc << 1.0, 0.0, 0.0, 0.0;
    Eigen::MatrixXd Dc(1, 1);
    Dc << 0.0;

    ctrl::StateSpace ssc(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(ssc, p.Ts, ctrl::C2dMethod::ZOH);
}

}  // namespace

// ===========================================================================
// 2. PID
// Plant gain d(theta_ddot)/du = -rp3/(J+m_bar*rp1^2) < 0 (negative-gain plant, see README
// "Sign convention"). With the documented r-y convention, a STABLE closed loop requires
// NEGATIVE Kp/Ki/Kd - verified by direct derivation (positive gains give positive feedback
// for this plant). omega_n~=0.25 rad/s, zeta~=0.9 (slightly overdamped to avoid overshoot).
//
// The large theta_ref steps in every scenario (10-40 deg) create a one-step derivative
// "kick" large enough to saturate u; the resulting anti-windup back-calculation then slams
// the integrator the wrong way for ~1-2 s before it can recover (confirmed empirically - the
// first build of this controller ran away to the actuator rail after every step).
// computeDoM() (derivative-on-measurement) looked like the fix, but it has the same problem
// at k=0: DiscretePID::reset() zeroes y_prev_ to 0.0, not the plant's actual initial
// measurement, so the very first computeDoM() call sees a fake (0 - theta0) "jump" and kicks
// just as hard (confirmed empirically). The robust fix needs no library change: on every
// theta_ref change (including the implicit "change" from NaN at the first call),
// pid_.bumplessInit(pid_.lastOutput(), new_error) explicitly re-seeds e_prev_/deriv_/
// integral_ so the very next compute(error) starts smoothly from the controller's last
// output - the same mechanism GainScheduledController already uses internally for bracket
// switches (see GainScheduledController.h), just invoked manually here since this is a
// single fixed controller, not a schedule.
//
// Plus an explicit trim feedforward u_ss = trimInput(theta_ref, rp1_ref): holding any pitch
// angle requires a roughly constant ~100-150 N actuator force to counteract the slider's own
// gravity moment (see README "Governing Equations"). ADRC's ESO and AirshipFBLCtrl's alpha
// term estimate/cancel this automatically; LQR/MPC already feed it forward explicitly; a
// plain PID relying on slow integral action alone to find this offset converges too slowly
// against it (confirmed empirically) - so it is added explicitly here too.
// ===========================================================================
PIDAirshipCtrl::PIDAirshipCtrl(const PlantParams& p)
    : p_(p)
    , pid_(ctrl::PIDParams{
        .Kp   = -251.0,
        .Ki   = -6.0,
        .Kd   = -1808.0,
        .N    = 10.0,
        .uMin = p.u_min,
        .uMax = p.u_max,
        .Kb   = 1.0
      }, p.Ts)
{}

double PIDAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);
    if (theta_ref != last_theta_ref_) {
        pid_.bumplessInit(pid_.lastOutput(), theta_ref - x(THETA));
        last_theta_ref_ = theta_ref;
    }
    return clampU(p_, u_ss + pid_.compute(theta_ref - x(THETA)));
}

void PIDAirshipCtrl::reset() {
    pid_.reset();
    last_theta_ref_ = std::numeric_limits<double>::quiet_NaN();
}

// ===========================================================================
// 3. ADRC
// compute(theta_ref - theta); b0 negative (sign convention above). omega_o=3.0, omega_c=0.3,
// omega_o*Ts = 3.0*0.05 = 0.15 < 0.5 (check). b0 evaluated at the representative rp1=-1.15 m
// used by s01/s02/s05 - ADRC tolerates b0 uncertainty within ~3x (Gao 2003), and rp1_ref
// across all 5 scenarios stays within [-1.15, -1.0] m, well inside that margin.
// ===========================================================================
ADRCAirshipCtrl::ADRCAirshipCtrl(const PlantParams& p)
    : p_(p)
    , adrc_(ctrl::ADRCParams{
        .omega_o = 3.0,
        .omega_c = 0.3,
        .b0      = -p.rp3 / (p.J + p.m_bar * 1.15 * 1.15),
        .uMin    = p.u_min,
        .uMax    = p.u_max
      }, p.Ts)
{}

double ADRCAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);
    adrc_.setReference(theta_ref);
    return clampU(p_, u_ss + adrc_.compute(theta_ref - x(THETA)));
}

void ADRCAirshipCtrl::reset() { adrc_.reset(); }

// ===========================================================================
// 4. SMC
// compute(theta - theta_ref): this repo's documented SMC convention (y - ref). For this
// negative-gain plant, a stable loop requires NEGATIVE K (verified by direct derivation -
// positive K with this convention is positive feedback). Plus the trim feedforward (see
// PID #2's comment).
//
// c_de=15 (much larger than a typical SMCParams value): inside the boundary layer, SMC's
// law reduces to u=-(K*c_e/phi)*e-(K*c_de/phi)*(e-e_prev) - a PD term with NO 1/Ts scaling
// on the derivative-like part, unlike DiscretePID's filtered derivative (which has an
// effective gain of Kd*N*alpha/(1-alpha) approx= Kd*20 for this Ts/N - see PID #2's gains).
// Matching ~PID #2's damping authority on this large-inertia plant therefore needs a c_de
// roughly 20x larger than the "convert from continuous lambda via c_de=lambda*Ts" rule of
// thumb in SMCParams' own docstring would suggest. Confirmed empirically: c_de=0.05-0.1
// (the docstring's typical range) left SMC critically under-damped - it overshot straight
// through theta_ref into the far negative range (where the slider's restoring moment is
// also working against an already-large ballistic v1 it had built up) and got stuck unable
// to recover, since the trim feedforward (evaluated at the *target* operating point) becomes
// actively wrong that far from it.
// ===========================================================================
SMCAirshipCtrl::SMCAirshipCtrl(const PlantParams& p)
    : p_(p)
    , smc_(ctrl::SMCParams{
        .c_e  = 1.0,
        .c_de = 15.0,
        .K    = -220.0,
        .phi  = 0.3,
        .uMin = p.u_min,
        .uMax = p.u_max
      }, p.Ts)
{}

double SMCAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);
    return clampU(p_, u_ss + smc_.compute(x(THETA) - theta_ref));
}

void SMCAirshipCtrl::reset() { smc_.reset(); }

// ===========================================================================
// 5. LQR
// 4-state [theta,q,rp1,w] design model (Eq. 24), numerically linearized + re-trimmed
// whenever (theta_ref, rp1_ref) changes (lazy rebuild - cheap, fires at most a handful of
// times per run). Bryson weights: theta_max=0.3 rad, q_max=0.3 rad/s, rp1_dev_max=0.5 m,
// w_max=0.3 m/s, R=1/u_max^2.
// ===========================================================================
LQRAirshipCtrl::LQRAirshipCtrl(const PlantParams& p)
    : p_(p)
    , theta_ref_design_(std::numeric_limits<double>::quiet_NaN())
    , rp1_ref_design_(std::numeric_limits<double>::quiet_NaN())
{}

void LQRAirshipCtrl::rebuildIfNeeded(double theta_ref, double rp1_ref) {
    if (lqr_ && theta_ref == theta_ref_design_ && rp1_ref == rp1_ref_design_) return;

    ctrl::StateSpace ssd = buildDesignModel(p_, theta_ref, rp1_ref, u_ss_);

    const double theta_max = 0.3, q_max = 0.3, rp1_dev_max = 0.5, w_max = 0.3;
    ctrl::LQRParams lp;
    lp.Q = Eigen::MatrixXd::Zero(4, 4);
    lp.Q(0, 0) = 1.0 / (theta_max * theta_max);
    lp.Q(1, 1) = 1.0 / (q_max * q_max);
    lp.Q(2, 2) = 1.0 / (rp1_dev_max * rp1_dev_max);
    lp.Q(3, 3) = 1.0 / (w_max * w_max);
    lp.R = Eigen::MatrixXd::Constant(1, 1, 1.0 / (p_.u_max * p_.u_max));

    lqr_ = std::make_shared<ctrl::DiscreteLQR>(ssd, lp);
    theta_ref_design_ = theta_ref;
    rp1_ref_design_   = rp1_ref;
}

double LQRAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    rebuildIfNeeded(theta_ref, rp1_ref);
    Eigen::VectorXd x4(4);
    x4 << x(THETA), x(Q), x(RP1), x(W);
    Eigen::VectorXd xref4(4);
    xref4 << theta_ref, 0.0, rp1_ref, 0.0;
    Eigen::VectorXd uff(1);
    uff << u_ss_;
    Eigen::VectorXd u = lqr_->compute(x4, xref4, uff);
    return clampU(p_, u(0));
}

void LQRAirshipCtrl::reset() {
    theta_ref_design_ = std::numeric_limits<double>::quiet_NaN();
    rp1_ref_design_   = std::numeric_limits<double>::quiet_NaN();
    lqr_.reset();
}

// ===========================================================================
// 6. MPC
// Same 4-state linearized design model as LQR, deviation-form. Np=20 (1 s), Nc=5,
// rho_y/rho_u set to the *same* Bryson weights as LQR #5 (1/theta_max^2, 1/u_max^2) rather
// than hand-picked MPC weights.
//
// Several longer-horizon / hand-tuned weight combinations (Np=150 down to Np=40, rho_y/rho_u
// ratios from 10:1 up to 1:2) were tried first and all diverged the same way: theta sails
// straight through theta_ref and keeps accelerating past it (ending 25-70+ deg beyond
// target, only the timing of the overshoot moved). Root cause: the 4-state design model
// excludes v1/v3 entirely (paper's "fixed-center" idealisation), so the QP's own prediction
// has no way to "see" the growing ballistic coupling that eventually arrests/reverses theta
// in the real plant - confirmed empirically by v3 growing throughout every divergent run.
// LQR uses the *same* mismatched design model yet stays robust because it commits to nothing
// beyond a single proportional reaction to the *current* real state every step; matching its
// exact weight ratio (rather than treating Np/rho_y/rho_u as free MPC-specific knobs) turned
// out to reproduce that same robustness almost exactly (IAE dropped from 25-47 to 5.4).
// ===========================================================================
MPCAirshipCtrl::MPCAirshipCtrl(const PlantParams& p)
    : p_(p)
    , theta_ref_design_(std::numeric_limits<double>::quiet_NaN())
    , rp1_ref_design_(std::numeric_limits<double>::quiet_NaN())
{}

void MPCAirshipCtrl::rebuildIfNeeded(double theta_ref, double rp1_ref) {
    if (mpc_ && theta_ref == theta_ref_design_ && rp1_ref == rp1_ref_design_) return;

    ctrl::StateSpace ssd = buildDesignModel(p_, theta_ref, rp1_ref, u_ss_);

    ctrl::MPCParams mp;
    mp.Np        = 20;
    mp.Nc        = 5;
    mp.rho_y     = 11.11;     // 1/theta_max^2, theta_max=0.3 rad - same Bryson weight as LQR
    mp.rho_u     = 6.25e-6;   // 1/u_max^2, u_max=400 N - same Bryson weight as LQR
    mp.uMin      = -p_.u_max;
    mp.uMax      =  p_.u_max;
    mp.duMin     = -p_.u_max;
    mp.duMax     =  p_.u_max;
    mp.qpMaxIter = 300;

    mpc_ = std::make_unique<ctrl::DiscreteMPC>(ssd, mp);
    theta_ref_design_ = theta_ref;
    rp1_ref_design_   = rp1_ref;
}

double MPCAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    rebuildIfNeeded(theta_ref, rp1_ref);

    Eigen::VectorXd x_dev(4);
    x_dev << x(THETA) - theta_ref, x(Q), x(RP1) - rp1_ref, x(W);
    Eigen::VectorXd r_dev(1);
    r_dev << 0.0;

    Eigen::VectorXd u = mpc_->computeRef(x_dev, r_dev);
    return clampU(p_, u_ss_ + u(0));
}

void MPCAirshipCtrl::reset() {
    theta_ref_design_ = std::numeric_limits<double>::quiet_NaN();
    rp1_ref_design_   = std::numeric_limits<double>::quiet_NaN();
    mpc_.reset();
}

// ===========================================================================
// 7. MRAC
// set_reference(theta_ref), compute(theta). Negative-gain plant -> negative gamma_r/gamma_y
// per MRACController.h's own documented rule. a_m=exp(-Ts/tau), tau~=4 s. Trim feedforward
// (see PID #2's comment) added on top of the adaptive law's own output: MRAC only needs to
// learn the residual once u_ss already cancels the bulk of the steady gravity load.
//
// gamma_r/gamma_y=-2 (much gentler than an initial -20 guess): with the trim feedforward
// already supplying ~140 N, a fast adaptation rate over-corrects the small residual and rings
// - confirmed empirically (gamma=-20 overshot past theta_ref into the far negative range and
// pinned u at the actuator rail for tens of seconds, the same failure pattern as SMC's first
// attempt above). A slower rate trades adaptation speed for damping, which this large-inertia
// plant needs more of given how little is actually left for MRAC to learn on top of the FF.
// ===========================================================================
MRACAirshipCtrl::MRACAirshipCtrl(const PlantParams& p)
    : p_(p)
    , mrac_([&]() {
        const double a_m = std::exp(-p.Ts / 4.0);
        ctrl::MRACParams mp;
        mp.a_m       = a_m;
        mp.b_m       = 1.0 - a_m;
        mp.gamma_r   = -2.0;
        mp.gamma_y   = -2.0;
        mp.sigma     = 0.01;
        mp.theta_max = 3000.0;
        mp.uMin      = p.u_min;
        mp.uMax      = p.u_max;
        return ctrl::MRACController(mp, p.Ts);
      }())
{}

double MRACAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    if (!ym_initialized_) {
        mrac_.setYm(x(THETA));
        ym_initialized_ = true;
    }
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);
    mrac_.setReference(theta_ref);
    return clampU(p_, u_ss + mrac_.compute(x(THETA)));
}

void MRACAirshipCtrl::reset() { mrac_.reset(); ym_initialized_ = false; }

// ===========================================================================
// 8. GainScheduled
// 3-point PID schedule on |theta_ref - theta|: gentle near setpoint, baseline mid-range,
// aggressive for large commanded swings (e.g. s03's ~40 deg step). All gains negative
// (same negative-gain-plant reasoning as PID #2), plus the same trim feedforward.
//
// GainScheduledController's own bumplessInit-on-switch protection (LinearBlend mode) only
// fires for a controller *newly entering* the active (lo,hi) bracket pair - a controller
// that stays part of the pair across a reference jump (e.g. the "mid" bracket, which is
// "hi" just below the jump and "lo" just above it) is NOT re-initialised by that mechanism
// and still takes the full reference-step derivative "kick" described in PID #2's comment.
// Confirmed empirically: zeroing Kd (avoiding the kick entirely) instead made every bracket
// pure PI on a relative-degree-2 plant, which oscillated between the +-rp1 mechanical stops
// without ever settling - PI alone cannot stabilise this plant (no derivative/damping path).
// So Kd is restored here, and instead *all three* brackets are defensively bumplessInit()'d
// on every theta_ref change (not just whichever one GainScheduledController itself would
// protect), using the same pattern as PID #2.
// ===========================================================================
GainScheduledAirshipCtrl::GainScheduledAirshipCtrl(const PlantParams& p)
    : p_(p)
    , gs_(p.Ts, ctrl::GainScheduleMode::LinearBlend)
    , pid_gentle_(std::make_shared<ctrl::DiscretePID>(
          ctrl::PIDParams{.Kp = -150.0, .Ki = -3.0,  .Kd = -1200.0, .N = 10.0,
                          .uMin = p.u_min, .uMax = p.u_max, .Kb = 1.0}, p.Ts))
    , pid_mid_(std::make_shared<ctrl::DiscretePID>(
          ctrl::PIDParams{.Kp = -251.0, .Ki = -6.0,  .Kd = -1808.0, .N = 10.0,
                          .uMin = p.u_min, .uMax = p.u_max, .Kb = 1.0}, p.Ts))
    , pid_aggr_(std::make_shared<ctrl::DiscretePID>(
          ctrl::PIDParams{.Kp = -350.0, .Ki = -8.0,  .Kd = -2200.0, .N = 10.0,
                          .uMin = p.u_min, .uMax = p.u_max, .Kb = 1.0}, p.Ts))
{
    gs_.addSchedulePoint(0.05, pid_gentle_);  // |error| < ~3 deg
    gs_.addSchedulePoint(0.30, pid_mid_);     // ~3-17 deg
    gs_.addSchedulePoint(0.80, pid_aggr_);    // > ~17 deg, up to ~46 deg
}

double GainScheduledAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    const double error = theta_ref - x(THETA);
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);

    if (theta_ref != last_theta_ref_) {
        const double u_target = gs_.lastOutput();
        pid_gentle_->bumplessInit(u_target, error);
        pid_mid_->bumplessInit(u_target, error);
        pid_aggr_->bumplessInit(u_target, error);
        last_theta_ref_ = theta_ref;
    }

    gs_.setSchedulingParam(std::abs(error));
    return clampU(p_, u_ss + gs_.compute(error));
}

void GainScheduledAirshipCtrl::reset() {
    gs_.reset();
    last_theta_ref_ = std::numeric_limits<double>::quiet_NaN();
}

// ===========================================================================
// 9. L1Adaptive
// set_reference(theta_ref), compute(theta). Positive Gamma (matches the working Solar Cooker
// precedent on a negative-gain plant - L1's sigma_hat adaptation law does not need the same
// gamma-negation rule as MRAC's direct algebraic law). a_m/b_m mirror MRAC's reference model.
//
// Gamma=200 (vs. an initial guess of 10, copied directly from Solar Cooker): u[k] =
// C(z)*(k_g*r[k] + eta[k]) with k_g*r contributing only ~O(0.5) (r is in radians) - nearly
// all of the corrective authority here has to come from eta=-sigma_hat, which only exists
// because the adaptation law learned it. Cooker's r was already in the natural unit of its
// output (degrees C, O(10-100)), so Gamma=10 gave it a meaningfully-sized eta quickly; here
// the same Gamma left sigma_hat barely moving over a 60 s run (confirmed empirically - L1's
// output tracked the constant trim feedforward almost exactly, i.e. close to no feedback
// authority at all). Gamma=200 + a faster omega_c=1.5 closes most of that gap.
// ===========================================================================
L1AdaptiveAirshipCtrl::L1AdaptiveAirshipCtrl(const PlantParams& p)
    : p_(p)
    , l1_([&]() {
        const double a_m = std::exp(-p.Ts / 4.0);
        ctrl::L1AdaptiveController::Params lp;
        lp.a_m       = a_m;
        lp.b_m       = 1.0 - a_m;
        lp.k_g       = 1.0;
        lp.Gamma     = 200.0;
        lp.omega_c   = 1.5;
        lp.sigma_max = 500.0;
        lp.Q_lyap    = 1.0;
        lp.uMin      = p.u_min;
        lp.uMax      = p.u_max;
        return ctrl::L1AdaptiveController(lp, p.Ts);
      }())
{}

double L1AdaptiveAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);
    l1_.setReference(theta_ref);
    return clampU(p_, u_ss + l1_.compute(x(THETA)));
}

void L1AdaptiveAirshipCtrl::reset() { l1_.reset(); }

// ===========================================================================
// 10. NeuralPID
// NeuralPID's [Kp,Ki,Kd] are softplus-activated (always > 0), so the negative-gain plant
// must be handled through the ERROR SIGN, not the gain sign - compute(theta - theta_ref)
// (y - ref), mirroring Solar Cooker's NeuralPIDCookerCtrl (the only working precedent in this
// repo for NeuralPID on a negative-gain plant). plant_gain negative (gradient-path scale,
// matches ADRC's b0 magnitude); Kp0/Ki0 seeded to match PID #2's |Kp|/|Ki|.
//
// Kd0=0: like GainScheduled above, NeuralPID's compute(error) has no derivative-on-
// measurement option and a nonzero Kd0 reproduces the same reference-step "kick" - confirmed
// empirically. The trim feedforward (see PID #2's comment) is added on top.
// ===========================================================================
NeuralPIDAirshipCtrl::NeuralPIDAirshipCtrl(const PlantParams& p)
    : p_(p)
    , npid_(ctrl::NeuralPID::Params{
        .n_hidden        = 6,
        .lr              = 1e-7,
        .Ts              = p.Ts,
        .plant_gain      = -p.rp3 / (p.J + p.m_bar * 1.15 * 1.15),
        .max_weight_norm = 50.0,
        .uMin            = p.u_min,
        .uMax            = p.u_max,
        .Kp0             = 251.0,
        .Ki0             = 6.0,
        .Kd0             = 0.0
      })
{}

double NeuralPIDAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);
    return clampU(p_, u_ss + npid_.compute(x(THETA) - theta_ref));
}

void NeuralPIDAirshipCtrl::reset() { npid_.reset(); }

// ===========================================================================
// 11. ILC
// Two-phase: PID feedback while learning (phase 1, k < N_TRIAL), PID + learned feedforward
// afterwards (phase 2). N_TRIAL=600 (30 s). PID gains negative (same convention as #2,
// slightly gentler since the ILC feedforward + trim feedforward share the load). Inner PID
// is defensively bumplessInit()'d on every theta_ref change, same pattern and same reasons
// as PID #2; the trim feedforward is added on top.
// ===========================================================================
namespace { constexpr int ILC_N_TRIAL = 600; }

ILCAirshipCtrl::ILCAirshipCtrl(const PlantParams& p)
    : p_(p)
    , pid_(ctrl::PIDParams{
        .Kp   = -180.0,
        .Ki   = -4.0,
        .Kd   = -1300.0,
        .N    = 10.0,
        .uMin = p.u_min,
        .uMax = p.u_max,
        .Kb   = 1.0
      }, p.Ts)
    , ilc_([&]() {
        ctrl::ILC::Params ip;
        ip.N        = ILC_N_TRIAL;
        ip.mode     = ctrl::ILC::Mode::PType;
        ip.Lp       = 0.3;
        ip.Ts       = p.Ts;
        ip.Q_filter = 0.95;
        ip.uMin     = p.u_min;
        ip.uMax     = p.u_max;
        return ctrl::ILC(ip);
      }())
{}

double ILCAirshipCtrl::compute(const State& x, double theta_ref, double rp1_ref, double) {
    const double e    = theta_ref - x(THETA);
    const double u_ss = trimInput(p_, theta_ref, rp1_ref);

    if (theta_ref != last_theta_ref_) {
        pid_.bumplessInit(pid_.lastOutput(), e);
        last_theta_ref_ = theta_ref;
    }
    const double u_pid = pid_.compute(e);

    if (!phase2_) {
        if (k_ < ILC_N_TRIAL) ilc_.recordError(k_, e);
        if (++k_ == ILC_N_TRIAL) {
            ilc_.updateFeedforward();
            phase2_ = true;
            k_      = 0;
        }
        return clampU(p_, u_ss + u_pid);
    }
    const int kc = std::min(k_++, ILC_N_TRIAL - 1);
    return clampU(p_, u_ss + u_pid + ilc_.feedforward(kc));
}

void ILCAirshipCtrl::reset() {
    ilc_.reset();
    pid_.reset();
    k_ = 0;
    phase2_ = false;
    last_theta_ref_ = std::numeric_limits<double>::quiet_NaN();
}

// ===========================================================================
// 12. AirshipFBLCtrl
// Paper's headline feedback-linearization law (Sec. 4.3.2, Theorem 2). y = phi1~ + k*phi2~;
// alpha/beta (the 3rd-order drift/control-gain terms) are obtained numerically via a short
// constant-input RK4 rollout + forward finite-difference stencils on the plant's own exact
// closed-form dynamics, NOT the paper's unpublished Appendix-A symbolic expressions
// (see README "Implementation Notes"). Self-correcting by construction: the control law's
// -(1/beta) factor automatically accounts for whatever sign beta numerically comes out to be,
// so (unlike PID/SMC/ADRC/etc above) no manual sign convention analysis is needed here.
// ===========================================================================
AirshipFBLCtrl::AirshipFBLCtrl(const PlantParams& p) : p_(p) {}

double AirshipFBLCtrl::yOutput(const State& x) const {
    const double theta = x(THETA), q = x(Q), rp1 = x(RP1), w = x(W);
    const double mu = p_.m_bar * p_.ms / (p_.m_bar + p_.ms);
    const double phi1 = p_.J * q + (rp1 * rp1 * q + p_.rp3 * p_.rp3 * q + p_.rp3 * w) * mu;
    const double L = std::sqrt(p_.J / mu + p_.rp3 * p_.rp3);
    const double phi2 = theta + (p_.rp3 / L) * std::atan(rp1 / L);
    return phi1 + k_gain_ * phi2;
}

void AirshipFBLCtrl::normalForm(const State& x0, double m0,
                                 double& xi1, double& xi2, double& xi3,
                                 double& alpha, double& beta) const {
    constexpr double H     = 1e-3;  // rollout sub-step [s]
    constexpr double EPS_U = 1.0;   // probe force for beta [N]

    auto rollout = [&](double u_const) {
        std::array<double, 4> y{};
        State x = x0;
        y[0] = yOutput(x);
        for (int i = 1; i <= 3; ++i) {
            const State k1 = ode(p_, x, u_const, m0);
            const State k2 = ode(p_, x + 0.5 * H * k1, u_const, m0);
            const State k3 = ode(p_, x + 0.5 * H * k2, u_const, m0);
            const State k4 = ode(p_, x + H * k3, u_const, m0);
            x += (H / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            y[i] = yOutput(x);
        }
        return y;
    };

    const auto y0 = rollout(0.0);
    xi1   = y0[0];
    xi2   = (-11.0 * y0[0] + 18.0 * y0[1] - 9.0 * y0[2] + 2.0 * y0[3]) / (6.0 * H);
    xi3   = (2.0 * y0[0] - 5.0 * y0[1] + 4.0 * y0[2] - y0[3]) / (H * H);
    alpha = (-y0[0] + 3.0 * y0[1] - 3.0 * y0[2] + y0[3]) / (H * H * H);

    const auto ye = rollout(EPS_U);
    const double y3_eps = (-ye[0] + 3.0 * ye[1] - 3.0 * ye[2] + ye[3]) / (H * H * H);
    beta = (y3_eps - alpha) / EPS_U;
}

double AirshipFBLCtrl::compute(const State& x, double theta_ref, double rp1_ref, double m0) {
    double xi1, xi2, xi3, alpha, beta;
    normalForm(x, m0, xi1, xi2, xi3, alpha, beta);

    // Equilibrium composite output: phi1~ = 0 at q=w=0 for any rp1 (every term in phi1~ has a
    // factor of q or w); phi2~ evaluated at the (theta_ref, rp1_ref) trim.
    const double mu = p_.m_bar * p_.ms / (p_.m_bar + p_.ms);
    const double L  = std::sqrt(p_.J / mu + p_.rp3 * p_.rp3);
    const double phi2_e = theta_ref + (p_.rp3 / L) * std::atan(rp1_ref / L);
    const double y_e = k_gain_ * phi2_e;

    if (std::abs(beta) < 1e-12) return 0.0;  // degenerate probe (should not occur in practice)
    const double u = -(1.0 / beta) * (lambda2_ * xi3 + lambda1_ * xi2 + lambda0_ * (xi1 - y_e) + alpha);
    return clampU(p_, u);
}

void AirshipFBLCtrl::reset() {}

// ===========================================================================
// Factory
// ===========================================================================
std::vector<std::unique_ptr<ControllerBase>> makeControllers(const PlantParams& p) {
    std::vector<std::unique_ptr<ControllerBase>> v;
    v.push_back(std::make_unique<OpenLoopCtrl>(p));
    v.push_back(std::make_unique<PIDAirshipCtrl>(p));
    v.push_back(std::make_unique<ADRCAirshipCtrl>(p));
    v.push_back(std::make_unique<SMCAirshipCtrl>(p));
    v.push_back(std::make_unique<LQRAirshipCtrl>(p));
    v.push_back(std::make_unique<MPCAirshipCtrl>(p));
    v.push_back(std::make_unique<MRACAirshipCtrl>(p));
    v.push_back(std::make_unique<GainScheduledAirshipCtrl>(p));
    v.push_back(std::make_unique<L1AdaptiveAirshipCtrl>(p));
    v.push_back(std::make_unique<NeuralPIDAirshipCtrl>(p));
    v.push_back(std::make_unique<ILCAirshipCtrl>(p));
    v.push_back(std::make_unique<AirshipFBLCtrl>(p));
    return v;
}

}  // namespace bouyancydrivenairshipinverticalplan
