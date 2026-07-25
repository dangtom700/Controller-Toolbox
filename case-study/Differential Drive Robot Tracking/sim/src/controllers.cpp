#include "controllers.h"
#include <algorithm>
#include <cmath>

namespace differentialdriverobottracking {

namespace {

/// sin(x)/x with the removable singularity at 0 handled.
double sinc(double x) {
    if (std::abs(x) < 1e-6) return 1.0 - x * x / 6.0;
    return std::sin(x) / x;
}

bool finite(const BodyError& e) {
    return std::isfinite(e.e1) && std::isfinite(e.e2) && std::isfinite(e.e3) &&
           std::isfinite(e.de1);
}

}  // namespace

// ===========================================================================
// 2. PID
// ===========================================================================
PIDCtrl::PIDCtrl(const PlantParams& p)
    : p_(p),
      pid_v_([&] {
          ctrl::PIDParams q;
          q.Kp = 2.0; q.Ki = 0.5; q.Kd = 0.05; q.N = 20.0;
          q.uMin = -p.v_max; q.uMax = p.v_max; q.Kb = 1.0;
          return q;
      }(), p.Tf),
      pid_w_([&] {
          ctrl::PIDParams q;
          q.Kp = 4.0; q.Ki = 0.8; q.Kd = 0.10; q.N = 20.0;
          q.uMin = -p.w_max; q.uMax = p.w_max; q.Kb = 1.0;
          return q;
      }(), p.Tf)
{}

Eigen::Vector2d PIDCtrl::compute(const BodyError& e, double v_r, double w_r) {
    // DiscretePID convention: compute(r - y). e1/e3 are already (reference - actual).
    const double dv = pid_v_.compute(e.e1);
    const double dw = pid_w_.compute(e.e3 + 0.5 * e.e2);
    return {std::clamp(v_r + dv, -p_.v_max, p_.v_max),
            std::clamp(w_r + dw, -p_.w_max, p_.w_max)};
}

void PIDCtrl::reset() { pid_v_.reset(); pid_w_.reset(); }

// ===========================================================================
// 3. Backstepping (paper's u1base / u2base)
// ===========================================================================
Eigen::Vector2d BacksteppingCtrl::law(const BodyError& e, double v_r, double w_r,
                                      double k1, double k2, double k3) {
    // Classical Lyapunov backstepping for the nonholonomic error dynamics
    //   e1' = w*e2 - v + v_r*cos(e3),  e2' = -w*e1 + v_r*sin(e3),  e3' = w_r - w
    // with V = (e1^2 + e2^2)/2 + (1 - cos e3)/k2, giving V' = -k1*e1^2 - k3*sin^2(e3)/k2 <= 0.
    // sinc(e3) keeps the e2 term well defined as e3 -> 0.
    const double v = v_r * std::cos(e.e3) + k1 * e.e1;
    const double w = w_r + k2 * v_r * e.e2 * sinc(e.e3) + k3 * std::sin(e.e3);
    return {v, w};
}

BacksteppingCtrl::BacksteppingCtrl(const PlantParams& p)
    : p_(p), k1_(2.0), k2_(4.0), k3_(2.0)   // paper Table 1: K1, K2, K3 = 2.0, 4.0, 2.0
{}

Eigen::Vector2d BacksteppingCtrl::compute(const BodyError& e, double v_r, double w_r) {
    if (!finite(e)) return {v_r, w_r};
    Eigen::Vector2d u = law(e, v_r, w_r, k1_, k2_, k3_);
    return {std::clamp(u(0), -p_.v_max, p_.v_max),
            std::clamp(u(1), -p_.w_max, p_.w_max)};
}

// ===========================================================================
// 4. SMC - paper's s = e2 + 0.8*e3 via c_de = 0
// ===========================================================================
SMCCtrl::SMCCtrl(const PlantParams& p)
    : p_(p),
      pid_v_([&] {
          ctrl::PIDParams q;
          q.Kp = 2.0; q.Ki = 0.5; q.Kd = 0.0; q.uMin = -p.v_max; q.uMax = p.v_max;
          return q;
      }(), p.Tf),
      smc_w_([&] {
          ctrl::SMCParams q;
          // c_de = 0 collapses s[k] = c_e*e + c_de*(e - e_prev) to s = c_e*e, so feeding
          // e_input = e2 + 0.8*e3 gives exactly the paper's surface with no extra state.
          q.c_e = 1.0; q.c_de = 0.0;
          q.K = 3.0;                // paper Table 1: Ks starts at 3.0
          q.phi = 0.2;              // paper Table 1: boundary layer Phi = 0.2
          q.uMin = -p.w_max; q.uMax = p.w_max;
          return q;
      }(), p.Tf)
{}

Eigen::Vector2d SMCCtrl::compute(const BodyError& e, double v_r, double w_r) {
    const double dv = pid_v_.compute(e.e1);
    // DiscreteSMC takes e = y - r, i.e. the negated (reference - actual) surface signal.
    const double s_in = -(e.e2 + 0.8 * e.e3);
    const double dw   = smc_w_.compute(s_in);
    return {std::clamp(v_r + dv, -p_.v_max, p_.v_max),
            std::clamp(w_r + dw, -p_.w_max, p_.w_max)};
}

void SMCCtrl::reset() { pid_v_.reset(); smc_w_.reset(); }

// ===========================================================================
// 5. AdaptiveSMC
// ===========================================================================
AdaptiveSMCCtrl::AdaptiveSMCCtrl(const PlantParams& p)
    : p_(p),
      pid_v_([&] {
          ctrl::PIDParams q;
          q.Kp = 2.0; q.Ki = 0.5; q.Kd = 0.0; q.uMin = -p.v_max; q.uMax = p.v_max;
          return q;
      }(), p.Tf),
      asmc_w_([&] {
          ctrl::AdaptiveSMCParams q;
          q.c_e = 1.0; q.c_de = 0.0;
          q.K0 = 3.0; q.Kmin = 3.0; q.Kmax = 5.0;   // paper: Ks in 3.0 - 4.6
          q.gamma = 0.35; q.epsilon = 0.02;
          q.phi = 0.2;
          q.uMin = -p.w_max; q.uMax = p.w_max;
          return q;
      }(), p.Tf)
{}

Eigen::Vector2d AdaptiveSMCCtrl::compute(const BodyError& e, double v_r, double w_r) {
    const double dv   = pid_v_.compute(e.e1);
    const double s_in = -(e.e2 + 0.8 * e.e3);
    const double dw   = asmc_w_.compute(s_in);
    return {std::clamp(v_r + dv, -p_.v_max, p_.v_max),
            std::clamp(w_r + dw, -p_.w_max, p_.w_max)};
}

void AdaptiveSMCCtrl::reset() { pid_v_.reset(); asmc_w_.reset(); }

CtrlTelemetry AdaptiveSMCCtrl::telemetry() const {
    CtrlTelemetry t;
    t.Ks = asmc_w_.adaptiveGain();
    return t;
}

// ===========================================================================
// 6. ADRC
// ===========================================================================
// ctrl::DiscreteADRC is a SECOND-order ADRC (z1 = y, z2 = ydot, z3 = total disturbance), so
// b0 must be the gain from the command to the SECOND derivative of the regulated signal. Here
// the outer loop commands a velocity that the inner PI tracks with time constant tau_i ~ 0.05 s,
// and that velocity integrates into position: y'' ~ u/tau_i, hence b0 ~ 1/tau_i ~ 20.
// The ESO gains are beta = (3*w0, 3*w0^2, w0^3); at Ts = 30 ms, w0 = 20 would give
// beta3*Ts = 240 - an enormously hot observer. w0 = 5 keeps beta3*Ts ~ 3.8.
ADRCCtrl::ADRCCtrl(const PlantParams& p)
    : p_(p),
      adrc_v_([&] {
          ctrl::ADRCParams q;
          q.omega_c = 1.5; q.omega_o = 5.0; q.b0 = 20.0;
          q.uMin = -p.v_max; q.uMax = p.v_max;
          return q;
      }(), p.Tf),
      adrc_w_([&] {
          ctrl::ADRCParams q;
          q.omega_c = 2.0; q.omega_o = 6.0; q.b0 = 20.0;
          q.uMin = -p.w_max; q.uMax = p.w_max;
          return q;
      }(), p.Tf)
{}

Eigen::Vector2d ADRCCtrl::compute(const BodyError& e, double v_r, double w_r) {
    // computeTracking(y, r): regulate the error signals to zero, so y = -e, r = 0.
    const double dv = adrc_v_.computeTracking(-e.e1, 0.0);
    const double dw = adrc_w_.computeTracking(-(e.e3 + 0.5 * e.e2), 0.0);
    return {std::clamp(v_r + dv, -p_.v_max, p_.v_max),
            std::clamp(w_r + dw, -p_.w_max, p_.w_max)};
}

void ADRCCtrl::reset() { adrc_v_.reset(); adrc_w_.reset(); }

// ===========================================================================
// 7. FuzzyTSK - fixed-weight version of the paper's NFS
// ===========================================================================
namespace {

/// Build a 5-term Gaussian input variable spanning [-lim, lim].
ctrl::LinguisticVariable gaussianVar(const std::string& nm, double lim) {
    ctrl::LinguisticVariable v;
    v.name = nm;
    v.lo = -lim; v.hi = lim;
    const char* labels[5] = {"NL", "NS", "ZE", "PS", "PL"};
    const double sigma = lim / 2.0;   // adjacent centres overlap at ~0.6 membership
    for (int i = 0; i < 5; ++i) {
        ctrl::LinguisticTerm t;
        t.name = labels[i];
        const double c = -lim + i * (2.0 * lim / 4.0);
        t.mf   = ctrl::mfGaussian(c, sigma);
        t.peak = c;
        v.terms.push_back(t);
    }
    return v;
}

/// Singleton output variable with 5 terms spanning [-lim, lim] (Takagi-Sugeno consequents).
ctrl::LinguisticVariable singletonVar(const std::string& nm, double lim) {
    ctrl::LinguisticVariable v;
    v.name = nm;
    v.lo = -lim; v.hi = lim;
    const char* labels[5] = {"NL", "NS", "ZE", "PS", "PL"};
    for (int i = 0; i < 5; ++i)
        v.terms.push_back(ctrl::ltSingleton(labels[i], -lim + i * (2.0 * lim / 4.0)));
    return v;
}

/// Standard 25-rule diagonal FAM table: consequent index = clamp(round((i + j)/2), 0, 4).
void addFamRules(ctrl::FuzzySystem& fs) {
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            ctrl::Rule r;
            r.antecedents = {{0, i}, {1, j}};
            const int k = std::clamp(static_cast<int>(std::lround((i + j) / 2.0)), 0, 4);
            r.consequent_term_idx = k;
            r.weight = 1.0;
            fs.addRule(r);
        }
    }
}

}  // namespace

FuzzyTSKCtrl::FuzzyTSKCtrl(const PlantParams& p) : p_(p) {
    fs_v_.params.inference = ctrl::InferenceMethod::TakagiSugeno;
    fs_v_.params.defuzz    = ctrl::DefuzzMethod::WeightedAverage;
    fs_v_.params.uMin = -p.v_max; fs_v_.params.uMax = p.v_max;
    fs_v_.addInput(gaussianVar("e1", 1.0));
    fs_v_.addInput(gaussianVar("de1", 2.0));
    fs_v_.addOutput(singletonVar("dv", p.v_max));
    addFamRules(fs_v_);

    fs_w_.params.inference = ctrl::InferenceMethod::TakagiSugeno;
    fs_w_.params.defuzz    = ctrl::DefuzzMethod::WeightedAverage;
    fs_w_.params.uMin = -p.w_max; fs_w_.params.uMax = p.w_max;
    fs_w_.addInput(gaussianVar("e3", 1.0));
    fs_w_.addInput(gaussianVar("de3", 2.0));
    fs_w_.addOutput(singletonVar("dw", p.w_max));
    addFamRules(fs_w_);

    in_v_.assign(2, 0.0);
    in_w_.assign(2, 0.0);
}

Eigen::Vector2d FuzzyTSKCtrl::compute(const BodyError& e, double v_r, double w_r) {
    if (!finite(e)) return {v_r, w_r};
    in_v_[0] = e.e1;              in_v_[1] = e.de1;
    in_w_[0] = e.e3 + 0.5 * e.e2; in_w_[1] = e.de3;
    const double dv = fs_v_.evaluate(in_v_);
    const double dw = fs_w_.evaluate(in_w_);
    return {std::clamp(v_r + dv, -p_.v_max, p_.v_max),
            std::clamp(w_r + dw, -p_.w_max, p_.w_max)};
}

void FuzzyTSKCtrl::reset() {}   // FuzzySystem inference is memoryless

// ===========================================================================
// 8. LQR
// ===========================================================================
LQRCtrl::LQRCtrl(const PlantParams& p, double v_r0, double w_r0) : p_(p) {
    // Error dynamics linearised about (e = 0, v = v_r0, w = w_r0):
    //   e1' =  w_r0*e2 - dv
    //   e2' = -w_r0*e1 + v_r0*e3
    //   e3' = -dw
    Eigen::MatrixXd Ac(3, 3);
    Ac <<      0.0,  w_r0, 0.0,
             -w_r0,   0.0, v_r0,
               0.0,   0.0, 0.0;
    Eigen::MatrixXd Bc(3, 2);
    Bc << -1.0,  0.0,
           0.0,  0.0,
           0.0, -1.0;

    // Forward-Euler discretisation at Tf is adequate: |eig(Ac)|*Tf << 1 for these speeds.
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(3, 3) + p.Tf * Ac;
    Eigen::MatrixXd Bd = p.Tf * Bc;
    ctrl::StateSpace ss(Ad, Bd, Eigen::MatrixXd::Identity(3, 3),
                        Eigen::MatrixXd::Zero(3, 2), p.Tf);

    ctrl::LQRParams q;
    q.Q = Eigen::MatrixXd::Zero(3, 3);
    q.Q.diagonal() << 10.0, 10.0, 5.0;
    q.R = Eigen::MatrixXd::Zero(2, 2);
    q.R.diagonal() << 1.0, 1.0;
    lqr_ = std::make_shared<ctrl::DiscreteLQR>(ss, q);

    x_ = Eigen::VectorXd::Zero(3);
}

Eigen::Vector2d LQRCtrl::compute(const BodyError& e, double v_r, double w_r) {
    if (!finite(e)) return {v_r, w_r};
    x_(0) = e.e1; x_(1) = e.e2; x_(2) = e.e3;
    // DiscreteLQR regulates x -> 0 with u = -K*x; our B already carries the -1 signs, so the
    // returned u is the (dv, dw) correction to add to the feedforward.
    const Eigen::VectorXd u = lqr_->compute(x_);
    return {std::clamp(v_r + u(0), -p_.v_max, p_.v_max),
            std::clamp(w_r + u(1), -p_.w_max, p_.w_max)};
}

// ===========================================================================
// 9. NMPC
// ===========================================================================
NMPCCtrl::NMPCCtrl(const PlantParams& p) : p_(p) {
    vr_ref_ = std::make_shared<double>(0.0);
    wr_ref_ = std::make_shared<double>(0.0);

    ctrl::NMPCParams q;
    q.Np = 10; q.Nu = 3;
    q.rho_y = 1.0; q.rho_u = 0.05;
    q.uMin = -p.w_max; q.uMax = p.w_max;   // widest of the two channels; per-channel clamp below
    q.Ts = p.Tf;
    q.n_states = 3; q.n_inputs = 2; q.n_outputs = 3;
    q.qpMaxIter = 60;      // keep the per-step cost bounded: 60 runs x 1000 steps
    q.qpTol = 1e-4;

    auto vr = vr_ref_;
    auto wr = wr_ref_;
    const double Tf = p.Tf;
    // Discrete nonholonomic error dynamics; u = [dv, dw] corrections about (v_r, w_r).
    auto f_d = [vr, wr, Tf](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        const double e1 = x(0), e2 = x(1), e3 = x(2);
        const double v = *vr + u(0);
        const double w = *wr + u(1);
        Eigen::VectorXd xn(3);
        xn(0) = e1 + Tf * ( w * e2 - v + *vr * std::cos(e3));
        xn(1) = e2 + Tf * (-w * e1 + *vr * std::sin(e3));
        xn(2) = e3 + Tf * ( *wr - w);
        return xn;
    };

    nmpc_ = std::make_shared<ctrl::NonlinearMPC>(q, f_d);
    x_    = Eigen::VectorXd::Zero(3);
    yref_ = Eigen::VectorXd::Zero(3);
}

Eigen::Vector2d NMPCCtrl::compute(const BodyError& e, double v_r, double w_r) {
    if (!finite(e)) return {v_r, w_r};
    *vr_ref_ = v_r;
    *wr_ref_ = w_r;
    x_(0) = e.e1; x_(1) = e.e2; x_(2) = e.e3;
    // Full MIMO path: computeRef(x, y_ref) returns u*_0 directly (2x1). y_ref = 0 drives the
    // error state to the origin.
    const Eigen::VectorXd u = nmpc_->computeRef(x_, yref_);
    const double dv = (u.size() > 0 && std::isfinite(u(0))) ? u(0) : 0.0;
    const double dw = (u.size() > 1 && std::isfinite(u(1))) ? u(1) : 0.0;
    return {std::clamp(v_r + dv, -p_.v_max, p_.v_max),
            std::clamp(w_r + dw, -p_.w_max, p_.w_max)};
}

void NMPCCtrl::reset() { nmpc_->reset(); }

// ===========================================================================
// 10. L1Adaptive
// ===========================================================================
L1AdaptiveCtrl::L1AdaptiveCtrl(const PlantParams& p)
    : p_(p),
      // Gamma is deliberately small: the pseudo-output y = -e is driven by a fast inner loop,
      // so an aggressive adaptation rate chases the reference model faster than the plant can
      // follow and the estimate winds up against sigma_max.
      l1_v_([&] {
          ctrl::L1AdaptiveController::Params q;
          q.a_m = 0.90; q.b_m = 0.10; q.k_g = 1.0;
          q.Gamma = 2.0; q.omega_c = 1.5; q.sigma_max = 3.0;
          q.uMin = -p.v_max; q.uMax = p.v_max;
          return q;
      }(), p.Tf),
      l1_w_([&] {
          ctrl::L1AdaptiveController::Params q;
          q.a_m = 0.88; q.b_m = 0.12; q.k_g = 1.0;
          q.Gamma = 2.0; q.omega_c = 2.0; q.sigma_max = 4.0;
          q.uMin = -p.w_max; q.uMax = p.w_max;
          return q;
      }(), p.Tf)
{}

Eigen::Vector2d L1AdaptiveCtrl::compute(const BodyError& e, double v_r, double w_r) {
    if (!finite(e)) return {v_r, w_r};
    // L1 convention: setReference(r) then compute(y_plant). Pseudo-output y = -e regulated to 0.
    //
    // L1 is layered ON TOP OF the nominal kinematic law rather than replacing it. Driving the
    // body-frame error directly from a bare L1 loop is unstable here: L1's internal first-order
    // reference model (pole a_m) assumes a stable relative-degree-1 plant, but velocity-command
    // to position-error is an INTEGRATOR, so the predictor and the plant diverge and sigma_hat
    // winds up against sigma_max (measured: ISE 2091 on the circle, versus 0.24 for the nominal
    // law alone). Augmenting a stabilised baseline is the standard L1 architecture anyway -
    // L1 supplies the adaptive uncertainty compensation, not the stabilising action.
    const Eigen::Vector2d u_nom = BacksteppingCtrl::law(e, v_r, w_r, 2.0, 4.0, 2.0);

    l1_v_.setReference(0.0);
    l1_w_.setReference(0.0);
    const double dv = l1_v_.compute(-e.e1);
    const double dw = l1_w_.compute(-(e.e3 + 0.5 * e.e2));
    return {std::clamp(u_nom(0) + dv, -p_.v_max, p_.v_max),
            std::clamp(u_nom(1) + dw, -p_.w_max, p_.w_max)};
}

void L1AdaptiveCtrl::reset() { l1_v_.reset(); l1_w_.reset(); }

// ===========================================================================
// 11. GainScheduled - scheduled on reference curvature |w_r / v_r|
// ===========================================================================
GainScheduledCtrl::GainScheduledCtrl(const PlantParams& p)
    : p_(p),
      pid_v_([&] {
          ctrl::PIDParams q;
          q.Kp = 2.0; q.Ki = 0.5; q.Kd = 0.0; q.uMin = -p.v_max; q.uMax = p.v_max;
          return q;
      }(), p.Tf)
{
    sched_w_ = std::make_shared<ctrl::GainScheduledController>(
        p.Tf, ctrl::GainScheduleMode::LinearBlend);

    // Three design points across path curvature kappa = |w_r/v_r| [1/m]:
    // straight (0), moderate (1), tight corner (4). Gains rise with curvature.
    const double kappa[3] = {0.0, 1.0, 4.0};
    const double Kp[3]    = {2.5, 4.5, 7.0};
    const double Ki[3]    = {0.4, 0.8, 1.2};
    const double Kd[3]    = {0.05, 0.10, 0.15};
    for (int i = 0; i < 3; ++i) {
        ctrl::PIDParams q;
        q.Kp = Kp[i]; q.Ki = Ki[i]; q.Kd = Kd[i]; q.N = 20.0;
        q.uMin = -p.w_max; q.uMax = p.w_max;
        sched_w_->addSchedulePoint(kappa[i], std::make_shared<ctrl::DiscretePID>(q, p.Tf));
    }
}

Eigen::Vector2d GainScheduledCtrl::compute(const BodyError& e, double v_r, double w_r) {
    if (!finite(e)) return {v_r, w_r};
    const double kappa = (std::abs(v_r) > 1e-3) ? std::abs(w_r / v_r) : 4.0;
    sched_w_->setSchedulingParam(std::clamp(kappa, 0.0, 4.0));
    const double dv = pid_v_.compute(e.e1);
    const double dw = sched_w_->compute(e.e3 + 0.5 * e.e2);
    return {std::clamp(v_r + dv, -p_.v_max, p_.v_max),
            std::clamp(w_r + dw, -p_.w_max, p_.w_max)};
}

void GainScheduledCtrl::reset() { pid_v_.reset(); sched_w_->reset(); }

// ===========================================================================
// 12. FUHAC - the paper's proposed controller
// ===========================================================================
FUHACCtrl::FUHACCtrl(const PlantParams& p)
    : p_(p),
      smc_([&] {
          ctrl::AdaptiveSMCParams q;
          // c_de = 0 => s = c_e*e_input; e_input = e2 + 0.8*e3 gives the paper's surface.
          q.c_e = 1.0; q.c_de = 0.0;
          q.K0 = 3.0; q.Kmin = 3.0; q.Kmax = 5.0;   // paper Table 1 / Table 2: Ks 3.0 -> 3.7-4.6
          q.gamma = 0.30; q.epsilon = 0.02;
          q.phi = 0.2;                              // paper Table 1: Phi = 0.2
          q.uMin = -p.w_max; q.uMax = p.w_max;
          return q;
      }(), p.Tf)
{
    // Gaussian centres on a uniform grid; widths chosen so adjacent MFs overlap at ~0.6.
    const double lim_e = 1.0, lim_de = 2.0;
    for (int i = 0; i < NF_GRID; ++i) {
        c_e_[i]  = -lim_e  + i * (2.0 * lim_e  / (NF_GRID - 1));
        c_de_[i] = -lim_de + i * (2.0 * lim_de / (NF_GRID - 1));
    }
    sigma_e_  = lim_e  / 2.0;
    sigma_de_ = lim_de / 2.0;
    reset();
}

void FUHACCtrl::reset() {
    w_.fill(0.0);
    dw_prev_.fill(0.0);
    mu_e_.fill(0.0);
    mu_de_.fill(0.0);
    phi_.fill(0.0);
    u1nf_prev_ = 0.0;
    smc_.reset();
    d_hat_ = 0.0;
    y_hat_ = 0.0;
    alpha_ = alpha_max_;
    t_elapsed_ = 0.0;
    perf_index_ = 0.0;
    osc_ = 0.0;
    u2_prev_ = 0.0;
    V_ = 0.0;
}

// -- Takagi-Sugeno-Kang neuro-fuzzy compensator with online learning ---------
// mu_i(x) = exp[-0.5*((x - c_i)/sigma_i)^2],  phi_ij = mu_i(e1)*mu_j(de1)
// u1NF[k]  = 0.8*u1NF[k-1] + 0.2 * sum(phi_ij * w_ij) / sum(phi_ij)
// Weights adapt by gradient descent with momentum, projected onto |w| <= w_max
// (the paper's Proj(tau, W) operator bounding weight drift).
double FUHACCtrl::neuroFuzzy(double e1, double de1) {
    double sum_phi = 0.0;
    for (int i = 0; i < NF_GRID; ++i) {
        const double ze = (e1 - c_e_[i]) / sigma_e_;
        mu_e_[i] = std::exp(-0.5 * ze * ze);
    }
    for (int j = 0; j < NF_GRID; ++j) {
        const double zd = (de1 - c_de_[j]) / sigma_de_;
        mu_de_[j] = std::exp(-0.5 * zd * zd);
    }
    for (int i = 0; i < NF_GRID; ++i) {
        for (int j = 0; j < NF_GRID; ++j) {
            const double ph = mu_e_[i] * mu_de_[j];
            phi_[i * NF_GRID + j] = ph;
            sum_phi += ph;
        }
    }
    if (!(sum_phi > 1e-9)) return u1nf_prev_;   // no rule fired: hold

    double num = 0.0;
    for (int k = 0; k < NF_GRID * NF_GRID; ++k) num += phi_[k] * w_[k];
    const double defuzz = num / sum_phi;

    // Online update: descend d(0.5*e1^2)/dw_ij = -e1 * phi_ij/sum_phi, with momentum.
    for (int k = 0; k < NF_GRID * NF_GRID; ++k) {
        const double grad = e1 * phi_[k] / sum_phi;
        const double dw   = eta1_ * grad + momentum_ * dw_prev_[k];
        dw_prev_[k] = dw;
        w_[k] = std::clamp(w_[k] + dw, -w_max_, w_max_);   // projection operator
    }

    u1nf_prev_ = 0.8 * u1nf_prev_ + 0.2 * defuzz;          // paper's output filter
    return u1nf_prev_;
}

// -- First-order disturbance observer ---------------------------------------
// dhat' = L*(y - yhat);  dhat[k] = gamma*dhat[k-1] + (1-gamma)*clamp(dhat[k], d_max)
// Noise transfer is -L/(s + L), i.e. a first-order low pass with cutoff L. L = 20 rad/s sits
// between the robot bandwidth (~3.3 rad/s) and the 30 ms-loop Nyquist (~105 rad/s), which is
// the paper's omega_dynamics << L << omega_noise rule.
//
// The observed channel is the longitudinal error: e1' = -(v - v_r) + d + (coupling). Speeding
// up therefore REDUCES e1, hence the leading minus on the applied velocity excess - getting
// this sign wrong makes the estimate diverge to its clamp and inject a constant bias.
double FUHACCtrl::disturbanceObserver(double e1, double u_applied) {
    const double Ts = p_.Tf;
    y_hat_ += Ts * (-u_applied + d_hat_);
    const double innov = e1 - y_hat_;
    const double d_raw = d_hat_ + Ts * L_dob_ * innov;
    const double d_clamped = std::clamp(d_raw, -d_max_, d_max_);
    d_hat_ = gamma_ * d_hat_ + (1.0 - gamma_) * d_clamped;
    if (!std::isfinite(d_hat_)) d_hat_ = 0.0;
    if (!std::isfinite(y_hat_)) y_hat_ = 0.0;
    return d_hat_;
}

// -- Predictive error compensator -------------------------------------------
// e_pred(k+i) = [e(k) + de(k)*i*dt] * exp(-0.5*i), i = 1..H. The mean predicted error
// relative to the current error gives a phase-lead multiplier Gpred ~ 1 + s*dT.
double FUHACCtrl::predictiveGain(const BodyError& e) const {
    double mean_pred = 0.0;
    for (int i = 1; i <= H_PRED; ++i) {
        mean_pred += (e.e1 + e.de1 * i * dt_pred_) * std::exp(-0.5 * i);
    }
    mean_pred /= H_PRED;
    if (std::abs(e.e1) < 1e-6) return 1.0;
    const double g = mean_pred / e.e1;
    return std::clamp(std::isfinite(g) ? g : 1.0, 0.8, 1.5);
}

// -- Adaptive blending factor ------------------------------------------------
// alpha_raw = alpha_min + (alpha_max - alpha_min)*[wt*T + we*E + wp*P + wo*O]
// The paper names the four indicators but never defines them; each is defined here as a
// normalised, saturated-to-[0,1] quantity (see README "Adaptive blending indicators"):
//   T(t) : elapsed-time maturity, 1 - exp(-t/5)   -> trust the model law once settled
//   E(e) : 1/(1 + ||e||)                          -> small error favours backstepping
//   P(I) : 1/(1 + perf_index)                     -> low accumulated ISE favours backstepping
//   O(l) : 1/(1 + osc)                            -> low control oscillation favours backstepping
double FUHACCtrl::updateAlpha(const BodyError& e) {
    const double norm_e = std::sqrt(e.e1 * e.e1 + e.e2 * e.e2 + e.e3 * e.e3);
    const double T = 1.0 - std::exp(-t_elapsed_ / 5.0);
    const double E = 1.0 / (1.0 + norm_e);
    const double P = 1.0 / (1.0 + perf_index_);
    const double O = 1.0 / (1.0 + osc_);
    const double blend = std::clamp(wt_ * T + we_ * E + wp_ * P + wo_ * O, 0.0, 1.0);
    const double alpha_raw = alpha_min_ + (alpha_max_ - alpha_min_) * blend;
    alpha_ = beta_ * alpha_ + (1.0 - beta_) * alpha_raw;   // paper Table 1: beta = 0.97
    alpha_ = std::clamp(alpha_, alpha_min_, alpha_max_);
    return alpha_;
}

Eigen::Vector2d FUHACCtrl::compute(const BodyError& e, double v_r, double w_r) {
    // NaN contract (CLAUDE.md Sec. 7): hold the feedforward on non-finite input.
    if (!finite(e)) return {v_r, w_r};

    t_elapsed_ += p_.Tf;

    // 1. Backstepping baseline (shared with BacksteppingCtrl).
    const Eigen::Vector2d u_base = BacksteppingCtrl::law(e, v_r, w_r, k1_, k2_, k3_);
    const double u1_base = u_base(0);
    const double u2_base = u_base(1);

    // 2. Neuro-fuzzy branch on the longitudinal channel. The NFS supplies a LEARNED RESIDUAL
    //    on top of the nominal kinematic law, not a replacement for it: the paper's role for
    //    the NFS is "approximate unknown nonlinear dynamics online". Were u1NF the bare
    //    defuzzified output, its zero-initialised weights would give the branch no authority
    //    at t = 0, and since alpha falls as the error grows, a large error would REMOVE
    //    corrective action exactly when it is most needed.
    const double u1_nf = v_r * std::cos(e.e3) + k1_ * e.e1 + neuroFuzzy(e.e1, e.de1);

    // 3. Robust stabilising term on the angular channel (high-gain proportional fallback).
    const double u2_stab = w_r + 3.0 * e.e3 + 1.5 * e.e2;

    // 4. Adaptive-gain SMC. AdaptiveSMC takes e = y - r, hence the negation.
    const double u2_smc = smc_.compute(-(e.e2 + 0.8 * e.e3));

    // 5. Blend (paper's composite law).
    const double a = updateAlpha(e);
    double u1 = a * u1_base + (1.0 - a) * u1_nf;
    double u2 = a * u2_base + (1.0 - a) * u2_stab + u2_smc;

    // 6. Disturbance feedforward + predictive phase lead. A positive d_hat means an unmodelled
    //    effect is pushing e1 up (robot falling behind), so it is cancelled by ADDING velocity.
    const double dhat = disturbanceObserver(e.e1, u1 - v_r);
    u1 += dhat;
    u1 *= predictiveGain(e);

    // 7. Running indicators for the next alpha update.
    perf_index_ += (e.e1 * e.e1 + e.e2 * e.e2) * p_.Tf;
    osc_ = 0.95 * osc_ + 0.05 * std::abs(u2 - u2_prev_) / p_.Tf;
    u2_prev_ = u2;

    // 8. Composite Lyapunov value (paper Step 1). The paper's candidate is
    //      V = |e|^2/2 + W~'W~/(2*eta1) + s^2/(2*eta2) + d~^2/(2*eta3)
    //    where W~ = W - W* and d~ = d - dhat are estimation ERRORS against the unknown ideal
    //    W* and true disturbance d. Neither ideal is observable in simulation, so the logged
    //    trace covers the three terms that ARE computable - tracking, sliding-surface and
    //    disturbance-estimate energy. Documented in README "Deviations from the paper".
    const double s = smc_.slidingSurface();
    V_ = 0.5 * (e.e1 * e.e1 + e.e2 * e.e2 + e.e3 * e.e3)
       + 0.5 * s * s
       + 0.5 * dhat * dhat;

    if (!std::isfinite(u1)) u1 = v_r;
    if (!std::isfinite(u2)) u2 = w_r;
    return {std::clamp(u1, -p_.v_max, p_.v_max),
            std::clamp(u2, -p_.w_max, p_.w_max)};
}

// Slow loop (150 ms): relax the adaptive weights toward nominal so they cannot drift under
// persistent excitation. This is the paper's slow subsystem theta' = eps*g(e, theta), and the
// leak rate is what keeps eps = Tf/Ts_slow = 0.2 a genuine time-scale separation.
void FUHACCtrl::slowTick() {
    constexpr double kLeak = 0.02;
    for (int k = 0; k < NF_GRID * NF_GRID; ++k) w_[k] *= (1.0 - kLeak);
    osc_ *= (1.0 - kLeak);
}

CtrlTelemetry FUHACCtrl::telemetry() const {
    CtrlTelemetry t;
    t.alpha = alpha_;
    t.Ks    = smc_.adaptiveGain();
    t.d_hat = d_hat_;
    t.V     = V_;
    return t;
}

// ===========================================================================
// Roster
// ===========================================================================
std::vector<ControllerPtr> makeControllers(const PlantParams& p) {
    std::vector<ControllerPtr> out;
    out.push_back(std::make_unique<OpenLoopCtrl>(p));
    out.push_back(std::make_unique<PIDCtrl>(p));
    out.push_back(std::make_unique<BacksteppingCtrl>(p));
    out.push_back(std::make_unique<SMCCtrl>(p));
    out.push_back(std::make_unique<AdaptiveSMCCtrl>(p));
    out.push_back(std::make_unique<ADRCCtrl>(p));
    out.push_back(std::make_unique<FuzzyTSKCtrl>(p));
    // LQR design point: nominal cruise of the a = 2 m lemniscate (~1.5 m/s, ~1.0 rad/s).
    out.push_back(std::make_unique<LQRCtrl>(p, 1.5, 1.0));
    out.push_back(std::make_unique<NMPCCtrl>(p));
    out.push_back(std::make_unique<L1AdaptiveCtrl>(p));
    out.push_back(std::make_unique<GainScheduledCtrl>(p));
    out.push_back(std::make_unique<FUHACCtrl>(p));
    return out;
}

}  // namespace differentialdriverobottracking
