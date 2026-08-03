// ex138_safety_supervised_adaptation.cpp - A safety filter is only as good as its model,
//                                          and only as good as WHO estimated that model.
//
// Fusion: ctrl::CBFSafetyFilter + ctrl::L1AdaptiveController + ctrl::CLFController
//         + ctrl::KalmanFilter + ctrl::ControllerMonitor
//
// docs/fusion_opportunity_backlog.md item B2: "the safety filter enforces constraints while
// adaptation runs underneath". Building it turns up two things that framing hides.
//
// FIRST: a CBF is not a guarantee. It is a guarantee CONDITIONAL on the control-affine model
// (f0, g) it was handed. `CBFSafetyFilter` enforces
//
//     (dh/dx . g(x)) . u  >=  -dh/dx . f0(x) - alpha . h(x)
//
// with the caller's f0 and g. Tell it the input gain is 1.0 when the plant's is 1.6 and it
// authorises 60 % more control action than the plant can safely absorb - and the state leaves
// the safe set while the filter reports success throughout. The barrier holds in the model.
//
// The repair looks obvious. Matched uncertainty - anything entering the input channel, which
// covers both a gain error and a load disturbance - is EXACTLY representable as a drift offset:
//
//     xdot = -a x + b_true u + d   ==   -a x + b_nom (u + sigma),
//     sigma = (b_true - b_nom)/b_nom . u + d/b_nom
//
// so f0_eff(x) = -a x + b_nom . sigma is the honest drift, and estimating sigma is exactly what
// an adaptive law does. Wire the adaptive layer's estimate into the safety filter's f0 and the
// constraint should become the one the true plant would have produced. Algebraically it does:
// at the barrier the shared-estimate constraint reduces to u <= (a.x_max - d)/b_true, which is
// precisely the true requirement (exact when b_nom = 1).
//
// SECOND, and this is the finding: L1AdaptiveController's own sigma_hat is the WRONG estimate
// to use, and using it makes things slightly worse rather than better. L1 advances its internal
// state predictor with the command IT computed. The CBF sits downstream and modifies that
// command before it reaches the plant. L1 never learns this happened, so its sigma_hat silently
// becomes "plant uncertainty PLUS whatever the filter just did" - and feeding that back into
// the filter closes a loop through the filter's own edits. Measured below: L1 reports
// sigma_hat ~ 0.10 where the true matched uncertainty is ~ 0.40, a 4x error, and the barrier
// violation is unchanged.
//
// This is an input-consistency break, not a tuning problem. It cannot be fixed by adjusting
// Gamma (swept: no value helps) because the shipped API has no way to tell L1 what was actually
// applied. The estimator has to be one that OBSERVES the applied command - so the drift
// correction comes from a ctrl::KalmanFilter on the augmented state [x; sigma], whose
// step(y, u_prev, u_current) takes the applied input explicitly. It recovers sigma exactly.
//
// L1 keeps its real job (adaptive tracking); the KF supplies the safety filter. That role split
// is the actual lesson: an adaptive controller's internal uncertainty estimate is a control
// signal, not a plant measurement, and nesting it under a filter that rewrites its output
// invalidates it for any downstream consumer.
//
// Five arms, identical scenario, load and setpoint throughout:
//
//   1. CLF only                    model-based, no adaptation, no filter
//   2. L1 only                     adaptive, no filter
//   3. L1 + CBF, nominal f0        filter blind to the uncertainty
//   4. L1 + CBF fed L1's sigma     the intuitive fusion - and it does not work
//   5. L1 + CBF fed a KF sigma     the fusion that does
//
// Composition notes:
//
//   1. CBFSafetyFilter forwards its scalar argument to the nominal controller untouched, so it
//      is convention-agnostic: L1AdaptiveController wants the absolute plant output (not an
//      error) and passing y straight through works. CLFController ignores the argument entirely
//      and reads state injected via setState().
//
//   2. CLFController is a REGULATOR toward V's minimum with no equilibrium feedforward and no
//      integral action, so on a plant whose equilibrium needs a nonzero input it parks at an
//      offset. Driving it to a setpoint means supplying u_eq yourself - hence the small
//      BiasedController adapter below, in the spirit of ex134's HangingController.
//
//   3. ControllerMonitor::nAlarms() only increments when an alarm callback is registered, and
//      SPC on a non-stationary signal alarms on motion rather than on faults (ex132's lesson,
//      re-learned here the hard way: charting u through the startup transient produced ~3900
//      alarms in every arm). The monitor therefore goes live only after commissioning, with its
//      target taken from the settled command.
//
// Scope: sigma is a MATCHED-uncertainty estimate. An error in the drift coefficient a is
// unmatched and is NOT exactly representable this way. This demo keeps a_true == a_nom so the
// claim under test stays the one stated.

#include <ControllerToolbox.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

namespace {

constexpr double kTs   = 0.02;
constexpr double kTsim = 40.0;
constexpr int    kN    = static_cast<int>(kTsim / kTs + 0.5);

constexpr double kA     = 0.50;   // drift; identical in model and truth (see scope note)
constexpr double kBNom  = 1.00;   // input gain the CBF and the L1 reference model believe
constexpr double kBTrue = 1.60;   // ... and what the plant actually has: +60 %

constexpr double kXMax    = 1.00; // hard limit: h(x) = kXMax - x >= 0
constexpr double kRef     = 1.15; // setpoint ABOVE the limit, so safety and tracking conflict
constexpr double kRefSafe = 0.50; // control condition: well inside the safe set

constexpr double kLoad     = 0.30;
constexpr double kLoadTime = 20.0;

constexpr double kAlpha = 2.0;
constexpr double kUMin  = -2.0, kUMax = 3.0;

constexpr double kCommissionTime = 15.0;  // monitor goes live here, after the transient settles

/// Adds a constant equilibrium input to a wrapped controller. CLFController regulates toward
/// V's minimum and has no feedforward of its own.
class BiasedController : public ctrl::IController
{
public:
    BiasedController(std::shared_ptr<ctrl::IController> inner, double bias, double Ts)
        : inner_(std::move(inner)), bias_(bias), Ts_(Ts) {}

    double compute(double s) override { return inner_->compute(s) + bias_; }
    void   reset() override { inner_->reset(); }
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "Biased(" + inner_->name() + ")"; }

private:
    std::shared_ptr<ctrl::IController> inner_;
    double bias_, Ts_;
};

/// Augmented model for the disturbance observer: state = [x; sigma], input = the APPLIED
/// command, output = x. sigma is modelled as a random walk.
ctrl::StateSpace augmentedModel()
{
    Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
    A << 1.0 - kA * kTs, kBNom * kTs,
         0.0,            1.0;
    B << kBNom * kTs,
         0.0;
    C << 1.0, 0.0;
    D << 0.0;
    return ctrl::StateSpace(A, B, C, D, kTs);
}

enum class Nominal { CLF, L1 };
enum class Sigma   { None, FromL1, FromKF };

struct ArmSpec {
    const char *name;
    Nominal     nominal;
    bool        use_cbf;
    Sigma       sigma_src;
    double      ref  = kRef;
    bool        load = true;
};

struct ArmResult {
    double max_x = 0.0, violation = 0.0, iae = 0.0;
    double sigma_used = 0.0, sigma_true = 0.0;
    int    interventions = 0, alarms = 0;
    bool   finite = true;
};

ArmResult runArm(const ArmSpec &spec)
{
    ArmResult r;
    double sigma_shared = 0.0;   // the drift correction handed to the CBF

    // -- nominal layer -------------------------------------------------------------------
    std::shared_ptr<ctrl::L1AdaptiveController> l1;
    std::shared_ptr<ctrl::CLFController>        clf;
    std::shared_ptr<ctrl::IController>          nominal;

    if (spec.nominal == Nominal::L1) {
        ctrl::L1AdaptiveController::Params lp;
        lp.a_m   = 1.0 - kA * kTs;
        lp.b_m   = kBNom * kTs;
        lp.k_g   = (1.0 - lp.a_m) / lp.b_m;
        lp.Gamma = 3.0;      // swept: >= 10 makes L1's own estimate wander badly
        lp.omega_c   = 8.0;
        lp.sigma_max = 20.0;
        lp.uMin = kUMin; lp.uMax = kUMax;
        l1 = std::make_shared<ctrl::L1AdaptiveController>(lp, kTs);
        nominal = l1;
    } else {
        ctrl::CLFParams cp;
        cp.alpha = 2.0;
        cp.uMin = kUMin; cp.uMax = kUMax;
        clf = std::make_shared<ctrl::CLFController>(
            [](const Eigen::VectorXd &e) { return 0.5 * e(0) * e(0); },   // V(e)
            [](const Eigen::VectorXd &e) { return -kA * e(0) * e(0); },   // LfV
            [](const Eigen::VectorXd &e) { return e(0) * kBNom; },        // LgV
            cp, kTs);
        nominal = std::make_shared<BiasedController>(clf, kA * spec.ref / kBNom, kTs);
    }

    // -- safety layer --------------------------------------------------------------------
    std::shared_ptr<ctrl::CBFSafetyFilter> cbf;
    std::shared_ptr<ctrl::IController>     applied = nominal;

    if (spec.use_cbf) {
        ctrl::CBFSafetyFilter::Params bp;
        bp.alpha = kAlpha;
        bp.uMin = kUMin; bp.uMax = kUMax;
        cbf = std::make_shared<ctrl::CBFSafetyFilter>(
            nominal,
            [](double x) { return kXMax - x; },
            [](double)   { return -1.0; },
            // sigma_shared is captured BY REFERENCE: whoever writes it decides how honest
            // this drift term is. Arm 3 never writes it, arm 4 writes L1's (corrupted)
            // estimate, arm 5 writes the KF's.
            [&sigma_shared](double x) { return -kA * x + kBNom * sigma_shared; },
            [](double)   { return kBNom; },
            bp, kTs);
        applied = cbf;
    }

    // -- disturbance observer on the APPLIED command --------------------------------------
    Eigen::MatrixXd Q(2, 2), Rn(1, 1);
    Q << 1e-8, 0.0,
         0.0,  1e-4;
    Rn << 1e-6;
    ctrl::KalmanFilter kf(augmentedModel(), Q, Rn,
                          Eigen::MatrixXd::Identity(2, 2) * 0.1);

    // -- health layer: SPC, live only after commissioning ---------------------------------
    ctrl::ControllerMonitor monitor;
    monitor.setSigma(0.05);
    monitor.setCUSUMParams(0.5, 5.0);
    monitor.setAlarmCallback([&r](std::string_view, double) { ++r.alarms; });
    bool monitor_attached = false;

    // -- simulate ------------------------------------------------------------------------
    double x = 0.0, u_prev = 0.0;
    for (int k = 0; k < kN; ++k) {
        const double t = k * kTs;
        const double d = (spec.load && t >= kLoadTime) ? kLoad : 0.0;

        if (l1)  l1->setReference(spec.ref);
        if (clf) { Eigen::VectorXd e(1); e(0) = x - spec.ref; clf->setState(e); }
        if (cbf) cbf->setState(x);

        const double u = applied->compute(x);

        // Commission the SPC charts once the loop has settled, targeting the settled command.
        // Charting the startup transient alarms on motion, not on faults (ex132).
        if (!monitor_attached && t >= kCommissionTime) {
            monitor.setTarget(u);
            applied->attachObserver(&monitor);
            monitor_attached = true;
        }

        // Refresh the drift correction for the NEXT tick.
        if (spec.sigma_src == Sigma::FromL1 && l1) {
            sigma_shared = l1->estimatedDisturbance();
        } else if (spec.sigma_src == Sigma::FromKF) {
            Eigen::VectorXd yv(1), up(1), uc(1);
            yv(0) = x; up(0) = u_prev; uc(0) = u;
            kf.step(yv, up, uc);
            sigma_shared = kf.state()(1);
        }

        x += kTs * (-kA * x + kBTrue * u + d);
        if (!std::isfinite(x) || !std::isfinite(u)) { r.finite = false; return r; }

        r.max_x = std::max(r.max_x, x);
        r.iae  += std::abs(spec.ref - x) * kTs;
        if (cbf && cbf->cbfActive()) ++r.interventions;

        // The matched uncertainty the filter WOULD have needed, given what was applied.
        r.sigma_true = (kBTrue - kBNom) / kBNom * u + d / kBNom;
        u_prev = u;
    }

    r.violation  = std::max(0.0, r.max_x - kXMax);
    r.sigma_used = sigma_shared;
    return r;
}

}  // namespace

int main()
{
    std::printf("=== ex138: a safety filter is only as good as WHO estimated its model ===\n\n");
    std::printf("plant    : xdot = -%.2f x + %.2f u + d(t)   (the CBF is told b = %.2f - the\n",
                kA, kBTrue, kBNom);
    std::printf("           real input gain is %.0f%% higher than the filter believes)\n",
                100.0 * (kBTrue / kBNom - 1.0));
    std::printf("safe set : h(x) = %.2f - x >= 0,  CBF alpha = %.1f\n", kXMax, kAlpha);
    std::printf("setpoint : %.2f, deliberately ABOVE the limit, so safety and tracking\n", kRef);
    std::printf("           genuinely conflict and the filter has to do real work\n");
    std::printf("load     : +%.2f matched disturbance at t = %.0f s\n\n", kLoad, kLoadTime);

    const ArmSpec specs[] = {
        {"1. CLF only (no filter)",        Nominal::CLF, false, Sigma::None},
        {"2. L1 only (no filter)",         Nominal::L1,  false, Sigma::None},
        {"3. L1 + CBF, nominal f0",        Nominal::L1,  true,  Sigma::None},
        {"4. L1 + CBF fed L1's sigma",     Nominal::L1,  true,  Sigma::FromL1},
        {"5. L1 + CBF fed a KF sigma",     Nominal::L1,  true,  Sigma::FromKF},
    };

    ArmResult res[5];
    for (int i = 0; i < 5; ++i) res[i] = runArm(specs[i]);

    std::printf("  %-30s %8s %10s %9s %8s %7s\n",
                "arm", "max x", "violation", "IAE", "CBF hits", "alarms");
    for (int i = 0; i < 5; ++i)
        std::printf("  %-30s %8.4f %10.4f %9.3f %8d %7d%s\n",
                    specs[i].name, res[i].max_x, res[i].violation, res[i].iae,
                    res[i].interventions, res[i].alarms,
                    res[i].violation > 0.05 ? "   <-- UNSAFE" : "");

    // -- the estimate, and why arm 4 fails --------------------------------------------------
    std::printf("\n  drift correction actually supplied to the safety filter:\n");
    std::printf("    %-24s %12s %12s %10s\n", "arm", "sigma used", "sigma TRUE", "error");
    for (int i = 3; i <= 4; ++i)
        std::printf("    %-24s %12.4f %12.4f %9.1fx\n",
                    specs[i].name + 3, res[i].sigma_used, res[i].sigma_true,
                    res[i].sigma_used != 0.0 ? res[i].sigma_true / res[i].sigma_used : 0.0);
    std::printf("\n  L1 advances its state predictor with the command IT computed. The CBF\n");
    std::printf("  rewrites that command downstream, and L1 is never told - so its sigma_hat\n");
    std::printf("  absorbs the filter's own edits and stops being a plant measurement. The KF\n");
    std::printf("  is driven by step(y, u_prev, u_applied) and therefore stays exact.\n");

    // -- control condition: is the filter inert when nothing threatens the barrier? ---------
    const ArmSpec quiet{"6. same stack, safe setpoint", Nominal::L1, true, Sigma::FromKF,
                        kRefSafe, /*load=*/false};
    const ArmResult qr = runArm(quiet);
    std::printf("\n  control condition - arm 5's stack, setpoint %.2f, no load:\n", kRefSafe);
    std::printf("    max x %.4f, %d CBF interventions, %d alarms\n",
                qr.max_x, qr.interventions, qr.alarms);

    // -- acceptance ---------------------------------------------------------------------------
    // Two-sided throughout. Arm 4 MUST fail: a demo where "adaptation present + filter present"
    // already sufficed would not be testing the input-consistency claim, which is the point.
    const bool all_finite = res[0].finite && res[1].finite && res[2].finite &&
                            res[3].finite && res[4].finite && qr.finite;

    const bool unfiltered_unsafe = res[0].violation > 0.05 && res[1].violation > 0.05;
    const bool nominal_cbf_unsafe = res[2].violation > 0.05;
    const bool l1_sigma_no_help   = res[3].violation > 0.05;
    // L1's estimate must be demonstrably wrong, not merely unhelpful.
    const bool l1_sigma_corrupted = std::abs(res[3].sigma_true - res[3].sigma_used) >
                                    0.5 * std::abs(res[3].sigma_true);
    // The KF's estimate must be right, and the filter must then hold the barrier. A discrete
    // CBF enforced at sample instants still permits a bounded one-step excursion, so the claim
    // is an order-of-magnitude reduction rather than zero.
    const bool kf_sigma_exact = std::abs(res[4].sigma_true - res[4].sigma_used) < 0.02;
    const bool kf_cbf_safe    = res[4].violation < 0.05 &&
                                res[4].violation < 0.15 * res[2].violation;
    const bool not_conservative = res[4].max_x > 0.95 * kXMax;
    const bool filter_worked    = res[4].interventions > 0;
    const bool monitor_discriminates = res[4].alarms > 0 && qr.alarms == 0;
    const bool quiet_inert      = qr.violation <= 1e-9 && qr.interventions == 0;

    std::printf("\n  no filter -> unsafe       = %s (violations %.4f, %.4f > 0.05)\n",
                unfiltered_unsafe ? "yes" : "no", res[0].violation, res[1].violation);
    std::printf("  CBF on nominal f0         = %s STILL UNSAFE (violation %.4f)\n",
                nominal_cbf_unsafe ? "yes," : "no -", res[2].violation);
    std::printf("  L1's own sigma does NOT help = %s (violation %.4f, no better than arm 3)\n",
                l1_sigma_no_help ? "correct" : "WRONG", res[3].violation);
    std::printf("  ...because it is corrupted= %s (reported %.4f vs true %.4f)\n",
                l1_sigma_corrupted ? "yes" : "no", res[3].sigma_used, res[3].sigma_true);
    std::printf("  KF sigma is exact         = %s (|%.4f - %.4f| < 0.02)\n",
                kf_sigma_exact ? "yes" : "no", res[4].sigma_used, res[4].sigma_true);
    std::printf("  ...and the barrier holds  = %s (violation %.4f < 15%% of arm 3's %.4f)\n",
                kf_cbf_safe ? "yes" : "no", res[4].violation, res[2].violation);
    std::printf("  ...not by being timid     = %s (max x %.4f still rides the %.2f barrier)\n",
                not_conservative ? "yes" : "no", res[4].max_x, kXMax);
    std::printf("  filter actually acted     = %s (%d interventions)\n",
                filter_worked ? "yes" : "no", res[4].interventions);
    std::printf("  monitor discriminates     = %s (%d alarms load-bearing, %d idle)\n",
                monitor_discriminates ? "yes" : "no", res[4].alarms, qr.alarms);
    std::printf("  filter inert when safe    = %s (%d interventions at setpoint %.2f)\n",
                quiet_inert ? "yes" : "no", qr.interventions, kRefSafe);

    const bool ok = all_finite && unfiltered_unsafe && nominal_cbf_unsafe && l1_sigma_no_help &&
                    l1_sigma_corrupted && kf_sigma_exact && kf_cbf_safe && not_conservative &&
                    filter_worked && monitor_discriminates && quiet_inert;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
