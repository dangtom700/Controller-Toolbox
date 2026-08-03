// ex139_robustness_driven_selection.cpp - Picking a controller on evidence, not on nominal IAE.
//
// Fusion: ctrl::nuGap + ctrl::robustStabilityRadius (MuAnalysis) + ctrl::findWorstCaseIAE
//         (WorstCaseSearch) + ctrl::isQuadraticallyStable (LyapunovRobustness)
//
// docs/fusion_opportunity_backlog.md item B4. All four of these have solo examples - ex60, ex85,
// ex86, ex87 - and each answers a different question about one controller. What has never
// existed is the thing an engineer actually wants: a repeatable PROCEDURE that takes a roster of
// candidate tunings and an uncertainty set, scores every candidate on every axis, and returns a
// defensible winner.
//
// The scoring axes are deliberately not interchangeable. Each one can pass while another fails:
//
//   nuGap                  How far the uncertainty set stretches the PLANT. A property of the
//                          set alone - it does not mention the controller, and it bounds how
//                          much robustness any candidate needs before you score a single one.
//   robustStabilityRadius  Largest multiplicative output uncertainty the loop survives, from
//                          the mu upper bound. Frequency-domain, stability only, says nothing
//                          about performance.
//   findWorstCaseIAE       CMA-ES hunt for the parameter vector inside the box that hurts
//                          tracking most. Performance, but only at the single worst point it
//                          manages to find - a search, not a proof.
//   isQuadraticallyStable  A common quadratic Lyapunov function across the box vertices. The
//                          only axis here that certifies stability under TIME-VARYING parameter
//                          variation rather than fixed-but-unknown; correspondingly the hardest
//                          to pass, and a proof rather than a search.
//
// Selection rule used below: hard-gate on a stability radius exceeding the set's own nu-gap
// radius AND a finite worst-case IAE, then rank the survivors by worst-case IAE. Nominal IAE is
// reported but never used to select - it is the number this whole procedure exists to distrust.
//
// FINDING - why isQuadraticallyStable is reported but is NOT part of the gate. The
// implementation is explicitly a heuristic, not an SDP: it SUMS the per-vertex Lyapunov
// solutions and projects onto the PD cone, and its own header warns it "works well for vertices
// clustered around a common nominal". Every closed loop here contains an integrator, so it has
// a pole near z = 1, so its per-vertex Lyapunov matrices are enormous - and the sum's cross
// terms then swamp the decrease condition. Swept across sample rates from Ts = 0.05 to 0.50 and
// box widths down to +-10 %, it certified NO integral-action candidate, even ones with a
// closed-loop spectral radius of 0.80. The sanity check at the end of this demo shows the same
// call succeeding on a well-damped, tightly-clustered vertex set, so the tool is not broken -
// this problem class is simply outside what a summing heuristic can certify. Using it as a gate
// would reject every candidate and make the procedure useless, so it is reported as evidence
// and the gate rests on the two axes that discriminate.
//
// SCOPE, and it is a real limit rather than an implementation gap: every one of these four
// tools is LTI. findWorstCaseIAE and robustStabilityRadius take the controller as a StateSpace;
// isQuadraticallyStable needs closed-loop vertex A-matrices. A DiscreteSMC, DiscreteMPC,
// FuzzyPID or the nonlinear part of DiscreteADRC cannot be handed to any of them. This
// procedure ranks LTI candidates against parametric uncertainty and nothing else - which is why
// the roster below is five tunings of the same LTI family rather than a cross-family bake-off.

#include <ControllerToolbox.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr double kTs = 0.05;

// -- Plant: gain K, dominant lag tau, plus two fixed faster lags --------------------------
// y/u DC gain is exactly K. Three lags rather than two is deliberate: with only two, an
// over-aggressive tuning never actually got into trouble anywhere in the box, so the roster's
// nominal-best candidate was also its robust-best and the whole procedure had nothing to
// disagree with. Enough accumulated phase is what makes high gain genuinely dangerous.
ctrl::StateSpace plantOf(const Eigen::VectorXd &p)
{
    const double K   = p(0);
    const double tau = std::max(p(1), 1e-3);

    const double a1 = std::exp(-kTs / tau),          b1 = K * (1.0 - a1);
    const double a2 = std::exp(-kTs / (0.25 * tau)), b2 = 1.0 - a2;
    const double a3 = std::exp(-kTs / (0.15 * tau)), b3 = 1.0 - a3;

    Eigen::MatrixXd A(3, 3), B(3, 1), C(1, 3), D(1, 1);
    A << a1,  0.0, 0.0,
         b2,  a2,  0.0,
         0.0, b3,  a3;
    B << b1, 0.0, 0.0;
    C << 0.0, 0.0, 1.0;
    D << 0.0;
    return ctrl::StateSpace(A, B, C, D, kTs);
}

// Uncertainty box: gain from half nominal to 2.5x nominal, dominant lag +-40 %.
const Eigen::Vector2d kNominal(2.00, 1.00);
const Eigen::Vector2d kLower  (1.00, 0.60);
const Eigen::Vector2d kUpper  (5.00, 1.60);
// findWorstCaseIAE works in relative units: search_width(i) = sigma(i)*max(|nominal(i)|,1).
const Eigen::Vector2d kSigma  (1.00, 0.50);

// -- Candidate roster: five LTI tunings, expressed directly as z^-1 transfer functions ----
// Written as transfer functions rather than lifted out of DiscretePID because the entire
// analysis stack consumes StateSpace - see the scope note in the file header.
ctrl::TransferFunction piTF(double Kp, double Ki)
{   // backward-Euler PI:  (Kp + Ki*Ts - Kp z^-1) / (1 - z^-1)
    return ctrl::TransferFunction({Kp + Ki * kTs, -Kp}, {1.0, -1.0}, kTs);
}

ctrl::TransferFunction pidTF(double Kp, double Ki, double Kd)
{   // backward-Euler PID over the common denominator (1 - z^-1)
    const double d = Kd / kTs;
    return ctrl::TransferFunction({Kp + Ki * kTs + d, -Kp - 2.0 * d, d}, {1.0, -1.0}, kTs);
}

struct Candidate {
    const char            *name;
    ctrl::TransferFunction tf;
};

struct Score {
    double nominal_iae   = 0.0;
    double worst_iae     = 0.0;
    Eigen::Vector2d worst_at{0.0, 0.0};
    double stab_radius   = 0.0;
    bool   quad_stable   = false;
    double lyap_residual = 0.0;
    bool   passes_gate   = false;
};

/// Closed-loop A matrix with plant `p` and controller `K_ss`, for the Lyapunov vertex test.
Eigen::MatrixXd closedLoopA(const Eigen::VectorXd &p, const ctrl::StateSpace &K_ss)
{
    const ctrl::StateSpace L = ctrl::SystemAnalysis::series(K_ss, plantOf(p));
    return ctrl::SystemAnalysis::feedback(L).A;
}

}  // namespace

int main()
{
    std::printf("=== ex139: robustness-driven controller selection ===\n\n");
    std::printf("plant  : DC gain K, dominant lag tau, plus fixed lags at tau/4 and 0.15 tau,\n"
                "         Ts = %.2f s\n", kTs);
    std::printf("box    : K in [%.2f, %.2f] (nominal %.2f), tau in [%.2f, %.2f] (nominal %.2f)\n\n",
                kLower(0), kUpper(0), kNominal(0), kLower(1), kUpper(1), kNominal(1));

    const ctrl::StateSpace G_nom = plantOf(kNominal);

    // ---- Axis 0: how big is the uncertainty set, in plant terms? -------------------------
    // A property of the SET, computed before any candidate is scored. Vinnicombe's argument
    // makes this the yardstick every candidate's stability margin has to clear.
    std::vector<Eigen::Vector2d> corners;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            corners.emplace_back(i ? kUpper(0) : kLower(0), j ? kUpper(1) : kLower(1));

    double nugap_radius = 0.0;
    for (const auto &c : corners)
        nugap_radius = std::max(nugap_radius, ctrl::nuGap(G_nom, plantOf(c), 200));

    std::printf("  [set] nu-gap radius of the box (max over %zu corners) = %.4f\n",
                corners.size(), nugap_radius);
    std::printf("        any candidate whose stability margin is below this is not credible\n"
                "        across the set, whatever its nominal numbers say.\n\n");

    // ---- The roster ----------------------------------------------------------------------
    // Five tunings of the same LTI family (see the scope note in the header - a cross-family
    // roster is not scoreable by these tools). "PI aggressive" is the trap: it wins on nominal
    // IAE and goes unstable inside the box.
    const std::vector<Candidate> roster = {
        {"PI conservative",  piTF(0.10, 0.30)},
        {"PI moderate",      piTF(0.25, 0.30)},
        {"PI balanced",      piTF(0.50, 0.30)},
        {"PI aggressive",    piTF(1.20, 1.00)},
        {"PI very aggr.",    piTF(1.60, 1.60)},
    };

    ctrl::UncertaintyStructure struc;
    struc.blocks.push_back({ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1});

    ctrl::WorstCaseSearchParams wcp;
    wcp.max_evals  = 240;      // enough for a 2-parameter box; keeps run.py brisk
    wcp.sigma_init = 0.35;
    wcp.seed       = 20260802u;

    std::vector<Score> scores(roster.size());

    for (std::size_t i = 0; i < roster.size(); ++i) {
        const ctrl::StateSpace K_ss = ctrl::tf2ss(roster[i].tf);
        Score &s = scores[i];

        // Axis 1 - nominal performance. Reported, never used to select.
        s.nominal_iae = ctrl::evaluateSample(0, G_nom, K_ss, G_nom, 1.0, 40.0).iae;

        // Axis 2 - frequency-domain stability margin under multiplicative output uncertainty.
        s.stab_radius = ctrl::robustStabilityRadius(G_nom, K_ss, struc, 2.0, 30);

        // Axis 3 - CMA-ES hunt for the worst point in the box.
        const ctrl::WorstCaseResult w =
            ctrl::findWorstCaseIAE(plantOf, K_ss, kNominal, kSigma, kLower, kUpper, 40.0, wcp);
        s.worst_iae = w.worst_cost;
        s.worst_at  = w.worst_params.head<2>();

        // Axis 4 - common quadratic Lyapunov function across the closed-loop box vertices.
        std::vector<Eigen::MatrixXd> verts;
        verts.reserve(corners.size());
        for (const auto &c : corners) verts.push_back(closedLoopA(c, K_ss));
        s.quad_stable   = ctrl::isQuadraticallyStable(verts);
        s.lyap_residual = ctrl::findCommonLyapunov(verts).residual;

        // Selection gate. Quadratic stability is deliberately NOT part of it - see the
        // FINDING in the file header.
        s.passes_gate = s.stab_radius > nugap_radius && std::isfinite(s.worst_iae);
    }

    // ---- Report --------------------------------------------------------------------------
    std::printf("  %-18s %11s %11s %10s %8s %8s\n",
                "candidate", "nominal IAE", "worst IAE", "stab.rad", "quad.st", "gate");
    for (std::size_t i = 0; i < roster.size(); ++i) {
        const Score &s = scores[i];
        char worst[24];
        if (std::isfinite(s.worst_iae)) std::snprintf(worst, sizeof worst, "%11.3f", s.worst_iae);
        else                            std::snprintf(worst, sizeof worst, "%11s", "unstable");
        std::printf("  %-18s %11.3f %s %10.3f %8s %8s\n",
                    roster[i].name, s.nominal_iae, worst, s.stab_radius,
                    s.quad_stable ? "yes" : "NO", s.passes_gate ? "pass" : "REJECT");
    }

    // -- is the Lyapunov axis broken, or is this problem class simply out of its reach? -----
    // Same call, on a well-damped tightly-clustered vertex set with no integrator. If this
    // certifies, the NOs above are a statement about integral-action loops, not a bug.
    std::vector<Eigen::MatrixXd> sane;
    for (double a : {0.40, 0.50, 0.60}) {
        Eigen::MatrixXd A(2, 2);
        A << a, 0.10, 0.0, 0.8 * a;
        sane.push_back(A);
    }
    const bool sanity_quad = ctrl::isQuadraticallyStable(sane);
    const double sanity_res = ctrl::findCommonLyapunov(sane).residual;
    std::printf("\n  Lyapunov axis certified %d of %zu candidates. Control check on a\n",
                [&] { int n = 0; for (const Score &s : scores) n += s.quad_stable; return n; }(),
                roster.size());
    std::printf("    well-damped, no-integrator vertex set: quad = %s (residual %+.4f)\n",
                sanity_quad ? "YES" : "no", sanity_res);
    std::printf("    -> the summing heuristic works; loops with a pole near z = 1 are simply\n");
    std::printf("       outside what it can certify. Reported as evidence, not used as a gate.\n");

    std::printf("\n  worst-case parameter vector found per candidate (K, tau):\n");
    for (std::size_t i = 0; i < roster.size(); ++i)
        std::printf("    %-18s K = %.3f, tau = %.3f   (Lyapunov residual %+.4f)\n",
                    roster[i].name, scores[i].worst_at(0), scores[i].worst_at(1),
                    scores[i].lyap_residual);

    // ---- Rank by each axis independently, to show they are not one measure ---------------
    auto rank_by = [&](auto key, bool ascending) {
        std::vector<std::size_t> idx(roster.size());
        for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
        std::stable_sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
            return ascending ? key(scores[a]) < key(scores[b]) : key(scores[a]) > key(scores[b]);
        });
        return idx;
    };
    const auto by_nominal = rank_by([](const Score &s) { return s.nominal_iae; }, true);
    const auto by_worst   = rank_by([](const Score &s) { return s.worst_iae;   }, true);
    const auto by_radius  = rank_by([](const Score &s) { return s.stab_radius; }, false);

    auto print_rank = [&](const char *label, const std::vector<std::size_t> &idx) {
        std::printf("    %-22s", label);
        for (std::size_t i : idx) std::printf(" %s >", roster[i].name);
        std::printf("\b \n");
    };
    std::printf("\n  ranking by each axis (best first):\n");
    print_rank("nominal IAE", by_nominal);
    print_rank("worst-case IAE", by_worst);
    print_rank("stability radius", by_radius);

    // ---- The decision ---------------------------------------------------------------------
    std::size_t winner = roster.size();
    for (std::size_t i : by_worst)
        if (scores[i].passes_gate) { winner = i; break; }

    const std::size_t nominal_best = by_nominal.front();

    std::printf("\n  nominal-IAE winner : %s  (IAE %.3f)\n",
                roster[nominal_best].name, scores[nominal_best].nominal_iae);
    if (winner < roster.size())
        std::printf("  SELECTED           : %s  (worst-case IAE %.3f finite over the whole box,\n"
                    "                       stability radius %.3f > set nu-gap radius %.4f)\n",
                    roster[winner].name, scores[winner].worst_iae,
                    scores[winner].stab_radius, nugap_radius);
    else
        std::printf("  SELECTED           : none - no candidate cleared the gate\n");

    // ---- acceptance -------------------------------------------------------------------------
    // The procedure is only worth running if it can DISAGREE with the cheap answer, so the
    // first assertion is that the roster actually contains that disagreement. A roster where
    // nominal IAE already picks the robust winner would let this demo pass while proving
    // nothing - the same trap ex136's first revision fell into.
    const bool have_winner   = winner < roster.size();
    const bool axes_disagree = have_winner && winner != nominal_best;
    const bool nominal_fails = !scores[nominal_best].passes_gate;
    const bool winner_certified = have_winner &&
                                  std::isfinite(scores[winner].worst_iae) &&
                                  scores[winner].stab_radius > nugap_radius;
    // The Lyapunov axis must be shown to be capable, or reporting its NOs proves nothing.
    const bool lyap_axis_sane = sanity_quad && sanity_res < 0.0;
    // The gate must discriminate: if everything passes, the gate is decoration.
    int n_pass = 0;
    for (const Score &s : scores) n_pass += s.passes_gate ? 1 : 0;
    const bool gate_discriminates = n_pass > 0 && n_pass < static_cast<int>(roster.size());
    // The axes must be non-redundant, or one of them would do.
    const bool axes_nonredundant = (by_nominal != by_worst) || (by_worst != by_radius);
    const bool set_measured = nugap_radius > 1e-3 && nugap_radius < 1.0;

    std::printf("\n  uncertainty set measured  = %s (nu-gap radius %.4f in (0, 1))\n",
                set_measured ? "yes" : "no", nugap_radius);
    std::printf("  gate discriminates        = %s (%d of %zu candidates cleared it)\n",
                gate_discriminates ? "yes" : "no", n_pass, roster.size());
    std::printf("  axes are non-redundant    = %s (the three rankings are not all identical)\n",
                axes_nonredundant ? "yes" : "no");
    std::printf("  robust winner != nominal  = %s (%s vs %s)\n",
                axes_disagree ? "yes" : "no",
                have_winner ? roster[winner].name : "none", roster[nominal_best].name);
    std::printf("  nominal winner is rejected= %s (it fails at least one robustness gate)\n",
                nominal_fails ? "yes" : "no");
    std::printf("  selection is certified    = %s (finite worst case, margin %.3f > set"
                " radius %.4f)\n", winner_certified ? "yes" : "no",
                have_winner ? scores[winner].stab_radius : 0.0, nugap_radius);

    std::printf("  Lyapunov axis is capable  = %s (certifies the control vertex set,"
                " residual %+.4f)\n", lyap_axis_sane ? "yes" : "no", sanity_res);

    const bool ok = set_measured && gate_discriminates && axes_nonredundant &&
                    axes_disagree && nominal_fails && winner_certified && lyap_axis_sane;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
