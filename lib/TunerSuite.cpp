#include "TunerSuite.h"
#include "IController.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <memory>

namespace ctrl
{

    // ============================================================
    //  Compatibility lookup tables
    //
    //  Each tuning method has three tiers for each CtrlKind:
    //    IDEAL    - no warning, full support
    //    SOFT     - warning + proceed (success=true, warned=true)
    //    FALLBACK - warning + default params (success=false, warned=true)
    //
    //  These tiers match the cheatsheet's "Applications" sections.
    // ============================================================

    // Compact tier enum used only inside this translation unit.
    enum class Tier
    {
        IDEAL,
        SOFT,
        FALLBACK
    };

    // Return the tier for a given (tuner, controller) pair.
    // Tuner names match the cheatsheet sections.
    static Tier tier(const char *tuner, CtrlKind k)
    {
        // ---------- Relay / ZN (cheatsheet Section 1) ----------
        if (!strcmp(tuner, "RelayZN"))
        {
            switch (k)
            {
            case CtrlKind::PID:
                return Tier::IDEAL;
            case CtrlKind::Smith:
                return Tier::SOFT; // tune the inner PID
            case CtrlKind::LeadLag:
                return Tier::SOFT; // Ku/Tu can give a gain hint
            default:
                return Tier::FALLBACK;
            }
        }
        // ---------- IMC-PID (cheatsheet Section 2) ----------
        if (!strcmp(tuner, "IMCPID"))
        {
            switch (k)
            {
            case CtrlKind::PID:
                return Tier::IDEAL;
            case CtrlKind::Smith:
                return Tier::SOFT;
            case CtrlKind::LeadLag:
                return Tier::SOFT; // Kp -> gain K hint
            default:
                return Tier::FALLBACK;
            }
        }
        // ---------- Cohen-Coon (cheatsheet Section 2 variant) ----------
        if (!strcmp(tuner, "CohenCoon"))
        {
            switch (k)
            {
            case CtrlKind::PID:
                return Tier::IDEAL;
            case CtrlKind::Smith:
                return Tier::SOFT;
            default:
                return Tier::FALLBACK;
            }
        }
        // ---------- Bryson's Rule (cheatsheet Section 3) ----------
        if (!strcmp(tuner, "Bryson"))
        {
            switch (k)
            {
            case CtrlKind::LQR:
                return Tier::IDEAL;
            case CtrlKind::LQG:
                return Tier::IDEAL;
            case CtrlKind::MPC:
                return Tier::SOFT; // rho_y/rho_u analogy
            case CtrlKind::SMC:
                return Tier::SOFT; // Q ratio -> c_e/c_de hint
            default:
                return Tier::FALLBACK;
            }
        }
        // ---------- Kalman Noise (cheatsheet Section 4 observer part) ----------
        if (!strcmp(tuner, "KalmanNoise"))
        {
            switch (k)
            {
            case CtrlKind::LQG:
                return Tier::IDEAL;
            case CtrlKind::Generic:
                return Tier::SOFT; // standalone KalmanFilter
            default:
                return Tier::FALLBACK;
            }
        }
        // ---------- MPC Horizon (cheatsheet Section 5) ----------
        if (!strcmp(tuner, "MPCHorizon"))
        {
            switch (k)
            {
            case CtrlKind::MPC:
                return Tier::IDEAL;
            case CtrlKind::LQR:
                return Tier::SOFT; // settling-time info is still useful
            default:
                return Tier::FALLBACK;
            }
        }
        // ---------- Loop Shaping (cheatsheet Section 6) ----------
        if (!strcmp(tuner, "LoopShaping"))
        {
            switch (k)
            {
            case CtrlKind::LeadLag:
                return Tier::IDEAL;
            case CtrlKind::PID:
                return Tier::SOFT; // omega_c -> bandwidth / N hint
            default:
                return Tier::FALLBACK;
            }
        }
        // ---------- Optimisation-based (cheatsheet Section 7) ----------
        // Always IDEAL: the method is controller-agnostic.
        if (!strcmp(tuner, "Optimise"))
            return Tier::IDEAL;

        return Tier::FALLBACK;
    }

    // ============================================================
    //  ctrlKindName
    // ============================================================
    const char *ctrlKindName(CtrlKind k) noexcept
    {
        switch (k)
        {
        case CtrlKind::PID:
            return "DiscretePID";
        case CtrlKind::LQR:
            return "DiscreteLQR";
        case CtrlKind::LQG:
            return "DiscreteLQG";
        case CtrlKind::MPC:
            return "DiscreteMPC";
        case CtrlKind::LeadLag:
            return "DiscreteLeadLag";
        case CtrlKind::SMC:
            return "DiscreteSMC";
        case CtrlKind::ADRC:
            return "DiscreteADRC";
        case CtrlKind::ESC:
            return "ExtremumSeeker";
        case CtrlKind::Smith:
            return "SmithPredictor";
        case CtrlKind::Generic:
            return "IController (generic)";
        }
        return "Unknown";
    }

    // ============================================================
    //  softWarn  - emit to std::clog and store in base
    // ============================================================
    void TunerSuite::softWarn(TuningResultBase &base, const std::string &msg)
    {
        base.warned = true;
        base.warning = msg;
        std::clog << "[TunerSuite] WARNING: " << msg << "\n";
    }

    // ============================================================
    //  Helper: build a soft or fallback warning message.
    // ============================================================
    static std::string buildWarnMsg(const char *tunerName,
                                    CtrlKind target,
                                    const char *dedicated,
                                    const char *suggestion)
    {
        std::ostringstream os;
        os << tunerName << " is dedicated to " << dedicated << ".\n"
           << "  Applying it to " << ctrlKindName(target)
           << " may give suboptimal or meaningless results.\n";
        if (suggestion && suggestion[0])
            os << "  Suggested tuner for " << ctrlKindName(target) << ": " << suggestion;
        return os.str();
    }

    // Suggestion strings per controller type, used in fallback messages.
    static const char *suggestFor(CtrlKind k)
    {
        switch (k)
        {
        case CtrlKind::PID:
            return "StepResponseTuner (IMC) or ZieglerNicholsTuner";
        case CtrlKind::LQR:
            return "LQRWeightTuner::brysonMethod(xmax, umax)";
        case CtrlKind::LQG:
            return "LQRWeightTuner::brysonMethod + KalmanWeightTuner::isotropic";
        case CtrlKind::MPC:
            return "MPCHorizonTuner::recommend(plant, Ts)";
        case CtrlKind::LeadLag:
            return "LoopShapingTuner::tuneImpl(omega_c, phase_add, gain)";
        case CtrlKind::SMC:
            return "set c_e, K, phi directly from sliding-surface theory";
        case CtrlKind::ADRC:
            return "set omega_c, omega_o, b0 via bandwidth parameterisation";
        case CtrlKind::ESC:
            return "set perturbFreq >> plant bandwidth; use TunerSuite::optimise";
        case CtrlKind::Smith:
            return "tune the inner DiscretePID, not SmithPredictor itself";
        case CtrlKind::Generic:
            return "TunerSuite::optimise() with a simulation-based cost function";
        }
        return "";
    }

    // ============================================================
    //  1. RelayZN
    // ============================================================
    PIDTuneResult TunerSuite::relayZN(CtrlKind target,
                                      const RelayAutoTuner &relay,
                                      PIDTuningRule rule,
                                      double lambda)
    {
        PIDTuneResult res;
        Tier t = tier("RelayZN", target);

        switch (t)
        {
        case Tier::IDEAL:
            break;

        case Tier::SOFT:
            softWarn(res, buildWarnMsg(
                              "RelayZN", target, "DiscretePID",
                              target == CtrlKind::Smith
                                  ? "tune the SmithPredictor's inner DiscretePID with the same call"
                                  : "use the returned PIDParams.Kp as a gain magnitude hint for DiscreteLeadLag"));
            break;

        case Tier::FALLBACK:
            softWarn(res, buildWarnMsg(
                              "RelayZN", target, "DiscretePID", suggestFor(target)));
            // Return default-constructed PIDParams so the caller can inspect Ku/Tu.
            res.params.Kp = relay.ultimateGain() * 0.6;
            res.params.Ki = 0.0;
            res.params.Kd = 0.0;
            res.success = false;
            return res;
        }

        res.params = relay.computePIDParams(rule, lambda);
        res.success = true;
        return res;
    }

    // ============================================================
    //  2. IMC-PID
    // ============================================================
    PIDTuneResult TunerSuite::imcPID(CtrlKind target,
                                     const StepResponseTuner::FOPDTModel &fopdt,
                                     double Ts,
                                     double lambda)
    {
        PIDTuneResult res;
        Tier t = tier("IMCPID", target);

        switch (t)
        {
        case Tier::IDEAL:
            break;

        case Tier::SOFT:
        {
            std::string extra;
            if (target == CtrlKind::Smith)
                extra = "Use the returned PIDParams for SmithPredictor's inner controller.";
            else if (target == CtrlKind::LeadLag)
                extra = "Use Kp as the gain K for DiscreteLeadLag; run LoopShapingTuner for z_c/p_c.";
            softWarn(res, buildWarnMsg("IMC-PID", target, "DiscretePID", extra.c_str()));
            break;
        }

        case Tier::FALLBACK:
            softWarn(res, buildWarnMsg("IMC-PID", target, "DiscretePID", suggestFor(target)));
            res.success = false;
            return res;
        }

        res.params = StepResponseTuner::computePIDParams(fopdt, Ts, PIDTuningRule::IMC, lambda);
        res.success = true;
        return res;
    }

    // ============================================================
    //  3. Cohen-Coon
    // ============================================================
    PIDTuneResult TunerSuite::cohenCoon(CtrlKind target,
                                        const StepResponseTuner::FOPDTModel &fopdt,
                                        double Ts)
    {
        PIDTuneResult res;
        Tier t = tier("CohenCoon", target);

        switch (t)
        {
        case Tier::IDEAL:
            break;

        case Tier::SOFT:
            softWarn(res, buildWarnMsg(
                              "Cohen-Coon", target, "DiscretePID",
                              "Use the returned PIDParams for SmithPredictor's inner controller."));
            break;

        case Tier::FALLBACK:
            softWarn(res, buildWarnMsg("Cohen-Coon", target, "DiscretePID", suggestFor(target)));
            res.success = false;
            return res;
        }

        // Cohen-Coon requires theta > 0; emit a secondary soft warning if near zero.
        if (fopdt.theta < 1e-6)
        {
            std::string msg =
                "Cohen-Coon: dead time theta approx = 0. Results may be inaccurate. "
                "Use IMC-PID (TunerSuite::imcPID) for near-zero dead time.";
            softWarn(res, msg);
            // Fall through - CohenCoonTuner will throw; we catch and return IMC instead.
            try
            {
                res.params = CohenCoonTuner::tuneImpl(fopdt, Ts);
            }
            catch (...)
            {
                res.params = StepResponseTuner::computePIDParams(fopdt, Ts, PIDTuningRule::IMC);
            }
            res.success = true;
            return res;
        }

        res.params = CohenCoonTuner::tuneImpl(fopdt, Ts);
        res.success = true;
        return res;
    }

    // ============================================================
    //  4. Bryson's Rule
    // ============================================================
    LQRTuneResult TunerSuite::bryson(CtrlKind target,
                                     const Eigen::VectorXd &xmax,
                                     const Eigen::VectorXd &umax)
    {
        LQRTuneResult res;
        Tier t = tier("Bryson", target);

        switch (t)
        {
        case Tier::IDEAL:
            break;

        case Tier::SOFT:
        {
            std::string extra;
            if (target == CtrlKind::MPC)
                extra = "For MPC: set rho_y = 1/xmax^2, rho_u = 1/umax^2 as weight scalars. "
                        "Use MPCHorizonTuner for Np and Nc.";
            else if (target == CtrlKind::SMC)
                extra = "For SMC: use sqrt(Q[0,0]/Q[1,1]) as a c_e/c_de ratio hint. "
                        "Set K directly from disturbance bounds.";
            softWarn(res, buildWarnMsg("Bryson's Rule", target, "DiscreteLQR / DiscreteLQG",
                                       extra.c_str()));
            break;
        }

        case Tier::FALLBACK:
            softWarn(res, buildWarnMsg("Bryson's Rule", target,
                                       "DiscreteLQR / DiscreteLQG", suggestFor(target)));
            res.success = false;
            return res;
        }

        res.params = LQRWeightTuner::brysonMethod(xmax, umax);
        res.success = true;
        return res;
    }

    // ============================================================
    //  5. Kalman Noise
    // ============================================================
    KalmanTuneResult TunerSuite::kalmanNoise(CtrlKind target,
                                             int nStates,
                                             int nOutputs,
                                             double sigmaProcess,
                                             double sigmaMeas)
    {
        KalmanTuneResult res;
        Tier t = tier("KalmanNoise", target);

        switch (t)
        {
        case Tier::IDEAL:
            break;

        case Tier::SOFT:
            softWarn(res,
                     "KalmanNoise is dedicated to DiscreteLQG.\n"
                     "  For a standalone KalmanFilter: pass the returned Qf/Rf directly "
                     "to KalmanFilter(plant, Qf, Rf, P0).");
            break;

        case Tier::FALLBACK:
            softWarn(res, buildWarnMsg("KalmanNoise", target, "DiscreteLQG", suggestFor(target)));
            res.success = false;
            return res;
        }

        res.params = KalmanWeightTuner::isotropic(nStates, nOutputs, sigmaProcess, sigmaMeas);
        res.success = true;
        return res;
    }

    // ============================================================
    //  6. MPC Horizon
    // ============================================================
    MPCTuneResult TunerSuite::mpcHorizon(CtrlKind target,
                                         const StateSpace &plant,
                                         double Ts,
                                         double rho_y,
                                         double rho_u)
    {
        MPCTuneResult res;
        Tier t = tier("MPCHorizon", target);

        switch (t)
        {
        case Tier::IDEAL:
            break;

        case Tier::SOFT:
            softWarn(res,
                     "MPCHorizonTuner is dedicated to DiscreteMPC.\n"
                     "  For DiscreteLQR: the estimated settling time and recommended Np "
                     "indicate the minimum horizon for prediction quality checks. "
                     "Use Bryson's Rule (TunerSuite::bryson) for the actual LQR Q/R.");
            break;

        case Tier::FALLBACK:
            softWarn(res, buildWarnMsg("MPCHorizonTuner", target, "DiscreteMPC", suggestFor(target)));
            res.success = false;
            return res;
        }

        res.params = MPCHorizonTuner::recommend(plant, Ts, rho_y, rho_u);
        res.success = true;
        return res;
    }

    // ============================================================
    //  7. Loop Shaping
    // ============================================================
    LeadLagTuneResult TunerSuite::loopShaping(CtrlKind target,
                                              const LoopShapingTuner::Input &in)
    {
        LeadLagTuneResult res;
        Tier t = tier("LoopShaping", target);

        switch (t)
        {
        case Tier::IDEAL:
            break;

        case Tier::SOFT:
            softWarn(res,
                     "LoopShapingTuner is dedicated to DiscreteLeadLag.\n"
                     "  For DiscretePID: use omega_c as the desired closed-loop bandwidth "
                     "to set N = omega_c and derive Kp from 1/gain_at_wc. "
                     "Then use IMC-PID (TunerSuite::imcPID) for Ki and Kd.");
            break;

        case Tier::FALLBACK:
            softWarn(res, buildWarnMsg("LoopShapingTuner", target, "DiscreteLeadLag",
                                       suggestFor(target)));
            res.success = false;
            return res;
        }

        res.params = LoopShapingTuner::tuneImpl(in);
        res.success = true;
        return res;
    }

    // ============================================================
    //  8. Optimisation-based Nelder-Mead  (cheatsheet Section 7)
    // ============================================================

    // ---- Nelder-Mead simplex minimiser ----------------------------------------
    // Standard algorithm:  Nelder & Mead (1965) as described in
    //   Lagarias et al. "Convergence Properties of the Nelder-Mead Simplex
    //   Method in Low Dimensions" (SIAM J. Optim. 1998).
    //
    // Parameters: alpha=1 (reflect), γ=2 (expand), ρ=0.5 (contract), sigma=0.5 (shrink).
    // Bounds are enforced by clamping candidates before each evaluation.
    // -------------------------------------------------------------------------
    std::vector<double> TunerSuite::nelderMead(
        std::function<double(const std::vector<double> &)> f,
        std::vector<double> x0,
        const std::vector<std::pair<double, double>> &bounds,
        int maxEvals,
        double tol,
        int *evalCount_out)
    {
        const int n = static_cast<int>(x0.size());
        if (n == 0)
            return x0;

        // Clamp helper
        auto clamp = [&](std::vector<double> v)
        {
            for (int i = 0; i < n; ++i)
                v[i] = std::max(bounds[i].first, std::min(bounds[i].second, v[i]));
            return v;
        };

        // Build initial simplex: x0 + n perturbed vertices.
        // Perturbation = 5 % of range (or 0.05 if degenerate).
        using Vertex = std::pair<std::vector<double>, double>;
        std::vector<Vertex> simplex;
        simplex.reserve(n + 1);

        simplex.push_back({clamp(x0), f(clamp(x0))});
        for (int i = 0; i < n; ++i)
        {
            auto xi = x0;
            double range = bounds[i].second - bounds[i].first;
            xi[i] += (range > 1e-12) ? 0.05 * range : 0.05;
            auto cxi = clamp(xi);
            simplex.push_back({cxi, f(cxi)});
        }

        int evals = n + 1;

        const double alpha = 1.0;
        const double gamma = 2.0;
        const double rho = 0.5;
        const double sigma = 0.5;

        auto centroid = [&](int exclude)
        {
            std::vector<double> c(n, 0.0);
            for (int i = 0; i <= n; ++i)
                if (i != exclude)
                    for (int j = 0; j < n; ++j)
                        c[j] += simplex[i].first[j];
            for (auto &v : c)
                v /= n;
            return c;
        };

        auto eval = [&](std::vector<double> v) -> Vertex
        {
            auto cv = clamp(v);
            return {cv, f(cv)};
        };

        while (evals < maxEvals)
        {
            // Sort: best first (smallest cost)
            std::sort(simplex.begin(), simplex.end(),
                      [](const Vertex &a, const Vertex &b)
                      { return a.second < b.second; });

            // Convergence check: simplex diameter
            double diam = 0.0;
            for (int i = 1; i <= n; ++i)
                for (int j = 0; j < n; ++j)
                    diam = std::max(diam, std::abs(simplex[i].first[j] - simplex[0].first[j]));
            if (diam < tol)
                break;

            auto xo = centroid(n);     // centroid of best n vertices
            auto [xr, fr] = eval([&] { // reflect
                std::vector<double> v(n);
                for (int j = 0; j < n; ++j)
                    v[j] = xo[j] + alpha * (xo[j] - simplex[n].first[j]);
                return v;
            }());
            ++evals;

            if (fr < simplex[0].second)
            {
                // Try expansion
                auto [xe, fe] = eval([&]
                                     {
                std::vector<double> v(n);
                for (int j=0;j<n;++j) v[j]=xo[j]+gamma*(xr[j]-xo[j]);
                return v; }());
                ++evals;
                simplex[n] = (fe < fr) ? Vertex{xe, fe} : Vertex{xr, fr};
            }
            else if (fr < simplex[n - 1].second)
            {
                simplex[n] = {xr, fr};
            }
            else
            {
                // Contraction
                bool outer = (fr < simplex[n].second);
                auto [xc, fc] = eval([&]
                                     {
                std::vector<double> v(n);
                const auto& xw = outer ? xr : simplex[n].first;
                for (int j=0;j<n;++j) v[j]=xo[j]+rho*(xw[j]-xo[j]);
                return v; }());
                ++evals;

                if (fc < (outer ? fr : simplex[n].second))
                {
                    simplex[n] = {xc, fc};
                }
                else
                {
                    // Shrink
                    auto &best = simplex[0].first;
                    for (int i = 1; i <= n && evals < maxEvals; ++i)
                    {
                        std::vector<double> v(n);
                        for (int j = 0; j < n; ++j)
                            v[j] = best[j] + sigma * (simplex[i].first[j] - best[j]);
                        simplex[i] = eval(v);
                        ++evals;
                    }
                }
            }
        }

        std::sort(simplex.begin(), simplex.end(),
                  [](const Vertex &a, const Vertex &b)
                  { return a.second < b.second; });

        if (evalCount_out)
            *evalCount_out = evals;
        return simplex[0].first;
    }

    // ---- Public optimise() wrapper --------------------------------------------
    OptimTuneResult TunerSuite::optimise(
        CtrlKind target,
        const std::vector<std::pair<double, double>> &paramBounds,
        std::function<double(const std::vector<double> &)> costFn,
        std::vector<double> x0,
        int maxEvals,
        double tol)
    {
        OptimTuneResult res;

        if (paramBounds.empty())
        {
            softWarn(res, "TunerSuite::optimise: paramBounds is empty - nothing to optimise.");
            res.success = false;
            return res;
        }

        if (!costFn)
        {
            softWarn(res, "TunerSuite::optimise: costFn is null - provide a simulation cost function.");
            res.success = false;
            return res;
        }

        const int n = static_cast<int>(paramBounds.size());

        // Default initial guess: midpoint of each bound
        if (x0.empty())
        {
            x0.resize(n);
            for (int i = 0; i < n; ++i)
                x0[i] = 0.5 * (paramBounds[i].first + paramBounds[i].second);
        }
        else if (static_cast<int>(x0.size()) != n)
        {
            softWarn(res, "TunerSuite::optimise: x0 size mismatch with paramBounds - using midpoints.");
            x0.resize(n);
            for (int i = 0; i < n; ++i)
                x0[i] = 0.5 * (paramBounds[i].first + paramBounds[i].second);
        }

        int evals = 0;
        auto best = nelderMead(costFn, x0, paramBounds, maxEvals, tol, &evals);

        res.bestParams = best;
        res.bestCost = costFn(best);
        res.evalCount = evals + 1;
        res.success = true;

        // All controller types are valid for optimisation - no warning needed.
        // Just record which target was requested (informational).
        (void)target;

        return res;
    }

    // ============================================================
    //  makeISECost / makeITAECost
    // ============================================================
    std::function<double(const std::vector<double> &)>
    TunerSuite::makeISECost(
        const StateSpace &plant,
        double ref,
        int N,
        std::function<std::unique_ptr<IController>(const std::vector<double> &)> factory)
    {
        return [plant, ref, N, factory](const std::vector<double> &params) -> double
        {
            std::unique_ptr<IController> ctrl;
            try
            {
                ctrl = factory(params);
            }
            catch (...)
            {
                return 1e30; // construction failed - reject this candidate
            }
            if (!ctrl)
                return 1e30;

            StateSpace p2 = plant; // local copy so state is independent
            Eigen::VectorXd x = Eigen::VectorXd::Zero(p2.stateSize());
            double y = 0.0;
            double ise = 0.0;
            const double Ts = p2.Ts;

            for (int k = 0; k < N; ++k)
            {
                double e = ref - y;
                double u;
                try
                {
                    u = ctrl->compute(e);
                }
                catch (...)
                {
                    return 1e30;
                }
                if (!std::isfinite(u))
                    return 1e30;

                Eigen::VectorXd uv(1);
                uv << u;
                Eigen::VectorXd yv = ssStep(p2, x, uv);
                y = yv(0);
                if (!std::isfinite(y))
                    return 1e30;

                ise += e * e * Ts;
            }
            return ise;
        };
    }

    std::function<double(const std::vector<double> &)>
    TunerSuite::makeITAECost(
        const StateSpace &plant,
        double ref,
        int N,
        std::function<std::unique_ptr<IController>(const std::vector<double> &)> factory)
    {
        return [plant, ref, N, factory](const std::vector<double> &params) -> double
        {
            std::unique_ptr<IController> ctrl;
            try
            {
                ctrl = factory(params);
            }
            catch (...)
            {
                return 1e30;
            }
            if (!ctrl)
                return 1e30;

            StateSpace p2 = plant;
            Eigen::VectorXd x = Eigen::VectorXd::Zero(p2.stateSize());
            double y = 0.0;
            double itae = 0.0;
            const double Ts = p2.Ts;

            for (int k = 0; k < N; ++k)
            {
                double t = k * Ts;
                double e = ref - y;
                double u;
                try
                {
                    u = ctrl->compute(e);
                }
                catch (...)
                {
                    return 1e30;
                }
                if (!std::isfinite(u))
                    return 1e30;

                Eigen::VectorXd uv(1);
                uv << u;
                Eigen::VectorXd yv = ssStep(p2, x, uv);
                y = yv(0);
                if (!std::isfinite(y))
                    return 1e30;

                itae += t * std::abs(e) * Ts;
            }
            return itae;
        };
    }

} // namespace ctrl
