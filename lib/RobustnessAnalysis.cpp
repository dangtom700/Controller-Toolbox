#include "RobustnessAnalysis.h"
#include "SystemAnalysis.h"
#include "GapMetric.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace ctrl
{
namespace
{
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    // Resolve a per-element relative sigma from a sigma vector that is either
    // length-1 (uniform) or full length (per-coefficient).
    double sigmaAt(const std::vector<double>& s, std::size_t i)
    {
        if (s.size() == 1) return s[0];
        return s[i];
    }

    // ----------------------------------------------------------------------
    // Closed-loop realisation of a unity-feedback loop (K acts on e = r - y).
    //
    //   S = (I + G K)^{-1}      maps r -> e   (shares A_cl, B_cl)
    //   T = G K (I + G K)^{-1}  maps r -> y   (= I - S)
    //
    // For plant G = (Ag,Bg,Cg,Dg) and controller K = (Ak,Bk,Ck,Dk):
    //   M   = I + Dk Dg            (m_g x m_g, invertible iff no algebraic loop)
    //   N   = (I + Dg Dk)^{-1}     (p_g x p_g)
    //   A_cl = [ Ag - Bg M^{-1} Dk Cg,   Bg M^{-1} Ck            ]
    //          [ -Bk N Cg,               Ak - Bk Dg M^{-1} Ck    ]
    //   B_cl = [ Bg M^{-1} Dk ; Bk N ]
    //   C_S  = [ -N Cg,  -Dg M^{-1} Ck ],   D_S = N
    //   C_T  = [  N Cg,   Dg M^{-1} Ck ],   D_T = Dg M^{-1} Dk
    // ----------------------------------------------------------------------
    struct ClosedLoop
    {
        StateSpace S; // r -> e
        StateSpace T; // r -> y
    };

    ClosedLoop buildClosedLoop(const StateSpace& G, const StateSpace& K)
    {
        const int ng = G.stateSize();
        const int nk = K.stateSize();
        const int mg = G.inputSize();   // plant inputs   = controller outputs
        const int pg = G.outputSize();  // plant outputs  = controller inputs (error dim)

        const Eigen::MatrixXd& Ag = G.A; const Eigen::MatrixXd& Bg = G.B;
        const Eigen::MatrixXd& Cg = G.C; const Eigen::MatrixXd& Dg = G.D;
        const Eigen::MatrixXd& Ak = K.A; const Eigen::MatrixXd& Bk = K.B;
        const Eigen::MatrixXd& Ck = K.C; const Eigen::MatrixXd& Dk = K.D;

        // Tiny regularisation guards a singular (I + Dk Dg) for pure-integrator
        // plants with a feedthrough controller (algebraic loop).
        const double eps = 1e-12;
        Eigen::MatrixXd M = Eigen::MatrixXd::Identity(mg, mg) + Dk * Dg
                          + eps * Eigen::MatrixXd::Identity(mg, mg);
        Eigen::MatrixXd Minv = M.inverse();
        Eigen::MatrixXd N = (Eigen::MatrixXd::Identity(pg, pg) + Dg * Dk
                          + eps * Eigen::MatrixXd::Identity(pg, pg)).inverse();

        const Eigen::MatrixXd BgMinv = Bg * Minv;       // ng x mg
        const Eigen::MatrixXd DgMinv = Dg * Minv;       // pg x mg

        const int n = ng + nk;
        Eigen::MatrixXd Acl(n, n);
        Acl.topLeftCorner(ng, ng)     = Ag - BgMinv * Dk * Cg;
        Acl.topRightCorner(ng, nk)    = BgMinv * Ck;
        Acl.bottomLeftCorner(nk, ng)  = -Bk * N * Cg;
        Acl.bottomRightCorner(nk, nk) = Ak - Bk * DgMinv * Ck;

        Eigen::MatrixXd Bcl(n, pg);
        Bcl.topRows(ng)    = BgMinv * Dk;   // ng x pg
        Bcl.bottomRows(nk) = Bk * N;        // nk x pg

        Eigen::MatrixXd Cs(pg, n), Ct(pg, n);
        Cs.leftCols(ng)  = -N * Cg;        Cs.rightCols(nk)  = -DgMinv * Ck;
        Ct.leftCols(ng)  =  N * Cg;        Ct.rightCols(nk)  =  DgMinv * Ck;

        Eigen::MatrixXd Ds = N;
        Eigen::MatrixXd Dt = DgMinv * Dk;

        return ClosedLoop{
            StateSpace(Acl, Bcl, Cs, Ds, G.Ts),
            StateSpace(Acl, Bcl, Ct, Dt, G.Ts)
        };
    }

    // Open-loop L = G*K (series: error e -> controller -> plant -> output y).
    // State order [x_k; x_g]. SISO when both G and K are SISO.
    StateSpace openLoopGK(const StateSpace& G, const StateSpace& K)
    {
        const int ng = G.stateSize();
        const int nk = K.stateSize();
        const Eigen::MatrixXd& Ag = G.A; const Eigen::MatrixXd& Bg = G.B;
        const Eigen::MatrixXd& Cg = G.C; const Eigen::MatrixXd& Dg = G.D;
        const Eigen::MatrixXd& Ak = K.A; const Eigen::MatrixXd& Bk = K.B;
        const Eigen::MatrixXd& Ck = K.C; const Eigen::MatrixXd& Dk = K.D;

        const int n = nk + ng;
        Eigen::MatrixXd A(n, n);
        A.topLeftCorner(nk, nk)     = Ak;
        A.topRightCorner(nk, ng).setZero();
        A.bottomLeftCorner(ng, nk)  = Bg * Ck;
        A.bottomRightCorner(ng, ng) = Ag;

        Eigen::MatrixXd B(n, K.inputSize());
        B.topRows(nk)    = Bk;
        B.bottomRows(ng) = Bg * Dk;

        Eigen::MatrixXd C(G.outputSize(), n);
        C.leftCols(nk)  = Dg * Ck;
        C.rightCols(ng) = Cg;

        Eigen::MatrixXd D = Dg * Dk;
        return StateSpace(A, B, C, D, G.Ts);
    }

    double percentile(const std::vector<double>& sorted, double p)
    {
        if (sorted.empty()) return kNaN;
        if (sorted.size() == 1) return sorted.front();
        const double idx = (p / 100.0) * (static_cast<double>(sorted.size()) - 1.0);
        const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
        const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
        const double frac = idx - static_cast<double>(lo);
        return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
    }

    MonteCarloResult::MetricStats
    computeStats(const std::vector<double>& values, bool worst_is_max)
    {
        MonteCarloResult::MetricStats s;
        std::vector<double> v;
        v.reserve(values.size());
        for (double x : values)
            if (std::isfinite(x)) v.push_back(x);

        if (v.empty())
        {
            s.mean = s.std_dev = s.p5 = s.p25 = s.p50 = s.p75 = s.p95 = s.worst = kNaN;
            return s;
        }

        double sum = 0.0;
        for (double x : v) sum += x;
        s.mean = sum / static_cast<double>(v.size());

        double var = 0.0;
        for (double x : v) var += (x - s.mean) * (x - s.mean);
        s.std_dev = std::sqrt(var / static_cast<double>(v.size()));

        std::sort(v.begin(), v.end());
        s.p5  = percentile(v, 5.0);
        s.p25 = percentile(v, 25.0);
        s.p50 = percentile(v, 50.0);
        s.p75 = percentile(v, 75.0);
        s.p95 = percentile(v, 95.0);
        s.worst = worst_is_max ? v.back() : v.front();
        return s;
    }

} // namespace

// ===========================================================================
// Model spawning
// ===========================================================================

std::vector<TransferFunction>
spawn_TF_samples(const TransferFunction& nominal,
                 uint32_t num_samples,
                 const std::vector<double>& numerator_sigma,
                 const std::vector<double>& denominator_sigma,
                 uint32_t seed)
{
    if (!(numerator_sigma.size() == 1 || numerator_sigma.size() == nominal.num.size()))
        throw std::invalid_argument(
            "spawn_TF_samples: numerator_sigma length must be 1 or num.size().");
    if (!(denominator_sigma.size() == 1 || denominator_sigma.size() == nominal.den.size()))
        throw std::invalid_argument(
            "spawn_TF_samples: denominator_sigma length must be 1 or den.size().");

    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, 1.0);

    std::vector<TransferFunction> out;
    out.reserve(num_samples);

    for (uint32_t s = 0; s < num_samples; ++s)
    {
        std::vector<double> num = nominal.num;
        std::vector<double> den = nominal.den;

        for (std::size_t i = 0; i < num.size(); ++i)
            num[i] *= (1.0 + sigmaAt(numerator_sigma, i) * nd(rng));

        // Keep den[0] == 1 (monic); perturb only a1..an.
        for (std::size_t i = 1; i < den.size(); ++i)
            den[i] *= (1.0 + sigmaAt(denominator_sigma, i) * nd(rng));

        out.emplace_back(std::move(num), std::move(den), nominal.Ts);
    }
    return out;
}

std::vector<StateSpace>
spawn_SS_samples(const StateSpace& nominal,
                 uint32_t num_samples,
                 double sigma_A,
                 double sigma_B,
                 double sigma_C,
                 double sigma_D,
                 uint32_t seed)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, 1.0);

    auto perturb = [&](const Eigen::MatrixXd& M, double sigma) {
        Eigen::MatrixXd P = M;
        if (sigma != 0.0)
            for (int j = 0; j < P.cols(); ++j)
                for (int i = 0; i < P.rows(); ++i)
                    P(i, j) *= (1.0 + sigma * nd(rng));
        return P;
    };

    std::vector<StateSpace> out;
    out.reserve(num_samples);
    for (uint32_t s = 0; s < num_samples; ++s)
    {
        out.emplace_back(perturb(nominal.A, sigma_A),
                         perturb(nominal.B, sigma_B),
                         perturb(nominal.C, sigma_C),
                         perturb(nominal.D, sigma_D),
                         nominal.Ts);
    }
    return out;
}

// ===========================================================================
// Monte-Carlo analysis
// ===========================================================================

RobustnessSample
evaluateSample(int sample_id,
               const StateSpace& plant,
               const StateSpace& controller_ss,
               const StateSpace& nominal_plant,
               double step_amplitude,
               double sim_duration_s,
               double settling_band)
{
    RobustnessSample r;
    r.sample_id = sample_id;

    const ClosedLoop cl = buildClosedLoop(plant, controller_ss);

    // ---- Stability from closed-loop poles --------------------------------
    r.is_stable = SystemAnalysis::isDiscreteStable(cl.T);

    // ---- Frequency-domain peaks ------------------------------------------
    r.hinf_sensitivity = SystemAnalysis::calculateHInfinityNorm(cl.S);
    r.hinf_comp_sens   = SystemAnalysis::calculateHInfinityNorm(cl.T);

    // ---- Gain/phase margins (SISO open loop only) ------------------------
    if (plant.inputSize() == 1 && plant.outputSize() == 1 &&
        controller_ss.inputSize() == 1 && controller_ss.outputSize() == 1)
    {
        const StabilityMargins m = SystemAnalysis::calculateMargins(openLoopGK(plant, controller_ss));
        r.gain_margin_db   = m.gainMarginDb;
        r.phase_margin_deg = m.phaseMarginDeg;
    }
    else
    {
        r.gain_margin_db   = kNaN;
        r.phase_margin_deg = kNaN;
    }

    // ---- Time-domain step response (r -> y via T) ------------------------
    if (r.is_stable)
    {
        const int    pg = cl.T.outputSize();
        const int    N  = std::max(1, static_cast<int>(std::ceil(sim_duration_s / cl.T.Ts)));
        const double tol = settling_band * std::abs(step_amplitude);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(cl.T.stateSize());
        Eigen::VectorXd ref = Eigen::VectorXd::Constant(pg, step_amplitude);

        double iae = 0.0;
        double peak = -std::numeric_limits<double>::infinity();
        double settling = kNaN;
        for (int k = 0; k < N; ++k)
        {
            const Eigen::VectorXd y = ssStep(cl.T, x, ref);
            const double y0 = y(0);
            const double err = step_amplitude - y0;
            iae += std::abs(err) * cl.T.Ts;
            peak = std::max(peak, y0);
            if (std::abs(err) > tol)
                settling = (k + 1) * cl.T.Ts; // last time outside the band
        }
        r.iae = iae;
        r.settling_time_s = settling; // NaN if never left the band (already settled)
        r.overshoot_pct = (step_amplitude != 0.0 && peak > step_amplitude)
                              ? (peak - step_amplitude) / std::abs(step_amplitude) * 100.0
                              : kNaN;
    }
    else
    {
        r.iae = std::numeric_limits<double>::infinity();
        r.settling_time_s = kNaN;
        r.overshoot_pct = kNaN;
    }

    // ---- nu-gap from the nominal plant -----------------------------------
    r.nu_gap_from_nominal = nuGap(nominal_plant, plant);

    return r;
}

MonteCarloResult
runMonteCarlo(const std::vector<StateSpace>& ensemble,
              const StateSpace& controller_ss,
              const StateSpace& nominal_plant,
              double step_amplitude,
              double sim_duration_s)
{
    MonteCarloResult res;
    res.n_samples = static_cast<int>(ensemble.size());
    res.samples.reserve(ensemble.size());

    std::vector<double> gm, pm, sens, csens, iae, settling, gap;
    gm.reserve(ensemble.size());       pm.reserve(ensemble.size());
    sens.reserve(ensemble.size());     csens.reserve(ensemble.size());
    iae.reserve(ensemble.size());      settling.reserve(ensemble.size());
    gap.reserve(ensemble.size());

    for (std::size_t i = 0; i < ensemble.size(); ++i)
    {
        RobustnessSample s = evaluateSample(static_cast<int>(i), ensemble[i],
                                            controller_ss, nominal_plant,
                                            step_amplitude, sim_duration_s);
        if (!s.is_stable) ++res.n_unstable;

        gm.push_back(s.gain_margin_db);
        pm.push_back(s.phase_margin_deg);
        sens.push_back(s.hinf_sensitivity);
        csens.push_back(s.hinf_comp_sens);
        iae.push_back(s.iae);
        settling.push_back(s.settling_time_s);
        gap.push_back(s.nu_gap_from_nominal);

        res.samples.push_back(std::move(s));
    }

    res.instability_probability =
        res.n_samples > 0 ? static_cast<double>(res.n_unstable) / res.n_samples : 0.0;

    res.gm_stats                    = computeStats(gm,       /*worst_is_max=*/false);
    res.pm_stats                    = computeStats(pm,       /*worst_is_max=*/false);
    res.sensitivity_peak_stats      = computeStats(sens,     /*worst_is_max=*/true);
    res.comp_sensitivity_peak_stats = computeStats(csens,    /*worst_is_max=*/true);
    res.iae_stats                   = computeStats(iae,      /*worst_is_max=*/true);
    res.settling_stats              = computeStats(settling, /*worst_is_max=*/true);
    res.nu_gap_stats                = computeStats(gap,      /*worst_is_max=*/true);

    return res;
}

MonteCarloResult
monteCarloAnalysis(const StateSpace& nominal_plant,
                   const StateSpace& controller_ss,
                   uint32_t num_samples,
                   double sigma_A,
                   double sigma_B,
                   double sigma_C,
                   double sigma_D,
                   uint32_t seed)
{
    const std::vector<StateSpace> ensemble =
        spawn_SS_samples(nominal_plant, num_samples, sigma_A, sigma_B, sigma_C, sigma_D, seed);
    return runMonteCarlo(ensemble, controller_ss, nominal_plant);
}

} // namespace ctrl
