#include "FuzzyLogic.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace ctrl
{

// Named constants - avoids magic literals scattered through inference code.
static constexpr int kCoGResolutionDefault   = 101; // CoG grid points (FuzzyPD/Supervisor default)
static constexpr int kTSPeakSearchResolution =  51; // fallback grid for WA peak search (non-singleton terms)

// ════════════════════════════════════════════════════════════════════════════
// Membership function factories
// ════════════════════════════════════════════════════════════════════════════

MF mfTriangular(double a, double c, double b)
{
    return [a, c, b](double x) -> double {
        if (x <= a || x >= b) return 0.0;
        if (x <= c) return (x - a) / (c - a + 1e-12);
        return (b - x) / (b - c + 1e-12);
    };
}

MF mfTrapezoidal(double a, double b, double c, double d)
{
    return [a, b, c, d](double x) -> double {
        if (x <= a || x >= d) return 0.0;
        if (x >= b && x <= c) return 1.0;
        if (x < b)  return (x - a) / (b - a + 1e-12);
        return (d - x) / (d - c + 1e-12);
    };
}

MF mfGaussian(double mean, double sigma)
{
    return [mean, sigma](double x) -> double {
        double z = (x - mean) / (sigma + 1e-12);
        return std::exp(-0.5 * z * z);
    };
}

MF mfSingleton(double value)
{
    return [value](double x) -> double {
        return (std::abs(x - value) < 1e-9) ? 1.0 : 0.0;
    };
}

LinguisticTerm ltSingleton(const std::string& name, double value)
{
    return LinguisticTerm{name, mfSingleton(value), value};
}

MF mfShoulderLeft(double a, double b)
{
    return [a, b](double x) -> double {
        if (x <= a) return 1.0;
        if (x >= b) return 0.0;
        return (b - x) / (b - a + 1e-12);
    };
}

MF mfShoulderRight(double a, double b)
{
    return [a, b](double x) -> double {
        if (x <= a) return 0.0;
        if (x >= b) return 1.0;
        return (x - a) / (b - a + 1e-12);
    };
}


// ════════════════════════════════════════════════════════════════════════════
// LinguisticVariable
// ════════════════════════════════════════════════════════════════════════════

std::vector<double> LinguisticVariable::fuzzify(double x) const
{
    std::vector<double> mu(terms.size());
    for (std::size_t i = 0; i < terms.size(); ++i)
        mu[i] = std::clamp(terms[i].mf(x), 0.0, 1.0);
    return mu;
}

int LinguisticVariable::termIndex(const std::string& termName) const
{
    for (int i = 0; i < static_cast<int>(terms.size()); ++i)
        if (terms[i].name == termName) return i;
    return -1;
}


// ════════════════════════════════════════════════════════════════════════════
// FuzzySystem
// ════════════════════════════════════════════════════════════════════════════

void FuzzySystem::rebuildWorkspace()
{
    mu_.resize(inputs_.size());
    for (std::size_t i = 0; i < inputs_.size(); ++i)
        mu_[i].resize(inputs_[i].terms.size(), 0.0);
    if (!outputs_.empty())
        strengths_.assign(outputs_[0].terms.size(), 0.0);
}

void FuzzySystem::addInput(const LinguisticVariable& var)
{
    inputs_.push_back(var);
    rebuildWorkspace();
}

void FuzzySystem::addOutput(const LinguisticVariable& var)
{
    if (!outputs_.empty())
        throw std::logic_error("FuzzySystem: only one output variable is supported. "
                               "Use separate FuzzySystem instances for MIMO outputs.");
    outputs_.push_back(var);
    rebuildWorkspace();
}

void FuzzySystem::addRule(const Rule& rule)                { rules_.push_back(rule);  }

double FuzzySystem::ruleStrength(const Rule& r,
                                  const std::vector<std::vector<double>>& mu) const
{
    // Product t-norm over all antecedents (product-AND)
    double strength = r.weight;
    for (auto& ant : r.antecedents)
        strength *= mu[ant.input_idx][ant.term_idx];
    return strength;
}

double FuzzySystem::evaluate(const std::vector<double>& inputs) const
{
    if (inputs.size() != inputs_.size())
        throw std::invalid_argument("FuzzySystem::evaluate: wrong number of inputs");
    if (outputs_.empty())
        throw std::logic_error("FuzzySystem::evaluate: no output variable defined");

    // Fuzzify all inputs - fill pre-allocated workspace mu_ in-place (no heap alloc)
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        const auto& terms = inputs_[i].terms;
        for (std::size_t t = 0; t < terms.size(); ++t)
            mu_[i][t] = std::clamp(terms[t].mf(inputs[i]), 0.0, 1.0);
    }

    // Fire all rules - accumulate activation strengths per output term (max aggregation)
    const int nTerms = static_cast<int>(outputs_[0].terms.size());
    std::fill(strengths_.begin(), strengths_.end(), 0.0);

    for (auto& rule : rules_) {
        double w = ruleStrength(rule, mu_);
        int tidx = rule.consequent_term_idx;
        if (tidx >= 0 && tidx < nTerms)
            strengths_[tidx] = std::max(strengths_[tidx], w);
    }

    // Defuzzify
    double result = 0.0;
    if (params.inference == InferenceMethod::TakagiSugeno ||
        params.defuzz    == DefuzzMethod::WeightedAverage)
        result = defuzzWeightedAvg(strengths_);
    else
        result = defuzzCoG(strengths_);

    return std::clamp(result, params.uMin, params.uMax);
}

// Centre of Gravity over a discrete universe grid
double FuzzySystem::defuzzCoG(const std::vector<double>& strengths) const
{
    const auto& out = outputs_[0];
    const int   N   = params.cog_resolution;
    const double lo = out.lo, hi = out.hi;
    const double step = (hi - lo) / (N - 1);

    double num = 0.0, den = 0.0;
    for (int k = 0; k < N; ++k) {
        double x  = lo + k * step;
        // Aggregate: max over all active rule-clipped output MFs
        double agg = 0.0;
        for (int t = 0; t < static_cast<int>(out.terms.size()); ++t) {
            double clipped = std::min(strengths[t], out.terms[t].mf(x));
            agg = std::max(agg, clipped);
        }
        num += x * agg;
        den += agg;
    }
    if (den < 1e-12) return (lo + hi) * 0.5;  // no rules fired - return centre
    return num / den;
}

// Weighted average (Takagi-Sugeno / singleton consequents)
double FuzzySystem::defuzzWeightedAvg(const std::vector<double>& strengths) const
{
    const auto& out = outputs_[0];
    double num = 0.0, den = 0.0;
    for (int t = 0; t < static_cast<int>(out.terms.size()); ++t) {
        double centre;
        if (out.terms[t].peak.has_value()) {
            // Exact peak provided (singletons, or explicitly set TS terms) - no grid search.
            centre = out.terms[t].peak.value();
        } else {
            // Fall back to grid search for continuous Mamdani-style output terms.
            centre = (out.lo + out.hi) * 0.5;
            constexpr int M = kTSPeakSearchResolution;
            double best_mu = 0.0;
            for (int k = 0; k < M; ++k) {
                double x  = out.lo + k * (out.hi - out.lo) / (M - 1);
                double mu = out.terms[t].mf(x);
                if (mu > best_mu) { best_mu = mu; centre = x; }
            }
        }
        num += strengths[t] * centre;
        den += strengths[t];
    }
    if (den < 1e-12) return (out.lo + out.hi) * 0.5;
    return num / den;
}


// ════════════════════════════════════════════════════════════════════════════
// FuzzyPD
// ════════════════════════════════════════════════════════════════════════════

// Canonical 5-term partition on [-1, 1]:  NL, NS, ZE, PS, PL
// 25-rule diagonal table (same as Mamdani textbook PD table)
static void build5TermPartition(LinguisticVariable& v)
{
    v.terms.push_back({"NL", mfShoulderLeft(-1.0, -0.5)});
    v.terms.push_back({"NS", mfTriangular (-1.0, -0.5, 0.0)});
    v.terms.push_back({"ZE", mfTriangular (-0.5,  0.0, 0.5)});
    v.terms.push_back({"PS", mfTriangular ( 0.0,  0.5, 1.0)});
    v.terms.push_back({"PL", mfShoulderRight(0.5,  1.0)});
}

// Diagonal 25-rule table: e_term x de_term -> u_term
// Index: NL=0, NS=1, ZE=2, PS=3, PL=4
static const int kPDRuleTable[5][5] = {
    // de: NL   NS   ZE   PS   PL        e->
    {      0,   0,   0,   1,   2  },  // NL
    {      0,   0,   1,   2,   3  },  // NS
    {      0,   1,   2,   3,   4  },  // ZE
    {      1,   2,   3,   4,   4  },  // PS
    {      2,   3,   4,   4,   4  },  // PL
};

void FuzzyPD::buildSystem()
{
    sys_ = FuzzySystem{};
    sys_.params.inference       = InferenceMethod::Mamdani;
    sys_.params.defuzz          = DefuzzMethod::CoG;
    sys_.params.cog_resolution  = kCoGResolutionDefault;
    sys_.params.uMin = -1.0;
    sys_.params.uMax =  1.0;

    LinguisticVariable ve; ve.name = "e";  ve.lo = -1.0; ve.hi = 1.0;
    LinguisticVariable vde; vde.name = "de"; vde.lo = -1.0; vde.hi = 1.0;
    LinguisticVariable vu;  vu.name = "u";  vu.lo = -1.0; vu.hi = 1.0;

    build5TermPartition(ve);
    build5TermPartition(vde);
    build5TermPartition(vu);

    sys_.addInput(ve);
    sys_.addInput(vde);
    sys_.addOutput(vu);

    // Build 25 rules from the diagonal table
    for (int ie = 0; ie < 5; ++ie) {
        for (int ide = 0; ide < 5; ++ide) {
            Rule r;
            r.antecedents.push_back({0, ie});
            r.antecedents.push_back({1, ide});
            r.consequent_term_idx = kPDRuleTable[ie][ide];
            r.weight = 1.0;
            sys_.addRule(r);
        }
    }
}

FuzzyPD::FuzzyPD(const FuzzyPDParams& p, double sampleTime)
    : p_(p), Ts_(sampleTime), e_prev_(0.0), u_prev_(0.0)
{
    buildSystem();
}

void FuzzyPD::setParams(const FuzzyPDParams& p)
{
    p_ = p;
    // Rebuild not needed - scaling is applied externally before evaluate()
}

double FuzzyPD::compute(double error)
{
    if (!std::isfinite(error)) return u_prev_;

    double de = (error - e_prev_) / Ts_;

    // Normalise to [-1, 1] universe
    double e_n  = std::clamp(error / (p_.e_scale  + 1e-12), -1.0, 1.0);
    double de_n = std::clamp(de     / (p_.de_scale + 1e-12), -1.0, 1.0);

    double u_n = sys_.evaluate({e_n, de_n});

    double u = std::clamp(u_n * p_.u_scale, p_.uMin, p_.uMax);

    e_prev_ = error;
    u_prev_ = u;
    return u;
}

void FuzzyPD::reset()
{
    e_prev_ = 0.0;
    u_prev_ = 0.0;
}


// ════════════════════════════════════════════════════════════════════════════
// FuzzyPID
// ════════════════════════════════════════════════════════════════════════════

FuzzyPID::FuzzyPID(const FuzzyPIDParams& p, double sampleTime)
    : p_(p), Ts_(sampleTime), pd_block_(p.pd, sampleTime)
    , integral_(0.0), u_prev_(0.0)
{}

void FuzzyPID::setParams(const FuzzyPIDParams& p)
{
    p_ = p;
    pd_block_.setParams(p.pd);
}

double FuzzyPID::compute(double error)
{
    if (!std::isfinite(error)) return u_prev_;

    // FuzzyPD block provides the P+D part
    double u_pd = pd_block_.compute(error);

    // Integral accumulation (backward Euler) + anti-windup
    double ki_update = p_.Ki * Ts_ * error;
    double u_unsat   = u_pd + integral_ + ki_update;
    double u_sat     = std::clamp(u_unsat, p_.uMin, p_.uMax);

    integral_ += ki_update + p_.Kb * (u_sat - u_unsat);
    u_prev_    = u_sat;
    return u_sat;
}

void FuzzyPID::reset()
{
    pd_block_.reset();
    integral_ = 0.0;
    u_prev_   = 0.0;
}

void FuzzyPID::bumplessInit(double u_target, double error)
{
    pd_block_.reset();
    // Estimate PD contribution at this error with de=0 (first call after reset).
    double u_pd_est = pd_block_.compute(error);
    pd_block_.reset();  // restore clean derivative state (e_prev_=0)
    // Set integral so that u_pd_est + integral_ approx = u_target on the next compute().
    integral_ = u_target - u_pd_est;
    u_prev_   = u_target;
}


// ════════════════════════════════════════════════════════════════════════════
// FuzzySupervisor
// ════════════════════════════════════════════════════════════════════════════

void FuzzySupervisor::buildSystem()
{
    sys_ = FuzzySystem{};
    sys_.params.inference      = InferenceMethod::Mamdani;
    sys_.params.defuzz         = DefuzzMethod::CoG;
    sys_.params.cog_resolution = kTSPeakSearchResolution;
    sys_.params.uMin = 0.0;
    sys_.params.uMax = 1.0;

    // Input 1: normalised error magnitude [0, 1+]
    LinguisticVariable verr;
    verr.name = "error_norm"; verr.lo = 0.0; verr.hi = 1.5;
    verr.terms.push_back({"Small",  mfTrapezoidal(0.0, 0.0, 0.25, 0.45)});
    verr.terms.push_back({"Medium", mfTriangular (0.30, 0.50, 0.70)});
    verr.terms.push_back({"Large",  mfShoulderRight(0.60, 0.90)});

    // Input 2: normalised error trend (d|e|/dt) [-1, 1]
    LinguisticVariable vtrend;
    vtrend.name = "trend"; vtrend.lo = -1.0; vtrend.hi = 1.0;
    vtrend.terms.push_back({"Decreasing", mfShoulderLeft(-0.6, -0.1)});
    vtrend.terms.push_back({"Steady",     mfTriangular  (-0.3,  0.0, 0.3)});
    vtrend.terms.push_back({"Increasing", mfShoulderRight(0.1,  0.6)});

    // Output: relinearize_signal [0, 1]
    LinguisticVariable vsig;
    vsig.name = "signal"; vsig.lo = 0.0; vsig.hi = 1.0;
    vsig.terms.push_back({"None",  mfTriangular(0.0, 0.0, 0.3)});   // 0
    vsig.terms.push_back({"Maybe", mfTriangular(0.3, 0.5, 0.7)});   // 1
    vsig.terms.push_back({"High",  mfTriangular(0.6, 0.8, 1.0)});   // 2
    vsig.terms.push_back({"Full",  mfShoulderRight(0.8, 1.0)});     // 3

    sys_.addInput(verr);
    sys_.addInput(vtrend);
    sys_.addOutput(vsig);

    // 9-rule table:  error_term(3) x trend_term(3) -> signal_term(4)
    // Rows = error: Small(0), Medium(1), Large(2)
    // Cols = trend: Decreasing(0), Steady(1), Increasing(2)
    // Output terms: None=0, Maybe=1, High=2, Full=3
    const int table[3][3] = {
        { 0, 0, 1 },   // Small error
        { 0, 1, 2 },   // Medium error
        { 1, 2, 3 },   // Large error
    };

    for (int ie = 0; ie < 3; ++ie) {
        for (int it = 0; it < 3; ++it) {
            Rule r;
            r.antecedents.push_back({0, ie});
            r.antecedents.push_back({1, it});
            r.consequent_term_idx = table[ie][it];
            r.weight = 1.0;
            sys_.addRule(r);
        }
    }
}

FuzzySupervisor::FuzzySupervisor(const SupervisorParams& p, double sampleTime)
    : p_(p), Ts_(sampleTime), abs_error_prev_(0.0), cooldown_remaining_(0)
{
    buildSystem();
}

void FuzzySupervisor::reset()
{
    abs_error_prev_    = 0.0;
    cooldown_remaining_ = 0;
}

SupervisorDecision FuzzySupervisor::update(double abs_error)
{
    // Normalise error
    double err_n = abs_error / (p_.e_threshold + 1e-12);

    // Trend: rate of change of |error|, normalised
    double d_err  = (abs_error - abs_error_prev_) / (Ts_ + 1e-12);
    double trend_n = std::clamp(d_err / (p_.trend_threshold + 1e-12), -1.0, 1.0);

    abs_error_prev_ = abs_error;

    // Fuzzy inference
    double signal = sys_.evaluate({std::clamp(err_n, 0.0, 1.5), trend_n});

    // Hysteresis / cooldown
    bool trigger = false;
    if (cooldown_remaining_ > 0) {
        --cooldown_remaining_;
    } else if (signal > p_.signal_threshold) {
        trigger = true;
        cooldown_remaining_ = p_.cooldown_steps;
    }

    return {signal, trigger, err_n, trend_n};
}

} // namespace ctrl
