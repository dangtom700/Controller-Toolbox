#pragma once
#include "IController.h"
#include <Eigen/Dense>
#include <functional>
#include <string>
#include <vector>

// Mamdani / Takagi-Sugeno fuzzy inference engine.
//
// ── Architecture ─────────────────────────────────────────────────────────────
//
//  1. INPUT VARIABLES  — each partitioned into N linguistic terms.
//     Membership functions:  triangular, trapezoidal, Gaussian, singleton.
//
//  2. RULE BASE  — list of if-then rules in the form:
//       IF  x1 is A_i  AND  x2 is B_j  ...  THEN  y is C_k
//     Rules are evaluated using product (t-norm) for AND.
//
//  3. INFERENCE METHODS
//     a) Mamdani   — output fuzzy sets are clipped/scaled, then aggregated.
//        Defuzzification: Centre of Gravity (CoG) over a discrete universe.
//     b) Takagi-Sugeno (TS) — consequent is a crisp function of inputs (or
//        a singleton value per rule).  Defuzzification: weighted average.
//
//  4. OUTPUT  — single real value (for SISO use) or vector (MIMO via
//     multiple FuzzySystem instances, one per output axis).
//
// ── SISO IController wrapper ──────────────────────────────────────────────────
//  FuzzyController wraps one FuzzySystem in the IController interface so it can
//  be used wherever DiscretePID, DiscreteSMC, etc. are used.
//
// ── Terminology ───────────────────────────────────────────────────────────────
//  MF       — membership function
//  crisp    — ordinary real-valued number (vs. a fuzzy set)
//  universe — the domain of an input or output variable
//
// Ref: Mamdani & Assilian (1975); Takagi & Sugeno (1985);
//      Jang, Sun, Mizutani "Neuro-Fuzzy and Soft Computing" (1997);
//      MATLAB Fuzzy Logic Toolbox documentation.

namespace ctrl
{

// ════════════════════════════════════════════════════════════════════════════
// Membership function types
// ════════════════════════════════════════════════════════════════════════════

// A callable that maps a crisp input to a membership degree in [0, 1].
using MF = std::function<double(double)>;

// Factory functions — return a properly typed MF.
// All parameters should match the universe of the variable.

// Triangular:  /\  peaks at c, zero at [a, b]  (a <= c <= b)
MF mfTriangular(double a, double c, double b);

// Trapezoidal: /‾\  flat-top from b to c, zero outside [a, d]
MF mfTrapezoidal(double a, double b, double c, double d);

// Gaussian:  exp(-0.5*((x-mean)/sigma)^2)
MF mfGaussian(double mean, double sigma);

// Singleton:  1 at x==value, 0 elsewhere (for TS consequents)
MF mfSingleton(double value);

// Shoulder functions — open-ended at one side (useful for NL / PL extremes)
MF mfShoulderLeft(double a, double b);   // 1 for x<=a, linear 1->0 from a to b
MF mfShoulderRight(double a, double b);  // linear 0->1 from a to b, 1 for x>=b


// ════════════════════════════════════════════════════════════════════════════
// Linguistic variable
// ════════════════════════════════════════════════════════════════════════════

struct LinguisticTerm {
    std::string name;
    MF          mf;
};

struct LinguisticVariable {
    std::string                  name;
    double                       lo, hi;   // universe of discourse
    std::vector<LinguisticTerm>  terms;

    // Evaluate all membership degrees for a crisp input x.
    // Result size == terms.size().
    std::vector<double> fuzzify(double x) const;

    // Index of a term by name (-1 if not found).
    int termIndex(const std::string& termName) const;
};


// ════════════════════════════════════════════════════════════════════════════
// Rule base
// ════════════════════════════════════════════════════════════════════════════

struct Antecedent {
    int input_idx;   // which input variable
    int term_idx;    // which term of that variable
};

struct Rule {
    std::vector<Antecedent> antecedents;   // AND-connected conditions
    int  consequent_term_idx;              // which output term fires
    double weight = 1.0;                  // optional rule weight
};


// ════════════════════════════════════════════════════════════════════════════
// FuzzySystem — core inference engine
// ════════════════════════════════════════════════════════════════════════════

enum class InferenceMethod { Mamdani, TakagiSugeno };
enum class DefuzzMethod    { CoG,     WeightedAverage };

struct FuzzySystemParams {
    InferenceMethod inference = InferenceMethod::Mamdani;
    DefuzzMethod    defuzz    = DefuzzMethod::CoG;
    int             cog_resolution = 101;   // CoG discretisation points
    double          uMin = -1e9;
    double          uMax =  1e9;
};

class FuzzySystem {
public:
    FuzzySystemParams params;

    // Build the system step by step.
    void addInput (const LinguisticVariable& var);
    void addOutput(const LinguisticVariable& var);   // one output supported
    void addRule  (const Rule& rule);

    // Evaluate: inputs = crisp values for each input variable (in order).
    // Returns the crisp defuzzified output.
    double evaluate(const std::vector<double>& inputs) const;

    int numInputs()  const { return static_cast<int>(inputs_.size());  }
    int numOutputs() const { return static_cast<int>(outputs_.size()); }
    int numRules()   const { return static_cast<int>(rules_.size());   }

    const LinguisticVariable& inputVar (int i) const { return inputs_[i];  }
    const LinguisticVariable& outputVar(int i) const { return outputs_[i]; }

private:
    std::vector<LinguisticVariable> inputs_;
    std::vector<LinguisticVariable> outputs_;
    std::vector<Rule>               rules_;

    // Activation strength of one rule given pre-fuzzified memberships.
    // mu[i] = vector of memberships for input variable i.
    double ruleStrength(const Rule& r,
                        const std::vector<std::vector<double>>& mu) const;

    double defuzzCoG           (const std::vector<double>& strengths) const;
    double defuzzWeightedAvg   (const std::vector<double>& strengths) const;
};


// ════════════════════════════════════════════════════════════════════════════
// FuzzyPD — convenience builder: 5-term PD fuzzy controller for one axis.
//
// Inputs:  error e,  error-rate de
// Output:  control effort u
//
// Standard 5-term partition: NL, NS, ZE, PS, PL using triangular + shoulder MFs.
// 25-rule Mamdani table (canonical diagonal structure).
//
// Scaling:
//   e_scale  — maps ±e_scale to universe [-1, 1]   before inference
//   de_scale — maps ±de_scale to universe [-1, 1]  before inference
//   u_scale  — output on universe [-1, 1] is scaled to ±u_scale
// ════════════════════════════════════════════════════════════════════════════

struct FuzzyPDParams {
    double e_scale  = 1.0;    // error normalisation
    double de_scale = 1.0;    // error-rate normalisation
    double u_scale  = 1.0;    // output scaling
    double uMin     = -1e9;
    double uMax     =  1e9;
};

class FuzzyPD : public IController {
public:
    explicit FuzzyPD(const FuzzyPDParams& p, double sampleTime);

    // signal = tracking error e[k]
    double compute(double error) override;

    void   reset()            override;
    double sampleTime() const override { return Ts_; }

    void             setParams(const FuzzyPDParams& p);
    const FuzzyPDParams& params() const { return p_; }

    double lastOutput() const { return u_prev_; }

private:
    FuzzyPDParams p_;
    double        Ts_;
    double        e_prev_;
    double        u_prev_;
    FuzzySystem   sys_;

    void buildSystem();
};


// ════════════════════════════════════════════════════════════════════════════
// FuzzyPID — extends FuzzyPD with a fuzzy integral term.
//
// Architecture: three decoupled fuzzy inference blocks:
//   u_P = FuzzyPD(e, de)            — proportional + derivative
//   u_I = k_i * integral(e)         — crisp accumulator with anti-windup
//   u   = clamp(u_P + u_I, uMin, uMax)
//
// The integral is identical to the PID back-calculation anti-windup scheme
// (Ki * Ts * e + Kb * (u_sat - u_unsat)), keeping it numerically safe.
// ════════════════════════════════════════════════════════════════════════════

struct FuzzyPIDParams {
    FuzzyPDParams pd;           // FuzzyPD tuning (e_scale, de_scale, u_scale)
    double        Ki    = 0.0;  // Integral gain (0 = pure FuzzyPD)
    double        Kb    = 1.0;  // Anti-windup back-calculation gain
    double        uMin  = -1e9;
    double        uMax  =  1e9;
};

class FuzzyPID : public IController {
public:
    explicit FuzzyPID(const FuzzyPIDParams& p, double sampleTime);

    double compute(double error) override;

    void   reset()            override;
    double sampleTime() const override { return Ts_; }

    void              setParams(const FuzzyPIDParams& p);
    const FuzzyPIDParams& params() const { return p_; }

    double lastOutput() const { return u_prev_; }

    // Bumpless init: set integral so next compute(error) ≈ u_target.
    void bumplessInit(double u_target, double error) override;

private:
    FuzzyPIDParams p_;
    double         Ts_;
    FuzzyPD        pd_block_;
    double         integral_;
    double         u_prev_;
};


// ════════════════════════════════════════════════════════════════════════════
// FuzzySupervisor — monitors closed-loop performance and decides whether the
// underlying controller's linearised plant model needs to be refreshed.
//
// Decision logic (Mamdani fuzzy, 2 inputs -> 1 output):
//
//   Input 1: normalised_error = |e| / e_threshold
//     Terms: Small (0-0.4), Medium (0.3-0.7), Large (0.6-1+)
//
//   Input 2: error_trend = d|e|/dt / trend_threshold  (positive = diverging)
//     Terms: Decreasing (<0), Steady (~0), Increasing (>0)
//
//   Output: relinearize_signal in [0, 1]
//     0 = no action, 1 = force relinearisation NOW
//
//   Representative rules:
//     IF error is Small  AND trend is Decreasing  THEN signal is None (0.0)
//     IF error is Small  AND trend is Steady      THEN signal is None (0.0)
//     IF error is Medium AND trend is Increasing  THEN signal is Maybe (0.5)
//     IF error is Large  AND trend is Steady      THEN signal is High (0.8)
//     IF error is Large  AND trend is Increasing  THEN signal is Full (1.0)
//     (full 9-rule table built internally)
//
// Usage pattern in simulation:
//   auto sig = supervisor.update(error_norm, delta_t);
//   if (sig.relinearize) mpc.setPlant(relinearize(plant, current_state));
//
// ════════════════════════════════════════════════════════════════════════════

struct SupervisorParams {
    double e_threshold    = 5.0;   // |e| at which error is considered "Large" (same units as error)
    double trend_threshold = 0.5;  // d|e|/dt considered "Increasing fast" (units/s)
    double signal_threshold = 0.5; // relinearize_signal > this -> trigger action
    // Hysteresis: once triggered, don't trigger again for cooldown_steps
    int    cooldown_steps = 20;
};

struct SupervisorDecision {
    double relinearize_signal;  // raw fuzzy output [0, 1]
    bool   relinearize;         // threshold decision
    double error_norm;
    double trend;
};

class FuzzySupervisor {
public:
    explicit FuzzySupervisor(const SupervisorParams& p, double sampleTime);

    // Feed current absolute position error (scalar, in metres or rad).
    // Returns a decision struct with the fuzzy signal and boolean flag.
    SupervisorDecision update(double abs_error);

    void reset();

    const SupervisorParams& params() const { return p_; }
    void setParams(const SupervisorParams& p) { p_ = p; }

private:
    SupervisorParams p_;
    double           Ts_;
    double           abs_error_prev_;
    int              cooldown_remaining_;
    FuzzySystem      sys_;

    void buildSystem();
};

} // namespace ctrl
