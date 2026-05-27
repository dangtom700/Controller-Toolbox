#pragma once
#include "IController.h"
#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

/**
 * @file FuzzyLogic.h
 * @brief Mamdani and Takagi-Sugeno fuzzy inference engine with IController wrappers.
 *
 * **Architecture overview:**
 * 1. **Input variables** — each partitioned into N linguistic terms with membership functions
 *    (triangular, trapezoidal, Gaussian, singleton, shoulder).
 * 2. **Rule base** — IF-THEN rules of the form: IF x1 is Aᵢ AND x2 is Bⱼ … THEN y is Cₖ.
 *    AND is implemented via the product t-norm.
 * 3. **Inference methods:**
 *    - *Mamdani* — output fuzzy sets are clipped/scaled then aggregated; defuzzified with CoG.
 *    - *Takagi-Sugeno* — consequent is a crisp value per rule; defuzzified via weighted average.
 * 4. **Output** — single real value (SISO) or one FuzzySystem per axis (MIMO).
 *
 * **IController wrappers:**
 * - FuzzyPD  — 5-term PD controller (25-rule diagonal Mamdani table).
 * - FuzzyPID — FuzzyPD augmented with a crisp integral + back-calculation anti-windup.
 * - FuzzySupervisor — monitors closed-loop performance and triggers plant relinearisation.
 *
 * @see Mamdani & Assilian, "An Experiment in Linguistic Synthesis," Int. J. Man-Machine Studies (1975).
 * @see Takagi & Sugeno, "Fuzzy Identification of Systems," IEEE Trans. SMC (1985).
 * @see Jang, Sun & Mizutani, "Neuro-Fuzzy and Soft Computing" (1997).
 */

namespace ctrl
{

// ─────────────────────────────────────────────────────────────────────────────
// Membership function types
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Value-type tagged-union membership function.
 *
 * Replaces `std::function<double(double)>` to eliminate heap allocation in the
 * RT inference hot path. `std::function` stores closures on the heap for
 * captures > ~16 bytes (implementation-defined SBO threshold), making it unsafe
 * for hard-RT use. MF is a plain struct (sizeof ≤ 5 doubles) that is trivially
 * copyable and stack-allocatable.
 *
 * Use the factory functions (mfTriangular, mfTrapezoidal, etc.) to construct
 * properly typed MF instances — the public API is identical to a callable except
 * that MF is a concrete value type, not a type-erased callable.
 */
struct MF {
    /**
     * @brief Membership function shape selector.
     */
    enum class Type {
        Triangular,    ///< /\  peaks at p[1]=c, zero at p[0]=a and p[2]=b.
        Trapezoidal,   ///< /‾\ flat-top from p[1]=b to p[2]=c, zero outside [p[0]=a, p[3]=d].
        Gaussian,      ///< exp(-0.5·((x-p[0])/p[1])²).
        Singleton,     ///< 1 at x == p[0], 0 elsewhere.
        ShoulderLeft,  ///< 1 for x ≤ p[0], linear 1→0 from p[0]=a to p[1]=b.
        ShoulderRight  ///< linear 0→1 from p[0]=a to p[1]=b, 1 for x ≥ p[1].
    };

    Type   type;
    double p[4] = {};   ///< Shape parameters; unused slots are zero.

    /**
     * @brief Evaluate the membership function at @p x.
     *
     * Inlined for zero-overhead in evaluate().
     *
     * @param x Crisp input value.
     * @return Membership degree ∈ [0, 1].
     */
    double operator()(double x) const;
};

/**
 * @brief Triangular membership function: /\  peaks at @p c, zero at @p a and @p b.
 * @param a Left zero-crossing (a ≤ c).
 * @param c Peak (a ≤ c ≤ b).
 * @param b Right zero-crossing (c ≤ b).
 * @return MF with Type::Triangular.
 */
MF mfTriangular(double a, double c, double b);

/**
 * @brief Trapezoidal membership function: /‾\ flat-top from @p b to @p c, zero outside [@p a, @p d].
 * @param a Left zero-crossing (a ≤ b).
 * @param b Left shoulder start.
 * @param c Right shoulder start (b ≤ c).
 * @param d Right zero-crossing (c ≤ d).
 * @return MF with Type::Trapezoidal.
 */
MF mfTrapezoidal(double a, double b, double c, double d);

/**
 * @brief Gaussian membership function: exp(-0.5·((x - @p mean) / @p sigma)²).
 * @param mean  Centre of the Gaussian.
 * @param sigma Standard deviation (width). Must be > 0.
 * @return MF with Type::Gaussian.
 */
MF mfGaussian(double mean, double sigma);

/**
 * @brief Singleton membership function: 1 at x == @p value, 0 elsewhere.
 *
 * Intended for Takagi-Sugeno consequents.
 *
 * @param value The singleton location.
 * @return MF with Type::Singleton.
 */
MF mfSingleton(double value);

/**
 * @brief Left-shoulder membership function: 1 for x ≤ @p a, linear 1→0 from @p a to @p b.
 *
 * Useful for the "Negative Large" (NL) extreme term.
 *
 * @param a Saturation point (full membership for x ≤ a).
 * @param b Right zero-crossing (a < b).
 * @return MF with Type::ShoulderLeft.
 */
MF mfShoulderLeft(double a, double b);

/**
 * @brief Right-shoulder membership function: linear 0→1 from @p a to @p b, 1 for x ≥ @p b.
 *
 * Useful for the "Positive Large" (PL) extreme term.
 *
 * @param a Left zero-crossing.
 * @param b Saturation point (full membership for x ≥ b, a < b).
 * @return MF with Type::ShoulderRight.
 */
MF mfShoulderRight(double a, double b);


// ─────────────────────────────────────────────────────────────────────────────
// Linguistic variable
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief A named fuzzy term with its membership function and optional peak location.
 */
struct LinguisticTerm {
    std::string            name; ///< Human-readable label (e.g., "Negative Large").
    MF                     mf;   ///< Membership function for this term.

    /**
     * @brief Explicit peak location for weighted-average defuzzification.
     *
     * Set automatically by ltSingleton(). Can be set manually for any unimodal MF
     * when grid-search peak-finding would be ambiguous or too coarse.
     * When `nullopt`, defuzzWeightedAvg falls back to a grid search.
     */
    std::optional<double>  peak = std::nullopt;
};

/**
 * @brief Build a fully specified LinguisticTerm for a Takagi-Sugeno singleton output.
 *
 * Sets both `mf` and `peak` so defuzzWeightedAvg does not rely on grid search.
 *
 * @param name  Label for the term.
 * @param value Singleton value (both MF location and peak).
 * @return Fully initialised LinguisticTerm.
 */
LinguisticTerm ltSingleton(const std::string& name, double value);

/**
 * @brief A fuzzy variable with a named universe of discourse and a set of linguistic terms.
 */
struct LinguisticVariable {
    std::string                  name;  ///< Variable name (e.g., "error").
    double                       lo;    ///< Lower bound of the universe of discourse.
    double                       hi;    ///< Upper bound of the universe of discourse.
    std::vector<LinguisticTerm>  terms; ///< Ordered linguistic terms for this variable.

    /**
     * @brief Evaluate all membership degrees for a crisp input @p x.
     * @param x Crisp value to fuzzify.
     * @return Membership degrees, one per term (same size as terms).
     */
    std::vector<double> fuzzify(double x) const;

    /**
     * @brief Find the index of a term by name.
     * @param termName Term label to search for.
     * @return Index in @c terms, or -1 if not found.
     */
    int termIndex(const std::string& termName) const;
};


// ─────────────────────────────────────────────────────────────────────────────
// Rule base
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief One input condition in a fuzzy rule antecedent.
 */
struct Antecedent {
    int input_idx; ///< Index of the input variable (0-based).
    int term_idx;  ///< Index of the linguistic term within that variable.
};

/**
 * @brief One fuzzy IF-THEN rule.
 *
 * Antecedents are AND-connected via the product t-norm.
 */
struct Rule {
    std::vector<Antecedent> antecedents;       ///< AND-connected input conditions.
    int                     consequent_term_idx; ///< Index of the output term that fires.
    double                  weight = 1.0;       ///< Optional rule weight ∈ (0, 1].
};


// ─────────────────────────────────────────────────────────────────────────────
// FuzzySystem — core inference engine
// ─────────────────────────────────────────────────────────────────────────────

/** @brief Inference method selector. */
enum class InferenceMethod {
    Mamdani,       ///< Clip/scale output MFs, aggregate, then defuzzify with CoG.
    TakagiSugeno   ///< Crisp consequent per rule; defuzzify with weighted average.
};

/** @brief Defuzzification method selector. */
enum class DefuzzMethod {
    CoG,            ///< Centre of Gravity over a discrete universe (Mamdani).
    WeightedAverage ///< Weighted average of consequent peaks (Takagi-Sugeno).
};

/**
 * @brief Configuration parameters for FuzzySystem.
 */
struct FuzzySystemParams {
    InferenceMethod inference      = InferenceMethod::Mamdani; ///< Inference method.
    DefuzzMethod    defuzz         = DefuzzMethod::CoG;         ///< Defuzzification method.
    int             cog_resolution = 101;   ///< Number of discretisation points for CoG.
    double          uMin           = -1e9;  ///< Hard lower output clamp.
    double          uMax           =  1e9;  ///< Hard upper output clamp.
};

/**
 * @brief Core Mamdani / Takagi-Sugeno fuzzy inference engine.
 *
 * Build the system by calling addInput(), addOutput(), then addRule() for each rule.
 * Then call evaluate() with a vector of crisp inputs to get the defuzzified output.
 *
 * @note Only a single output variable is supported. For MIMO fuzzy control, instantiate
 *       one FuzzySystem per output axis.
 */
class FuzzySystem {
public:
    FuzzySystemParams params; ///< Runtime-configurable inference and defuzz settings.

    /**
     * @brief Register an input linguistic variable.
     * @param var Fully defined LinguisticVariable (name, universe, terms).
     */
    void addInput(const LinguisticVariable& var);

    /**
     * @brief Register the single output linguistic variable.
     *
     * Only ONE output variable is supported per FuzzySystem instance.
     *
     * @param var Fully defined LinguisticVariable (name, universe, terms).
     * @throws std::logic_error If called more than once.
     */
    void addOutput(const LinguisticVariable& var);

    /**
     * @brief Add a rule and validate all indices against registered variables.
     *
     * Validation catches index typos before they silently produce zero rule strength
     * at runtime.
     *
     * @param rule Rule to add.
     * @throws std::out_of_range If any antecedent input_idx or term_idx is out of bounds,
     *                           or if consequent_term_idx is out of bounds.
     * @throws std::logic_error  If addOutput() has not been called yet.
     */
    void addRule(const Rule& rule);

    /**
     * @brief Run inference and return the crisp defuzzified output.
     * @param inputs Crisp values for each registered input variable (in registration order).
     * @return Defuzzified output clamped to [uMin, uMax].
     */
    double evaluate(const std::vector<double>& inputs);

    int numInputs()  const { return static_cast<int>(inputs_.size());  }  ///< Number of registered input variables.
    int numOutputs() const { return static_cast<int>(outputs_.size()); }  ///< Always 0 or 1.
    int numRules()   const { return static_cast<int>(rules_.size());   }  ///< Number of registered rules.

    /** @brief Read-only access to input variable @p i. */
    const LinguisticVariable& inputVar (int i) const { return inputs_[i];  }
    /** @brief Read-only access to output variable @p i. */
    const LinguisticVariable& outputVar(int i) const { return outputs_[i]; }

private:
    std::vector<LinguisticVariable> inputs_;
    std::vector<LinguisticVariable> outputs_;
    std::vector<Rule>               rules_;

    // Pre-allocated workspace sized in addInput/addOutput, reused every evaluate()
    // call to avoid per-call heap allocation on RT targets.
    std::vector<std::vector<double>> mu_;        ///< mu_[i][t] = membership of input i, term t.
    std::vector<double>              strengths_; ///< Per-output-term rule activation strength.

    void rebuildWorkspace();

    double ruleStrength(const Rule& r,
                        const std::vector<std::vector<double>>& mu) const;

    double defuzzCoG         (const std::vector<double>& strengths) const;
    double defuzzWeightedAvg (const std::vector<double>& strengths) const;
};


// ─────────────────────────────────────────────────────────────────────────────
// FuzzyPD — convenience 5-term PD fuzzy controller
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Tuning parameters for FuzzyPD.
 */
struct FuzzyPDParams {
    double e_scale  = 1.0;   ///< Error normalisation: maps ±e_scale to universe [-1, 1].
    double de_scale = 1.0;   ///< Error-rate normalisation: maps ±de_scale to universe [-1, 1].
    double u_scale  = 1.0;   ///< Output scaling: universe [-1, 1] → ±u_scale.
    double uMin     = -1e9;  ///< Hard lower output clamp.
    double uMax     =  1e9;  ///< Hard upper output clamp.
};

/**
 * @brief 5-term PD fuzzy controller implementing IController.
 *
 * Inputs: error e[k] and error-rate de[k] = (e[k] − e[k−1]) / Ts.
 * Output: control effort u[k].
 *
 * Uses a standard 5-term partition (NL, NS, ZE, PS, PL) with triangular and
 * shoulder membership functions and a 25-rule canonical diagonal Mamdani table.
 *
 * **Scaling:**
 * @code
 *   e_norm  = e  / e_scale   ∈ [-1, 1]
 *   de_norm = de / de_scale  ∈ [-1, 1]
 *   u       = u_norm * u_scale
 * @endcode
 */
class FuzzyPD : public IController {
public:
    /**
     * @brief Construct a FuzzyPD with given parameters and sample time.
     * @param p          Scaling and output limit parameters.
     * @param sampleTime Ts [s]; used to compute error-rate de = (e − e_prev) / Ts.
     */
    explicit FuzzyPD(const FuzzyPDParams& p, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k].
     * @param error Tracking error e[k] = r[k] − y[k].
     * @return Control effort u[k].
     */
    double compute(double error) override;

    /** @brief Reset error history and previous output to zero. */
    void   reset()            override;
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Update parameters and rebuild the internal FuzzySystem.
     * @param p New FuzzyPDParams.
     */
    void                 setParams(const FuzzyPDParams& p);
    const FuzzyPDParams& params()  const { return p_; }

    /** @brief Last computed control output u[k−1]. */
    double lastOutput() const { return u_prev_; }

private:
    FuzzyPDParams p_;
    double        Ts_;
    double        e_prev_;
    double        u_prev_;
    FuzzySystem   sys_;

    void buildSystem();
};


// ─────────────────────────────────────────────────────────────────────────────
// FuzzyPID — FuzzyPD augmented with crisp integral + anti-windup
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Tuning parameters for FuzzyPID.
 */
struct FuzzyPIDParams {
    FuzzyPDParams pd;           ///< FuzzyPD scaling (e_scale, de_scale, u_scale).
    double        Ki   = 0.0;   ///< Integral gain (0 = pure FuzzyPD).
    double        Kb   = 1.0;   ///< Anti-windup back-calculation gain.
    double        uMin = -1e9;  ///< Hard lower output clamp.
    double        uMax =  1e9;  ///< Hard upper output clamp.
};

/**
 * @brief FuzzyPD extended with a crisp integral term and back-calculation anti-windup.
 *
 * Architecture — three decoupled blocks:
 * @code
 *   u_P  = FuzzyPD(e, de)
 *   u_I += Ki * Ts * e  +  Kb * (u_sat − u_unsat)     // back-calculation anti-windup
 *   u    = clamp(u_P + u_I, uMin, uMax)
 * @endcode
 *
 * Setting Ki = 0 recovers pure FuzzyPD behaviour.
 */
class FuzzyPID : public IController {
public:
    /**
     * @brief Construct a FuzzyPID with given parameters and sample time.
     * @param p          FuzzyPID tuning parameters.
     * @param sampleTime Ts [s].
     */
    explicit FuzzyPID(const FuzzyPIDParams& p, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k].
     * @param error Tracking error e[k].
     * @return Control effort u[k].
     */
    double compute(double error) override;

    /** @brief Reset integral accumulator, error history, and previous output to zero. */
    void   reset()            override;
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Update parameters (rebuilds internal FuzzyPD).
     * @param p New FuzzyPIDParams.
     */
    void               setParams(const FuzzyPIDParams& p);
    const FuzzyPIDParams& params() const { return p_; }

    /** @brief Last computed control output u[k−1]. */
    double lastOutput() const { return u_prev_; }

    /**
     * @brief Bumpless initialisation: set integral so next compute(error) ≈ u_target.
     * @param u_target  Desired initial output.
     * @param error     Current tracking error.
     */
    void bumplessInit(double u_target, double error) override;

private:
    FuzzyPIDParams p_;
    double         Ts_;
    FuzzyPD        pd_block_;
    double         integral_;
    double         u_prev_;
};


// ─────────────────────────────────────────────────────────────────────────────
// FuzzySupervisor — performance monitor for adaptive relinearisation
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Tuning parameters for FuzzySupervisor.
 */
struct SupervisorParams {
    double e_threshold     = 5.0;   ///< |e| at which error is considered "Large" [same units as error].
    double trend_threshold = 0.5;   ///< d|e|/dt considered "Increasing fast" [units/s].
    double signal_threshold = 0.5;  ///< relinearize_signal > this value triggers the action flag.

    /**
     * @brief Hysteresis cooldown.
     *
     * Once a relinearisation is triggered, the supervisor will not trigger again
     * for @c cooldown_steps control steps.
     */
    int    cooldown_steps  = 20;
};

/**
 * @brief Output of a FuzzySupervisor::update() call.
 */
struct SupervisorDecision {
    double relinearize_signal; ///< Raw fuzzy output ∈ [0, 1].
    bool   relinearize;        ///< true when signal > signal_threshold and cooldown has expired.
    double error_norm;         ///< Normalised |e| / e_threshold used as fuzzy input.
    double trend;              ///< Normalised d|e|/dt / trend_threshold used as fuzzy input.
};

/**
 * @brief Fuzzy supervisory monitor that decides when a linearised controller needs re-tuning.
 *
 * Decision logic (2-input → 1-output Mamdani fuzzy system):
 * - **Input 1:** normalised_error = |e| / e_threshold; terms: Small, Medium, Large.
 * - **Input 2:** error_trend = d|e|/dt / trend_threshold; terms: Decreasing, Steady, Increasing.
 * - **Output:** relinearize_signal ∈ [0, 1]; 0 = no action, 1 = force relinearisation.
 *
 * Representative rules (full 9-rule table built internally):
 * @code
 *   IF error is Small  AND trend is Decreasing  THEN signal is None  (0.0)
 *   IF error is Medium AND trend is Increasing  THEN signal is Maybe (0.5)
 *   IF error is Large  AND trend is Increasing  THEN signal is Full  (1.0)
 * @endcode
 *
 * **Typical usage in an adaptive MPC loop:**
 * @code
 *   auto sig = supervisor.update(abs_error);
 *   if (sig.relinearize)
 *       mpc.setPlant(relinearize(plant, current_state));
 * @endcode
 */
class FuzzySupervisor {
public:
    /**
     * @brief Construct the supervisor with given parameters and sample time.
     * @param p          Supervisor tuning parameters.
     * @param sampleTime Ts [s]; used to compute error trend de/dt.
     */
    explicit FuzzySupervisor(const SupervisorParams& p, double sampleTime);

    /**
     * @brief Feed the current absolute position or output error.
     *
     * Computes error_norm and trend, runs inference, and applies the cooldown mechanism.
     *
     * @param abs_error Absolute scalar error |e[k]| [same units as e_threshold].
     * @return SupervisorDecision with the fuzzy signal and boolean trigger flag.
     */
    SupervisorDecision update(double abs_error);

    /** @brief Reset error history and cooldown counter to zero. */
    void reset();

    const SupervisorParams& params() const { return p_; }

    /**
     * @brief Update supervisor parameters at runtime.
     * @param p New SupervisorParams.
     */
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
