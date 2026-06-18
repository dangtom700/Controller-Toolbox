/*
 * fuzzy_performance.h - Mamdani Fuzzy Performance Estimator
 * ===========================================================
 * Estimates a scalar performance score [0, 100] and a qualitative grade
 * from closed-loop metrics. Suitable for unknown-plant scenarios where an
 * analytical performance bound cannot be derived.
 *
 * Inputs (all normalized to fixed ranges):
 *   ise_norm  \in [0, 1]   - ISE / ISE_worst  (worst = open-loop drift)
 *   os_pct    \in [0, 100] - percentage overshoot
 *   st_norm   \in [0, 1]   - settling_time / simulation_duration
 *
 * Linguistic variables:
 *   ISE:       Low [0,0,0.2],   Medium [0.1,0.3,0.5],   High [0.4,1,1]
 *   Overshoot: Low [0,0,10],    Medium [5,15,30],        High [20,100,100]
 *   Settling:  Fast [0,0,0.3],  Medium [0.2,0.5,0.8],   Slow [0.6,1,1]
 *
 * Rules (Mamdani AND, min-activation, centroid defuzzification):
 *   1. ISE=Low  && OS=Low  && ST=Fast   -> 95  (Excellent)
 *   2. ISE=Low  && OS=Low  && ST=Med    -> 80  (Good)
 *   3. ISE=Low  && OS=Med  && ST=Fast   -> 75  (Good)
 *   4. ISE=Med  && OS=Low  && ST=Fast   -> 70  (Good)
 *   5. ISE=Low  && OS=Low  && ST=Slow   -> 65  (Fair)
 *   6. ISE=Med  && OS=Med  && ST=Med    -> 55  (Fair)
 *   7. ISE=Med  && OS=High && ST=Med    -> 40  (Poor)
 *   8. ISE=High && OS=Low  && ST=Med    -> 45  (Poor)
 *   9. ISE=High && OS=Med  && ST=Slow   -> 25  (Bad)
 *  10. ISE=High && OS=High && ST=Slow   -> 10  (Bad)
 *  11. ISE=Med  && OS=Low  && ST=Slow   -> 50  (Fair)
 *  12. ISE=Low  && OS=High && ST=any    -> 35  (Poor)
 *
 * Output: score \in [0,100], grade string.
 *
 * Defuzzification: weighted centroid over rule consequents (Sugeno-style
 * singleton consequents are equivalent here and are computationally simpler).
 */

#pragma once
#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace fuzzy {

// -------------------------------------------------------------------------
// Membership functions
// -------------------------------------------------------------------------
inline double trimf(double x, double a, double b, double c) {
    if (x <= a || x >= c) return 0.0;
    if (b == a && x == a) return 1.0;
    if (b == c && x == c) return 1.0;
    if (x <= b && (b - a) > 0.0) return (x - a) / (b - a);
    if ((c - b) > 0.0)           return (c - x) / (c - b);
    return 0.0;
}

// One-sided rising ramp: 0 for x<=a, 1 for x>=b
inline double rising_mf(double x, double a, double b) {
    if (x <= a) return 0.0;
    if (x >= b) return 1.0;
    return (x - a) / (b - a);
}

// One-sided falling ramp: 1 for x<=a, 0 for x>=b
inline double falling_mf(double x, double a, double b) {
    if (x <= a) return 1.0;
    if (x >= b) return 0.0;
    return (b - x) / (b - a);
}

// -------------------------------------------------------------------------
// Compute membership degrees for ISE_norm
// -------------------------------------------------------------------------
struct ISEMembership {
    double low, medium, high;
    explicit ISEMembership(double ise_norm) {
        low    = falling_mf(ise_norm, 0.0,  0.2)
               + trimf(ise_norm, 0.0, 0.0, 0.2);   // saturate at 1 for x<=0
        low    = falling_mf(ise_norm, 0.0,  0.25);
        medium = trimf(ise_norm, 0.1,  0.30, 0.55);
        high   = rising_mf(ise_norm,  0.40, 0.80);
    }
};

// -------------------------------------------------------------------------
// Compute membership degrees for overshoot [%]
// -------------------------------------------------------------------------
struct OSMembership {
    double low, medium, high;
    explicit OSMembership(double os_pct) {
        low    = falling_mf(os_pct, 0.0,  12.0);
        medium = trimf(os_pct, 5.0,  15.0, 32.0);
        high   = rising_mf(os_pct, 22.0, 40.0);
    }
};

// -------------------------------------------------------------------------
// Compute membership degrees for settling time (normalised)
// -------------------------------------------------------------------------
struct STMembership {
    double fast, medium, slow;
    explicit STMembership(double st_norm) {
        fast   = falling_mf(st_norm, 0.0,  0.30);
        medium = trimf(st_norm, 0.20, 0.50, 0.80);
        slow   = rising_mf(st_norm,  0.60, 0.90);
    }
};

// -------------------------------------------------------------------------
// Fuzzy inference engine (Mamdani, singleton consequents, weighted centroid)
// -------------------------------------------------------------------------
struct FuzzyResult {
    double score;     // [0, 100]
    std::string grade;
    double confidence; // sum of rule activation weights / max_possible
};

inline FuzzyResult evaluate(double ise_norm, double os_pct, double st_norm) {
    // Clamp inputs to valid ranges
    ise_norm = std::clamp(ise_norm, 0.0, 1.0);
    os_pct   = std::clamp(os_pct,   0.0, 100.0);
    st_norm  = std::clamp(st_norm,  0.0, 1.0);

    ISEMembership ise(ise_norm);
    OSMembership  os(os_pct);
    STMembership  st(st_norm);

    // Rule activations (Mamdani AND = min)
    auto AND = [](double a, double b, double c = 1.0) {
        return std::min({a, b, c});
    };

    struct Rule { double w; double c; };  // weight, consequent singleton

    Rule rules[] = {
        { AND(ise.low,    os.low,    st.fast),   95.0 },  //  1 Excellent
        { AND(ise.low,    os.low,    st.medium), 80.0 },  //  2 Good
        { AND(ise.low,    os.medium, st.fast),   75.0 },  //  3 Good
        { AND(ise.medium, os.low,    st.fast),   70.0 },  //  4 Good
        { AND(ise.low,    os.low,    st.slow),   65.0 },  //  5 Fair
        { AND(ise.medium, os.low,    st.medium), 60.0 },  //  6 Fair
        { AND(ise.medium, os.medium, st.medium), 55.0 },  //  7 Fair
        { AND(ise.medium, os.medium, st.slow),   42.0 },  //  8 Poor
        { AND(ise.medium, os.high,   st.medium), 38.0 },  //  9 Poor
        { AND(ise.high,   os.low,    st.medium), 45.0 },  // 10 Poor
        { AND(ise.high,   os.medium, st.slow),   25.0 },  // 11 Bad
        { AND(ise.high,   os.high,   st.slow),   10.0 },  // 12 Bad
        { AND(ise.low,    os.high,   1.0),        35.0 },  // 13 Poor (high OS always hurts)
    };

    double wsum = 0.0, wcsum = 0.0;
    for (const auto& r : rules) {
        wsum  += r.w;
        wcsum += r.w * r.c;
    }

    double score = (wsum > 1e-9) ? (wcsum / wsum) : 50.0;
    score = std::clamp(score, 0.0, 100.0);

    std::string grade;
    if      (score >= 85.0) grade = "Excellent";
    else if (score >= 68.0) grade = "Good";
    else if (score >= 50.0) grade = "Fair";
    else if (score >= 30.0) grade = "Poor";
    else                    grade = "Bad";

    double confidence = std::clamp(wsum, 0.0, 1.0);
    return { score, grade, confidence };
}

// -------------------------------------------------------------------------
// Print a formatted fuzzy evaluation report
// -------------------------------------------------------------------------
inline void print_report(const std::string& label,
                          double ise_norm, double os_pct, double st_norm,
                          const FuzzyResult& r) {
    std::cout << std::fixed;
    std::cout << "\n  [Fuzzy Estimator] " << label << "\n"
              << "    Inputs : ISE_norm=" << std::setprecision(4) << ise_norm
              << "  OS=" << std::setprecision(1) << os_pct << "%"
              << "  ST_norm=" << std::setprecision(4) << st_norm << "\n"
              << "    Score  : " << std::setprecision(1) << r.score << " / 100"
              << "  Grade: " << r.grade
              << "  Confidence: " << std::setprecision(2) << r.confidence * 100.0 << "%\n";
}

// -------------------------------------------------------------------------
// Convenience: compute ISE_norm and ST_norm from raw values
// ise_worst = baseline ISE (e.g., open-loop or worst observed)
// T_sim     = total simulation time [s]
// -------------------------------------------------------------------------
inline FuzzyResult evaluate_raw(double ise, double ise_worst,
                                 double os_pct,
                                 double settling_time, double T_sim) {
    double ise_norm = (ise_worst > 0) ? std::min(ise / ise_worst, 1.0) : 0.5;
    double st_norm  = (T_sim     > 0) ? std::min(settling_time / T_sim, 1.0) : 0.5;
    return evaluate(ise_norm, os_pct, st_norm);
}

} // namespace fuzzy
