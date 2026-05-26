#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>
#include <optional>

// =============================================================================
// Discrete-Time H-infinity Controller Synthesis  (DGKF two-Riccati method)
//
// Synthesises a discrete-time dynamic output-feedback controller K(z) that
// minimises the closed-loop H-infinity norm:
//
//   ||F_l(P, K)||_inf < gamma
//
// where P is the generalised plant (augmented with weighting functions) and
// F_l denotes the lower Linear Fractional Transformation.
//
// -- Generalised Plant Format ------------------------------------------------
//
//   P: x[k+1] = A x[k] + B1 w[k] + B2 u[k]
//       z[k]  = C1 x[k] + D11 w[k] + D12 u[k]   (performance output)
//       y[k]  = C2 x[k] + D21 w[k] + D22 u[k]   (measurement output)
//
//   w = exogenous input (disturbances, references)
//   z = performance output (weighted tracking error, control effort)
//   u = control input
//   y = measured output
//
// -- Mixed-Sensitivity Design ------------------------------------------------
//
// The most common use case: S/KS/T mixed sensitivity.  Use MixedSensitivity
// (below) to build P automatically from a SISO plant G and three weights W1
// (tracking), W2 (control effort), W3 (complementary sensitivity/robustness).
//
//   ||[ W1 S  ]||
//   ||[ W2 KS ]||  < gamma
//   ||[ W3 T  ]||  inf
//
// -- Algorithm ---------------------------------------------------------------
//
// Discrete DGKF (Doyle, Glover, Khargonekar & Francis 1989 - discrete version
// by Stoorvogel 1992 / Iglesias & Glover 1991):
//
//  Assumptions (relaxed form - checked in solve()):
//   (A1) (A, B2) is stabilisable,   (A, C2) is detectable
//   (A2) D12 has full column rank,  D21 has full row rank
//   (A3) D22 = 0  (standard form; MixedSensitivity guarantees this)
//        D11 may be nonzero; gamma must exceed ||D11||_2 (Iglesias & Glover 1991, Lemma 2.1)
//
//  Step 1: solve the Control DARE for X_inf:
//    A' X A - X - A' X B (R_tilde)^{-1} B' X A + Q_tilde = 0
//    where  B = [B1 B2],  R_tilde = [I+D'D  D12'], Q_tilde = C1'C1
//
//  Step 2: solve the Filter DARE for Y_inf:
//    A Y A' - Y - A Y C (R_hat)^{-1} C Y A' + Q_hat = 0
//    where  C = [C1; C2],  Q_hat = B1 B1'
//
//  Step 3: compute controller matrices from X_inf, Y_inf.
//    K has order n (same as the plant).
//
//  The gamma iteration (bisection on gamma) finds the minimum achievable H_inf
//  norm within a specified tolerance.
//
// -- Usage -------------------------------------------------------------------
//
//   // 1. Build augmented plant (use MixedSensitivity helper for S/KS/T design)
//   ctrl::GeneralisedPlant P = ctrl::MixedSensitivity::build(G, W1, W2, W3);
//
//   // 2. Synthesise controller
//   ctrl::HinfParams hp;
//   hp.gammaInit = 2.0;   // initial gamma (bisect down from here)
//   hp.gammaTol  = 0.01;  // stop when gamma bracket < 0.01
//   auto result = ctrl::DiscreteHinf::solve(P, hp);
//
//   // 3. Check feasibility
//   if (result.feasible) {
//       ctrl::DiscreteHinf ctrl(result);
//       double u = ctrl.compute(y_measurement);
//   }
//
// References:
//   Doyle, Glover, Khargonekar & Francis, IEEE TAC 34(8), 1989.
//   Stoorvogel, "The Discrete-Time H-inf Control Problem," SIAM 1992.
//   Iglesias & Glover, "State-space approach to discrete-time H-inf control,"
//     Int. J. Control 54(5), 1991.
//   Skogestad & Postlethwaite, "Multivariable Feedback Design" Ch. 9, 2005.
//   MATLAB hinfsyn() documentation.
// =============================================================================

namespace ctrl
{

// ---------------------------------------------------------------------------
// Generalised Plant
//   x[k+1] = A  x + B1 w + B2 u
//   z[k]   = C1 x + D11 w + D12 u
//   y[k]   = C2 x + D21 w + D22 u
//
// Dimensions:
//   n  = state size
//   nw = exogenous input size (disturbances + references)
//   nu = control input size
//   nz = performance output size (weighted signals)
//   ny = measurement output size
// ---------------------------------------------------------------------------
struct GeneralisedPlant
{
    Eigen::MatrixXd A;    // n  * n
    Eigen::MatrixXd B1;   // n  * nw
    Eigen::MatrixXd B2;   // n  * nu
    Eigen::MatrixXd C1;   // nz * n
    Eigen::MatrixXd C2;   // ny * n
    Eigen::MatrixXd D11;  // nz * nw
    Eigen::MatrixXd D12;  // nz * nu
    Eigen::MatrixXd D21;  // ny * nw
    Eigen::MatrixXd D22;  // ny * nu
    double Ts = 0.01;

    int stateSize()  const { return static_cast<int>(A.rows()); }
    int nw()         const { return static_cast<int>(B1.cols()); }
    int nu()         const { return static_cast<int>(B2.cols()); }
    int nz()         const { return static_cast<int>(C1.rows()); }
    int ny()         const { return static_cast<int>(C2.rows()); }
};

// ---------------------------------------------------------------------------
// H-infinity synthesis parameters
// ---------------------------------------------------------------------------
struct HinfParams
{
    double gammaInit  = 10.0;  // Initial (upper bound) gamma for bisection
    double gammaTol   = 1e-3;  // Stop bisection when gamma bracket < this
    int    maxIter    = 60;    // Maximum bisection iterations
    double dareTol    = 1e-12; // DARE convergence tolerance
    int    dareMaxIter = 200;  // DARE iteration limit
};

// ---------------------------------------------------------------------------
// H-infinity synthesis result
// ---------------------------------------------------------------------------
struct HinfResult
{
    bool   feasible  = false;  // true when synthesis succeeded for achievedGamma
    double achievedGamma = 0.0;// smallest feasible gamma found by bisection
    double Ts = 0.0;           // plant sample time (propagated from GeneralisedPlant)

    // Controller state-space matrices (nu * ny SISO or MIMO dynamic controller)
    // K(z): xk[k+1] = Ak xk[k] + Bk y[k]
    //        u[k]   = Ck xk[k] + Dk y[k]
    Eigen::MatrixXd Ak; // nk * nk  (nk == plant state size n)
    Eigen::MatrixXd Bk; // nk * ny
    Eigen::MatrixXd Ck; // nu * nk
    Eigen::MatrixXd Dk; // nu * ny

    // DARE solutions (available for diagnostics)
    Eigen::MatrixXd X_inf; // Control Riccati solution
    Eigen::MatrixXd Y_inf; // Filter  Riccati solution

    // Synthesis diagnostics
    int    dareItersX   = 0;
    int    dareItersY   = 0;
    bool   dareConvX    = false;
    bool   dareConvY    = false;
    double spectralRadius = 0.0; // rho(X_inf * Y_inf) - must be < gamma^2
};

// ---------------------------------------------------------------------------
// DiscreteHinf - dynamic output-feedback H-inf controller
//
// Implements K(z):
//   xk[k+1] = Ak xk[k] + Bk y[k]
//    u[k]   = Ck xk[k] + Dk y[k]
//
// where y[k] is the measurement vector from the plant.
// For SISO plants produced by MixedSensitivity, ny = 1 and nu = 1.
// ---------------------------------------------------------------------------
class DiscreteHinf : public IController
{
public:
    // Construct from a completed synthesis result (result.feasible must be true).
    explicit DiscreteHinf(const HinfResult &result);

    // -------------------------------------------------------------------
    // Synthesis (static factory)
    // -------------------------------------------------------------------
    // Solve the discrete-time H-inf optimal control problem for the given
    // generalised plant P and parameter set.  Returns a HinfResult; check
    // result.feasible before constructing a DiscreteHinf.
    //
    // The algorithm bisects gamma from [gammaLow, params.gammaInit] until
    // the bracket is narrower than params.gammaTol.  gammaLow is set
    // automatically to a numerically safe lower bound.
    static HinfResult solve(const GeneralisedPlant &P, const HinfParams &params = {});

    // -------------------------------------------------------------------
    // IController SISO interface (ny == 1, nu == 1)
    // signal = measurement output y[k]  (NOT tracking error)
    // Returns control input u[k].
    // -------------------------------------------------------------------
    double compute(double signal) override;

    // MIMO interface: y is the ny-dimensional measurement vector.
    Eigen::VectorXd computeVec(const Eigen::VectorXd &y) override;

    void   reset()            override;
    double sampleTime() const override { return Ts_; }

    // Controller state (nk-dimensional).
    const Eigen::VectorXd &controllerState() const { return xk_; }

    // Achieved H-inf bound from synthesis.
    double achievedGamma() const { return gamma_; }

    // Read-only access to controller matrices.
    const Eigen::MatrixXd &Ak() const { return Ak_; }
    const Eigen::MatrixXd &Bk() const { return Bk_; }
    const Eigen::MatrixXd &Ck() const { return Ck_; }
    const Eigen::MatrixXd &Dk() const { return Dk_; }

private:
    Eigen::MatrixXd Ak_, Bk_, Ck_, Dk_;
    Eigen::VectorXd xk_;   // controller internal state
    double Ts_;
    double gamma_;

    // Core DARE solver (doubling algorithm - same implementation strategy as DiscreteLQR).
    // Solves: A'XA - X - A'XB(R+B'XB)^{-1}B'XA + Q = 0
    // where R may be indefinite (the gamma-scaled Hamiltonian form).
    // Returns {solution, converged, iterations}.
    struct DareOut { Eigen::MatrixXd X; bool conv; int iters; };
    static DareOut solveHinfDARE(const Eigen::MatrixXd &A,
                                 const Eigen::MatrixXd &B,
                                 const Eigen::MatrixXd &Q,
                                 const Eigen::MatrixXd &R,
                                 double tol, int maxIter);

    // Attempt synthesis at a fixed gamma. Returns true if all conditions hold.
    static bool trySolve(const GeneralisedPlant &P, double gamma,
                         double dareTol, int dareMaxIter,
                         HinfResult &out);
};


// =============================================================================
// MixedSensitivity - Helper that builds a generalised plant for S/KS/T design
//
// Augments a SISO discrete-time plant G(z) with three weighting transfer
// functions (as state-space models) to form:
//
//   Objective:  ||[ W1(z).S(z)  ]||
//               ||[ W2(z).K(z)S ]||  < gamma
//               ||[ W3(z).T(z)  ]||  inf
//
// where S = 1/(1+GK),  T = GK/(1+GK),  KS = K.S.
//
// -- Weight Guidelines -------------------------------------------------------
//
//   W1 (tracking / disturbance rejection):
//     - High gain at low frequencies -> tight reference tracking and disturbance rejection.
//     - Typical: W1 = (s/M + omega_B) / (s + omega_B*eps)
//       => crossover at omega_B, low-freq gain M, DC gain 1/eps.
//     - Discrete: use a 1st or 2nd order filter discretised with Tustin or ZOH.
//
//   W2 (control effort / actuator limits):
//     - Limits high-frequency control action; prevents actuator saturation.
//     - Typical: constant W2 = 1/u_max, or a high-pass with rolloff at omega_u.
//
//   W3 (robustness / unmodelled dynamics):
//     - Complements S; ensures T rolls off at high frequencies.
//     - Typical: W3 = (s + omega_B/Mt) / (eps.s + omega_B)
//       => rolls up above omega_B with asymptotic gain 1/eps.
//
// -- Usage -------------------------------------------------------------------
//
//   // Plant: G(s) = 1/(s+1), ZOH at Ts = 0.01
//   auto G = ctrl::c2d(G_continuous, 0.01, ctrl::C2dMethod::ZOH);
//
//   // W1: integrating weight, crossover at 10 rad/s, eps=0.001
//   auto W1 = ctrl::MixedSensitivity::makeW1(10.0, 1.0, 1e-3, 0.01);
//
//   // W2: constant gain (limit control to +/- 5)
//   auto W2 = ctrl::MixedSensitivity::makeW2constant(1.0/5.0, 0.01);
//
//   // W3: high-frequency robustness, rollup above 50 rad/s
//   auto W3 = ctrl::MixedSensitivity::makeW3(50.0, 1.5, 1e-3, 0.01);
//
//   // Assemble augmented plant
//   auto P = ctrl::MixedSensitivity::build(G, W1, W2, W3);
//
//   // Synthesise
//   ctrl::HinfParams hp; hp.gammaInit = 5.0;
//   auto result = ctrl::DiscreteHinf::solve(P, hp);
//
// =============================================================================
class MixedSensitivity
{
public:
    // -----------------------------------------------------------------------
    // Weight factory functions (discrete-time, first-order state-space)
    // -----------------------------------------------------------------------

    // W1 - sensitivity weight for tracking/disturbance rejection.
    //   Low-freq gain  ~ 1/eps  (large, enforces tight tracking at DC)
    //   Crossover at   omega_B  [rad/s]
    //   High-freq gain ~ M      (upper bound on |W1| at high frequencies)
    //   Discretised with Tustin at sample time Ts [s].
    //
    //   Continuous: W1(s) = (s/M + omega_B) / (s + omega_B * eps)
    static StateSpace makeW1(double omega_B, double M, double eps, double Ts);

    // W2 - scalar constant weight (flat gain at all frequencies).
    //   Useful for simple actuator-level constraints: set gain = 1/u_max.
    static StateSpace makeW2constant(double gain, double Ts);

    // W2 - first-order high-pass weight for control effort roll-off.
    //   Low-freq gain  ~ eps   (allows large low-frequency control)
    //   High-freq gain ~ 1     (limits high-frequency control effort)
    //   Rolloff at     omega_u [rad/s]
    //   Continuous: W2(s) = (s + omega_u * eps) / (s/1 + omega_u)
    static StateSpace makeW2highpass(double omega_u, double eps, double Ts);

    // W3 - complementary-sensitivity weight for robustness.
    //   Low-freq gain  ~ eps   (allows T ~ 1 at low frequencies)
    //   High-freq gain ~ 1/Mt  (forces T rolloff above omega_T)
    //   Continuous: W3(s) = (s + omega_T / Mt) / (eps * s + omega_T)
    static StateSpace makeW3(double omega_T, double Mt, double eps, double Ts);

    // -----------------------------------------------------------------------
    // Build the generalised plant P from G, W1, W2, W3.
    // All inputs must be discrete-time (Ts > 0) with the same sample time.
    // G must be SISO.  W1, W2, W3 may be any order (first-order recommended).
    //
    // The augmented state is: xa = [xG; xW1; xW2; xW3]
    //
    // Generalised plant structure:
    //   Exogenous inputs w = [r; d]  (reference and output disturbance) - nw = 2
    //   Performance outputs z = [W1*(r-y); W2*u; W3*y]                - nz = 3
    //   Control input u                                                - nu = 1
    //   Measurement output y = G*u + d                                - ny = 1
    //
    // For a pure reference-tracking design, d is left connected but may be
    // set to zero during simulation.
    //
    // Troubleshooting when solve() returns feasible = false:
    //   1. Increase gammaInit: start at 10 * ||W1||_inf * ||W3||_inf as a rule of thumb.
    //   2. Check that W1's crossover frequency is below the plant's open-loop bandwidth.
    //      A W1 that demands high gain above the plant's roll-off frequency is infeasible.
    //   3. For non-minimum-phase plants (RHP zeros or excess delay), the achievable
    //      sensitivity peak S is bounded below by the Poisson integral; even a theoretically
    //      perfect controller cannot drive ||W1*S||_inf below this bound. Reduce W1 gain
    //      or relax the crossover requirement. See Skogestad & Postlethwaite Section 6.3.
    //   4. If W1 and W3 both have high gain in overlapping frequency bands, the design
    //      objective may be fundamentally infeasible (S + T = 1 sets a hard trade-off).
    //      Reduce |W1(jw)| * |W3(jw)| to satisfy the S-T complementarity constraint.
    //   5. If gammaInit is too small (below the achievable gamma), bisection cannot start.
    //      Increase gammaInit until solve() returns feasible = true, then tighten.
    // -----------------------------------------------------------------------
    static GeneralisedPlant build(const StateSpace &G,
                                  const StateSpace &W1,
                                  const StateSpace &W2,
                                  const StateSpace &W3);
};

} // namespace ctrl
