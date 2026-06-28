#pragma once
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

/**
 * @file LPSolver.h
 * @brief Header-only two-phase simplex solver for bounded-variable linear programs (Phase 3 OC4).
 *
 * Solves:
 * @code
 *   min_{x}   c'x
 *   s.t.      A_ineq*x <= b_ineq   (optional, A_ineq.rows() == 0 is valid)
 *             A_eq*x   == b_eq     (optional, A_eq.rows() == 0 is valid)
 *             lb <= x <= ub        (required, finite -- this toolbox's "unbounded" convention
 *                                    is +-1e9, e.g. BacksteppingParams::uMin/uMax)
 * @endcode
 *
 * **Bound magnitudes above ~1e6 are clamped internally, not honored exactly.** `lb`/`ub` are
 * clamped to `+-1e6` *before* anything else is computed -- not just inside the box row, but in
 * the variable shift `y = x - lb` itself, since that shift's literal magnitude is injected into
 * every row referencing that variable's column. Skipping this and using the literal `+-1e9`
 * sentinel (this toolbox's "unbounded" convention) costs ~9 orders of magnitude of float64
 * precision to every Gauss-Jordan step that touches an affected row or column, via catastrophic
 * cancellation -- caught empirically as both a false `Infeasible` (a trivially feasible textbook
 * LP's Phase-1 residual landing at ~1e-7 against tol=1e-8) and a genuine wrong answer (an LPMPC
 * instance's Phase-1 "optimum" landing at ~3e9, not noise). This is safe for every realistic
 * control-engineering bound (`uMin`/`uMax`/etc. in this codebase are always far below 1e6 in
 * magnitude) but means a problem whose *true* optimal `x_i` needs to exceed ~1e6 will get a
 * silently wrong (clamped) answer -- this solver is not intended for that scale.
 *
 * **Not an extension of GradientProjectionQP.** That solver's only constraint handling is
 * box-clamping (`cwiseMax(lb).cwiseMin(ub)`) inside a first-order FISTA iteration -- it has no
 * general-inequality machinery and does not produce exact LP vertices. This is a from-scratch
 * two-phase simplex; see `docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md` for the
 * pre-implementation audit that corrected the roadmap's original "extends GradientProjectionQP"
 * claim.
 *
 * **Algorithm (two-phase simplex, Bland's rule, dense tableau):**
 * 1. Shift every variable to be nonnegative: `y = x - lb >= 0`. The upper bound becomes an
 *    extra inequality row `y_i <= ub_i - lb_i` (n extra rows). This keeps the tableau in the
 *    textbook all-variables->=0 form instead of requiring a bounded-variable revised simplex.
 * 2. Standardize: every inequality row gets a slack column; every row (after sign-normalizing
 *    so its RHS is >= 0) gets one artificial-variable column.
 * 3. **Phase 1** minimizes the sum of artificials. If the optimum exceeds `tol`, the problem is
 *    infeasible.
 * 4. **Phase 2** recomputes the reduced-cost row from the real cost vector and current basis,
 *    then continues simplex pivoting with artificial columns permanently excluded from entering.
 * 5. Bland's rule (smallest-index entering column, minimum-ratio leaving row with smallest-basis-
 *    index tie-break) guarantees no cycling -- required for the "infeasible LP must not loop
 *    forever" contract.
 *
 * **On `LPStatus::Unbounded` being structurally rare:** every variable's own box row gives it an
 * explicit upper bound, so the LP's feasible region (whenever nonempty) is always a subset of a
 * bounded box -- a bounded polytope cannot have an unbounded objective. `Unbounded` is kept as a
 * defensive, separately-reported status (distinct from `IterationLimit`) for a state that should
 * not arise for a problem honoring the box-bound contract, not a primary code path.
 *
 * @see Dantzig, G.B. (1947/1963). Linear Programming and Extensions.
 * @see Bland, R.G. (1977). New finite pivoting rules for the simplex method. Math. Oper. Res. 2(2).
 */

namespace ctrl
{

/** @brief Terminal status of an LPSolver::solve() call. */
enum class LPStatus
{
    Optimal,        ///< x/cost are meaningful.
    Infeasible,     ///< No x satisfies all constraints (Phase 1 objective > tol).
    Unbounded,      ///< Cost decreases without bound (should not occur for a finite-box LPProblem).
    IterationLimit  ///< maxIter pivots exhausted before reaching Optimal/Infeasible/Unbounded.
};

/** @brief A bounded-variable linear program: minimize c'x s.t. A_ineq*x<=b_ineq, A_eq*x==b_eq, lb<=x<=ub. */
struct LPProblem
{
    Eigen::VectorXd c;        ///< Cost vector (n x 1).
    Eigen::MatrixXd A_ineq;   ///< (m_ineq x n); 0 rows is valid (no inequality constraints).
    Eigen::VectorXd b_ineq;   ///< (m_ineq x 1).
    Eigen::MatrixXd A_eq;     ///< (m_eq x n); 0 rows is valid (no equality constraints).
    Eigen::VectorXd b_eq;     ///< (m_eq x 1).
    Eigen::VectorXd lb, ub;   ///< (n x 1) each; required finite bounds (use +-1e9 for "unbounded").
};

/** @brief Result of an LPSolver::solve() call. */
struct LPResult
{
    LPStatus        status = LPStatus::IterationLimit;
    Eigen::VectorXd x;       ///< Optimal point; meaningful only when status == Optimal.
    double          cost = 0.0; ///< c.dot(x); meaningful only when status == Optimal.
    int             iters = 0;  ///< Total simplex pivots across both phases.
};

/**
 * @brief Two-phase simplex solver for bounded-variable linear programs.
 */
class LPSolver
{
public:
    /**
     * @brief Solve a bounded-variable LP via two-phase simplex.
     * @param problem The LP (see LPProblem).
     * @param maxIter Pivot budget shared across both phases.
     * @param tol     Feasibility/optimality tolerance (reduced-cost and ratio-test floor).
     * @return LPResult with status/x/cost/iters.
     * @throws std::invalid_argument If c is empty, lb/ub sizes mismatch c, any lb(i) > ub(i),
     *         or A_ineq/A_eq sizes mismatch their respective b vector or c's dimension.
     */
    static LPResult solve(const LPProblem &problem, int maxIter = 200, double tol = 1e-8)
    {
        const int n = static_cast<int>(problem.c.size());
        if (n < 1)
            throw std::invalid_argument("LPSolver::solve: c must have at least one element");
        if (problem.lb.size() != n || problem.ub.size() != n)
            throw std::invalid_argument("LPSolver::solve: lb/ub size mismatch with c");
        for (int i = 0; i < n; ++i)
            if (problem.lb(i) > problem.ub(i))
                throw std::invalid_argument("LPSolver::solve: lb(i) > ub(i)");

        const int m_ineq_orig = static_cast<int>(problem.b_ineq.size());
        const int m_eq        = static_cast<int>(problem.b_eq.size());
        if (m_ineq_orig > 0 &&
            (problem.A_ineq.rows() != m_ineq_orig || problem.A_ineq.cols() != n))
            throw std::invalid_argument("LPSolver::solve: A_ineq size mismatch");
        if (m_eq > 0 && (problem.A_eq.rows() != m_eq || problem.A_eq.cols() != n))
            throw std::invalid_argument("LPSolver::solve: A_eq size mismatch");

        // Clamp lb/ub to +-1e6 *before* shifting, not just inside the box row. The shift
        // y = x - lb (used to map every variable to y >= 0) injects lb's literal magnitude into
        // EVERY row that references that variable's column -- not just its own box row -- so a
        // sentinel lb=-1e9 (this toolbox's "unbounded" convention, e.g. LPMPCParams::duMin's
        // default) contaminates the whole tableau with ~1e9-magnitude numbers, not just one row.
        // An earlier version of this fix only capped the box row's RHS and left the shift itself
        // unbounded; it still failed (Phase-1 "optimum" landing at ~3e9, not infeasibility, not
        // precision noise -- a real wrong-answer bug, not residual rounding). Clamping lb/ub
        // *before* computing anything keeps every number in the tableau within ~2e6 in magnitude,
        // for a Phase-1 residual safely under tol. Documented consequence: a problem whose true
        // optimal x_i needs |x_i| > 1e6 gets a silently wrong (clamped) answer -- this solver is
        // not intended for that scale (see the file-level doc comment).
        constexpr double kEffectiveBoundCap = 1e6;
        const Eigen::VectorXd lb_eff = problem.lb.cwiseMax(-kEffectiveBoundCap);
        // .cwiseMax(lb_eff) is a safety net for the pathological case where the *entire* true
        // [lb,ub] interval sits beyond the cap on one side (e.g. lb=5e6, ub=9e6): without it,
        // ub_eff could land below lb_eff and produce a malformed negative-range row. Collapsing
        // to a zero-width interval at lb_eff is consistent with the documented "outside +-1e6 ->
        // clamped, not exact" limitation rather than a structurally broken tableau.
        const Eigen::VectorXd ub_eff = problem.ub.cwiseMin(kEffectiveBoundCap).cwiseMax(lb_eff);

        const int m_ineq_aug = m_ineq_orig + n; // original rows + n box upper-bound rows
        const int m_total    = m_ineq_aug + m_eq;
        const int slack_start = n;
        const int art_start   = n + m_ineq_aug;
        const int N           = art_start + m_total; // y | slack | artificial

        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(m_total + 1, N + 1);
        std::vector<int> basis(static_cast<size_t>(m_total));

        int row       = 0;
        int slack_col = slack_start;
        int art_col   = art_start;

        // Append one standardized row: a_y'*y {<=,==} rhs, optionally with its own slack column.
        // Sign-normalizes so the row's RHS is >= 0 *before* attaching the artificial column, so
        // the artificial's initial value (= RHS) is always a valid nonnegative basic value.
        auto addRow = [&](const Eigen::RowVectorXd &a_y, double rhs, bool has_slack)
        {
            T.block(row + 1, 0, 1, n) = a_y;
            if (has_slack)
            {
                T(row + 1, slack_col) = 1.0;
                ++slack_col;
            }
            if (rhs < 0.0)
            {
                T.row(row + 1) *= -1.0;
                rhs = -rhs;
            }
            T(row + 1, art_col) = 1.0;
            T(row + 1, N)       = rhs;
            basis[static_cast<size_t>(row)] = art_col;
            ++art_col;
            ++row;
        };

        for (int i = 0; i < m_ineq_orig; ++i)
            addRow(problem.A_ineq.row(i), problem.b_ineq(i) - problem.A_ineq.row(i) * lb_eff, true);

        for (int i = 0; i < n; ++i)
        {
            Eigen::RowVectorXd e_i = Eigen::RowVectorXd::Zero(n);
            e_i(i) = 1.0;
            addRow(e_i, ub_eff(i) - lb_eff(i), true);
        }

        for (int i = 0; i < m_eq; ++i)
            addRow(problem.A_eq.row(i), problem.b_eq(i) - problem.A_eq.row(i) * lb_eff, false);

        // Phase 1 objective row: minimize sum(artificials). w_j = 1 for artificial columns, else 0.
        // row0[j] = w_j - sum_i T(i+1,j)  (reduced cost relative to the all-artificial basis).
        for (int j = 0; j <= N; ++j)
        {
            double col_sum = 0.0;
            for (int i = 0; i < m_total; ++i)
                col_sum += T(i + 1, j);
            const bool is_artificial = (j >= art_start && j < N);
            T(0, j) = (is_artificial ? 1.0 : 0.0) - col_sum;
        }

        int total_iters = 0;
        LPStatus status = LPStatus::IterationLimit;

        // Phase 1: drive the artificial sum to zero (or detect infeasibility).
        bool phase1_reached_optimal = false;
        for (; total_iters < maxIter; ++total_iters)
        {
            int q = -1;
            for (int j = 0; j < N; ++j)
                if (T(0, j) < -tol) { q = j; break; }
            if (q < 0) { phase1_reached_optimal = true; break; } // Phase-1 optimal

            const int p_row = pivotRow(T, basis, q, m_total, N, tol);
            if (p_row < 0) { phase1_reached_optimal = true; break; } // defensive: see "Unbounded" doc note
            pivot(T, p_row + 1, q);
            basis[static_cast<size_t>(p_row)] = q;
        }

        // Budget exhausted before Phase 1 resolved -- this is NOT proof of infeasibility, just an
        // unresolved search; report IterationLimit rather than mislabeling it Infeasible.
        if (!phase1_reached_optimal)
            return LPResult{LPStatus::IterationLimit, Eigen::VectorXd::Zero(n), 0.0, total_iters};

        // Feasibility uses a looser floor than the caller's `tol`, decoupled from it: `tol` also
        // governs the ratio-test eligibility cutoff and the Phase-2 optimality check, where a
        // tight value is exactly what a caller asking for high precision wants. But the Phase-1
        // objective is a *sum* of m_total artificials, each accumulating Gauss-Jordan rounding
        // error across every pivot that touched its row -- caught empirically: a genuinely
        // feasible ~25-variable LP (LPMPC, Np=15/Nc=5) left phase1_obj=1.01e-8 after 65 pivots,
        // 1% over the default tol=1e-8, misreporting Infeasible. A literal `tol` floor doesn't
        // scale with pivot count; `std::max(tol, kFeasibilityFloor)` keeps a user's looser `tol`
        // intact while guaranteeing this floor for tight/default tol values.
        constexpr double kFeasibilityFloor = 1e-6;
        const double feas_tol  = std::max(tol, kFeasibilityFloor);
        const double phase1_obj = -T(0, N);
        if (phase1_obj > feas_tol)
            return LPResult{LPStatus::Infeasible, Eigen::VectorXd::Zero(n), 0.0, total_iters};

        // Phase 2: recompute reduced costs against the real objective and current basis.
        Eigen::RowVectorXd c2 = Eigen::RowVectorXd::Zero(N);
        c2.head(n) = problem.c.transpose();
        for (int j = 0; j <= N; ++j)
        {
            double basis_cost_dot_col = 0.0;
            for (int i = 0; i < m_total; ++i)
                basis_cost_dot_col += c2(basis[static_cast<size_t>(i)]) * T(i + 1, j);
            const double cj = (j < N) ? c2(j) : 0.0;
            T(0, j) = cj - basis_cost_dot_col;
        }

        for (; total_iters < maxIter; ++total_iters)
        {
            int q = -1;
            for (int j = 0; j < art_start; ++j) // artificials permanently excluded from entering
                if (T(0, j) < -tol) { q = j; break; }
            if (q < 0) { status = LPStatus::Optimal; break; }

            const int p_row = pivotRow(T, basis, q, m_total, N, tol);
            if (p_row < 0) { status = LPStatus::Unbounded; break; }
            pivot(T, p_row + 1, q);
            basis[static_cast<size_t>(p_row)] = q;
        }

        Eigen::VectorXd y_sol = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < m_total; ++i)
        {
            const int b = basis[static_cast<size_t>(i)];
            if (b < n) y_sol(b) = T(i + 1, N);
        }

        LPResult result;
        result.status = status;
        result.x      = lb_eff + y_sol;
        result.cost   = (status == LPStatus::Optimal) ? problem.c.dot(result.x) : 0.0;
        result.iters  = total_iters;
        return result;
    }

private:
    /**
     * @brief Minimum-ratio test for entering column @p q, Bland's-rule tie-break.
     * @return Leaving row index in [0, m_total), or -1 if no eligible row (unbounded direction).
     */
    static int pivotRow(const Eigen::MatrixXd &T, const std::vector<int> &basis,
                         int q, int m_total, int /*N*/, double tol)
    {
        int    p_row      = -1;
        double best_ratio = 0.0;
        for (int i = 0; i < m_total; ++i)
        {
            const double a_iq = T(i + 1, q);
            if (a_iq <= tol) continue;
            const double ratio = T(i + 1, T.cols() - 1) / a_iq;
            if (p_row < 0 || ratio < best_ratio - 1e-12 ||
                (std::abs(ratio - best_ratio) <= 1e-12 &&
                 basis[static_cast<size_t>(i)] < basis[static_cast<size_t>(p_row)]))
            {
                p_row      = i;
                best_ratio = ratio;
            }
        }
        return p_row;
    }

    /** @brief Gauss-Jordan elimination: normalize pivot row, clear pivot column elsewhere. */
    static void pivot(Eigen::MatrixXd &T, int prow, int q)
    {
        const double piv = T(prow, q);
        T.row(prow) /= piv;
        for (int i = 0; i < T.rows(); ++i)
        {
            if (i == prow) continue;
            const double factor = T(i, q);
            if (factor != 0.0)
                T.row(i) -= factor * T.row(prow);
        }
    }
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(lp_solver)
