#include "NARMAXIdentifier.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

namespace {

constexpr int kMaxLibrarySize = 5000; ///< Safety cap on the candidate-term count.

/// Enumerate all monomials (multisets) of total degree 0..degree over p base regressors.
/// Index sequences are non-decreasing so each monomial appears exactly once; the empty
/// sequence is the constant term.
void enumerateCombos(int start, int p, int remaining, std::vector<int> &current,
                     std::vector<std::vector<int>> &out)
{
    if (remaining == 0)
    {
        out.push_back(current);
        return;
    }
    for (int i = start; i < p; ++i)
    {
        current.push_back(i);
        enumerateCombos(i, p, remaining - 1, current, out);
        current.pop_back();
    }
}

std::vector<std::vector<int>> generateTermFactors(int p, int degree)
{
    std::vector<std::vector<int>> out;
    out.push_back({}); // constant term
    std::vector<int> current;
    for (int d = 1; d <= degree; ++d)
    {
        if (p == 0) break;
        enumerateCombos(0, p, d, current, out);
    }
    return out;
}

/// Build the (Neff x p) base-regressor matrix and the (Neff) target.
/// Row r corresponds to time index k = maxLag + r.
void buildBase(const Eigen::VectorXd &u, const Eigen::VectorXd &y, const Eigen::VectorXd &e,
               int na, int nb, int nc, int maxLag, int Neff,
               Eigen::MatrixXd &base, Eigen::VectorXd &target)
{
    const int p = na + nb + nc;
    base.resize(Neff, p);
    target.resize(Neff);
    for (int r = 0; r < Neff; ++r)
    {
        const int k = maxLag + r;
        int c = 0;
        for (int i = 0; i < na; ++i) base(r, c++) = y(k - 1 - i);
        for (int i = 0; i < nb; ++i) base(r, c++) = u(k - 1 - i);
        for (int i = 0; i < nc; ++i) base(r, c++) = e(k - 1 - i);
        target(r) = y(k);
    }
}

/// Expand the base matrix into the polynomial candidate matrix P (Neff x M).
Eigen::MatrixXd expandPolynomial(const Eigen::MatrixXd &base,
                                 const std::vector<std::vector<int>> &terms)
{
    const int Neff = static_cast<int>(base.rows());
    const int M    = static_cast<int>(terms.size());
    Eigen::MatrixXd P(Neff, M);
    for (int t = 0; t < M; ++t)
    {
        Eigen::VectorXd col = Eigen::VectorXd::Ones(Neff);
        for (int f : terms[t]) col = col.cwiseProduct(base.col(f));
        P.col(t) = col;
    }
    return P;
}

/// Orthogonal Forward Regression with ERR. Returns selected column indices (in selection
/// order) and sets errSum to the cumulative Error Reduction Ratio.
std::vector<int> orthogonalForwardRegression(const Eigen::MatrixXd &P, const Eigen::VectorXd &t,
                                              double tol, int maxTerms, double &errSum)
{
    const int M = static_cast<int>(P.cols());
    const double sigma = t.dot(t);
    std::vector<int> selected;
    errSum = 0.0;
    if (sigma <= 0.0 || M == 0) return selected;

    std::vector<Eigen::VectorXd> basis; // orthogonal basis vectors
    std::vector<char> used(M, 0);
    const int cap = std::min(maxTerms, M);

    for (int step = 0; step < cap; ++step)
    {
        double best_err = 0.0;
        int    best_j   = -1;
        Eigen::VectorXd best_q;
        for (int j = 0; j < M; ++j)
        {
            if (used[j]) continue;
            Eigen::VectorXd q = P.col(j);
            for (const auto &qb : basis)
            {
                const double d = qb.dot(qb);
                if (d > 1e-300) q -= (qb.dot(P.col(j)) / d) * qb;
            }
            const double denom = q.dot(q);
            if (denom < 1e-12) continue; // collinear with already-selected terms
            const double g   = q.dot(t) / denom;
            const double err = g * g * denom / sigma;
            if (err > best_err)
            {
                best_err = err;
                best_j   = j;
                best_q   = q;
            }
        }
        if (best_j < 0) break; // no candidate reduces the error further
        basis.push_back(best_q);
        used[best_j] = 1;
        selected.push_back(best_j);
        errSum += best_err;
        if (errSum >= 1.0 - tol) break;
    }
    return selected;
}

std::string termLabel(const std::vector<int> &factors, int na, int nb, int nc)
{
    if (factors.empty()) return "1";
    std::string s;
    for (std::size_t i = 0; i < factors.size(); ++i)
    {
        if (i) s += "*";
        const int f = factors[i];
        if (f < na)
            s += "y(k-" + std::to_string(f + 1) + ")";
        else if (f < na + nb)
            s += "u(k-" + std::to_string(f - na + 1) + ")";
        else
            s += "e(k-" + std::to_string(f - na - nb + 1) + ")";
    }
    return s;
}

} // namespace

NARMAXResult NARMAXIdentifier::fit(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                   const NARMAXParams &params)
{
    if (u.size() != y.size())
        throw std::invalid_argument("NARMAXIdentifier::fit: u and y must have equal length.");
    if (params.na < 0 || params.nb < 0 || params.nc < 0)
        throw std::invalid_argument("NARMAXIdentifier::fit: lag orders must be >= 0.");
    if (params.poly_degree < 1)
        throw std::invalid_argument("NARMAXIdentifier::fit: poly_degree must be >= 1.");

    const int N      = static_cast<int>(y.size());
    const int na     = params.na, nb = params.nb, nc = params.nc;
    const int maxLag = std::max({na, nb, nc, 1});
    const int Neff   = N - maxLag;
    if (Neff < 2)
        throw std::invalid_argument("NARMAXIdentifier::fit: data record too short for the requested lags.");

    // Library-size guard (combinations with repetition: C(p+degree, degree)).
    const int p = na + nb + nc;
    {
        long M = 1;
        for (int d = 1; d <= params.poly_degree; ++d)
        {
            M = M * (p + d) / d; // running C(p+d, d)
            if (M > kMaxLibrarySize)
                throw std::invalid_argument(
                    "NARMAXIdentifier::fit: candidate library too large; lower poly_degree or lag orders.");
        }
    }

    Eigen::VectorXd e_hat = Eigen::VectorXd::Zero(N);

    // Extended Least Squares: an initial NARX pass populates the residual series used by the
    // noise (e-lag) regressors in the full pass.
    if (nc > 0)
    {
        const auto narxTerms = generateTermFactors(na + nb, params.poly_degree);
        Eigen::MatrixXd base0;
        Eigen::VectorXd t0;
        buildBase(u, y, e_hat, na, nb, /*nc=*/0, maxLag, Neff, base0, t0);
        const Eigen::MatrixXd P0 = expandPolynomial(base0, narxTerms);
        double errSum0 = 0.0;
        const std::vector<int> sel0 =
            orthogonalForwardRegression(P0, t0, params.significance_tol, params.max_terms, errSum0);
        if (!sel0.empty())
        {
            Eigen::MatrixXd Psel0(Neff, static_cast<int>(sel0.size()));
            for (std::size_t i = 0; i < sel0.size(); ++i) Psel0.col(static_cast<int>(i)) = P0.col(sel0[i]);
            const Eigen::VectorXd theta0 = Psel0.colPivHouseholderQr().solve(t0);
            const Eigen::VectorXd resid  = t0 - Psel0 * theta0;
            for (int r = 0; r < Neff; ++r) e_hat(maxLag + r) = resid(r);
        }
    }

    // Full pass over the complete candidate library.
    const auto terms = generateTermFactors(p, params.poly_degree);
    Eigen::MatrixXd base;
    Eigen::VectorXd target;
    buildBase(u, y, e_hat, na, nb, nc, maxLag, Neff, base, target);
    const Eigen::MatrixXd P = expandPolynomial(base, terms);

    NARMAXResult result;
    result.na = na;
    result.nb = nb;
    result.nc = nc;

    const std::vector<int> sel =
        orthogonalForwardRegression(P, target, params.significance_tol, params.max_terms,
                                     result.final_err_sum);
    if (sel.empty())
        return result;

    Eigen::MatrixXd Psel(Neff, static_cast<int>(sel.size()));
    for (std::size_t i = 0; i < sel.size(); ++i) Psel.col(static_cast<int>(i)) = P.col(sel[i]);
    result.coefficients = Psel.colPivHouseholderQr().solve(target);

    result.selected_terms.reserve(sel.size());
    result.term_factors.reserve(sel.size());
    for (int idx : sel)
    {
        result.selected_terms.push_back(termLabel(terms[idx], na, nb, nc));
        result.term_factors.push_back(terms[idx]);
    }
    return result;
}

double NARMAXIdentifier::predict(const NARMAXResult &model, const Eigen::VectorXd &u_hist,
                                 const Eigen::VectorXd &y_hist)
{
    if (y_hist.size() < model.na || u_hist.size() < model.nb)
        throw std::invalid_argument("NARMAXIdentifier::predict: history shorter than model lag orders.");

    const int p = model.na + model.nb + model.nc;
    Eigen::VectorXd base = Eigen::VectorXd::Zero(p);
    int c = 0;
    for (int i = 0; i < model.na; ++i) base(c++) = y_hist(y_hist.size() - 1 - i);
    for (int i = 0; i < model.nb; ++i) base(c++) = u_hist(u_hist.size() - 1 - i);
    // remaining e-lag entries stay 0 (one-step-ahead: future residuals unknown)

    double yhat = 0.0;
    for (std::size_t t = 0; t < model.term_factors.size(); ++t)
    {
        double term = 1.0;
        for (int f : model.term_factors[t]) term *= base(f);
        yhat += model.coefficients(static_cast<int>(t)) * term;
    }
    return yhat;
}

} // namespace ctrl
