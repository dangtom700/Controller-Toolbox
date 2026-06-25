#include "SelfTuningRegulator.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

namespace
{

// Expands product_i (1 - roots(i)*z) into ascending-power coefficients [c0, c1, ..., c_n].
Eigen::VectorXd polyFromRoots(const Eigen::VectorXd &roots)
{
    Eigen::VectorXd coeffs(1);
    coeffs(0) = 1.0;
    for (int i = 0; i < roots.size(); ++i)
    {
        Eigen::VectorXd next = Eigen::VectorXd::Zero(coeffs.size() + 1);
        for (int j = 0; j < coeffs.size(); ++j)
        {
            next(j) += coeffs(j);
            next(j + 1) += -roots(i) * coeffs(j);
        }
        coeffs = next;
    }
    return coeffs;
}

// Sylvester resultant matrix of A(z) (size na+1, [1,a1,...,a_na]) and B(z) (size nb+1,
// [0,b1,...,b_nb]), for the Diophantine equation A.R + B.S = Acl with deg(R)<=nb-1, deg(S)<=na-1.
// Returns an (na+nb) x (na+nb) matrix; columns [0,nb) are the R-block, [nb,nb+na) the S-block.
Eigen::MatrixXd buildSylvesterMatrix(const Eigen::VectorXd &A, const Eigen::VectorXd &B,
                                      int nb, int na)
{
    const int n = na + nb;
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);
    for (int col = 0; col < nb; ++col)
        for (int i = 0; i < A.size(); ++i)
        {
            const int row = col + i;
            if (row < n) M(row, col) = A(i);
        }
    for (int col = 0; col < na; ++col)
        for (int i = 0; i < B.size(); ++i)
        {
            const int row = col + i;
            if (row < n) M(row, nb + col) = B(i);
        }
    return M;
}

} // namespace

SelfTuningRegulator::SelfTuningRegulator(const STRParams &params, double Ts)
    : rls_(params.na, params.nb, Ts, params.lambda), p_(params), Ts_(Ts),
      probeRng_(params.probeSeed)
{
    if (p_.na < 1)
        throw std::invalid_argument("SelfTuningRegulator: na must be >= 1");
    if (p_.nb < 1)
        throw std::invalid_argument("SelfTuningRegulator: nb must be >= 1");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("SelfTuningRegulator: Ts must be > 0");
    if (p_.uMin >= p_.uMax)
        throw std::invalid_argument("SelfTuningRegulator: uMin must be < uMax");
    if (p_.bMin <= 0.0)
        throw std::invalid_argument("SelfTuningRegulator: bMin must be > 0");
    if (p_.mode == STRMode::PolePlacement &&
        p_.desired_poles.size() != p_.na + p_.nb - 1)
        throw std::invalid_argument(
            "SelfTuningRegulator: desired_poles.size() must equal na + nb - 1");

    yHist_ = Eigen::VectorXd::Zero(p_.na);
    uHist_ = Eigen::VectorXd::Zero(std::max(p_.nb - 1, 0));
}

double SelfTuningRegulator::compute(double y_plant)
{
    if (!std::isfinite(y_plant))
    {
        notifyObserver(uPrev_, y_plant);
        return uPrev_;
    }

    if (havePrevU_)
        rls_.update(y_plant, uPrev_);

    for (int i = p_.na - 1; i > 0; --i) yHist_(i) = yHist_(i - 1);
    yHist_(0) = y_plant;

    double u = (p_.mode == STRMode::MinimumVariance) ? computeMinimumVariance()
                                                       : computePolePlacement();
    if (!std::isfinite(u)) u = uPrev_;
    if (p_.probeAmplitude > 0.0) u += p_.probeAmplitude * probeDist_(probeRng_);
    u = std::clamp(u, p_.uMin, p_.uMax);

    if (uHist_.size() > 0)
    {
        for (int i = static_cast<int>(uHist_.size()) - 1; i > 0; --i) uHist_(i) = uHist_(i - 1);
        uHist_(0) = u;
    }

    uPrev_ = u;
    havePrevU_ = true;
    notifyObserver(u, y_plant);
    return u;
}

double SelfTuningRegulator::fallbackProportional() const
{
    // The adaptive law is ill-conditioned (RLS hasn't yet seen enough excitation to identify a
    // usable leading coefficient - notably at cold start, where theta_ is still zero). Holding
    // the last output (typically 0 at cold start) would deadlock: zero output never excites the
    // plant, so RLS never gets informative data, so the leading coefficient never leaves zero.
    // A small fixed-gain proportional fallback breaks this deadlock and is itself a perfectly
    // reasonable controller while the adaptive law warms up.
    constexpr double kFallbackGain = 0.1;
    return std::clamp(kFallbackGain * (r_ - yHist_(0)), p_.uMin, p_.uMax);
}

double SelfTuningRegulator::computeMinimumVariance()
{
    const Eigen::VectorXd &b = rls_.numerator();   // [b1..b_nb]
    const Eigen::VectorXd &a = rls_.denominator();  // [1,a1..a_na]

    const double b1 = b(0);
    if (std::abs(b1) < p_.bMin) return fallbackProportional();

    double sumA = 0.0;
    for (int i = 0; i < p_.na; ++i) sumA += a(i + 1) * yHist_(i);

    double sumB = 0.0;
    for (int i = 1; i < p_.nb; ++i) sumB += b(i) * uHist_(i - 1);

    return (r_ + sumA - sumB) / b1;
}

double SelfTuningRegulator::computePolePlacement()
{
    const Eigen::VectorXd &bNum = rls_.numerator();   // [b1..b_nb]
    const Eigen::VectorXd &aDen = rls_.denominator();  // [1,a1..a_na]

    Eigen::VectorXd B = Eigen::VectorXd::Zero(p_.nb + 1);
    B.tail(p_.nb) = bNum; // [0, b1, ..., b_nb]

    const Eigen::MatrixXd M = buildSylvesterMatrix(aDen, B, p_.nb, p_.na);
    const Eigen::VectorXd Acl = polyFromRoots(p_.desired_poles); // size na+nb

    Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
    if (!lu.isInvertible()) return fallbackProportional();
    const Eigen::VectorXd x = lu.solve(Acl); // [r0..r_{nb-1}, s0..s_{na-1}]

    const double r0 = x(0);
    if (std::abs(r0) < p_.bMin) return fallbackProportional();

    const double B1 = B.sum();
    if (std::abs(B1) < p_.bMin) return fallbackProportional();
    const double T = Acl.sum() / B1;

    double sumS = 0.0;
    for (int i = 0; i < p_.na; ++i) sumS += x(p_.nb + i) * yHist_(i);

    double sumR = 0.0;
    for (int i = 1; i < p_.nb; ++i) sumR += x(i) * uHist_(i - 1);

    return (T * r_ - sumS - sumR) / r0;
}

void SelfTuningRegulator::reset()
{
    rls_.reset();
    yHist_.setZero();
    uHist_.setZero();
    uPrev_ = 0.0;
    havePrevU_ = false;
    notifyObserverReset();
}

} // namespace ctrl
