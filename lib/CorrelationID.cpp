#include "CorrelationID.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace ctrl {

namespace {

// Maximal-length Galois-LFSR tap masks for n_bits = 2..20, each empirically verified
// (period == 2^n_bits - 1, checked from 3 independent seeds) - see the derivation in
// docs/superpowers/specs/2026-06-24-small-foundational-utilities-design.md's PRBS note.
// Galois-form transition: lsb = state & 1; state >>= 1; if (lsb) state ^= mask. The mask's
// top bit (1 << (n_bits-1)) is always set, which is what makes this transition bijective
// (no nonzero state can ever map to the absorbing all-zero state).
unsigned lfsrTapsMask(int n_bits)
{
    static const std::unordered_map<int, unsigned> taps = {
        {2,  0x00003u}, {3,  0x00005u}, {4,  0x00009u}, {5,  0x00012u},
        {6,  0x00021u}, {7,  0x00041u}, {8,  0x0008eu}, {9,  0x00108u},
        {10, 0x00204u}, {11, 0x00402u}, {12, 0x00829u}, {13, 0x0100du},
        {14, 0x02015u}, {15, 0x04001u}, {16, 0x08016u}, {17, 0x10004u},
        {18, 0x20013u}, {19, 0x40013u}, {20, 0x80004u},
    };
    auto it = taps.find(n_bits);
    if (it == taps.end())
        throw std::invalid_argument("CorrelationID::generatePRBS: n_bits must be in [2, 20].");
    return it->second;
}

} // namespace

CorrelationIDResult CorrelationID::identify(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                             double /*Ts*/, const CorrelationIDParams &params)
{
    if (u.size() != y.size())
        throw std::invalid_argument("CorrelationID::identify: u and y must have the same length.");

    const int N = static_cast<int>(u.size());
    if (params.max_lag < 0 || params.max_lag >= N)
        throw std::invalid_argument("CorrelationID::identify: max_lag must be in [0, u.size()).");

    Eigen::VectorXd uu = u;
    Eigen::VectorXd yy = y;

    if (params.whiten_input)
    {
        // First-order AR pre-whitening: estimate rho = R_uu(1)/R_uu(0) from the raw signal,
        // then apply the same causal first-difference filter to both u and y - LTI filtering
        // commutes with convolution, so the u->y impulse-response relationship is preserved.
        const double r0 = u.dot(u) / static_cast<double>(N);
        double r1 = 0.0;
        for (int n = 1; n < N; ++n) r1 += u(n) * u(n - 1);
        r1 /= static_cast<double>(std::max(N - 1, 1));
        const double rho = (std::abs(r0) > 1e-12) ? (r1 / r0) : 0.0;

        for (int n = 1; n < N; ++n)
        {
            uu(n) = u(n) - rho * u(n - 1);
            yy(n) = y(n) - rho * y(n - 1);
        }
    }

    const int L = params.max_lag;
    CorrelationIDResult result;
    result.autocorr_u   = Eigen::VectorXd::Zero(L + 1);
    result.crosscorr_uy = Eigen::VectorXd::Zero(L + 1);

    for (int k = 0; k <= L; ++k)
    {
        double s_uu = 0.0, s_uy = 0.0;
        const int count = N - k;
        for (int n = k; n < N; ++n)
        {
            s_uu += uu(n) * uu(n - k);
            s_uy += uu(n - k) * yy(n);
        }
        result.autocorr_u(k)   = s_uu / static_cast<double>(count);
        result.crosscorr_uy(k) = s_uy / static_cast<double>(count);
    }

    const double r0 = result.autocorr_u(0);
    const double r0_safe = (std::abs(r0) > 1e-12) ? r0 : (r0 >= 0.0 ? 1e-12 : -1e-12);
    result.impulse_response = result.crosscorr_uy / r0_safe;

    return result;
}

Eigen::VectorXd CorrelationID::generatePRBS(int length, int n_bits, unsigned seed)
{
    if (length <= 0)
        throw std::invalid_argument("CorrelationID::generatePRBS: length must be positive.");
    const unsigned mask = lfsrTapsMask(n_bits);

    unsigned state = seed & ((1u << n_bits) - 1u);
    if (state == 0) state = 1;

    Eigen::VectorXd out(length);
    for (int k = 0; k < length; ++k)
    {
        const unsigned lsb = state & 1u;
        out(k) = lsb ? 1.0 : -1.0;
        state >>= 1;
        if (lsb) state ^= mask;
    }
    return out;
}

} // namespace ctrl
