#include "LFTSystem.h"
#include "GapMetric.h"
#include <cmath>
#include <stdexcept>

namespace ctrl {

namespace {

std::vector<double> logFreqGrid(double Ts, int N, double omega_min)
{
    if (N < 2) N = 2;
    const double omega_max = 3.14159265358979323846 / Ts;
    const double log_min   = std::log(omega_min);
    const double log_max   = std::log(omega_max);
    std::vector<double> grid(N);
    for (int i = 0; i < N; ++i)
        grid[i] = std::exp(log_min + (log_max - log_min) * i / (N - 1));
    return grid;
}

} // namespace

LFTSystem::LFTSystem(const StateSpace &M0, const UncertaintyStructure &struc,
                      const LFTChannelMap &map)
    : M0_(M0), struc_(struc), map_(map)
{
    const std::size_t F = struc_.blocks.size();
    if (map_.rowStart.size() != F || map_.colStart.size() != F)
        throw std::invalid_argument(
            "LFTSystem: LFTChannelMap.rowStart/colStart must have one entry per "
            "UncertaintyStructure block.");

    const int p_out = static_cast<int>(M0_.C.rows()); // M0's output dimension
    const int m_in  = static_cast<int>(M0_.B.cols()); // M0's input dimension

    // Build the flattened, block-ordered gather index lists, validating bounds and
    // checking for any overlap between blocks' claimed ranges.
    std::vector<std::pair<int, int>> rowRanges, colRanges; // (start, end) per block
    for (std::size_t i = 0; i < F; ++i)
    {
        const int rOut = struc_.blocks[i].r_out;
        const int rIn  = struc_.blocks[i].r_in;
        const int rs = map_.rowStart[i], re = rs + rOut;
        const int cs = map_.colStart[i], ce = cs + rIn;

        if (rs < 0 || re > p_out)
            throw std::invalid_argument(
                "LFTSystem: block " + std::to_string(i) + "'s row range is out of bounds "
                "for M0's output dimension.");
        if (cs < 0 || ce > m_in)
            throw std::invalid_argument(
                "LFTSystem: block " + std::to_string(i) + "'s column range is out of bounds "
                "for M0's input dimension.");

        for (const auto &r : rowRanges)
            if (rs < r.second && r.first < re)
                throw std::invalid_argument(
                    "LFTSystem: block " + std::to_string(i) + "'s row range overlaps another "
                    "block's row range.");
        for (const auto &c : colRanges)
            if (cs < c.second && c.first < ce)
                throw std::invalid_argument(
                    "LFTSystem: block " + std::to_string(i) + "'s column range overlaps "
                    "another block's column range.");

        rowRanges.emplace_back(rs, re);
        colRanges.emplace_back(cs, ce);

        for (int r = rs; r < re; ++r) rowIdx_.push_back(r);
        for (int c = cs; c < ce; ++c) colIdx_.push_back(c);
    }
}

std::vector<Eigen::MatrixXcd> LFTSystem::closedLoopFreqResponse(
    const std::vector<double> &omegas) const
{
    const auto M0_freq = freqResponseGrid(M0_, omegas);

    std::vector<Eigen::MatrixXcd> gathered;
    gathered.reserve(M0_freq.size());
    for (const auto &M : M0_freq)
    {
        Eigen::MatrixXcd Mg(rowIdx_.size(), colIdx_.size());
        for (std::size_t i = 0; i < rowIdx_.size(); ++i)
            for (std::size_t j = 0; j < colIdx_.size(); ++j)
                Mg(i, j) = M(rowIdx_[i], colIdx_[j]);
        gathered.push_back(std::move(Mg));
    }
    return gathered;
}

PeakMuResult LFTSystem::peakMu(int freq_points, double omega_min) const
{
    if (freq_points < 2) freq_points = 2;

    const auto grid    = logFreqGrid(M0_.Ts, freq_points, omega_min);
    const auto M_freq   = closedLoopFreqResponse(grid);
    const auto curve    = computeMu(M_freq, struc_, false);

    PeakMuResult res;
    res.mu_curve         = curve;
    res.peak             = MuBound{0.0, 0.0};
    res.peak_omega_rad_s = grid.empty() ? 0.0 : grid[0];
    for (std::size_t i = 0; i < curve.size(); ++i)
    {
        if (curve[i].upper > res.peak.upper)
        {
            res.peak             = curve[i];
            res.peak_omega_rad_s = grid[i];
        }
    }
    return res;
}

} // namespace ctrl
