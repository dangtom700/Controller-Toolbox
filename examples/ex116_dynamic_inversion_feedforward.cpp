/**
 * @file ex116_dynamic_inversion_feedforward.cpp
 * @brief Dynamic-inversion feedforward via invertTransferFunction() + FeedforwardController.
 *
 * Demonstrates that "dynamic inversion" (docs/control_strategies_deep_dive.md's name for it)
 * needs no dedicated class: invertTransferFunction(G) builds G^-1(z), tf2ss() realises it as a
 * StateSpace, and the existing FeedforwardController runs it against the reference signal -
 * u_ff[k] = G^-1(z) * r[k], so that G(z) * u_ff(z) == r(z) exactly (perfect feedforward
 * inversion, zero tracking error from the feedforward path alone, given an exact model).
 *
 * @see docs/superpowers/specs/2026-06-26-small-extensions-batch-design.md, item 1.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <vector>

int main()
{
    const double Ts = 0.1;

    // Plant: G(s) = 2/(s+1) -> ZOH discretisation.
    const ctrl::StateSpace plant_c(
        Eigen::MatrixXd::Constant(1, 1, -1.0),
        Eigen::MatrixXd::Constant(1, 1,  1.0),
        Eigen::MatrixXd::Constant(1, 1,  2.0),
        Eigen::MatrixXd::Zero(1, 1), 0.0);
    const ctrl::StateSpace plant = ctrl::c2d(plant_c, Ts, ctrl::C2dMethod::ZOH);
    const ctrl::TransferFunction G = ctrl::ss2tf(plant);

    std::cout << "G(z^-1): num=[" << G.num[0] << ", " << G.num[1]
              << "]  den=[" << G.den[0] << ", " << G.den[1] << "]\n";

    // Invert G and build the feedforward filter G^-1(z).
    const ctrl::TransferFunction Ginv = ctrl::invertTransferFunction(G);
    std::cout << "G^-1(z^-1): num=[" << Ginv.num[0] << ", " << Ginv.num[1]
              << "]  den=[" << Ginv.den[0] << ", " << Ginv.den[1] << "]\n";

    const ctrl::StateSpace Gff_model = ctrl::tf2ss(Ginv);
    ctrl::FeedforwardController ff(Gff_model);

    // Open-loop check: drive the feedforward filter with a piecewise-changing reference, feed
    // its output straight into the plant (no feedback at all), and confirm the plant output
    // tracks r EXACTLY at every single step - not just eventually. This is the signature of
    // perfect dynamic-inversion feedforward: G^-1(z)*G(z) == 1 identically, so the cascade has
    // no transient whatsoever (unlike a feedback loop, which would need time to settle).
    Eigen::VectorXd x_plant = Eigen::VectorXd::Zero(plant.stateSize());

    const std::vector<double> r_seq = {1.0, 1.0, 1.0, 2.5, 2.5, -1.0, -1.0, 0.0, 3.0, 3.0};
    bool all_ok = true;
    std::cout << std::fixed;
    std::cout.precision(6);
    for (double r : r_seq)
    {
        const double u_ff = ff.compute(r);
        Eigen::VectorXd uv(1);
        uv << u_ff;
        const double y = ctrl::ssStep(plant, x_plant, uv)(0);
        std::cout << "  r=" << r << "  u_ff=" << u_ff << "  y=" << y << "\n";
        all_ok = all_ok && std::isfinite(y) && std::fabs(y - r) < 1e-6;
    }

    std::cout << (all_ok ? "PASS" : "FAIL") << "\n";
    return all_ok ? 0 : 1;
}
