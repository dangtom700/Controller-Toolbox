// ============================================================
//  ex125_computational_delay_wrapper.cpp
//  ComputationalDelayWrapper - one-sample computational delay decorator.
//
//  Every real digital loop applies the control computed at step k only at step
//  k+1 (ADC + compute + DAC latency). This wrapper models that delay so a design
//  can be evaluated against the phase margin the deployed loop actually has.
//
//  Demo: the SAME PID on the SAME lightly-damped plant, with vs without the
//  one-sample delay. The delay erodes phase margin -> larger overshoot. Both
//  loops still settle (integral action), so the wrapper EXPOSES the margin cost;
//  it does not fix it.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <memory>

namespace
{
// Returns peak output (overshoot indicator) and settled value (mean of the last
// 100 samples); optionally inserts a one-sample computational delay around the PID.
void run(const ctrl::StateSpace &plant, double Ts, bool withDelay,
         double &peak, double &settled)
{
    // Gentle gains so BOTH loops settle within the horizon; the delay's cost then
    // shows up purely as extra overshoot rather than sustained ringing.
    ctrl::PIDParams pp;
    pp.Kp = 0.7; pp.Ki = 0.7; pp.Kd = 0.12; pp.N = 20.0;
    pp.uMin = -1e6; pp.uMax = 1e6;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    std::shared_ptr<ctrl::IController> ctl =
        withDelay ? std::static_pointer_cast<ctrl::IController>(
                        std::make_shared<ctrl::ComputationalDelayWrapper>(pid, 0.0))
                  : std::static_pointer_cast<ctrl::IController>(pid);

    const int N = 1000, WIN = 100;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0, sum = 0.0;
    peak = 0.0;
    const double r = 1.0;
    for (int k = 0; k < N; ++k)
    {
        const double u = ctl->compute(r - y); // PID sign convention: r - y
        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);
        peak = std::max(peak, y);
        if (k >= N - WIN) sum += y;
    }
    settled = sum / WIN;
}
} // namespace

int main()
{
    const double Ts = 0.1;
    // Lightly-damped 2nd-order plant (wn=1, zeta=0.3), ZOH-discretised.
    Eigen::MatrixXd Ac(2, 2); Ac << 0.0, 1.0, -1.0, -0.6;
    Eigen::MatrixXd Bc(2, 1); Bc << 0.0, 1.0;
    Eigen::MatrixXd Cc(1, 2); Cc << 1.0, 0.0;
    Eigen::MatrixXd Dc = Eigen::MatrixXd::Zero(1, 1);
    ctrl::StateSpace plant = ctrl::c2d(ctrl::StateSpace(Ac, Bc, Cc, Dc, 0.0),
                                       Ts, ctrl::C2dMethod::ZOH);

    double peakNoDelay, settledNoDelay, peakDelay, settledDelay;
    run(plant, Ts, /*withDelay=*/false, peakNoDelay, settledNoDelay);
    run(plant, Ts, /*withDelay=*/true,  peakDelay,   settledDelay);

    const double osNoDelay = (peakNoDelay - 1.0) * 100.0;
    const double osDelay   = (peakDelay   - 1.0) * 100.0;
    std::cout << "=== ComputationalDelayWrapper: one-sample delay cost ===\n"
              << std::fixed << std::setprecision(2)
              << "  no delay : peak=" << peakNoDelay << "  overshoot=" << osNoDelay
              << "%  settled=" << settledNoDelay << "\n"
              << "  1-sample : peak=" << peakDelay   << "  overshoot=" << osDelay
              << "%  settled=" << settledDelay << "\n";

    // The delayed loop should overshoot more (reduced phase margin) yet still settle to r.
    const bool ok = std::isfinite(settledDelay) && std::isfinite(settledNoDelay)
                  && peakDelay > peakNoDelay
                  && std::abs(settledDelay - 1.0) < 0.05
                  && std::abs(settledNoDelay - 1.0) < 0.05;
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
