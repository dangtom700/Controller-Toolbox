// ex131_plc_jitter_mpc.cpp - Network-jitter-compensated MPC over a master/slave (server/PLC) link.
//
// Fusion: ctrl::DiscreteMPC + ctrl::NetworkChannel + ctrl::SmithPredictor
//         + ctrl::ComputationalDelayWrapper
//
// Architecture:
//
//     slave (PLC)  : plant, integrated at Ts_plant; holds the last command on starvation
//        |  measurement -> NetworkChannel "up"   (latency + jitter + loss)
//     master (srv) : MPC at Tf, one tick of compute delay
//        |  command     -> NetworkChannel "down" (latency + jitter + loss)
//     slave (PLC)  : applies the newest command that has arrived
//
// The point of the demo: a round-trip transport delay of ~0.5 plant time constants destabilises
// an aggressively-tuned MPC that assumes its measurement is current. Wrapping the SAME MPC in a
// SmithPredictor - which advances a delay-free plant model over the nominal dead time so the
// controller reasons about where the plant will be when the command lands - recovers tracking.
//
// Two design notes, both deliberate:
//
//   1. ctrl::ComputationalDelayWrapper models only a FIXED one-sample delay, so it cannot
//      represent jitter. It is used here for its honest role - the master's own one-tick compute
//      latency - and is applied identically to BOTH arms. The variable transport delay comes
//      from ctrl::NetworkChannel.
//
//   2. The inner predictive controller is ctrl::DiscreteMPC, not ctrl::NonlinearMPC.
//      SmithPredictor drives its inner controller purely through compute(error); NonlinearMPC
//      additionally requires setState() before every call, so the two do not compose.
//      DiscreteMPC carries its own internal state estimate across compute() calls and does.
//
// Both arms run over channels constructed with the SAME seed and the same call sequence, so they
// see a byte-identical delivery trace. Without that pairing a lower IAE would only prove the
// second arm drew an easier random link.

#include <ControllerToolbox.h>

#include <cmath>
#include <cstdio>
#include <memory>

namespace {

// -- Slave-side plant: first-order lag  x' = -a*x + b*u  ----------------------
constexpr double kA        = 4.0;    // 1/tau, tau = 0.25 s
constexpr double kB        = 4.0;    // unity DC gain
constexpr double kTsPlant  = 0.002;  // slave integration step [s]
constexpr double kTf       = 0.020;  // master control period [s]
constexpr int    kSubSteps = static_cast<int>(kTf / kTsPlant + 0.5);

constexpr double kRef      = 1.0;
constexpr double kTsim     = 12.0;
constexpr int    kNSteps   = static_cast<int>(kTsim / kTf + 0.5);

// One-way link characteristics. Round trip is 2*latency_mean = 0.12 s ~ 0.5*tau, which is
// firmly in the range where an uncompensated predictive controller starts to ring.
constexpr double kLatency  = 0.060;
constexpr double kJitter   = 0.012;
constexpr double kLoss     = 0.02;
constexpr unsigned kSeed   = 20260726u;

/// Delay-free discrete model of the plant at the master rate (exact ZOH of the first-order lag).
ctrl::StateSpace masterModel()
{
    const double ad = std::exp(-kA * kTf);
    const double bd = (kB / kA) * (1.0 - ad);
    Eigen::MatrixXd A(1, 1), B(1, 1), C(1, 1), D(1, 1);
    A << ad;  B << bd;  C << 1.0;  D << 0.0;
    return ctrl::StateSpace(A, B, C, D, kTf);
}

ctrl::MPCParams mpcTuning()
{
    ctrl::MPCParams q;
    q.Np    = 20;
    q.Nc    = 4;
    q.rho_y = 1.0;
    q.rho_u = 0.02;   // aggressive: this is what the transport delay punishes
    q.uMin  = -4.0;
    q.uMax  =  4.0;
    return q;
}

struct ArmResult {
    double   iae         = 0.0;
    double   final_err   = 0.0;
    double   max_abs_y   = 0.0;
    unsigned rx_up       = 0;
    unsigned drop_up     = 0;
    double   last_lat_up = 0.0;
};

/// Run one closed loop over a freshly-seeded pair of links.
/// @param compensate  true  -> SmithPredictor wrapping the MPC (dead-time compensated)
///                    false -> the bare MPC on the stale measurement
ArmResult runArm(bool compensate)
{
    const ctrl::StateSpace model = masterModel();

    auto mpc = std::make_shared<ctrl::DiscreteMPC>(model, mpcTuning());

    // The master's control law. Both arms carry the same one-tick compute delay; only the
    // Smith dead-time compensation differs.
    std::shared_ptr<ctrl::IController> law = mpc;
    if (compensate) {
        // theta = nominal ROUND TRIP (up + down). The jitter around it is exactly the modelling
        // error the Smith structure has to tolerate.
        law = std::make_shared<ctrl::SmithPredictor>(mpc, model, 2.0 * kLatency, kTf);
    }
    ctrl::ComputationalDelayWrapper master(law, 0.0);

    ctrl::NetworkChannelParams lp;
    lp.latency_mean = kLatency;
    lp.jitter_sigma = kJitter;
    lp.loss_prob    = kLoss;
    lp.seed         = kSeed;
    ctrl::NetworkChannel<double> up(lp), down(lp);

    double x = 0.0;        // plant state (slave)
    double y_hold = 0.0;   // newest measurement the master has seen
    double u_hold = 0.0;   // newest command the slave has received

    ArmResult r;
    for (int k = 0; k < kNSteps; ++k) {
        const double t = k * kTf;

        // -- slave -> master -------------------------------------------------
        up.send(x, t);
        double rx = 0.0;
        if (up.tryReceive(rx, t)) y_hold = rx;   // else: master keeps the stale value

        // -- master ----------------------------------------------------------
        const double u_cmd = master.compute(kRef - y_hold);

        // -- master -> slave -------------------------------------------------
        down.send(u_cmd, t);
        if (down.tryReceive(rx, t)) u_hold = rx; // else: slave holds its last command

        // -- slave: integrate the plant under the held command ---------------
        for (int i = 0; i < kSubSteps; ++i)
            x += kTsPlant * (-kA * x + kB * u_hold);

        r.iae      += std::abs(kRef - x) * kTf;
        r.max_abs_y = std::max(r.max_abs_y, std::abs(x));
    }

    r.final_err   = std::abs(kRef - x);
    r.rx_up       = up.delivered();
    r.drop_up     = up.dropped();
    r.last_lat_up = up.lastLatency();
    return r;
}

}  // namespace

int main()
{
    std::printf("=== ex131: network-jitter-compensated MPC over a master/slave link ===\n\n");
    std::printf("plant      : x' = -%.1f x + %.1f u   (tau = %.2f s)\n", kA, kB, 1.0 / kA);
    std::printf("rates      : slave %.0f ms | master %.0f ms (%d sub-steps)\n",
                kTsPlant * 1e3, kTf * 1e3, kSubSteps);
    std::printf("link       : %.0f ms +/- %.0f ms one way, %.0f%% loss, seed %u\n",
                kLatency * 1e3, kJitter * 1e3, kLoss * 100.0, kSeed);
    std::printf("round trip : %.0f ms = %.2f tau\n\n", 2.0 * kLatency * 1e3, 2.0 * kLatency * kA);

    const ArmResult naive = runArm(/*compensate=*/false);
    const ArmResult smith = runArm(/*compensate=*/true);

    std::printf("  %-22s %10s %12s %11s\n", "arm", "IAE", "final |e|", "max |y|");
    std::printf("  %-22s %10.4f %12.4f %11.4f\n",
                "MPC (uncompensated)", naive.iae, naive.final_err, naive.max_abs_y);
    std::printf("  %-22s %10.4f %12.4f %11.4f\n",
                "MPC + SmithPredictor", smith.iae, smith.final_err, smith.max_abs_y);

    // Paired-trace sanity: identical seeds and identical call sequences must give identical
    // link statistics. If these ever diverge the comparison above is not paired and the IAE
    // improvement would be meaningless.
    const bool paired = (naive.rx_up == smith.rx_up) && (naive.drop_up == smith.drop_up);
    std::printf("\n  uplink: %u delivered, %u dropped (identical across arms = %s)\n",
                naive.rx_up, naive.drop_up, paired ? "yes" : "NO");

    const double improvement = (naive.iae > 0.0)
                                 ? 100.0 * (naive.iae - smith.iae) / naive.iae : 0.0;
    std::printf("  IAE improvement from dead-time compensation: %.1f %%\n", improvement);

    const bool finite_ok = std::isfinite(naive.iae) && std::isfinite(smith.iae);
    const bool better    = smith.iae < naive.iae;
    const bool tracks    = smith.final_err < 0.10;

    std::printf("\n  paired trace      = %s\n", paired ? "yes" : "no");
    std::printf("  Smith beats naive = %s\n", better ? "yes" : "no");
    std::printf("  Smith arm tracks  = %s (final |e| = %.4f < 0.10)\n",
                tracks ? "yes" : "no", smith.final_err);

    const bool ok = finite_ok && paired && better && tracks;
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
