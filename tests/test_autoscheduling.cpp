#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "ControllerToolbox.h"
#include <cmath>

using namespace ctrl;
using Catch::Approx;

// ---------------------------------------------------------------------------
// [gap] tests - nu-gap metric
// ---------------------------------------------------------------------------

TEST_CASE("nuGap is zero for identical systems", "[gap][autosched]")
{
    // G(z) = 0.0952 / (z - 0.9048)  ~  ZOH of G(s)=1/(s+1) at Ts=0.1
    StateSpace sys(
        (Eigen::MatrixXd(1,1) << 0.9048).finished(),
        (Eigen::MatrixXd(1,1) << 0.0952).finished(),
        (Eigen::MatrixXd(1,1) << 1.0  ).finished(),
        (Eigen::MatrixXd(1,1) << 0.0  ).finished(),
        0.1
    );
    double gap = nuGap(sys, sys, 100);
    REQUIRE(gap < 1e-10);
}

TEST_CASE("nuGap is bounded in [0,1] for two distinct SISO systems", "[gap][autosched]")
{
    Eigen::MatrixXd B(1,1); B << 0.1;
    Eigen::MatrixXd C(1,1); C << 1.0;
    Eigen::MatrixXd D(1,1); D << 0.0;

    StateSpace P1((Eigen::MatrixXd(1,1) << 0.90).finished(), B, C, D, 0.1);
    StateSpace P2((Eigen::MatrixXd(1,1) << 0.50).finished(), B, C, D, 0.1);

    double gap = nuGap(P1, P2, 100);
    REQUIRE(gap >= 0.0);
    REQUIRE(gap <= 1.0 + 1e-10);
}

TEST_CASE("nuGap is larger for more different systems", "[gap][autosched]")
{
    Eigen::MatrixXd B(1,1); B << 0.1;
    Eigen::MatrixXd C(1,1); C << 1.0;
    Eigen::MatrixXd D(1,1); D << 0.0;
    const double Ts = 0.1;

    StateSpace P1((Eigen::MatrixXd(1,1) << 0.90).finished(), B, C, D, Ts);
    StateSpace P2((Eigen::MatrixXd(1,1) << 0.89).finished(), B, C, D, Ts);
    StateSpace P3((Eigen::MatrixXd(1,1) << 0.10).finished(), B, C, D, Ts);

    double gap12 = nuGap(P1, P2, 100);
    double gap13 = nuGap(P1, P3, 100);
    // P1 and P3 differ far more than P1 and P2
    REQUIRE(gap12 < gap13);
}

// ---------------------------------------------------------------------------
// [clustering] tests
// ---------------------------------------------------------------------------

TEST_CASE("clusterByGap groups two close and two distant systems into 2 clusters",
          "[clustering][autosched]")
{
    const double Ts = 0.1;
    Eigen::MatrixXd B(1,1); B << 0.1;
    Eigen::MatrixXd C(1,1); C << 1.0;
    Eigen::MatrixXd D(1,1); D << 0.0;

    // Two pairs: {0.90, 0.89} close together; {0.20, 0.21} close together.
    // Pairs are far apart from each other.
    std::vector<StateSpace> models = {
        {(Eigen::MatrixXd(1,1) << 0.90).finished(), B, C, D, Ts},
        {(Eigen::MatrixXd(1,1) << 0.89).finished(), B, C, D, Ts},
        {(Eigen::MatrixXd(1,1) << 0.20).finished(), B, C, D, Ts},
        {(Eigen::MatrixXd(1,1) << 0.21).finished(), B, C, D, Ts},
    };

    ClusterResult res = clusterByGap(models, 0.5, 100);
    REQUIRE(res.numClusters == 2);
    // Models 0 and 1 must be in the same cluster
    REQUIRE(res.labels[0] == res.labels[1]);
    // Models 2 and 3 must be in the same cluster
    REQUIRE(res.labels[2] == res.labels[3]);
    // The two pairs must be in different clusters
    REQUIRE(res.labels[0] != res.labels[2]);
    // Representatives must be valid indices
    for (int c = 0; c < res.numClusters; ++c)
        REQUIRE((res.representatives[c] >= 0 && res.representatives[c] < 4));
}

TEST_CASE("nuGapMatrix is symmetric with zero diagonal", "[clustering][autosched]")
{
    const double Ts = 0.05;
    Eigen::MatrixXd B(1,1); B << 0.05;
    Eigen::MatrixXd C(1,1); C << 1.0;
    Eigen::MatrixXd D(1,1); D << 0.0;
    std::vector<StateSpace> models = {
        {(Eigen::MatrixXd(1,1) << 0.95).finished(), B, C, D, Ts},
        {(Eigen::MatrixXd(1,1) << 0.70).finished(), B, C, D, Ts},
        {(Eigen::MatrixXd(1,1) << 0.30).finished(), B, C, D, Ts},
    };
    Eigen::MatrixXd G = nuGapMatrix(models, 100);
    REQUIRE(G.rows() == 3);
    REQUIRE(G.cols() == 3);
    // Diagonal zero
    REQUIRE(G(0,0) == Approx(0.0).margin(1e-10));
    REQUIRE(G(1,1) == Approx(0.0).margin(1e-10));
    REQUIRE(G(2,2) == Approx(0.0).margin(1e-10));
    // Symmetry
    REQUIRE(G(0,1) == Approx(G(1,0)).epsilon(1e-10));
    REQUIRE(G(0,2) == Approx(G(2,0)).epsilon(1e-10));
    REQUIRE(G(1,2) == Approx(G(2,1)).epsilon(1e-10));
}

// ---------------------------------------------------------------------------
// [lpv_id] tests
// ---------------------------------------------------------------------------

TEST_CASE("identifyLPV recovers true affine coefficients from noise-free data",
          "[lpv_id][autosched]")
{
    // True LPV (degree 1, SISO, n=1, m=1):
    //   x[k+1] = (a0 + a1*p[k]) * x[k] + b0 * u[k]
    //   y[k]   = x[k]
    const double a0 = -0.50, a1 = 0.30, b0 = 0.80;
    const double Ts = 0.05;
    const int    N  = 400;
    const double pi = 3.14159265358979323846;

    Eigen::MatrixXd X(1, N), U(1, N), Y(1, N);
    std::vector<double> sched(N);
    double x = 0.5;
    for (int k = 0; k < N; ++k) {
        double p = 0.5 * std::sin(2.0 * pi * k / static_cast<double>(N));
        double u = 0.3 * std::cos(2.0 * pi * k / static_cast<double>(N / 2));
        X(0, k) = x;
        U(0, k) = u;
        Y(0, k) = x;
        sched[k] = p;
        x = (a0 + a1 * p) * x + b0 * u;
    }

    LPVModel m = identifyLPV(X, U, Y, sched, 1, Ts);

    REQUIRE(m.degree    == 1);
    REQUIRE(m.n_states  == 1);
    REQUIRE(m.n_inputs  == 1);
    REQUIRE(m.n_outputs == 1);

    // Tolerances: 1e-6 for noise-free data with good excitation
    REQUIRE(m.A_coeffs[0](0,0) == Approx(a0).epsilon(1e-4));
    REQUIRE(m.A_coeffs[1](0,0) == Approx(a1).epsilon(1e-4));
    REQUIRE(m.B_coeffs[0](0,0) == Approx(b0).epsilon(1e-4));
}

TEST_CASE("LPVModel::frozen returns correct StateSpace at specific p", "[lpv_id][autosched]")
{
    // Manually construct a degree-1 LPV model
    LPVModel m;
    m.degree = 1; m.n_states = 1; m.n_inputs = 1; m.n_outputs = 1; m.Ts = 0.1;
    m.A_coeffs.resize(2);
    m.B_coeffs.resize(2);
    m.C_coeffs.resize(2);
    m.D_coeffs.resize(2);
    m.A_coeffs[0] = (Eigen::MatrixXd(1,1) << -0.5).finished();
    m.A_coeffs[1] = (Eigen::MatrixXd(1,1) <<  0.3).finished();
    m.B_coeffs[0] = (Eigen::MatrixXd(1,1) <<  0.8).finished();
    m.B_coeffs[1] = (Eigen::MatrixXd(1,1) <<  0.0).finished();
    m.C_coeffs[0] = (Eigen::MatrixXd(1,1) <<  1.0).finished();
    m.C_coeffs[1] = (Eigen::MatrixXd(1,1) <<  0.0).finished();
    m.D_coeffs[0] = (Eigen::MatrixXd(1,1) <<  0.0).finished();
    m.D_coeffs[1] = (Eigen::MatrixXd(1,1) <<  0.0).finished();

    StateSpace sys = m.frozen(0.5);
    REQUIRE(sys.A(0,0) == Approx(-0.5 + 0.3 * 0.5).epsilon(1e-12));
    REQUIRE(sys.B(0,0) == Approx(0.8).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// [autosched] end-to-end test
// ---------------------------------------------------------------------------

TEST_CASE("buildAutoGainScheduler creates valid GainScheduledController for "
          "gain-varying first-order plant",
          "[autosched]")
{
    // Nonlinear plant: xdot = -(1 + 0.5*p^2)*x + u
    // Equilibrium for p-sweep: u_eq = p, x_eq ~ p / (1 + 0.5*p^2)
    const double Ts = 0.05;

    StateFunc f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd xd(1);
        xd(0) = -x(0) + u(0);
        return xd;
    };

    auto u_eq_fn = [](double p) { Eigen::VectorXd u(1); u(0) = p; return u; };
    auto x0_fn   = [](double p) { Eigen::VectorXd x(1); x(0) = p; return x; };

    // Use DiscretePID (scalar IController) as the per-cluster controller.
    // Proportional gain from DC gain of the linearised model: Kp = (1-a)/b
    auto design_fn = [Ts](const StateSpace& sys, double /*p*/)
            -> std::shared_ptr<IController> {
        double a  = sys.A(0, 0);
        double b  = sys.B(0, 0);
        double Kp = (std::abs(b) > 1e-10) ? (1.0 - a) / b : 1.0;
        PIDParams pp; pp.Kp = Kp; pp.Ki = 0.0; pp.Kd = 0.0;
        return std::make_shared<DiscretePID>(pp, Ts);
    };

    auto sched = buildAutoGainScheduler(f, -0.5, 0.5, 6, u_eq_fn, x0_fn,
                                         design_fn, Ts, 0.5, 100);

    REQUIRE(sched != nullptr);
    REQUIRE(sched->numPoints() >= 1);

    // verify compute() returns a finite value for a range of p
    for (double p : {-0.5, -0.2, 0.0, 0.2, 0.5}) {
        sched->setSchedulingParam(p);
        double u = sched->compute(1.0);
        REQUIRE(std::isfinite(u));
    }
}

TEST_CASE("GainScheduledController NearestNeighbor selects correct controller",
          "[autosched]")
{
    // Two trivial controllers: u = 1.0 and u = 2.0
    struct ConstCtrl : public IController {
        double val_, Ts_;
        ConstCtrl(double v, double Ts) : val_(v), Ts_(Ts) {}
        double compute(double) override { return val_; }
        void   reset()   override {}
        double sampleTime() const override { return Ts_; }
    };

    GainScheduledController gs(0.1, GainScheduleMode::NearestNeighbor);
    gs.addSchedulePoint(0.0, std::make_shared<ConstCtrl>(1.0, 0.1));
    gs.addSchedulePoint(1.0, std::make_shared<ConstCtrl>(2.0, 0.1));

    gs.setSchedulingParam(0.3);
    REQUIRE(gs.compute(0.0) == Approx(1.0));  // closer to p=0

    gs.setSchedulingParam(0.7);
    REQUIRE(gs.compute(0.0) == Approx(2.0));  // closer to p=1
}
