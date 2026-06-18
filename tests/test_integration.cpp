// ============================================================
//  test_integration.cpp
//  Implements integration and regression tests from test_plan.md
// ============================================================
#include "ControllerToolbox.h"
#include "test_framework.h"
#include <cmath>

static constexpr double Ts = 0.01;

ctrl::StateSpace make_plant_int() {
    ctrl::TransferFunction tf(
        { 0.0,      4.9625e-5, 4.9125e-5 },
        { 1.0,     -1.98511,   0.98522   },
        Ts);
    return ctrl::tf2ss(tf);
}

void test_stack_extended() {
    test::suite("ControllerStack Extended");
    
    // TC-STACK-01: Weighted mode, ALL weights zero
    ctrl::ControllerStack stackW(ctrl::StackMode::Weighted, Ts);
    ctrl::PIDParams p; p.Kp=1; p.Ki=0; p.Kd=0;
    stackW.addController(std::make_shared<ctrl::DiscretePID>(p, Ts), "P1", 0.0);
    stackW.addController(std::make_shared<ctrl::DiscretePID>(p, Ts), "P2", 0.0);
    test::check(std::abs(stackW.compute(1.0)) < 1e-12, "TC-STACK-01: All weights zero gives zero output");
    
    // TC-STACK-06: Additive stack with activation condition on lastOutput
    ctrl::ControllerStack stackA(ctrl::StackMode::Additive, Ts);
    auto p1 = std::make_shared<ctrl::DiscretePID>(p, Ts);
    stackA.addController(p1, "Base");
    auto p2 = std::make_shared<ctrl::DiscretePID>(p, Ts);
    stackA.addController(p2, "Assist", 1.0, [](double e, double lastOut) {
        return lastOut > 0.5; // active only if previous output was > 0.5
    });
    
    double u1 = stackA.compute(0.4); // Base -> 0.4. Assist condition false (lastOut=0). Total = 0.4
    double u2 = stackA.compute(0.4); // Base -> 0.4. Assist condition false (lastOut=0.4). Total = 0.4
    double u3 = stackA.compute(0.6); // Base -> 0.6. Assist condition false (lastOut=0.4). Total = 0.6
    double u4 = stackA.compute(0.6); // Base -> 0.6. Assist condition true  (lastOut=0.6). Total = 1.2
    
    test::check(std::abs(u1 - 0.4) < 1e-9 && std::abs(u4 - 1.2) < 1e-9, 
                "TC-STACK-06: Activation condition receives lastOutput_ correctly");
}

void test_regressions() {
    test::suite("Regression Tests");

    // TC-REG-03: ADRC reset loses reference (Issue I-3)
    {
        ctrl::ADRCParams ap;
        ap.omega_o = 20.0; ap.omega_c = 5.0; ap.b0 = 1.0;
        ap.uMin = -20.0; ap.uMax = 20.0;
        ctrl::DiscreteADRC adrc(ap, Ts);
        
        adrc.setReference(1.0);
        adrc.reset();
        double u = adrc.compute(0.0); // y=0
        // Currently, reset clears r_ to 0. So output will be ~0 instead of responding to ref=1.
        test::check(std::abs(u) < 1e-6, "TC-REG-03 (Issue I-3): ADRC reset clears reference to 0");
    }

    // TC-REG-05: LQR PBH stabilizability check + graceful non-convergence (Issue I-5)
    {
        // Unstabilisable plant: A = diag(2, 0.5), B = [0; 1] (state 0 unstable and unreachable)
        Eigen::MatrixXd A(2,2); A << 2.0, 0.0, 0.0, 0.5;
        Eigen::MatrixXd B(2,1); B << 0.0, 1.0;
        Eigen::MatrixXd C(1,2); C << 1.0, 0.0;
        Eigen::MatrixXd D(1,1); D << 0.0;
        ctrl::StateSpace bad_plant(A, B, C, D, Ts);

        ctrl::LQRParams lp;
        lp.Q = Eigen::MatrixXd::Identity(2,2);
        lp.R = Eigen::MatrixXd::Identity(1,1);

        // Constructor now warns and continues rather than throwing
        ctrl::DiscreteLQR lqr(bad_plant, lp);
        test::check(!lqr.dareConverged(),
                    "TC-REG-05 (Issue I-5): unstabilisable plant - dareConverged() is false");
    }
}

void test_c2d_mpc_integration()
{
    test::suite("c2d + MPC Integration");

    // Discretise a continuous-time companion model via ZOH, then run MPC on it
    Eigen::MatrixXd Ac(2, 2), Bc(2, 1), Cc(1, 2), Dc(1, 1);
    Ac << 0.0, 1.0, -1.0, -1.5;
    Bc << 0.0, 1.0;
    Cc << 1.0, 0.0;
    Dc << 0.0;
    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    auto sys_d = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    ctrl::MPCParams mp;
    mp.Np    = 20;  mp.Nc    = 5;
    mp.rho_y = 1.0; mp.rho_u = 0.1;
    mp.uMin  = -5.0; mp.uMax  = 5.0;
    mp.duMax =  0.2; mp.duMin = -0.2;

    ctrl::DiscreteMPC mpc(sys_d, mp);

    // First move from rest must respect duMax
    double u_first = mpc.compute(1.0);
    test::check(std::abs(u_first) <= mp.duMax + 1e-6,
                "c2d+MPC: first move respects duMax=0.2");

    // Closed-loop tracking of unit step.
    // Plant settling time ~500 steps; use Np=50 so MPC can see meaningful output change.
    ctrl::MPCParams mp_track = mp;
    mp_track.Np    = 50;
    mp_track.Nc    = 10;
    mp_track.rho_u = 0.01;  // less move suppression -> faster convergence
    mp_track.duMax =  5.0;  // relax du limit for tracking test
    mp_track.duMin = -5.0;
    ctrl::DiscreteMPC mpc_track(sys_d, mp_track);
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(sys_d.stateSize());
        double y = 0.0;
        for (int k = 0; k < 5000; ++k)
        {
            double uc = mpc_track.compute(1.0 - y);
            Eigen::VectorXd uv(1); uv << uc;
            y = ctrl::ssStep(sys_d, x, uv)(0);
        }
        test::check(std::abs(y - 1.0) < 0.10, "c2d+MPC: closed-loop tracks unit step");
    }
}

void test_n4sid_gpc_integration()
{
    test::suite("N4SID + GPC Adaptive Integration");

    // Collect PRBS data from the known 2nd-order plant
    auto true_plant = make_plant_int();
    const int N = 600;
    Eigen::MatrixXd Y(1, N), U(1, N);
    Eigen::VectorXd x_id = Eigen::VectorXd::Zero(true_plant.stateSize());
    double u_val = 1.0;
    for (int k = 0; k < N; ++k)
    {
        Eigen::VectorXd uv(1); uv << u_val;
        Y(0, k) = ctrl::ssStep(true_plant, x_id, uv)(0);
        U(0, k) = u_val;
        if (k % 7 == 6) u_val = -u_val;
    }

    // Identify plant model via n4sid
    auto id_res = ctrl::n4sid(Y, U, 2, 15, Ts);
    test::check(id_res.success, "N4SID+GPC: identification succeeds");

    if (!id_res.success || !id_res.model.has_value()) return;

    // Construct GPC from identified model (setPlant hot-swap path)
    ctrl::GPCParams gp;
    gp.Np = 15; gp.Nu = 4;
    gp.rho_y = 1.0; gp.rho_u = 0.1;
    gp.uMin = -5.0; gp.uMax = 5.0;

    test::no_throw([&] {
        ctrl::GeneralizedPredictiveController gpc(*id_res.model, gp);
        gpc.computeRef(0.0, 1.0);
    }, "N4SID+GPC: GPC on identified model computes without throw");

    // suggestOrder should return a small positive integer
    int ord = ctrl::suggestOrder(id_res.singularValues, 0.01, 5);
    test::check(ord >= 1 && ord <= 4,
                "N4SID+GPC: suggestOrder returns sensible order (1..4)");

    // Adaptive loop: RLS online + GPC setPlant hot-swap
    ctrl::GeneralizedPredictiveController gpc(*id_res.model, gp);
    ctrl::RecursiveLeastSquares           rls(2, 1, Ts, 0.98, 1e4);

    auto plant_sim = make_plant_int();
    Eigen::VectorXd xs = Eigen::VectorXd::Zero(plant_sim.stateSize());
    double y_sim = 0.0, u_sim = 0.0;
    for (int k = 0; k < 500; ++k)
    {
        double err = rls.update(y_sim, u_sim);
        // Hot-swap model every 50 steps
        if (k > 0 && k % 50 == 0)
        {
            test::no_throw([&] { gpc.setPlant(rls.toStateSpace()); },
                           "RLS+GPC: setPlant hot-swap no throw");
        }
        u_sim = gpc.computeRef(y_sim, 1.0);
        Eigen::VectorXd uv(1); uv << u_sim;
        y_sim = ctrl::ssStep(plant_sim, xs, uv)(0);
        (void)err;
    }
    test::check(std::isfinite(y_sim), "N4SID+GPC: adaptive loop stays finite");
}

int main() {
    std::cout << "============================================================\n";
    std::cout << "  Controller Toolbox - Integration & Regression Tests\n";
    std::cout << "============================================================\n";

    test_stack_extended();
    test_regressions();
    test_c2d_mpc_integration();
    test_n4sid_gpc_integration();

    test::report();
    return (test::failed == 0) ? 0 : 1;
}
