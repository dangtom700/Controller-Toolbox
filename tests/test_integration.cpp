// ============================================================
//  test_integration.cpp
//  Implements integration and regression tests from test_plan.md
// ============================================================
#include "ControllerToolbox.h"
#include "test_framework.h"

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

int main() {
    std::cout << "============================================================\n";
    std::cout << "  Controller Toolbox - Integration & Regression Tests\n";
    std::cout << "============================================================\n";

    test_stack_extended();
    test_regressions();

    test::report();
    return (test::failed == 0) ? 0 : 1;
}
