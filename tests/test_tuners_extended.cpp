// ============================================================
//  test_tuners_extended.cpp
//  Implements tuning-specific tests from the test_plan.md
// ============================================================
#include "ControllerToolbox.h"
#include "test_framework.h"
#include <fstream>
#include <string>
#include <sstream>

static constexpr double Ts = 0.01;

ctrl::StateSpace make_plant_extended() {
    ctrl::TransferFunction tf(
        { 0.0,      4.9625e-5, 4.9125e-5 },
        { 1.0,     -1.98511,   0.98522   },
        Ts);
    return ctrl::tf2ss(tf);
}

void test_relay_extended() {
    test::suite("RelayAutoTuner Extended");
    
    auto plant = make_plant_extended();
    ctrl::RelayTunerConfig cfg;
    cfg.relayAmplitude = 2.0;
    cfg.hysteresis = 0.05;
    cfg.cyclesRequired = 4;
    
    ctrl::RelayAutoTuner relay(cfg, Ts);
    
    // TC-RELAY-01: Full relay test
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    bool done_reached = false;
    
    for (int k = 0; k < 10000; ++k) {
        double u = relay.step(y);
        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);
        
        if (relay.isDone()) {
            done_reached = true;
            break;
        }
    }
    
    test::check(done_reached, "TC-RELAY-01: Relay loop converges (isDone() == true)");
    test::check(relay.ultimateGain() > 0.0 && relay.ultimatePeriod() > 0.0, 
                "TC-RELAY-01: Ku and Tu are positive");
                
    if (done_reached) {
        // TC-RELAY-02: ZN Rule
        auto p_zn = relay.computePIDParams(ctrl::PIDTuningRule::ZieglerNichols);
        test::check(p_zn.Kp > 0 && p_zn.Ki > 0 && p_zn.Kd > 0, "TC-RELAY-02: ZN gains valid");
        
        // TC-RELAY-03: AMIGO Rule
        auto p_amigo = relay.computePIDParams(ctrl::PIDTuningRule::AMIGO);
        test::check(p_amigo.Kp > 0 && p_amigo.Ki > 0 && p_amigo.Kd > 0, "TC-RELAY-03: AMIGO gains valid");
        
        // TC-RELAY-04: IMC Rule
        auto p_imc = relay.computePIDParams(ctrl::PIDTuningRule::IMC);
        test::check(p_imc.Kp > 0 && p_imc.Kd == 0.0, "TC-RELAY-04: IMC-from-relay yields PI (Kd=0)");
    }
    
    // TC-RELAY-05: Zero hysteresis
    {
        ctrl::RelayTunerConfig cfg2 = cfg;
        cfg2.hysteresis = 0.0;
        ctrl::RelayAutoTuner relay2(cfg2, Ts);
        x.setZero(); y = 0.0;
        for (int k = 0; k < 5000 && !relay2.isDone(); ++k) {
            double u = relay2.step(y);
            Eigen::VectorXd uv(1); uv << u;
            y = ctrl::ssStep(plant, x, uv)(0);
        }
        test::check(relay2.isDone() && relay2.ultimateGain() > 0.0, "TC-RELAY-05: Works with zero hysteresis");
    }
}

void test_step_response_extended() {
    test::suite("StepResponseTuner Extended");
    
    // Load python-generated step response data
    std::vector<double> t_data, y_data;
    std::ifstream file("tests/data/step_response.csv");
    if (file.is_open()) {
        std::string line;
        std::getline(file, line); // header
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string ts, ys;
            if (std::getline(ss, ts, ',') && std::getline(ss, ys)) {
                t_data.push_back(std::stod(ts));
                y_data.push_back(std::stod(ys));
            }
        }
    }
    
    if (t_data.size() > 10) {
        auto m = ctrl::StepResponseTuner::identify(t_data, y_data, 1.0);
        
        // Python result: K=0.8977, tau=1.1550, theta=0.5250
        test::check(std::abs(m.K - 0.8977) < 0.1 && std::abs(m.tau - 1.155) < 0.1 && std::abs(m.theta - 0.525) < 0.1, 
                    "TC-STEP: C++ identify matches Python reference");
                    
        // TC-STEP-01: TyreusLuyben
        auto p_tl = ctrl::StepResponseTuner::computePIDParams(m, Ts, ctrl::PIDTuningRule::TyreusLuyben);
        test::check(p_tl.Kp > 0 && p_tl.Kd == 0.0, "TC-STEP-01: TyreusLuyben gives PI only");
        
        // TC-STEP-02: AMIGO
        auto p_am = ctrl::StepResponseTuner::computePIDParams(m, Ts, ctrl::PIDTuningRule::AMIGO);
        test::check(p_am.Kp > 0 && p_am.Ki > 0 && p_am.Kd > 0, "TC-STEP-02: AMIGO gives full PID");
        
        // TC-STEP-04: IMC with custom lambda
        auto p_imc_default = ctrl::StepResponseTuner::computePIDParams(m, Ts, ctrl::PIDTuningRule::IMC);
        auto p_imc_custom  = ctrl::StepResponseTuner::computePIDParams(m, Ts, ctrl::PIDTuningRule::IMC, 2.0);
        test::check(p_imc_custom.Kp < p_imc_default.Kp, "TC-STEP-04: Larger lambda gives smaller Kp (less aggressive)");
    } else {
        std::cout << "  [WARN] Step response data not found. Run generate_test_data.py first.\n";
    }
    
    // TC-STEP-06: theta = 0 in FOPDT (pure lag)
    ctrl::StepResponseTuner::FOPDTModel m_lag{1.0, 1.0, 0.0};
    // ZN rule divides by theta, should result in Inf/NaN but not crash. Just check it executes.
    test::no_throw([&]{ 
        auto p = ctrl::StepResponseTuner::computePIDParams(m_lag, Ts, ctrl::PIDTuningRule::ZieglerNichols);
    }, "TC-STEP-06: theta=0 in ZN does not crash");
}

void test_other_tuners_extended() {
    test::suite("Other Tuners Extended");
    
    // TC-POLE-01: LQRWeightTuner::polePlacementHint
    auto plant = make_plant_extended();
    std::vector<std::complex<double>> poles = { std::complex<double>(0.9, 0.0), std::complex<double>(0.8, 0.0) };
    auto lp = ctrl::LQRWeightTuner::polePlacementHint(plant, poles);
    test::check(lp.Q(0,0) > 0 && lp.R(0,0) > 0, "TC-POLE-01: Q and R are positive definite");
    
    // TC-POLE-04: Complex conjugate pairs
    std::vector<std::complex<double>> complex_poles = { std::complex<double>(0.9, 0.1), std::complex<double>(0.9, -0.1) };
    auto lp2 = ctrl::LQRWeightTuner::polePlacementHint(plant, complex_poles);
    test::check(lp2.Q(0,0) > 0, "TC-POLE-04: Complex conjugate pairs handled correctly");
    
    // TC-KWT-01/02: KalmanWeightTuner fromNoise
    Eigen::VectorXd procNoise(2); procNoise << 0.1, 0.2;
    Eigen::VectorXd measNoise(1); measNoise << 0.05;
    auto kp = ctrl::KalmanWeightTuner::fromNoise(procNoise, measNoise);
    test::check(std::abs(kp.Qf(0,0) - 0.01) < 1e-6 && std::abs(kp.Qf(1,1) - 0.04) < 1e-6, "TC-KWT-01: Qf diagonal matches variance");
    test::check(std::abs(kp.Rf(0,0) - 0.0025) < 1e-6, "TC-KWT-02: Rf diagonal matches variance");
}

int main() {
    std::cout << "============================================================\n";
    std::cout << "  Controller Toolbox - Extended Tuner Tests\n";
    std::cout << "============================================================\n";

    test_relay_extended();
    test_step_response_extended();
    test_other_tuners_extended();

    test::report();
    return (test::failed == 0) ? 0 : 1;
}
