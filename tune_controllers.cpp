// ============================================================
//  tune_controllers.cpp
//
//  Input:  A list of controller types (ControllerDesign) + a plant model
//          (TransferFunction or StateSpace).
//  Output: Printed tuned parameters for each controller in the list.
//
//  Supported controller types and their auto-tuning strategy:
//    DiscretePID      -> RelayAutoTuner (relay feedback test on plant)
//    DiscreteLQR      -> LQRWeightTuner::brysonMethod
//    DiscreteLQG      -> LQRWeightTuner::brysonMethod + KalmanWeightTuner::isotropic
//    DiscreteMPC      -> MPCHorizonTuner::recommend
//    DiscreteLeadLag  -> LoopShapingTuner (requires omega_c, phase_add_deg)
//    DiscreteSMC      -> Relay-based heuristic (Ku, Tu -> c_e, K, phi)
//    DiscreteADRC     -> Bandwidth parameterisation (omega_c, omega_o, b0)
//    SmithPredictor   -> PID inner controller tuned via relay
//    ExtremumSeeker   -> Manual (no closed-form tuner; parameters printed as-is)
//
//  Build:
//    cd build && cmake .. && cmake --build . --target tune_controllers
//  Run:
//    ./tune_controllers
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <variant>
#include <functional>

namespace
{

    // -- Plant model (second-order example: G(s)=1/(s^2+1.5s+1), ZOH Ts=0.01s) --
    constexpr double Ts = 0.01;

    ctrl::StateSpace buildExamplePlant()
    {
        ctrl::TransferFunction tf(
            {0.0, 4.9625e-5, 4.9125e-5},
            {1.0, -1.98511, 0.98522},
            Ts);
        return ctrl::tf2ss(tf);
    }

    // -- Relay test helper (simulates relay on the given plant) --
    ctrl::RelayAutoTuner runRelayTest(const ctrl::StateSpace &plant,
                                      double relayAmp = 0.5,
                                      int cyclesReq = 4)
    {
        ctrl::RelayTunerConfig cfg;
        cfg.relayAmplitude = relayAmp;
        cfg.cyclesRequired = cyclesReq;

        ctrl::RelayAutoTuner tuner(cfg, plant.Ts);
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        double y = 0.0;

        while (!tuner.isDone())
        {
            Eigen::VectorXd uv(1);
            uv << tuner.step(y);
            y = ctrl::ssStep(plant, x, uv)(0);
        }
        return tuner;
    }

    // -----------------------------------------------------------------------------
    //  Controller design descriptors
    // -----------------------------------------------------------------------------

    struct PIDDesign
    {
        ctrl::PIDTuningRule rule = ctrl::PIDTuningRule::TyreusLuyben;
        double uMin = -10.0, uMax = 10.0;
    };

    struct LQRDesign
    {
        Eigen::VectorXd xmax; // maximum state deviations (Bryson)
        Eigen::VectorXd umax; // maximum control efforts
    };

    struct LQGDesign
    {
        Eigen::VectorXd xmax;
        Eigen::VectorXd umax;
        double sigmaProcess = 0.01;
        double sigmaMeas = 0.10;
    };

    struct MPCDesign
    {
        double rho_y = 1.0;
        double rho_u = 0.1;
        double uMin = -10.0, uMax = 10.0;
    };

    struct LeadLagDesign
    {
        double omega_c = 5.0;        // desired crossover [rad/s]
        double phase_add_deg = 45.0; // phase lead to add [deg]
        double gain_at_wc = 1.0;     // |G(jomegac)| before compensator
    };

    struct SMCDesign
    {
        // Derived from relay test: c_e, K, phi set heuristically
        double c_e = 1.0;
        double c_de = 0.5;
        double phi = 0.3;
        double uMin = -10.0, uMax = 10.0;
    };

    struct ADRCDesign
    {
        double omega_c = 5.0;
        double omega_o = 20.0; // typically 3-10* omega_c
        double b0 = 1.0;
        double uMin = -20.0, uMax = 20.0;
    };

    struct SmithPredictorDesign
    {
        int delaySteps = 5;
        double uMin = -10.0, uMax = 10.0;
    };

    struct ESCDesign
    {
        double perturbAmp = 0.1;
        double perturbFreq = 1.0;
        double lpfCutoff = 0.1;
        double hpfCutoff = 0.05;
        double integGain = 1.0;
        bool seekMinimum = true;
    };

    using ControllerDesign = std::variant<
        PIDDesign,
        LQRDesign,
        LQGDesign,
        MPCDesign,
        LeadLagDesign,
        SMCDesign,
        ADRCDesign,
        SmithPredictorDesign,
        ESCDesign>;

    // -----------------------------------------------------------------------------
    //  Tuning dispatcher
    // -----------------------------------------------------------------------------

    void tuneAndPrint(const ControllerDesign &design, const ctrl::StateSpace &plant)
    {
        std::cout << std::fixed << std::setprecision(6);
        const int n = plant.stateSize();
        const int m = plant.inputSize();
        const int p = plant.outputSize();

        std::visit([&](auto &&d)
                   {
        using T = std::decay_t<decltype(d)>;

        // -- PID --------------------------------------------------------------
        if constexpr (std::is_same_v<T, PIDDesign>) {
            std::cout << "=== DiscretePID - Relay Auto-Tuner ===\n";
            auto tuner = runRelayTest(plant);
            std::cout << "  Relay test: Ku=" << tuner.ultimateGain()
                      << "  Tu=" << tuner.ultimatePeriod() << " s\n";

            ctrl::PIDParams pp = tuner.computePIDParamsFor<ctrl::DiscretePID>(d.rule);
            pp.uMin = d.uMin; pp.uMax = d.uMax;

            const char* rule_name = (d.rule == ctrl::PIDTuningRule::ZieglerNichols) ? "ZN" :
                                    (d.rule == ctrl::PIDTuningRule::TyreusLuyben)   ? "TyreusLuyben" :
                                    (d.rule == ctrl::PIDTuningRule::IMC)            ? "IMC" : "AMIGO";
            std::cout << "  Rule: " << rule_name << "\n";
            std::cout << "  Kp=" << pp.Kp << "  Ki=" << pp.Ki
                      << "  Kd=" << pp.Kd << "  N=" << pp.N
                      << "  Kb=" << pp.Kb << "\n";
            std::cout << "  uMin=" << pp.uMin << "  uMax=" << pp.uMax << "\n\n";
        }

        // -- LQR --------------------------------------------------------------
        else if constexpr (std::is_same_v<T, LQRDesign>) {
            std::cout << "=== DiscreteLQR - Bryson's Method ===\n";
            ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethodFor<ctrl::DiscreteLQR>(
                d.xmax, d.umax);
            std::cout << "  Q =\n" << lp.Q << "\n";
            std::cout << "  R =\n" << lp.R << "\n";
            ctrl::DiscreteLQR lqr(plant, lp);
            std::cout << "  Gain K =\n" << lqr.gainMatrix() << "\n\n";
        }

        // -- LQG --------------------------------------------------------------
        else if constexpr (std::is_same_v<T, LQGDesign>) {
            std::cout << "=== DiscreteLQG - Bryson + Kalman Noise Tuner ===\n";
            ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethodFor<ctrl::DiscreteLQG>(
                d.xmax, d.umax);
            ctrl::KalmanNoiseParams kp = ctrl::KalmanWeightTuner::isotropicFor<ctrl::DiscreteLQG>(
                n, p, d.sigmaProcess, d.sigmaMeas);
            std::cout << "  Q(LQR) =\n" << lp.Q << "\n";
            std::cout << "  R(LQR) =\n" << lp.R << "\n";
            std::cout << "  Qf(Kalman process noise) =\n" << kp.Qf << "\n";
            std::cout << "  Rf(Kalman meas noise)    =\n" << kp.Rf << "\n";
            ctrl::DiscreteLQG lqg(plant, lp, kp.Qf, kp.Rf, kp.P0);
            std::cout << "  LQR Gain K =\n" << lqg.gainMatrix() << "\n\n";
        }

        // -- MPC --------------------------------------------------------------
        else if constexpr (std::is_same_v<T, MPCDesign>) {
            std::cout << "=== DiscreteMPC - MPCHorizonTuner ===\n";
            auto rec = ctrl::MPCHorizonTuner::recommendFor<ctrl::DiscreteMPC>(
                plant, plant.Ts, d.rho_y, d.rho_u);
            std::cout << "  Estimated settling time: " << rec.estimatedSettlingTime << " s\n";
            std::cout << "  Np=" << rec.Np << "  Nc=" << rec.Nc
                      << "  rho_y=" << rec.rho_y << "  rho_u=" << rec.rho_u << "\n";
            std::cout << "  uMin=" << d.uMin << "  uMax=" << d.uMax << "\n\n";
        }

        // -- Lead-Lag ----------------------------------------------------------
        else if constexpr (std::is_same_v<T, LeadLagDesign>) {
            std::cout << "=== DiscreteLeadLag - LoopShapingTuner ===\n";
            ctrl::LoopShapingTuner::Input in;
            in.omega_c       = d.omega_c;
            in.phase_add_deg = d.phase_add_deg;
            in.gain_at_wc    = d.gain_at_wc;
            ctrl::LeadLagParams lp =
                ctrl::LoopShapingTuner::tuneFor<ctrl::DiscreteLeadLag>(in);
            std::cout << "  continuousZero z_c = " << lp.continuousZero << " rad/s\n";
            std::cout << "  continuousPole p_c = " << lp.continuousPole << " rad/s\n";
            std::cout << "  Gain K             = " << lp.gain << "\n";
            ctrl::DiscreteLeadLag ll(lp, plant.Ts);
            std::cout << "  Phase at omega_c   = "
                      << ll.phaseAt(d.omega_c) * 180.0 / M_PI << " deg\n\n";
        }

        // -- SMC --------------------------------------------------------------
        else if constexpr (std::is_same_v<T, SMCDesign>) {
            std::cout << "=== DiscreteSMC - Relay-Heuristic ===\n";
            auto tuner = runRelayTest(plant);
            const double Ku = tuner.ultimateGain();
            // Heuristic: K ~ 0.5*Ku, phi from boundary-layer spec
            ctrl::SMCParams sp;
            sp.c_e  = d.c_e;
            sp.c_de = d.c_de;
            sp.K    = 0.5 * Ku;
            sp.phi  = d.phi;
            sp.uMin = d.uMin;
            sp.uMax = d.uMax;
            std::cout << "  Relay: Ku=" << Ku << "  Tu=" << tuner.ultimatePeriod() << " s\n";
            std::cout << "  c_e=" << sp.c_e << "  c_de=" << sp.c_de
                      << "  K=" << sp.K << "  phi=" << sp.phi << "\n";
            std::cout << "  uMin=" << sp.uMin << "  uMax=" << sp.uMax << "\n\n";
        }

        // -- ADRC -------------------------------------------------------------
        else if constexpr (std::is_same_v<T, ADRCDesign>) {
            std::cout << "=== DiscreteADRC - Bandwidth Parameterisation ===\n";
            ctrl::ADRCParams ap;
            ap.omega_c = d.omega_c;
            ap.omega_o = d.omega_o;
            ap.b0      = d.b0;
            ap.uMin    = d.uMin;
            ap.uMax    = d.uMax;
            std::cout << "  omega_c=" << ap.omega_c << " rad/s  (controller BW)\n";
            std::cout << "  omega_o=" << ap.omega_o << " rad/s  (ESO BW, ~" 
                      << ap.omega_o / ap.omega_c << "x omega_c)\n";
            std::cout << "  b0=" << ap.b0 << "  (approx. plant input gain)\n";
            std::cout << "  => beta1=" << 3.0*ap.omega_o
                      << "  beta2=" << 3.0*ap.omega_o*ap.omega_o
                      << "  beta3=" << ap.omega_o*ap.omega_o*ap.omega_o << "\n";
            std::cout << "  uMin=" << ap.uMin << "  uMax=" << ap.uMax << "\n\n";
        }

        // -- Smith Predictor --------------------------------------------------
        else if constexpr (std::is_same_v<T, SmithPredictorDesign>) {
            std::cout << "=== SmithPredictor - Inner PID via Relay ===\n";
            auto tuner = runRelayTest(plant);
            ctrl::PIDParams pp = tuner.computePIDParams(ctrl::PIDTuningRule::TyreusLuyben);
            pp.uMin = d.uMin; pp.uMax = d.uMax;
            std::cout << "  Dead-time steps d=" << d.delaySteps
                      << " (" << d.delaySteps * plant.Ts << " s)\n";
            std::cout << "  Inner PID: Kp=" << pp.Kp << "  Ki=" << pp.Ki
                      << "  Kd=" << pp.Kd << "\n";
            std::cout << "  => Construct: SmithPredictor(inner_pid, delay_free_model, "
                      << d.delaySteps << ")\n\n";
        }

        // -- ESC --------------------------------------------------------------
        else if constexpr (std::is_same_v<T, ESCDesign>) {
            std::cout << "=== ExtremumSeeker - Manual Bandwidth Parameterisation ===\n";
            std::cout << "  perturbAmp="  << d.perturbAmp
                      << "  perturbFreq=" << d.perturbFreq << " Hz\n";
            std::cout << "  lpfCutoff="   << d.lpfCutoff
                      << " Hz  hpfCutoff=" << d.hpfCutoff << " Hz\n";
            std::cout << "  integGain="   << d.integGain
                      << "  seekMinimum=" << (d.seekMinimum ? "true" : "false") << "\n";
            std::cout << "  NOTE: No closed-form auto-tuner exists for ESC.\n"
                      << "        Verify: plant_BW << f_p << 1/(Ts*N_filter)\n\n";
        } }, design);
    }

} // anonymous namespace

// -----------------------------------------------------------------------------
//  main - edit this list to select which controllers to tune
// -----------------------------------------------------------------------------
int main()
{
    // -- Plant model ----------------------------------------------------------
    ctrl::StateSpace plant = buildExamplePlant();
    const int n = plant.stateSize();
    const int m = plant.inputSize();

    std::cout << "Controller Tuning Script\n";
    std::cout << "Plant: n=" << n << " states, m=" << m << " inputs, Ts=" << Ts << "s\n";
    std::cout << std::string(60, '=') << "\n\n";

    // -- Controller design list -----------------------------------------------
    // Edit this vector to select which controllers to tune.
    std::vector<ControllerDesign> designList;

    // PID via Tyreus-Luyben
    designList.push_back(PIDDesign{ctrl::PIDTuningRule::TyreusLuyben, -10.0, 10.0});

    // LQR via Bryson (xmax=[1,1], umax=[10])
    {
        LQRDesign d;
        d.xmax.resize(n);
        d.xmax.setOnes();
        d.umax.resize(m);
        d.umax << 10.0;
        designList.push_back(d);
    }

    // LQG via Bryson + isotropic noise
    {
        LQGDesign d;
        d.xmax.resize(n);
        d.xmax.setOnes();
        d.umax.resize(m);
        d.umax << 10.0;
        d.sigmaProcess = 0.01;
        d.sigmaMeas = 0.10;
        designList.push_back(d);
    }

    // MPC
    designList.push_back(MPCDesign{1.0, 0.1, -10.0, 10.0});

    // Lead compensator at omega_c=5 rad/s, +45 deg
    designList.push_back(LeadLagDesign{5.0, 45.0, 1.0});

    // SMC
    designList.push_back(SMCDesign{1.0, 0.5, 0.3, -10.0, 10.0});

    // ADRC
    designList.push_back(ADRCDesign{5.0, 20.0, 1.0, -20.0, 20.0});

    // Smith Predictor (5-step delay inner PID)
    designList.push_back(SmithPredictorDesign{5, -10.0, 10.0});

    // ESC
    designList.push_back(ESCDesign{0.1, 1.0, 0.1, 0.05, 1.0, true});

    // -- Tune and print -------------------------------------------------------
    for (const auto &d : designList)
        tuneAndPrint(d, plant);

    return 0;
}
