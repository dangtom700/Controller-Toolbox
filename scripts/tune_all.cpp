// ============================================================
//  tune_all.cpp  -  Automated tuning for every controller in lib/
//
//  Input:  example plant G(s) = 1/(s^2+1.5s+1),  ZOH Ts = 0.01 s
//  Output: tuned parameters for each controller design, printed to
//          stdout and written to tuned_params.txt for downstream use.
//
//  Tuning strategy per controller:
//    DiscretePID     -> open-loop step response -> StepResponseTuner (IMC)
//    DiscreteLQR     -> LQRWeightTuner::brysonMethod
//    DiscreteLQG     -> brysonMethod  +  KalmanWeightTuner::isotropic
//    DiscreteMPC     -> MPCHorizonTuner::recommend
//    DiscreteLeadLag -> LoopShapingTuner
//    DiscreteSMC     -> physics-based bandwidth parameterisation
//    DiscreteADRC    -> bandwidth parameterisation (Gao 2003)
//    ExtremumSeeker  -> plant-bandwidth-based dither design
//    SmithPredictor  -> inner PID via StepResponseTuner
//
//  Build: cmake target  tune_all
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>

// ---- Example plant: G(s)=1/(s^2+1.5s+1), ZOH Ts=0.01 s ------------------
static constexpr double Ts = 0.01;

static ctrl::StateSpace make_plant()
{
    ctrl::TransferFunction tf(
        {0.0, 4.9625e-5, 4.9125e-5},
        {1.0, -1.98511, 0.98522},
        Ts);
    return ctrl::tf2ss(tf);
}

// Collect open-loop step response (N samples, step magnitude = 1)
static void collect_step_response(const ctrl::StateSpace &plant,
                                  int N,
                                  std::vector<double> &t_out,
                                  std::vector<double> &y_out)
{
    t_out.resize(N);
    y_out.resize(N);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd uv(1);
    uv << 1.0;
    for (int k = 0; k < N; ++k)
    {
        y_out[k] = ctrl::ssStep(plant, x, uv)(0);
        t_out[k] = k * Ts;
    }
}

// ---- Print separator -------------------------------------------------------
static void section(const std::string &name)
{
    std::cout << "\n"
              << std::string(60, '=') << "\n";
    std::cout << "  " << name << "\n";
    std::cout << std::string(60, '=') << "\n";
}

// ============================================================
int main()
{
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "============================================================\n";
    std::cout << "  Controller Tuner  -  All controllers in lib/\n";
    std::cout << "  Plant: G(s) = 1/(s^2 + 1.5s + 1),  Ts = " << Ts << " s\n";
    std::cout << "============================================================\n";

    auto plant = make_plant();
    const int n = plant.stateSize();  // 2
    const int m = plant.inputSize();  // 1
    const int p = plant.outputSize(); // 1

    // Shared: open-loop step response for FOPDT-based tuners
    std::vector<double> t_data, y_data;
    collect_step_response(plant, 1500, t_data, y_data);
    auto fopdt = ctrl::StepResponseTuner::identify(t_data, y_data, 1.0);

    std::cout << "\n--- FOPDT Model (from step response) ---\n";
    std::cout << "  K     = " << fopdt.K << "\n";
    std::cout << "  tau   = " << fopdt.tau << " s\n";
    std::cout << "  theta = " << fopdt.theta << " s\n";

    // Open output file
    std::ofstream fout("tuned_params.txt");
    fout << std::fixed << std::setprecision(6);
    fout << "# Tuned controller parameters\n";
    fout << "# Plant: G(s)=1/(s^2+1.5s+1), ZOH Ts=" << Ts << "\n";
    fout << "# FOPDT: K=" << fopdt.K << " tau=" << fopdt.tau << " theta=" << fopdt.theta << "\n\n";

    // =========================================================
    //  1. DiscretePID - IMC tuning from step response
    // =========================================================
    section("1. DiscretePID  (IMC-PID via StepResponseTuner)");
    {
        ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::IMC);
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        pp.N = 20.0;

        std::cout << "  Kp   = " << pp.Kp << "\n"
                  << "  Ki   = " << pp.Ki << "\n"
                  << "  Kd   = " << pp.Kd << "\n"
                  << "  N    = " << pp.N << "\n"
                  << "  uMin = " << pp.uMin << "  uMax = " << pp.uMax << "\n"
                  << "  Kb   = " << pp.Kb << "\n";

        // Verify: ZN variant for comparison
        ctrl::PIDParams pp_zn = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::ZieglerNichols);
        std::cout << "  [ZN variant] Kp=" << pp_zn.Kp << " Ki=" << pp_zn.Ki
                  << " Kd=" << pp_zn.Kd << "\n";

        // Cohen-Coon (if theta > 0)
        if (fopdt.theta > 1e-6)
        {
            ctrl::PIDParams pp_cc = ctrl::CohenCoonTuner::tuneImpl(fopdt, Ts);
            std::cout << "  [Cohen-Coon]  Kp=" << pp_cc.Kp << " Ki=" << pp_cc.Ki
                      << " Kd=" << pp_cc.Kd << "\n";
        }

        fout << "[PID_IMC]\n"
             << "Kp=" << pp.Kp << "\nKi=" << pp.Ki << "\nKd=" << pp.Kd
             << "\nN=" << pp.N << "\nuMin=" << pp.uMin << "\nuMax=" << pp.uMax
             << "\nKb=" << pp.Kb << "\n\n";
    }

    // =========================================================
    //  2. DiscreteLQR - Bryson's rule
    // =========================================================
    section("2. DiscreteLQR  (Bryson's rule)");
    {
        Eigen::VectorXd xmax(n);
        xmax << 1.0, 1.0; // max allowable state deviation
        Eigen::VectorXd umax(m);
        umax << 5.0; // max allowable control effort
        ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethodFor<ctrl::DiscreteLQR>(xmax, umax);
        ctrl::DiscreteLQR lqr(plant, lp);

        std::cout << "  Q diagonal: [" << lp.Q(0, 0) << ", " << lp.Q(1, 1) << "]\n";
        std::cout << "  R          : " << lp.R(0, 0) << "\n";
        std::cout << "  K gain     : [";
        for (int j = 0; j < n; ++j)
            std::cout << lqr.gainMatrix()(0, j) << (j < n - 1 ? ", " : "");
        std::cout << "]\n";

        fout << "[LQR_Bryson]\n"
             << "Q_diag=" << lp.Q(0, 0) << "," << lp.Q(1, 1)
             << "\nR=" << lp.R(0, 0)
             << "\nK=" << lqr.gainMatrix()(0, 0) << "," << lqr.gainMatrix()(0, 1) << "\n\n";
    }

    // =========================================================
    //  3. DiscreteLQG - Bryson + isotropic Kalman noise
    // =========================================================
    section("3. DiscreteLQG  (Bryson LQR + isotropic Kalman noise)");
    {
        Eigen::VectorXd xmax(n);
        xmax << 1.0, 1.0;
        Eigen::VectorXd umax(m);
        umax << 5.0;
        ctrl::LQRParams lp = ctrl::LQRWeightTuner::brysonMethodFor<ctrl::DiscreteLQG>(xmax, umax);

        auto kp = ctrl::KalmanWeightTuner::isotropicFor<ctrl::DiscreteLQG>(
            n, p, 0.01, 0.1);

        ctrl::DiscreteLQG lqg(plant, lp, kp.Qf, kp.Rf, kp.P0);

        std::cout << "  Q diagonal : [" << lp.Q(0, 0) << ", " << lp.Q(1, 1) << "]\n";
        std::cout << "  R (control): " << lp.R(0, 0) << "\n";
        std::cout << "  Qf trace   : " << kp.Qf.trace() << "\n";
        std::cout << "  Rf trace   : " << kp.Rf.trace() << "\n";
        std::cout << "  K gain     : [";
        for (int j = 0; j < n; ++j)
            std::cout << lqg.gainMatrix()(0, j) << (j < n - 1 ? ", " : "");
        std::cout << "]\n";

        fout << "[LQG_Bryson_Isotropic]\n"
             << "Q_diag=" << lp.Q(0, 0) << "," << lp.Q(1, 1)
             << "\nR=" << lp.R(0, 0)
             << "\nQf_sigma=0.01\nRf_sigma=0.1\n"
             << "K=" << lqg.gainMatrix()(0, 0) << "," << lqg.gainMatrix()(0, 1) << "\n\n";
    }

    // =========================================================
    //  4. DiscreteMPC - MPCHorizonTuner
    // =========================================================
    section("4. DiscreteMPC  (MPCHorizonTuner)");
    {
        auto rec = ctrl::MPCHorizonTuner::recommendFor<ctrl::DiscreteMPC>(plant, Ts);
        ctrl::MPCParams mp;
        mp.Np = rec.Np;
        mp.Nc = rec.Nc;
        mp.rho_y = rec.rho_y;
        mp.rho_u = rec.rho_u;
        mp.uMin = -5.0;
        mp.uMax = 5.0;

        std::cout << "  Estimated settling time: " << rec.estimatedSettlingTime << " s\n";
        std::cout << "  Np   = " << mp.Np << "\n";
        std::cout << "  Nc   = " << mp.Nc << "\n";
        std::cout << "  rho_y= " << mp.rho_y << "\n";
        std::cout << "  rho_u= " << mp.rho_u << "\n";
        std::cout << "  uMin = " << mp.uMin << "  uMax = " << mp.uMax << "\n";

        fout << "[MPC_HorizonTuner]\n"
             << "Np=" << mp.Np << "\nNc=" << mp.Nc
             << "\nrho_y=" << mp.rho_y << "\nrho_u=" << mp.rho_u
             << "\nuMin=" << mp.uMin << "\nuMax=" << mp.uMax << "\n\n";
    }

    // =========================================================
    //  5. DiscreteLeadLag - LoopShapingTuner
    // =========================================================
    section("5. DiscreteLeadLag  (LoopShapingTuner - lead compensator)");
    {
        // Desired crossover at omega_c = 3 rad/s, phase lead = 40 deg
        // Approximate |G(j*3)| for the continuous G(s)=1/(s^2+1.5s+1)
        const double omega_c = 3.0;
        const double w = omega_c;
        const double gain_at_wc = 1.0 / std::sqrt(
                                            (1.0 - w * w) * (1.0 - w * w) + (1.5 * w) * (1.5 * w));

        ctrl::LoopShapingTuner::Input in{omega_c, 40.0, gain_at_wc};
        ctrl::LeadLagParams lp = ctrl::LoopShapingTuner::tuneFor<ctrl::DiscreteLeadLag>(in);

        std::cout << "  omega_c    = " << omega_c << " rad/s\n";
        std::cout << "  |G(jw_c)|  = " << gain_at_wc << "\n";
        std::cout << "  zero  z_c  = " << lp.continuousZero << " rad/s\n";
        std::cout << "  pole  p_c  = " << lp.continuousPole << " rad/s\n";
        std::cout << "  gain  K    = " << lp.gain << "\n";

        ctrl::DiscreteLeadLag ll(lp, Ts);
        std::cout << "  Phase at omega_c: " << ll.phaseAt(omega_c) * 180.0 / M_PI << " deg\n";

        fout << "[LeadLag_LoopShaping]\n"
             << "continuousZero=" << lp.continuousZero
             << "\ncontinuousPole=" << lp.continuousPole
             << "\ngain=" << lp.gain << "\n\n";
    }

    // =========================================================
    //  6. DiscreteSMC - bandwidth parameterisation
    // =========================================================
    section("6. DiscreteSMC  (bandwidth-parameterised)");
    {
        // c_e:  surface slope approx = closed-loop BW  (~3 rad/s -> c_e=1)
        // c_de: error rate weight (approx = 0.1 * c_e)
        // K:    switching gain >= max disturbance + some margin
        // phi:  boundary layer approx = acceptable steady-state error * c_e
        ctrl::SMCParams sp;
        sp.c_e = 1.0;
        sp.c_de = 0.1;
        sp.K = 5.0;
        sp.phi = 0.5;
        sp.uMin = -10.0;
        sp.uMax = 10.0;

        std::cout << "  c_e  = " << sp.c_e << "  (surface error weight)\n";
        std::cout << "  c_de = " << sp.c_de << "  (surface rate weight)\n";
        std::cout << "  K    = " << sp.K << "  (switching gain)\n";
        std::cout << "  phi  = " << sp.phi << "  (boundary layer thickness)\n";
        std::cout << "  uMin = " << sp.uMin << "  uMax = " << sp.uMax << "\n";

        fout << "[SMC_BandwidthParam]\n"
             << "c_e=" << sp.c_e << "\nc_de=" << sp.c_de
             << "\nK=" << sp.K << "\nphi=" << sp.phi
             << "\nuMin=" << sp.uMin << "\nuMax=" << sp.uMax << "\n\n";
    }

    // =========================================================
    //  7. DiscreteADRC - Gao 2003 bandwidth parameterisation
    // =========================================================
    section("7. DiscreteADRC  (bandwidth parameterisation, Gao 2003)");
    {
        // From FOPDT: approximate b0 approx = K/tau
        const double b0 = fopdt.K / fopdt.tau;
        const double omega_c = 3.0;           // desired closed-loop BW [rad/s]
        const double omega_o = 5.0 * omega_c; // ESO BW: 5* controller BW (rule of thumb)

        ctrl::ADRCParams ap;
        ap.omega_o = omega_o;
        ap.omega_c = omega_c;
        ap.b0 = b0;
        ap.uMin = -20.0;
        ap.uMax = 20.0;

        std::cout << "  b0      = " << ap.b0 << "  (approx input gain K/tau)\n";
        std::cout << "  omega_c = " << ap.omega_c << " rad/s\n";
        std::cout << "  omega_o = " << ap.omega_o << " rad/s  (5* omega_c)\n";
        std::cout << "  uMin    = " << ap.uMin << "  uMax = " << ap.uMax << "\n";

        fout << "[ADRC_Gao2003]\n"
             << "b0=" << ap.b0 << "\nomega_c=" << ap.omega_c
             << "\nomega_o=" << ap.omega_o
             << "\nuMin=" << ap.uMin << "\nuMax=" << ap.uMax << "\n\n";
    }

    // =========================================================
    //  8. ExtremumSeeker - plant-bandwidth-based dither design
    // =========================================================
    section("8. ExtremumSeeker  (plant-bandwidth-based dither)");
    {
        // Rule: f_p >> plant BW, f_lpf < f_p, f_hpf < f_lpf
        // Plant BW ~ 1/(2*pi*tau) approx = 0.08 Hz  -> f_p = 5 Hz
        const double plant_bw_hz = 1.0 / (2.0 * M_PI * fopdt.tau);

        ctrl::ExtremumSeekerParams ep;
        ep.perturbFreq = std::max(5.0 * plant_bw_hz, 1.0); // at least 5* plant BW
        ep.perturbAmp = 0.05;                              // small relative to output range
        ep.lpfCutoff = ep.perturbFreq / 10.0;              // decade below dither
        ep.hpfCutoff = ep.lpfCutoff / 5.0;                 // below LPF
        ep.integGain = 0.5;
        ep.seekMinimum = true;

        std::cout << "  Plant BW   ~ " << plant_bw_hz << " Hz\n";
        std::cout << "  perturbFreq = " << ep.perturbFreq << " Hz\n";
        std::cout << "  perturbAmp  = " << ep.perturbAmp << "\n";
        std::cout << "  lpfCutoff   = " << ep.lpfCutoff << " Hz\n";
        std::cout << "  hpfCutoff   = " << ep.hpfCutoff << " Hz\n";
        std::cout << "  integGain   = " << ep.integGain << "\n";

        fout << "[ESC_PlantBW]\n"
             << "perturbFreq=" << ep.perturbFreq << "\nperturbAmp=" << ep.perturbAmp
             << "\nlpfCutoff=" << ep.lpfCutoff << "\nhpfCutoff=" << ep.hpfCutoff
             << "\nintegGain=" << ep.integGain << "\nseekMinimum=1\n\n";
    }

    // =========================================================
    //  9. SmithPredictor - inner PID via IMC, delay = 5 steps
    // =========================================================
    section("9. SmithPredictor  (inner IMC-PID, delay = 5 steps)");
    {
        ctrl::PIDParams pp = ctrl::StepResponseTuner::computePIDParams(
            fopdt, Ts, ctrl::PIDTuningRule::IMC);
        pp.uMin = -10.0;
        pp.uMax = 10.0;
        pp.N = 20.0;
        const int delay_steps = 5;

        std::cout << "  Inner PID: Kp=" << pp.Kp << " Ki=" << pp.Ki
                  << " Kd=" << pp.Kd << "\n";
        std::cout << "  Dead-time model: same plant (delay-free part)\n";
        std::cout << "  delaySteps = " << delay_steps << "  ("
                  << delay_steps * Ts * 1000 << " ms)\n";

        fout << "[SmithPredictor_IMC]\n"
             << "inner_Kp=" << pp.Kp << "\ninner_Ki=" << pp.Ki
             << "\ninner_Kd=" << pp.Kd << "\ninner_N=" << pp.N
             << "\ndelaySteps=" << delay_steps << "\n\n";
    }

    fout.close();

    std::cout << "\n============================================================\n";
    std::cout << "  Tuned parameters written to tuned_params.txt\n";
    std::cout << "============================================================\n";

    return 0;
}
