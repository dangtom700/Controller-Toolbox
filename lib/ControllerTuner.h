#pragma once
#include "PlantModel.h"
#include "DiscretePID.h"
#include "DiscreteLQR.h"
#include "DiscreteMPC.h"
#include "DiscreteLeadLag.h"
#include "ControllerTraits.h"
#include <vector>
#include <complex>
#include <functional>

// ============================================================
//  ControllerTuner.h  - Offline and online tuning utilities
//
//  Every tuner exposes two interfaces:
//
//    tuneImpl(...)        - unchecked, for internal use or when
//                           the controller type is already known.
//
//    tuneFor<C>(...)      - template wrapper; triggers a
//                           static_assert (hard compiler error)
//                           when C is incompatible, with a
//                           diagnostic naming the correct tuner.
//
//  Existing tuners (RelayAutoTuner, StepResponseTuner,
//  LQRWeightTuner, MPCHorizonTuner) gain an additional
//  *For<C>() overload - their original methods are unchanged.
//
//  New tuners added here:
//    ZieglerNicholsTuner  - standalone ZN from (Ku, Tu)
//    CohenCoonTuner       - FOPDT-based PID (Cohen & Coon 1953)
//    LoopShapingTuner     - crossover freq + phase margin -> LeadLagParams
//    KalmanWeightTuner    - noise levels -> Qf / Rf for DiscreteLQG
//
//  Strategies from the literature that require external solvers
//  or adaptive runtime infrastructure are NOT implemented here:
//    Hinf / mu-synthesis   -> requires SLICOT, Python-control, or MATLAB
//    QFT                -> requires CAD tool (QFT Toolbox, QDESIGN)
//    GA / PSO / DE      -> use an optimisation framework (Optuna, DEAP)
//    Bayesian opt.      -> use BoTorch / GPyOpt / scikit-optimize
//    MRAC / STR / IFT / VRFT -> adaptive control; separate library
//
//  Ref: Åström & Hägglund (1988, 2006); Bryson & Ho (1975);
//       Cohen & Coon (1953); Rivera, Morari & Skogestad (1986);
//       Camacho & Bordons MPC Ch 3; Franklin, Powell & Emami-Naeini.
// ============================================================
namespace ctrl
{

    // ============================================================
    // PID Tuning Rule Selector
    // ============================================================
    enum class PIDTuningRule
    {
        ZieglerNichols, // Classic ZN - fast, ~25 % overshoot  (Ziegler & Nichols 1942)
        TyreusLuyben,   // Conservative ZN variant - better load rejection (Tyreus & Luyben 1992)
        IMC,            // Internal Model Control (lambda-tuning) - needs FOPDT model + lambda
        AMIGO           // Approximate M-constrained Integral Gain Optimisation (Åström 2006)
    };

    // ============================================================
    // Relay Auto-Tuner  (Åström-Hägglund relay feedback test)
    // ============================================================
    // Applies a relay +/-d to the closed plant, records the resulting limit cycle,
    // then derives ultimate gain Ku and ultimate period Tu.
    //   Ku = (4.d) / (pi.a_y)   where a_y = peak-to-peak output amplitude / 2
    //   Tu = average oscillation period from zero-crossing times
    //
    // Usage pattern (simulation loop):
    //   while (!tuner.isDone()) {
    //       u = tuner.step(plant_output);
    //       apply u, advance plant;
    //   }
    //   auto params = tuner.computePIDParams(PIDTuningRule::TyreusLuyben);
    // ============================================================
    struct RelayTunerConfig
    {
        double relayAmplitude = 1.0; // Relay output amplitude +/-d
        double hysteresis = 0.0;     // Dead-band to prevent noise-driven chatter
        int cyclesRequired = 3;      // Stable oscillation cycles needed before exit
    };

    class RelayAutoTuner
    {
    public:
        RelayAutoTuner(const RelayTunerConfig &cfg, double sampleTime);

        // Feed y[k], receive relay output u[k].
        double step(double y);

        bool isDone() const { return done_; }

        // Original unchecked method - unchanged for backward compatibility.
        PIDParams computePIDParams(PIDTuningRule rule = PIDTuningRule::TyreusLuyben,
                                   double lambda = -1.0) const;

        // Type-checked wrapper: hard compiler error for non-PID controllers.
        template <typename C>
        PIDParams computePIDParamsFor(PIDTuningRule rule = PIDTuningRule::TyreusLuyben,
                                      double lambda = -1.0) const
        {
            static_assert(ControllerTraits<C>::supports_heuristic_pid,
                          "\n[RelayAutoTuner::computePIDParamsFor<C>]"
                          " C does not support relay-based PID tuning.\n"
                          "  Relay tuning produces PIDParams from (Ku, Tu);"
                          " only valid for DiscretePID.\n"
                          "  --> DiscreteLQR / DiscreteLQG :"
                          " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                          "  --> DiscreteMPC               :"
                          " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                          "  --> DiscreteLeadLag           :"
                          " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                          "  --> DiscreteSMC / ADRC / ESC  :"
                          " set parameters directly (no standard auto-tuner)\n"
                          "  --> SmithPredictor            :"
                          " tune the inner IController, not the wrapper\n");
            return computePIDParams(rule, lambda);
        }

        double ultimateGain() const { return Ku_; }
        double ultimatePeriod() const { return Tu_; }

    private:
        RelayTunerConfig cfg_;
        double Ts_;
        bool done_ = false;
        bool relayHigh_ = false;
        bool collecting_ = false;
        double Ku_ = 0.0;
        double Tu_ = 0.0;
        double y_prev_ = 0.0;
        double peak_pos_ = -1e9;
        double peak_neg_ = 1e9;
        int step_cnt_ = 0;
        std::vector<double> crossTimes_;
    };

    // ============================================================
    // Step-Response Tuner  (open-loop FOPDT identification)
    // ============================================================
    // Fits a First-Order Plus Dead-Time (FOPDT) model to open-loop
    // step response data using the process-reaction-curve tangent method.
    //   G(s) approx = K.e^{-θs} / (τs + 1)
    // ============================================================
    class StepResponseTuner
    {
    public:
        struct FOPDTModel
        {
            double K;     // Process (static) gain
            double tau;   // First-order time constant  [s]
            double theta; // Dead time                  [s]
        };

        static FOPDTModel identify(const std::vector<double> &time,
                                   const std::vector<double> &output,
                                   double stepMagnitude);

        // Original unchecked method - unchanged.
        static PIDParams computePIDParams(const FOPDTModel &model,
                                          double Ts,
                                          PIDTuningRule rule = PIDTuningRule::IMC,
                                          double lambda = -1.0);

        // Type-checked wrapper.
        template <typename C>
        static PIDParams computePIDParamsFor(const FOPDTModel &model,
                                             double Ts,
                                             PIDTuningRule rule = PIDTuningRule::IMC,
                                             double lambda = -1.0)
        {
            static_assert(ControllerTraits<C>::supports_heuristic_pid,
                          "\n[StepResponseTuner::computePIDParamsFor<C>]"
                          " C does not support FOPDT-based PID tuning.\n"
                          "  FOPDT tuning produces PIDParams; only valid for DiscretePID.\n"
                          "  --> DiscreteLQR / DiscreteLQG :"
                          " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                          "  --> DiscreteMPC               :"
                          " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                          "  --> DiscreteLeadLag           :"
                          " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                          "  --> DiscreteSMC / ADRC / ESC  :"
                          " set parameters directly (no standard auto-tuner)\n"
                          "  --> SmithPredictor            :"
                          " tune the inner IController, not the wrapper\n");
            return computePIDParams(model, Ts, rule, lambda);
        }
    };

    // ============================================================
    // LQR Weight Tuner
    // ============================================================
    class LQRWeightTuner
    {
    public:
        // Original unchecked methods - unchanged.
        static LQRParams brysonMethod(const Eigen::VectorXd &maxStateDeviation,
                                      const Eigen::VectorXd &maxControlEffort);

        static LQRParams polePlacementHint(const StateSpace &plant,
                                           const std::vector<std::complex<double>> &desiredPoles,
                                           int maxIter = 200);

        // Type-checked wrapper for Bryson's method.
        template <typename C>
        static LQRParams brysonMethodFor(const Eigen::VectorXd &maxStateDeviation,
                                         const Eigen::VectorXd &maxControlEffort)
        {
            static_assert(ControllerTraits<C>::supports_lqr_tuning,
                          "\n[LQRWeightTuner::brysonMethodFor<C>]"
                          " C does not support LQR weight tuning.\n"
                          "  Bryson's method returns LQRParams (Q, R diagonal matrices);\n"
                          "  only valid for DiscreteLQR and DiscreteLQG.\n"
                          "  --> DiscretePID     : ZieglerNicholsTuner or"
                          " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                          "  --> DiscreteMPC     : MPC has its own Q/R via MPCParams;"
                          " use MPCHorizonTuner::recommendFor<DiscreteMPC>()\n"
                          "  --> DiscreteLeadLag : LoopShapingTuner::tuneFor<DiscreteLeadLag>()\n"
                          "  --> DiscreteSMC / ADRC / ESC :"
                          " set parameters directly (no standard auto-tuner)\n");
            return brysonMethod(maxStateDeviation, maxControlEffort);
        }

        // Type-checked wrapper for pole-placement hint.
        // Emits a [[deprecated]] WARNING (not error) when C = DiscreteLQG,
        // because pole placement only tunes the LQR part - the Kalman
        // observer gains (Qf, Rf) still need separate attention.
        template <typename C>
        static LQRParams polePlacementHintFor(const StateSpace &plant,
                                              const std::vector<std::complex<double>> &desiredPoles,
                                              int maxIter = 200)
        {
            static_assert(ControllerTraits<C>::supports_lqr_tuning,
                          "\n[LQRWeightTuner::polePlacementHintFor<C>]"
                          " C does not support LQR weight tuning.\n"
                          "  Pole-placement hint adjusts LQR Q/R to steer closed-loop poles;\n"
                          "  only valid for DiscreteLQR and DiscreteLQG.\n"
                          "  --> DiscretePID     : ZieglerNicholsTuner or"
                          " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                          "  --> DiscreteMPC     : use MPCHorizonTuner::recommendFor<DiscreteMPC>()\n"
                          "  --> DiscreteLeadLag : LoopShapingTuner::tuneFor<DiscreteLeadLag>()\n");

            // LQG warning: observer gains are not addressed by pole placement.
            if constexpr (std::is_same_v<C, DiscreteLQG>)
            {
                detail::emit_PolePlacement_LQG_Warning<C>();
            }

            return polePlacementHint(plant, desiredPoles, maxIter);
        }
    };

    // ============================================================
    // MPC Horizon Tuner
    // ============================================================
    class MPCHorizonTuner
    {
    public:
        struct Recommendation
        {
            int Np;
            int Nc;
            double rho_y;
            double rho_u;
            double estimatedSettlingTime;
        };

        static double estimateSettlingTime(const StateSpace &plant, int maxSteps = 5000);

        // Original unchecked method - unchanged.
        static Recommendation recommend(const StateSpace &plant,
                                        double Ts,
                                        double rho_y = 1.0,
                                        double rho_u = 0.1);

        // Type-checked wrapper.
        template <typename C>
        static Recommendation recommendFor(const StateSpace &plant,
                                           double Ts,
                                           double rho_y = 1.0,
                                           double rho_u = 0.1)
        {
            static_assert(ControllerTraits<C>::supports_mpc_tuning,
                          "\n[MPCHorizonTuner::recommendFor<C>]"
                          " C does not support MPC horizon tuning.\n"
                          "  MPC horizon tuning (Np, Nc, rho) only applies to DiscreteMPC.\n"
                          "  --> DiscretePID     :"
                          " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                          "  --> DiscreteLQR     :"
                          " LQRWeightTuner::brysonMethodFor<DiscreteLQR>(xmax, umax)\n"
                          "  --> DiscreteLQG     :"
                          " LQRWeightTuner::brysonMethodFor<DiscreteLQG>()"
                          " + KalmanWeightTuner::fromNoiseFor<DiscreteLQG>()\n"
                          "  --> DiscreteLeadLag :"
                          " LoopShapingTuner::tuneFor<DiscreteLeadLag>()\n"
                          "  --> DiscreteSMC / ADRC / ESC :"
                          " set parameters directly (no standard auto-tuner)\n");
            return recommend(plant, Ts, rho_y, rho_u);
        }
    };

    // ============================================================
    // KalmanNoiseParams - output of KalmanWeightTuner
    // ============================================================
    struct KalmanNoiseParams
    {
        Eigen::MatrixXd Qf; // process noise covariance  (n*n)
        Eigen::MatrixXd Rf; // measurement noise covariance (p*p)
        Eigen::MatrixXd P0; // initial state covariance estimate
    };

    // ============================================================
    // Ziegler-Nichols Tuner  (standalone, from relay-test data)
    // ============================================================
    // Classic ZN rules derived from ultimate gain Ku and period Tu.
    // Produces aggressive settings; consider de-tuning Kp by 20-30 %
    // or switching to TyreusLuyben / AMIGO for better robustness.
    //
    // Ref: Ziegler & Nichols "Optimum Settings for Automatic Controllers" (1942).
    // ============================================================
    class ZieglerNicholsTuner
    {
    public:
        struct Input
        {
            double Ku; // ultimate gain from relay test or manual search
            double Tu; // ultimate period [s]
        };

        // Unchecked - for internal delegation or when C is known externally.
        static PIDParams tuneImpl(const Input &in);

        // Type-checked: hard compiler error for non-PID controllers.
        template <typename C>
        static PIDParams tuneFor(const Input &in)
        {
            static_assert(ControllerTraits<C>::supports_heuristic_pid,
                          "\n[ZieglerNicholsTuner::tuneFor<C>]"
                          " C does not support ZN tuning.\n"
                          "  ZN produces PIDParams from (Ku, Tu);"
                          " only valid for DiscretePID.\n"
                          "  --> DiscreteLQR / DiscreteLQG :"
                          " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                          "  --> DiscreteMPC               :"
                          " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                          "  --> DiscreteLeadLag           :"
                          " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                          "  --> DiscreteSMC / ADRC / ESC  :"
                          " set parameters directly (no standard auto-tuner)\n"
                          "  --> SmithPredictor            :"
                          " tune the inner IController, not the wrapper\n");
            return tuneImpl(in);
        }
    };

    // ============================================================
    // Cohen-Coon Tuner  (FOPDT process-reaction-curve method)
    // ============================================================
    // More accurate than ZN for processes where dead time θ is a
    // significant fraction of the lag τ (θ/τ in [0.1, 1.0]).
    //
    // Formulas (PID, Cohen & Coon 1953):
    //   Kp = (τ / (K.θ)) . (4/3 + r/4)    where r = θ/τ
    //   Ti = θ . (32 + 6r) / (13 + 8r)
    //   Td = 4θ / (11 + 2r)
    //
    // Ref: Cohen & Coon "Theoretical Consideration of Retarded Control" (1953).
    // ============================================================
    class CohenCoonTuner
    {
    public:
        // Requires a FOPDT model (obtain via StepResponseTuner::identify).
        static PIDParams tuneImpl(const StepResponseTuner::FOPDTModel &m, double Ts);

        template <typename C>
        static PIDParams tuneFor(const StepResponseTuner::FOPDTModel &m, double Ts)
        {
            static_assert(ControllerTraits<C>::supports_heuristic_pid,
                          "\n[CohenCoonTuner::tuneFor<C>]"
                          " C does not support Cohen-Coon tuning.\n"
                          "  Cohen-Coon produces PIDParams from a FOPDT model;"
                          " only valid for DiscretePID.\n"
                          "  --> DiscreteLQR / DiscreteLQG :"
                          " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                          "  --> DiscreteMPC               :"
                          " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                          "  --> DiscreteLeadLag           :"
                          " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                          "  --> DiscreteSMC / ADRC / ESC  :"
                          " set parameters directly (no standard auto-tuner)\n"
                          "  --> SmithPredictor            :"
                          " tune the inner IController, not the wrapper\n");
            return tuneImpl(m, Ts);
        }
    };

    // ============================================================
    // Loop-Shaping Tuner  (frequency-domain -> LeadLagParams)
    // ============================================================
    // Designs a lead compensator C(s) = K.(s+z)/(s+p) so that:
    //   - The gain crossover occurs at omega_c  (|L(j.omega_c)| = 1)
    //   - The compensator contributes phase_add_deg of phase lead at omega_c
    //
    // Derivation (lead, phase_add_deg > 0):
    //   beta = sin(φ),   alpha = (1+beta)/(1-beta)
    //   z = omega_c/\sqrtalpha,   p = omega_c.\sqrtalpha,   K = \sqrtalpha / |G(j.omega_c)|
    //
    // where |G(j.omega_c)| = gain_at_wc (open-loop plant magnitude before
    // adding the compensator).
    //
    // Ref: Franklin, Powell & Emami-Naeini "Feedback Control of Dynamic
    //      Systems" Section 9.3.
    // ============================================================
    class LoopShapingTuner
    {
    public:
        struct Input
        {
            double omega_c;       // desired gain crossover frequency [rad/s]
            double phase_add_deg; // phase lead to add at omega_c [deg], must be in (0, 90)
            double gain_at_wc;    // open-loop |G(j.omega_c)| without the compensator
        };

        static LeadLagParams tuneImpl(const Input &in);

        template <typename C>
        static LeadLagParams tuneFor(const Input &in)
        {
            static_assert(ControllerTraits<C>::supports_freq_tuning,
                          "\n[LoopShapingTuner::tuneFor<C>]"
                          " C does not support frequency-domain tuning.\n"
                          "  LoopShapingTuner returns LeadLagParams;"
                          " only valid for DiscreteLeadLag.\n"
                          "  --> DiscretePID     :"
                          " ZieglerNicholsTuner or"
                          " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                          "  --> DiscreteLQR     :"
                          " LQRWeightTuner::brysonMethodFor<DiscreteLQR>(xmax, umax)\n"
                          "  --> DiscreteLQG     :"
                          " LQRWeightTuner::brysonMethodFor<DiscreteLQG>()"
                          " + KalmanWeightTuner::fromNoiseFor<DiscreteLQG>()\n"
                          "  --> DiscreteMPC     :"
                          " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                          "  --> DiscreteSMC / ADRC / ESC :"
                          " set parameters directly (no standard auto-tuner)\n");
            return tuneImpl(in);
        }
    };

    // ============================================================
    // Kalman Weight Tuner  (noise covariance selection for DiscreteLQG)
    // ============================================================
    // Provides heuristic starting points for Qf (process noise covariance)
    // and Rf (measurement noise covariance).  Both scale as sigma^2, where sigma is
    // the expected noise standard deviation per channel.
    //
    // Bryson-like rule:
    //   Qf = diag(sigma_process_i^2),   Rf = diag(sigma_meas_j^2)
    //   P0 = Qf   (initial uncertainty approx = process noise level)
    //
    // The separation principle guarantees that the LQR and Kalman gains
    // can be tuned independently; use LQRWeightTuner for the LQR part.
    //
    // Ref: Bryson & Ho "Applied Optimal Control" (1975) Section 14.3.
    // ============================================================
    class KalmanWeightTuner
    {
    public:
        // Per-channel noise: Qf = diag(maxProcessNoise^2), Rf = diag(maxMeasNoise^2).
        static KalmanNoiseParams fromNoise(const Eigen::VectorXd &maxProcessNoise,
                                           const Eigen::VectorXd &maxMeasNoise);

        // Isotropic (scalar) noise: Qf = sigmap^2.I, Rf = sigmam^2.I.
        static KalmanNoiseParams isotropic(int nStates,
                                           int nOutputs,
                                           double sigmaProcess,
                                           double sigmaMeas);

        // Type-checked wrappers.
        template <typename C>
        static KalmanNoiseParams fromNoiseFor(const Eigen::VectorXd &maxProcessNoise,
                                              const Eigen::VectorXd &maxMeasNoise)
        {
            static_assert(ControllerTraits<C>::supports_kalman_tuning,
                          "\n[KalmanWeightTuner::fromNoiseFor<C>]"
                          " C does not contain a Kalman observer.\n"
                          "  Kalman noise parameters (Qf, Rf) only apply to DiscreteLQG.\n"
                          "  --> DiscreteLQR     :"
                          " full state is assumed known; no observer is needed\n"
                          "  --> DiscretePID     :"
                          " ZieglerNicholsTuner or StepResponseTuner\n"
                          "  --> DiscreteMPC     :"
                          " MPCHorizonTuner::recommendFor<DiscreteMPC>()\n"
                          "  --> Standalone KalmanFilter :"
                          " pass Qf/Rf directly to KalmanFilter's constructor\n");
            return fromNoise(maxProcessNoise, maxMeasNoise);
        }

        template <typename C>
        static KalmanNoiseParams isotropicFor(int nStates,
                                              int nOutputs,
                                              double sigmaProcess,
                                              double sigmaMeas)
        {
            static_assert(ControllerTraits<C>::supports_kalman_tuning,
                          "\n[KalmanWeightTuner::isotropicFor<C>]"
                          " C does not contain a Kalman observer.\n"
                          "  Same constraint as KalmanWeightTuner::fromNoiseFor<C>.\n"
                          "  Only DiscreteLQG embeds a Kalman filter with tunable Qf/Rf.\n");
            return isotropic(nStates, nOutputs, sigmaProcess, sigmaMeas);
        }
    };

} // namespace ctrl
