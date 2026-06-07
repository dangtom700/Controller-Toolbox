#include "controllers.h"
#include <algorithm>
#include <cmath>

namespace solar {

// Clamp helpers
static constexpr double kMwMin = 0.02;
static constexpr double kMwMax = 0.22;
// Operating-point spray flow for Tw1 approx = 40 ^\circC (Tw1 = T_w2 + Q_cond/(m_dot_w*cp_w))
static constexpr double kMwNom = 0.033;
static constexpr double kKrMin = 0.30;
static constexpr double kKrMax = 1.00;

static double clampMw(double v) { return std::clamp(v, kMwMin, kMwMax); }
static double clampKr(double v) { return std::clamp(v, kKrMin, kKrMax); }

// ---------------------------------------------------------------------------
// 1. PIDController
// Plant has NEGATIVE gain: higher m_dot_w -> lower Tw1 (Tw1 = T_w2 + Q/(m*cp)).
// Formula: m_dot_w = kMwNom - pid.compute(e),  e = ref - measured.
// Positive e (too cold) -> positive u -> m_dot_w < kMwNom -> less flow -> Tw1 rises (check)
// Kp < 1/|K_plant| approx = 1/1000 for linearised stability at the operating point.
// ---------------------------------------------------------------------------
PIDController::PIDController(double Ts)
    : pid_(ctrl::PIDParams{
        .Kp   =  0.0005,
        .Ki   =  0.00005,
        .Kd   =  0.0,
        .N    =  5.0,
        .uMin = -(kMwMax - kMwNom),   // -0.187: maps to m_dot_w = kMwMax
        .uMax =   kMwNom - kMwMin,    // +0.013: maps to m_dot_w = kMwMin
        .Kb   =  0.5
      }, Ts)
{}

void PIDController::reset()
{
    pid_.reset();
}

ControlInput PIDController::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double e       = ref_Tw1 - measured_Tw1;
    double m_dot_w = clampMw(kMwNom - pid_.compute(e));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 2. FFPIDController
// Feedforward: delta_mw = G_to_mw_ * G  anticipates increased thermal load.
// PID trims the remaining error on a +/-0.10 kg/s authority around nominal.
// Pump speed scheduled linearly from the tracking error.
// ---------------------------------------------------------------------------
FFPIDController::FFPIDController(double Ts)
    : pid_(ctrl::PIDParams{
        .Kp   =  0.0005,
        .Ki   =  0.00005,
        .Kd   =  0.0,
        .N    =  5.0,
        .uMin = -0.25,   // wide inner range; outer clampMw provides hard bound
        .uMax =  0.25,
        .Kb   =  0.5
      }, Ts)
{}

void FFPIDController::reset()
{
    pid_.reset();
}

ControlInput FFPIDController::compute(double ref_Tw1, double measured_Tw1, double G)
{
    double e       = ref_Tw1 - measured_Tw1;
    double u_ff    = G_to_mw_ * std::max(G, 0.0);  // bias (integral cancels at steady-state)
    double u_fb    = pid_.compute(e);
    // Subtract feedback (positive e -> reduce flow -> raise Tw1); add feedforward bias.
    double m_dot_w = clampMw(kMwNom + u_ff - u_fb);
    double kr      = clampKr(kr_base_ + kr_e_slope_ * (e / 10.0));
    return {m_dot_w, kr};
}

// ---------------------------------------------------------------------------
// 3. MPCController
// FOPDT model of Tw1 vs m_dot_w linearised around Tw1_nom.
// Continuous plant: dTw1/dt = -a*Tw1 + b*m_dot_w
// Discrete (Euler): Tw1[k+1] = (1-a*Ts)*Tw1[k] + b*Ts*m_dot_w[k]
// DiscreteMPC from the toolbox handles the condensed QP internally.
// ---------------------------------------------------------------------------
MPCController::MPCController(double Ts, double Tw1_nom, double a, double b)
    : mpc_(
        // FOPDT in deviation form around (Tw1_nom, kMwNom).
        // B is NEGATIVE because higher m_dot_w -> lower Tw1 (negative plant gain).
        ctrl::StateSpace(
            (Eigen::Matrix<double,1,1>() << 1.0 - a*Ts).finished(),  // A (1x1)
            (Eigen::Matrix<double,1,1>() << -b*Ts).finished(),        // B negative (check)
            (Eigen::Matrix<double,1,1>() << 1.0).finished(),          // C (1x1)
            (Eigen::Matrix<double,1,1>() << 0.0).finished(),          // D (1x1)
            Ts
        ),
        ctrl::MPCParams{
            .Np    = 8,
            .Nc    = 3,
            .rho_y = 10.0,
            .rho_u = 0.5,             // reduced so tracking dominates effort penalty
            .uMin  = kMwMin - kMwNom, // deviation lower bound = -0.013
            .uMax  = kMwMax - kMwNom, // deviation upper bound = +0.187
            .duMin = -0.05,
            .duMax =  0.05,
        }
      ),
      Tw1_nom_(Tw1_nom)
{}

void MPCController::reset()
{
    mpc_.reset();
    // Seed state estimate at design-point deviation = 0
    Eigen::VectorXd x0(1);
    x0 << 0.0;
    mpc_.setState(x0);
}

ControlInput MPCController::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    // Deviation variables: x=DeltaTw1, u=Deltam_dot_w (around kMwNom).
    double x_dev = measured_Tw1 - Tw1_nom_;
    double r_dev = ref_Tw1      - Tw1_nom_;

    Eigen::VectorXd x(1); x << x_dev;
    Eigen::VectorXd r(1); r << r_dev;

    mpc_.setState(x);
    Eigen::VectorXd u = mpc_.computeRef(x, r);

    // u(0) is Deltam_dot_w; add operating-point to get absolute flow
    double m_dot_w = clampMw(kMwNom + u(0));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 4. ADRCSolarCtrl
// b0 = 4.5 ^\circC/s/(kg/s): continuous FOPDT input gain (dTw1/dt = -a*Tw1 + b*mw).
// omega_o = 0.04 rad/s -> omega_o*Ts = 0.4 < 0.5 (backward Euler stable).
// omega_c = omega_o/5 = 0.008 rad/s -> closed-loop time constant approx = 125 s.
// ESO estimates lumped disturbance from irradiance load shifts automatically.
// ---------------------------------------------------------------------------
ADRCSolarCtrl::ADRCSolarCtrl(double Ts)
    : adrc_(ctrl::ADRCParams{
        .omega_o = 0.04,
        .omega_c = 0.008,
        .b0      = 4.5,
        // Output u is subtracted from kMwNom: m_dot_w = kMwNom - u.
        // Bounds ensure m_dot_w stays in [kMwMin, kMwMax].
        .uMin    = -(kMwMax - kMwNom),  // -0.187
        .uMax    =   kMwNom - kMwMin,   // +0.013
      }, Ts)
{}

void ADRCSolarCtrl::reset() { adrc_.reset(); }

ControlInput ADRCSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    // Subtract ADRC output from nominal: positive e (too cold) -> positive u
    // -> m_dot_w = kMwNom - u < kMwNom -> less flow -> Tw1 rises (check)
    double e       = ref_Tw1 - measured_Tw1;
    double m_dot_w = clampMw(kMwNom - adrc_.compute(e));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 5. FuzzyPIDSolarCtrl
// 25-rule Mamdani FuzzyPD + crisp integral accumulator.
// e_scale  = 10 ^\circC  (normalises +/-10 ^\circC error to +/-1).
// de_scale =  1 ^\circC/step (normalises error increment per sample step).
// u_scale  =  0.10 kg/s (maps +/-1 normalised output to +/-0.10 kg/s).
// FuzzyPID::compute(e) computes the derivative internally from consecutive errors.
// ---------------------------------------------------------------------------
FuzzyPIDSolarCtrl::FuzzyPIDSolarCtrl(double Ts)
    : fuzzy_(ctrl::FuzzyPIDParams{
        .pd  = {.e_scale  = 10.0,
                .de_scale =  1.0,
                .u_scale  =  0.10,
                .uMin     = -(kMwMax - kMwMin) / 2.0,
                .uMax     =  (kMwMax - kMwMin) / 2.0},
        .Ki  = 0.0003,
        .Kb  = 0.5,
        .uMin = kMwMin,
        .uMax = kMwMax
      }, Ts)
{}

void FuzzyPIDSolarCtrl::reset() { fuzzy_.reset(); }

ControlInput FuzzyPIDSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double e       = ref_Tw1 - measured_Tw1;
    double m_dot_w = clampMw(fuzzy_.compute(e));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 6. SmithPredictorSolarCtrl
// FOPDT delay-free model: Tw1[k+1] = (1-a*Ts)*Tw1[k] + b*Ts*m_dot_w[k]
// Dead-time: 2 steps * Ts = 20 s estimated thermal measurement lag.
// Inner PID is slightly more aggressive than standalone (Kp=0.003) because
// the predictor compensates for the effective delay before the PID sees it.
// ---------------------------------------------------------------------------
SmithPredictorSolarCtrl::SmithPredictorSolarCtrl(double Ts)
    : sp_(
        std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
            .Kp   =  0.003,
            .Ki   =  0.0003,
            .Kd   =  0.0,
            .N    =  5.0,
            .uMin =  kMwMin,
            .uMax =  kMwMax,
            .Kb   =  0.5
        }, Ts),
        ctrl::StateSpace(
            (Eigen::Matrix<double,1,1>() << 1.0 - 0.018*Ts).finished(),
            (Eigen::Matrix<double,1,1>() << 4.5 * Ts).finished(),
            (Eigen::Matrix<double,1,1>() << 1.0).finished(),
            (Eigen::Matrix<double,1,1>() << 0.0).finished(),
            Ts),
        2   // 2-step dead-time
      )
{}

void SmithPredictorSolarCtrl::reset() { sp_.reset(); }

ControlInput SmithPredictorSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double m_dot_w = clampMw(sp_.compute(ref_Tw1 - measured_Tw1));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 7. MRACSolarCtrl
// Reference model: y_m[k+1] = 0.70*y_m[k] + 0.30*r[k]  (DC gain = 1).
// Adaptation: theta_r, theta_y updated via gradient descent + sigma-modification.
// compute() takes PLANT OUTPUT Tw1, not error (MRACController API convention).
// Plant is minimum-phase (no zeros for first-order FOPDT) -> MRAC conditions met.
// ---------------------------------------------------------------------------
MRACSolarCtrl::MRACSolarCtrl(double Ts)
    : mrac_(ctrl::MRACParams{
        .a_m       = 0.70,
        .b_m       = 0.30,
        .gamma_r   = 0.5,
        .gamma_y   = 0.5,
        .sigma     = 0.02,
        .theta_max = 50.0,
        .uMin      = kMwMin,
        .uMax      = kMwMax
      }, Ts)
{}

void MRACSolarCtrl::reset() { mrac_.reset(); }

ControlInput MRACSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    mrac_.setReference(ref_Tw1);
    double m_dot_w = clampMw(mrac_.compute(measured_Tw1));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 8. ESCSolarCtrl
// ExtremumSeeker maximises EER_grid by perturbing m_dot_w.
// An inner PID keeps Tw1 near ref_Tw1; ESC adds a perturbation on top.
// EER feedback arrives via setLastEER() called by the runner each step.
// ---------------------------------------------------------------------------

ESCSolarCtrl::ESCSolarCtrl(double Ts)
    : esc_(ctrl::ExtremumSeekerParams{
        .perturbAmp  = 0.005,     // 5 g/s perturbation on m_dot_w
        .perturbFreq = 0.008,     // Hz (much slower than thermal dynamics ~18 mHz)
        .lpfCutoff   = 0.002,
        .hpfCutoff   = 0.005,
        .integGain   = 0.5,
        .seekMinimum = true       // feed -EER_grid as cost to minimise
      }, Ts)
    , pid_Tw1_(ctrl::PIDParams{
        .Kp   =  0.002,
        .Ki   =  0.0003,
        .Kd   =  0.0,
        .N    =  5.0,
        .uMin = -0.05,
        .uMax =  0.05,
        .Kb   =  0.5
      }, Ts)
{}

void ESCSolarCtrl::reset()
{
    esc_.reset();
    pid_Tw1_.reset();
    last_eer_ = 3.0;
}

ControlInput ESCSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    // Inner PID provides baseline m_dot_w tracking; ESC adds perturbation
    double e_Tw1    = ref_Tw1 - measured_Tw1;
    double mw_pid   = 0.12 + pid_Tw1_.compute(e_Tw1);   // PID around nominal

    // ESC seeker: feed -EER_grid (seeking minimum of -EER = maximum EER)
    double mw_esc = esc_.compute(-last_eer_);   // returns the ESC-adjusted flow
    // Blend: PID dominates tracking, ESC adds efficiency perturbation
    double mw = clampMw(mw_pid + 0.3 * (mw_esc - 0.12));  // 30% ESC influence

    return {mw, kr_nom_};
}

// ---------------------------------------------------------------------------
// 9. GPCRLSSolarCtrl
// GPC with online RLS model identification on FOPDT structure.
// RLS identifies 1st-order ARX: y[k] = -a1*y[k-1] + b1*u[k-1]
// GPC uses this identified model for prediction.
// ---------------------------------------------------------------------------

GPCRLSSolarCtrl::GPCRLSSolarCtrl(double Ts, double Tw1_nom)
    : gpc_([&]() {
        // FOPDT: a=0.018 /s, b=4.5 [^\circC/(kg/s)/s], Ts=Ts
        // Discrete: A(z) = 1 - (1-a*Ts)*z^-1, B(z) = b*Ts*z^-1
        double a_d = 1.0 - 0.018 * Ts;
        double b_d = 4.5 * Ts;

        Eigen::MatrixXd Ac(1,1); Ac << a_d;
        Eigen::MatrixXd Bc(1,1); Bc << b_d;
        Eigen::MatrixXd Cc(1,1); Cc << 1.0;
        Eigen::MatrixXd Dc(1,1); Dc << 0.0;
        ctrl::StateSpace ss_fopdt(Ac, Bc, Cc, Dc, Ts);

        ctrl::GPCParams gp;
        gp.Np    = 20;
        gp.Nu    = 5;
        gp.alpha = 0.5;
        gp.uMin  = kMwMin;
        gp.uMax  = kMwMax;
        return ctrl::GeneralizedPredictiveController(ss_fopdt, gp);
    }())
    , rls_(1, 1, Ts, 0.98)   // na=1, nb=1, lambda=0.98
    , Tw1_nom_(Tw1_nom)
{}

void GPCRLSSolarCtrl::reset()
{
    gpc_.reset();
    rls_.reset();
    u_prev_ = 0.12;
    step_   = 0;
}

ControlInput GPCRLSSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double e = ref_Tw1 - measured_Tw1;

    // Update RLS with current y and previous u (deviation from nominal)
    double y_dev = measured_Tw1 - Tw1_nom_;
    double u_dev = u_prev_ - 0.12;    // deviation from nominal flow
    rls_.update(y_dev, u_dev);   // y first, u second (RLS API)

    // Periodically update GPC plant model from RLS estimates
    if (step_ >= kWarmup && (step_ % kRLSInterval) == 0) {
        Eigen::VectorXd th = rls_.params();   // theta = [a1, b1]
        if (th.size() >= 2) {
            double a_hat = -th(0);   // th = [-a, b]
            double b_hat =  th(1);
            if (std::abs(b_hat) > 1e-6) {
                Eigen::MatrixXd A(1,1); A << a_hat;
                Eigen::MatrixXd B(1,1); B << b_hat;
                Eigen::MatrixXd C(1,1); C << 1.0;
                Eigen::MatrixXd D(1,1); D << 0.0;
                gpc_.setPlant(ctrl::StateSpace(A, B, C, D, rls_.sampleTime()));
            }
        }
    }

    // GPC compute
    double u_opt_dev = gpc_.compute(e);
    double mw = clampMw(0.12 + u_opt_dev);

    u_prev_ = mw;
    ++step_;
    return {mw, kr_nom_};
}

// ---------------------------------------------------------------------------
// 10. ILCSolarCtrl
// ---------------------------------------------------------------------------
ILCSolarCtrl::ILCSolarCtrl(double Ts)
    : ilc_([&]() {
        ctrl::ILC::Params ip;
        ip.N        = N_TRIAL;
        ip.mode     = ctrl::ILC::Mode::PType;
        ip.Lp       = 0.5;
        ip.Ts       = Ts;
        ip.Q_filter = 0.95;
        ip.uMin     = -(kMwMax - kMwNom);
        ip.uMax     =   kMwNom - kMwMin;
        return ctrl::ILC(ip);
      }())
    , pid_(ctrl::PIDParams{
        .Kp   =  0.0005,
        .Ki   =  0.00005,
        .Kd   =  0.0,
        .N    =  5.0,
        .uMin = -(kMwMax - kMwNom),
        .uMax =   kMwNom - kMwMin,
        .Kb   =  0.5
      }, Ts)
{}

void ILCSolarCtrl::reset()
{
    ilc_.reset();
    pid_.reset();
    k_      = 0;
    phase2_ = false;
}

ControlInput ILCSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double e     = ref_Tw1 - measured_Tw1;
    double u_pid = pid_.compute(e);

    if (!phase2_) {
        if (k_ < N_TRIAL)
            ilc_.recordError(k_, e);
        if (++k_ == N_TRIAL) {
            ilc_.updateFeedforward();
            phase2_ = true;
            k_      = 0;
        }
        return {clampMw(kMwNom - u_pid), kr_nom_};
    } else {
        int    kc   = std::min(k_++, N_TRIAL - 1);
        double u_ff = ilc_.feedforward(kc);
        return {clampMw(kMwNom - (u_pid + u_ff)), kr_nom_};
    }
}

// ---------------------------------------------------------------------------
// 11. NeuralPIDSolarCtrl
// ---------------------------------------------------------------------------
NeuralPIDSolarCtrl::NeuralPIDSolarCtrl(double Ts)
    : npid_(ctrl::NeuralPID::Params{
        .n_hidden   = 8,
        .lr         = 1e-7,
        .Ts         = Ts,
        .plant_gain = 4.5 * Ts,
        .uMin       = -(kMwMax - kMwNom),
        .uMax       =   kMwNom - kMwMin,
        .Kp0        = 0.0005,
        .Ki0        = 0.00005,
        .Kd0        = 0.0
      })
{}

void NeuralPIDSolarCtrl::reset() { npid_.reset(); }

ControlInput NeuralPIDSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double e       = ref_Tw1 - measured_Tw1;
    double m_dot_w = clampMw(kMwNom - npid_.compute(e));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 12. L1AdaptiveSolarCtrl
// ---------------------------------------------------------------------------
L1AdaptiveSolarCtrl::L1AdaptiveSolarCtrl(double Ts)
    : l1_(ctrl::L1AdaptiveController::Params{
        .a_m       = 0.70,
        .b_m       = 0.30,
        .k_g       = 1.0,
        .Gamma     = 10.0,
        .omega_c   = 0.02,
        .sigma_max = 20.0,
        .Q_lyap    = 1.0,
        .uMin      = kMwMin,
        .uMax      = kMwMax
      }, Ts)
{}

void L1AdaptiveSolarCtrl::reset() { l1_.reset(); }

ControlInput L1AdaptiveSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    l1_.setReference(ref_Tw1);
    double m_dot_w = clampMw(l1_.compute(measured_Tw1));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 13. DynaSolarCtrl
// ---------------------------------------------------------------------------
DynaSolarCtrl::DynaSolarCtrl(double Ts)
    : dyna_(ctrl::DynaController::Params{
        .n_collect     = 30,
        .n_refit_every = 15,
        .Ts            = Ts
      },
      std::make_shared<ctrl::DiscretePID>(
          ctrl::PIDParams{
              .Kp   =  0.0005,
              .Ki   =  0.00005,
              .Kd   =  0.0,
              .N    =  5.0,
              .uMin = -(kMwMax - kMwNom),
              .uMax =   kMwNom - kMwMin,
              .Kb   =  0.5
          }, Ts))
{}

void DynaSolarCtrl::reset() { dyna_.reset(); }

ControlInput DynaSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double e       = ref_Tw1 - measured_Tw1;
    double m_dot_w = clampMw(kMwNom - dyna_.compute(e));
    return {m_dot_w, kr_nom_};
}

// ---------------------------------------------------------------------------
// 14. CEMSolarCtrl
// FOPDT deviation model; B is negative (more spray -> lower Tw1).
// ---------------------------------------------------------------------------
static ctrl::CEMController makeSolarCEM(double Ts)
{
    const double A = 1.0 - 0.018 * Ts;
    const double B = -4.5 * Ts;

    ctrl::StateFunc f = [A, B](const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd xn(1);
        xn(0) = A * x(0) + B * u(0);
        return xn;
    };
    Eigen::MatrixXd C(1, 1); C << 1.0;

    ctrl::CEMController::Params cp;
    cp.Np        = 10;
    cp.N_samples = 50;
    cp.n_iter    = 3;
    cp.Q         = 10.0;
    cp.R         = 0.5;
    cp.uMin      = kMwMin - kMwNom;
    cp.uMax      = kMwMax - kMwNom;
    return ctrl::CEMController(cp, f, C, Ts);
}

CEMSolarCtrl::CEMSolarCtrl(double Ts, double Tw1_nom)
    : cem_(makeSolarCEM(Ts))
    , Tw1_nom_(Tw1_nom)
{}

void CEMSolarCtrl::reset() { cem_.reset(); }

ControlInput CEMSolarCtrl::compute(double ref_Tw1, double measured_Tw1, double /*G*/)
{
    double x_dev = measured_Tw1 - Tw1_nom_;
    double r_dev = ref_Tw1      - Tw1_nom_;

    Eigen::VectorXd x(1); x << x_dev;
    Eigen::VectorXd r(1); r << r_dev;

    cem_.setState(x);
    Eigen::VectorXd u = cem_.computeRef(x, r);

    double m_dot_w = clampMw(kMwNom + u(0));
    return {m_dot_w, kr_nom_};
}

} // namespace solar
