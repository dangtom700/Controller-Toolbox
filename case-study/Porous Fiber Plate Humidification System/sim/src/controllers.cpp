#include "controllers.h"
#include <algorithm>
#include <cmath>

namespace humid {

// Clamp helpers (in header as inline statics, repeated here for clarity)
static double cFan(double v) { return std::clamp(v, 1.0, 3.5); }

// phi error convention: e = ref - measured (positive = too dry -> increase fan speed)

// ---------------------------------------------------------------------------
// 1. PID
// Gains: Kp=0.10, Ki=1.39e-5 (continuous), Kd=0 - conservative PI.
// output = u_fan [m/s], centred at kFanMid = 2.25 m/s.
// uMin/uMax = [1.0, 3.5] m/s.
// ---------------------------------------------------------------------------
PIDHumidCtrl::PIDHumidCtrl(double Ts)
    : pid_(ctrl::PIDParams{
        .Kp   =  0.10,
        .Ki   =  1.39e-5,
        .Kd   =  0.0,
        .N    =  5.0,
        .uMin =  1.0,
        .uMax =  3.5,
        .Kb   =  0.1
      }, Ts)
{}

void PIDHumidCtrl::reset() { pid_.reset(); }

ControlInput PIDHumidCtrl::compute(double phi_measured, double ref_phi)
{
    // phi error in % (controller designed for % units -> multiply by 100)
    double e   = (ref_phi - phi_measured) * 100.0;
    double fan = cFan(pid_.compute(e));
    return {fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 2. PID_AW - identical gains to PID; included to showcase Kb clamping clearly
// Kb=0.3 - tighter back-calculation so windup clears faster on s05 low-demand.
// ---------------------------------------------------------------------------
PID_AWHumidCtrl::PID_AWHumidCtrl(double Ts)
    : pid_(ctrl::PIDParams{
        .Kp   =  0.10,
        .Ki   =  1.39e-5,
        .Kd   =  0.0,
        .N    =  5.0,
        .uMin =  1.0,
        .uMax =  3.5,
        .Kb   =  0.3
      }, Ts)
{}

void PID_AWHumidCtrl::reset() { pid_.reset(); }

ControlInput PID_AWHumidCtrl::compute(double phi_measured, double ref_phi)
{
    double e   = (ref_phi - phi_measured) * 100.0;
    double fan = cFan(pid_.compute(e));
    return {fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 3. Feedforward + PID
// FF path: when phi_out drops (dry outdoor air) -> need more humidification.
//   u_ff = kFanMid + FF_gain * (phi_ref - phi_out_equivalent_at_Troom)
// FF_gain = 3.0 m/s / (0.45 span in phi units)
// PID trims residual error on +/-0.5 m/s authority.
// ---------------------------------------------------------------------------
FFPIDHumidCtrl::FFPIDHumidCtrl(double Ts)
    : pid_(ctrl::PIDParams{
        .Kp   =  0.06,
        .Ki   =  8.0e-6,
        .Kd   =  0.0,
        .N    =  5.0,
        .uMin = -0.5,
        .uMax =  0.5,
        .Kb   =  0.1
      }, Ts)
{}

void FFPIDHumidCtrl::reset() { pid_.reset(); }

ControlInput FFPIDHumidCtrl::compute(double phi_measured, double ref_phi)
{
    // Convert outdoor phi to equivalent at T_room (approx =22^\circC) using Psat ratio.
    // At T_out = -10^\circC: Psat_ratio approx = 259.8/2642 = 0.098
    // phi_out_equiv = phi_out_outdoor * Psat(T_out)/Psat(T_room)
    // We don't have T_out here, so use phi_out_ directly scaled: phi_out_at_room approx = phi_out_ * 0.10
    double phi_out_room = phi_out_ * 0.10;   // rough: Psat(-10^\circC)/Psat(22^\circC) approx = 0.10

    // FF: more fan speed needed when outdoor is dry -> shortfall driving humidity
    double shortfall = ref_phi - phi_out_room;   // how much the room needs vs outdoor
    double u_ff = kFanMid + 2.0 * (shortfall - 0.35);  // centred at nominal shortfall 0.35

    double e    = (ref_phi - phi_measured) * 100.0;
    double u_fb = pid_.compute(e);

    return {cFan(u_ff + u_fb), kTa_nom};
}

// ---------------------------------------------------------------------------
// 4. Cascade PID
// Outer PI: phi_room % error -> H_ref [g/h].
// Inner (static physics inversion): H_ref -> u_fan via H ~ sqrt(u) approximation.
//   u_fan = u_nom * (H_ref / H_nom)^2  (Sh ~ Re^0.5 ~ u^0.5 -> H ~ u^0.5 -> u ~ H^2)
// ---------------------------------------------------------------------------
CascadePIDHumidCtrl::CascadePIDHumidCtrl(double Ts)
    : outer_pi_(ctrl::PIDParams{
        .Kp   = 12.0,      // [g/h] per % error
        .Ki   = 1.67e-3,   // continuous Ki [g/h / (%.s)]
        .Kd   = 0.0,
        .N    = 5.0,
        .uMin = 50.0,      // minimum H demand [g/h]
        .uMax = 320.0,     // maximum H demand [g/h]
        .Kb   = 0.05
      }, Ts)
{}

void CascadePIDHumidCtrl::reset()
{
    outer_pi_.reset();
    u_fan_prev_ = kFanMid;
}

ControlInput CascadePIDHumidCtrl::compute(double phi_measured, double ref_phi)
{
    double e      = (ref_phi - phi_measured) * 100.0;
    double H_ref  = outer_pi_.compute(e);   // [g/h]

    // Physics inversion: H ~ sqrt(u_fan) -> u_fan = u_nom * (H_ref/H_nom)^2
    double ratio  = H_ref / H_nom_;
    double u_fan  = cFan(kFanMid * ratio * ratio);

    u_fan_prev_ = u_fan;
    return {u_fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 5. Gain-scheduled PID
// 3 schedule points indexed by phi_room (use measured phi as scheduling variable).
// NearestNeighbor: one PID active at a time -> no integral state bleed.
// Gains:
//   phi=0.30 (dry room):    Kp=0.14, Ki=1.94e-5  (more aggressive)
//   phi=0.45 (nominal):     Kp=0.10, Ki=1.39e-5
//   phi=0.60 (humid room):  Kp=0.07, Ki=9.72e-6  (less aggressive)
// ---------------------------------------------------------------------------
GainSchedHumidCtrl::GainSchedHumidCtrl(double Ts)
    : sched_(Ts, ctrl::GainScheduleMode::NearestNeighbor)
{
    sched_.addSchedulePoint(30.0,
        std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
            .Kp=0.14, .Ki=1.94e-5, .Kd=0.0, .N=5.0,
            .uMin=1.0, .uMax=3.5, .Kb=0.1
        }, Ts));
    sched_.addSchedulePoint(45.0,
        std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
            .Kp=0.10, .Ki=1.39e-5, .Kd=0.0, .N=5.0,
            .uMin=1.0, .uMax=3.5, .Kb=0.1
        }, Ts));
    sched_.addSchedulePoint(60.0,
        std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
            .Kp=0.07, .Ki=9.72e-6, .Kd=0.0, .N=5.0,
            .uMin=1.0, .uMax=3.5, .Kb=0.1
        }, Ts));
}

void GainSchedHumidCtrl::reset() { sched_.reset(); }

ControlInput GainSchedHumidCtrl::compute(double phi_measured, double ref_phi)
{
    // Schedule on measured phi in % units
    sched_.setSchedulingParam(phi_measured * 100.0);
    double e   = (ref_phi - phi_measured) * 100.0;
    double fan = cFan(sched_.compute(e));
    return {fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 6. Smith Predictor
// FOPDT model: A = 0.99583, B = 0.04917 (phi_room [%] per (m/s)) at nominal.
// Dead time: 2 steps * 30 s = 60 s (matches plant sensor delay).
// Inner PI: Kp=0.12, Ki=1.67e-5 (slightly more aggressive since delay compensated).
// ---------------------------------------------------------------------------
SmithHumidCtrl::SmithHumidCtrl(double Ts)
    : sp_(
        std::make_shared<ctrl::DiscretePID>(ctrl::PIDParams{
            .Kp   = 0.12,
            .Ki   = 1.67e-5,
            .Kd   = 0.0,
            .N    = 5.0,
            .uMin = 1.0,
            .uMax = 3.5,
            .Kb   = 0.1
        }, Ts),
        ctrl::StateSpace(
            (Eigen::Matrix<double,1,1>() << 0.99583).finished(),  // A
            (Eigen::Matrix<double,1,1>() << 0.04917 * 0.01).finished(), // B (phi in frac)
            (Eigen::Matrix<double,1,1>() << 1.0).finished(),       // C
            (Eigen::Matrix<double,1,1>() << 0.0).finished(),       // D
            Ts),
        2  // 2-step dead-time
      )
{}

void SmithHumidCtrl::reset() { sp_.reset(); }

ControlInput SmithHumidCtrl::compute(double phi_measured, double ref_phi)
{
    // Error in fraction (model operates in fraction units, unlike PID which uses %)
    double e   = ref_phi - phi_measured;
    // Scale error to % for the inner PID (Kp designed for % error -> m/s output)
    double fan = cFan(sp_.compute(e * 100.0));
    return {fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 7. ADRC
// First-order LADRC on phi_room tracking.
// b0 = K/tau = 11.8/7200 = 1.639e-3 [%RH/s per (m/s)] for the continuous plant.
// omega_o = 0.015 rad/s -> omega_o * Ts = 0.015 * 30 = 0.45 < 0.50 (stable).
// omega_c = 0.003 rad/s.
// ---------------------------------------------------------------------------
ADRCHumidCtrl::ADRCHumidCtrl(double Ts)
    : adrc_(ctrl::ADRCParams{
        .omega_o = 0.015,
        .omega_c = 0.003,
        .b0      = 1.639e-3,
        .uMin    = 1.0,
        .uMax    = 3.5
      }, Ts)
{}

void ADRCHumidCtrl::reset() { adrc_.reset(); }

ControlInput ADRCHumidCtrl::compute(double phi_measured, double ref_phi)
{
    double e   = (ref_phi - phi_measured) * 100.0;
    double fan = cFan(adrc_.compute(e));
    return {fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 8. MPC
// FOPDT linearised at phi_nom=0.45 (45% RH), Ta=40^\circC, u_fan=2.25 m/s.
// Deviation variables: x = phi_room - phi_nom [%], u = u_fan - u_nom [m/s].
// A = 0.99583, B = 0.04917 [%/(m/s)] -> note these are %RH-to-m/s.
// Np=20 (600 s horizon), Nc=5.
// ---------------------------------------------------------------------------
MPCHumidCtrl::MPCHumidCtrl(double Ts)
    : mpc_(
        ctrl::StateSpace(
            (Eigen::Matrix<double,1,1>() << 0.99583).finished(),
            (Eigen::Matrix<double,1,1>() << 0.04917).finished(),
            (Eigen::Matrix<double,1,1>() << 1.0).finished(),
            (Eigen::Matrix<double,1,1>() << 0.0).finished(),
            Ts),
        ctrl::MPCParams{
            .Np    = 20,
            .Nc    = 5,
            .rho_y = 1.0,
            .rho_u = 10.0,
            .uMin  = -1.25,   // deviation from kFanMid
            .uMax  =  1.25,
            .duMin = -0.1,
            .duMax =  0.1,
        }
      )
{}

void MPCHumidCtrl::reset()
{
    mpc_.reset();
    Eigen::VectorXd x0(1); x0 << 0.0;
    mpc_.setState(x0);
}

ControlInput MPCHumidCtrl::compute(double phi_measured, double ref_phi)
{
    double x_dev = phi_measured * 100.0 - kPhiNom * 100.0;
    double r_dev = ref_phi      * 100.0 - kPhiNom * 100.0;

    Eigen::VectorXd x(1); x << x_dev;
    Eigen::VectorXd r(1); r << r_dev;
    mpc_.setState(x);

    Eigen::VectorXd u = mpc_.computeRef(x, r);
    double fan = cFan(kFanMid + u(0));
    return {fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 9. MRAC
// Reference model: y_m[k+1] = a_m*y_m[k] + b_m*r[k]
//   a_m = exp(-Ts/tau_m), tau_m = 600 s  -> a_m approx = 0.9512, b_m = 0.0488.
//   DC gain = b_m/(1-a_m) = 1 (tracks step references exactly).
// compute() passes plant output (phi_measured * 100 in %) NOT the error.
// Output is u_fan [m/s]; internal MRACController adapts the gain.
// ---------------------------------------------------------------------------
MRACHumidCtrl::MRACHumidCtrl(double Ts)
    : mrac_(ctrl::MRACParams{
        .a_m       = 0.9512,
        .b_m       = 0.0488,
        .gamma_r   = 0.005,
        .gamma_y   = 0.005,
        .sigma     = 0.005,
        .theta_max = 200.0,
        .uMin      = 1.0,
        .uMax      = 3.5
      }, Ts)
{}

void MRACHumidCtrl::reset() { mrac_.reset(); }

ControlInput MRACHumidCtrl::compute(double phi_measured, double ref_phi)
{
    mrac_.setReference(ref_phi * 100.0);
    double fan = cFan(mrac_.compute(phi_measured * 100.0));
    return {fan, kTa_nom};
}

// ---------------------------------------------------------------------------
// 10. GPC + RLS
// GPC: Np=15, Nu=4, alpha=1.0 (uniform output weighting).
// RLS: na=1, nb=1, lambda=0.97 - faster adaptation than Solar study (lambda=0.98)
//      because room conditions change more quickly across scenarios.
// Warmup: 60 steps = 1800 s (0.25 room time constants).
// ---------------------------------------------------------------------------
GPCRLSHumidCtrl::GPCRLSHumidCtrl(double Ts)
    : gpc_([&]() {
        // FOPDT initial model: A=0.99583, B=0.04917
        Eigen::MatrixXd A(1,1); A << 0.99583;
        Eigen::MatrixXd B(1,1); B << 0.04917;
        Eigen::MatrixXd C(1,1); C << 1.0;
        Eigen::MatrixXd D(1,1); D << 0.0;
        ctrl::StateSpace ss(A, B, C, D, Ts);

        ctrl::GPCParams gp;
        gp.Np    = 15;
        gp.Nu    = 4;
        gp.alpha = 1.0;
        gp.uMin  = 1.0;
        gp.uMax  = 3.5;
        return ctrl::GeneralizedPredictiveController(ss, gp);
    }())
    , rls_(1, 1, Ts, 0.97)
{}

void GPCRLSHumidCtrl::reset()
{
    gpc_.reset();
    rls_.reset();
    u_prev_ = kFanMid;
    step_   = 0;
}

ControlInput GPCRLSHumidCtrl::compute(double phi_measured, double ref_phi)
{
    double y_pct  = phi_measured * 100.0;
    double r_pct  = ref_phi      * 100.0;

    // RLS update: y first, u second (RLS API convention)
    rls_.update(y_pct, u_prev_);

    // Periodically update GPC model from RLS estimates
    if (step_ >= kWarmup && (step_ % kRLSInterval) == 0) {
        Eigen::VectorXd th = rls_.params();
        if (th.size() >= 2) {
            double a_hat =  th(0);
            double b_hat =  th(1);
            if (std::abs(b_hat) > 1e-4) {
                Eigen::MatrixXd A(1,1); A << a_hat;
                Eigen::MatrixXd B(1,1); B << b_hat;
                Eigen::MatrixXd C(1,1); C << 1.0;
                Eigen::MatrixXd D(1,1); D << 0.0;
                gpc_.setPlant(ctrl::StateSpace(A, B, C, D, rls_.sampleTime()));
            }
        }
    }

    double e   = r_pct - y_pct;
    double fan = cFan(gpc_.compute(e));

    u_prev_ = fan;
    ++step_;
    return {fan, kTa_nom};
}

} // namespace humid
