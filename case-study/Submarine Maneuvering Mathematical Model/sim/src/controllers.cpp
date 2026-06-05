#include "controllers.h"
#include <algorithm>
#include <cmath>

namespace sub {

// ---------------------------------------------------------------------------
// Shared plant state-space construction for MPC / LQR / Smith
// Continuous 3-state heading model: x = [v, r, ψ], u = [deltar], y = [ψ]
// Linearised around straight-ahead motion at design speed U.
// ---------------------------------------------------------------------------
static ctrl::StateSpace buildHeadingSS(const PlantParams& p, double Ts)
{
    const double A22 = p.A22(), A66 = p.A66();
    const double mU  = p.m * p.U;

    // Continuous A, B, C, D
    Eigen::MatrixXd Ac(3, 3);
    Ac << p.Yv / A22,  (p.Yr - mU) / A22,  0.0,
          p.Nv / A66,   p.Nr / A66,          0.0,
          0.0,           1.0,                 0.0;

    Eigen::MatrixXd Bc(3, 1);
    Bc << p.Ydr / A22,
          p.Ndr / A66,
          0.0;

    Eigen::MatrixXd Cc(1, 3);
    Cc << 0.0, 0.0, 1.0;

    Eigen::MatrixXd Dc(1, 1);
    Dc << 0.0;

    ctrl::StateSpace sys_c(Ac, Bc, Cc, Dc, 0.0);
    return ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);
}

// ---------------------------------------------------------------------------
// ControllerBase - shared depth two-loop control
// Outer P: theta_ref = Kz * (z_ref - z), clamped to +/-10^\circ = +/-0.175 rad
// Inner PID on theta_error = theta_ref - theta -> deltas (stern plane)
// ---------------------------------------------------------------------------
ctrl::PIDParams ControllerBase::buildDepthPID()
{
    ctrl::PIDParams p;
    p.Kp   = 0.4;
    p.Ki   = 0.05;
    p.Kd   = 0.5;
    p.N    = 5.0;
    p.Kb   = 1.0;
    p.uMin = -0.611;
    p.uMax =  0.611;
    return p;
}

double ControllerBase::computeDepth(double z_ref, double z,
                                    double theta, double /*Ts*/)
{
    constexpr double Kz    = 0.05;
    const double tht_ref  = std::clamp(Kz * (z_ref - z), -0.175, 0.175);
    return depth_pid_.compute(tht_ref - theta);
}

void ControllerBase::resetDepth()
{
    depth_pid_.reset();
}

// ---------------------------------------------------------------------------
// Shared PID factory
// ---------------------------------------------------------------------------
static ctrl::PIDParams makePID(double Kp, double Ki, double Kd, double N = 5.0)
{
    ctrl::PIDParams p;
    p.Kp   = Kp;
    p.Ki   = Ki;
    p.Kd   = Kd;
    p.N    = N;
    p.Kb   = 1.0;
    p.uMin = -0.611;
    p.uMax =  0.611;
    return p;
}

// ===========================================================================
// 1. PID_ZN - Ziegler-Nichols style (aggressive)
// ===========================================================================
PIDZNCtrl::PIDZNCtrl(double Ts) : pid_(makePID(3.0, 0.3, 8.0), Ts) {}

double PIDZNCtrl::computeHeading(double psi_ref, double psi, double, double)
{
    return pid_.compute(wrapAngle(psi_ref - psi));
}

void PIDZNCtrl::reset() { pid_.reset(); resetDepth(); }

// ===========================================================================
// 2. PID_Cons - Conservative PID
// ===========================================================================
PIDConsCtrl::PIDConsCtrl(double Ts) : pid_(makePID(1.5, 0.1, 5.0), Ts) {}

double PIDConsCtrl::computeHeading(double psi_ref, double psi, double, double)
{
    return pid_.compute(wrapAngle(psi_ref - psi));
}

void PIDConsCtrl::reset() { pid_.reset(); resetDepth(); }

// ===========================================================================
// 3. MPC - Discrete MPC on linearised 3-state heading model
// ===========================================================================
static ctrl::MPCParams makeMPCParams()
{
    ctrl::MPCParams mp;
    mp.Np    = 20;
    mp.Nc    = 5;
    mp.rho_y = 10.0;
    mp.rho_u = 0.01;
    mp.uMin  = -0.611;
    mp.uMax  =  0.611;
    mp.duMin = -0.10;
    mp.duMax =  0.10;
    return mp;
}

MPCHeadingCtrl::MPCHeadingCtrl(const PlantParams& p, double Ts)
    : mpc_(buildHeadingSS(p, Ts), makeMPCParams())
    , x_hat_(Eigen::VectorXd::Zero(3))
{}

double MPCHeadingCtrl::computeHeading(double psi_ref, double psi,
                                       double v, double r)
{
    x_hat_ << v, r, psi;
    double u = mpc_.compute(wrapAngle(psi_ref - psi));
    u_prev_ = u;
    return u;
}

void MPCHeadingCtrl::reset()
{
    mpc_.reset();
    u_prev_ = 0.0;
    x_hat_.setZero();
    resetDepth();
}

// ===========================================================================
// 4. LQR - Full-state LQR on [v, r, ψ]
// ===========================================================================
static ctrl::LQRParams makeLQRParams()
{
    ctrl::LQRParams lp;
    lp.Q = Eigen::MatrixXd::Zero(3, 3);
    lp.Q(0, 0) = 0.1;    // v
    lp.Q(1, 1) = 0.01;   // r
    lp.Q(2, 2) = 100.0;  // ψ
    lp.R = Eigen::MatrixXd::Identity(1, 1) * 0.001;
    return lp;
}

LQRHeadingCtrl::LQRHeadingCtrl(const PlantParams& p, double Ts)
    : lqr_(buildHeadingSS(p, Ts), makeLQRParams())
{}

double LQRHeadingCtrl::computeHeading(double psi_ref, double psi,
                                       double v, double r)
{
    Eigen::VectorXd x(3), x_ref(3);
    x     << v, r, psi;
    x_ref << 0.0, 0.0, psi_ref;
    auto u = lqr_.compute(x, x_ref);
    return std::clamp(u(0), -0.611, 0.611);
}

// ===========================================================================
// 5. ADRC - 2nd-order ADRC
//    omega_o*Ts = 2.0*0.05 = 0.10 < 0.5 (check)  (CLAUDE.md stability condition)
// ===========================================================================
ADRCHeadingCtrl::ADRCHeadingCtrl(double Ts)
    : adrc_([]() {
        ctrl::ADRCParams ap;
        ap.b0      = 0.654;   // Ndr/A66 = 1014/1550
        ap.omega_c = 0.5;
        ap.omega_o = 2.0;     // omega_o*Ts = 0.10 < 0.5 (check)
        ap.uMin    = -0.611;
        ap.uMax    =  0.611;
        return ap;
    }(), Ts)
{}

double ADRCHeadingCtrl::computeHeading(double psi_ref, double psi, double, double)
{
    return adrc_.compute(wrapAngle(psi_ref - psi));
}

void ADRCHeadingCtrl::reset() { adrc_.reset(); resetDepth(); }

// ===========================================================================
// 6. SMC - DiscreteSMC
//    Per CLAUDE.md: DiscreteSMC::compute(y - ref)
//    SMCParams: c_e, c_de (= lambda*Ts), K, phi
// ===========================================================================
SMCHeadingCtrl::SMCHeadingCtrl(double Ts)
    : smc_([]() {
        ctrl::SMCParams sp;
        sp.c_e  = 1.0;
        sp.c_de = 0.3 * 0.05;  // lambda * Ts
        sp.K    = 1.5;
        sp.phi  = 0.1;
        sp.uMin = -0.611;
        sp.uMax =  0.611;
        return sp;
    }(), Ts)
{}

double SMCHeadingCtrl::computeHeading(double psi_ref, double psi, double, double)
{
    return smc_.compute(wrapAngle(psi - psi_ref));   // y - ref per CLAUDE.md
}

void SMCHeadingCtrl::reset() { smc_.reset(); resetDepth(); }

// ===========================================================================
// 7. MRAC
//    MRACController::compute(y_plant); setReference(r) before each call.
//    MRACParams: a_m, b_m, gamma_r, gamma_y, sigma, theta_max, uMin, uMax
// ===========================================================================
MRACHeadingCtrl::MRACHeadingCtrl(double Ts)
    : mrac_([]() {
        ctrl::MRACParams mp;
        mp.a_m      = 0.70;   // stable reference model pole  (|a_m| < 1)
        mp.b_m      = 0.30;   // = 1 - a_m for unity DC gain
        mp.gamma_r  = 0.002;  // conservative adaptation rate
        mp.gamma_y  = 0.002;
        mp.sigma    = 0.01;
        mp.theta_max= 5.0;
        mp.uMin     = -0.611;
        mp.uMax     =  0.611;
        return mp;
    }(), Ts)
{}

double MRACHeadingCtrl::computeHeading(double psi_ref, double psi, double, double)
{
    mrac_.setReference(psi_ref);
    return mrac_.compute(psi);
}

void MRACHeadingCtrl::reset() { mrac_.reset(); resetDepth(); }

// ===========================================================================
// 8. L1Adaptive
//    L1AdaptiveController::compute(y_plant); setReference(r) before each call.
// ===========================================================================
L1HeadingCtrl::L1HeadingCtrl(double Ts)
    : l1_([]() {
        ctrl::L1AdaptiveController::Params lp;
        lp.a_m      = 0.985;  // exp(-0.3*Ts) approx = slow stable reference model
        lp.b_m      = 0.015;  // = 1 - a_m
        lp.k_g      = 1.0;
        lp.Gamma    = 50.0;
        lp.omega_c  = 0.3;
        lp.sigma_max= 5.0;
        lp.Q_lyap   = 1.0;
        lp.uMin     = -0.611;
        lp.uMax     =  0.611;
        return lp;
    }(), Ts)
{}

double L1HeadingCtrl::computeHeading(double psi_ref, double psi, double, double)
{
    l1_.setReference(psi_ref);
    return l1_.compute(psi);
}

void L1HeadingCtrl::reset() { l1_.reset(); resetDepth(); }

// ===========================================================================
// 9. GainSched - 3 PIDs scheduled on |ψ_error| (NearestNeighbor)
// ===========================================================================
GainSchedHeadingCtrl::GainSchedHeadingCtrl(double Ts)
    : sched_(Ts, ctrl::GainScheduleMode::NearestNeighbor)
{
    // Operating points by absolute error magnitude
    sched_.addSchedulePoint(0.000,
        std::make_shared<ctrl::DiscretePID>(makePID(1.5, 0.1, 5.0), Ts));  // <=15^\circ
    sched_.addSchedulePoint(0.262,
        std::make_shared<ctrl::DiscretePID>(makePID(2.5, 0.2, 7.0), Ts));  // <=30^\circ
    sched_.addSchedulePoint(0.524,
        std::make_shared<ctrl::DiscretePID>(makePID(3.5, 0.3, 9.0), Ts));  // >30^\circ
}

double GainSchedHeadingCtrl::computeHeading(double psi_ref, double psi,
                                              double, double)
{
    const double err = wrapAngle(psi_ref - psi);
    sched_.setSchedulingParam(std::abs(err));
    return sched_.compute(err);
}

void GainSchedHeadingCtrl::reset() { sched_.reset(); resetDepth(); }

// ===========================================================================
// 10. Smith Predictor - compensates 1-step heading sensor delay
//     Constructor: SmithPredictor(shared_ptr<IController>, StateSpace, delaySteps)
// ===========================================================================
SmithHeadingCtrl::SmithHeadingCtrl(double Ts)
    : smith_(
        std::make_shared<ctrl::DiscretePID>(makePID(2.5, 0.25, 7.0), Ts),
        buildHeadingSS(PlantParams{}, Ts),  // delay-free heading model
        1)                                   // 1-step sensor delay
{}

double SmithHeadingCtrl::computeHeading(double psi_ref, double psi, double, double)
{
    return smith_.compute(wrapAngle(psi_ref - psi));
}

void SmithHeadingCtrl::reset() { smith_.reset(); resetDepth(); }

} // namespace sub
