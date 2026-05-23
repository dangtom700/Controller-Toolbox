/*
 * siso_coupled.cpp - Coupled SISO Known-Plant: Full Suite of Methods
 * ====================================================================
 * Case 2: Strongly coupled second-order SISO plant
 *
 * Plant: G(s) = 1 / (s^2 + 1.5s + 1)
 *   State-space (controllable canonical form, ZOH Ts=0.01 s):
 *     A = [ 1.98511  -0.98522 ]    B = [ 4.9625e-5 ]
 *         [ 1.0       0.0    ]        [ 4.9125e-5 ]
 *     C = [ 1.0  0.0 ]               D = [ 0 ]
 *   Eigenvalues: -0.75+/-0.9614j  (continuous)  ->  lightly underdamped
 *   DC gain: 1.0
 *
 * "Coupled" refers to the strong internal-state coupling: the off-diagonal
 * elements of A couple position-like and velocity-like states, causing
 * every control action to excite both modes simultaneously.
 *
 * All controllers evaluated on unit-step reference, u \in [-10, 10]:
 *   PID:  ZN (relay), IMC (lambda=0.3,0.5,1.0), Cohen-Coon, NM-optimised
 *   LQR:  Bryson, NM-optimised
 *   SMC:  default params, NM-optimised
 *   ADRC: Gao bandwidth params, NM-optimised
 *   LeadLag: loop-shaping (omegac=2, φ=50^\circ)
 *   Smith Predictor: IMC inner PID + 3-step delay model
 *
 * Cost: J = ISE + 0.1.ITAE + 0.01.\intu^2dt
 * Pareto analysis: ranks all methods by J, fuzzy score for each.
 *
 * Expected output:
 *   - System equations and eigenvalues
 *   - Per-method table: ISE, ITAE, OS[%], T_settle, \intu^2, J
 *   - Pareto-optimal highlighted
 *   - Fuzzy score for each method
 *   - Results -> siso_coupled_results.csv
 *
 * Build: see examples/CMakeLists.txt
 */

#include "fuzzy_performance.h"
#include "../../lib/ControllerToolbox.h"

#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <memory>

using namespace ctrl;

// =========================================================================
// Plant constants (G(s)=1/(s^2+1.5s+1), Ts=0.01 s, ZOH)
// =========================================================================
static constexpr double Ts = 0.01;
static constexpr double UMAX = 10.0;
static constexpr int SIM_STEPS = 2000;

// Discrete TF coefficients (ZOH Ts=0.01)
static const std::vector<double> NUM = {0.0, 4.9625e-5, 4.9125e-5};
static const std::vector<double> DEN = {1.0, -1.98511, 0.98522};

inline StateSpace make_plant()
{
    return tf2ss(TransferFunction(NUM, DEN, Ts));
}

// =========================================================================
// One-step helper: scalar input -> scalar output, state updated in-place
// =========================================================================
inline double plant_step(const StateSpace &sys, Eigen::VectorXd &x, double u)
{
    Eigen::VectorXd uv(1);
    uv << u;
    return ssStep(sys, x, uv)(0);
}

// =========================================================================
// Performance metrics for one SISO run
// =========================================================================
struct SISOMetrics
{
    double ISE = 0.0;
    double ITAE = 0.0;
    double energy = 0.0;
    double overshoot_pct = 0.0;
    double settle_time = 0.0; // [s] last crossing of 2% band
    double J = 0.0;           // composite cost
};

static SISOMetrics compute_metrics(const std::vector<double> &y,
                                   const std::vector<double> &u,
                                   double ref)
{
    int N = (int)y.size();
    SISOMetrics m;
    double y_max = *std::max_element(y.begin(), y.end());
    m.overshoot_pct = std::max(0.0, (y_max - ref) / std::abs(ref) * 100.0);
    for (int k = 0; k < N; ++k)
    {
        double t = k * Ts;
        double e = ref - y[k];
        m.ISE += e * e * Ts;
        m.ITAE += t * std::abs(e) * Ts;
        m.energy += u[k] * u[k] * Ts;
    }
    m.settle_time = SIM_STEPS * Ts;
    for (int k = N - 1; k >= 0; --k)
    {
        if (std::abs(y[k] - ref) > 0.02 * std::abs(ref))
        {
            m.settle_time = (k + 1) * Ts;
            break;
        }
    }
    m.J = m.ISE + 0.1 * m.ITAE + 0.01 * m.energy;
    return m;
}

static void print_metrics(const std::string &name, const SISOMetrics &m)
{
    std::cout << std::setw(22) << name
              << std::fixed << std::setprecision(5)
              << " | ISE=" << std::setw(9) << m.ISE
              << " ITAE=" << std::setw(9) << m.ITAE
              << " E_u=" << std::setw(9) << m.energy
              << std::setprecision(1)
              << " OS=" << std::setw(6) << m.overshoot_pct << "%"
              << std::setprecision(3)
              << " Ts=" << std::setw(6) << m.settle_time << "s"
              << std::setprecision(5)
              << " J=" << std::setw(9) << m.J << "\n";
}

// =========================================================================
// Nelder-Mead (alpha=1, γ=2, ρ=0.5, sigma=0.5 - matches lib/TunerSuite.cpp)
// =========================================================================
static std::vector<double>
nelder_mead(std::function<double(const std::vector<double> &)> f,
            std::vector<double> x0,
            const std::vector<std::pair<double, double>> &bounds,
            int maxEvals = 400, double tol = 1e-6)
{
    const double alpha = 1.0, gamma = 2.0, rho = 0.5, sigma = 0.5;
    int n = (int)x0.size();
    auto clip = [&](std::vector<double> v)
    {
        for (int i = 0; i < n; ++i)
            v[i] = std::clamp(v[i], bounds[i].first, bounds[i].second);
        return v;
    };
    std::vector<std::vector<double>> simp = {clip(x0)};
    for (int i = 0; i < n; ++i)
    {
        auto v = x0;
        v[i] *= 1.05;
        simp.push_back(clip(v));
    }
    std::vector<double> c;
    for (auto &s : simp)
        c.push_back(f(s));
    int ev = n + 1;
    while (ev < maxEvals)
    {
        std::vector<int> idx(n + 1);
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&](int a, int b)
                  { return c[a] < c[b]; });
        std::vector<std::vector<double>> s2;
        std::vector<double> c2;
        for (int i : idx)
        {
            s2.push_back(simp[i]);
            c2.push_back(c[i]);
        }
        simp = s2;
        c = c2;
        double diam = 0;
        for (int i = 1; i <= n; ++i)
            for (int j = 0; j < n; ++j)
                diam = std::max(diam, std::abs(simp[i][j] - simp[0][j]));
        if (diam < tol)
            break;
        std::vector<double> cen(n, 0.0);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                cen[j] += simp[i][j] / n;
        std::vector<double> xr(n);
        for (int j = 0; j < n; ++j)
            xr[j] = cen[j] + alpha * (cen[j] - simp[n][j]);
        xr = clip(xr);
        double fr = f(xr);
        ++ev;
        if (fr < c[0])
        {
            std::vector<double> xe(n);
            for (int j = 0; j < n; ++j)
                xe[j] = cen[j] + gamma * (xr[j] - cen[j]);
            xe = clip(xe);
            double fe = f(xe);
            ++ev;
            simp[n] = (fe < fr) ? xe : xr;
            c[n] = (fe < fr) ? fe : fr;
        }
        else if (fr < c[n - 1])
        {
            simp[n] = xr;
            c[n] = fr;
        }
        else
        {
            std::vector<double> xc(n);
            for (int j = 0; j < n; ++j)
                xc[j] = cen[j] + rho * (simp[n][j] - cen[j]);
            xc = clip(xc);
            double fc = f(xc);
            ++ev;
            if (fc < c[n])
            {
                simp[n] = xc;
                c[n] = fc;
            }
            else
            {
                for (int i = 1; i <= n; ++i)
                {
                    for (int j = 0; j < n; ++j)
                        simp[i][j] = simp[0][j] + sigma * (simp[i][j] - simp[0][j]);
                    simp[i] = clip(simp[i]);
                    c[i] = f(simp[i]);
                    ++ev;
                }
            }
        }
    }
    return simp[0];
}

// =========================================================================
// Generic simulation runner: ctrl_fn(ref, y_prev, k) -> u
// =========================================================================
template <typename CtrlFn>
static SISOMetrics sim(CtrlFn ctrl_fn, double ref = 1.0)
{
    StateSpace plant = make_plant();
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    std::vector<double> y_hist(SIM_STEPS, 0.0), u_hist(SIM_STEPS, 0.0);
    double y_prev = 0.0;
    for (int k = 0; k < SIM_STEPS; ++k)
    {
        double u = std::clamp(ctrl_fn(ref, y_prev, k), -UMAX, UMAX);
        u_hist[k] = u;
        y_hist[k] = plant_step(plant, x, u);
        y_prev = y_hist[k];
    }
    return compute_metrics(y_hist, u_hist, ref);
}

// =========================================================================
// A. PID helper - build from gains, simulate, print
// =========================================================================
static SISOMetrics pid_rule(double Kp, double Ki, double Kd, const std::string &tag)
{
    PIDParams pp;
    pp.Kp = Kp;
    pp.Ki = Ki;
    pp.Kd = Kd;
    pp.N = 10.0;
    pp.uMin = -UMAX;
    pp.uMax = UMAX;
    DiscretePID pid(pp, Ts);
    auto m = sim([&](double r, double y, int)
                 { return pid.compute(r - y); });
    print_metrics(tag, m);
    return m;
}

// =========================================================================
// Collect step-response data and return FOPDT model
// =========================================================================
static StepResponseTuner::FOPDTModel identify_fopdt(double step_mag = 1.0,
                                                    int n_steps = 2000)
{
    StateSpace plant = make_plant();
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    std::vector<double> t_data, y_data;
    t_data.reserve(n_steps);
    y_data.reserve(n_steps);
    for (int k = 0; k < n_steps; ++k)
    {
        t_data.push_back(k * Ts);
        y_data.push_back(plant_step(plant, x, step_mag));
    }
    return StepResponseTuner::identify(t_data, y_data, step_mag);
}

// =========================================================================
// main
// =========================================================================
int main()
{
    std::cout << "=================================================================\n";
    std::cout << " Coupled SISO - All Tuning Methods\n";
    std::cout << " G(s) = 1/(s^2+1.5s+1), Ts=0.01s, u \\in [-10,10]\n";
    std::cout << "=================================================================\n";
    std::cout << "\n  Plant discrete SS (ZOH, Ts=0.01 s):\n"
              << "    A = [1.98511, -0.98522; 1.0, 0.0]\n"
              << "    B = [4.9625e-5; 4.9125e-5],  C = [1.0, 0.0]\n"
              << "  DC gain: 1.0   Eigenvalues: -0.75+/-0.961j (continuous)\n\n";

    std::vector<std::string> labels;
    std::vector<SISOMetrics> results;
    auto add = [&](const std::string &lbl, SISOMetrics m)
    {
        labels.push_back(lbl);
        results.push_back(m);
    };

    // -----------------------------------------------------------------------
    // A1. PID - Relay -> Ziegler-Nichols
    // -----------------------------------------------------------------------
    {
        RelayTunerConfig rcfg;
        rcfg.relayAmplitude = 0.5;
        rcfg.hysteresis = 0.0;
        rcfg.cyclesRequired = 4;
        RelayAutoTuner relay(rcfg, Ts);

        StateSpace plant_r = make_plant();
        Eigen::VectorXd xr = Eigen::VectorXd::Zero(plant_r.stateSize());
        double y_r = 0.0;
        for (int k = 0; k < 6000 && !relay.isDone(); ++k)
            y_r = plant_step(plant_r, xr, relay.step(y_r));

        double Ku = relay.ultimateGain();
        double Tu = relay.ultimatePeriod();
        // Classic ZN PID formula
        double Kp = 0.6 * Ku, Ki = Kp / (0.5 * Tu), Kd = Kp * (0.125 * Tu);
        std::cout << "  ZN relay: Ku=" << std::setprecision(4) << Ku
                  << " Tu=" << Tu << " -> Kp=" << Kp
                  << " Ki=" << Ki << " Kd=" << Kd << "\n";
        add("PID-ZN", pid_rule(Kp, Ki, Kd, "PID-ZN"));
    }

    // -----------------------------------------------------------------------
    // A2. PID - IMC (lambda = 0.3, 0.5, 1.0)
    // -----------------------------------------------------------------------
    {
        auto fopdt = identify_fopdt();
        std::cout << "  FOPDT: K=" << std::setprecision(4) << fopdt.K
                  << " τ=" << fopdt.tau << " θ=" << fopdt.theta << "\n";
        for (double lam : {0.3, 0.5, 1.0})
        {
            auto res = TunerSuite::imcPID(CtrlKind::PID, fopdt, Ts, lam);
            std::string tag = "PID-IMC-" + std::to_string((int)(lam * 10));
            if (res.success)
            {
                std::cout << "  " << tag << ": Kp=" << std::setprecision(4)
                          << res.params.Kp << " Ki=" << res.params.Ki
                          << " Kd=" << res.params.Kd << "\n";
                add(tag, pid_rule(res.params.Kp, res.params.Ki, res.params.Kd, tag));
            }
        }
    }

    // -----------------------------------------------------------------------
    // A3. PID - Cohen-Coon
    // -----------------------------------------------------------------------
    {
        auto fopdt = identify_fopdt();
        auto res = TunerSuite::cohenCoon(CtrlKind::PID, fopdt, Ts);
        if (res.success)
        {
            std::cout << "  CohenCoon: Kp=" << std::setprecision(4) << res.params.Kp
                      << " Ki=" << res.params.Ki << " Kd=" << res.params.Kd << "\n";
            add("PID-CohenCoon", pid_rule(res.params.Kp, res.params.Ki,
                                          res.params.Kd, "PID-CohenCoon"));
        }
    }

    // -----------------------------------------------------------------------
    // A4. PID - Nelder-Mead optimised (ISE+ITAE+energy)
    // -----------------------------------------------------------------------
    {
        auto cost = [](const std::vector<double> &p) -> double
        {
            if (p[0] <= 0 || p[1] < 0 || p[2] < 0)
                return 1e6;
            PIDParams pp;
            pp.Kp = p[0];
            pp.Ki = p[1];
            pp.Kd = p[2];
            pp.N = 10.0;
            pp.uMin = -UMAX;
            pp.uMax = UMAX;
            DiscretePID pid(pp, Ts);
            StateSpace pl = make_plant();
            Eigen::VectorXd xl = Eigen::VectorXd::Zero(pl.stateSize());
            double J = 0, yp = 0;
            for (int k = 0; k < SIM_STEPS; ++k)
            {
                double u = std::clamp(pid.compute(1.0 - yp), -UMAX, UMAX);
                double y = plant_step(pl, xl, u);
                double t = k * Ts, e = 1.0 - y;
                J += e * e * Ts + 0.1 * t * std::abs(e) * Ts + 0.01 * u * u * Ts;
                yp = y;
            }
            return J;
        };
        auto xopt = nelder_mead(cost, {5.0, 1.0, 0.5}, {{0.01, 30}, {0, 20}, {0, 5}}, 500);
        std::cout << "  NM-PID: Kp=" << std::setprecision(4) << xopt[0]
                  << " Ki=" << xopt[1] << " Kd=" << xopt[2] << "\n";
        add("PID-NM", pid_rule(xopt[0], xopt[1], xopt[2], "PID-NM"));
    }

    // -----------------------------------------------------------------------
    // B1. LQR - Bryson's Rule (Q=I, R=0.04)
    // -----------------------------------------------------------------------
    {
        StateSpace plant_ref = make_plant();
        LQRParams lp;
        lp.Q = Eigen::Matrix2d::Identity();
        lp.R = 0.04 * Eigen::Matrix<double, 1, 1>::Identity();
        DiscreteLQR lqr(plant_ref, lp);
        Eigen::MatrixXd K = lqr.gainMatrix(); // 1*2

        // Feedforward Nbar for setpoint tracking: Nbar = -1/(C*(A_cl-I)^{-1}*B)
        Eigen::MatrixXd A_cl = plant_ref.A - plant_ref.B * K;
        double Nbar = -1.0 /
                      (plant_ref.C * (A_cl - Eigen::MatrixXd::Identity(2, 2)).inverse() * plant_ref.B)(0, 0);

        StateSpace plant_lqr = make_plant();
        Eigen::VectorXd xl = Eigen::VectorXd::Zero(2);
        std::vector<double> yh(SIM_STEPS), uh(SIM_STEPS);
        for (int k = 0; k < SIM_STEPS; ++k)
        {
            double u = std::clamp((-K * xl)(0) + Nbar * 1.0, -UMAX, UMAX);
            uh[k] = u;
            yh[k] = plant_step(plant_lqr, xl, u);
        }
        auto m = compute_metrics(yh, uh, 1.0);
        std::cout << "  LQR-Bryson: K=" << std::setprecision(4)
                  << K(0, 0) << ", " << K(0, 1) << "  Nbar=" << Nbar << "\n";
        print_metrics("LQR-Bryson", m);
        add("LQR-Bryson", m);
    }

    // -----------------------------------------------------------------------
    // B2. LQR - Nelder-Mead optimised (q1, q2, r in natural scale)
    // -----------------------------------------------------------------------
    {
        auto cost = [](const std::vector<double> &p) -> double
        {
            if (p[0] <= 0 || p[1] <= 0 || p[2] <= 0)
                return 1e6;
            StateSpace pr = make_plant();
            LQRParams lp;
            lp.Q = Eigen::Matrix2d::Zero();
            lp.Q(0, 0) = p[0];
            lp.Q(1, 1) = p[1];
            lp.R = p[2] * Eigen::Matrix<double, 1, 1>::Identity();
            DiscreteLQR lqr(pr, lp);
            Eigen::MatrixXd K = lqr.gainMatrix();
            Eigen::MatrixXd A_cl = pr.A - pr.B * K;
            double Nbar = -1.0 / (pr.C * (A_cl - Eigen::MatrixXd::Identity(2, 2)).inverse() * pr.B)(0, 0);
            StateSpace plant = make_plant();
            Eigen::VectorXd xl = Eigen::VectorXd::Zero(2);
            double J = 0;
            for (int k = 0; k < SIM_STEPS; ++k)
            {
                double u = std::clamp((-K * xl)(0) + Nbar * 1.0, -UMAX, UMAX);
                double y = plant_step(plant, xl, u);
                double t = k * Ts, e = 1 - y;
                J += e * e * Ts + 0.1 * t * std::abs(e) * Ts + 0.01 * u * u * Ts;
            }
            return J;
        };
        auto xopt = nelder_mead(cost, {1.0, 0.25, 0.04},
                                {{0.001, 100}, {0.001, 100}, {0.001, 10}}, 400);
        std::cout << "  NM-LQR: q1=" << std::setprecision(4) << xopt[0]
                  << " q2=" << xopt[1] << " r=" << xopt[2] << "\n";

        StateSpace pr = make_plant();
        LQRParams lp;
        lp.Q = Eigen::Matrix2d::Zero();
        lp.Q(0, 0) = xopt[0];
        lp.Q(1, 1) = xopt[1];
        lp.R = xopt[2] * Eigen::Matrix<double, 1, 1>::Identity();
        DiscreteLQR lqr(pr, lp);
        Eigen::MatrixXd K = lqr.gainMatrix();
        Eigen::MatrixXd A_cl = pr.A - pr.B * K;
        double Nbar = -1.0 / (pr.C * (A_cl - Eigen::MatrixXd::Identity(2, 2)).inverse() * pr.B)(0, 0);

        StateSpace plant_lqr = make_plant();
        Eigen::VectorXd xl = Eigen::VectorXd::Zero(2);
        std::vector<double> yh(SIM_STEPS), uh(SIM_STEPS);
        for (int k = 0; k < SIM_STEPS; ++k)
        {
            uh[k] = std::clamp((-K * xl)(0) + Nbar, -UMAX, UMAX);
            yh[k] = plant_step(plant_lqr, xl, uh[k]);
        }
        auto m = compute_metrics(yh, uh, 1.0);
        print_metrics("LQR-NM", m);
        add("LQR-NM", m);
    }

    // -----------------------------------------------------------------------
    // C1. SMC - default params
    // -----------------------------------------------------------------------
    {
        SMCParams sp;
        sp.c_e = 1.0;
        sp.c_de = 10.0;
        sp.K = 5.0;
        sp.phi = 0.1;
        sp.uMin = -UMAX;
        sp.uMax = UMAX;
        DiscreteSMC smc(sp, Ts);
        auto m = sim([&](double r, double y, int)
                     { return smc.compute(r - y); });
        print_metrics("SMC-default", m);
        add("SMC-default", m);
    }

    // -----------------------------------------------------------------------
    // C2. SMC - Nelder-Mead optimised
    // -----------------------------------------------------------------------
    {
        auto cost = [](const std::vector<double> &p) -> double
        {
            if (p[0] <= 0 || p[1] <= 0 || p[2] <= 0 || p[3] <= 0)
                return 1e6;
            SMCParams sp;
            sp.c_e = p[0];
            sp.c_de = p[1];
            sp.K = p[2];
            sp.phi = p[3];
            sp.uMin = -UMAX;
            sp.uMax = UMAX;
            DiscreteSMC smc(sp, Ts);
            StateSpace pl = make_plant();
            Eigen::VectorXd xl = Eigen::VectorXd::Zero(2);
            double J = 0, yp = 0;
            for (int k = 0; k < SIM_STEPS; ++k)
            {
                double u = std::clamp(smc.compute(1.0 - yp), -UMAX, UMAX);
                double y = plant_step(pl, xl, u);
                double t = k * Ts, e = 1 - y;
                J += e * e * Ts + 0.1 * t * std::abs(e) * Ts + 0.01 * u * u * Ts;
                yp = y;
            }
            return J;
        };
        auto xopt = nelder_mead(cost, {1.0, 10.0, 5.0, 0.1},
                                {{0.01, 10}, {0.1, 50}, {0.1, 20}, {0.01, 2}}, 400);
        std::cout << "  NM-SMC: c_e=" << std::setprecision(4) << xopt[0]
                  << " c_de=" << xopt[1] << " K=" << xopt[2] << " phi=" << xopt[3] << "\n";
        SMCParams sp;
        sp.c_e = xopt[0];
        sp.c_de = xopt[1];
        sp.K = xopt[2];
        sp.phi = xopt[3];
        sp.uMin = -UMAX;
        sp.uMax = UMAX;
        DiscreteSMC smc2(sp, Ts);
        auto m2 = sim([&](double r, double y, int)
                      { return smc2.compute(r - y); });
        print_metrics("SMC-NM", m2);
        add("SMC-NM", m2);
    }

    // -----------------------------------------------------------------------
    // D1. ADRC - Gao bandwidth params
    // -----------------------------------------------------------------------
    {
        ADRCParams ap;
        ap.omega_o = 20.0;
        ap.omega_c = 4.0;
        ap.b0 = 1e-4;
        ap.uMin = -UMAX;
        ap.uMax = UMAX;
        DiscreteADRC adrc(ap, Ts);
        auto m = sim([&](double r, double y, int)
                     { return adrc.computeTracking(y, r); });
        print_metrics("ADRC-default", m);
        add("ADRC-default", m);
    }

    // -----------------------------------------------------------------------
    // D2. ADRC - Nelder-Mead optimised
    // -----------------------------------------------------------------------
    {
        auto cost = [](const std::vector<double> &p) -> double
        {
            if (p[0] <= 0 || p[1] <= 0 || p[2] <= 0)
                return 1e6;
            ADRCParams ap;
            ap.omega_o = p[0];
            ap.omega_c = p[1];
            ap.b0 = p[2];
            ap.uMin = -UMAX;
            ap.uMax = UMAX;
            DiscreteADRC adrc(ap, Ts);
            StateSpace pl = make_plant();
            Eigen::VectorXd xl = Eigen::VectorXd::Zero(2);
            double J = 0, yp = 0;
            for (int k = 0; k < SIM_STEPS; ++k)
            {
                double u = std::clamp(adrc.computeTracking(yp, 1.0), -UMAX, UMAX);
                double y = plant_step(pl, xl, u);
                double t = k * Ts, e = 1 - y;
                J += e * e * Ts + 0.1 * t * std::abs(e) * Ts + 0.01 * u * u * Ts;
                yp = y;
            }
            return J;
        };
        auto xopt = nelder_mead(cost, {20.0, 4.0, 1e-4},
                                {{1.0, 100}, {0.5, 20}, {1e-6, 1e-2}}, 400);
        std::cout << "  NM-ADRC: omegao=" << std::setprecision(4) << xopt[0]
                  << " omegac=" << xopt[1] << " b0=" << xopt[2] << "\n";
        ADRCParams ap;
        ap.omega_o = xopt[0];
        ap.omega_c = xopt[1];
        ap.b0 = xopt[2];
        ap.uMin = -UMAX;
        ap.uMax = UMAX;
        DiscreteADRC adrc2(ap, Ts);
        auto m2 = sim([&](double r, double y, int)
                      { return adrc2.computeTracking(y, r); });
        print_metrics("ADRC-NM", m2);
        add("ADRC-NM", m2);
    }

    // -----------------------------------------------------------------------
    // E. Lead-Lag - frequency-domain loop shaping (omegac=2 rad/s, φ=50^\circ)
    // -----------------------------------------------------------------------
    {
        // |G(j.2)| for G(s)=1/(s^2+1.5s+1): |G(j2)| = 1/|((j2)^2+1.5(j2)+1)|
        // = 1/|(-4+1+3j)| = 1/|(-3+3j)| = 1/(3\sqrt2) approx = 0.2357
        LoopShapingTuner::Input lsin;
        lsin.omega_c = 2.0;
        lsin.phase_add_deg = 50.0;
        lsin.gain_at_wc = 1.0 / std::sqrt(9.0 + 9.0); // |G(j2)| approx = 0.2357
        auto llp = LoopShapingTuner::tuneImpl(lsin);
        DiscreteLeadLag ll(llp, Ts);
        auto m = sim([&](double r, double y, int)
                     { return ll.compute(r - y); });
        std::cout << "  LeadLag-LS: gain=" << std::setprecision(4) << llp.gain
                  << " z=" << llp.continuousZero << " p=" << llp.continuousPole << "\n";
        print_metrics("LeadLag-LS", m);
        add("LeadLag-LS", m);
    }

    // -----------------------------------------------------------------------
    // F. Smith Predictor - IMC inner PID + 3-step delay model
    // -----------------------------------------------------------------------
    {
        auto fopdt = identify_fopdt();
        auto res = TunerSuite::imcPID(CtrlKind::Smith, fopdt, Ts, 0.5);
        PIDParams pp;
        if (res.success)
        {
            pp.Kp = res.params.Kp;
            pp.Ki = res.params.Ki;
            pp.Kd = res.params.Kd;
        }
        else
        {
            pp.Kp = 2.0;
            pp.Ki = 1.0;
            pp.Kd = 0.1; // safe fallback
        }
        pp.N = 10.0;
        pp.uMin = -UMAX;
        pp.uMax = UMAX;
        auto inner = std::make_shared<DiscretePID>(pp, Ts);

        StateSpace model_sp = make_plant();
        constexpr int DELAY = 3;
        SmithPredictor smith(inner, model_sp, DELAY);

        StateSpace plant_sp = make_plant();
        Eigen::VectorXd xs = Eigen::VectorXd::Zero(plant_sp.stateSize());
        std::vector<double> yh(SIM_STEPS), uh(SIM_STEPS);
        double yp = 0.0;
        for (int k = 0; k < SIM_STEPS; ++k)
        {
            uh[k] = smith.compute(1.0 - yp);
            yh[k] = plant_step(plant_sp, xs, uh[k]);
            yp = yh[k];
        }
        auto m = compute_metrics(yh, uh, 1.0);
        print_metrics("SmithPredictor", m);
        add("SmithPredictor", m);
    }

    // =========================================================================
    // Summary: Pareto ranking by composite cost J
    // =========================================================================
    std::cout << "\n=== Pareto Summary (J = ISE + 0.1.ITAE + 0.01.Energy) ===\n";
    // Build sorted index
    std::vector<size_t> rank_idx(results.size());
    std::iota(rank_idx.begin(), rank_idx.end(), 0);
    std::sort(rank_idx.begin(), rank_idx.end(),
              [&](size_t a, size_t b)
              { return results[a].J < results[b].J; });
    for (size_t r = 0; r < rank_idx.size(); ++r)
    {
        size_t i = rank_idx[r];
        std::cout << (r == 0 ? "  * " : "    ") << "#" << (r + 1) << " "
                  << std::setw(22) << labels[i]
                  << "  J=" << std::setprecision(5) << results[i].J << "\n";
    }

    // =========================================================================
    // Fuzzy performance evaluation for all methods
    // =========================================================================
    double ise_worst = 0.0;
    for (auto &m : results)
        ise_worst = std::max(ise_worst, m.ISE);
    std::cout << "\n=== Fuzzy Performance Scores ===\n";
    for (size_t i = 0; i < results.size(); ++i)
    {
        double ise_n = results[i].ISE / std::max(ise_worst, 1e-9);
        double st_n = results[i].settle_time / (SIM_STEPS * Ts);
        auto fr = fuzzy::evaluate(ise_n, results[i].overshoot_pct, st_n);
        fuzzy::print_report(labels[i], ise_n, results[i].overshoot_pct, st_n, fr);
    }

    // =========================================================================
    // Save CSV
    // =========================================================================
    std::ofstream f("siso_coupled_results.csv");
    f << "method,ISE,ITAE,energy,overshoot_pct,settle_time,J\n";
    for (size_t i = 0; i < results.size(); ++i)
        f << labels[i] << "," << results[i].ISE << "," << results[i].ITAE << ","
          << results[i].energy << "," << results[i].overshoot_pct << ","
          << results[i].settle_time << "," << results[i].J << "\n";
    std::cout << "\n  Results saved -> siso_coupled_results.csv\n";
    return 0;
}
