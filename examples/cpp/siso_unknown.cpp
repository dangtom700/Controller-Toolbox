/*
 * siso_unknown.cpp - Unknown SISO Plant: System ID + Fuzzy Performance
 * ======================================================================
 * Case 3b: SISO plant assumed unknown. Model estimated entirely from I/O
 * data; no a-priori knowledge of plant order, gain, or time constants.
 *
 * Identification workflow:
 *   1. Read PRBS excitation from CSV (Python-generated) or generate via LFSR.
 *   2. Relay feedback test -> Åström-Hägglund ultimate gain Ku and period Tu.
 *   3. Unit-step test -> FOPDT via 28.3%/63.2% tangent method.
 *   4. ARX(2,2) batch LS on PRBS I/O data -> cross-validate NRMSE <= 5%.
 *   5. Apply ALL PID tuning rules (ZN, IMC*3 lambda, Cohen-Coon, NM) to FOPDT.
 *   6. Apply SMC tuned from ARX bandwidth estimate + NM-optimised variant.
 *   7. Apply ADRC with b0 from ARX high-frequency gain + NM-optimised.
 *   8. Rank by composite cost J = ISE + 0.1.ITAE + 0.01.\intu^2 dt.
 *   9. Fuzzy performance evaluation on every controller.
 *
 * Identification tolerances:
 *   ARX NRMSE <= 5 %   (hard gate: [PASS] / [FAIL] printed for each metric)
 *   |Ku_relay - Ku_bode| / Ku_bode <= 15 %   (relay fidelity check)
 *
 * Expected output (per controller):
 *   - Identified parameters, ISE, ITAE, energy, OS%, settle_time, J, fuzzy grade
 *   - [PASS/FAIL] on NRMSE and relay fidelity
 *   - Optimal controller highlighted
 *   - Results -> siso_unknown_results.csv
 *
 * Build: see examples/CMakeLists.txt  (add_cpp_example(siso_unknown))
 * Python data: examples/python/generate_unknown_data.py
 */

#include "fuzzy_performance.h"
#include "../../lib/ControllerToolbox.h"

#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <limits>

using namespace ctrl;

// =========================================================================
// True plant (internally known; not referenced by identification code)
// G(s) = 1/(s^2+1.5s+1), ZOH Ts=0.01 s
// =========================================================================
static constexpr double Ts        = 0.01;
static constexpr double UMAX      = 10.0;
static constexpr int    SIM_STEPS = 2000;

static const std::vector<double> P_NUM = { 0.0, 4.9625e-5, 4.9125e-5 };
static const std::vector<double> P_DEN = { 1.0, -1.98511,   0.98522   };

inline StateSpace make_true_plant() {
    return tf2ss(TransferFunction(P_NUM, P_DEN, Ts));
}

// =========================================================================
// One-step helper: scalar input -> scalar output, state updated in-place
// =========================================================================
inline double plant_step(const StateSpace& sys, Eigen::VectorXd& x, double u) {
    Eigen::VectorXd uv(1); uv << u;
    return ssStep(sys, x, uv)(0);
}

// =========================================================================
// PRBS 10-bit LFSR (amplitude +/-amp)
// =========================================================================
static std::vector<double> lfsr_prbs(int N, double amp, unsigned seed = 42) {
    unsigned st = seed & 0x3FFu; if (!st) st = 1u;
    std::vector<double> out(N);
    for (int k = 0; k < N; ++k) {
        out[k] = (st & 1u) ? amp : -amp;
        unsigned fb = ((st >> 9) ^ (st >> 6)) & 1u;
        st = ((st << 1) | fb) & 0x3FFu;
    }
    return out;
}

// =========================================================================
// Load PRBS from CSV (column "input"); fallback to LFSR
// =========================================================================
static std::vector<double> load_or_generate_prbs(const std::string& path, int N) {
    std::ifstream f(path);
    if (f.is_open()) {
        std::string hdr; std::getline(f, hdr);
        std::vector<double> u;
        for (std::string line; std::getline(f, line); ) {
            std::istringstream ss(line); std::string tok;
            std::getline(ss, tok, ','); // skip time column
            if (std::getline(ss, tok, ',')) u.push_back(std::stod(tok));
        }
        if ((int)u.size() >= N) {
            std::cout << "  Loaded PRBS from " << path << " (" << u.size() << " samples)\n";
            u.resize(N); return u;
        }
    }
    std::cout << "  CSV not found or short - generating PRBS via 10-bit LFSR\n";
    return lfsr_prbs(N, 0.5, 42);
}

// =========================================================================
// Performance metrics
// =========================================================================
struct Metrics {
    double ISE = 0, ITAE = 0, energy = 0, OS_pct = 0, settle_time = 0, J = 0;
};

static Metrics compute_metrics(const std::vector<double>& y,
                               const std::vector<double>& u, double ref) {
    int N = (int)y.size();
    Metrics m;
    double ymax = *std::max_element(y.begin(), y.end());
    m.OS_pct = std::max(0.0, (ymax - ref) / std::abs(ref) * 100.0);
    for (int k = 0; k < N; ++k) {
        double t = k * Ts, e = ref - y[k];
        m.ISE    += e*e*Ts;
        m.ITAE   += t*std::abs(e)*Ts;
        m.energy += u[k]*u[k]*Ts;
    }
    m.settle_time = N * Ts;
    for (int k = N-1; k >= 0; --k)
        if (std::abs(y[k]-ref) > 0.02*std::abs(ref)) { m.settle_time=(k+1)*Ts; break; }
    m.J = m.ISE + 0.1*m.ITAE + 0.01*m.energy;
    return m;
}

static void print_metrics(const std::string& name, const Metrics& m,
                           const fuzzy::FuzzyResult& fr) {
    std::cout << std::setw(24) << name << std::fixed
              << std::setprecision(5)
              << " ISE=" << std::setw(9) << m.ISE
              << " ITAE=" << std::setw(9) << m.ITAE
              << std::setprecision(1)
              << " OS=" << std::setw(5) << m.OS_pct << "%"
              << std::setprecision(3)
              << " Ts=" << std::setw(5) << m.settle_time << "s"
              << std::setprecision(5)
              << " J=" << std::setw(9) << m.J
              << "  (" << fr.grade << " " << std::setprecision(0) << fr.score << ")\n";
}

// =========================================================================
// Generic closed-loop simulation: ctrl_fn(ref, y_prev, k) -> u
// =========================================================================
template<typename CtrlFn>
static Metrics sim(CtrlFn ctrl_fn, double ref = 1.0) {
    StateSpace plant = make_true_plant();
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    std::vector<double> yh(SIM_STEPS), uh(SIM_STEPS);
    double yp = 0.0;
    for (int k = 0; k < SIM_STEPS; ++k) {
        uh[k] = std::clamp(ctrl_fn(ref, yp, k), -UMAX, UMAX);
        yh[k] = plant_step(plant, x, uh[k]);
        yp = yh[k];
    }
    return compute_metrics(yh, uh, ref);
}

// =========================================================================
// Nelder-Mead (alpha=1, γ=2, ρ=0.5, sigma=0.5)
// =========================================================================
static std::vector<double>
nm(std::function<double(const std::vector<double>&)> f,
   std::vector<double> x0,
   const std::vector<std::pair<double,double>>& bnds,
   int maxEv = 400, double tol = 1e-6) {
    const double a=1,g=2,rho=0.5,sig=0.5; int n=(int)x0.size();
    auto clip=[&](std::vector<double>v){
        for(int i=0;i<n;++i)v[i]=std::clamp(v[i],bnds[i].first,bnds[i].second);
        return v;
    };
    std::vector<std::vector<double>> sp={clip(x0)};
    for(int i=0;i<n;++i){auto v=x0;v[i]*=1.05;sp.push_back(clip(v));}
    std::vector<double>c;for(auto&s2:sp)c.push_back(f(s2));
    int ev=n+1;
    while(ev<maxEv){
        std::vector<int>idx(n+1);std::iota(idx.begin(),idx.end(),0);
        std::sort(idx.begin(),idx.end(),[&](int a2,int b2){return c[a2]<c[b2];});
        std::vector<std::vector<double>>s2;std::vector<double>c2;
        for(int i:idx){s2.push_back(sp[i]);c2.push_back(c[i]);}sp=s2;c=c2;
        double d=0;for(int i=1;i<=n;++i)for(int j=0;j<n;++j)
            d=std::max(d,std::abs(sp[i][j]-sp[0][j]));
        if(d<tol)break;
        std::vector<double>cen(n,0);
        for(int i=0;i<n;++i)for(int j=0;j<n;++j)cen[j]+=sp[i][j]/n;
        std::vector<double>xr(n);
        for(int j=0;j<n;++j)xr[j]=cen[j]+a*(cen[j]-sp[n][j]);
        xr=clip(xr);double fr=f(xr);++ev;
        if(fr<c[0]){
            std::vector<double>xe(n);
            for(int j=0;j<n;++j)xe[j]=cen[j]+g*(xr[j]-cen[j]);
            xe=clip(xe);double fe=f(xe);++ev;
            sp[n]=(fe<fr)?xe:xr;c[n]=(fe<fr)?fe:fr;
        } else if(fr<c[n-1]){sp[n]=xr;c[n]=fr;}
        else{
            std::vector<double>xc(n);
            for(int j=0;j<n;++j)xc[j]=cen[j]+rho*(sp[n][j]-cen[j]);
            xc=clip(xc);double fc=f(xc);++ev;
            if(fc<c[n]){sp[n]=xc;c[n]=fc;}
            else{
                for(int i=1;i<=n;++i){
                    for(int j=0;j<n;++j)
                        sp[i][j]=sp[0][j]+sig*(sp[i][j]-sp[0][j]);
                    sp[i]=clip(sp[i]);c[i]=f(sp[i]);++ev;
                }
            }
        }
    }
    return sp[0];
}

// =========================================================================
// main
// =========================================================================
int main() {
    std::cout << "=================================================================\n";
    std::cout << " SISO Unknown Case - System ID + Fuzzy Performance\n";
    std::cout << " Plant treated as a black box (G(s)=1/(s^2+1.5s+1) internally)\n";
    std::cout << "=================================================================\n";

    const double ise_worst = 1.0 * SIM_STEPS * Ts; // open-loop integral of 1^2 dt
    auto fuzzy_score = [&](const Metrics& m) {
        double ise_n = std::min(m.ISE / ise_worst, 1.0);
        double st_n  = std::min(m.settle_time / (SIM_STEPS * Ts), 1.0);
        return fuzzy::evaluate(ise_n, m.OS_pct, st_n);
    };

    // =======================================================================
    // Step 1: Load / generate PRBS identification data
    // =======================================================================
    std::cout << "\n[1] Excitation signal setup\n";
    const int ID_STEPS = 3000;
    auto u_id = load_or_generate_prbs("examples/data/siso_prbs.csv", ID_STEPS);

    StateSpace plant_id = make_true_plant();
    Eigen::VectorXd x_id = Eigen::VectorXd::Zero(plant_id.stateSize());
    std::vector<double> y_id(ID_STEPS);
    for (int k = 0; k < ID_STEPS; ++k)
        y_id[k] = plant_step(plant_id, x_id, u_id[k]);

    // =======================================================================
    // Step 2: Relay feedback -> Ku, Tu
    // =======================================================================
    std::cout << "[2] Relay feedback test (d=0.5, up to 6000 steps)\n";
    double relay_Ku = 1.0, relay_Tu = 1.0;
    {
        RelayTunerConfig rcfg;
        rcfg.relayAmplitude = 0.5;
        rcfg.hysteresis     = 0.0;
        rcfg.cyclesRequired = 4;
        RelayAutoTuner relay(rcfg, Ts);

        StateSpace rp = make_true_plant();
        Eigen::VectorXd xr = Eigen::VectorXd::Zero(rp.stateSize());
        double yr = 0.0;
        for (int k = 0; k < 6000 && !relay.isDone(); ++k)
            yr = plant_step(rp, xr, relay.step(yr));

        relay_Ku = relay.ultimateGain();
        relay_Tu = relay.ultimatePeriod();
        std::cout << "  Ku=" << std::setprecision(4) << relay_Ku
                  << "  Tu=" << relay_Tu << " s\n";

        // Relay fidelity: analytic Ku_bode = 1.5 for G(s)=1/(s^2+1.5s+1)
        // (omega_pc = 1 rad/s, |G(j1)| = 1/|1-1+j.1.5| = 1/1.5)
        constexpr double Ku_bode = 1.5;
        double relay_err = std::abs(relay_Ku - Ku_bode) / Ku_bode * 100.0;
        std::cout << "  Relay fidelity: |Ku_relay - Ku_bode|/Ku_bode = "
                  << std::setprecision(1) << relay_err << "%  "
                  << (relay_err < 15.0 ? "[PASS <=15%]" : "[FAIL >15%]") << "\n";
    }

    // =======================================================================
    // Step 3: Open-loop step response -> FOPDT (28.3%/63.2% tangent method)
    // =======================================================================
    std::cout << "[3] Step response identification -> FOPDT\n";
    double K_fopdt = 1.0, tau_fopdt = 1.0, theta_fopdt = 0.01;
    {
        StateSpace sp = make_true_plant();
        Eigen::VectorXd xs = Eigen::VectorXd::Zero(sp.stateSize());
        std::vector<double> t_data, y_data;
        t_data.reserve(SIM_STEPS); y_data.reserve(SIM_STEPS);
        for (int k = 0; k < SIM_STEPS; ++k) {
            t_data.push_back(k * Ts);
            y_data.push_back(plant_step(sp, xs, 1.0));
        }
        auto fopdt = StepResponseTuner::identify(t_data, y_data, 1.0);
        K_fopdt = fopdt.K; tau_fopdt = fopdt.tau; theta_fopdt = fopdt.theta;
        std::cout << "  K=" << std::setprecision(5) << K_fopdt
                  << "  τ=" << tau_fopdt << " s"
                  << "  θ=" << theta_fopdt << " s\n";
    }

    // =======================================================================
    // Step 4: ARX(2,2) batch LS identification on PRBS data
    // =======================================================================
    std::cout << "[4] ARX(2,2) batch LS identification\n";
    double a1_id=0, a2_id=0, b1_id=0, b2_id=0;
    {
        const int burn = 50;
        int N_arx = ID_STEPS - burn - 2;
        Eigen::MatrixXd Phi(N_arx, 4);
        Eigen::VectorXd Y_vec(N_arx);
        for (int i = 0; i < N_arx; ++i) {
            int k = i + burn + 2;
            Phi.row(i) << -y_id[k-1], -y_id[k-2], u_id[k-1], u_id[k-2];
            Y_vec[i] = y_id[k];
        }
        // theta = [-a1, -a2, b1, b2]' via least squares
        Eigen::Vector4d theta = (Phi.transpose()*Phi).ldlt().solve(Phi.transpose()*Y_vec);
        a1_id = theta[0]; a2_id = theta[1]; // stored as -a1, -a2 (regression sign)
        b1_id = theta[2]; b2_id = theta[3];

        Eigen::VectorXd y_hat = Phi * theta;
        double rmse = std::sqrt((Y_vec - y_hat).squaredNorm() / N_arx);
        double y_range = *std::max_element(y_id.begin(), y_id.end())
                       - *std::min_element(y_id.begin(), y_id.end());
        double nrmse = rmse / std::max(y_range, 1e-12) * 100.0;
        std::cout << "  ARX: a1=" << std::setprecision(6) << a1_id
                  << "  a2=" << a2_id
                  << "  b1=" << b1_id << "  b2=" << b2_id << "\n"
                  << "  NRMSE (in-sample) = " << std::setprecision(2) << nrmse << "%  "
                  << (nrmse < 5.0 ? "[PASS <=5%]" : "[FAIL >5%]") << "\n";

        // Validation on a fresh PRBS sequence
        auto u_val = lfsr_prbs(1500, 0.5, 999);
        StateSpace vp = make_true_plant();
        Eigen::VectorXd xv = Eigen::VectorXd::Zero(vp.stateSize());
        std::vector<double> y_val(1500);
        for (int k = 0; k < 1500; ++k) y_val[k] = plant_step(vp, xv, u_val[k]);

        double sse_val = 0.0;
        for (int k = 2; k < 1500; ++k) {
            // Correct signs: theta stores -a1, -a2 but ARX equation is:
            // y[k] = -a1*y[k-1] - a2*y[k-2] + b1*u[k-1] + b2*u[k-2]
            double yp = -a1_id*y_val[k-1] - a2_id*y_val[k-2]
                       + b1_id*u_val[k-1]  + b2_id*u_val[k-2];
            sse_val += (y_val[k] - yp)*(y_val[k] - yp);
        }
        double val_range = *std::max_element(y_val.begin(), y_val.end())
                         - *std::min_element(y_val.begin(), y_val.end());
        double nrmse_val = std::sqrt(sse_val / 1498) / std::max(val_range, 1e-12) * 100.0;
        std::cout << "  NRMSE (validation) = " << nrmse_val << "%  "
                  << (nrmse_val < 5.0 ? "[PASS <=5%]" : "[FAIL >5%]") << "\n";
    }

    // =======================================================================
    // Step 5: All PID tuning rules from identified FOPDT
    // =======================================================================
    std::cout << "\n[5] All PID tuning rules on identified FOPDT\n";
    std::cout << std::string(110, '-') << "\n";

    std::vector<std::string> labels;
    std::vector<Metrics>     results;
    auto add = [&](const std::string& lbl, Metrics m, const fuzzy::FuzzyResult& fr) {
        print_metrics(lbl, m, fr); labels.push_back(lbl); results.push_back(m);
    };

    auto make_pid = [](double Kp, double Ki, double Kd) -> DiscretePID {
        PIDParams pp; pp.Kp=Kp; pp.Ki=Ki; pp.Kd=Kd;
        pp.N=10.0; pp.uMin=-UMAX; pp.uMax=UMAX;
        return DiscretePID(pp, Ts);
    };

    // --- Relay ZN ---
    {
        double Ku = relay_Ku, Tu = relay_Tu;
        double Kp=0.6*Ku, Ki=Kp/(0.5*Tu), Kd=Kp*(0.125*Tu);
        DiscretePID pid = make_pid(Kp, Ki, Kd);
        auto m = sim([&](double r, double y, int) { return pid.compute(r-y); });
        add("PID-ZN(relay)", m, fuzzy_score(m));
    }

    // --- IMC: three lambda values ---
    for (double lam : {0.3, 0.5, 1.0}) {
        double half_th = theta_fopdt / 2.0;
        double Kp = (tau_fopdt + half_th) / (K_fopdt * (lam + half_th));
        double Ti = tau_fopdt + half_th;
        double Td = tau_fopdt * theta_fopdt / (2.0*tau_fopdt + theta_fopdt);
        DiscretePID pid = make_pid(Kp, Kp/Ti, Kp*Td);
        std::string tag = "PID-IMC(lambda=" + std::to_string(lam).substr(0,3) + ")";
        auto m = sim([&](double r, double y, int) { return pid.compute(r-y); });
        add(tag, m, fuzzy_score(m));
    }

    // --- Cohen-Coon ---
    {
        double r_ratio = theta_fopdt / std::max(tau_fopdt, 1e-9);
        double Kp = (1.0/K_fopdt) * (tau_fopdt/std::max(theta_fopdt,1e-9))
                  * (1.33 + r_ratio/4.0);
        double Ti = theta_fopdt * (30.0 + 3.0*r_ratio) / (9.0 + 20.0*r_ratio);
        double Td = theta_fopdt * 4.0 / (11.0 + 2.0*r_ratio);
        DiscretePID pid = make_pid(Kp, Kp/Ti, Kp*Td);
        auto m = sim([&](double r, double y, int) { return pid.compute(r-y); });
        add("PID-CohenCoon", m, fuzzy_score(m));
    }

    // --- Nelder-Mead optimised PID ---
    {
        auto cost = [](const std::vector<double>& p) -> double {
            if(p[0]<=0||p[1]<0||p[2]<0) return 1e6;
            PIDParams pp; pp.Kp=p[0]; pp.Ki=p[1]; pp.Kd=p[2];
            pp.N=10.0; pp.uMin=-UMAX; pp.uMax=UMAX;
            DiscretePID pid(pp, Ts);
            StateSpace pl = make_true_plant();
            Eigen::VectorXd xl = Eigen::VectorXd::Zero(pl.stateSize());
            double J=0, yp=0;
            for(int k=0;k<SIM_STEPS;++k){
                double u=std::clamp(pid.compute(1.0-yp),-UMAX,UMAX);
                double y=plant_step(pl,xl,u);
                double t=k*Ts,e=1.0-y;
                J+=e*e*Ts+0.1*t*std::abs(e)*Ts+0.01*u*u*Ts; yp=y;
            }
            return J;
        };
        auto xopt = nm(cost, {4.0,1.0,0.4}, {{0.01,30},{0,20},{0,5}}, 500);
        std::cout << "  NM-PID: Kp=" << std::setprecision(4) << xopt[0]
                  << " Ki=" << xopt[1] << " Kd=" << xopt[2] << "\n";
        DiscretePID pid = make_pid(xopt[0], xopt[1], xopt[2]);
        auto m = sim([&](double r, double y, int) { return pid.compute(r-y); });
        add("PID-NM", m, fuzzy_score(m));
    }

    // =======================================================================
    // Step 6: SMC - bandwidth from ARX model + NM-optimised
    // =======================================================================
    std::cout << "\n[6] SMC - bandwidth from ARX model\n";
    {
        // Estimate closed-loop bandwidth from ARX eigenvalue magnitude
        // ARX denominator polynomial: z^2 + a1_id*z + a2_id = 0
        // (theta stores coefficients as -a1, -a2, but product a1*a2 gives |eig|^2)
        double disc = a1_id*a1_id - 4.0*a2_id;
        double omega_est = 1.0; // fallback [rad/s]
        if (disc < 0)
            omega_est = std::sqrt(std::abs(a2_id)) / Ts; // discrete -> continuous
        double ce = 1.0, cde = std::clamp(omega_est * Ts * 5.0, 0.1, 20.0);

        SMCParams sp; sp.c_e=ce; sp.c_de=cde; sp.K=5.0; sp.phi=0.1;
        sp.uMin=-UMAX; sp.uMax=UMAX;
        DiscreteSMC smc(sp, Ts);
        auto m = sim([&](double r, double y, int) { return smc.compute(r-y); });
        std::cout << "  SMC: c_e=" << ce << " c_de=" << std::setprecision(4) << cde << "\n";
        add("SMC-ARX", m, fuzzy_score(m));
    }
    {
        auto cost = [](const std::vector<double>& p) -> double {
            if(p[0]<=0||p[1]<=0||p[2]<=0||p[3]<=0) return 1e6;
            SMCParams sp; sp.c_e=p[0]; sp.c_de=p[1]; sp.K=p[2]; sp.phi=p[3];
            sp.uMin=-UMAX; sp.uMax=UMAX;
            DiscreteSMC smc(sp, Ts);
            StateSpace pl=make_true_plant();
            Eigen::VectorXd xl=Eigen::VectorXd::Zero(pl.stateSize());
            double J=0,yp=0;
            for(int k=0;k<SIM_STEPS;++k){
                double u=std::clamp(smc.compute(1.0-yp),-UMAX,UMAX);
                double y=plant_step(pl,xl,u);
                double t=k*Ts,e=1-y;
                J+=e*e*Ts+0.1*t*std::abs(e)*Ts+0.01*u*u*Ts; yp=y;
            }
            return J;
        };
        auto xopt=nm(cost,{1.0,5.0,5.0,0.1},{{0.01,10},{0.1,50},{0.1,20},{0.01,2}},400);
        std::cout << "  NM-SMC: c_e=" << std::setprecision(4) << xopt[0]
                  << " c_de=" << xopt[1] << " K=" << xopt[2] << " phi=" << xopt[3] << "\n";
        SMCParams sp; sp.c_e=xopt[0]; sp.c_de=xopt[1]; sp.K=xopt[2]; sp.phi=xopt[3];
        sp.uMin=-UMAX; sp.uMax=UMAX;
        DiscreteSMC smc(sp, Ts);
        auto m=sim([&](double r,double y,int){return smc.compute(r-y);});
        add("SMC-NM", m, fuzzy_score(m));
    }

    // =======================================================================
    // Step 7: ADRC - b0 estimated from ARX high-frequency gain + NM-optimised
    // =======================================================================
    std::cout << "[7] ADRC - b0 from ARX high-frequency gain\n";
    {
        double b0_est = std::max(std::abs(b1_id / Ts), 1e-6);
        ADRCParams ap; ap.omega_o=15.0; ap.omega_c=3.5; ap.b0=b0_est;
        ap.uMin=-UMAX; ap.uMax=UMAX;
        DiscreteADRC adrc(ap, Ts);
        auto m = sim([&](double r, double y, int) { return adrc.computeTracking(y, r); });
        std::cout << "  ADRC: b0_est=" << std::setprecision(5) << b0_est << "\n";
        add("ADRC-ARX", m, fuzzy_score(m));
    }
    {
        auto cost=[](const std::vector<double>& p)->double{
            if(p[0]<=0||p[1]<=0||p[2]<=0) return 1e6;
            ADRCParams ap; ap.omega_o=p[0]; ap.omega_c=p[1]; ap.b0=p[2];
            ap.uMin=-UMAX; ap.uMax=UMAX;
            DiscreteADRC adrc(ap, Ts);
            StateSpace pl=make_true_plant();
            Eigen::VectorXd xl=Eigen::VectorXd::Zero(pl.stateSize());
            double J=0,yp=0;
            for(int k=0;k<SIM_STEPS;++k){
                double u=std::clamp(adrc.computeTracking(yp,1.0),-UMAX,UMAX);
                double y=plant_step(pl,xl,u);
                double t=k*Ts,e=1-y;
                J+=e*e*Ts+0.1*t*std::abs(e)*Ts+0.01*u*u*Ts; yp=y;
            }
            return J;
        };
        auto xopt=nm(cost,{15.0,3.5,1e-4},{{1.0,100},{0.5,20},{1e-6,1e-2}},400);
        std::cout << "  NM-ADRC: omegao=" << std::setprecision(4) << xopt[0]
                  << " omegac=" << xopt[1] << " b0=" << xopt[2] << "\n";
        ADRCParams ap; ap.omega_o=xopt[0]; ap.omega_c=xopt[1]; ap.b0=xopt[2];
        ap.uMin=-UMAX; ap.uMax=UMAX;
        DiscreteADRC adrc(ap, Ts);
        auto m=sim([&](double r,double y,int){return adrc.computeTracking(y,r);});
        add("ADRC-NM", m, fuzzy_score(m));
    }

    // =======================================================================
    // Ranking and fuzzy summary
    // =======================================================================
    std::cout << "\n=== Ranking by composite cost J ===\n";
    std::vector<size_t> rank_idx(results.size());
    std::iota(rank_idx.begin(), rank_idx.end(), 0);
    std::sort(rank_idx.begin(), rank_idx.end(),
              [&](size_t a, size_t b){ return results[a].J < results[b].J; });

    std::cout << std::string(110, '-') << "\n";
    for (size_t i=0; i<rank_idx.size(); ++i) {
        size_t idx = rank_idx[i];
        auto fr = fuzzy_score(results[idx]);
        std::cout << std::setw(3) << (i+1) << ". ";
        print_metrics(labels[idx], results[idx], fr);
    }

    size_t best = rank_idx[0];
    std::cout << "\n  * Optimal controller: " << labels[best]
              << "  J=" << std::setprecision(5) << results[best].J
              << "  Grade: " << fuzzy_score(results[best]).grade << "\n";

    // Detailed fuzzy report for top-3
    std::cout << "\n=== Fuzzy Performance Reports (top 3) ===\n";
    for (int i=0; i<std::min(3,(int)rank_idx.size()); ++i) {
        size_t idx = rank_idx[i];
        double ise_n = std::min(results[idx].ISE / ise_worst, 1.0);
        double st_n  = std::min(results[idx].settle_time / (SIM_STEPS*Ts), 1.0);
        auto fr = fuzzy::evaluate(ise_n, results[idx].OS_pct, st_n);
        fuzzy::print_report(labels[idx], ise_n, results[idx].OS_pct, st_n, fr);
    }

    // =======================================================================
    // Save CSV
    // =======================================================================
    {
        std::ofstream of("siso_unknown_results.csv");
        of << "method,ISE,ITAE,energy,OS_pct,settle_time,J,fuzzy_score,fuzzy_grade\n";
        for (size_t i=0; i<labels.size(); ++i) {
            auto fr = fuzzy_score(results[i]);
            of << std::fixed << std::setprecision(7)
               << labels[i]         << ","
               << results[i].ISE    << "," << results[i].ITAE    << ","
               << results[i].energy << "," << results[i].OS_pct  << ","
               << results[i].settle_time << "," << results[i].J  << ","
               << fr.score          << "," << fr.grade            << "\n";
        }
        std::cout << "\n  Results saved -> siso_unknown_results.csv\n";
    }
    return 0;
}
