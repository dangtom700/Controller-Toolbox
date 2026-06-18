// ============================================================
// toolbox_examples.cpp  - 12 worked examples for ctrl::
//
// Each example is a standalone function that exercises a
// specific part of the toolbox, runs a simulation loop,
// writes a CSV file, and prints a short console table.
//
// Visualisation: load the CSV files in Python / pandas / matplotlib.
//
// Build: add to case-study/CMakeLists.txt
//   add_executable(toolbox_examples toolbox_examples.cpp)
//   target_link_libraries(toolbox_examples PRIVATE controller_toolbox)
//   target_compile_features(toolbox_examples PRIVATE cxx_std_17)
//
// Dependencies: Eigen 3.4, C++17, ControllerToolbox (lib/)
// ============================================================

#include <ControllerToolbox.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <random>
#include <string>
#include <algorithm>
#include <vector>

// ============================================================
// Shared plant definitions
// ============================================================

// Continuous second-order underdamped SISO plant used in most SISO examples:
//   G_c(s) = omegan^2 / (s^2 + 2zetaomegans + omegan^2),  omegan=1 rad/s, zeta=0.5
//
// Ac = [[0, 1], [-omegan^2, -2zetaomegan]]
// Bc = [[0], [omegan^2]]
// Cc = [[1, 0]]
// Dc = [[0]]
static ctrl::StateSpace makeContinuousPlant()
{
    const double wn = 1.0, zeta = 0.5;
    Eigen::MatrixXd Ac(2, 2), Bc(2, 1), Cc(1, 2), Dc(1, 1);
    Ac << 0,  1,
          -wn*wn, -2*zeta*wn;
    Bc << 0, wn*wn;
    Cc << 1, 0;
    Dc << 0;
    return ctrl::StateSpace(Ac, Bc, Cc, Dc, 0.0); // Ts = 0 -> continuous
}

// Minimal nonlinear boiler step (continuous dynamics, forward-Euler, Ts=1s).
// x = [drum_pressure, electric_power, water_level], u = [u1, u2, u3].
static Eigen::VectorXd boilerStep(const Eigen::VectorXd &x,
                                  const Eigen::VectorXd &u,
                                  double Ts)
{
    double x1 = x(0), x2 = x(1), x3 = x(2);
    double u1 = u(0), u2 = u(1), u3 = u(2);
    double x1_98 = std::pow(std::max(x1, 1e-3), 9.0 / 8.0);

    Eigen::VectorXd dx(3);
    dx(0) = -0.0018 * u2 * x1_98 + 0.9 * u1 - 0.15 * u3;
    dx(1) = (0.073 * u2 - 0.016) * x1_98 - 0.1 * x2;
    dx(2) = (141.0 * u3 - (1.1 * u2 - 0.19) * x1) / 85.0;
    return x + Ts * dx;
}

// Nonlinear boiler output function y = h(x, u).
static Eigen::VectorXd boilerOutput(const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &u)
{
    double x1 = x(0), x2 = x(1), x3 = x(2);
    double u1 = u(0), u2 = u(1), u3 = u(2);
    Eigen::VectorXd y(3);
    y(0) = x1;
    y(1) = x2;
    double acs = ((1.0 - 0.001538 * x3) * 0.8 * x1 - 25.6) /
                 (x3 * (1.0394 - 0.0012304 * x1));
    double qe  = (0.854 * u2 - 0.147) * x1 + 45.59 * u1 - 2.514 * u3 - 2.096;
    y(2) = 0.05 * (0.13073 * x3 + 100.0 * acs + qe / 9.0 - 67.975);
    return y;
}

// Boiler operating point B (medium load)
static const double OP_B_X1 = 97.2,  OP_B_X2 = 50.5,  OP_B_X3 = 469.51;
static const double OP_B_U1 = 0.27049, OP_B_U2 = 0.62082, OP_B_U3 = 0.33979;

// Forward-Euler linearisation of boiler at given operating point -> double-precision ctrl::StateSpace
static ctrl::StateSpace linearizeBoilerFE(double x1, double x2, double x3,
                                          double u2, double Ts)
{
    double x1_18 = std::pow(x1, 1.0 / 8.0);
    double x1_98 = std::pow(x1, 9.0 / 8.0);

    Eigen::MatrixXd Ac(3, 3), Bc(3, 3), Cc(3, 3), Dc(3, 3);
    Ac.setZero(); Bc.setZero(); Cc.setZero(); Dc.setZero();

    Ac(0, 0) = -0.0018 * u2 * (9.0 / 8.0) * x1_18;
    Ac(1, 0) = (0.073 * u2 - 0.016) * (9.0 / 8.0) * x1_18;
    Ac(1, 1) = -0.1;
    Ac(2, 0) = -(1.1 * u2 - 0.19) / 85.0;

    Bc(0, 0) = 0.9; Bc(0, 1) = -0.0018 * x1_98; Bc(0, 2) = -0.15;
    Bc(1, 1) = 0.073 * x1_98;
    Bc(2, 1) = -1.1 * x1 / 85.0; Bc(2, 2) = 141.0 / 85.0;

    double numer = (1.0 - 0.001538 * x3) * 0.8 * x1 - 25.6;
    double denom = x3 * (1.0394 - 0.0012304 * x1);
    double denom2 = denom * denom;
    double dn_dx1 = 0.8 - 0.0012304 * x3;
    double dn_dx3 = -0.0012304 * x1;
    double dd_dx1 = -0.0012304 * x3;
    double dd_dx3 = 1.0394 - 0.0012304 * x1;
    double dacs_dx1 = (dn_dx1 * denom - numer * dd_dx1) / denom2;
    double dacs_dx3 = (dn_dx3 * denom - numer * dd_dx3) / denom2;

    Cc(0, 0) = 1.0;
    Cc(1, 1) = 1.0;
    Cc(2, 0) = 0.05 * (100.0 * dacs_dx1 + (0.854 * u2 - 0.147) / 9.0);
    Cc(2, 2) = 0.05 * (0.13073 + 100.0 * dacs_dx3);

    // Forward-Euler discrete: Ad = I + Ts*Ac, Bd = Ts*Bc
    Eigen::MatrixXd Ad = Eigen::Matrix3d::Identity() + Ts * Ac;
    Eigen::MatrixXd Bd = Ts * Bc;
    return ctrl::StateSpace(Ad, Bd, Cc, Dc, Ts);
}

// ============================================================
// Example 1: PID tuned via relay auto-tuner (Astrom-Hagglund)
// ============================================================
//
// Chain: c2d(ZOH) -> RelayAutoTuner -> PIDParams(TyreusLuyben) -> DiscretePID
//        -> step response -> MetricsAnalyzer
//
// Demonstrates the typical commissioning workflow for an unknown SISO plant.
void ex01_relay_autotuner()
{
    std::cout << "\n=== Ex01: PID via Relay Auto-Tuner ===\n";
    const double Ts = 0.1;

    ctrl::StateSpace plant = ctrl::c2d(makeContinuousPlant(), Ts, ctrl::C2dMethod::ZOH);

    // -- Relay test ----------------------------------------------------------
    ctrl::RelayTunerConfig cfg;
    cfg.relayAmplitude = 1.0;
    cfg.hysteresis     = 0.01;
    cfg.cyclesRequired = 3;
    ctrl::RelayAutoTuner tuner(cfg, Ts);

    Eigen::VectorXd x  = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd uv(1);
    double y = 0.0;

    while (!tuner.isDone())
    {
        double u = tuner.step(y);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);
    }

    std::cout << "Ku = " << tuner.ultimateGain()
              << "  Tu = " << tuner.ultimatePeriod() << " s\n";

    ctrl::PIDParams pp = tuner.computePIDParams(ctrl::PIDTuningRule::TyreusLuyben);
    pp.uMin = -2.0;
    pp.uMax =  2.0;
    std::cout << "PID: Kp=" << pp.Kp << " Ki=" << pp.Ki << " Kd=" << pp.Kd << "\n";

    // -- Closed-loop step response --------------------------------------------
    ctrl::DiscretePID pid(pp, Ts);
    x.setZero();
    const double r = 1.0;
    const int N    = 400;

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex01_relay_pid.csv");
    f << "Time,y,u,e\n";

    std::vector<double> t_vec, y_vec;
    for (int k = 0; k < N; ++k)
    {
        double e = r - y;
        double u = pid.compute(e);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);

        f << std::fixed << std::setprecision(4)
          << k * Ts << "," << y << "," << u << "," << e << "\n";
        t_vec.push_back(k * Ts);
        y_vec.push_back(y);
    }

    ctrl::TimeDomainMetrics m = ctrl::MetricsAnalyzer::calculate(t_vec, y_vec, r);
    std::cout << "Rise: " << m.riseTime << "s  Settle: " << m.settlingTime
              << "s  Overshoot: " << m.peakOvershoot << "%  SSE: " << m.steadyStateError << "\n";
    std::cout << "-> ex01_relay_pid.csv\n";
}

// ============================================================
// Example 2: Continuous-to-Discrete - ZOH vs Tustin
// ============================================================
//
// Same second-order continuous plant discretized at Ts=0.5 s with both
// ZOH (exact for piecewise-constant inputs) and Tustin (frequency-preserving).
// At this coarser sample rate the two methods produce noticeably different
// step responses, illustrating the frequency-axis warping of the Tustin method.
void ex02_c2d_comparison()
{
    std::cout << "\n=== Ex02: c2d - ZOH vs Tustin ===\n";
    const double Ts = 0.5; // coarse grid to show method difference

    ctrl::StateSpace sys_c   = makeContinuousPlant();
    ctrl::StateSpace sys_zoh = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);
    ctrl::StateSpace sys_tus = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::Tustin);

    std::cout << "ZOH Ad:\n"    << sys_zoh.A << "\nZOH Bd:\n" << sys_zoh.B << "\n";
    std::cout << "Tustin Ad:\n" << sys_tus.A << "\nTustin Bd:\n" << sys_tus.B << "\n";

    const int N = 80;
    Eigen::VectorXd x_z = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd x_t = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd uv(1); uv << 1.0; // unit step

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex02_c2d.csv");
    f << "Time,y_zoh,y_tustin\n";
    std::cout << std::setw(8) << "t[s]"
              << std::setw(12) << "y_zoh"
              << std::setw(12) << "y_tustin\n"
              << std::string(32, '-') << "\n";

    for (int k = 0; k < N; ++k)
    {
        double yz = ctrl::ssStep(sys_zoh, x_z, uv)(0);
        double yt = ctrl::ssStep(sys_tus, x_t, uv)(0);

        f << std::fixed << std::setprecision(4)
          << k * Ts << "," << yz << "," << yt << "\n";

        if (k % 8 == 0)
            std::cout << std::setw(8)  << k * Ts
                      << std::setw(12) << yz
                      << std::setw(12) << yt << "\n";
    }
    std::cout << "-> ex02_c2d.csv\n";
}

// ============================================================
// Example 3: RLS Online Identification - demonstrating reset()
// ============================================================
//
// A true ARX(2,2) plant changes parameters at k=200.
// The new reset() correctly restores P to P0_scale*I so the filter
// re-converges immediately instead of staying locked near the old estimate.
void ex03_rls_identification()
{
    std::cout << "\n=== Ex03: RLS Online Identification (with reset) ===\n";
    const double Ts = 0.1;
    std::mt19937 rng(0);
    std::normal_distribution<double> noise(0.0, 0.02);

    // True ARX: y[k] = -a1*y[k-1] - a2*y[k-2] + b1*u[k-1] + b2*u[k-2] + e[k]
    // Phase A (k=0..199):   a1=-1.50, a2=0.70, b1=0.50, b2=0.30
    // Phase B (k=200..399): a1=-1.30, a2=0.60, b1=0.70, b2=0.25  (plant shift)
    auto trueARX = [](double a1, double a2, double b1, double b2,
                      double y1, double y2, double u1, double u2, double e)
    {
        return -a1 * y1 - a2 * y2 + b1 * u1 + b2 * u2 + e;
    };

    ctrl::RecursiveLeastSquares rls(2, 2, Ts, 0.97, 1e4);

    double y = 0, y1 = 0, y2 = 0, u1 = 0, u2 = 0;
    int prbs_state = 1;

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex03_rls.csv");
    f << "Time,y,u,e_pred,theta_a1,theta_a2,theta_b1,theta_b2,phase\n";
    std::cout << std::setw(6)  << "k"
              << std::setw(9)  << "a1_est"
              << std::setw(9)  << "a2_est"
              << std::setw(9)  << "b1_est"
              << std::setw(9)  << "b2_est"
              << std::setw(7)  << "phase\n"
              << std::string(49, '-') << "\n";

    for (int k = 0; k < 400; ++k)
    {
        // PRBS input: toggle every 7 steps
        if (k % 7 == 0) prbs_state = -prbs_state;
        double u = static_cast<double>(prbs_state);

        // Switch plant and reset RLS at k=200
        int phase = (k < 200) ? 1 : 2;
        if (k == 200)
        {
            rls.reset();
            std::cout << " [plant shift at k=200 - rls.reset() called]\n";
        }

        // Simulate true plant
        if (phase == 1)
            y = trueARX(-1.50, 0.70, 0.50, 0.30, y1, y2, u1, u2, noise(rng));
        else
            y = trueARX(-1.30, 0.60, 0.70, 0.25, y1, y2, u1, u2, noise(rng));

        double e = rls.update(y, u);
        const Eigen::VectorXd &th = rls.params();

        f << std::fixed << std::setprecision(4)
          << k * Ts << "," << y << "," << u << "," << e << ","
          << th(0) << "," << th(1) << "," << th(2) << "," << th(3) << ","
          << phase << "\n";

        if (k % 50 == 0)
            std::cout << std::setw(6)  << k
                      << std::setw(9)  << th(0)
                      << std::setw(9)  << th(1)
                      << std::setw(9)  << th(2)
                      << std::setw(9)  << th(3)
                      << std::setw(7)  << phase << "\n";

        y2 = y1; y1 = y;
        u2 = u1; u1 = u;
    }
    std::cout << "-> ex03_rls.csv\n";
}

// ============================================================
// Example 4: N4SID Subspace ID + suggestOrder()
// ============================================================
//
// Collects 600 steps of PRBS-excited I/O data from the ZOH-discretized
// second-order plant, then runs n4sid() with i=15 block rows.
// suggestOrder() automatically picks n=2 (the true order) from the
// singular-value elbow, eliminating manual inspection.
void ex04_n4sid_order_selection()
{
    std::cout << "\n=== Ex04: N4SID + suggestOrder ===\n";
    const double Ts = 0.1;
    const int    N  = 600;

    ctrl::StateSpace plant = ctrl::c2d(makeContinuousPlant(), Ts, ctrl::C2dMethod::ZOH);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());

    // Collect PRBS data
    Eigen::MatrixXd Y(1, N), U(1, N);
    int prbs = 1;
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.01);

    for (int k = 0; k < N; ++k)
    {
        if (k % 11 == 0) prbs = -prbs;
        double u = static_cast<double>(prbs);
        Eigen::VectorXd uv(1); uv << u;
        double y = ctrl::ssStep(plant, x, uv)(0) + noise(rng);
        Y(0, k) = y;
        U(0, k) = u;
    }

    // -- N4SID identification -------------------------------------------------
    const int i_horizon = 15;
    ctrl::SubspaceIDResult res = ctrl::n4sid(Y, U, 2, i_horizon, Ts);

    std::cout << "Singular values: ";
    for (int i = 0; i < std::min((int)res.singularValues.size(), 6); ++i)
        std::cout << std::fixed << std::setprecision(3) << res.singularValues(i) << "  ";
    std::cout << "...\n";

    int suggested = ctrl::suggestOrder(res.singularValues, 0.01, 8);
    std::cout << "suggestOrder() -> n = " << suggested
              << "  (true order = 2)\n";

    if (!res.success)
    {
        std::cerr << "N4SID failed: " << res.message << "\n";
        return;
    }

    // -- Validate: one-step-ahead prediction on fresh data -------------------
    x.setZero();
    Eigen::VectorXd x_id = Eigen::VectorXd::Zero(res.model->stateSize());
    double rmse = 0.0;
    prbs = 1;

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex04_n4sid.csv");
    f << "Time,y_true,y_id,error\n";

    for (int k = 0; k < 200; ++k)
    {
        if (k % 11 == 0) prbs = -prbs;
        Eigen::VectorXd uv(1); uv << static_cast<double>(prbs);
        double y_true = ctrl::ssStep(plant, x, uv)(0);
        double y_id   = ctrl::ssStep(*res.model, x_id, uv)(0);
        double err    = y_true - y_id;
        rmse += err * err;
        f << k * Ts << "," << y_true << "," << y_id << "," << err << "\n";
    }
    rmse = std::sqrt(rmse / 200.0);
    std::cout << "One-step-ahead RMSE: " << rmse << "\n";
    std::cout << "-> ex04_n4sid.csv\n";
}

// ============================================================
// Example 5: MPC with hard rate constraints (gradient-projection QP)
// ============================================================
//
// Boiler linearized at op_B.  Tight valve-rate limits duMin/duMax are
// passed directly; the new gradient-projection QP respects them over
// the full control horizon rather than just clamping the first move.
// The CSV lets you verify ||Deltau[k]||_inf <= duMax for every step.
void ex05_mpc_constrained()
{
    std::cout << "\n=== Ex05: MPC with Hard Rate Constraints (GP-QP) ===\n";
    const double Ts = 1.0;

    ctrl::StateSpace plant = linearizeBoilerFE(OP_B_X1, OP_B_X2, OP_B_X3,
                                               OP_B_U2, Ts);

    ctrl::MPCParams mp;
    mp.Np    = 15;
    mp.Nc    = 4;
    mp.rho_y = 1.0;
    mp.rho_u = 0.5;
    // Tight rate limit: valve can move <= 0.007 per step (fuel valve limit)
    mp.duMin = -0.007;
    mp.duMax =  0.007;
    mp.uMin  = -0.5;   // absolute deviation from op (valves in [0,1] -> max +/-0.5)
    mp.uMax  =  0.5;
    mp.qpMaxIter = 300;
    mp.qpTol     = 1e-9;

    ctrl::DiscreteMPC mpc(plant, mp);

    Eigen::VectorXd dx(3);  dx << 5.0, 3.0, -10.0; // same kick as case study
    const Eigen::VectorXd r_ref = Eigen::VectorXd::Zero(3);

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex05_mpc_constrained.csv");
    f << "Time,dx1,dx2,dx3,du1,du2,du3,norm_du\n";
    std::cout << std::setw(6) << "k"
              << std::setw(10) << "dx1"
              << std::setw(10) << "dx2"
              << std::setw(10) << "dx3"
              << std::setw(10) << "||du||inf\n"
              << std::string(46, '-') << "\n";

    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(3);

    for (int k = 0; k <= 200; ++k)
    {
        Eigen::VectorXd u = mpc.computeRef(dx, r_ref);
        Eigen::VectorXd du = u - u_prev;
        double norm_du = du.cwiseAbs().maxCoeff();

        f << k * Ts << ","
          << dx(0) << "," << dx(1) << "," << dx(2) << ","
          << du(0) << "," << du(1) << "," << du(2) << ","
          << norm_du << "\n";

        if (k % 20 == 0)
            std::cout << std::setw(6)  << k
                      << std::fixed << std::setprecision(4)
                      << std::setw(10) << dx(0)
                      << std::setw(10) << dx(1)
                      << std::setw(10) << dx(2)
                      << std::setw(10) << norm_du << "\n";

        ctrl::ssStep(plant, dx, u);
        u_prev = u;
    }
    std::cout << "-> ex05_mpc_constrained.csv\n";
}

// ============================================================
// Example 6: Adaptive GPC - RLS feeds plant model hot-swap
// ============================================================
//
// A SISO second-order ARX plant slowly drifts.  RLS identifies it online;
// every 60 steps the GPC's internal model is hot-swapped via setPlant(),
// reducing the prediction error as the adaptation catches up.
//
// This is the self-tuning regulator pattern: RLS + GPC(setPlant()).
void ex06_adaptive_gpc()
{
    std::cout << "\n=== Ex06: Adaptive GPC (RLS + GPC hot-swap) ===\n";
    const double Ts = 0.1;

    // Initial linear plant (discrete via c2d ZOH)
    ctrl::StateSpace plant_true = ctrl::c2d(makeContinuousPlant(), Ts,
                                            ctrl::C2dMethod::ZOH);

    // RLS identifies a 2nd-order model
    ctrl::RecursiveLeastSquares rls(2, 2, Ts, 0.97, 1e4);

    // GPC: start with approximate model (same as true initially)
    ctrl::GPCParams gp;
    gp.Np    = 12;
    gp.Nu    = 3;
    gp.rho_y = 1.0;
    gp.rho_u = 0.3;
    gp.alpha = 0.2; // soft reference trajectory
    gp.uMin  = -3.0;
    gp.uMax  =  3.0;

    ctrl::GeneralizedPredictiveController gpc(plant_true, gp);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant_true.stateSize());
    std::mt19937 rng(3);
    std::normal_distribution<double> noise(0.0, 0.02);

    const double r = 1.0;
    double y = 0.0, u_prev_scalar = 0.0;
    int swap_count = 0;

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex06_adaptive_gpc.csv");
    f << "Time,y,u,r,e,swapped\n";
    std::cout << std::setw(6)  << "k"
              << std::setw(10) << "y"
              << std::setw(10) << "u"
              << std::setw(8)  << "swapped\n"
              << std::string(34, '-') << "\n";

    for (int k = 0; k < 600; ++k)
    {
        // Slow plant drift: omegan increases linearly with time
        // (we approximate by occasionally regenerating the true plant)
        if (k > 0 && k % 150 == 0)
        {
            double wn_drift = 1.0 + 0.3 * (k / 150.0);
            Eigen::MatrixXd Ac(2,2), Bc(2,1), Cc(1,2), Dc(1,1);
            Ac << 0, 1, -wn_drift*wn_drift, -1.0;
            Bc << 0, wn_drift*wn_drift;
            Cc << 1, 0; Dc << 0;
            plant_true = ctrl::c2d(ctrl::StateSpace(Ac, Bc, Cc, Dc, 0.0),
                                   Ts, ctrl::C2dMethod::ZOH);
            x = Eigen::VectorXd::Zero(plant_true.stateSize());
        }

        // Every 60 steps: if RLS has converged, hot-swap GPC model
        bool swapped = false;
        if (k > 30 && k % 60 == 0)
        {
            ctrl::StateSpace rls_model = rls.toStateSpace();
            gpc.setPlant(rls_model);
            swapped = true;
            ++swap_count;
        }

        double e = r - y;
        double u = gpc.computeRef(y, r);

        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(plant_true, x, uv)(0) + noise(rng);
        rls.update(y, u);

        f << std::fixed << std::setprecision(4)
          << k * Ts << "," << y << "," << u << "," << r << "," << e << ","
          << (swapped ? 1 : 0) << "\n";

        if (k % 60 == 0)
            std::cout << std::setw(6)  << k
                      << std::setw(10) << y
                      << std::setw(10) << u
                      << std::setw(8)  << (swapped ? "YES" : "-") << "\n";

        u_prev_scalar = u;
    }
    std::cout << "Total model swaps: " << swap_count << "\n";
    std::cout << "-> ex06_adaptive_gpc.csv\n";
}

// ============================================================
// Example 7: Extended Kalman Filter on the nonlinear boiler
// ============================================================
//
// Full nonlinear boiler with process and measurement noise.
// EKF uses numerical Jacobians (via EKF::numericalJacobian) so no
// analytical differentiation is required - just define f() and h().
// Runs alongside an LQR that uses the true state; the EKF estimate
// approximates the true state despite nonlinear dynamics.
void ex07_ekf_nonlinear()
{
    std::cout << "\n=== Ex07: EKF on Nonlinear Boiler ===\n";
    const double Ts = 1.0;

    // EKF state and measurement dimensions
    const int n = 3, p = 3;

    // Nonlinear process function (forward-Euler integration of boiler)
    ctrl::StateFunc boilerF = [Ts](const Eigen::VectorXd &x,
                                   const Eigen::VectorXd &u)
    { return boilerStep(x, u, Ts); };

    // Nonlinear measurement function
    ctrl::MeasFunc boilerH = [](const Eigen::VectorXd &x,
                                const Eigen::VectorXd &u)
    { return boilerOutput(x, u); };

    // Numerical Jacobians (central difference, eps=1e-5)
    ctrl::JacobianFn Fjac = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    {
        return ctrl::ExtendedKalmanFilter::numericalJacobian(
            [&](const Eigen::VectorXd &xx) { return boilerF(xx, u); }, x);
    };
    ctrl::JacobianFn Hjac = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    {
        return ctrl::ExtendedKalmanFilter::numericalJacobian(
            [&](const Eigen::VectorXd &xx) { return boilerH(xx, u); }, x);
    };

    Eigen::MatrixXd Q = 1e-4 * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd R = Eigen::Matrix3d::Zero();
    R(0,0) = 0.25; R(1,1) = 1.0; R(2,2) = 25.0;

    ctrl::ExtendedKalmanFilter ekf(n, p, boilerF, boilerH, Fjac, Hjac, Q, R, Ts);

    // Initial conditions: operating point B + perturbation
    Eigen::VectorXd x_true(3);
    x_true << OP_B_X1 + 5.0, OP_B_X2 + 3.0, OP_B_X3 - 10.0;
    Eigen::VectorXd x_est(3);
    x_est << OP_B_X1, OP_B_X2, OP_B_X3; // EKF starts at op point
    ekf.setState(x_est);

    const Eigen::VectorXd u0(Eigen::Vector3d{OP_B_U1, OP_B_U2, OP_B_U3});

    // LQR stabilises around op_B using the linearized model
    ctrl::StateSpace lin_plant = linearizeBoilerFE(OP_B_X1, OP_B_X2, OP_B_X3,
                                                   OP_B_U2, Ts);
    Eigen::VectorXd xmax(3), umax(3);
    xmax << 5.0, 10.0, 1.0;
    umax << 0.3,  0.3, 0.1;
    ctrl::LQRParams lqr_p = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
    ctrl::DiscreteLQR lqr(lin_plant, lqr_p);

    std::mt19937 rng(42);
    std::normal_distribution<double> pn(0.0, 0.01);
    std::normal_distribution<double> mn1(0.0, 0.5), mn2(0.0, 1.0), mn3(0.0, 5.0);

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex07_ekf.csv");
    f << "Time,x1,x2,x3,x1_est,x2_est,x3_est,err1,err2,err3\n";
    std::cout << std::setw(6)  << "k"
              << std::setw(10) << "x1"
              << std::setw(10) << "x1_est"
              << std::setw(10) << "err1\n"
              << std::string(36, '-') << "\n";

    Eigen::VectorXd u = u0;
    double rmse1 = 0, rmse2 = 0, rmse3 = 0;

    for (int k = 0; k <= 400; ++k)
    {
        // LQR control on deviation from op
        Eigen::VectorXd dx = x_true - Eigen::Vector3d(OP_B_X1, OP_B_X2, OP_B_X3);
        Eigen::VectorXd du = lqr.compute(dx, Eigen::VectorXd::Zero(3));
        u = (u0 + du).cwiseMax(0.0).cwiseMin(1.0);

        // Noisy measurement
        Eigen::VectorXd y = boilerOutput(x_true, u);
        y(0) += mn1(rng); y(1) += mn2(rng); y(2) += mn3(rng);

        // EKF update
        ekf.step(y, u);
        const Eigen::VectorXd &xe = ekf.state();

        double e1 = x_true(0) - xe(0);
        double e2 = x_true(1) - xe(1);
        double e3 = x_true(2) - xe(2);
        rmse1 += e1*e1; rmse2 += e2*e2; rmse3 += e3*e3;

        f << k*Ts << ","
          << x_true(0) << "," << x_true(1) << "," << x_true(2) << ","
          << xe(0) << "," << xe(1) << "," << xe(2) << ","
          << e1 << "," << e2 << "," << e3 << "\n";

        if (k % 40 == 0)
            std::cout << std::setw(6)  << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(10) << x_true(0)
                      << std::setw(10) << xe(0)
                      << std::setw(10) << e1 << "\n";

        // Advance true plant with process noise
        Eigen::VectorXd w(3); w << pn(rng), pn(rng), pn(rng);
        x_true = boilerStep(x_true, u, Ts) + w;
    }
    int steps = 401;
    std::cout << "RMS errors: x1=" << std::sqrt(rmse1/steps)
              << " x2=" << std::sqrt(rmse2/steps)
              << " x3=" << std::sqrt(rmse3/steps) << "\n";
    std::cout << "-> ex07_ekf.csv\n";
}

// ============================================================
// Example 8: UKF vs EKF - estimation accuracy comparison
// ============================================================
//
// Identical boiler setup, both filters run in parallel on the same
// noisy measurement stream.  UKF requires no Jacobians; EKF uses the
// numerical Jacobians from Ex07.  CSV records both for comparison.
void ex08_ukf_vs_ekf()
{
    std::cout << "\n=== Ex08: UKF vs EKF Comparison ===\n";
    const double Ts = 1.0;
    const int n = 3, p = 3;

    auto boilerF_fn = [Ts](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    { return boilerStep(x, u, Ts); };
    auto boilerH_fn = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    { return boilerOutput(x, u); };

    ctrl::JacobianFn Fjac = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    {
        return ctrl::ExtendedKalmanFilter::numericalJacobian(
            [&](const Eigen::VectorXd &xx) { return boilerF_fn(xx, u); }, x);
    };
    ctrl::JacobianFn Hjac = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u)
    {
        return ctrl::ExtendedKalmanFilter::numericalJacobian(
            [&](const Eigen::VectorXd &xx) { return boilerH_fn(xx, u); }, x);
    };

    Eigen::MatrixXd Q = 1e-4 * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd R = Eigen::Matrix3d::Zero();
    R(0,0) = 0.25; R(1,1) = 1.0; R(2,2) = 25.0;

    ctrl::ExtendedKalmanFilter ekf(n, p, boilerF_fn, boilerH_fn, Fjac, Hjac, Q, R, Ts);
    ctrl::UnscentedKalmanFilter ukf(n, p, boilerF_fn, boilerH_fn, Q, R, Ts,
                                    Eigen::MatrixXd(), 1e-3, 2.0, 0.0);

    Eigen::VectorXd x0(3); x0 << OP_B_X1, OP_B_X2, OP_B_X3;
    ekf.setState(x0);
    ukf.setState(x0);

    Eigen::VectorXd x_true(3);
    x_true << OP_B_X1 + 5.0, OP_B_X2 + 3.0, OP_B_X3 - 10.0;
    const Eigen::VectorXd u(Eigen::Vector3d{OP_B_U1, OP_B_U2, OP_B_U3});

    std::mt19937 rng(17);
    std::normal_distribution<double> pn(0.0, 0.01);
    std::normal_distribution<double> mn1(0.0, 0.5), mn2(0.0, 1.0), mn3(0.0, 5.0);

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex08_ukf_vs_ekf.csv");
    f << "Time,x1_true,x1_ekf,x1_ukf,err_ekf,err_ukf\n";
    std::cout << std::setw(6)  << "k"
              << std::setw(12) << "err_ekf_x1"
              << std::setw(12) << "err_ukf_x1\n"
              << std::string(30, '-') << "\n";

    double rmse_ekf1 = 0, rmse_ukf1 = 0;

    for (int k = 0; k <= 400; ++k)
    {
        Eigen::VectorXd y = boilerOutput(x_true, u);
        y(0) += mn1(rng); y(1) += mn2(rng); y(2) += mn3(rng);

        ekf.step(y, u);
        ukf.step(y, u);

        double ee = x_true(0) - ekf.state()(0);
        double eu = x_true(0) - ukf.state()(0);
        rmse_ekf1 += ee*ee; rmse_ukf1 += eu*eu;

        f << k*Ts << ","
          << x_true(0) << "," << ekf.state()(0) << "," << ukf.state()(0) << ","
          << ee << "," << eu << "\n";

        if (k % 50 == 0)
            std::cout << std::setw(6)  << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(12) << ee
                      << std::setw(12) << eu << "\n";

        Eigen::VectorXd w(3); w << pn(rng), pn(rng), pn(rng);
        x_true = boilerStep(x_true, u, Ts) + w;
    }
    int steps = 401;
    std::cout << "RMSE x1 - EKF: " << std::sqrt(rmse_ekf1/steps)
              << "  UKF: " << std::sqrt(rmse_ukf1/steps) << "\n";
    std::cout << "-> ex08_ukf_vs_ekf.csv\n";
}

// ============================================================
// Example 9: ADRC disturbance rejection vs PID
// ============================================================
//
// Double-integrator plant: G_c(s) = b0/s^2 discretized at Ts=0.05 s.
// A step disturbance (magnitude 2) is injected at t=1 s.
// ADRC's ESO estimates and cancels it.  PID (ZN-tuned) is shown
// for comparison; its steady-state error persists for much longer.
void ex09_adrc_disturbance()
{
    std::cout << "\n=== Ex09: ADRC Disturbance Rejection vs PID ===\n";
    const double Ts = 0.05;
    const double b0 = 1.0; // double-integrator gain

    // Discrete double integrator: x[k+1] = Ax + Bu,  y = Cx
    Eigen::MatrixXd Ad(2,2), Bd(2,1), Cd(1,2), Dd(1,1);
    Ad << 1, Ts, 0, 1;
    Bd << 0.5*Ts*Ts, Ts;
    Cd << 1, 0; Dd << 0;
    ctrl::StateSpace plant(Ad, Bd, Cd, Dd, Ts);

    // ADRC: omega_c = 3 rad/s, omega_o = 20 rad/s (3-10x rule)
    ctrl::ADRCParams ap;
    ap.omega_c = 3.0;
    ap.omega_o = 20.0;
    ap.b0      = b0;
    ap.uMin    = -10.0;
    ap.uMax    =  10.0;
    ctrl::DiscreteADRC adrc(ap, Ts);

    // PID tuned via ZN with manually chosen Ku/Tu for double integrator
    ctrl::ZieglerNicholsTuner::Input zn{3.5, 2.0};
    ctrl::PIDParams pp = ctrl::ZieglerNicholsTuner::tuneImpl(zn);
    pp.uMin = -10.0; pp.uMax = 10.0;
    ctrl::DiscretePID pid(pp, Ts);

    Eigen::VectorXd x_adrc = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd x_pid  = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd uv(1);

    const double r    = 1.0;
    const double dist = 2.0; // step disturbance at t=1 s

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex09_adrc.csv");
    f << "Time,y_adrc,y_pid,u_adrc,u_pid,d\n";
    std::cout << std::setw(7)  << "t[s]"
              << std::setw(10) << "y_adrc"
              << std::setw(10) << "y_pid"
              << std::setw(10) << "z3(ESO)\n"
              << std::string(37, '-') << "\n";

    const int N = 300;
    for (int k = 0; k < N; ++k)
    {
        double t = k * Ts;
        double d = (t >= 1.0) ? dist : 0.0;

        double y_adrc = (plant.C * x_adrc)(0);
        double y_pid  = (plant.C * x_pid)(0);

        double u_adrc = adrc.computeTracking(y_adrc, r);
        double u_pid  = pid.compute(r - y_pid);

        // Inject disturbance as additive input perturbation
        uv << u_adrc + d;
        ctrl::ssStep(plant, x_adrc, uv);
        uv << u_pid + d;
        ctrl::ssStep(plant, x_pid, uv);

        f << t << "," << y_adrc << "," << y_pid << ","
          << u_adrc << "," << u_pid << "," << d << "\n";

        if (k % 30 == 0)
            std::cout << std::setw(7)  << t
                      << std::fixed << std::setprecision(4)
                      << std::setw(10) << y_adrc
                      << std::setw(10) << y_pid
                      << std::setw(10) << adrc.esoState()(2) << "\n";
    }
    std::cout << "-> ex09_adrc.csv\n";
}

// ============================================================
// Example 10: Smith Predictor for integer dead-time compensation
// ============================================================
//
// The second-order plant is augmented with a 5-step dead time (external
// delay buffer).  A plain PID converges slowly and may go unstable at
// aggressive gains; wrapping the same PID in a Smith Predictor restores
// near-nominal performance by cancelling the delay in the feedback path.
void ex10_smith_predictor()
{
    std::cout << "\n=== Ex10: Smith Predictor vs Plain PID (5-step dead time) ===\n";
    const double Ts      = 0.1;
    const int    d_steps = 5; // dead-time samples

    ctrl::StateSpace delay_free = ctrl::c2d(makeContinuousPlant(), Ts,
                                            ctrl::C2dMethod::ZOH);

    // Conservative PID (TyreusLuyben via relay test result)
    ctrl::PIDParams pp;
    pp.Kp = 1.2; pp.Ki = 0.25; pp.Kd = 0.0;
    pp.N  = 10.0; pp.uMin = -3.0; pp.uMax = 3.0;

    // -- Smith Predictor ------------------------------------------------------
    auto inner_sp  = std::make_shared<ctrl::DiscretePID>(pp, Ts);
    ctrl::SmithPredictor sp(inner_sp, delay_free, d_steps);

    // -- Plain PID ------------------------------------------------------------
    ctrl::DiscretePID pid_plain(pp, Ts);

    // Two plant instances (both with external delay buffer)
    Eigen::VectorXd x_sp  = Eigen::VectorXd::Zero(delay_free.stateSize());
    Eigen::VectorXd x_pid = Eigen::VectorXd::Zero(delay_free.stateSize());
    std::vector<double> delay_sp(d_steps, 0.0);
    std::vector<double> delay_pid(d_steps, 0.0);
    int head_sp = 0, head_pid = 0;

    auto delayedOutput = [&](std::vector<double> &buf, int &head,
                             Eigen::VectorXd &x, double u_now) -> double
    {
        Eigen::VectorXd uv(1); uv << u_now;
        double y_now  = ctrl::ssStep(delay_free, x, uv)(0);
        double y_del  = buf[head];        // output from d_steps ago
        buf[head]     = y_now;
        head          = (head + 1) % d_steps;
        return y_del;
    };

    const double r = 1.0;
    const int    N = 500;

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex10_smith.csv");
    f << "Time,y_sp,y_pid\n";
    std::cout << std::setw(7)  << "t[s]"
              << std::setw(10) << "y_smith"
              << std::setw(10) << "y_plain\n"
              << std::string(27, '-') << "\n";

    for (int k = 0; k < N; ++k)
    {
        double t = k * Ts;

        // Smith predictor sees delayed y but compensates internally
        double y_sp_del  = delay_sp[head_sp];
        double u_sp      = sp.compute(r - y_sp_del);
        double y_sp_true = delayedOutput(delay_sp, head_sp, x_sp, u_sp);

        // Plain PID sees delayed y directly
        double y_pid_del = delay_pid[head_pid];
        double u_pid     = pid_plain.compute(r - y_pid_del);
        double y_pid_true = delayedOutput(delay_pid, head_pid, x_pid, u_pid);

        f << t << "," << y_sp_true << "," << y_pid_true << "\n";

        if (k % 50 == 0)
            std::cout << std::setw(7)  << t
                      << std::fixed << std::setprecision(4)
                      << std::setw(10) << y_sp_true
                      << std::setw(10) << y_pid_true << "\n";
    }
    std::cout << "-> ex10_smith.csv\n";
}

// ============================================================
// Example 11: Repetitive Controller - periodic disturbance cancellation
// ============================================================
//
// A sinusoidal disturbance with period T=5 s (N=50 steps at Ts=0.1 s)
// is injected at the plant output.  The Repetitive Controller wraps a
// base PID and accumulates a correction over successive periods.
// After ~2 periods the periodic error is suppressed by >90 %.
void ex11_repetitive_control()
{
    std::cout << "\n=== Ex11: Repetitive Controller (periodic disturbance) ===\n";
    const double Ts     = 0.1;
    const int    N_per  = 50; // disturbance period in samples (T = 5 s)
    const double T_dist = N_per * Ts;

    ctrl::StateSpace plant = ctrl::c2d(makeContinuousPlant(), Ts,
                                       ctrl::C2dMethod::ZOH);

    // Base PID
    ctrl::PIDParams pp;
    pp.Kp = 1.5; pp.Ki = 0.3; pp.Kd = 0.1;
    pp.N  = 8.0; pp.uMin = -4.0; pp.uMax = 4.0;

    // -- Repetitive Controller wrapping the PID -------------------------------
    ctrl::RepetitiveParams rcp;
    rcp.periodSteps = N_per;
    rcp.Krc         = 0.4;  // learning gain
    rcp.Q           = 0.97; // forgetting factor for model-error robustness
    rcp.uMin        = -4.0;
    rcp.uMax        =  4.0;
    auto base_pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);
    ctrl::RepetitiveController rc(base_pid, rcp, Ts);

    // -- Plain PID for comparison ---------------------------------------------
    ctrl::DiscretePID pid_plain(pp, Ts);

    Eigen::VectorXd x_rc  = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd x_pid = Eigen::VectorXd::Zero(plant.stateSize());
    Eigen::VectorXd uv(1);

    const double r = 1.0;
    const int    N = 4 * N_per; // 4 disturbance periods

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex11_rc.csv");
    f << "Time,e_rc,e_pid,u_rc,u_pid,d\n";
    std::cout << std::setw(7)  << "t[s]"
              << std::setw(10) << "e_rc"
              << std::setw(10) << "e_pid"
              << std::setw(10) << "correction\n"
              << std::string(37, '-') << "\n";

    for (int k = 0; k < N; ++k)
    {
        double t = k * Ts;
        double d = 0.5 * std::sin(2.0 * M_PI * t / T_dist); // periodic disturbance

        double y_rc  = (plant.C * x_rc)(0)  + d;
        double y_pid = (plant.C * x_pid)(0) + d;

        double e_rc  = r - y_rc;
        double e_pid = r - y_pid;

        double u_rc  = rc.compute(e_rc);
        double u_pid = pid_plain.compute(e_pid);

        uv << u_rc;  ctrl::ssStep(plant, x_rc,  uv);
        uv << u_pid; ctrl::ssStep(plant, x_pid, uv);

        f << t << "," << e_rc << "," << e_pid << ","
          << u_rc << "," << u_pid << "," << d << "\n";

        if (k % N_per == 0)
            std::cout << std::setw(7)  << t
                      << std::fixed << std::setprecision(4)
                      << std::setw(10) << e_rc
                      << std::setw(10) << e_pid
                      << std::setw(10) << rc.correction() << "\n";
    }
    std::cout << "-> ex11_rc.csv\n";
}

// ============================================================
// Example 12: Supervisory Controller Stack - gain-scheduled switching
// ============================================================
//
// Two PIDs with different tunings live in a Supervisory ControllerStack.
// An activation condition selects between them based on error magnitude:
//   |e| > 0.5  ->  PID_A (Ziegler-Nichols, aggressive - fast large-error pull-in)
//   |e| <= 0.5  ->  PID_B (Tyreus-Luyben, conservative - low overshoot near setpoint)
//
// MetricsAnalyzer runs on the recorded step response to quantify the
// benefit of the gain-scheduled approach vs either fixed tuning alone.
void ex12_supervisory_stack()
{
    std::cout << "\n=== Ex12: Supervisory Stack - gain-scheduled PID ===\n";
    const double Ts = 0.1;

    ctrl::StateSpace plant = ctrl::c2d(makeContinuousPlant(), Ts,
                                       ctrl::C2dMethod::ZOH);

    // Tuning A: Ziegler-Nichols (aggressive, Ku=3.0, Tu=6.3 s for this plant)
    ctrl::ZieglerNicholsTuner::Input zn{3.0, 6.3};
    ctrl::PIDParams pp_A = ctrl::ZieglerNicholsTuner::tuneImpl(zn);
    pp_A.uMin = -3.0; pp_A.uMax = 3.0;

    // Tuning B: Tyreus-Luyben (conservative)
    ctrl::PIDParams pp_B = pp_A;
    pp_B.Kp *= 0.45;  // TL approximation: Kp = Ku/3.1
    pp_B.Ki *= 0.30;
    pp_B.Kd *= 0.50;

    // -- Supervisory stack ----------------------------------------------------
    ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);

    auto pid_A = std::make_shared<ctrl::DiscretePID>(pp_A, Ts);
    auto pid_B = std::make_shared<ctrl::DiscretePID>(pp_B, Ts);

    // PID_A active when error is large
    stack.addController(pid_A, "PID_A", 1.0,
        [](double e, double /*lastU*/) { return std::abs(e) > 0.5; });
    // PID_B active otherwise (no condition = always eligible as fallback)
    stack.addController(pid_B, "PID_B");

    // -- Three comparison runs ------------------------------------------------
    auto runController = [&](ctrl::IController &ctrl_obj,
                             const std::string &label) -> std::vector<double>
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        Eigen::VectorXd uv(1);
        const double r = 1.0;
        std::vector<double> y_hist;

        for (int k = 0; k < 500; ++k)
        {
            double y = (plant.C * x)(0);
            double u = ctrl_obj.compute(r - y);
            uv << u; ctrl::ssStep(plant, x, uv);
            y_hist.push_back(y);
        }
        ctrl_obj.reset();
        return y_hist;
    };

    auto y_stack = runController(stack, "Stack");
    auto y_A     = runController(*pid_A, "PID_A");
    auto y_B     = runController(*pid_B, "PID_B");

    std::ofstream f(std::string(PROJECT_DATA_DIR) + "/ex12_stack.csv");
    f << "Time,y_stack,y_A,y_B\n";
    for (int k = 0; k < 500; ++k)
        f << k*Ts << "," << y_stack[k] << "," << y_A[k] << "," << y_B[k] << "\n";

    std::vector<double> tvec(500);
    for (int i = 0; i < 500; ++i) tvec[i] = i * Ts;

    auto print_metrics = [&](const std::string &name, const std::vector<double> &y)
    {
        auto m = ctrl::MetricsAnalyzer::calculate(tvec, y, 1.0);
        std::cout << std::left << std::setw(10) << name
                  << "  rise=" << std::fixed << std::setprecision(2) << m.riseTime << "s"
                  << "  settle=" << m.settlingTime << "s"
                  << "  OS="    << std::setprecision(1) << m.peakOvershoot << "%"
                  << "  SSE="   << std::setprecision(4) << m.steadyStateError << "\n";
    };

    std::cout << "\nStep-response metrics (r=1):\n";
    print_metrics("Stack",  y_stack);
    print_metrics("PID_A",  y_A);
    print_metrics("PID_B",  y_B);
    std::cout << "-> ex12_stack.csv\n";
}

// ============================================================
// main
// ============================================================
int main()
{
    ex01_relay_autotuner();
    ex02_c2d_comparison();
    ex03_rls_identification();
    ex04_n4sid_order_selection();
    ex05_mpc_constrained();
    ex06_adaptive_gpc();
    ex07_ekf_nonlinear();
    ex08_ukf_vs_ekf();
    ex09_adrc_disturbance();
    ex10_smith_predictor();
    ex11_repetitive_control();
    ex12_supervisory_stack();
    return 0;
}
