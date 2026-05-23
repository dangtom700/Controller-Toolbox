#include <ControllerToolbox.h>
#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <random>

class BoilerTurbine
{
public:
    const float Ts = 1.0f;

    float u1 = 0.5f, u2 = 0.5f, u3 = 0.5f;

    float x1, x2, x3;
    float y1, y2, y3;

    float du1 = 0.0f, du2 = 0.0f, du3 = 0.0f;

    inline void constrain_valve()
    {
        u1 = std::clamp(u1, 0.0f, 1.0f);
        u2 = std::clamp(u2, 0.0f, 1.0f);
        u3 = std::clamp(u3, 0.0f, 1.0f);
    }

    inline void constrain_valve_rate()
    {
        du1 = std::clamp(u1 - u1_prev_, -0.007f, 0.007f);
        du2 = std::clamp(u2 - u2_prev_, -2.0f, 0.02f);
        du3 = std::clamp(u3 - u3_prev_, -0.05f, 0.05f);
        u1 = u1_prev_ + du1;
        u2 = u2_prev_ + du2;
        u3 = u3_prev_ + du3;
        u1_prev_ = u1;
        u2_prev_ = u2;
        u3_prev_ = u3;
    }

    void update()
    {
        float x1_98 = std::pow(x1, 9.0f / 8.0f);

        float dx1 = -0.0018f * u2 * x1_98 + 0.9f * u1 - 0.15f * u3;
        float dx2 = (0.073f * u2 - 0.016f) * x1_98 - 0.1f * x2;
        float dx3 = (141.0f * u3 - (1.1f * u2 - 0.19f) * x1) / 85.0f;

        y1 = x1;
        y2 = x2;
        float acs = ((1.0f - 0.001538f * x3) * 0.8f * x1 - 25.6f) / (x3 * (1.0394f - 0.0012304f * x1));
        float qe = (0.854f * u2 - 0.147f) * x1 + 45.59f * u1 - 2.514f * u3 - 2.096f;
        y3 = 0.05f * (0.13073f * x3 + 100.0f * acs + qe / 9.0f - 67.975f);

        x1 += Ts * dx1;
        x2 += Ts * dx2;
        x3 += Ts * dx3;
    }

private:
    float u1_prev_ = 0.5f, u2_prev_ = 0.5f, u3_prev_ = 0.5f;
};

struct operating_point
{
    float x1, x2, x3; // State
    float u1, u2, u3; // Control input
    float y1, y2, y3; // Output

    // Set y1 = x1, y2=x2
    operating_point(float x1_, float x2_, float x3_, float u1_, float u2_, float u3_, float y3_)
        : x1(x1_), x2(x2_), x3(x3_), u1(u1_), u2(u2_), u3(u3_), y1(x1_), y2(x2_), y3(y3_) {}
};

operating_point op_A = {75.6, 15.3, 508.97, 0.11926, 0.38063, 0.12262, 0.098414};
operating_point op_B = {97.2, 50.5, 469.51, 0.27049, 0.62082, 0.33979, 0.097038};
operating_point op_C = {140, 128, 323.68, 0.59589, 0.89447, 0.78829, 0.09797};

void random_valve(float &u1, float &u2, float &u3)
{
    // Random between change rate
    float du1 = ((float)rand() / RAND_MAX) * 0.014f - 0.007f; // [-0.007, 0.007]
    float du2 = ((float)rand() / RAND_MAX) * 0.04f - 0.02f;   // [-0.02, 0.02]
    float du3 = ((float)rand() / RAND_MAX) * 0.1f - 0.05f;    // [-0.05, 0.05]

    u1 += du1;
    u2 += du2;
    u3 += du3;
}

void plant_model_data()
{
    BoilerTurbine bt;

    // Initial conditions
    bt.x1 = 100.0f; // Drum pressure
    bt.x2 = 50.0f;  // Electric power
    bt.x3 = 20.0f;  // Water level

    // Control inputs
    bt.u1 = 0.5f; // Fuel flow valve position
    bt.u2 = 0.5f; // Steam control valve position
    bt.u3 = 0.5f; // Feedwater flow valve position

    std::ofstream data_file("boiler_turbine_data.csv");
    data_file << "Time,Drum Pressure,Electric Power,Water Level Deviation,u1,u2,u3\n";

    for (int i = 0; i < 100; ++i)
    {
        random_valve(bt.u1, bt.u2, bt.u3);
        bt.constrain_valve_rate();
        bt.constrain_valve();
        bt.update();
        data_file << i * bt.Ts << "," << bt.y1 << "," << bt.y2 << "," << bt.y3 << "," << bt.u1 << "," << bt.u2 << "," << bt.u3 << "\n";
    }

    std::cout << "Boiler-turbine data generated in boiler_turbine_data.csv\n";
}

struct LinearStateSpace
{
    Eigen::Matrix3f Ad, Bd, Cd, Dd;

    void print(const std::string &label) const
    {
        std::cout << "\n=== " << label << " ===\n";
        std::cout << "Ad:\n"
                  << Ad << "\n\n";
        std::cout << "Bd:\n"
                  << Bd << "\n\n";
        std::cout << "Cd:\n"
                  << Cd << "\n\n";
        std::cout << "Dd:\n"
                  << Dd << "\n";
    }
};

LinearStateSpace linearize(const operating_point &op, float Ts = 1.0f)
{
    const float x1 = op.x1, x3 = op.x3;
    const float u2 = op.u2;

    const float x1_18 = std::pow(x1, 1.0f / 8.0f); // x1^(1/8)
    const float x1_98 = std::pow(x1, 9.0f / 8.0f); // x1^(9/8)

    // f1 = -0.0018*u2*x1^(9/8) + 0.9*u1 - 0.15*u3
    // f2 = (0.073*u2 - 0.016)*x1^(9/8) - 0.1*x2
    // f3 = (141*u3 - (1.1*u2 - 0.19)*x1) / 85
    Eigen::Matrix3f Ac = Eigen::Matrix3f::Zero();
    Ac(0, 0) = -0.0018f * u2 * (9.0f / 8.0f) * x1_18;
    Ac(1, 0) = (0.073f * u2 - 0.016f) * (9.0f / 8.0f) * x1_18;
    Ac(1, 1) = -0.1f;
    Ac(2, 0) = -(1.1f * u2 - 0.19f) / 85.0f;

    Eigen::Matrix3f Bc = Eigen::Matrix3f::Zero();
    Bc(0, 0) = 0.9f;
    Bc(0, 1) = -0.0018f * x1_98;
    Bc(0, 2) = -0.15f;
    Bc(1, 1) = 0.073f * x1_98;
    Bc(2, 1) = -1.1f * x1 / 85.0f;
    Bc(2, 2) = 141.0f / 85.0f;

    // acs = numer / denom
    //   numer = (1 - 0.001538*x3)*0.8*x1 - 25.6
    //   denom = x3*(1.0394 - 0.0012304*x1)
    const float numer = (1.0f - 0.001538f * x3) * 0.8f * x1 - 25.6f;
    const float denom = x3 * (1.0394f - 0.0012304f * x1);
    const float denom2 = denom * denom;

    const float dn_dx1 = 0.8f - 0.0012304f * x3;    // dnumer/dx1
    const float dn_dx3 = -0.0012304f * x1;          // dnumer/dx3
    const float dd_dx1 = -0.0012304f * x3;          // ddenom/dx1
    const float dd_dx3 = 1.0394f - 0.0012304f * x1; // ddenom/dx3

    const float dacs_dx1 = (dn_dx1 * denom - numer * dd_dx1) / denom2;
    const float dacs_dx3 = (dn_dx3 * denom - numer * dd_dx3) / denom2;

    // y3 = 0.05*(0.13073*x3 + 100*acs + qe/9 - 67.975)
    // qe = (0.854*u2 - 0.147)*x1 + 45.59*u1 - 2.514*u3 - 2.096
    Eigen::Matrix3f Cc = Eigen::Matrix3f::Zero();
    Cc(0, 0) = 1.0f;
    Cc(1, 1) = 1.0f;
    Cc(2, 0) = 0.05f * (100.0f * dacs_dx1 + (0.854f * u2 - 0.147f) / 9.0f);
    Cc(2, 2) = 0.05f * (0.13073f + 100.0f * dacs_dx3);

    // ── Output feedthrough  dy/du ──────────────────────────────────────────
    Eigen::Matrix3f Dc = Eigen::Matrix3f::Zero();
    Dc(2, 0) = 0.05f * 45.59f / 9.0f;
    Dc(2, 1) = 0.05f * 0.854f * x1 / 9.0f;
    Dc(2, 2) = -0.05f * 2.514f / 9.0f;

    // ── Forward-Euler discretisation: Ad = I + Ts*Ac,  Bd = Ts*Bc ──────────
    LinearStateSpace ss;
    ss.Ad = Eigen::Matrix3f::Identity() + Ts * Ac;
    ss.Bd = Ts * Bc;
    ss.Cd = Cc;
    ss.Dd = Dc;
    return ss;
}

void state_space_at_operating_points()
{
    constexpr float Ts = 1.0f;
    linearize(op_A, Ts).print("A - Low Load    (x1=75.6,   x2=15.3,   x3=508.97)");
    linearize(op_B, Ts).print("B - Medium Load (x1=97.2,   x2=50.5,   x3=469.51)");
    linearize(op_C, Ts).print("C - High Load   (x1=140.0,  x2=128.0,  x3=323.68)");
}

// LQR regulator linearised around op: drives dx -> 0 from an initial perturbation.
// Outputs a console table and writes lqr_op_<label>.csv.
void ss_lqr(const LinearStateSpace &ss, const operating_point &op, const std::string &label, float Ts = 0.5f, int update_freq = 10, int sim_steps = 500)
{
    // Cast float matrices to double for the toolbox API
    ctrl::StateSpace plant(
        ss.Ad.cast<double>(), ss.Bd.cast<double>(),
        ss.Cd.cast<double>(), ss.Dd.cast<double>(), static_cast<double>(Ts));

    std::cout << "\n=== LQR @ Operating Point " << label << " ===\n";
    std::cout << "Ad:\n"
              << plant.A << "\nBd:\n"
              << plant.B << "\n\n";

    // ── Bryson weights ───────────────────────────────────────────────────────
    // xmax: max acceptable deviation from equilibrium
    //   x1 (drum pressure)  +/-5,  x2 (electric power) +/-10,  x3 (water level) +/-1
    // umax: max valve deviation from operating point (all valves \in [0,1])
    Eigen::VectorXd xmax(3), umax(3);
    xmax << 5.0, 10.0, 1.0;
    umax << 0.3, 0.3, 0.1;
    ctrl::LQRParams lqr_p = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);

    std::cout << "Q =\n"
              << lqr_p.Q << "\nR =\n"
              << lqr_p.R << "\n\n";

    ctrl::DiscreteLQR lqr(plant, lqr_p);
    std::cout << "K =\n"
              << lqr.gainMatrix() << "\n\n";

    // ── Initial perturbation and reference ───────────────────────────────────
    // Regulate dx -> 0 (return to operating point)
    const Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dx(3);
    dx << 5.0, 3.0, -10.0; // small kick: +5 pressure, +3 power, -10 level

    // ── CSV and console output ───────────────────────────────────────────────
    const std::string fname = "lqr_op_" + label + ".csv";
    std::ofstream f(fname);
    f << "Time,dx1,dx2,dx3,u1,u2,u3\n";

    int spacing[] = {10, 12, 8, 10, 10, 10, 10, 10, 8};
    int sum_spacing = 0;
    for (size_t i = 0; i < sizeof(spacing) / sizeof(spacing[0]); ++i)
        sum_spacing += spacing[i];

    std::cout << std::setw(spacing[0]) << "k"
              << std::setw(spacing[1]) << "t[s]"
              << std::setw(spacing[2]) << "dx1"
              << std::setw(spacing[3]) << "dx2"
              << std::setw(spacing[4]) << "dx3"
              << std::setw(spacing[5]) << "u1"
              << std::setw(spacing[6]) << "u2"
              << std::setw(spacing[7]) << "u3" << "\n";
    std::cout << std::string(sum_spacing, '-') << "\n";

    const Eigen::Vector3d u0(op.u1, op.u2, op.u3);

    for (int k = 0; k <= sim_steps; ++k)
    {
        Eigen::VectorXd du = lqr.compute(dx, x_ref);

        // Clamp each absolute valve position to [0, 1]
        for (int i = 0; i < 3; ++i)
        {
            double u_abs = std::max(0.0, std::min(1.0, u0(i) + du(i)));
            du(i) = u_abs - u0(i);
        }

        f << k * Ts << ","
          << dx(0) << "," << dx(1) << "," << dx(2) << ","
          << u0(0) + du(0) << "," << u0(1) + du(1) << "," << u0(2) + du(2) << "\n";

        if (k % update_freq == 0)
            std::cout << std::setw(spacing[0]) << k
                      << std::fixed << std::setprecision(4)
                      << std::setw(spacing[1]) << k * Ts
                      << std::setw(spacing[2]) << dx(0)
                      << std::setw(spacing[3]) << dx(1)
                      << std::setw(spacing[4]) << dx(2)
                      << std::setw(spacing[5]) << u0(0) + du(0)
                      << std::setw(spacing[6]) << u0(1) + du(1)
                      << std::setw(spacing[7]) << u0(2) + du(2) << "\n";

        ctrl::ssStep(plant, dx, du);
    }

    std::cout << "LQR data written to " << fname << "\n";
}

void ss_mpc(const LinearStateSpace &ss, const operating_point &op, const std::string &label, float Ts = 1.0f, int update_freq = 10, int sim_steps = 2000)
{
    ctrl::StateSpace plant(
        ss.Ad.cast<double>(), ss.Bd.cast<double>(),
        ss.Cd.cast<double>(), ss.Dd.cast<double>(), static_cast<double>(Ts));

    std::cout << "\n=== MPC @ Operating Point " << label << " ===\n";

    // ── Horizon recommendation ───────────────────────────────────────────────
    // x3 (water level) has an integrating mode: Ad(2,2) = 1 + Ts*df3/dx3 = 1.
    // estimateSettlingTime() hits its maxSteps cap and returns ts = 5000 s,
    // which inflates Np to 5000 and makes the 5001x5001 Hessian rebuild each step.
    // Cap Np/Nc to physically meaningful values for a 1 s sample system.
    auto rec = ctrl::MPCHorizonTuner::recommend(plant, static_cast<double>(Ts));
    std::cout << "Tuner raw:  Np=" << rec.Np << "  Nc=" << rec.Nc
              << "  t_settle~" << std::fixed << std::setprecision(1)
              << rec.estimatedSettlingTime << "s  (capped: Np=20, Nc=5)\n\n";

    ctrl::MPCParams mp;
    mp.Np = std::min(rec.Np, 20);
    mp.Nc = std::min(rec.Nc, 5);
    mp.rho_y = rec.rho_y;
    mp.rho_u = rec.rho_u;
    mp.uMin = -0.5; // du bounds in valve-position space ([0,1] enforced below)
    mp.uMax = 0.5;
    ctrl::DiscreteMPC mpc(plant, mp);

    // ── Initial perturbation and reference ───────────────────────────────────
    // Drive dx -> 0 (return to operating point from a small kick)
    const Eigen::VectorXd r_ref = Eigen::VectorXd::Zero(plant.outputSize());
    Eigen::VectorXd dx(3);
    dx << 5.0, 3.0, -10.0; // +5 pressure, +3 power, -10 level (same as LQR)

    // ── CSV output ───────────────────────────────────────────────────────────
    const std::string fname = "mpc_op_" + label + ".csv";
    std::ofstream f(fname);
    f << "Time,dx1,dx2,dx3,u1,u2,u3\n";

    int spacing[] = {10, 12, 8, 10, 10, 10, 10, 10};
    int sum_spacing = 0;
    for (size_t i = 0; i < sizeof(spacing) / sizeof(spacing[0]); ++i)
        sum_spacing += spacing[i];

    std::cout << std::setw(spacing[0]) << "k"
              << std::setw(spacing[1]) << "t[s]"
              << std::setw(spacing[2]) << "dx1"
              << std::setw(spacing[3]) << "dx2"
              << std::setw(spacing[4]) << "dx3"
              << std::setw(spacing[5]) << "u1"
              << std::setw(spacing[6]) << "u2"
              << std::setw(spacing[7]) << "u3" << "\n";
    std::cout << std::string(sum_spacing, '-') << "\n";

    const Eigen::Vector3d u0(op.u1, op.u2, op.u3);

    for (int k = 0; k <= sim_steps; ++k)
    {
        // MIMO MPC: compute optimal du given current dx and zero output reference
        Eigen::VectorXd du = mpc.computeRef(dx, r_ref);

        // Clamp each absolute valve position to [0, 1]
        for (int i = 0; i < 3; ++i)
        {
            double u_abs = std::max(0.0, std::min(1.0, u0(i) + du(i)));
            du(i) = u_abs - u0(i);
        }

        f << k * Ts << ","
          << dx(0) << "," << dx(1) << "," << dx(2) << ","
          << u0(0) + du(0) << "," << u0(1) + du(1) << "," << u0(2) + du(2) << "\n";

        if (k % update_freq == 0)
            std::cout << std::setw(spacing[0]) << k
                      << std::fixed << std::setprecision(4)
                      << std::setw(spacing[1]) << k * Ts
                      << std::setw(spacing[2]) << dx(0)
                      << std::setw(spacing[3]) << dx(1)
                      << std::setw(spacing[4]) << dx(2)
                      << std::setw(spacing[5]) << u0(0) + du(0)
                      << std::setw(spacing[6]) << u0(1) + du(1)
                      << std::setw(spacing[7]) << u0(2) + du(2) << "\n";

        ctrl::ssStep(plant, dx, du);
    }

    std::cout << "MPC data written to " << fname << "\n";
}

void ss_lqg_kalman(const LinearStateSpace &ss, const operating_point &op, const std::string &label, float Ts = 1.0f, int update_freq = 10, int sim_steps = 2000)
{
    ctrl::StateSpace plant(
        ss.Ad.cast<double>(), ss.Bd.cast<double>(),
        ss.Cd.cast<double>(), ss.Dd.cast<double>(), static_cast<double>(Ts));

    std::cout << "\n=== LQG/Kalman @ Operating Point " << label << " ===\n";

    Eigen::VectorXd xmax(3), umax(3);
    xmax << 5.0, 10.0, 1.0;
    umax << 0.3, 0.3, 0.1;
    ctrl::LQRParams lqr_p = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);

    Eigen::Matrix3d Qn = 1e-4 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rn = Eigen::Matrix3d::Zero();
    Rn(0, 0) = 0.25; // y1 sigma=0.5
    Rn(1, 1) = 1.0;  // y2 sigma=1.0
    Rn(2, 2) = 25.0; // y3 sigma=5.0

    ctrl::DiscreteLQG lqg(plant, lqr_p, Qn, Rn);
    ctrl::DiscreteLQR lqr_ideal(plant, lqr_p);

    std::mt19937 rng(42);
    std::normal_distribution<double> meas_n1(0.0, 0.5);
    std::normal_distribution<double> meas_n2(0.0, 1.0);
    std::normal_distribution<double> meas_n3(0.0, 5.0);

    // Nonlinear plant: operating point + same initial kick as LQR/MPC
    BoilerTurbine bt;
    bt.x1 = op.x1 + 5.0f;
    bt.x2 = op.x2 + 3.0f;
    bt.x3 = op.x3 - 10.0f;
    bt.u1 = op.u1;
    bt.u2 = op.u2;
    bt.u3 = op.u3;

    // Compute initial outputs before the first update() call
    bt.y1 = bt.x1;
    bt.y2 = bt.x2;
    {
        float acs = ((1.0f - 0.001538f * bt.x3) * 0.8f * bt.x1 - 25.6f) /
                    (bt.x3 * (1.0394f - 0.0012304f * bt.x1));
        float qe = (0.854f * bt.u2 - 0.147f) * bt.x1 + 45.59f * bt.u1 - 2.514f * bt.u3 - 2.096f;
        bt.y3 = 0.05f * (0.13073f * bt.x3 + 100.0f * acs + qe / 9.0f - 67.975f);
    }

    // Ideal LQR: linear deviation model for reference comparison
    Eigen::VectorXd dx_ideal(3);
    dx_ideal << 5.0, 3.0, -10.0;
    const Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd du_lqg(3);
    du_lqg.setZero();
    Eigen::VectorXd du_ideal(3);
    du_ideal.setZero();
    const Eigen::Vector3d u0(op.u1, op.u2, op.u3);

    // Previous valve positions for du tracking
    float u1_prev = op.u1, u2_prev = op.u2, u3_prev = op.u3;

    const std::string fname = "lqg_op_" + label + ".csv";
    std::ofstream f(fname);
    f << "Time,y1,y2,y3,u1,u2,u3,du1,du2,du3,dx1_est,dx2_est,dx3_est,u1_ideal,u2_ideal,u3_ideal\n";

    int spacing[] = {8, 10, 9, 9, 9, 8, 8, 8, 9, 9, 9};
    int sum_spacing = 0;
    for (size_t i = 0; i < sizeof(spacing) / sizeof(spacing[0]); ++i)
        sum_spacing += spacing[i];

    std::cout << std::setw(spacing[0]) << "k"
              << std::setw(spacing[1]) << "t[s]"
              << std::setw(spacing[2]) << "y1"
              << std::setw(spacing[3]) << "y2"
              << std::setw(spacing[4]) << "y3"
              << std::setw(spacing[5]) << "u1"
              << std::setw(spacing[6]) << "u2"
              << std::setw(spacing[7]) << "u3"
              << std::setw(spacing[8]) << "dx1_est"
              << std::setw(spacing[9]) << "dx2_est"
              << std::setw(spacing[10]) << "dx3_est" << "\n";
    std::cout << std::string(sum_spacing, '-') << "\n";

    for (int k = 0; k <= sim_steps; ++k)
    {
        // Noisy output measurement from nonlinear plant in deviation space
        Eigen::VectorXd dy_noisy(3);
        dy_noisy << static_cast<double>(bt.y1 - op.y1) + meas_n1(rng),
            static_cast<double>(bt.y2 - op.y2) + meas_n2(rng),
            static_cast<double>(bt.y3 - op.y3) + meas_n3(rng);

        // LQG: Kalman filter update + LQR on estimated state
        du_lqg = lqg.step(dy_noisy, du_lqg, x_ref);

        // Clamp absolute valve positions to [0, 1]
        for (int i = 0; i < 3; ++i)
        {
            double u_abs = std::max(0.0, std::min(1.0, u0(i) + du_lqg(i)));
            du_lqg(i) = u_abs - u0(i);
        }

        // Apply control to nonlinear plant
        bt.u1 = static_cast<float>(u0(0) + du_lqg(0));
        bt.u2 = static_cast<float>(u0(1) + du_lqg(1));
        bt.u3 = static_cast<float>(u0(2) + du_lqg(2));

        // Valve rate of change
        const float du1 = bt.u1 - u1_prev;
        const float du2 = bt.u2 - u2_prev;
        const float du3 = bt.u3 - u3_prev;
        u1_prev = bt.u1;
        u2_prev = bt.u2;
        u3_prev = bt.u3;

        // Ideal LQR (linear deviation space, no noise)
        du_ideal = lqr_ideal.compute(dx_ideal, x_ref);
        for (int i = 0; i < 3; ++i)
        {
            double u_abs = std::max(0.0, std::min(1.0, u0(i) + du_ideal(i)));
            du_ideal(i) = u_abs - u0(i);
        }

        const Eigen::VectorXd x_est = lqg.stateEstimate();

        f << k * Ts << ","
          << bt.y1 << "," << bt.y2 << "," << bt.y3 << ","
          << bt.u1 << "," << bt.u2 << "," << bt.u3 << ","
          << du1 << "," << du2 << "," << du3 << ","
          << x_est(0) << "," << x_est(1) << "," << x_est(2) << ","
          << u0(0) + du_ideal(0) << "," << u0(1) + du_ideal(1) << "," << u0(2) + du_ideal(2) << "\n";

        if (k % update_freq == 0)
            std::cout << std::setw(spacing[0]) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(spacing[1]) << k * Ts
                      << std::setw(spacing[2]) << bt.y1
                      << std::setw(spacing[3]) << bt.y2
                      << std::setw(spacing[4]) << bt.y3
                      << std::setw(spacing[5]) << bt.u1
                      << std::setw(spacing[6]) << bt.u2
                      << std::setw(spacing[7]) << bt.u3
                      << std::setw(spacing[8]) << x_est(0)
                      << std::setw(spacing[9]) << x_est(1)
                      << std::setw(spacing[10]) << x_est(2) << "\n";

        // Advance nonlinear plant and ideal linear reference
        bt.update();
        dx_ideal = plant.A * dx_ideal + plant.B * du_ideal;
    }

    std::cout << "LQG data written to " << fname << "\n";
}

void ss_PID(const LinearStateSpace &ss, const operating_point &op, const std::string &label, float Ts = 1.0f, int update_freq = 10, int sim_steps = 2000)
{
    (void)ss;
    std::cout << "\n=== PID @ Operating Point " << label << " ===\n";

    // Three decentralised SISO PIDs: y1->u1, y2->u2, y3->u3
    // Output is a deviation du; absolute valve = u0 + du, clamped to [0,1]
    ctrl::PIDParams p;
    p.Kp = 0.05;
    p.Ki = 0.002;
    p.Kd = 0.02;
    p.N = 5.0;
    p.uMin = -0.5;
    p.uMax = 0.5;

    ctrl::DiscretePID pid1(p, Ts), pid2(p, Ts), pid3(p, Ts);

    BoilerTurbine bt;
    bt.x1 = op.x1 + 5.0f;
    bt.x2 = op.x2 + 3.0f;
    bt.x3 = op.x3 - 10.0f;
    bt.u1 = op.u1;
    bt.u2 = op.u2;
    bt.u3 = op.u3;

    bt.y1 = bt.x1;
    bt.y2 = bt.x2;
    {
        float acs = ((1.0f - 0.001538f * bt.x3) * 0.8f * bt.x1 - 25.6f) /
                    (bt.x3 * (1.0394f - 0.0012304f * bt.x1));
        float qe = (0.854f * bt.u2 - 0.147f) * bt.x1 + 45.59f * bt.u1 - 2.514f * bt.u3 - 2.096f;
        bt.y3 = 0.05f * (0.13073f * bt.x3 + 100.0f * acs + qe / 9.0f - 67.975f);
    }

    float u1_prev = op.u1, u2_prev = op.u2, u3_prev = op.u3;

    const std::string fname = "pid_op_" + label + ".csv";
    std::ofstream f(fname);
    f << "Time,y1,y2,y3,u1,u2,u3,du1,du2,du3\n";

    int spacing[] = {8, 10, 9, 9, 9, 8, 8, 8};
    int sum_spacing = 0;
    for (size_t i = 0; i < sizeof(spacing) / sizeof(spacing[0]); ++i)
        sum_spacing += spacing[i];

    std::cout << std::setw(spacing[0]) << "k"
              << std::setw(spacing[1]) << "t[s]"
              << std::setw(spacing[2]) << "y1"
              << std::setw(spacing[3]) << "y2"
              << std::setw(spacing[4]) << "y3"
              << std::setw(spacing[5]) << "u1"
              << std::setw(spacing[6]) << "u2"
              << std::setw(spacing[7]) << "u3" << "\n";
    std::cout << std::string(sum_spacing, '-') << "\n";

    for (int k = 0; k <= sim_steps; ++k)
    {
        // Regulate dy -> 0: error = -(y - y_op)
        bt.u1 = std::clamp(static_cast<float>(op.u1 + pid1.compute(-(bt.y1 - op.y1))), 0.0f, 1.0f);
        bt.u2 = std::clamp(static_cast<float>(op.u2 + pid2.compute(-(bt.y2 - op.y2))), 0.0f, 1.0f);
        bt.u3 = std::clamp(static_cast<float>(op.u3 + pid3.compute(-(bt.y3 - op.y3))), 0.0f, 1.0f);

        const float du1 = bt.u1 - u1_prev;
        const float du2 = bt.u2 - u2_prev;
        const float du3 = bt.u3 - u3_prev;
        u1_prev = bt.u1;
        u2_prev = bt.u2;
        u3_prev = bt.u3;

        f << k * Ts << ","
          << bt.y1 << "," << bt.y2 << "," << bt.y3 << ","
          << bt.u1 << "," << bt.u2 << "," << bt.u3 << ","
          << du1 << "," << du2 << "," << du3 << "\n";

        if (k % update_freq == 0)
            std::cout << std::setw(spacing[0]) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(spacing[1]) << k * Ts
                      << std::setw(spacing[2]) << bt.y1
                      << std::setw(spacing[3]) << bt.y2
                      << std::setw(spacing[4]) << bt.y3
                      << std::setw(spacing[5]) << bt.u1
                      << std::setw(spacing[6]) << bt.u2
                      << std::setw(spacing[7]) << bt.u3 << "\n";

        bt.update();
    }

    std::cout << "PID data written to " << fname << "\n";
}

void ss_SMC(const LinearStateSpace &ss, const operating_point &op, const std::string &label, float Ts = 1.0f, int update_freq = 10, int sim_steps = 2000)
{
    (void)ss;
    std::cout << "\n=== SMC @ Operating Point " << label << " ===\n";

    // Three decentralised SISO SMCs: y1->u1, y2->u2, y3->u3
    ctrl::SMCParams p;
    p.c_e = 1.0;
    p.c_de = 0.2;
    p.K = 0.05;
    p.phi = 0.3;
    p.uMin = -0.5;
    p.uMax = 0.5;

    ctrl::DiscreteSMC smc1(p, Ts), smc2(p, Ts), smc3(p, Ts);

    BoilerTurbine bt;
    bt.x1 = op.x1 + 5.0f;
    bt.x2 = op.x2 + 3.0f;
    bt.x3 = op.x3 - 10.0f;
    bt.u1 = op.u1;
    bt.u2 = op.u2;
    bt.u3 = op.u3;

    bt.y1 = bt.x1;
    bt.y2 = bt.x2;
    {
        float acs = ((1.0f - 0.001538f * bt.x3) * 0.8f * bt.x1 - 25.6f) /
                    (bt.x3 * (1.0394f - 0.0012304f * bt.x1));
        float qe = (0.854f * bt.u2 - 0.147f) * bt.x1 + 45.59f * bt.u1 - 2.514f * bt.u3 - 2.096f;
        bt.y3 = 0.05f * (0.13073f * bt.x3 + 100.0f * acs + qe / 9.0f - 67.975f);
    }

    float u1_prev = op.u1, u2_prev = op.u2, u3_prev = op.u3;

    const std::string fname = "smc_op_" + label + ".csv";
    std::ofstream f(fname);
    f << "Time,y1,y2,y3,u1,u2,u3,du1,du2,du3\n";

    int spacing[] = {8, 10, 9, 9, 9, 8, 8, 8};
    int sum_spacing = 0;
    for (size_t i = 0; i < sizeof(spacing) / sizeof(spacing[0]); ++i)
        sum_spacing += spacing[i];

    std::cout << std::setw(spacing[0]) << "k"
              << std::setw(spacing[1]) << "t[s]"
              << std::setw(spacing[2]) << "y1"
              << std::setw(spacing[3]) << "y2"
              << std::setw(spacing[4]) << "y3"
              << std::setw(spacing[5]) << "u1"
              << std::setw(spacing[6]) << "u2"
              << std::setw(spacing[7]) << "u3" << "\n";
    std::cout << std::string(sum_spacing, '-') << "\n";

    for (int k = 0; k <= sim_steps; ++k)
    {
        bt.u1 = std::clamp(static_cast<float>(op.u1 + smc1.compute(-(bt.y1 - op.y1))), 0.0f, 1.0f);
        bt.u2 = std::clamp(static_cast<float>(op.u2 + smc2.compute(-(bt.y2 - op.y2))), 0.0f, 1.0f);
        bt.u3 = std::clamp(static_cast<float>(op.u3 + smc3.compute(-(bt.y3 - op.y3))), 0.0f, 1.0f);

        const float du1 = bt.u1 - u1_prev;
        const float du2 = bt.u2 - u2_prev;
        const float du3 = bt.u3 - u3_prev;
        u1_prev = bt.u1;
        u2_prev = bt.u2;
        u3_prev = bt.u3;

        f << k * Ts << ","
          << bt.y1 << "," << bt.y2 << "," << bt.y3 << ","
          << bt.u1 << "," << bt.u2 << "," << bt.u3 << ","
          << du1 << "," << du2 << "," << du3 << "\n";

        if (k % update_freq == 0)
            std::cout << std::setw(spacing[0]) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(spacing[1]) << k * Ts
                      << std::setw(spacing[2]) << bt.y1
                      << std::setw(spacing[3]) << bt.y2
                      << std::setw(spacing[4]) << bt.y3
                      << std::setw(spacing[5]) << bt.u1
                      << std::setw(spacing[6]) << bt.u2
                      << std::setw(spacing[7]) << bt.u3 << "\n";

        bt.update();
    }

    std::cout << "SMC data written to " << fname << "\n";
}

void ss_extremum_seeker(const LinearStateSpace &ss, const operating_point &op, const std::string &label, float Ts = 1.0f, int update_freq = 10, int sim_steps = 2000)
{
    (void)ss;
    std::cout << "\n=== Extremum Seeker @ Operating Point " << label << " ===\n";

    // ESC optimises u3 (feedwater flow) to maximise y3 (boiler efficiency).
    // u1, u2 held at operating point; ESC returns absolute u3 = theta + dither.
    // Starts at operating point (no kick): ESC assumes quasi-static plant behaviour.
    ctrl::ExtremumSeekerParams p;
    p.perturbAmp = 0.005; // small dither on feedwater valve
    p.perturbFreq = 0.02; // Hz — 1 cycle per 50 s at Ts=1
    p.lpfCutoff = 0.005;  // Hz — gradient smoothing
    p.hpfCutoff = 0.002;  // Hz — DC removal
    p.integGain = 0.5;
    p.seekMinimum = false; // maximise y3

    ctrl::ExtremumSeeker esc(p, Ts);

    BoilerTurbine bt;
    bt.x1 = op.x1;
    bt.x2 = op.x2;
    bt.x3 = op.x3; // start at equilibrium
    bt.u1 = op.u1;
    bt.u2 = op.u2;
    bt.u3 = op.u3;

    bt.y1 = bt.x1;
    bt.y2 = bt.x2;
    {
        float acs = ((1.0f - 0.001538f * bt.x3) * 0.8f * bt.x1 - 25.6f) /
                    (bt.x3 * (1.0394f - 0.0012304f * bt.x1));
        float qe = (0.854f * bt.u2 - 0.147f) * bt.x1 + 45.59f * bt.u1 - 2.514f * bt.u3 - 2.096f;
        bt.y3 = 0.05f * (0.13073f * bt.x3 + 100.0f * acs + qe / 9.0f - 67.975f);
    }

    float u3_prev = op.u3;

    const std::string fname = "esc_op_" + label + ".csv";
    std::ofstream f(fname);
    f << "Time,y1,y2,y3,u1,u2,u3,du3,theta\n";

    int spacing[] = {8, 10, 9, 9, 9, 8, 8, 8, 9};
    int sum_spacing = 0;
    for (size_t i = 0; i < sizeof(spacing) / sizeof(spacing[0]); ++i)
        sum_spacing += spacing[i];

    std::cout << std::setw(spacing[0]) << "k"
              << std::setw(spacing[1]) << "t[s]"
              << std::setw(spacing[2]) << "y1"
              << std::setw(spacing[3]) << "y2"
              << std::setw(spacing[4]) << "y3"
              << std::setw(spacing[5]) << "u1"
              << std::setw(spacing[6]) << "u2"
              << std::setw(spacing[7]) << "u3"
              << std::setw(spacing[8]) << "theta" << "\n";
    std::cout << std::string(sum_spacing, '-') << "\n";

    for (int k = 0; k <= sim_steps; ++k)
    {
        bt.u1 = op.u1;
        bt.u2 = op.u2;
        bt.u3 = std::clamp(static_cast<float>(esc.compute(bt.y3)), 0.0f, 1.0f);

        const float du3 = bt.u3 - u3_prev;
        u3_prev = bt.u3;

        f << k * Ts << ","
          << bt.y1 << "," << bt.y2 << "," << bt.y3 << ","
          << bt.u1 << "," << bt.u2 << "," << bt.u3 << ","
          << du3 << "," << esc.currentEstimate() << "\n";

        if (k % update_freq == 0)
            std::cout << std::setw(spacing[0]) << k
                      << std::fixed << std::setprecision(4)
                      << std::setw(spacing[1]) << k * Ts
                      << std::setw(spacing[2]) << bt.y1
                      << std::setw(spacing[3]) << bt.y2
                      << std::setw(spacing[4]) << bt.y3
                      << std::setw(spacing[5]) << bt.u1
                      << std::setw(spacing[6]) << bt.u2
                      << std::setw(spacing[7]) << bt.u3
                      << std::setw(spacing[8]) << esc.currentEstimate() << "\n";

        bt.update();
    }

    std::cout << "ESC data written to " << fname << "\n";
}

void run_case_study(const operating_point &op, const std::string &label, const float Ts, int sim_steps = 2000, int update_freq = 10)
{
    LinearStateSpace ss_op = linearize(op, Ts);
    ss_op.print(label);
    ss_lqr(ss_op, op, label, Ts, update_freq, sim_steps);
    ss_mpc(ss_op, op, label, Ts, update_freq, sim_steps);
    ss_lqg_kalman(ss_op, op, label, Ts, update_freq, sim_steps);
    ss_PID(ss_op, op, label, Ts, update_freq, sim_steps);
    ss_SMC(ss_op, op, label, Ts, update_freq, sim_steps);
    ss_extremum_seeker(ss_op, op, label, Ts, update_freq, sim_steps);
}

int main()
{
    plant_model_data();
    int update_freq = 60; // print every N steps
    int sim_steps = 3600; // simulate for M steps
    float Ts = 1.0f;      // sample time of 1 s (matches the system's natural timescale)
    run_case_study(op_A, "A - Low Load", Ts, sim_steps, update_freq);
    run_case_study(op_B, "B - Medium Load", Ts, sim_steps, update_freq);
    run_case_study(op_C, "C - High Load", Ts, sim_steps, update_freq);
    return 0;
}