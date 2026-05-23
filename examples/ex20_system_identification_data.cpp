// ============================================================
//  ex20_system_identification_data.cpp
//  Simulates a plant with process and measurement noise
//  and writes the data to a CSV file. This data can be 
//  used for system identification (e.g. ARMAX, N4SID).
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <random>

int main()
{
    const double Ts = 0.01;

    // Plant: 3rd order system 1/((s+1)(s+2)(s+3))
    ctrl::TransferFunction tf({0.0, 1.079e-4, 3.734e-4, 1.614e-4}, {1.0, -2.7067, 2.4367, -0.7307}, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf);

    std::mt19937 rng(42);
    std::normal_distribution<double> input_noise(0.0, 0.2); // process noise on u
    std::normal_distribution<double> meas_noise(0.0, 0.05); // measurement noise on y

    std::ofstream out("sysid_data.csv");
    if (!out) {
        std::cerr << "Failed to open sysid_data.csv\n";
        return 1;
    }

    out << "time,u,y\n";

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());

    // Generate 1000 samples (~10 seconds)
    // We'll apply a sequence of steps (PRBS-like) to excite dynamics
    double u_cmd = 0.0;

    std::cout << "Generating dataset for System Identification...\n";

    for (int k = 0; k <= 1000; ++k) {
        double t = k * Ts;

        // PRBS-like input
        if (k % 200 == 0) u_cmd = 1.0;
        else if (k % 200 == 100) u_cmd = -0.5;

        // Apply process noise to input
        double u_actual = u_cmd + input_noise(rng);
        Eigen::VectorXd uv(1); uv << u_actual;

        double y_true = ctrl::ssStep(plant, x, uv)(0);
        double y_meas = y_true + meas_noise(rng);

        out << std::fixed << std::setprecision(4) 
            << t << "," << u_actual << "," << y_meas << "\n";
    }

    std::cout << "Done! Saved to sysid_data.csv\n";
    return 0;
}
