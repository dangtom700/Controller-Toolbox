// ============================================================
//  ex21_boiler_turbine_case_study.cpp
//  Boiler-Turbine Case Study: Successive Online Model Linearisation
//
//  Recreates the study from:
//  "Nonlinear predictive control of a boiler-turbine unit: A state-space 
//  approach with successive on-line model linearization and quadratic optimization"
//  by Maciej Lawrynczuk (ISA Transactions, 2017).
//
//  This example demonstrates that using a fixed linear MPC model causes
//  degradation at off-design operating points, while updating the MPC model 
//  via successive online linearisation restores high performance.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <vector>
#include <cmath>

static constexpr double Ts = 1.0; // 1s sampling as in the paper

// ---------------------------------------------------------------------------
// Boiler-Turbine Nonlinear Model (Bell-Astrom model structure)
// x1: Drum pressure
// x2: Electric power
// x3: Fluid density
// u1: Fuel valve
// u2: Steam valve
// u3: Feed-water valve
// ---------------------------------------------------------------------------
struct BTState {
    double P, E, rho;
    Eigen::VectorXd toVector() const {
        Eigen::VectorXd x(3); x << P, E, rho; return x;
    }
};

struct BTInputs {
    double u1, u2, u3;
    Eigen::VectorXd toVector() const {
        Eigen::VectorXd u(3); u << u1, u2, u3; return u;
    }
};

class BoilerTurbine {
public:
    BTState x;
    
    BoilerTurbine(BTState x0) : x(x0) {}

    // dx/dt calculation
    BTState dynamics(const BTState& st, const BTInputs& u) const {
        BTState dxdt;
        double P98 = std::pow(st.P, 9.0/8.0); // P^(9/8)
        
        // Bell-Astrom nonlinear ODEs
        dxdt.P = -0.0018 * u.u2 * P98 + 0.9 * u.u1 - 0.15 * u.u3;
        dxdt.E = (0.073 * u.u2 - 0.016) * P98 - 0.1 * st.E;
        dxdt.rho = (141.0 * u.u3 - (1.1 * u.u2 - 0.19) * st.P) / 85.0;
        
        return dxdt;
    }

    // Step plant using RK4 integration
    void step(const BTInputs& u) {
        BTState k1 = dynamics(x, u);
        BTState x2 = {x.P + 0.5*Ts*k1.P, x.E + 0.5*Ts*k1.E, x.rho + 0.5*Ts*k1.rho};
        BTState k2 = dynamics(x2, u);
        BTState x3 = {x.P + 0.5*Ts*k2.P, x.E + 0.5*Ts*k2.E, x.rho + 0.5*Ts*k2.rho};
        BTState k3 = dynamics(x3, u);
        BTState x4 = {x.P + Ts*k3.P, x.E + Ts*k3.E, x.rho + Ts*k3.rho};
        BTState k4 = dynamics(x4, u);

        x.P += (Ts/6.0) * (k1.P + 2*k2.P + 2*k3.P + k4.P);
        x.E += (Ts/6.0) * (k1.E + 2*k2.E + 2*k3.E + k4.E);
        x.rho += (Ts/6.0) * (k1.rho + 2*k2.rho + 2*k3.rho + k4.rho);
    }
    
    // Outputs (y1=Pressure, y2=Power, y3=Water level deviation approximation)
    Eigen::VectorXd output() const {
        Eigen::VectorXd y(3);
        y(0) = x.P;
        y(1) = x.E;
        // Simplified water level deviation for demonstration
        y(2) = 0.05 * (0.13073 * x.rho - 67.975); 
        return y;
    }
    
    // Online Linearization (Jacobian via finite differences)
    ctrl::StateSpace linearize(const BTInputs& u) const {
        const int n = 3, m = 3, p = 3;
        Eigen::MatrixXd Ac(n, n), Bc(n, m);
        const double eps = 1e-5;
        
        BTState f0 = dynamics(x, u);
        
        // A matrix: d(f) / dx
        BTState x_pert = x; x_pert.P += eps;
        BTState fx = dynamics(x_pert, u);
        Ac(0,0) = (fx.P - f0.P)/eps; Ac(1,0) = (fx.E - f0.E)/eps; Ac(2,0) = (fx.rho - f0.rho)/eps;
        
        x_pert = x; x_pert.E += eps;
        fx = dynamics(x_pert, u);
        Ac(0,1) = (fx.P - f0.P)/eps; Ac(1,1) = (fx.E - f0.E)/eps; Ac(2,1) = (fx.rho - f0.rho)/eps;
        
        x_pert = x; x_pert.rho += eps;
        fx = dynamics(x_pert, u);
        Ac(0,2) = (fx.P - f0.P)/eps; Ac(1,2) = (fx.E - f0.E)/eps; Ac(2,2) = (fx.rho - f0.rho)/eps;
        
        // B matrix: d(f) / du
        BTInputs u_pert = u; u_pert.u1 += eps;
        BTState fu = dynamics(x, u_pert);
        Bc(0,0) = (fu.P - f0.P)/eps; Bc(1,0) = (fu.E - f0.E)/eps; Bc(2,0) = (fu.rho - f0.rho)/eps;
        
        u_pert = u; u_pert.u2 += eps;
        fu = dynamics(x, u_pert);
        Bc(0,1) = (fu.P - f0.P)/eps; Bc(1,1) = (fu.E - f0.E)/eps; Bc(2,1) = (fu.rho - f0.rho)/eps;
        
        u_pert = u; u_pert.u3 += eps;
        fu = dynamics(x, u_pert);
        Bc(0,2) = (fu.P - f0.P)/eps; Bc(1,2) = (fu.E - f0.E)/eps; Bc(2,2) = (fu.rho - f0.rho)/eps;
        
        // C matrix (dy/dx)
        Eigen::MatrixXd Cc = Eigen::MatrixXd::Zero(p, n);
        Cc(0,0) = 1.0; 
        Cc(1,1) = 1.0; 
        Cc(2,2) = 0.05 * 0.13073;
        
        Eigen::MatrixXd Dc = Eigen::MatrixXd::Zero(p, m);
        
        // Convert Continuous to Discrete (Euler approximation for simplicity here)
        // Ad = I + Ac*Ts, Bd = Bc*Ts
        Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(n, n) + Ac * Ts;
        Eigen::MatrixXd Bd = Bc * Ts;
        
        return ctrl::StateSpace(Ad, Bd, Cc, Dc, Ts);
    }
};

void run_case_study() {
    // OP B: Mid-load operating point
    BTState op_B = {97.2, 50.5, 500.0}; // rho roughly 500 for OP B
    BTInputs u_B = {0.3, 0.5, 0.4};     // Approximation of inputs at OP B
    
    BoilerTurbine plant(op_B);
    
    // Create base linear model at OP B
    ctrl::StateSpace linear_opB = plant.linearize(u_B);
    
    ctrl::MPCParams params;
    params.Np = 20; 
    params.Nc = 5;
    params.rho_y = 1.0;
    params.rho_u = 0.1;
    params.uMin = 0.0; params.uMax = 1.0;
    params.duMin = -0.1; params.duMax = 0.1;

    // 1. Fixed MPC (Standard)
    ctrl::DiscreteMPC fixed_mpc(linear_opB, params);
    
    // 2. Successive Linearized MPC
    ctrl::DiscreteMPC adaptive_mpc(linear_opB, params);
    
    std::cout << "--- Simulating TC1: Abrupt Setpoint Change (OP B -> OP C) ---\n";
    
    // OP C Reference
    Eigen::VectorXd ref(3); ref << 140.0, 128.0, 12.0; // P, E, L
    
    // Simulate Fixed MPC
    BoilerTurbine plant_fixed(op_B);
    BTInputs u_fixed = u_B;
    fixed_mpc.setState(plant_fixed.x.toVector());
    
    double ise_fixed = 0.0;
    for (int k = 0; k < 50; ++k) {
        Eigen::VectorXd y = plant_fixed.output();
        ise_fixed += (y - ref).squaredNorm();
        
        Eigen::VectorXd u_vec = fixed_mpc.computeRef(plant_fixed.x.toVector(), ref);
        u_fixed.u1 = u_vec(0); u_fixed.u2 = u_vec(1); u_fixed.u3 = u_vec(2);
        plant_fixed.step(u_fixed);
    }
    
    // Simulate Adaptive MPC
    BoilerTurbine plant_adaptive(op_B);
    BTInputs u_adaptive = u_B;
    adaptive_mpc.setState(plant_adaptive.x.toVector());
    
    double ise_adaptive = 0.0;
    for (int k = 0; k < 50; ++k) {
        Eigen::VectorXd y = plant_adaptive.output();
        ise_adaptive += (y - ref).squaredNorm();
        
        // Successive online linearisation
        ctrl::StateSpace current_linear_model = plant_adaptive.linearize(u_adaptive);
        adaptive_mpc.setPlant(current_linear_model); // Update MPC internal model
        
        Eigen::VectorXd u_vec = adaptive_mpc.computeRef(plant_adaptive.x.toVector(), ref);
        u_adaptive.u1 = u_vec(0); u_adaptive.u2 = u_vec(1); u_adaptive.u3 = u_vec(2);
        plant_adaptive.step(u_adaptive);
    }
    
    std::cout << "Fixed MPC ISE:    " << ise_fixed << "\n";
    std::cout << "Adaptive MPC ISE: " << ise_adaptive << "\n";
    
    if (ise_adaptive < ise_fixed) {
        std::cout << "-> Adaptive MPC achieves tighter tracking by anticipating nonlinear gain changes.\n";
    }
}

int main() {
    run_case_study();
    return 0;
}
