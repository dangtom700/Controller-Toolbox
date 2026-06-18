/**
 * ex82_metaheuristics.cpp
 *
 * Demonstrates GeneticAlgorithm, ParticleSwarmOptimizer, and DifferentialEvolution
 * on a 2D Rosenbrock function: f(x,y) = (1-x)^2 + 100*(y-x^2)^2, global min at (1,1).
 *
 * All three return TunerResult{params, cost, nEvals, nGens, converged}.
 */

#include "ControllerToolbox.h"
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>

// 2D Rosenbrock banana function (min at [1,1], cost=0)
static double rosenbrock(const Eigen::VectorXd& x) {
    double a = 1.0 - x(0);
    double b = x(1) - x(0) * x(0);
    return a * a + 100.0 * b * b;
}

int main() {
    std::cout << std::fixed << std::setprecision(6);

    Eigen::VectorXd lb(2); lb << -2.0, -2.0;
    Eigen::VectorXd ub(2); ub <<  3.0,  3.0;

    // ---- Genetic Algorithm --------------------------------------------------
    {
        ctrl::GAParams p;
        p.n_dim      = 2;
        p.population = 60;
        p.max_gen    = 200;
        p.crossover  = 0.8;
        p.mutation   = 0.05;
        p.alpha      = 0.3;
        p.lower = lb;
        p.upper = ub;
        p.seed  = 42;

        ctrl::GeneticAlgorithm ga(p);
        auto r = ga.optimize(rosenbrock);

        std::cout << "=== GeneticAlgorithm (Rosenbrock 2D) ===\n"
                  << "  Best params:  [" << r.params(0) << ", " << r.params(1) << "]\n"
                  << "  Best cost:    " << r.cost << "\n"
                  << "  Evaluations:  " << r.nEvals << "\n"
                  << "  Generations:  " << r.nGens  << "\n"
                  << "  Converged:    " << (r.converged ? "yes" : "no") << "\n\n";
    }

    // ---- Particle Swarm Optimizer -------------------------------------------
    {
        ctrl::PSOParams p;
        p.n_dim       = 2;
        p.n_particles = 40;
        p.max_iter    = 200;
        p.w  = 0.729;
        p.c1 = 1.495;
        p.c2 = 1.495;
        p.lower = lb;
        p.upper = ub;
        p.seed  = 17;

        ctrl::ParticleSwarmOptimizer pso(p);
        auto r = pso.optimize(rosenbrock);

        std::cout << "=== ParticleSwarmOptimizer (Rosenbrock 2D) ===\n"
                  << "  Best params:  [" << r.params(0) << ", " << r.params(1) << "]\n"
                  << "  Best cost:    " << r.cost << "\n"
                  << "  Evaluations:  " << r.nEvals << "\n"
                  << "  Iterations:   " << r.nGens  << "\n"
                  << "  Converged:    " << (r.converged ? "yes" : "no") << "\n\n";
    }

    // ---- Differential Evolution ---------------------------------------------
    {
        ctrl::DEParams p;
        p.n_dim      = 2;
        p.population = 30;
        p.max_gen    = 200;
        p.F  = 0.8;
        p.CR = 0.9;
        p.lower = lb;
        p.upper = ub;
        p.seed  = 7;

        ctrl::DifferentialEvolution de(p);
        auto r = de.optimize(rosenbrock);

        std::cout << "=== DifferentialEvolution (Rosenbrock 2D) ===\n"
                  << "  Best params:  [" << r.params(0) << ", " << r.params(1) << "]\n"
                  << "  Best cost:    " << r.cost << "\n"
                  << "  Evaluations:  " << r.nEvals << "\n"
                  << "  Generations:  " << r.nGens  << "\n"
                  << "  Converged:    " << (r.converged ? "yes" : "no") << "\n";
    }

    return 0;
}
