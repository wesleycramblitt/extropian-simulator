#include <iostream>
#include <vector>
#include <cmath>
#include <exd/opt/opt.hpp>

// A simple test of the optimization library
// This mimics the CMA-ES test in the optimization repo using a rotor-like function

using namespace exd::opt;

// Simple 2D "rotor" function - a simple test case
double rotor_function(const std::vector<double>& x) {
    // A simplified representation of a simple turbine blade (with some variation)
    double a = 0.5;
    double b = 0.3;
    double c = 0.2;
    double d = 0.8;

    double angle = 2 * M_PI * x[0];  // Angle from [0,1] 
    double radius = 0.5 + 0.3 * std::sin(angle);  // Radius varies with angle
    
    // Simple function that has an optimum around (0.5, 0.5) in parametric space
    double r = (radius - 0.7) * (radius - 0.7) + (x[1] - 0.5) * (x[1] - 0.5);
    
    // Add some noise to make it less trivial to optimize 
    double noise = a * std::sin(b * angle) + c * std::cos(d * angle * M_PI);
    
    return r + 0.1 * noise;
}

int main() {
    std::cout << "Testing extropian-optimization integration..." << std::endl;
    std::cout << "Running CMA-ES on a simple rotor function (2D)..." << std::endl;

    // Build the problem definition
    Problem p;
    p.variables.assign(2, Variable{});
    
    OptimizeOptions o;
    o.max_evaluations = 400;
    o.seed = 42;
    o.n_threads = 1; // Use single-thread for simplicity

    auto objective = [](const design& x) {
        return Evaluation{{rotor_function(x)}};
    };

    auto result = optimize(p, Algo::CMAES, objective, o);
    
    if (result.ok()) {
        std::cout << "OK - Optimization successful!" << std::endl;
        std::cout << "Best x: [";
        for (size_t i = 0; i < result.best_x.size(); ++i) {
            std::cout << result.best_x[i];
            if (i < result.best_x.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        std::cout << "Best fitness (function value): " << result.best_fitness[0] << std::endl;
        std::cout << "Evaluations performed: " << result.evaluations << std::endl;
        std::cout << "Generations: " << result.generations << std::endl;
        
        // Verify that it achieved a reasonable minimum (function value close to zero)
        if (result.best_fitness[0] < 1e-2) {
            std::cout << "SUCCESS: Function converged to acceptable local optimum." << std::endl;
            return 0; // Success
        } else {
            std::cout << "WARNING: Function did not converge well - may require tuning." << std::endl;
            return 1; // Failed to converge properly
        }
    } else {
        std::cout << "FAILED: Optimization returned error state." << std::endl;
        return 1;
    }
}