//
// Created by suraj on 7/5/20.
//

#include <iostream>
#include "../include/mpc_ifopt/mpc.h"
#include <cppad/ipopt/solve_result.hpp>
#include <cppad/ipopt/solve.hpp>

// We are implementing functions in the below namespace
using namespace mpc_ipopt;

// Vector of doubles
// Can also use std::vector or std::valarray

// Diffrentiable vector of doubles


MPC::MPC(Params p) : params(p), dt(1.0 / p.forward.frequency), steps(params.forward.steps), _indices(steps) {
    assert(params.forward.steps > 2);

    // Num_vars = [num_accelerations] * [num timesteps - 2]
    // We only need position at last step
    // thus dont need velocitys at last step
    // thus dont need accel at last two steps
    num_vars = steps - 2;

    vars = {2 * (steps - 2)};

    vars_b = {{vars.size()},
              {vars.size()}};

    // vars auto inited to 0
    // Init bounds
    for (auto i : Range(0, vars.size())) {
        vars_b.low[i] = params.limits.acel.low;
        vars_b.high[i] = params.limits.acel.high;
    }


    // Constraints:
    // For all but last time step we need to constrain velocity
    cons_b = {{2 * steps},
              {2 * steps}};

    for (auto i : _indices.v_r()) {
        cons_b.low[i] = params.limits.acel.low;
        cons_b.high[i] = params.limits.acel.high;
    }

    for (auto i : _indices.v_l()) {
        cons_b.low[i] = params.limits.acel.low;
        cons_b.high[i] = params.limits.acel.high;
    }
}

void MPC::run() {
    const size_t num_vars = 1;
    const size_t num_constraints = 1;

    std::cout << "run" << std::endl;

    std::string options;


    Dvector vars{num_vars};
    vars[0] = 3;

    Dvector vars_lb{num_vars}, vars_ub{num_vars};
    vars_lb[0] = 0.1;
    vars_ub[0] = 10;

    Dvector constraints_lb{num_constraints}, constraints_ub{num_constraints};
    constraints_lb[0] = 1;
    constraints_ub[0] = 20;

    CppAD::ipopt::solve_result<Dvector> solution;

    // CppAD::ipopt::solve<Dvector, MPC>(options, vars, vars_lb, vars_ub, constraints_lb, constraints_ub, *this, solution);
    CppAD::ipopt::solve(options, vars, vars_lb, vars_ub, constraints_lb, constraints_ub, *this, solution);

    std::cout << solution.status << " " << solution.x.size() << std::endl;
}

void MPC::operator()(ADvector &outputs, ADvector &vars) {
    outputs[0] = vars[0];
    outputs[1] = vars[0] * vars[0];
}
