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


MPC::MPC(Params p) : params(p), dt(1.0 / p.forward.frequency), steps(params.forward.steps),
                     _indices(params.forward.steps) {
    assert(params.forward.steps > 2);

    // Initialize variables
    // Num_vars = [num_accelerations] * [num timesteps - 2]
    // We only need position at last step
    // thus dont need velocitys at last step
    // thus dont need accel at last two steps
    num_vars = steps - 2;

    _vars = {2 * (steps - 2)};

    vars_b = {{_vars.size()},
              {_vars.size()}};

    // _vars auto inited to 0
    // Init bounds
    for (auto i : Range(0, _vars.size())) {
        vars_b.low[i] = params.limits.acel.low;
        vars_b.high[i] = params.limits.acel.high;
    }


    // Constraints:
    // For all but last time step we need to constrain velocity
    cons_b = {{2 * (steps - 1)},
              {2 * (steps - 1)}};
    std::cout << cons_b.high.size() << std::endl;

    for (auto i : _indices.v_r()) {
        cons_b.low[i] = params.limits.vel.low;
        cons_b.high[i] = params.limits.vel.high;
        std::cout << i << std::endl;

    }


    for (auto i : _indices.v_l()) {
        cons_b.low[i] = params.limits.vel.low;
        cons_b.high[i] = params.limits.vel.high;
    }
    std::cout << "b" << std::endl;
}

void MPC::run() {

    std::cout << "run" << std::endl;

    std::string options;


    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve(options, _vars, vars_b.low, vars_b.high, cons_b.low, cons_b.high, *this, solution);

    std::cout << solution.status << " " << solution.x << std::endl << solution.obj_value << std::endl << solution.g
              << std::endl;
}

void MPC::operator()(ADvector &outputs, ADvector &vars) {
    outputs[0] = 0;
    auto &objective_func = outputs[0];

//    ADvector v_r = {_indices.v_r().length()};
//    ADvector v_l = {_indices.v_l().length()};

    // First timestep
    size_t vi = 0;
    outputs[1 + *_indices.v_r()] = state.v_r + state.a_r * dt;
    for (auto i : _indices.v_r(1)) {
        outputs[1 + i] = outputs[1 + i - 1] + vars[vi++] * dt;
    }

    vi = 0;
    outputs[1 + *_indices.v_l()] = state.v_r + state.a_l * dt;
    for (auto i : _indices.v_l(1)) {
        outputs[1 + i] = outputs[1 + i - 1] + vars[steps - 2 + vi++] * dt;
    }

    for (auto i : _indices.v_r() + _indices.v_l()) {
        objective_func -= outputs[1 + i];
    }
}