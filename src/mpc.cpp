//
// Created by suraj on 7/5/20.
//

#include <iostream>

#include <mpc_ipopt/mpc.h>
#include <mpc_ipopt/helpers.h>

#include <cppad/ipopt/solve_result.hpp>
#include <cppad/ipopt/solve.hpp>

// We are implementing functions in the below namespace
using namespace mpc_ipopt;

// Vector of doubles
// Can also use std::vector or std::valarray

// Diffrentiable vector of doubles


MPC::MPC(Params p) : params(p), dt(1.0 / p.forward.frequency), steps(params.forward.steps),
                     cons_indices(params.forward.steps), varIndices(params.forward.steps) {
    assert(params.forward.steps > 2);

    // Initialize variables
    // Num_vars = [num_accelerations] * [num timesteps - 2]
    // We only need position at last step
    // thus dont need velocitys at last step
    // thus dont need accel at last two steps

    _vars = {2 * (steps - 2)};

    vars_b = {{_vars.size()},
              {_vars.size()}};

    // _vars auto inited to 0
    // Init bounds
    for (auto i : Range(0, _vars.size())) {
        vars_b.low[i] = params.limits.acc.low;
        vars_b.high[i] = params.limits.acc.high;
    }


    // Constraints:
    // For all but last time step we need to constrain velocity
    cons_b = {{2 * (steps - 1)},
              {2 * (steps - 1)}};
    std::cout << cons_b.high.size() << std::endl;

    for (auto i : cons_indices.v_r()) {
        cons_b.low[i] = params.limits.vel.low;
        cons_b.high[i] = params.limits.vel.high;
        std::cout << i << std::endl;

    }


    for (auto i : cons_indices.v_l()) {
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

    // Note: the first timestep (from state.v -> cons_indices.v_()[0] is basically from the time period
    // That happens while the algo is running

    // Update v_r and v_l
    outputs[1 + cons_indices.v_r()[0]] = state.v_r + state.a_r * dt;
    for (auto[i_acc, i_v] : zip2(varIndices.a_r(), cons_indices.v_r(1))) {
        outputs[1 + i_v] = outputs[1 + i_v - 1] + vars[i_acc] * dt;
    }

    outputs[1 + cons_indices.v_l()[0]] = state.v_l + state.a_l * dt;
    for (auto[i_acc, i_v] : zip2(varIndices.a_l(), cons_indices.v_l(1))) {
        outputs[1 + i_v] = outputs[1 + i_v - 1] + vars[i_acc] * dt;
    }

    // TODO: Should state be modelled complicatedly using the rotate around the IAR by theta method?
    ADvector x{steps}, y{steps}, theta{steps};

    theta[0] = state.theta + (state.v_r - state.v_l) * dt / params.wheel_dist;
    x[0] = state.x + (state.v_r + state.v_l) * dt / 2 * cos(state.theta);
    y[0] = state.y + (state.v_r + state.v_l) * dt / 2 * sin(state.theta);
    for (auto[t_i, v_r_i, v_l_i] : zip3(Range(1, x.size()), cons_indices.v_r(), cons_indices.v_l())) {
        theta[t_i] = theta[t_i - 1] + (outputs[1 + v_r_i] - outputs[1 + v_l_i]) * dt / params.wheel_dist;
        x[t_i] = x[t_i - 1] + (outputs[1 + v_r_i] + outputs[1 + v_l_i]) * dt / 2 * CppAD::cos(theta[t_i - 1]);
        y[t_i] = y[t_i - 1] + (outputs[1 + v_r_i] + outputs[1 + v_l_i]) * dt / 2 * CppAD::sin(theta[t_i - 1]);
    }

    std::cout << x << std::endl << y << std::endl << theta << std::endl;

    for (auto i : Range(0, x.size())) {
        objective_func += x[i];
        objective_func -= y[i];
    }
    
    for (auto i : cons_indices.v_r() + cons_indices.v_l()) {
        objective_func -= outputs[1 + i];
    }
}