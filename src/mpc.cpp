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

MPC::MPC(Params p) : params(p), dt(1.0 / p.forward.frequency), steps(params.forward.steps),
                     indices(params.forward.steps), state{} {
    assert(params.forward.steps > 1);

    /*
    * State consists of x, y, theta, v_r, v_l.
    * Input values should be in robot frame at t=0 for the model,
    * i.e state.{x, y, theta} = 0;
    *
    * Caller can assume `state` variable is never changed by the model.
    */

    // General idea

    // Thus (a_r, a_l)_(0...N-1)] are the VARIABLES
    // We get their indices from varIndices.a_{r,l}()
    // They will be accessed through `vars`

    // Constraints:
    // We are constraining (v_r, v_l)_(0...N-1)
    // We get their indices from consIndices.v_{r,l}()
    // They are accessed through `cons`


    // Initialize variables
    // Num_vars = [num_accelerations] * [num timesteps - 1]


    _vars = {2 * (steps - 1)};

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

    for (auto i : indices.v_r()) {
        cons_b.low[i] = params.limits.vel.low;
        cons_b.high[i] = params.limits.vel.high;
        std::cout << i << std::endl;

    }


    for (auto i : indices.v_l()) {
        cons_b.low[i] = params.limits.vel.low;
        cons_b.high[i] = params.limits.vel.high;
    }
}

static const std::map<size_t, std::string> ipopt_error_to_string{
        {0,  "not_defined"},
        {1,  "success"},
        {2,  "maxiter_exceeded"},
        {3,  "stop_at_tiny_step"},
        {4,  "stop_at_acceptable_point"},
        {5,  "local_infeasibility"},
        {6,  "user_requested_stop"},
        {7,  "feasible_point_found"},
        {8,  "diverging_iterates"},
        {9,  "restoration_failure"},
        {10, "error_in_step_computation"},
        {11, "invalid_number_detected"},
        {12, "too_few_degrees_of_freedom"},
        {13, "internal_error"},
        {14, "unknown"}
};

void MPC::run() {

    std::cout << "run" << std::endl;

    std::string options;
    options += "Integer print_level  0\n";

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve(options, _vars, vars_b.low, vars_b.high, cons_b.low, cons_b.high, *this, solution);

    std::cout << std::fixed
              << "Stat: " << ipopt_error_to_string.at(solution.status) << std::endl
              << "cost: " << solution.obj_value << std::endl
              << " Acc: " << solution.x
              << "cons: " << std::endl << "cons:" << solution.g << std::endl
              << std::scientific;
}

void MPC::operator()(ADvector &outputs, ADvector &vars) {
    outputs[0] = 0;
    auto &objective_func = outputs[0];
    ConsWrapper cons{outputs};

    /* Indicing
     *
     * Note, when foo_s is the state that the robot is estimated to have after we have finished running this iteration
     *       of the controller. The caller of solve is required to estimate that.
     *
     *
     * In this controller we start our model from that point in time, foo_s
     *
     * What we are calculating:
     * For 4 timesteps
     *
     *  time:      0                  1                  2                  3                  4
     *             |                  |                  |                  |                  |
     * state:     x_s                x_0                x_1                x_2                x_3
     *             |                  |                  |                  |                  |
     *             |                  |                  |                  |                  |
     * accel:     a_0                a_1                a_2                a_3                 |
     *             |                  |                  |                  |                  |
     *   vel:  v_s => v_0         v_0 => v_1         v_1 => v_2         v_2 => v_3             |
     *             |                  |                  |                  |                  |
     *             |                  |                  |                  |                  |
     *   err:     e_s                e_0                e_1                e_2                e_3
     *
     * Thus v_i-1 + a_i => v_i
     *      x_i-1 + v_i => x_i
     *              x_i => e_i
     *
     * There are n-1 of each acceleration, and state_variable
     *
     * We assume velocity changes instantly.
     *
     * foo_s is stored in state
     * a_i is stored in `vars` using indices
     * v_i is stored in `cons`
     *
     * To access v_r_4, do:
     *     <  get  index  >
     * cons[indices.v_r()[4]] // Incorrect indices are a runtime error from CppAD
     *
     * To iterate through all values of a_l do:
     *
     * for (auto a_l_i : indices.a_l()) {
     *     vars[a_l_i];
     *     // Do something
     * }
     *
     * To iterate through 2-3 sets of indices together:
     *
     * for (auto [a_r_i, a_l_i : zip2(indices.a_r(), indices.a_l())) {
     *     std::cout << vars[a_r_i] << " " << vars[a_l_i] << std::endl;
     * }
     *
     * ONLY two Ranges that are adjacent can be combined using the '+' operator. Dont use it.
     */

    // Update v_r and v_l
    // v_i = v_i-1 + a_i * dt
    cons[indices.v_r()[0]] = state.v_r + vars[indices.a_r()[0]] * dt;
    for (auto[i_acc, i_v] : zip2(indices.a_r(1), indices.v_r(1))) {
        cons[i_v] = cons[i_v - 1] + vars[i_acc] * dt;
    }

    cons[indices.v_l()[0]] = state.v_l + vars[indices.a_l()[0]] * dt;
    for (auto[i_acc, i_v] : zip2(indices.a_l(1), indices.v_l(1))) {
        cons[i_v] = cons[i_v - 1] + vars[i_acc] * dt;
    }

    // TODO: Should state be modelled complicatedly using the rotate around the IAR by theta method?
    ADvector x{steps - 1}, y{steps - 1}, theta{steps - 1};

//    theta[0] = state.theta + (state.v_r - state.v_l) * dt / params.wheel_dist;
//    x[0] = state.x + (cons[indices.v_r()[0]] + cons[indices.v_l()[0]]) * dt * CppAD::cos(state.theta) / 2;
//    y[0] = state.y + (cons[indices.v_r()[0]] + cons[indices.v_l()[0]]) * dt * CppAD::sin(state.theta) / 2;
//    for (auto[t_i, v_r_i, v_l_i] : zip3(Range(1, steps - 1), indices.v_r(1), indices.v_l(1))) {
//        std::cout << t_i << v_r_i << v_l_i << std::endl;
//        theta[t_i] = theta[t_i - 1] + (cons[v_r_i] - cons[v_l_i]) * dt / params.wheel_dist;
//        x[t_i] = x[t_i - 1

//    for (auto i : Range(0, x.size())) {
//        objective_func += x[i];
//        objective_func -= y[i];
//    }

    for (auto i : indices.v_r() + indices.v_l()) {
        objective_func -= outputs[1 + i];
    }
}