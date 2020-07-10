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


    // CppAD::ipopt options
    options += "Integer print_level  0\n";
    // TODO take as pparams
    options += "Sparse  true        forward\n";
    //options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";


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
    // Num_vars = [num_accelerations] * [num timesteps]


    _vars = {indices.vars_length};

    vars_b = {{indices.vars_length},
              {indices.vars_length}};

    // _vars auto inited to 0
    // Init bounds
    for (auto i : indices.a_r() + indices.a_l()) {
        vars_b.low[i] = params.limits.acc.low;
        vars_b.high[i] = params.limits.acc.high;
    }


    // Constraints:
    // v_r, v_l
    cons_b = {{indices.cons_length},
              {indices.cons_length}};

    for (auto i : indices.v_r() + indices.v_l()) {
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


bool MPC::solve(std::pair<double, double> &acc) {
    std::cout << "run" << std::endl;

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve(options, _vars, vars_b.low, vars_b.high, cons_b.low, cons_b.high, *this, solution);
    if (solution.status != CppAD::ipopt::solve_result<Dvector>::success)
        return false;
    /*std::cout << std::fixed
              << "Stat: " << ipopt_error_to_string.at(solution.status) << std::endl
              << "cost: " << solution.obj_value << std::endl
              << " Acc: " << solution.x << std::endl
              << "cons:" << solution.g << std::endl
              //              << "   x:" << x << std::endl
              //              << "   y:" << y << std::endl
              //              << "thet:" << theta << std::endl
              << std::scientific;

    std::cout << std::fixed << "[";
    for (auto s : get_states(solution.g, state)) {
        std::cout << "(" << s.x << ", " << s.y << ", " << s.theta << "), ";
    }
    std::cout << "]" << std::endl << std::scientific;*/
    acc.first = solution.x[indices.a_r()[0]];
    acc.second = solution.x[indices.a_l()[0]];

    return true;
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
     *      cte = sqrt(|f(x) - y|)
     *      etheta = |
     *
     * There are n of each acceleration, and state_variable
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

    ADvector::value_type x{0}, y{0}, theta{0};
    ADvector old_state{5};
    {
        old_state[0] = state.x;
        old_state[1] = state.y;
        old_state[2] = state.theta;
        old_state[3] = state.v_r;
        old_state[4] = state.v_l;
    }

    // TODO: move inside loop?
    ADState prev{old_state[0], old_state[1], old_state[2], old_state[3], old_state[4]};

    Range a_r_r{indices.a_r()}, a_l_r{indices.a_l()},
            v_r_r{indices.v_r()}, v_l_r{indices.v_l()};

    // I think we have to use only CppAD operations (pow) for differentiability
    for (auto t : Range{0, steps}) {

        cons[*v_r_r] = prev.v_r + vars[*a_r_r] * dt;
        cons[*v_l_r] = prev.v_l + vars[*a_l_r] * dt;

        x = prev.x + (cons[*v_r_r] + cons[*v_l_r]) * dt * CppAD::cos(prev.theta) / 2;
        y = prev.y + (cons[*v_r_r] + cons[*v_l_r]) * dt * CppAD::sin(prev.theta) / 2;
        theta = prev.theta + (cons[*v_r_r] - cons[*v_l_r]) * dt / params.wheel_dist;


        objective_func += params.wt.acc * CppAD::pow(vars[*a_r_r] + vars[*a_l_r], 2);
        objective_func += params.wt.acc * CppAD::pow(vars[*a_r_r] - vars[*a_l_r], 2);

        objective_func += params.wt.vel * CppAD::pow(cons[*v_r_r] + cons[*v_l_r] - 2 * params.v_ref, 2);
        objective_func += params.wt.vel * CppAD::pow(cons[*v_r_r] - cons[*v_l_r], 2);// - 2 * params.v_ref, 2);

        objective_func += params.wt.cte * CppAD::pow(polyeval(x, global_plan) - y, 2);

        prev.x = x, prev.y = y, prev.theta = theta, prev.v_r = cons[*v_r_r], prev.v_l = cons[*v_l_r];
        ++a_r_r, ++a_l_r, ++v_r_r, ++v_l_r;
    }
}

std::vector<State> MPC::get_states(const Dvector &cons, const State &initial) {
    std::vector<State> v;
    v.emplace_back(initial);

    Range a_r_r{indices.a_r()}, a_l_r{indices.a_l()},
            v_r_r{indices.v_r()}, v_l_r{indices.v_l()};

    for (auto t : Range{0, steps}) {
        const auto &prev = v.back();

        State cur{
                prev.x + (cons[*v_r_r] + cons[*v_l_r]) * dt * CppAD::cos(prev.theta) / 2,
                prev.y + (cons[*v_r_r] + cons[*v_l_r]) * dt * CppAD::sin(prev.theta) / 2,
                prev.theta + (cons[*v_r_r] - cons[*v_l_r]) * dt / params.wheel_dist,
                cons[*v_r_r],
                cons[*v_l_r]
        };
        v.push_back(cur);

        /*double v_r = cons[*v_r_r], v_l = cons[*v_l_r];
        v.emplace_back(State{
                prev.x + (cons[*v_r_r] + cons[*v_l_r]) * dt * CppAD::cos(prev.theta) / 2,
                prev.y + (cons[*v_r_r] + cons[*v_l_r]) * dt * CppAD::sin(prev.theta) / 2,
                prev.theta + (cons[*v_r_r] - cons[*v_l_r]) * dt / params.wheel_dist,
                v_r,
                v_l
        });*/

        ++a_r_r, ++a_l_r, ++v_r_r, ++v_l_r;
    }

    return v;
}
