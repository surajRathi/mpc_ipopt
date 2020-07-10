//
// Created by suraj on 7/5/20.
//

#ifndef MPC_IPOPT_MPC_H
#define MPC_IPOPT_MPC_H

#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>

#include "helpers.h"

/*
 * Using MPC:
 * Initialize MPC object with the Params object
 * For every iteration"
 * Set the state and global plan member variables
 * Call solve and get acceleration
 * Profit
 */

namespace mpc_ipopt {
    // Can also use std::vector or std::valarray or eigen::vector or CppAD::vector
    // TODO: What should we use
    template<typename T>
    using vector = CppAD::vector<T>;

    // Non-differentiable vector of doubles
    using Dvector = vector<double>;

    template<typename T> // Templatised so we can use for double and AD<double>
    struct State_ {
        T x, y, theta, v_r, v_l;
    };

    using State = State_<Dvector::value_type>;


    // Stores a pair of values: 'low' and 'high'
    template<typename T>
    struct LH {
        T low, high;
    };

    struct Params {
        struct Forward {
            double frequency;   // Hz
            size_t steps;       // time steps
        } forward;

        struct Limits {
            LH<double> vel, acc;
        } limits;

        struct Weights {
            double acc, vel, cte;
        } wt;

        double v_ref;
        /*unsigned*/ double wheel_dist; // meters
    };


    class MPC {
    public:
        // Diffrentiable vector of doubles
        // Can also use std::vector or std::valarray
        using ADvector = vector<CppAD::AD<double>>; // Exposed for ipopt
    private:
        // Diffrentiable version of state
        // required in operator()
        using ADState = State_<ADvector::value_type &>;


        // Parameters
        // TODO: Make editable with automatic indices reload.
        const Params params;
        const double dt; /* = 1 / params.forward.frequency */
        const size_t &steps;

        std::string options; // TODO: Parameterize options


        // TODO: Better declaration format
        // Stores indices of variables and constraints
        const class Indices {
        private:
            // Variables
            const size_t _a_r, _a_l;
        public:
            const size_t vars_length;

            [[nodiscard]] Range a_r(size_t offset = 0) const { return {0 + offset, _a_r}; }

            [[nodiscard]] Range a_l(size_t offset = 0) const { return {_a_r + offset, _a_l}; }

        private:
            // Constraints
            const size_t _v_r, _v_l;
        public:
            const size_t cons_length;

            [[nodiscard]] Range v_r(size_t offset = 0) const { return {0 + offset, _v_r}; }

            [[nodiscard]] Range v_l(size_t offset = 0) const { return {_v_r + offset, _v_l}; }

            // Constructor
        public:
            explicit Indices(const size_t N) :
            // Variables
                    _a_r{N}, _a_l{_a_r + N},
                    vars_length{_a_l},
                    // Constraints
                    _v_r{N}, _v_l{_v_r + N},
                    cons_length{_v_l} {}
        } indices;


        // Vector with inital value of variables. Doesn't change
        Dvector _vars;
        LH<Dvector> vars_b, cons_b;

        // TODO: move to helper.h
        class ConsWrapper {
            ADvector &_outputs;
        public:
            explicit ConsWrapper(ADvector &outputs) : _outputs(outputs) {}

            ADvector::value_type &operator[](size_t index) { return _outputs[1 + index]; }

            // const ADvector::value_type &operator[](size_t index) const { return _outputs[1 + index]; }
        };


        // Calculates x,y,theta from velocity (stored in the constraints) and initial state.
        std::vector<State> get_states(const Dvector &cons, const State &initial);

    public:

        explicit MPC(Params p);


        // These should be updated before calling solve
        State state;
        Dvector global_plan;

        // Calculate's optimal acceleration for given state and constraints.
        bool solve(std::pair<double, double> &acc);


        // This sets the cost function and calculates constraints from variables
        // ONLY CALLED BY IPOPT
        void operator()(ADvector &outputs, ADvector &vars);
    };

    // Finds f(x) where f = coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 ...
    template<typename Tc, typename Tx>
    Tx polyeval(const Tx &x, const Tc &coeffs) {
        Tx ret = 0, pow = 1;
        for (decltype(coeffs.size()) i = 0; i < coeffs.size(); i++, pow *= x) {
            ret += coeffs[i] * pow;
        }
        return ret;
    }
}

#endif //MPC_IPOPT_MPC_H
